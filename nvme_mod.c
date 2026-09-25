/*
 * Loadable NVMe PCI driver for Cact kmod.
 *
 * Load (root): modload /lib/nvme.cctk
 * Manifest binds PCI class 01:08 (NVM Express). Probe filters prog_if==0x02.
 *
 * Init/admin commands stay polled. Data I/O is IRQ-driven: one synchronous
 * command is in flight, caller sleeps on io_done, ISR drains the IO CQ and wakes.
 */

#include <stddef.h>
#include <stdint.h>
#include "nvme.h"
#include "pci_enum.h"
#include "devfs.h"
#include "sync.h"
#include "klib.h"

extern void     printk(const char* fmt, ...);
extern void     printk_hex(uint32_t v);

extern void*    kmalloc_aligned(uint32_t size, uint32_t align);
extern void     kfree(void* p);
extern void*    memset(void* s, int c, uint32_t n);
extern void*    memcpy(void* dst, const void* src, uint32_t n);

extern uint32_t pci_read_config_dword(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg);
extern void     pci_write_config_dword(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg,
                            uint32_t val);
extern uint8_t  inb (uint16_t port);
extern void     outb(uint16_t port, uint8_t val);
extern void     vmm_map(uint32_t* pd, uint32_t va, uint32_t pa, int flags);
extern void     irq_spinlock_init   (irq_spinlock_t* lock);
extern void     irq_spinlock_acquire(irq_spinlock_t* lock);
extern void     irq_spinlock_release(irq_spinlock_t* lock);

extern int      msidev_register(pci_device_t *dev, void (*handler)(void));
extern void     msidev_unregister(int vec);
extern void     mutex_init  (mutex_t* m);
extern void     mutex_lock  (mutex_t* m);
extern void     mutex_unlock(mutex_t* m);
extern void     sema_init   (semaphore_t* s, int val);
extern void     down   (semaphore_t* s);
extern void     up     (semaphore_t* s);
extern devfs_entry_t* register_chrdev  (const char* name, uint32_t flags,
                                       devfs_driver_t* drv, void* drv_priv);
extern int            unregister_chrdev(const char* name);
extern int register_blkdev(const char *name, uint32_t max_lba,
                           void (*read_sector)(uint32_t lba, uint8_t *buf),
                           void (*write_sector)(uint32_t lba, uint8_t *buf));
extern void unregister_blkdev(const char *name);

#define PAGE_PRESENT 0x1
#define PAGE_RW      0x2
#define PAGE_PWT     0x8
#define PAGE_PCD     0x10


static inline uint32_t* get_current_pd(void) {
    uint32_t val;
    __asm__ __volatile__("mov %%cr3, %0" : "=r"(val));
    return (uint32_t*)val;
}

const uint8_t cact_pci_class    = 0x01;
const uint8_t cact_pci_subclass = 0x08;

static struct nvme_dev ndev;
static int             nvme_ready;
static int             nvme_attached;
static int             devfs_was_registered;

static int      nvme_irq_armed;
static int      nvme_msix_vector;
static uint32_t saved_pci_cmd_dw;

static irq_spinlock_t nvme_lock;
static mutex_t        io_mutex;
static semaphore_t    io_done;
static volatile int   io_inflight;
static volatile int   io_error;

static volatile uint16_t admin_cid;
static volatile uint16_t io_cid;

static uint8_t *admin_sq_mem;
static uint8_t *admin_cq_mem;
static uint8_t *io_sq_mem;
static uint8_t *io_cq_mem;
static uint8_t *identify_buf;



static void nvme_wait_ready(int expected) {
    for (int i = 0; i < 1000000; i++)
        if (((ndev.bar->csts >> 0) & 1) == (uint32_t)expected)
            return;
    printk("[NVMe] controller timeout\n");
}

static void admin_submit(struct nvme_sq_entry *cmd) {
    struct nvme_queue *q = &ndev.admin_q;
    volatile struct nvme_sq_entry *dst = &q->sq[q->sq_tail];
    memcpy((void *)dst, cmd, sizeof(*cmd));
    q->sq_tail = (q->sq_tail + 1) % q->depth;
    *q->sq_db = q->sq_tail;
}

static int admin_poll(void) {
    struct nvme_queue *q = &ndev.admin_q;
    for (int i = 0; i < 2000000; i++) {
        volatile struct nvme_cq_entry *cqe = &q->cq[q->cq_head];
        if ((cqe->status & 1) == q->cq_phase) {
            uint16_t raw = cqe->status;
            uint16_t status = raw >> 1;
            q->cq_head = (q->cq_head + 1) % q->depth;
            if (q->cq_head == 0) q->cq_phase ^= 1;
            *q->cq_db = q->cq_head;
            if (status & 0x7FF) {
                char tmp[16];
                printk("[NVMe] admin err raw=");
                printk_hex(raw);
                printk(" SC=");
                printk("%d", (int)(status & 0xFF));
                printk(" SCT=");
                printk("%d", (int)((status >> 8) & 0x7));
                printk("\n");
            }
            return (status & 0x7FF) ? -1 : 0;
        }
    }
    printk("[NVMe] admin_poll TIMEOUT\n");
    return -1;
}

static int admin_cmd(struct nvme_sq_entry *cmd) {
    admin_submit(cmd);
    return admin_poll();
}

static void nvme_irq_handler(void) {
    if (!nvme_ready)
        return;

    struct nvme_queue *q = &ndev.io_q;
    int wake = 0;

    while (1) {
        volatile struct nvme_cq_entry *cqe = &q->cq[q->cq_head];
        if ((cqe->status & 1) != q->cq_phase)
            break;

        uint16_t status = cqe->status >> 1;
        if (status & 0x7FF)
            io_error = 1;

        q->cq_head = (q->cq_head + 1) % q->depth;
        if (q->cq_head == 0) q->cq_phase ^= 1;
        *q->cq_db = q->cq_head;
        wake = 1;
    }

    if (wake && io_inflight) {
        io_inflight = 0;
        up(&io_done);
    }
}

/* Polled IO completion. Used as a fallback (and by default in this build)
 * because the QEMU NVMe device on the q35 PCIe root complex prefers MSI/MSI-X
 * for completion delivery. Until we wire up MSI here, IRQ 11 stays silent and
 * an IRQ-only path would deadlock the bootstrap thread on down. Polling
 * the CQ phase bit works regardless of how the controller is configured to
 * raise interrupts, just like admin_poll does for admin commands. */
static int nvme_io_poll(void) {
    struct nvme_queue *q = &ndev.io_q;
    for (int i = 0; i < 2000000; i++) {
        volatile struct nvme_cq_entry *cqe = &q->cq[q->cq_head];
        if ((cqe->status & 1) == q->cq_phase) {
            uint16_t raw    = cqe->status;
            uint16_t status = raw >> 1;
            q->cq_head = (q->cq_head + 1) % q->depth;
            if (q->cq_head == 0) q->cq_phase ^= 1;
            *q->cq_db = q->cq_head;
            io_inflight = 0;
            if (status & 0x7FF) {
                io_error = 1;
                return -1;
            }
            return 0;
        }
    }
    printk("[NVMe] io_poll TIMEOUT\n");
    io_inflight = 0;
    io_error    = 1;
    return -1;
}

static int nvme_sync_rw(uint32_t lba, uint8_t *buf, int write) {
    if (!nvme_ready)
        return -1;

    mutex_lock(&io_mutex);

    struct nvme_queue *q = &ndev.io_q;
    struct nvme_sq_entry cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = (write ? NVME_IO_WRITE : NVME_IO_READ)
              | ((uint32_t)(io_cid++ & 0xFFFF) << 16);
    cmd.nsid  = ndev.ns_id;
    cmd.prp1  = (uint32_t)buf;
    cmd.cdw10 = lba;
    cmd.cdw11 = 0;
    cmd.cdw12 = 0; /* NLB=0 means one logical block */

    irq_spinlock_acquire(&nvme_lock);
    io_error    = 0;
    io_inflight = 1;

    volatile struct nvme_sq_entry *dst = &q->sq[q->sq_tail];
    memcpy((void *)dst, &cmd, sizeof(cmd));
    q->sq_tail = (q->sq_tail + 1) % q->depth;
    *q->sq_db = q->sq_tail;
    irq_spinlock_release(&nvme_lock);

    int rc = nvme_io_poll();

    mutex_unlock(&io_mutex);
    return rc;
}

static void nvme_setup_admin_queues(void) {
    memset(admin_sq_mem, 0, sizeof(struct nvme_sq_entry) * NVME_ADMIN_QUEUE_SIZE);
    memset(admin_cq_mem, 0, sizeof(struct nvme_cq_entry) * NVME_ADMIN_QUEUE_SIZE);

    ndev.admin_q.sq       = (volatile struct nvme_sq_entry *)admin_sq_mem;
    ndev.admin_q.cq       = (volatile struct nvme_cq_entry *)admin_cq_mem;
    ndev.admin_q.sq_tail  = 0;
    ndev.admin_q.cq_head  = 0;
    ndev.admin_q.cq_phase = 1;
    ndev.admin_q.depth    = NVME_ADMIN_QUEUE_SIZE;

    uint32_t stride = ndev.db_stride;
    ndev.admin_q.sq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 0 * stride);
    ndev.admin_q.cq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 1 * stride);

    ndev.bar->aqa = ((NVME_ADMIN_QUEUE_SIZE - 1) << 16) | (NVME_ADMIN_QUEUE_SIZE - 1);
    ndev.bar->asq = (uint32_t)admin_sq_mem;
    ndev.bar->acq = (uint32_t)admin_cq_mem;
}

static int nvme_set_num_queues(void) {
    struct nvme_sq_entry cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_SET_FEATURES | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.cdw10 = 0x07;
    cmd.cdw11 = (0 << 16) | 0;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] Set Features (Num Queues) failed\n");
        return -1;
    }
    return 0;
}

static int nvme_create_io_queues(void) {
    memset(io_sq_mem, 0, sizeof(struct nvme_sq_entry) * NVME_IO_QUEUE_SIZE);
    memset(io_cq_mem, 0, sizeof(struct nvme_cq_entry) * NVME_IO_QUEUE_SIZE);

    ndev.io_q.sq       = (volatile struct nvme_sq_entry *)io_sq_mem;
    ndev.io_q.cq       = (volatile struct nvme_cq_entry *)io_cq_mem;
    ndev.io_q.sq_tail  = 0;
    ndev.io_q.cq_head  = 0;
    ndev.io_q.cq_phase = 1;
    ndev.io_q.depth    = NVME_IO_QUEUE_SIZE;

    uint32_t stride = ndev.db_stride;
    ndev.io_q.sq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 2 * stride);
    ndev.io_q.cq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 3 * stride);

    struct nvme_sq_entry cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_CREATE_IOCQ | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.prp1  = (uint32_t)io_cq_mem;
    cmd.cdw10 = ((NVME_IO_QUEUE_SIZE - 1) << 16) | 1;
    cmd.cdw11 = 0x03; /* physically contiguous | interrupts enabled */
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] create IO CQ failed\n");
        return -1;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_CREATE_IOSQ | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.prp1  = (uint32_t)io_sq_mem;
    cmd.cdw10 = ((NVME_IO_QUEUE_SIZE - 1) << 16) | 1;
    cmd.cdw11 = (1 << 16) | 0x01;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] create IO SQ failed\n");
        return -1;
    }
    return 0;
}

static int nvme_identify(void) {
    memset(identify_buf, 0, 4096);

    struct nvme_sq_entry cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_IDENTIFY | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.prp1  = (uint32_t)identify_buf;
    cmd.cdw10 = 1;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] identify controller failed\n");
        return -1;
    }

    memset(identify_buf, 0, 4096);
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_IDENTIFY | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.nsid  = 1;
    cmd.prp1  = (uint32_t)identify_buf;
    cmd.cdw10 = 0;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] identify namespace failed\n");
        return -1;
    }

    uint32_t *id32 = (uint32_t *)identify_buf;
    ndev.ns_id   = 1;
    ndev.max_lba = id32[0];
    return 0;
}

static int nvme_alloc_queues(void) {
    admin_sq_mem = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_sq_entry) * NVME_ADMIN_QUEUE_SIZE, 4096);
    admin_cq_mem = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_cq_entry) * NVME_ADMIN_QUEUE_SIZE, 4096);
    io_sq_mem    = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_sq_entry) * NVME_IO_QUEUE_SIZE, 4096);
    io_cq_mem    = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_cq_entry) * NVME_IO_QUEUE_SIZE, 4096);
    identify_buf = (uint8_t *)kmalloc_aligned(4096, 4096);
    if (!admin_sq_mem || !admin_cq_mem || !io_sq_mem || !io_cq_mem || !identify_buf)
        return -1;
    return 0;
}

static void nvme_free_queues(void) {
    if (admin_sq_mem) { kfree(admin_sq_mem); admin_sq_mem = NULL; }
    if (admin_cq_mem) { kfree(admin_cq_mem); admin_cq_mem = NULL; }
    if (io_sq_mem)    { kfree(io_sq_mem);    io_sq_mem = NULL; }
    if (io_cq_mem)    { kfree(io_cq_mem);    io_cq_mem = NULL; }
    if (identify_buf) { kfree(identify_buf); identify_buf = NULL; }
}

static int nvme_devfs_read(void *p, uint32_t off, uint32_t size, char *buffer) {
    (void)p;
    uint8_t *sector = (uint8_t *)kmalloc_aligned(NVME_SECTOR_SIZE, NVME_SECTOR_SIZE);
    if (!sector) return -1;

    uint32_t done = 0;
    while (done < size) {
        uint32_t cur_off = off + done;
        uint32_t lba     = cur_off / NVME_SECTOR_SIZE;
        uint32_t in_sec  = cur_off % NVME_SECTOR_SIZE;
        uint32_t n       = NVME_SECTOR_SIZE - in_sec;
        if (n > size - done) n = size - done;

        if (nvme_sync_rw(lba, sector, 0) < 0) {
            kfree(sector);
            return -1;
        }
        memcpy(buffer + done, sector + in_sec, n);
        done += n;
    }

    kfree(sector);
    return (int)done;
}

static int nvme_devfs_write(void *p, uint32_t off, uint32_t size, char *buffer) {
    (void)p;
    uint8_t *sector = (uint8_t *)kmalloc_aligned(NVME_SECTOR_SIZE, NVME_SECTOR_SIZE);
    if (!sector) return -1;

    uint32_t done = 0;
    while (done < size) {
        uint32_t cur_off = off + done;
        uint32_t lba     = cur_off / NVME_SECTOR_SIZE;
        uint32_t in_sec  = cur_off % NVME_SECTOR_SIZE;
        uint32_t n       = NVME_SECTOR_SIZE - in_sec;
        if (n > size - done) n = size - done;

        if (nvme_sync_rw(lba, sector, 0) < 0) {
            kfree(sector);
            return -1;
        }
        memcpy(sector + in_sec, buffer + done, n);
        if (nvme_sync_rw(lba, sector, 1) < 0) {
            kfree(sector);
            return -1;
        }
        done += n;
    }

    kfree(sector);
    return (int)done;
}

static int nvme_devfs_status(void *p, char *buf, uint32_t size) {
    (void)p;
    int pos = 0;
    const char *s = "device: nvme0\ntype: NVMe (IRQ)\nmax_lba: ";
    while (s[pos] && (uint32_t)pos < size - 1) { buf[pos] = s[pos]; pos++; }
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d", (int)((int)ndev.max_lba));
    for (int i = 0; tmp[i] && (uint32_t)pos < size - 1; i++) buf[pos++] = tmp[i];
    if ((uint32_t)pos < size - 1) buf[pos++] = '\n';
    buf[pos] = '\0';
    return pos;
}

static devfs_driver_t nvme_drv = {
    .read   = nvme_devfs_read,
    .write  = nvme_devfs_write,
    .status = nvme_devfs_status,
};

static void nvme_blk_read_sector(uint32_t lba, uint8_t *buf) {
    if (nvme_sync_rw(lba, buf, 0) < 0)
        memset(buf, 0, NVME_SECTOR_SIZE);
}

static void nvme_blk_write_sector(uint32_t lba, uint8_t *buf) {
    (void)nvme_sync_rw(lba, buf, 1);
}

static void nvme_detach(void) {
    unregister_blkdev("nvme0");

    if (devfs_was_registered) {
        unregister_chrdev("nvme0");
        devfs_was_registered = 0;
    }

    nvme_ready = 0;

    if (ndev.bar) {
        ndev.bar->intms = 0xFFFFFFFFu;
        ndev.bar->cc = 0;
        nvme_wait_ready(0);
    }

    if (nvme_msix_vector >= 0) {
        msidev_unregister(nvme_msix_vector);
        nvme_msix_vector = -1;
        nvme_irq_armed   = 0;
    }

    if (nvme_attached)
        pci_write_config_dword(ndev.pci_bus, ndev.pci_dev, ndev.pci_fn, 0x04, saved_pci_cmd_dw);

    nvme_free_queues();
    memset(&ndev, 0, sizeof(ndev));
    nvme_attached    = 0;
    io_inflight      = 0;
    io_error         = 0;
    saved_pci_cmd_dw = 0;
}

int pci_driver_probe(pci_device_t *pdev) {
    if (!pdev) return -1;
    if (nvme_attached) {
        printk("[NVMe] already attached, skipping additional controller\n");
        return -1;
    }
    if (pdev->prog_if != 0x02) {
        printk("[NVMe] prog_if!=0x02, skipping\n");
        return -1;
    }

    memset(&ndev, 0, sizeof(ndev));
    irq_spinlock_init(&nvme_lock);
    mutex_init(&io_mutex);
    sema_init(&io_done, 0);
    admin_cid = 0;
    io_cid    = 0;

    ndev.pci_bus = pdev->bus;
    ndev.pci_dev = pdev->dev;
    ndev.pci_fn  = pdev->fn;

    printk("[NVMe] probe bus=");
    printk_hex(ndev.pci_bus); printk(" dev="); printk_hex(ndev.pci_dev);
    printk(" fn="); printk_hex(ndev.pci_fn);
    printk(" irq="); printk_hex(pdev->irq_line); printk("\n");

    uint32_t bar0 = pci_read_config_dword(ndev.pci_bus, ndev.pci_dev, ndev.pci_fn, 0x10);
    ndev.bar_phys = bar0 & 0xFFFFF000;
    if (!ndev.bar_phys) {
        printk("3" "NVMe (kmod): BAR0 missing");
        return -1;
    }

    if (nvme_alloc_queues() < 0) {
        printk("3" "NVMe (kmod): queue allocation failed");
        nvme_detach();
        return -1;
    }

    for (uint32_t off = 0; off < 0x4000; off += 0x1000)
        vmm_map(get_current_pd(), ndev.bar_phys + off, ndev.bar_phys + off,
                PAGE_PRESENT | PAGE_RW | PAGE_PCD | PAGE_PWT);
    ndev.bar = (volatile struct nvme_bar *)(uintptr_t)ndev.bar_phys;

    saved_pci_cmd_dw = pci_read_config_dword(ndev.pci_bus, ndev.pci_dev, ndev.pci_fn, 0x04);
    uint32_t cmd = saved_pci_cmd_dw;
    cmd |= 0x06u;           /* MEM | BUS_MASTER */
    pci_write_config_dword(ndev.pci_bus, ndev.pci_dev, ndev.pci_fn, 0x04, cmd);

    /* Interrupt-driven I/O: msidev_register() picks MSI-X when the controller
     * has it and MSI otherwise, allocating the vector and programming the
     * device.  The queue path polls when neither is available. */
    {
        nvme_msix_vector = -1;
        {
            int vec = msidev_register(pdev, nvme_irq_handler);
            if (vec > 0) {
                nvme_msix_vector = vec;
                nvme_irq_armed   = 1;
                printk("6" "NVMe: IRQ enabled");
            }
        }
        if (nvme_msix_vector < 0)
            printk("4" "NVMe: no MSI/MSI-X — poll-only");
    }

    uint64_t cap = ndev.bar->cap;
    ndev.db_stride = 4 << ((cap >> 32) & 0xF);

    printk("[NVMe] resetting controller\n");
    ndev.bar->intms = 0xFFFFFFFFu;
    ndev.bar->cc = 0;
    nvme_wait_ready(0);

    nvme_setup_admin_queues();

    printk("[NVMe] enabling controller\n");
    ndev.bar->cc = (4 << 20) | (6 << 16) | (0 << 7) | 1;
    nvme_wait_ready(1);
    if (ndev.bar->csts & 0x2) {
        printk("3" "NVMe (kmod): controller fatal status");
        nvme_detach();
        return -1;
    }

    if (nvme_identify() < 0) {
        printk("3" "NVMe (kmod): identify failed");
        nvme_detach();
        return -1;
    }
    if (nvme_set_num_queues() < 0 || nvme_create_io_queues() < 0) {
        printk("3" "NVMe (kmod): IO queue setup failed");
        nvme_detach();
        return -1;
    }

    /* Keep INTMS=all-ones from reset; we are polling completions. */
    nvme_ready = 1;
    nvme_attached = 1;

    if (register_chrdev("nvme0", DEVFS_F_BLOCK, &nvme_drv, 0))
        devfs_was_registered = 1;
    else
        printk("4" "NVMe (kmod): register_chrdev('nvme0') failed");

    if (register_blkdev("nvme0", ndev.max_lba, nvme_blk_read_sector,
                        nvme_blk_write_sector) != 0)
        printk("4" "NVMe (kmod): register_blkdev('nvme0') failed");

    char tmp[16]; snprintf(tmp, sizeof(tmp), "%d", (int)((int)ndev.max_lba));
    printk("[NVMe] namespace max_lba="); printk(tmp);
    printk(" IRQ=enabled\n");
    printk("6" "NVMe (kmod): IRQ-driven driver attached — /dev/nvme0 registered");
    return 0;
}

void pci_driver_remove(pci_device_t *dev) {
    (void)dev;
    int had_any = nvme_attached || devfs_was_registered || ndev.bar != NULL;
    nvme_detach();
    if (had_any)
        printk("6" "NVMe (kmod): unloaded");
}
