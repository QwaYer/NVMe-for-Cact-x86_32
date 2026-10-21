/*
 * Loadable NVMe PCI driver for Cact kmod.
 *
 * Load (root): modload /lib/nvme.cctk
 * Manifest binds PCI class 01:08 (NVM Express). Probe accepts prog_if 0x00,
 *   0x01, 0x02 and the vendor-specific range; anything but 0x02 must pass a
 *   capability check before the controller is reset.
 *
 * Init/admin commands stay polled. Data I/O is IRQ-driven: one synchronous
 * command is in flight.  The completion is reaped by whoever sees it first -
 * the ISR, or the submitting thread's bounded poll - so I/O works with or
 * without a delivered interrupt.
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
extern int  blkdev_set_ident(const char *name, const char *ident);

#define PAGE_PRESENT 0x1
#define PAGE_RW      0x2
#define PAGE_PWT     0x8
#define PAGE_PCD     0x10


static inline uint32_t* get_current_pd(void) {
    uint32_t val;
    __asm__ __volatile__("mov %%cr3, %0" : "=r"(val));
    return (uint32_t*)val;
}

/* ~1 ms busy wait with the PAUSE hint (ahci_udelay's twin).  The module ABI
 * exports no sleep primitive, and controller reset/IDENTIFY on real hardware
 * needs delays rather than a bare MMIO spin. */
static void nvme_udelay(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 10; i++)
        __asm__ __volatile__("pause");
}

const uint8_t cact_pci_class    = 0x01;
const uint8_t cact_pci_subclass = 0x08;

static struct nvme_dev ndev;
static char            nvme_model[41];   // Identify Controller MN (bytes 24..63)
static char            nvme_serial[21];  // Identify Controller SN (bytes 4..23)
static int             nvme_ready;
static int             nvme_attached;
static int             devfs_was_registered;

static int      nvme_irq_armed;
static int      nvme_msix_vector;
static uint32_t saved_pci_cmd_dw;

static irq_spinlock_t nvme_lock;
static mutex_t        io_mutex;
static volatile int   io_inflight;
static volatile int   io_error;

static volatile uint16_t admin_cid;
static volatile uint16_t io_cid;

static uint8_t *admin_sq_mem;
static uint8_t *admin_cq_mem;
static uint8_t *io_sq_mem;
static uint8_t *io_cq_mem;
static uint8_t *identify_buf;
static uint8_t *bounce_block;    /* one ns_sector_size block, page aligned */


/* Wait for CSTS.RDY to reach `expected`.  A real controller can take hundreds
 * of milliseconds to complete a reset, so the loop is generous and pauses
 * between polls; QEMU flips RDY almost immediately. */
static int nvme_wait_ready(int expected) {
    for (int i = 0; i < 5000; i++) {
        if (((ndev.bar->csts >> 0) & 1) == (uint32_t)expected)
            return 0;
        nvme_udelay(1000);
    }
    printk("[NVMe] controller RDY wait timed out\n");
    return -1;
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
    for (int i = 0; i < 5000; i++) {
        volatile struct nvme_cq_entry *cqe = &q->cq[q->cq_head];
        if ((cqe->status & 1) == q->cq_phase) {
            uint16_t raw = cqe->status;
            uint16_t status = raw >> 1;
            q->last_result = cqe->result;
            q->cq_head = (q->cq_head + 1) % q->depth;
            if (q->cq_head == 0) q->cq_phase ^= 1;
            *q->cq_db = q->cq_head;
            if (status & 0x7FF) {
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
        nvme_udelay(1000);
    }
    printk("[NVMe] admin_poll TIMEOUT\n");
    return -1;
}

static int admin_cmd(struct nvme_sq_entry *cmd) {
    admin_submit(cmd);
    return admin_poll();
}

/* Reap every completed command from the IO CQ.  Shared by the ISR and the
 * submitting thread and serialised by nvme_lock: either side may be the one
 * that sees the phase-bit flip, and neither may advance cq_head twice. */
static void nvme_reap_io(void) {
    struct nvme_queue *q = &ndev.io_q;
    int reaped = 0;

    irq_spinlock_acquire(&nvme_lock);
    while (1) {
        volatile struct nvme_cq_entry *cqe = &q->cq[q->cq_head];
        if ((cqe->status & 1) != q->cq_phase)
            break;
        if ((cqe->status >> 1) & 0x7FF)
            io_error = 1;
        q->cq_head = (q->cq_head + 1) % q->depth;
        if (q->cq_head == 0) q->cq_phase ^= 1;
        *q->cq_db = q->cq_head;
        reaped++;
    }
    if (reaped)
        io_inflight = 0;
    irq_spinlock_release(&nvme_lock);
}

static void nvme_irq_handler(void) {
    if (!nvme_ready)
        return;
    nvme_reap_io();
}

/* Bounded wait for the command submitted by nvme_rw_block().  The completion
 * is picked up by whoever sees it first — the ISR, or this poll — so I/O works
 * whether or not the controller actually raises the interrupt.  The module ABI
 * has no timeout-aware sleep, hence the bounded poll. */
static int nvme_wait_io(void) {
    for (int i = 0; i < 5000; i++) {
        if (!io_inflight)
            return io_error ? -1 : 0;
        nvme_reap_io();
        if (!io_inflight)
            return io_error ? -1 : 0;
        nvme_udelay(1000);
    }
    printk("[NVMe] io completion timeout\n");
    io_inflight = 0;
    io_error    = 1;
    return -1;
}

/* Submit one command for one logical block and wait for completion.  lba is
 * in units of the namespace logical block (ndev.ns_sector_size) and `buf`
 * must hold a whole block and be 4-byte aligned.  The caller holds io_mutex. */
static int nvme_submit_block(uint64_t lba, uint8_t *buf, int write) {
    if (!nvme_ready)
        return -1;

    struct nvme_queue *q = &ndev.io_q;
    struct nvme_sq_entry cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = (write ? NVME_IO_WRITE : NVME_IO_READ)
              | ((uint32_t)(io_cid++ & 0xFFFF) << 16);
    cmd.nsid  = ndev.ns_id;
    cmd.prp1  = (uint32_t)(uintptr_t)buf;
    cmd.cdw10 = (uint32_t)lba;          /* LBA 63:32 in cdw11 for big media */
    cmd.cdw11 = (uint32_t)(lba >> 32);
    cmd.cdw12 = 0;                      /* NLB=0 means one logical block */

    irq_spinlock_acquire(&nvme_lock);
    io_error    = 0;
    io_inflight = 1;
    {
        volatile struct nvme_sq_entry *dst = &q->sq[q->sq_tail];
        memcpy((void *)dst, &cmd, sizeof(cmd));
        q->sq_tail = (q->sq_tail + 1) % q->depth;
        *q->sq_db = q->sq_tail;
    }
    irq_spinlock_release(&nvme_lock);

    return nvme_wait_io();
}

/* 512-byte-sector accessor used by the block layer and the /dev compat node.
 * The block layer addresses 512-byte sectors; the namespace may use 512-byte
 * or 4096-byte logical blocks.  A 4Kn namespace is reached with a
 * read-modify-write on a single bounce block (no cache of its own). */
static int nvme_read_slice(uint32_t lba512, uint8_t *buf) {
    mutex_lock(&io_mutex);
    int rc;
    if (ndev.ns_sector_size == NVME_SECTOR_SIZE) {
        rc = nvme_submit_block(lba512, buf, 0);
    } else {
        rc = nvme_submit_block(lba512 / (ndev.ns_sector_size / NVME_SECTOR_SIZE),
                               bounce_block, 0);
        if (rc == 0)
            memcpy(buf, bounce_block +
                        (lba512 % (ndev.ns_sector_size / NVME_SECTOR_SIZE)) * NVME_SECTOR_SIZE,
                   NVME_SECTOR_SIZE);
    }
    mutex_unlock(&io_mutex);
    return rc;
}

static int nvme_write_slice(uint32_t lba512, uint8_t *buf) {
    mutex_lock(&io_mutex);
    int rc;
    if (ndev.ns_sector_size == NVME_SECTOR_SIZE) {
        rc = nvme_submit_block(lba512, buf, 1);
    } else {
        uint32_t per = ndev.ns_sector_size / NVME_SECTOR_SIZE;
        uint64_t blk = lba512 / per;
        rc = nvme_submit_block(blk, bounce_block, 0);      /* read-modify-write */
        if (rc == 0) {
            memcpy(bounce_block + (lba512 % per) * NVME_SECTOR_SIZE,
                   buf, NVME_SECTOR_SIZE);
            rc = nvme_submit_block(blk, bounce_block, 1);
        }
    }
    mutex_unlock(&io_mutex);
    return rc;
}

static void nvme_setup_admin_queues(void) {
    memset(admin_sq_mem, 0, sizeof(struct nvme_sq_entry) * NVME_ADMIN_QUEUE_SIZE);
    memset(admin_cq_mem, 0, sizeof(struct nvme_cq_entry) * NVME_ADMIN_QUEUE_SIZE);

    /* AQA must not promise more entries than CAP.MQES allows. */
    uint32_t adepth = NVME_ADMIN_QUEUE_SIZE;
    if (ndev.admin_q.depth >= 2 && adepth > ndev.admin_q.depth)
        adepth = ndev.admin_q.depth;
    ndev.admin_q.depth    = (uint16_t)adepth;
    ndev.admin_q.sq       = (volatile struct nvme_sq_entry *)admin_sq_mem;
    ndev.admin_q.cq       = (volatile struct nvme_cq_entry *)admin_cq_mem;
    ndev.admin_q.sq_tail  = 0;
    ndev.admin_q.cq_head  = 0;
    ndev.admin_q.cq_phase = 1;

    uint32_t stride = ndev.db_stride;
    ndev.admin_q.sq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 0 * stride);
    ndev.admin_q.cq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 1 * stride);

    ndev.bar->aqa = ((adepth - 1) << 16) | (adepth - 1);
    ndev.bar->asq = (uint32_t)admin_sq_mem;
    ndev.bar->acq = (uint32_t)admin_cq_mem;
}

/* Ask for I/O queues.  A real controller rejects (or returns zero for) a
 * request that does not fit CAP.MQES, and a zero request would leave QID 1
 * unusable — so request the number we want and keep whatever comes back,
 * which is reported in the completion's result dword. */
static int nvme_set_num_queues(void) {
    struct nvme_sq_entry cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_SET_FEATURES | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.cdw10 = 0x07;                                  /* Feature: Num Queues */
    cmd.cdw11 = (((uint32_t)NVME_IO_QUEUE_SIZE - 1) << 16) |
                ((uint32_t)NVME_IO_QUEUE_SIZE - 1);
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] Set Features (Num Queues) failed\n");
        return -1;
    }

    uint32_t r    = ndev.admin_q.last_result;
    uint32_t ncqr = (((r >> 16) & 0xFFFF) + 1);
    uint32_t nsqr = ((r & 0xFFFF) + 1);
    uint32_t depth = NVME_IO_QUEUE_SIZE;
    if (ncqr < depth) depth = ncqr;
    if (nsqr < depth) depth = nsqr;
    if (depth < 2) {
        printk("[NVMe] controller grants no I/O queues\n");
        return -1;
    }
    ndev.io_q.depth = (uint16_t)depth;

    {
        char b[16];
        printk("[NVMe] IO queue depth=");
        snprintf(b, sizeof(b), "%d", (int)depth);
        printk(b);
        printk(" (CQ=");
        snprintf(b, sizeof(b), "%d", (int)ncqr);
        printk(b);
        printk(" SQ=");
        snprintf(b, sizeof(b), "%d", (int)nsqr);
        printk(b);
        printk(" granted)\n");
    }
    return 0;
}

static int nvme_create_io_queues(void) {
    uint16_t depth = ndev.io_q.depth;
    if (depth < 2 || depth > NVME_IO_QUEUE_SIZE)
        depth = NVME_IO_QUEUE_SIZE;

    memset(io_sq_mem, 0, sizeof(struct nvme_sq_entry) * NVME_IO_QUEUE_SIZE);
    memset(io_cq_mem, 0, sizeof(struct nvme_cq_entry) * NVME_IO_QUEUE_SIZE);

    ndev.io_q.sq       = (volatile struct nvme_sq_entry *)io_sq_mem;
    ndev.io_q.cq       = (volatile struct nvme_cq_entry *)io_cq_mem;
    ndev.io_q.sq_tail  = 0;
    ndev.io_q.cq_head  = 0;
    ndev.io_q.cq_phase = 1;
    ndev.io_q.depth    = depth;

    uint32_t stride = ndev.db_stride;
    ndev.io_q.sq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 2 * stride);
    ndev.io_q.cq_db = (volatile uint32_t *)((uint8_t *)ndev.bar + 0x1000 + 3 * stride);

    struct nvme_sq_entry cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_CREATE_IOCQ | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.prp1  = (uint32_t)io_cq_mem;
    cmd.cdw10 = (((uint32_t)depth - 1) << 16) | 1;
    cmd.cdw11 = 0x03; /* physically contiguous | interrupts enabled */
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] create IO CQ failed\n");
        return -1;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_CREATE_IOSQ | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.prp1  = (uint32_t)io_sq_mem;
    cmd.cdw10 = (((uint32_t)depth - 1) << 16) | 1;
    cmd.cdw11 = (1 << 16) | 0x01;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] create IO SQ failed\n");
        return -1;
    }
    return 0;
}

static uint64_t nvme_id64(const uint8_t *b) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= (uint64_t)b[i] << (8 * i);
    return v;
}

static int nvme_identify(void) {
    struct nvme_sq_entry cmd;

    /* 1. Identify Controller (CNS=1): model + serial for /dev/disk/by-id. */
    memset(identify_buf, 0, 4096);
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_IDENTIFY | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.prp1  = (uint32_t)identify_buf;
    cmd.cdw10 = 1;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] identify controller failed\n");
        return -1;
    }

    // Identify Controller carries SN (bytes 4..23) and MN (24..63), space
    // padded; keep them for the /dev/disk/by-id identity.
    for (int i = 0; i < 20; i++) nvme_serial[i] = (char)identify_buf[4 + i];
    nvme_serial[20] = '\0';
    for (int i = 0; i < 40; i++) nvme_model[i] = (char)identify_buf[24 + i];
    nvme_model[40] = '\0';
    for (int i = 19; i >= 0 && nvme_serial[i] == ' '; i--) nvme_serial[i] = '\0';
    for (int i = 39; i >= 0 && nvme_model[i]  == ' '; i--) nvme_model[i]  = '\0';

    /* 2. Active Namespace List (CNS=2): the namespace is not necessarily
     * NSID 1 — take the first active one instead of assuming. */
    memset(identify_buf, 0, 4096);
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_IDENTIFY | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.prp1  = (uint32_t)identify_buf;
    cmd.cdw10 = 0x02;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] identify active namespace list failed\n");
        return -1;
    }
    uint32_t nsid = 0;
    {
        uint32_t *ns = (uint32_t *)identify_buf;
        for (int i = 0; i < 1024; i++)
            if (ns[i]) { nsid = ns[i]; break; }
    }
    if (!nsid) {
        printk("[NVMe] controller has no active namespace\n");
        return -1;
    }

    /* 3. Identify Namespace (CNS=0): NSZE (64-bit) and the LBA format. */
    memset(identify_buf, 0, 4096);
    memset(&cmd, 0, sizeof(cmd));
    cmd.cdw0  = NVME_OPC_IDENTIFY | ((uint32_t)(admin_cid++ & 0xFFFF) << 16);
    cmd.nsid  = nsid;
    cmd.prp1  = (uint32_t)identify_buf;
    cmd.cdw10 = 0;
    if (admin_cmd(&cmd) < 0) {
        printk("[NVMe] identify namespace failed\n");
        return -1;
    }

    uint64_t nsze  = nvme_id64(identify_buf);          /* bytes 0..7  */
    uint8_t  flbas = identify_buf[27] & 0x0F;          /* byte 27     */
    uint8_t  lbads = identify_buf[128 + (uint32_t)flbas * 4 + 2];
    uint32_t lsz   = lbads ? (1u << lbads) : NVME_SECTOR_SIZE;

    if (lsz != 512 && lsz != 4096) {
        printk("[NVMe] unsupported logical block size\n");
        return -1;
    }

    /* The block layer addresses 512-byte sectors, so express a 4Kn namespace
     * in those units and clamp to what the 32-bit layer can reach. */
    uint64_t lba512 = nsze * (lsz / NVME_SECTOR_SIZE);
    if (lba512 > (uint64_t)NVME_MAX_LBA) {
        printk("[NVMe] namespace larger than the 2 TiB the 32-bit block layer "
               "addresses - clamping\n");
        lba512 = NVME_MAX_LBA;
    }

    ndev.ns_id          = nsid;
    ndev.ns_sector_size = lsz;
    ndev.max_lba        = (uint32_t)lba512;
    return 0;
}

/* Confirm the register block really is an NVMe controller.  Used when the
 * programming interface is not the documented 0x02: CAP must describe a
 * 2..65536 entry queue with a sane doorbell stride, and VS must report
 * version 1.0 or newer.  Read-only; nothing is reset before this passes. */
static int nvme_cap_sane(void) {
    uint64_t cap   = ndev.bar->cap;
    uint32_t mqes  = (uint32_t)(cap & 0xFFFF) + 1;      /* max queue entries */
    uint32_t dstrd = (uint32_t)((cap >> 32) & 0xF);     /* doorbell stride   */
    uint32_t vs    = ndev.bar->vs;

    if (cap == 0 || cap == ~(uint64_t)0)
        return 0;
    if (mqes < 2 || mqes > 65536)
        return 0;
    if (dstrd > 8)
        return 0;
    if (vs == 0 || vs == 0xFFFFFFFFu)
        return 0;
    if ((vs >> 16) == 0)                                /* major version */
        return 0;
    return 1;
}

/* Is this programming interface one an NVMe controller may report?  0x02 is
 * NVMe, 0x01 is the original NVMHCI value, 0x00 is vendor specific and
 * 0x80..0xFF is the PCI vendor-specific range — real SSDs and OEM firmware
 * use them.  Non-0x02 values are trusted only after nvme_cap_sane(). */
static int nvme_prog_if_ok(uint8_t prog_if) {
    return prog_if <= 0x02 || prog_if >= 0x80;
}

static int nvme_alloc_queues(void) {
    admin_sq_mem = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_sq_entry) * NVME_ADMIN_QUEUE_SIZE, 4096);
    admin_cq_mem = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_cq_entry) * NVME_ADMIN_QUEUE_SIZE, 4096);
    io_sq_mem    = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_sq_entry) * NVME_IO_QUEUE_SIZE, 4096);
    io_cq_mem    = (uint8_t *)kmalloc_aligned(sizeof(struct nvme_cq_entry) * NVME_IO_QUEUE_SIZE, 4096);
    identify_buf = (uint8_t *)kmalloc_aligned(4096, 4096);
    bounce_block = (uint8_t *)kmalloc_aligned(NVME_MAX_LBS, NVME_MAX_LBS);
    if (!admin_sq_mem || !admin_cq_mem || !io_sq_mem || !io_cq_mem ||
        !identify_buf || !bounce_block)
        return -1;
    return 0;
}

static void nvme_free_queues(void) {
    if (admin_sq_mem) { kfree(admin_sq_mem); admin_sq_mem = NULL; }
    if (admin_cq_mem) { kfree(admin_cq_mem); admin_cq_mem = NULL; }
    if (io_sq_mem)    { kfree(io_sq_mem);    io_sq_mem = NULL; }
    if (io_cq_mem)    { kfree(io_cq_mem);    io_cq_mem = NULL; }
    if (identify_buf) { kfree(identify_buf); identify_buf = NULL; }
    if (bounce_block) { kfree(bounce_block); bounce_block = NULL; }
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

        if (nvme_read_slice(lba, sector) < 0) {
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

        if (nvme_read_slice(lba, sector) < 0) {
            kfree(sector);
            return -1;
        }
        memcpy(sector + in_sec, buffer + done, n);
        if (nvme_write_slice(lba, sector) < 0) {
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
    if (nvme_read_slice(lba, buf) < 0)
        memset(buf, 0, NVME_SECTOR_SIZE);
}

static void nvme_blk_write_sector(uint32_t lba, uint8_t *buf) {
    (void)nvme_write_slice(lba, buf);
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
    /* Class 01/08 is the NVM Express controller class.  NVMe 1.0 reports
     * prog_if 0x02, but real SSDs and OEM firmware also report 0x01 (NVMHCI),
     * 0x00 (vendor specific) or a value in the vendor-specific range.  Accept
     * those and verify the register block below instead of relying on the
     * interface byte alone (Linux matches this class the same way). */
    int std_prog_if = (pdev->prog_if == 0x02);
    if (!nvme_prog_if_ok(pdev->prog_if)) {
        char b[8];
        printk("[NVMe] prog_if=");
        snprintf(b, sizeof(b), "0x%x", (unsigned)pdev->prog_if);
        printk(b);
        printk(": reserved NVM interface, not NVMe - skipping\n");
        return -1;
    }
    if (!std_prog_if) {
        char b[8];
        printk("[NVMe] prog_if=");
        snprintf(b, sizeof(b), "0x%x", (unsigned)pdev->prog_if);
        printk(b);
        printk(" (non-standard): verifying the NVMe register block\n");
    }

    memset(&ndev, 0, sizeof(ndev));
    irq_spinlock_init(&nvme_lock);
    mutex_init(&io_mutex);
    admin_cid = 0;
    io_cid    = 0;

    ndev.pci_bus = pdev->bus;
    ndev.pci_dev = pdev->dev;
    ndev.pci_fn  = pdev->fn;

    printk("[NVMe] probe bus=");
    printk_hex(ndev.pci_bus); printk(" dev="); printk_hex(ndev.pci_dev);
    printk(" fn="); printk_hex(ndev.pci_fn);
    printk(" irq="); printk_hex(pdev->irq_line); printk("\n");

    /* BAR0 holds the controller registers.  NVMe BARs are 64-bit typed and may
     * sit above 4 GiB, which the 32-bit identity MMIO map cannot reach — the
     * old raw read also silently dropped the upper half of such a BAR. */
    uint64_t bar64 = 0;
    uint64_t bsz64 = 0;
    if (pdev->bars[0].base) {
        if (pdev->bars[0].base >= 0x100000000ULL) {
            printk("\x01" "3" "NVMe (kmod): BAR0 above 4 GiB — not supported");
            return -1;
        }
        bar64 = pdev->bars[0].base;
        bsz64 = pdev->bars[0].size;
    } else {
        uint32_t bar0 = pci_read_config_dword(ndev.pci_bus, ndev.pci_dev,
                                              ndev.pci_fn, 0x10);
        bar64 = bar0 & 0xFFFFFFF0u;
        bsz64 = 0x4000;
    }
    ndev.bar_phys = (uint32_t)bar64;
    if (!ndev.bar_phys) {
        printk("\x01" "3" "NVMe (kmod): BAR0 missing");
        return -1;
    }
    /* Controller registers (0x1000) plus one doorbell pair per queue.  Map what
     * the BAR claims, floored at the register block and capped at 64 KiB. */
    if (bsz64 < 0x2000)  bsz64 = 0x2000;
    if (bsz64 > 0x10000) bsz64 = 0x10000;
    ndev.bar_size = (uint32_t)bsz64;

    if (nvme_alloc_queues() < 0) {
        printk("\x01" "3" "NVMe (kmod): queue allocation failed");
        nvme_detach();
        return -1;
    }

    for (uint32_t off = 0; off < ndev.bar_size; off += 0x1000)
        vmm_map(get_current_pd(), ndev.bar_phys + off, ndev.bar_phys + off,
                PAGE_PRESENT | PAGE_RW | PAGE_PCD | PAGE_PWT);
    ndev.bar = (volatile struct nvme_bar *)(uintptr_t)ndev.bar_phys;

    /* A non-0x02 prog_if was accepted on the class code alone; the capability
     * registers are the only evidence that this is an NVMe controller, so
     * check them before the reset sequence touches anything. */
    if (!std_prog_if && !nvme_cap_sane()) {
        printk("\x01" "3" "NVMe (kmod): prog_if is not NVMe and the BAR does "
                         "not look like an NVMe register block");
        nvme_detach();
        return -1;
    }

    saved_pci_cmd_dw = pci_read_config_dword(ndev.pci_bus, ndev.pci_dev,
                                             ndev.pci_fn, 0x04);
    uint32_t cmd = saved_pci_cmd_dw;
    cmd |= 0x06u;           /* MEM | BUS_MASTER */
    pci_write_config_dword(ndev.pci_bus, ndev.pci_dev, ndev.pci_fn, 0x04, cmd);

    /* Register the completion handler: msidev_register() picks MSI-X when the
     * controller offers it and MSI otherwise.  Completions are still polled as
     * well, so a controller that never raises the interrupt keeps working. */
    nvme_msix_vector = -1;
    {
        int vec = msidev_register(pdev, nvme_irq_handler);
        if (vec > 0) {
            nvme_msix_vector = vec;
            nvme_irq_armed   = 1;
            printk("\x01" "6" "NVMe: IRQ enabled");
        } else {
            printk("\x01" "4" "NVMe: no MSI/MSI-X — poll-only");
        }
    }

    uint64_t cap   = ndev.bar->cap;
    ndev.db_stride = 4u << ((uint32_t)((cap >> 32) & 0xF));
    uint32_t mqes  = (uint32_t)(cap & 0xFFFF) + 1;    /* entries, per CAP.MQES */

    /* Queue depths are bounded by CAP.MQES; buffers are allocated at maximum. */
    ndev.admin_q.depth = (mqes < NVME_ADMIN_QUEUE_SIZE) ? (uint16_t)mqes
                                                       : NVME_ADMIN_QUEUE_SIZE;
    if (ndev.admin_q.depth < 2) ndev.admin_q.depth = 2;
    ndev.io_q.depth    = (mqes < NVME_IO_QUEUE_SIZE) ? (uint16_t)mqes
                                                    : NVME_IO_QUEUE_SIZE;
    if (ndev.io_q.depth < 2) ndev.io_q.depth = 2;

    printk("[NVMe] resetting controller\n");
    ndev.bar->intms = 0xFFFFFFFFu;
    ndev.bar->cc    = 0;
    if (nvme_wait_ready(0) < 0) {
        printk("\x01" "3" "NVMe (kmod): controller did not disable");
        nvme_detach();
        return -1;
    }

    nvme_setup_admin_queues();

    printk("[NVMe] enabling controller\n");
    ndev.bar->cc = (4u << 20) | (6u << 16) | (0u << 7) | 1u;
    if (nvme_wait_ready(1) < 0) {
        printk("\x01" "3" "NVMe (kmod): controller did not enable");
        nvme_detach();
        return -1;
    }
    if (ndev.bar->csts & 0x2) {
        printk("\x01" "3" "NVMe (kmod): controller fatal status");
        nvme_detach();
        return -1;
    }

    if (nvme_identify() < 0) {
        printk("\x01" "3" "NVMe (kmod): identify failed");
        nvme_detach();
        return -1;
    }
    if (nvme_set_num_queues() < 0 || nvme_create_io_queues() < 0) {
        printk("\x01" "3" "NVMe (kmod): IO queue setup failed");
        nvme_detach();
        return -1;
    }

    /* Queues are live — unmask completion interrupts (INTMS was set by the
     * reset path and is never cleared by the enable sequence). */
    ndev.bar->intmc = 0xFFFFFFFFu;

    nvme_ready = 1;
    nvme_attached = 1;

    /* Flat node at /dev/nvme0 — the Linux layout; /dev/disk holds only the
     * by-id symlinks (published by vfsdev from the disk identity). */
    if (register_chrdev("nvme0", DEVFS_F_BLOCK, &nvme_drv, 0))
        devfs_was_registered = 1;
    else
        printk("\x01" "4" "NVMe (kmod): register_chrdev('nvme0') failed");

    if (register_blkdev("nvme0", ndev.max_lba, nvme_blk_read_sector,
                        nvme_blk_write_sector) != 0)
        printk("\x01" "4" "NVMe (kmod): register_blkdev('nvme0') failed");

    /* Stable identity for /dev/disk/by-id from the Identify Controller
     * model + serial, spaces turned into '_'. */
    {
        char ident[80];
        snprintf(ident, sizeof(ident), "nvme-%s_%s", nvme_model, nvme_serial);
        for (char *q = ident; *q; q++)
            if (*q == ' ') *q = '_';
        blkdev_set_ident("nvme0", ident);
    }

    {
        char tmp[32];
        printk("[NVMe] nsid=");
        snprintf(tmp, sizeof(tmp), "%d", (int)ndev.ns_id);
        printk(tmp);
        printk(" blocksize=");
        snprintf(tmp, sizeof(tmp), "%d", (int)ndev.ns_sector_size);
        printk(tmp);
        printk(" sectors(512B)=");
        snprintf(tmp, sizeof(tmp), "%d", (int)ndev.max_lba);
        printk(tmp);
        printk(" iodepth=");
        snprintf(tmp, sizeof(tmp), "%d", (int)ndev.io_q.depth);
        printk(tmp);
        printk("\n");
    }
    printk("\x01" "6" "NVMe (kmod): attached - /dev/nvme0 registered");
    return 0;
}

void pci_driver_remove(pci_device_t *dev) {
    (void)dev;
    int had_any = nvme_attached || devfs_was_registered || ndev.bar != NULL;
    nvme_detach();
    if (had_any)
        printk("6" "NVMe (kmod): unloaded");
}
