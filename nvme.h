#ifndef NVME_MOD_H
#define NVME_MOD_H

#include <stdint.h>

#define NVME_ADMIN_QUEUE_SIZE  16
#define NVME_IO_QUEUE_SIZE     64
#define NVME_SECTOR_SIZE       512
// Largest addressable LBA in the 32-bit block layer (2 TiB of 512-byte
// sectors).  A bigger namespace is clamped to this, with a warning.
#define NVME_MAX_LBA           0xFFFFFFFFu
// Highest logical-block size supported.  A namespace with 4096-byte blocks
// (4Kn) is reached through a bounce block; anything above that is refused.
#define NVME_MAX_LBS           4096

#define NVME_OPC_IDENTIFY      0x06
#define NVME_OPC_CREATE_IOSQ   0x01
#define NVME_OPC_CREATE_IOCQ   0x05
#define NVME_OPC_SET_FEATURES  0x09

#define NVME_IO_WRITE          0x01
#define NVME_IO_READ           0x02

/* 64-byte NVMe submission queue entry (NVMe base spec, Figure "Common Command
 * Format"). Byte layout: CDW0 @0, NSID @4, cdw2/3 reserved @8, MPTR @16,
 * PRP1 @24, PRP2 @32, CDW10..CDW15 @40..63. */
struct nvme_sq_entry {
    uint32_t cdw0;
    uint32_t nsid;
    uint64_t rsvd;     /* cdw2 / cdw3 */
    uint64_t mptr;
    uint64_t prp1;
    uint64_t prp2;
    uint32_t cdw10;
    uint32_t cdw11;
    uint32_t cdw12;
    uint32_t cdw13;
    uint32_t cdw14;
    uint32_t cdw15;
} __attribute__((packed));

struct nvme_cq_entry {
    uint32_t result;
    uint32_t rsvd;
    uint16_t sq_head;
    uint16_t sq_id;
    uint16_t cid;
    uint16_t status;
} __attribute__((packed));

/* MMIO layout (NVMe base spec): CAP 0x00, VS 0x08, INTMS 0x0c, INTMC 0x10,
 * CC 0x14, rsvd 0x18, CSTS 0x1c, NSSR 0x20, AQA 0x24, ASQ 0x28, ACQ 0x30. */
struct nvme_bar {
    uint64_t cap;
    uint32_t vs;
    uint32_t intms;
    uint32_t intmc;
    uint32_t cc;
    uint32_t rsvd;
    uint32_t csts;
    uint32_t nssr;
    uint32_t aqa;
    uint64_t asq;
    uint64_t acq;
} __attribute__((packed));

struct nvme_queue {
    volatile struct nvme_sq_entry *sq;
    volatile struct nvme_cq_entry *cq;
    volatile uint32_t             *sq_db;
    volatile uint32_t             *cq_db;
    uint16_t sq_tail;
    uint16_t cq_head;
    uint16_t cq_phase;
    uint16_t depth;
    uint32_t last_result;          // CQE.result of the last reaped command
};

struct nvme_dev {
    volatile struct nvme_bar *bar;
    uint32_t                  bar_phys;
    uint32_t                  bar_size;
    uint32_t                  db_stride;
    uint32_t                  ns_id;
    uint32_t                  max_lba;        // capacity in 512-byte sectors
    uint32_t                  ns_sector_size; // logical block size (512/4096)
    struct nvme_queue         admin_q;
    struct nvme_queue         io_q;
    uint8_t                   pci_bus;
    uint8_t                   pci_dev;
    uint8_t                   pci_fn;
};

#endif
