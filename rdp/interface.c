//
// rdp/interface.c: RDP interface.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "bus/address.h"
#include "rdp/cpu.h"
#include "rdp/interface.h"

#define DP_XBUS_DMEM_DMA          0x00000001
#define DP_FREEZE                 0x00000002
#define DP_FLUSH                  0x00000004

#define DP_CLEAR_XBUS_DMEM_DMA    0x00000001
#define DP_SET_XBUS_DMEM_DMA      0x00000002
#define DP_CLEAR_FREEZE           0x00000004
#define DP_SET_FREEZE             0x00000008
#define DP_CLEAR_FLUSH            0x00000010
#define DP_SET_FLUSH              0x00000020

#define DP_STATUS_CMD_BUSY			  0x00000040
#define DP_STATUS_CBUF_READY		  0x00000080
#define DP_STATUS_DMA_BUSY			  0x00000100
#define DP_STATUS_END_VALID		    0x00000200
#define DP_STATUS_START_VALID		  0x00000400

// Reads a word from the DP MMIO register space.
int read_dp_regs(void *opaque, uint32_t address, uint32_t *word) {
  struct rdp *rdp = (struct rdp *) opaque;
  uint32_t offset = address - DP_REGS_BASE_ADDRESS;
  enum dp_register reg = (offset >> 2);

  pthread_mutex_lock(&rdp->sync_mutex);
  *word = rdp->regs[reg];
  pthread_mutex_unlock(&rdp->sync_mutex);

  debug_mmio_read(dp, dp_register_mnemonics[reg], *word);
  return 0;
}

// Writes a word to the DP MMIO register space.
int write_dp_regs(void *opaque, uint32_t address, uint32_t word, uint32_t dqm) {
  struct rdp *rdp = (struct rdp *) opaque;
  uint32_t offset = address - DP_REGS_BASE_ADDRESS;
  enum dp_register reg = (offset >> 2);

  debug_mmio_write(dp, dp_register_mnemonics[reg], word, dqm);

  cen64_mutex_lock(&rdp->sync_mutex);

  switch (reg) {
    case DPC_START_REG:
      rdp->regs[DPC_CURRENT_REG] = word;
      rdp->regs[DPC_START_REG] = word;
      rdp->regs[DPC_STATUS_REG] |= DP_STATUS_START_VALID;
      break;

    case DPC_END_REG:
      rdp->regs[DPC_END_REG] = word;
      rdp->regs[DPC_STATUS_REG] &= ~DP_STATUS_CBUF_READY;
      rdp->regs[DPC_STATUS_REG] |= DP_STATUS_END_VALID |
        DP_STATUS_CMD_BUSY | DP_STATUS_DMA_BUSY;
      cen64_mutex_unlock(&rdp->sync_mutex);
      pthread_cond_signal(&rdp->sync_cv);
      return 0;

    case DPC_STATUS_REG:
      if (word & DP_CLEAR_XBUS_DMEM_DMA)
        rdp->regs[DPC_STATUS_REG] &= ~DP_XBUS_DMEM_DMA;
      else if (word & DP_SET_XBUS_DMEM_DMA)
        rdp->regs[DPC_STATUS_REG] |= DP_XBUS_DMEM_DMA;

      if (word & DP_CLEAR_FREEZE)
        rdp->regs[DPC_STATUS_REG] &= ~DP_FREEZE;
//      else if (word & DP_SET_FREEZE)
//        rdp->regs[DPC_STATUS_REG] |= DP_FREEZE;

      if (word & DP_CLEAR_FLUSH)
        rdp->regs[DPC_STATUS_REG] &= ~DP_FLUSH;
      else if (word & DP_SET_FLUSH)
        rdp->regs[DPC_STATUS_REG] |= DP_FLUSH;
      break;

    default:
      rdp->regs[reg] &= ~dqm;
      rdp->regs[reg] |= word;
      break;
  }

  cen64_mutex_unlock(&rdp->sync_mutex);
  return 0;
}

