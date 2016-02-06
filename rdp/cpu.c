//
// rdp/cpu.c: RDP processor container.
//
// CEN64: Cycle-Accurate Nintendo 64 Emulator.
// Copyright (C) 2015, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#include "common.h"
#include "rdp/cpu.h"
#include "thread.h"

#ifdef DEBUG_MMIO_REGISTER_ACCESS
const char *dp_register_mnemonics[NUM_DP_REGISTERS] = {
#define X(reg) #reg,
#include "rdp/registers.md"
#undef X
};
#endif

#define DP_STATUS_END_VALID		    0x00000200

void rdp_process_list(void);

// Sets the opaque pointer used for external accesses.
static void rdp_connect_bus(struct rdp *rdp, struct bus_controller *bus) {
  rdp->bus = bus;
}

// Initializes the RDP component.
int rdp_init(struct rdp *rdp, struct bus_controller *bus) {
  rdp_connect_bus(rdp, bus);

  if (cen64_mutex_create(&rdp->sync_mutex)) {
    printf("Failed to create the RDP synchronization mutex.\n");
    return 1;
  }

  if (cen64_cv_create(&rdp->sync_cv)) {
    printf("Failed to create the RDP synchronization CV.\n");
    cen64_mutex_destroy(&rdp->sync_mutex);
    return 2;
  }

  if (cen64_thread_create(&rdp->thread, rdp_thread, rdp)) {
    printf("Failed to create the RDP thread.\n");
    return 3;
  }

  return 0;
}

CEN64_THREAD_RETURN_TYPE rdp_thread(void *opaque) {
  struct rdp *rdp = (struct rdp *) opaque;

  cen64_mutex_lock(&rdp->sync_mutex);

  while (1) {
    pthread_cond_wait(&rdp->sync_cv, &rdp->sync_mutex);

    // RDP might have written to us while we were busy in the pipe.
    // Better check to see if we have more imcoming commands here!
    while (rdp->regs[DPC_STATUS_REG] & DP_STATUS_END_VALID)
      rdp_process_list();
  }

  cen64_mutex_unlock(&rdp->sync_mutex);
  return CEN64_THREAD_RETURN_VAL;
}

