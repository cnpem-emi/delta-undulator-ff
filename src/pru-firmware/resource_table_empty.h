/*
 * resource_table_empty.h
  *  ======== resource_table_empty.h ========
 *
 *  Define the resource table entries for all PRU cores. This will be
 *  incorporated into corresponding base images, and used by the remoteproc
 *  on the host-side to allocated/reserve resources.  Note the remoteproc
 *  driver requires that all PRU firmware be built with a resource table.
 *
 *  This file contains an empty resource table.  It can be used either as:
 *
 *        1) A template, or
 *        2) As-is if a PRU application does not need to configure PRU_INTC
 *                  or interact with the rpmsg driver
 *
 */

#ifndef _RSC_TABLE_PRU_H_
#define _RSC_TABLE_PRU_H_

#include <stddef.h>
#include <rsc_types.h>
#include "pru_virtio_ids.h"

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define PRU_RPMSG_VQ0_SIZE  2
#define PRU_RPMSG_VQ1_SIZE  2

/* flip up bits whose indices represent features we support */
#define RPMSG_PRU_C0_FEATURES   1

/* Definition for unused interrupts */
#define HOST_UNUSED     255

/* Mapping sysevts to a channel. Each pair contains a sysevt, channel */
struct ch_map pru_intc_map[] = {};

struct my_resource_table {
    struct resource_table base;

    uint32_t offset[1]; /* Should match 'num' in actual definition */

    /* intc definition */
    struct fw_rsc_custom pru_ints;
};

#pragma DATA_SECTION(pru_remoteproc_ResourceTable, ".resource_table")
#pragma RETAIN(pru_remoteproc_ResourceTable)
struct my_resource_table pru_remoteproc_ResourceTable = {
    1,  /* we're the first version that implements this */
    0,  /* number of entries in the table */
    0, 0,   /* reserved, must be zero */
    0,  /* offset[0] */
};

#endif /* _RSC_TABLE_PRU_H_ */



