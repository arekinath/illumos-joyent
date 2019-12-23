/*
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 */

/*
 * Copyright 2019, The University of Queensland
 * Copyright (c) 2018, Joyent, Inc. 
 */

/*
 * Mellanox Connect-X 4/5/6 driver.
 */

/*
 * The PRM for this family of parts is freely available, and can be found at:
 * https://www.mellanox.com/related-docs/user_manuals/ \
 *   Ethernet_Adapters_Programming_Manual.pdf
 */

/*
 *
 * Data flow overview
 * ------------------
 *
 * This driver is a MAC ring-enabled driver which maps rings to send and recv
 * queues in hardware on the device.
 *
 * Each SQ and RQ is set up to report to its own individual CQ, to ensure
 * sufficient space, and simplify the logic needed to work out which buffer
 * was completed.
 *
 * The CQs are then round-robin allocated onto EQs, of which we set up one per
 * interrupt that the system gives us for the device. Normally this means we
 * have 8 EQs.
 *
 * EQ #0 is reserved for all event types other than completion events, and has
 * no CQs associated with it at any time. EQs #1 and upwards are only used for
 * handling CQ completion events.
 *
 * +------+     +------+           +------+        +---------+
 * | SQ 0 |---->| CQ 0 |-----+     | EQ 0 |------> | MSI-X 0 |     mlxcx_intr_0
 * +------+     +------+     |     +------+        +---------+
 *                           |
 * +------+     +------+     |
 * | SQ 1 |---->| CQ 1 |---+ |     +------+
 * +------+     +------+   | +---> |      |
 *                         |       |      |
 * +------+     +------+   |       | EQ 1 |        +---------+
 * | SQ 2 |---->| CQ 2 |---------> |      |------> | MSI-X 1 |     mlxcx_intr_n
 * +------+     +------+   | +---> |      |        +---------+
 *                         | |     +------+
 *                         | |
 *   ...                   | |
 *                         | |     +------+
 * +------+     +------+   +-----> |      |
 * | RQ 0 |---->| CQ 3 |---------> |      |        +---------+
 * +------+     +------+     |     | EQ 2 |------> | MSI-X 2 |     mlxcx_intr_n
 *                           |     |      |        +---------+
 * +------+     +------+     | +-> |      |
 * | RQ 1 |---->| CQ 4 |-----+ |   +------+
 * +------+     +------+       |
 *                             |     ....
 * +------+     +------+       |
 * | RQ 2 |---->| CQ 5 |-------+
 * +------+     +------+
 *
 *   ...
 *
 * For TX, we advertise all of the SQs we create as plain rings to MAC with
 * no TX groups. This puts MAC in "virtual group" mode where it will allocate
 * and use the rings as it sees fit.
 *
 * For RX, we advertise actual groups in order to make use of hardware
 * classification.
 *
 * The hardware classification we use is based around Flow Tables, and we
 * currently ignore all of the eswitch features of the card. The NIC VPORT
 * is always set to promisc mode so that the eswitch sends us all of the
 * traffic that arrives on the NIC, and we use flow entries to manage
 * everything.
 *
 * We use 2 layers of flow tables for classification: traffic arrives at the
 * root RX flow table which contains MAC address filters. Those then send
 * matched traffic to the per-group L1 VLAN filter tables which contain VLAN
 * presence and VID filters.
 *
 * Since these parts only support doing RSS hashing on a single protocol at a
 * time, we have to use a third layer of flow tables as well to break traffic
 * down by L4 and L3 protocol (TCPv6, TCPv4, UDPv6, UDPv4, IPv6, IPv4 etc)
 * so that it can be sent to the appropriate TIR for hashing.
 *
 * Incoming packets
 *        +           +---------+      +---------+
 *        |        +->| group 0 |      | group 0 |
 *        |        |  | vlan ft |  +-->| hash ft |
 *        v        |  |   L1    |  |   |   L2    |
 *   +----+----+   |  +---------+  |   +---------+    +-----+    +-----+------+
 *   | eswitch |   |  |         |  |   |  TCPv6  |--->| TIR |--->|     |  RQ0 |
 *   +----+----+   |  |         |  |   +---------+    +-----+    |     +------+
 *        |        |  |         |  |   |  UDPv6  |--->| TIR |--->|     |  RQ1 |
 *        |        |  |         |  |   +---------+    +-----+    |     +------+
 *        |        |  |         |  |   |  TCPv4  |--->| TIR |--->|     |  RQ2 |
 *        v        |  |         |  |   +---------+    +-----+    | RQT +------+
 *   +----+----+   |  +---------+  |   |  UDPv4  |--->| TIR |--->|     |  ... |
 *   | root rx |   |  | default |--+   +---------+    +-----+    |     |      |
 *   | flow tb |   |  +---------+  |   |  IPv6   |--->| TIR |--->|     |      |
 *   |    L0   |   |  | promisc |--+   +---------+    +-----+    |     |      |
 *   +---------+   |  +---------+  ^   |  IPv4   |--->| TIR |--->|     |      |
 *   |  bcast  |---+               |   +---------+    +-----+    +-----+------+
 *   +---------+   |               |   |  other  |-+
 *   |  MAC 0  |---+               |   +---------+ |  +-----+    +-----+
 *   +---------+                   |               +->| TIR |--->| RQ0 |
 *   |  MAC 1  |-+                 |                  +-----+    +-----+
 *   +---------+ | +---------------+
 *   |  MAC 2  |-+ |               ^
 *   +---------+ | |               |
 *   |  MAC 3  |-+ |  +---------+  |   +---------+
 *   +---------+ | |  | group 1 |  |   | group 1 |
 *   |  .....  | +--->| vlan ft |  | +>| hash ft |
 *   |         |   |  |   L1    |  | | |   L2    |
 *   +---------+   |  +---------+  | | +---------+    +-----+    +-----+------+
 *   | promisc |---+  | VLAN 0  |----+ |  TCPv6  |--->| TIR |--->|     |  RQ3 |
 *   +---------+      +---------+  |   +---------+    +-----+    |     +------+
 *                    |  .....  |  |   |  UDPv6  |--->| TIR |--->|     |  RQ4 |
 *                    |         |  |   +---------+    +-----+    |     +------+
 *                    |         |  |   |  TCPv4  |--->| TIR |--->|     |  RQ5 |
 *                    |         |  |   +---------+    +-----+    | RQT +------+
 *                    +---------+  |   |  UDPv4  |--->| TIR |--->|     |  ... |
 *                    |         |  |   +---------+    +-----+    |     |      |
 *                    +---------+  |   |  IPv6   |--->| TIR |--->|     |      |
 *                    | promisc |--+   +---------+    +-----+    |     |      |
 *                    +---------+      |  IPv4   |--->| TIR |--->|     |      |
 *                                     +---------+    +-----+    +-----+------+
 *                                     |  other  |-+
 *                                     +---------+ |
 *                      .......                    |  +-----+    +-----+
 *                                                 +->| TIR |--->| RQ3 |
 *                                                    +-----+    +-----+
 *
 * Note that the "promisc" flow entries are only set/enabled when promisc
 * mode is enabled for the NIC. All promisc flow entries point directly at
 * group 0's hashing flowtable (so all promisc-only traffic lands on group 0,
 * the "default group" in MAC).
 *
 * The "default" entry in the L1 VLAN filter flow tables is used when there
 * are no VLANs set for the group, to accept any traffic regardless of tag. It
 * is deleted as soon as a VLAN filter is added (and re-instated if the
 * last VLAN filter is removed).
 *
 * The actual descriptor ring structures for RX on Connect-X4 don't contain any
 * space for packet data (they're a collection of scatter pointers only). TX
 * descriptors contain some space for "inline headers" (and the card requires
 * us to put at least the L2 Ethernet headers there for the eswitch to look at)
 * but all the rest of the data comes from the gather pointers.
 *
 * When we get completions back they simply contain the ring index number of
 * the WR (work request) which completed. So, we manage the buffers for actual
 * packet data completely independently of the descriptors in this driver. When
 * a WR is enqueued in a WQE (work queue entry), we stamp the packet data buffer
 * with the WQE index that we put it at, and therefore don't have to look at
 * the original descriptor at all when handling completions.
 *
 * For RX, we create sufficient packet data buffers to fill 150% of the
 * available descriptors for each ring. These all are pre-set-up for DMA and
 * have an mblk_t associated with them (with desballoc()).
 *
 * For TX we either borrow the mblk's memory and DMA bind it (if the packet is
 * large enough), or we copy it into a pre-allocated buffer set up in the same
 * as as for RX.
 */

/*
 * Buffer lifecycle: RX
 * --------------------
 *
 * The lifecycle of an mlxcx_buffer_t (packet buffer) used for RX is pretty
 * straightforward.
 *
 * It is created (and has all its memory allocated) at the time of starting up
 * the RX ring it belongs to. Then it is placed on the "free" list in the
 * mlxcx_buffer_shard_t associated with its RQ. When mlxcx_rq_refill() wants
 * more buffers to add to the RQ, it takes one off and marks it as "on WQ"
 * before making a WQE for it.
 *
 * After a completion event occurs, the packet is either discarded (and the
 * buffer_t returned to the free list), or it is readied for loaning to MAC.
 *
 * Once MAC and the rest of the system have finished with the packet, they call
 * freemsg() on its mblk, which will call mlxcx_buf_mp_return and return the
 * buffer_t to the free list.
 *
 * At detach/teardown time, buffers are only every destroyed from the free list.
 *
 *
 *                         +
 *                         |
 *                         | mlxcx_buf_create
 *                         |
 *                         v
 *                    +----+----+
 *                    | created |
 *                    +----+----+
 *                         |
 *                         |
 *                         | mlxcx_buf_return
 *                         |
 *                         v
 * mlxcx_buf_destroy  +----+----+
 *          +---------|  free   |<---------------+
 *          |         +----+----+                |
 *          |              |                     |
 *          |              |                     | mlxcx_buf_return
 *          v              | mlxcx_buf_take      |
 *      +---+--+           v                     |
 *      | dead |       +---+---+                 |
 *      +------+       | on WQ |- - - - - - - - >O
 *                     +---+---+                 ^
 *                         |                     |
 *                         |                     |
 *                         | mlxcx_buf_loan      | mlxcx_buf_mp_return
 *                         v                     |
 *                 +-------+--------+            |
 *                 | on loan to MAC |----------->O
 *                 +----------------+  freemsg()
 *
 */

/*
 * Buffer lifecycle: TX
 * --------------------
 *
 * mlxcx_buffer_ts used for TX are divided into two kinds: regular buffers, and
 * "foreign" buffers.
 *
 * The former have their memory allocated and DMA bound by this driver, while
 * the latter (the "foreign" buffers) are on loan from MAC. Their memory is
 * not owned by us, though we do DMA bind it (and take responsibility for
 * un-binding it when we're done with them).
 *
 * We use separate mlxcx_buf_shard_ts for foreign and local buffers on each
 * SQ. Thus, there is a separate free list and mutex for each kind.
 *
 * Since a TX packet might consist of multiple mblks, we translate each mblk
 * into exactly one buffer_t. The buffer_ts are chained together in the same
 * order as the mblks, using the mlb_tx_chain/mlb_tx_chain_entry list_t.
 *
 * Each chain of TX buffers may consist of foreign or driver buffers, in any
 * mixture.
 *
 * The head of a TX buffer chain has mlb_tx_head == itself, which distinguishes
 * it from the rest of the chain buffers.
 *
 * TX buffer chains are always returned to the free list by
 * mlxcx_buf_return_chain(), which takes care of walking the mlb_tx_chain and
 * freeing all of the members.
 *
 * We only call freemsg() once, on the head of the TX buffer chain's original
 * mblk. This is true whether we copied it or bound it in a foreign buffer.
 */

/*
 * Startup and command interface
 * -----------------------------
 *
 * The command interface is the primary way in which we give control orders to
 * the hardware (e.g. actions like "create this queue" or "delete this flow
 * entry"). The command interface is never used to transmit or receive packets
 * -- that takes place only on the queues that are set up through it.
 *
 * In mlxcx_cmd.c we implement our use of the command interface on top of a
 * simple taskq. Since it's not performance critical, we busy-wait on command
 * completions and only process a single command at a time.
 *
 * If this becomes a problem later we can wire command completions up to EQ 0
 * once we have interrupts running.
 *
 * The startup/attach process for this card involves a bunch of different steps
 * which are summarised pretty well in the PRM. We have to send a number of
 * commands which do different things to start the card up, give it some pages
 * of our own memory for it to use, then start creating all the entities that
 * we need to use like EQs, CQs, WQs, as well as their dependencies like PDs
 * and TDoms.
 */

/*
 * UARs
 * ----
 *
 * The pages of the PCI BAR other than the first few are reserved for use as
 * "UAR" sections in this device. Each UAR section can be used as a set of
 * doorbells for our queues.
 *
 * Currently we just make one single UAR for all of our queues. It doesn't
 * seem to be a major limitation yet.
 *
 * When we're sending packets through an SQ, the PRM is not awful clear about
 * exactly how we're meant to use the first 16 bytes of the Blueflame buffers
 * (it's clear on the pattern of alternation you're expected to use between
 * even and odd for Blueflame sends, but not for regular doorbells).
 *
 * Currently we don't do the even-odd alternating pattern for ordinary
 * doorbells, and we don't use Blueflame at all. This seems to work fine, at
 * least on Connect-X4 Lx.
 */

/*
 * Lock ordering
 * -------------
 *
 * Interrupt side:
 *
 *  - mleq_mtx
 *    - mlcq_mtx
 *      - mlcq_bufbmtx
 *      - mlwq_mtx
 *        - mlbs_mtx
 *    - mlp_mtx
 *
 * GLD side:
 *
 *  - mlp_mtx
 *    - mlg_mtx
 *      - mlg_*.mlft_mtx
 *    - mlp_*.mlft_mtx
 *    - mlwq_mtx
 *      - mlbs_mtx
 *      - mlcq_bufbmtx
 *  - mleq_mtx
 *    - mlcq_mtx
 *
 */

#include <sys/modctl.h>
#include <sys/conf.h>
#include <sys/devops.h>
#include <sys/sysmacros.h>

#include <sys/mac_provider.h>

#include <mlxcx.h>

#define	MLXCX_MODULE_NAME	"mlxcx"
/*
 * We give this to the firmware, so it has to be in a fixed format that it
 * understands.
 */
#define	MLXCX_DRIVER_VERSION	"illumos,mlxcx,1.0.0,1,000,000000"

/*
 * Firmware may take a while to reclaim pages. Try a set number of times.
 */
clock_t mlxcx_reclaim_delay = 1000 * 50; /* 50 ms in us */
uint_t mlxcx_reclaim_tries = 100; /* Wait at most 5000ms */

/* These are all set by mlxcx_load_props() based on mlxcx.conf */
uint_t mlxcx_eq_size_shift;
uint_t mlxcx_cq_size_shift;
uint_t mlxcx_rq_size_shift;
uint_t mlxcx_sq_size_shift;
uint_t mlxcx_cqemod_period_usec;
uint_t mlxcx_cqemod_count;
uint_t mlxcx_intrmod_period_usec;
uint_t mlxcx_rx_ngroups_large;
uint_t mlxcx_rx_ngroups_small;
uint_t mlxcx_rx_nrings_per_large_group;
uint_t mlxcx_rx_nrings_per_small_group;
uint_t mlxcx_tx_ngroups;
uint_t mlxcx_tx_nrings_per_group;
uint_t mlxcx_ftbl_root_size_shift;
size_t mlxcx_tx_bind_threshold;
uint_t mlxcx_ftbl_vlan_size_shift;

/*
 * Fault detection thresholds.
 */
uint_t mlxcx_doorbell_tries = MLXCX_DOORBELL_TRIES_DFLT;
uint_t mlxcx_stuck_intr_count = MLXCX_STUCK_INTR_COUNT_DFLT;

static void
mlxcx_load_props(mlxcx_t *mlxp)
{
	mlxcx_eq_size_shift = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "eq_size_shift",
	    MLXCX_EQ_SIZE_SHIFT_DFLT);
	mlxcx_cq_size_shift = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "cq_size_shift",
	    MLXCX_CQ_SIZE_SHIFT_DFLT);
	mlxcx_sq_size_shift = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "sq_size_shift",
	    MLXCX_SQ_SIZE_SHIFT_DFLT);
	mlxcx_rq_size_shift = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "rq_size_shift",
	    MLXCX_RQ_SIZE_SHIFT_DFLT);

	mlxcx_cqemod_period_usec = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "cqemod_period_usec",
	    MLXCX_CQEMOD_PERIOD_USEC_DFLT);
	mlxcx_cqemod_count = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "cqemod_count",
	    MLXCX_CQEMOD_COUNT_DFLT);
	mlxcx_intrmod_period_usec = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "intrmod_period_usec",
	    MLXCX_INTRMOD_PERIOD_USEC_DFLT);

	mlxcx_tx_ngroups = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "tx_ngroups",
	    MLXCX_TX_NGROUPS_DFLT);
	mlxcx_tx_nrings_per_group = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "tx_nrings_per_group",
	    MLXCX_TX_NRINGS_PER_GROUP_DFLT);

	mlxcx_rx_ngroups_large = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "rx_ngroups_large",
	    MLXCX_RX_NGROUPS_LARGE_DFLT);
	mlxcx_rx_ngroups_small = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "rx_ngroups_small",
	    MLXCX_RX_NGROUPS_SMALL_DFLT);
	mlxcx_rx_nrings_per_large_group = ddi_getprop(DDI_DEV_T_ANY,
	    mlxp->mlx_dip, DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS,
	    "rx_nrings_per_large_group", MLXCX_RX_NRINGS_PER_LARGE_GROUP_DFLT);
	mlxcx_rx_nrings_per_small_group = ddi_getprop(DDI_DEV_T_ANY,
	    mlxp->mlx_dip, DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS,
	    "rx_nrings_per_small_group", MLXCX_RX_NRINGS_PER_SMALL_GROUP_DFLT);

	mlxcx_ftbl_root_size_shift = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "ftbl_root_size_shift",
	    MLXCX_FTBL_ROOT_SIZE_SHIFT_DFLT);

	mlxcx_tx_bind_threshold = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "tx_bind_threshold",
	    MLXCX_TX_BIND_THRESHOLD_DFLT);

	mlxcx_ftbl_vlan_size_shift = ddi_getprop(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_CANSLEEP | DDI_PROP_DONTPASS, "ftbl_vlan_size_shift",
	    MLXCX_FTBL_VLAN_SIZE_SHIFT_DFLT);
}

void
mlxcx_note(mlxcx_t *mlxp, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (mlxp != NULL && mlxp->mlx_dip != NULL) {
		vdev_err(mlxp->mlx_dip, CE_NOTE, fmt, ap);
	} else {
		vcmn_err(CE_NOTE, fmt, ap);
	}
	va_end(ap);
}

void
mlxcx_warn(mlxcx_t *mlxp, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (mlxp != NULL && mlxp->mlx_dip != NULL) {
		vdev_err(mlxp->mlx_dip, CE_WARN, fmt, ap);
	} else {
		vcmn_err(CE_WARN, fmt, ap);
	}
	va_end(ap);
}

void
mlxcx_panic(mlxcx_t *mlxp, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (mlxp != NULL && mlxp->mlx_dip != NULL) {
		vdev_err(mlxp->mlx_dip, CE_PANIC, fmt, ap);
	} else {
		vcmn_err(CE_PANIC, fmt, ap);
	}
	va_end(ap);
}

uint16_t
mlxcx_get16(mlxcx_t *mlxp, uintptr_t off)
{
	uintptr_t addr = off + (uintptr_t)mlxp->mlx_regs_base;
	return (ddi_get16(mlxp->mlx_regs_handle, (void *)addr));
}

uint32_t
mlxcx_get32(mlxcx_t *mlxp, uintptr_t off)
{
	uintptr_t addr = off + (uintptr_t)mlxp->mlx_regs_base;
	return (ddi_get32(mlxp->mlx_regs_handle, (void *)addr));
}

uint64_t
mlxcx_get64(mlxcx_t *mlxp, uintptr_t off)
{
	uintptr_t addr = off + (uintptr_t)mlxp->mlx_regs_base;
	return (ddi_get64(mlxp->mlx_regs_handle, (void *)addr));
}

void
mlxcx_put32(mlxcx_t *mlxp, uintptr_t off, uint32_t val)
{
	uintptr_t addr = off + (uintptr_t)mlxp->mlx_regs_base;
	ddi_put32(mlxp->mlx_regs_handle, (void *)addr, val);
}

void
mlxcx_put64(mlxcx_t *mlxp, uintptr_t off, uint64_t val)
{
	uintptr_t addr = off + (uintptr_t)mlxp->mlx_regs_base;
	ddi_put64(mlxp->mlx_regs_handle, (void *)addr, val);
}

void
mlxcx_uar_put32(mlxcx_t *mlxp, mlxcx_uar_t *mlu, uintptr_t off, uint32_t val)
{
	uintptr_t addr = off + (uintptr_t)mlu->mlu_base +
	    (uintptr_t)mlxp->mlx_regs_base;
	ddi_put32(mlxp->mlx_regs_handle, (void *)addr, val);
}

void
mlxcx_uar_put64(mlxcx_t *mlxp, mlxcx_uar_t *mlu, uintptr_t off, uint64_t val)
{
	uintptr_t addr = off + (uintptr_t)mlu->mlu_base +
	    (uintptr_t)mlxp->mlx_regs_base;
	ddi_put64(mlxp->mlx_regs_handle, (void *)addr, val);
}

static void
mlxcx_fm_fini(mlxcx_t *mlxp)
{
	if (mlxp->mlx_fm_caps == 0)
		return;

	if (DDI_FM_ERRCB_CAP(mlxp->mlx_fm_caps))
		ddi_fm_handler_unregister(mlxp->mlx_dip);

	if (DDI_FM_EREPORT_CAP(mlxp->mlx_fm_caps) ||
	    DDI_FM_ERRCB_CAP(mlxp->mlx_fm_caps))
		pci_ereport_teardown(mlxp->mlx_dip);

	ddi_fm_fini(mlxp->mlx_dip);
}

void
mlxcx_fm_ereport(mlxcx_t *mlxp, const char *detail)
{
	uint64_t ena;
	char buf[FM_MAX_CLASS];

	(void) snprintf(buf, FM_MAX_CLASS, "%s.%s", DDI_FM_DEVICE, detail);
	ena = fm_ena_generate(0, FM_ENA_FMT1);
	if (DDI_FM_EREPORT_CAP(mlxp->mlx_fm_caps)) {
		ddi_fm_ereport_post(mlxp->mlx_dip, buf, ena, DDI_NOSLEEP,
		    FM_VERSION, DATA_TYPE_UINT8, FM_EREPORT_VERS0,
		    NULL);
	}
}	

static void
mlxcx_fm_init(mlxcx_t *mlxp)
{
	ddi_iblock_cookie_t iblk;
	/* XXX Come back and add error handler some day */
	int def = DDI_FM_EREPORT_CAPABLE | DDI_FM_ACCCHK_CAPABLE |
	    DDI_FM_DMACHK_CAPABLE;

	mlxp->mlx_fm_caps = ddi_prop_get_int(DDI_DEV_T_ANY, mlxp->mlx_dip,
	    DDI_PROP_DONTPASS, "fm_capable", def);

	if (mlxp->mlx_fm_caps < 0) {
		mlxp->mlx_fm_caps = 0;
	} else if (mlxp->mlx_fm_caps & ~def) {
		mlxp->mlx_fm_caps &= def;
	}

	if (mlxp->mlx_fm_caps == 0)
		return;

	ddi_fm_init(mlxp->mlx_dip, &mlxp->mlx_fm_caps, &iblk);
	if (DDI_FM_EREPORT_CAP(mlxp->mlx_fm_caps) ||
	    DDI_FM_ERRCB_CAP(mlxp->mlx_fm_caps)) {
		pci_ereport_setup(mlxp->mlx_dip);
	}
}

static void
mlxcx_mlbs_teardown(mlxcx_t *mlxp, mlxcx_buf_shard_t *s)
{
	mlxcx_buffer_t *buf;

	mutex_enter(&s->mlbs_mtx);
	while (!list_is_empty(&s->mlbs_busy))
		cv_wait(&s->mlbs_free_nonempty, &s->mlbs_mtx);
	while ((buf = list_head(&s->mlbs_free)) != NULL) {
		mlxcx_buf_destroy(mlxp, buf);
	}
	list_destroy(&s->mlbs_free);
	list_destroy(&s->mlbs_busy);
	mutex_exit(&s->mlbs_mtx);

	cv_destroy(&s->mlbs_free_nonempty);
	mutex_destroy(&s->mlbs_mtx);
}

static void
mlxcx_teardown_bufs(mlxcx_t *mlxp)
{
	mlxcx_buf_shard_t *s;

	while ((s = list_remove_head(&mlxp->mlx_buf_shards)) != NULL) {
		mlxcx_mlbs_teardown(mlxp, s);
		kmem_free(s, sizeof (mlxcx_buf_shard_t));
	}
	list_destroy(&mlxp->mlx_buf_shards);

	kmem_cache_destroy(mlxp->mlx_bufs_cache);
}

static void
mlxcx_teardown_pages(mlxcx_t *mlxp)
{
	uint_t nzeros = 0;
	const ddi_dma_cookie_t *ck;

	mutex_enter(&mlxp->mlx_pagemtx);

	while (mlxp->mlx_npages > 0) {
		int32_t req, ret;
		uint64_t pas[MLXCX_MANAGE_PAGES_MAX_PAGES];

		ASSERT0(list_is_empty(&mlxp->mlx_pages));
		req = MIN(mlxp->mlx_npages, MLXCX_MANAGE_PAGES_MAX_PAGES);

		if (!mlxcx_cmd_return_pages(mlxp, req, pas, &ret)) {
			mlxcx_warn(mlxp, "hardware refused to return pages, "
			    "leaking %u remaining pages", mlxp->mlx_npages);
			return;
		}

		for (int32_t i = 0; i < ret; i++) {
			mlxcx_dev_page_t *mdp = NULL;
			for (mdp = list_head(&mlxp->mlx_pages); mdp != NULL;
			    mdp = list_next(&mlxp->mlx_pages, mdp)) {
				ck = mlxcx_dma_cookie_one(&mdp->mxdp_dma);
				if (pas[i] == ck->dmac_laddress) {
					break;
				}
			}

			if (mdp != NULL) {
				list_remove(&mlxp->mlx_pages, mdp);
				mlxp->mlx_npages--;
				mlxcx_dma_free(&mdp->mxdp_dma);
				kmem_free(mdp, sizeof (mlxcx_dev_page_t));
			} else {
				mlxcx_panic(mlxp, "hardware returned a page "
				    "with PA 0x%" PRIx64 " but we have no "
				    "record of giving out such a page", pas[i]);
			}
		}

		/*
		 * If no pages were returned, note that fact.
		 */
		if (ret == 0) {
			nzeros++;
			if (nzeros > mlxcx_reclaim_tries) {
				mlxcx_warn(mlxp, "hardware refused to return "
				    "pages, leaking %u remaining pages",
				    mlxp->mlx_npages);
				return;
			}
			delay(drv_usectohz(mlxcx_reclaim_delay));
		}
	}

	list_destroy(&mlxp->mlx_pages);

	mutex_exit(&mlxp->mlx_pagemtx);
	mutex_destroy(&mlxp->mlx_pagemtx);
}

static boolean_t
mlxcx_eq_alloc_dma(mlxcx_t *mlxp, mlxcx_event_queue_t *mleq)
{
	ddi_device_acc_attr_t acc;
	ddi_dma_attr_t attr;
	boolean_t ret;
	size_t sz, i;

	VERIFY0(mleq->mleq_state & MLXCX_EQ_ALLOC);

	mleq->mleq_entshift = mlxcx_eq_size_shift;
	mleq->mleq_nents = (1 << mleq->mleq_entshift);
	sz = mleq->mleq_nents * sizeof (mlxcx_eventq_ent_t);
	ASSERT3U(sz & (MLXCX_HW_PAGE_SIZE - 1), ==, 0);

	mlxcx_dma_acc_attr(mlxp, &acc);
	mlxcx_dma_queue_attr(mlxp, &attr);

	ret = mlxcx_dma_alloc(mlxp, &mleq->mleq_dma, &attr, &acc,
	    B_TRUE, sz, B_TRUE);
	if (!ret) {
		mlxcx_warn(mlxp, "failed to allocate EQ memory");
		return (B_FALSE);
	}

	mleq->mleq_ent = (mlxcx_eventq_ent_t *)mleq->mleq_dma.mxdb_va;

	for (i = 0; i < mleq->mleq_nents; ++i)
		mleq->mleq_ent[i].mleqe_owner = MLXCX_EQ_OWNER_INIT;

	mleq->mleq_state |= MLXCX_EQ_ALLOC;

	return (B_TRUE);
}

static void
mlxcx_eq_rele_dma(mlxcx_t *mlxp, mlxcx_event_queue_t *mleq)
{
	VERIFY(mleq->mleq_state & MLXCX_EQ_ALLOC);
	if (mleq->mleq_state & MLXCX_EQ_CREATED)
		VERIFY(mleq->mleq_state & MLXCX_EQ_DESTROYED);

	mlxcx_dma_free(&mleq->mleq_dma);
	mleq->mleq_ent = NULL;

	mleq->mleq_state &= ~MLXCX_EQ_ALLOC;
}

void
mlxcx_teardown_flow_table(mlxcx_t *mlxp, mlxcx_flow_table_t *ft)
{
	mlxcx_flow_group_t *fg;
	mlxcx_flow_entry_t *fe;
	int i;

	ASSERT(mutex_owned(&ft->mlft_mtx));

	for (i = ft->mlft_nents - 1; i >= 0; --i) {
		fe = &ft->mlft_ent[i];
		if (fe->mlfe_state & MLXCX_FLOW_ENTRY_CREATED) {
			if (!mlxcx_cmd_delete_flow_table_entry(mlxp, fe)) {
				mlxcx_panic(mlxp, "failed to delete flow "
				    "entry %u on table %u", i,
				    ft->mlft_num);
			}
		}
	}

	while ((fg = list_remove_head(&ft->mlft_groups)) != NULL) {
		if (fg->mlfg_state & MLXCX_FLOW_GROUP_CREATED &&
		    !(fg->mlfg_state & MLXCX_FLOW_GROUP_DESTROYED)) {
			if (!mlxcx_cmd_destroy_flow_group(mlxp, fg)) {
				mlxcx_panic(mlxp, "failed to destroy flow "
				    "group %u", fg->mlfg_num);
			}
		}
		kmem_free(fg, sizeof (mlxcx_flow_group_t));
	}
	list_destroy(&ft->mlft_groups);
	if (ft->mlft_state & MLXCX_FLOW_TABLE_CREATED &&
	    !(ft->mlft_state & MLXCX_FLOW_TABLE_DESTROYED)) {
		if (!mlxcx_cmd_destroy_flow_table(mlxp, ft)) {
			mlxcx_panic(mlxp, "failed to destroy flow table %u",
			    ft->mlft_num);
		}
	}
	kmem_free(ft->mlft_ent, ft->mlft_entsize);
	ft->mlft_ent = NULL;
	mutex_exit(&ft->mlft_mtx);
	mutex_destroy(&ft->mlft_mtx);
	kmem_free(ft, sizeof (mlxcx_flow_table_t));
}

static void
mlxcx_teardown_ports(mlxcx_t *mlxp)
{
	uint_t i;
	mlxcx_port_t *p;
	mlxcx_flow_table_t *ft;

	for (i = 0; i < mlxp->mlx_nports; ++i) {
		p = &mlxp->mlx_ports[i];

		mutex_enter(&p->mlp_mtx);
		if ((ft = p->mlp_rx_flow) != NULL) {
			mutex_enter(&ft->mlft_mtx);
			mlxcx_teardown_flow_table(mlxp, ft);
		}
		mutex_exit(&p->mlp_mtx);
		mutex_destroy(&p->mlp_mtx);
	}

	kmem_free(mlxp->mlx_ports, mlxp->mlx_ports_size);
	mlxp->mlx_ports = NULL;
}

static void
mlxcx_teardown_wqs(mlxcx_t *mlxp)
{
	mlxcx_work_queue_t *mlwq;

	while ((mlwq = list_head(&mlxp->mlx_wqs)) != NULL) {
		mlxcx_wq_teardown(mlxp, mlwq);
	}
	list_destroy(&mlxp->mlx_wqs);
}

static void
mlxcx_teardown_cqs(mlxcx_t *mlxp)
{
	mlxcx_event_queue_t *mleq;
	mlxcx_completion_queue_t *mlcq;
	mlxcx_buffer_t *b;

	while ((mlcq = list_head(&mlxp->mlx_cqs)) != NULL) {
		mlxcx_cq_teardown(mlxp, mlcq);
	}
	list_destroy(&mlxp->mlx_cqs);
}

static void
mlxcx_teardown_eqs(mlxcx_t *mlxp)
{
	mlxcx_event_queue_t *mleq;
	uint_t i;

	for (i = 0; i < mlxp->mlx_intr_count; ++i) {
		mleq = &mlxp->mlx_eqs[i];
		mutex_enter(&mleq->mleq_mtx);
		if (mleq->mleq_hascheck) {
			ddi_periodic_delete(mleq->mleq_checktimer);
		}
		if ((mleq->mleq_state & MLXCX_EQ_CREATED) &&
		    !(mleq->mleq_state & MLXCX_EQ_DESTROYED)) {
			if (!mlxcx_cmd_destroy_eq(mlxp, mleq)) {
				mlxcx_warn(mlxp, "failed to destroy "
				    "event queue idx %u eqn %u",
				    i, mleq->mleq_num);
			}
		}
		if (mleq->mleq_state & MLXCX_EQ_ALLOC) {
			mlxcx_eq_rele_dma(mlxp, mleq);
		}
		mutex_exit(&mleq->mleq_mtx);
	}
}

static void
mlxcx_teardown(mlxcx_t *mlxp)
{
	uint_t i;
	dev_info_t *dip = mlxp->mlx_dip;

	if (mlxp->mlx_attach & MLXCX_ATTACH_GROUPS) {
		mlxcx_teardown_groups(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_GROUPS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_WQS) {
		mlxcx_teardown_wqs(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_WQS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_CQS) {
		mlxcx_teardown_cqs(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_CQS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_BUFS) {
		mlxcx_teardown_bufs(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_BUFS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_PORTS) {
		mlxcx_teardown_ports(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_PORTS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_INTRS) {
		mlxcx_teardown_eqs(mlxp);
		mlxcx_intr_teardown(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_INTRS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_UAR_PD_TD) {
		if (mlxp->mlx_uar.mlu_allocated) {
			if (!mlxcx_cmd_dealloc_uar(mlxp, &mlxp->mlx_uar)) {
				mlxcx_warn(mlxp, "failed to release UAR");
			}
			for (i = 0; i < MLXCX_BF_PER_UAR; ++i)
				mutex_destroy(&mlxp->mlx_uar.mlu_bf[i].mbf_mtx);
		}
		if (mlxp->mlx_pd.mlpd_allocated &&
		    !mlxcx_cmd_dealloc_pd(mlxp, &mlxp->mlx_pd)) {
			mlxcx_warn(mlxp, "failed to release PD");
		}
		if (mlxp->mlx_tdom.mltd_allocated &&
		    !mlxcx_cmd_dealloc_tdom(mlxp, &mlxp->mlx_tdom)) {
			mlxcx_warn(mlxp, "failed to release TDOM");
		}
		mlxp->mlx_attach &= ~MLXCX_ATTACH_UAR_PD_TD;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_INIT_HCA) {
		if (!mlxcx_cmd_teardown_hca(mlxp)) {
			mlxcx_warn(mlxp, "failed to send teardown HCA "
			    "command during device detach");
		}
		mlxp->mlx_attach &= ~MLXCX_ATTACH_INIT_HCA;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_PAGE_LIST) {
		mlxcx_teardown_pages(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_PAGE_LIST;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_ENABLE_HCA) {
		if (!mlxcx_cmd_disable_hca(mlxp)) {
			mlxcx_warn(mlxp, "failed to send DISABLE HCA command "
			    "during device detach");
		}
		mlxp->mlx_attach &= ~MLXCX_ATTACH_ENABLE_HCA;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_CMD) {
		mlxcx_cmd_queue_fini(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_CMD;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_CAPS) {
		kmem_free(mlxp->mlx_caps, sizeof (mlxcx_caps_t));
		mlxp->mlx_caps = NULL;
		mlxp->mlx_attach &= ~MLXCX_ATTACH_CAPS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_REGS) {
		ddi_regs_map_free(&mlxp->mlx_regs_handle);
		mlxp->mlx_regs_handle = NULL;
		mlxp->mlx_attach &= ~MLXCX_ATTACH_REGS;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_PCI_CONFIG) {
		pci_config_teardown(&mlxp->mlx_cfg_handle);
		mlxp->mlx_cfg_handle = NULL;
		mlxp->mlx_attach &= ~MLXCX_ATTACH_PCI_CONFIG;
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_FM) {
		mlxcx_fm_fini(mlxp);
		mlxp->mlx_attach &= ~MLXCX_ATTACH_FM;
	}

	VERIFY3S(mlxp->mlx_attach, ==, 0);
	kmem_free(mlxp, sizeof (mlxcx_t));
	ddi_set_driver_private(dip, NULL);
}	

static boolean_t
mlxcx_regs_map(mlxcx_t *mlxp)
{
	off_t memsize;
	int ret;
	ddi_device_acc_attr_t da;

	if (ddi_dev_regsize(mlxp->mlx_dip, MLXCX_REG_NUMBER, &memsize) !=
	    DDI_SUCCESS) {
		mlxcx_warn(mlxp, "failed to get register set size");
		return (B_FALSE);
	}

	/*
	 * All data in the main BAR is kept in big-endian even though it's a PCI
	 * device.
	 */
	bzero(&da, sizeof (ddi_device_acc_attr_t));
	da.devacc_attr_version = DDI_DEVICE_ATTR_V0;
	da.devacc_attr_endian_flags = DDI_STRUCTURE_BE_ACC;
	da.devacc_attr_dataorder = DDI_STRICTORDER_ACC;
	if (DDI_FM_ACC_ERR_CAP(mlxp->mlx_fm_caps)) {
		da.devacc_attr_access = DDI_FLAGERR_ACC;
	} else {
		da.devacc_attr_access = DDI_DEFAULT_ACC;
	}

	ret = ddi_regs_map_setup(mlxp->mlx_dip, MLXCX_REG_NUMBER,
	    &mlxp->mlx_regs_base, 0, memsize, &da, &mlxp->mlx_regs_handle);

	if (ret != DDI_SUCCESS) {
		mlxcx_warn(mlxp, "failed to map device registers: %d", ret);
		return (B_FALSE);
	}

	return (B_TRUE);
}

static boolean_t
mlxcx_check_issi(mlxcx_t *mlxp)
{
	uint32_t issi;

	if (!mlxcx_cmd_query_issi(mlxp, &issi)) {
		mlxcx_warn(mlxp, "failed to get ISSI");
		return (B_FALSE);
	}

	if ((issi & (1 << MLXCX_CURRENT_ISSI)) == 0) {
		mlxcx_warn(mlxp, "hardware does not support software ISSI, "
		    "hw vector 0x%x, sw version %u", issi, MLXCX_CURRENT_ISSI);
		return (B_FALSE);
	}

	if (!mlxcx_cmd_set_issi(mlxp, MLXCX_CURRENT_ISSI)) {
		mlxcx_warn(mlxp, "failed to set ISSI to %u",
		    MLXCX_CURRENT_ISSI);
		return (B_FALSE);
	}

	return (B_TRUE);
}

boolean_t
mlxcx_give_pages(mlxcx_t *mlxp, int32_t npages)
{
	ddi_device_acc_attr_t acc;
	ddi_dma_attr_t attr;
	int32_t i;
	list_t plist;
	mlxcx_dev_page_t *mdp;

	/*
	 * If there are no pages required, then we're done here.
	 */
	if (npages <= 0) {
		return (B_TRUE);
	}

	list_create(&plist, sizeof (mlxcx_dev_page_t),
	    offsetof(mlxcx_dev_page_t, mxdp_list));

	for (i = 0; i < npages; i++) {
		mdp = kmem_zalloc(sizeof (mlxcx_dev_page_t), KM_SLEEP);
		mlxcx_dma_acc_attr(mlxp, &acc);
		mlxcx_dma_page_attr(mlxp, &attr);
		if (!mlxcx_dma_alloc(mlxp, &mdp->mxdp_dma, &attr, &acc,
		    B_TRUE, MLXCX_HW_PAGE_SIZE, B_TRUE)) {
			mlxcx_warn(mlxp, "failed to allocate 4k page %u/%u", i,
			    npages);
			goto cleanup_npages;
		}

		list_insert_tail(&plist, mdp);
	}

	/*
	 * Now that all of the pages have been allocated, given them to hardware
	 * in chunks.
	 */
	while (npages > 0) {
		mlxcx_dev_page_t *pages[MLXCX_MANAGE_PAGES_MAX_PAGES];
		int32_t togive = MIN(MLXCX_MANAGE_PAGES_MAX_PAGES, npages);

		for (i = 0; i < togive; i++) {
			pages[i] = list_remove_head(&plist);
		}

		if (!mlxcx_cmd_give_pages(mlxp,
		    MLXCX_MANAGE_PAGES_OPMOD_GIVE_PAGES, togive, pages)) {
			mlxcx_warn(mlxp, "!hardware refused our gift of %u "
			    "pages!", togive);
			for (i = 0; i < togive; i++) {
				list_insert_tail(&plist, pages[i]);
			}
			goto cleanup_npages;
		} 

		mutex_enter(&mlxp->mlx_pagemtx);
		for (i = 0; i < togive; i++) {
			list_insert_tail(&mlxp->mlx_pages, pages[i]);
		}
		mlxp->mlx_npages += togive;
		mutex_exit(&mlxp->mlx_pagemtx);
		npages -= togive;
	}

	list_destroy(&plist);

	return (B_TRUE);

cleanup_npages:
	while ((mdp = list_remove_head(&plist)) != NULL) {
		mlxcx_dma_free(&mdp->mxdp_dma);
		kmem_free(mdp, sizeof (mlxcx_dev_page_t));
	}
	list_destroy(&plist);
	return (B_FALSE);
}

static boolean_t
mlxcx_init_pages(mlxcx_t *mlxp, uint_t type)
{
	int32_t npages;
	boolean_t ret;

	if (!mlxcx_cmd_query_pages(mlxp, type,
	    &npages)) {
		mlxcx_warn(mlxp, "failed to determine boot pages");
		return (B_FALSE);
	}

	return (mlxcx_give_pages(mlxp, npages));
}

static int
mlxcx_bufs_cache_constr(void *arg, void *cookie, int kmflags)
{
	mlxcx_t *mlxp = cookie;
	mlxcx_buffer_t *b = arg;

	bzero(b, sizeof (mlxcx_buffer_t));
	b->mlb_mlx = mlxp;
	b->mlb_state = MLXCX_BUFFER_INIT;

	return (0);
}

static void
mlxcx_bufs_cache_destr(void *arg, void *cookie)
{
	mlxcx_t *mlxp = cookie;
	mlxcx_buffer_t *b = arg;
	VERIFY(b->mlb_state == MLXCX_BUFFER_FREE ||
	    b->mlb_state == MLXCX_BUFFER_INIT);
}

mlxcx_buf_shard_t *
mlxcx_mlbs_create(mlxcx_t *mlxp)
{
	mlxcx_buf_shard_t *s;

	s = kmem_zalloc(sizeof (mlxcx_buf_shard_t), KM_SLEEP);

	mutex_init(&s->mlbs_mtx, "mlxcx_bufs_shard_mtx", MUTEX_DRIVER,
	    DDI_INTR_PRI(mlxp->mlx_intr_pri));
	list_create(&s->mlbs_busy, sizeof (mlxcx_buffer_t),
	    offsetof(mlxcx_buffer_t, mlb_entry));
	list_create(&s->mlbs_free, sizeof (mlxcx_buffer_t),
	    offsetof(mlxcx_buffer_t, mlb_entry));
	cv_init(&s->mlbs_free_nonempty, "mlxcx_bufs_free_cv", CV_DRIVER, NULL);

	list_insert_tail(&mlxp->mlx_buf_shards, s);

	return (s);
}

static boolean_t
mlxcx_setup_bufs(mlxcx_t *mlxp)
{
	uint_t i;
	char namebuf[KSTAT_STRLEN];

	(void) snprintf(namebuf, KSTAT_STRLEN, "mlxcx%d_bufs_cache",
	    ddi_get_instance(mlxp->mlx_dip));
	mlxp->mlx_bufs_cache = kmem_cache_create(namebuf,
	    sizeof (mlxcx_buffer_t), sizeof (uint64_t),
	    mlxcx_bufs_cache_constr, mlxcx_bufs_cache_destr,
	    NULL, mlxp, NULL, 0);

	list_create(&mlxp->mlx_buf_shards, sizeof (mlxcx_buf_shard_t),
	    offsetof(mlxcx_buf_shard_t, mlbs_entry));

	return (B_TRUE);
}

int
mlxcx_dmac_fe_compare(const void *arg0, const void *arg1)
{
	const mlxcx_flow_entry_t *left = arg0;
	const mlxcx_flow_entry_t *right = arg1;
	int bcmpr;
	bcmpr = memcmp(left->mlfe_dmac, right->mlfe_dmac,
	    sizeof (left->mlfe_dmac));
	if (bcmpr < 0)
		return (-1);
	if (bcmpr > 0)
		return (1);
	if (left->mlfe_vid < right->mlfe_vid)
		return (-1);
	if (left->mlfe_vid > right->mlfe_vid)
		return (1);
	return (0);
}

int
mlxcx_grmac_compare(const void *arg0, const void *arg1)
{
	const mlxcx_group_mac_t *left = arg0;
	const mlxcx_group_mac_t *right = arg1;
	int bcmpr;
	bcmpr = memcmp(left->mlgm_mac, right->mlgm_mac,
	    sizeof (left->mlgm_mac));
	if (bcmpr < 0)
		return (-1);
	if (bcmpr > 0)
		return (1);
	return (0);
}

static boolean_t
mlxcx_setup_ports(mlxcx_t *mlxp)
{
	uint_t i, j;
	mlxcx_port_t *p;
	mlxcx_flow_table_t *ft;
	mlxcx_flow_group_t *fg;
	mlxcx_flow_entry_t *fe;
	char speedbuf[128];

	VERIFY3U(mlxp->mlx_nports, >, 0);
	mlxp->mlx_ports_size = mlxp->mlx_nports * sizeof (mlxcx_port_t);
	mlxp->mlx_ports = kmem_zalloc(mlxp->mlx_ports_size, KM_SLEEP);

	for (i = 0; i < mlxp->mlx_nports; ++i) {
		p = &mlxp->mlx_ports[i];
		p->mlp_num = i;
		mutex_init(&p->mlp_mtx, "mlx_port_mtx", MUTEX_DRIVER,
		    DDI_INTR_PRI(mlxp->mlx_intr_pri));
		mutex_enter(&p->mlp_mtx);
		if (!mlxcx_cmd_query_nic_vport_ctx(mlxp, p)) {
			kmem_free(mlxp->mlx_ports, mlxp->mlx_ports_size);
			mlxp->mlx_ports = NULL;
			return (B_FALSE);
		}
		if (!mlxcx_cmd_query_port_mtu(mlxp, p)) {
			kmem_free(mlxp->mlx_ports, mlxp->mlx_ports_size);
			mlxp->mlx_ports = NULL;
			return (B_FALSE);
		}
		if (!mlxcx_cmd_query_port_status(mlxp, p)) {
			kmem_free(mlxp->mlx_ports, mlxp->mlx_ports_size);
			mlxp->mlx_ports = NULL;
			return (B_FALSE);
		}
		if (!mlxcx_cmd_query_port_speed(mlxp, p)) {
			kmem_free(mlxp->mlx_ports, mlxp->mlx_ports_size);
			mlxp->mlx_ports = NULL;
			return (B_FALSE);
		}
		if (!mlxcx_cmd_modify_nic_vport_ctx(mlxp, p,
		    MLXCX_MODIFY_NIC_VPORT_CTX_PROMISC)) {
			kmem_free(mlxp->mlx_ports, mlxp->mlx_ports_size);
			mlxp->mlx_ports = NULL;
			return (B_FALSE);
		}

		speedbuf[0] = 0;
		mlxcx_eth_proto_to_string(p->mlp_oper_proto,
		    speedbuf, sizeof (speedbuf));
		mlxcx_note(mlxp, "found ethernet port %u: max_mtu %u, "
		    "mac %02x:%02x:%02x:%02x:%02x:%02x, status %s/%s, "
		    "speed %s", i, p->mlp_max_mtu,
		    p->mlp_mac_address[0], p->mlp_mac_address[1],
		    p->mlp_mac_address[2], p->mlp_mac_address[3],
		    p->mlp_mac_address[4], p->mlp_mac_address[5],
		    mlxcx_port_status_string(p->mlp_admin_status),
		    mlxcx_port_status_string(p->mlp_oper_status),
		    speedbuf);
		mutex_exit(&p->mlp_mtx);
	}

	for (i = 0; i < mlxp->mlx_nports; ++i) {
		p = &mlxp->mlx_ports[i];
		mutex_enter(&p->mlp_mtx);
		p->mlp_rx_flow = (ft = kmem_zalloc(sizeof (mlxcx_flow_table_t),
		    KM_SLEEP));
		mutex_init(&ft->mlft_mtx, "mlx_rx_flow_mtx", MUTEX_DRIVER,
		    DDI_INTR_PRI(mlxp->mlx_intr_pri));

		mutex_enter(&ft->mlft_mtx);

		ft->mlft_type = MLXCX_FLOW_TABLE_NIC_RX;
		ft->mlft_port = p;
		ft->mlft_entshift = mlxcx_ftbl_root_size_shift;
		if (ft->mlft_entshift > mlxp->mlx_caps->mlc_max_rx_ft_shift)
			ft->mlft_entshift = mlxp->mlx_caps->mlc_max_rx_ft_shift;
		ft->mlft_nents = (1 << ft->mlft_entshift);
		ft->mlft_entsize = ft->mlft_nents * sizeof (mlxcx_flow_entry_t);
		ft->mlft_ent = kmem_zalloc(ft->mlft_entsize, KM_SLEEP);
		list_create(&ft->mlft_groups, sizeof (mlxcx_flow_group_t),
		    offsetof(mlxcx_flow_group_t, mlfg_entry));

		for (j = 0; j < ft->mlft_nents; ++j) {
			ft->mlft_ent[j].mlfe_table = ft;
			ft->mlft_ent[j].mlfe_index = j;
		}

		if (!mlxcx_cmd_create_flow_table(mlxp, ft)) {
			mutex_exit(&ft->mlft_mtx);
			mutex_exit(&p->mlp_mtx);
			goto err;
		}

		if (!mlxcx_cmd_set_flow_table_root(mlxp, ft)) {
			mutex_exit(&ft->mlft_mtx);
			mutex_exit(&p->mlp_mtx);
			goto err;
		}

		/*
		 * We match broadcast at the top of the root flow table, then
		 * all multicast/unicast MACs, then the promisc entry is down
		 * the very bottom.
		 *
		 * This way when promisc is on, that entry simply catches any
		 * remaining traffic that earlier flows haven't matched.
		 */
		fg = kmem_zalloc(sizeof (mlxcx_flow_group_t), KM_SLEEP);
		list_insert_tail(&ft->mlft_groups, fg);
		fg->mlfg_table = ft;
		fg->mlfg_size = 1;
		fg->mlfg_mask |= MLXCX_FLOW_MATCH_DMAC;
		if (!mlxcx_setup_flow_group(mlxp, ft, fg)) {
			mutex_exit(&ft->mlft_mtx);
			mutex_exit(&p->mlp_mtx);
			goto err;
		}
		p->mlp_bcast = fg;
		fe = list_head(&fg->mlfg_entries);
		fe->mlfe_action = MLXCX_FLOW_ACTION_FORWARD;
		memset(fe->mlfe_dmac, 0xff, sizeof (fe->mlfe_dmac));
		fe->mlfe_state |= MLXCX_FLOW_ENTRY_DIRTY;

		fg = kmem_zalloc(sizeof (mlxcx_flow_group_t), KM_SLEEP);
		list_insert_tail(&ft->mlft_groups, fg);
		fg->mlfg_table = ft;
		fg->mlfg_size = ft->mlft_nents - 2;
		fg->mlfg_mask |= MLXCX_FLOW_MATCH_DMAC;
		if (!mlxcx_setup_flow_group(mlxp, ft, fg)) {
			mutex_exit(&ft->mlft_mtx);
			mutex_exit(&p->mlp_mtx);
			goto err;
		}
		p->mlp_umcast = fg;

		fg = kmem_zalloc(sizeof (mlxcx_flow_group_t), KM_SLEEP);
		list_insert_tail(&ft->mlft_groups, fg);
		fg->mlfg_table = ft;
		fg->mlfg_size = 1;
		if (!mlxcx_setup_flow_group(mlxp, ft, fg)) {
			mutex_exit(&ft->mlft_mtx);
			mutex_exit(&p->mlp_mtx);
			goto err;
		}
		p->mlp_promisc = fg;
		fe = list_head(&fg->mlfg_entries);
		fe->mlfe_action = MLXCX_FLOW_ACTION_FORWARD;
		fe->mlfe_state |= MLXCX_FLOW_ENTRY_DIRTY;

		avl_create(&p->mlp_dmac_fe, mlxcx_dmac_fe_compare,
		    sizeof (mlxcx_flow_entry_t), offsetof(mlxcx_flow_entry_t,
		    mlfe_dmac_entry));

		mutex_exit(&ft->mlft_mtx);
		mutex_exit(&p->mlp_mtx);
	}

	return (B_TRUE);

err:
	for (i = 0; i < mlxp->mlx_nports; ++i) {
		p = &mlxp->mlx_ports[i];
		if ((ft = p->mlp_rx_flow) != NULL) {
			mutex_enter(&ft->mlft_mtx);
			mlxcx_teardown_flow_table(mlxp, ft);
		}
	}
	kmem_free(mlxp->mlx_ports, mlxp->mlx_ports_size);
	mlxp->mlx_ports = NULL;
	return (B_FALSE);
}

void
mlxcx_remove_all_vlan_entries(mlxcx_t *mlxp, mlxcx_ring_group_t *g)
{
	mlxcx_flow_table_t *ft = g->mlg_rx_vlan_ft;
	mlxcx_flow_group_t *fg = g->mlg_rx_vlan_fg;
	mlxcx_flow_group_t *dfg = g->mlg_rx_vlan_def_fg;
	mlxcx_flow_entry_t *fe;
	mlxcx_group_vlan_t *v;

	ASSERT(mutex_owned(&g->mlg_mtx));

	mutex_enter(&ft->mlft_mtx);

	if (!list_is_empty(&g->mlg_rx_vlans)) {
		fe = list_head(&dfg->mlfg_entries);
		(void) mlxcx_cmd_set_flow_table_entry(mlxp, fe);
	}

	while ((v = list_remove_head(&g->mlg_rx_vlans)) != NULL) {
		fe = v->mlgv_fe;
		ASSERT3P(fe->mlfe_table, ==, ft);
		ASSERT3P(fe->mlfe_group, ==, fg);
		kmem_free(v, sizeof (mlxcx_group_vlan_t));

		(void) mlxcx_cmd_delete_flow_table_entry(mlxp, fe);
		fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_RESERVED;
	}

	mutex_exit(&ft->mlft_mtx);
}

boolean_t
mlxcx_remove_vlan_entry(mlxcx_t *mlxp, mlxcx_ring_group_t *g,
    boolean_t tagged, uint16_t vid)
{
	mlxcx_flow_table_t *ft = g->mlg_rx_vlan_ft;
	mlxcx_flow_group_t *fg = g->mlg_rx_vlan_fg;
	mlxcx_flow_group_t *dfg = g->mlg_rx_vlan_def_fg;
	mlxcx_flow_entry_t *fe;
	mlxcx_group_vlan_t *v;
	boolean_t found = B_FALSE;

	ASSERT(mutex_owned(&g->mlg_mtx));

	mutex_enter(&ft->mlft_mtx);

	v = list_head(&g->mlg_rx_vlans);
	for (; v != NULL; v = list_next(&g->mlg_rx_vlans, v)) {
		if (v->mlgv_tagged == tagged && v->mlgv_vid == vid) {
			found = B_TRUE;
			break;
		}
	}
	if (!found) {
		mutex_exit(&ft->mlft_mtx);
		return (B_FALSE);
	}

	list_remove(&g->mlg_rx_vlans, v);

	/*
	 * If this is the last VLAN entry, we have to go back to accepting
	 * any VLAN (which means re-enabling the default entry).
	 *
	 * Do this before we remove the flow entry for the last specific
	 * VLAN so that we don't lose any traffic in the transition.
	 */
	if (list_is_empty(&g->mlg_rx_vlans)) {
		fe = list_head(&dfg->mlfg_entries);
		if (!mlxcx_cmd_set_flow_table_entry(mlxp, fe)) {
			list_insert_tail(&g->mlg_rx_vlans, v);
			mutex_exit(&ft->mlft_mtx);
			return (B_FALSE);
		}
	}

	fe = v->mlgv_fe;
	ASSERT(fe->mlfe_state & MLXCX_FLOW_ENTRY_RESERVED);
	ASSERT(fe->mlfe_state & MLXCX_FLOW_ENTRY_CREATED);
	ASSERT3P(fe->mlfe_table, ==, ft);
	ASSERT3P(fe->mlfe_group, ==, fg);

	if (!mlxcx_cmd_delete_flow_table_entry(mlxp, fe)) {
		/* XXX: delete the default flow entry if we set it? */
		list_insert_tail(&g->mlg_rx_vlans, v);
		mutex_exit(&ft->mlft_mtx);
		return (B_FALSE);
	}

	fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_RESERVED;

	kmem_free(v, sizeof (mlxcx_group_vlan_t));
	v = NULL;

	mutex_exit(&ft->mlft_mtx);
	return (B_TRUE);
}

boolean_t
mlxcx_add_vlan_entry(mlxcx_t *mlxp, mlxcx_ring_group_t *g, boolean_t tagged,
    uint16_t vid)
{
	mlxcx_flow_table_t *ft = g->mlg_rx_vlan_ft;
	mlxcx_flow_group_t *fg = g->mlg_rx_vlan_fg;
	mlxcx_flow_group_t *dfg = g->mlg_rx_vlan_def_fg;
	mlxcx_flow_entry_t *fe;
	mlxcx_group_vlan_t *v;
	boolean_t found = B_FALSE;
	boolean_t first = B_FALSE;

	ASSERT(mutex_owned(&g->mlg_mtx));

	mutex_enter(&ft->mlft_mtx);

	v = list_head(&g->mlg_rx_vlans);
	for (; v != NULL; v = list_next(&g->mlg_rx_vlans, v)) {
		if (v->mlgv_tagged == tagged && v->mlgv_vid == vid) {
			mutex_exit(&ft->mlft_mtx);
			return (B_TRUE);
		}
	}
	if (list_is_empty(&g->mlg_rx_vlans))
		first = B_TRUE;

	fe = list_head(&fg->mlfg_entries);
	for (; fe != NULL; fe = list_next(&fg->mlfg_entries, fe)) {
		if (!(fe->mlfe_state & MLXCX_FLOW_ENTRY_RESERVED)) {
			found = B_TRUE;
			break;
		}
	}
	if (!found) {
		mutex_exit(&ft->mlft_mtx);
		return (B_FALSE);
	}

	v = kmem_zalloc(sizeof (mlxcx_group_vlan_t), KM_SLEEP);
	v->mlgv_fe = fe;
	v->mlgv_tagged = tagged;
	v->mlgv_vid = vid;

	fe->mlfe_state |= MLXCX_FLOW_ENTRY_RESERVED;
	fe->mlfe_state |= MLXCX_FLOW_ENTRY_DIRTY;
	fe->mlfe_vid = vid;
	if (tagged) {
		fe->mlfe_vlan_type = MLXCX_VLAN_TYPE_CVLAN;
	} else {
		fe->mlfe_vlan_type = MLXCX_VLAN_TYPE_NONE;
	}

	if (!mlxcx_cmd_set_flow_table_entry(mlxp, fe)) {
		fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_DIRTY;
		fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_RESERVED;
		kmem_free(v, sizeof (mlxcx_group_vlan_t));
		mutex_exit(&ft->mlft_mtx);
		return (B_FALSE);
	}

	list_insert_tail(&g->mlg_rx_vlans, v);

	/*
	 * If the vlan list was empty for this group before adding this one,
	 * then we no longer want the "default" entry to allow all VLANs
	 * through.
	 */
	if (first) {
		fe = list_head(&dfg->mlfg_entries);
		(void) mlxcx_cmd_delete_flow_table_entry(mlxp, fe);
	}

	mutex_exit(&ft->mlft_mtx);
	return (B_TRUE);
}

void
mlxcx_remove_all_umcast_entries(mlxcx_t *mlxp, mlxcx_port_t *port,
    mlxcx_ring_group_t *group)
{
	mlxcx_flow_group_t *fg;
	mlxcx_flow_entry_t *fe;
	mlxcx_flow_table_t *ft = port->mlp_rx_flow;
	mlxcx_group_mac_t *gm, *ngm;
	boolean_t found = B_FALSE;

	ASSERT(mutex_owned(&port->mlp_mtx));
	ASSERT(mutex_owned(&group->mlg_mtx));

	mutex_enter(&ft->mlft_mtx);

	gm = avl_first(&group->mlg_rx_macs);
	for (; gm != NULL; gm = ngm) {
		ngm = AVL_NEXT(&group->mlg_rx_macs, gm);

		ASSERT3P(gm->mlgm_group, ==, group);
		fe = gm->mlgm_fe;
		ASSERT3P(fe->mlfe_table, ==, ft);

		avl_remove(&group->mlg_rx_macs, gm);
		list_remove(&fe->mlfe_ring_groups, gm);
		kmem_free(gm, sizeof (mlxcx_group_mac_t));

		fe->mlfe_ndest = 0;
		gm = list_head(&fe->mlfe_ring_groups);
		for (; gm != NULL; gm = list_next(&fe->mlfe_ring_groups, gm)) {
			fe->mlfe_dest[fe->mlfe_ndest++].mlfed_flow =
			    gm->mlgm_group->mlg_rx_vlan_ft;
		}
		fe->mlfe_state |= MLXCX_FLOW_ENTRY_DIRTY;

		if (fe->mlfe_ndest > 0) {
			(void) mlxcx_cmd_set_flow_table_entry(mlxp, fe);
			continue;
		}

		avl_remove(&port->mlp_dmac_fe, fe);
		(void) mlxcx_cmd_delete_flow_table_entry(mlxp, fe);
		list_destroy(&fe->mlfe_ring_groups);
		fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_RESERVED;
	}

	mutex_exit(&ft->mlft_mtx);
}

boolean_t
mlxcx_remove_umcast_entry(mlxcx_t *mlxp, mlxcx_port_t *port,
    mlxcx_ring_group_t *group, const uint8_t *macaddr)
{
	mlxcx_flow_group_t *fg;
	mlxcx_flow_entry_t *fe;
	mlxcx_flow_table_t *ft = port->mlp_rx_flow;
	mlxcx_group_mac_t *gm, probe;
	boolean_t found = B_FALSE;

	ASSERT(mutex_owned(&port->mlp_mtx));
	ASSERT(mutex_owned(&group->mlg_mtx));

	bzero(&probe, sizeof (probe));
	bcopy(macaddr, probe.mlgm_mac, sizeof (probe.mlgm_mac));

	mutex_enter(&ft->mlft_mtx);

	gm = avl_find(&group->mlg_rx_macs, &probe, NULL);
	if (gm == NULL) {
		mutex_exit(&ft->mlft_mtx);
		return (B_FALSE);
	}
	ASSERT3P(gm->mlgm_group, ==, group);
	ASSERT0(bcmp(macaddr, gm->mlgm_mac, sizeof (gm->mlgm_mac)));

	fe = gm->mlgm_fe;
	ASSERT3P(fe->mlfe_table, ==, ft);
	ASSERT0(bcmp(macaddr, fe->mlfe_dmac, sizeof (fe->mlfe_dmac)));

	list_remove(&fe->mlfe_ring_groups, gm);
	avl_remove(&group->mlg_rx_macs, gm);
	kmem_free(gm, sizeof (mlxcx_group_mac_t));

	fe->mlfe_ndest = 0;
	gm = list_head(&fe->mlfe_ring_groups);
	for (; gm != NULL; gm = list_next(&fe->mlfe_ring_groups, gm)) {
		fe->mlfe_dest[fe->mlfe_ndest++].mlfed_flow =
		    gm->mlgm_group->mlg_rx_vlan_ft;
	}
	fe->mlfe_state |= MLXCX_FLOW_ENTRY_DIRTY;

	if (fe->mlfe_ndest > 0) {
		if (!mlxcx_cmd_set_flow_table_entry(mlxp, fe)) {
			mutex_exit(&ft->mlft_mtx);
			return (B_FALSE);
		}
		return (B_TRUE);
	}

	avl_remove(&port->mlp_dmac_fe, fe);
	(void) mlxcx_cmd_delete_flow_table_entry(mlxp, fe);
	list_destroy(&fe->mlfe_ring_groups);

	fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_RESERVED;

	mutex_exit(&ft->mlft_mtx);

	return (B_TRUE);
}

boolean_t
mlxcx_add_umcast_entry(mlxcx_t *mlxp, mlxcx_port_t *port,
    mlxcx_ring_group_t *group, const uint8_t *macaddr)
{
	mlxcx_flow_group_t *fg;
	mlxcx_flow_entry_t *fe, probe;
	mlxcx_flow_table_t *ft = port->mlp_rx_flow;
	mlxcx_group_mac_t *gm;
	boolean_t found = B_FALSE;

	ASSERT(mutex_owned(&port->mlp_mtx));
	ASSERT(mutex_owned(&group->mlg_mtx));

	bzero(&probe, sizeof (probe));
	bcopy(macaddr, probe.mlfe_dmac, sizeof (probe.mlfe_dmac));

	mutex_enter(&ft->mlft_mtx);

	fe = avl_find(&port->mlp_dmac_fe, &probe, NULL);

	if (fe != NULL)
		goto addmac;

	fg = port->mlp_umcast;
	fe = list_head(&fg->mlfg_entries);
	for (; fe != NULL; fe = list_next(&fg->mlfg_entries, fe)) {
		if (!(fe->mlfe_state & MLXCX_FLOW_ENTRY_RESERVED)) {
			found = B_TRUE;
			break;
		}
	}
	if (!found) {
		mutex_exit(&ft->mlft_mtx);
		return (B_FALSE);
	}
	list_create(&fe->mlfe_ring_groups, sizeof (mlxcx_group_mac_t),
	    offsetof(mlxcx_group_mac_t, mlgm_fe_entry));
	fe->mlfe_state |= MLXCX_FLOW_ENTRY_RESERVED;
	fe->mlfe_action = MLXCX_FLOW_ACTION_FORWARD;
	bcopy(macaddr, fe->mlfe_dmac, sizeof (fe->mlfe_dmac));

	avl_add(&port->mlp_dmac_fe, fe);

addmac:
	fe->mlfe_dest[fe->mlfe_ndest++].mlfed_flow = group->mlg_rx_vlan_ft;
	fe->mlfe_state |= MLXCX_FLOW_ENTRY_DIRTY;

	if (!mlxcx_cmd_set_flow_table_entry(mlxp, fe)) {
		fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_DIRTY;
		if (--fe->mlfe_ndest == 0) {
			fe->mlfe_state &= ~MLXCX_FLOW_ENTRY_RESERVED;
		}
		mutex_exit(&ft->mlft_mtx);
		return (B_FALSE);
	}

	gm = kmem_zalloc(sizeof (mlxcx_group_mac_t), KM_SLEEP);
	gm->mlgm_group = group;
	gm->mlgm_fe = fe;
	bcopy(macaddr, gm->mlgm_mac, sizeof (gm->mlgm_mac));
	avl_add(&group->mlg_rx_macs, gm);
	list_insert_tail(&fe->mlfe_ring_groups, gm);

	mutex_exit(&ft->mlft_mtx);

	return (B_TRUE);
}

boolean_t
mlxcx_setup_flow_group(mlxcx_t *mlxp, mlxcx_flow_table_t *ft,
    mlxcx_flow_group_t *fg)
{
	mlxcx_flow_entry_t *fe;
	uint_t i, startidx, idx;

	ASSERT(mutex_owned(&ft->mlft_mtx));
	ASSERT(ft->mlft_state & MLXCX_FLOW_TABLE_CREATED);
	ASSERT3P(fg->mlfg_table, ==, ft);

	if (ft->mlft_next_ent + fg->mlfg_size > ft->mlft_nents)
		return (B_FALSE);
	fg->mlfg_start_idx = ft->mlft_next_ent;

	if (!mlxcx_cmd_create_flow_group(mlxp, fg)) {
		return (B_FALSE);
	}

	list_create(&fg->mlfg_entries, sizeof (mlxcx_flow_entry_t),
	    offsetof(mlxcx_flow_entry_t, mlfe_group_entry));
	for (i = 0; i < fg->mlfg_size; ++i) {
		idx = fg->mlfg_start_idx + i;
		fe = &ft->mlft_ent[idx];
		fe->mlfe_group = fg;
		list_insert_tail(&fg->mlfg_entries, fe);
	}
	fg->mlfg_avail = fg->mlfg_size;
	ft->mlft_next_ent += fg->mlfg_size;

	return (B_TRUE);
}

static void
mlxcx_eq_check(void *arg)
{
	mlxcx_event_queue_t *eq = arg;
	mlxcx_t *mlxp = eq->mleq_mlx;

	mutex_enter(&eq->mleq_mtx);
	(void) mlxcx_cmd_query_eq(mlxp, eq);
	mutex_exit(&eq->mleq_mtx);
}

static boolean_t
mlxcx_setup_eq0(mlxcx_t *mlxp)
{
	mlxcx_event_queue_t *mleq = &mlxp->mlx_eqs[0];

	mutex_enter(&mleq->mleq_mtx);
	if (!mlxcx_eq_alloc_dma(mlxp, mleq)) {
		/* mlxcx_teardown_eqs() will clean this up */
		mutex_exit(&mleq->mleq_mtx);
		return (B_FALSE);
	}
	mleq->mleq_mlx = mlxp;
	mleq->mleq_uar = &mlxp->mlx_uar;
	mleq->mleq_events =
	    (1 << MLXCX_EVENT_PAGE_REQUEST) |
	    (1 << MLXCX_EVENT_PORT_STATE) |
	    (1 << MLXCX_EVENT_INTERNAL_ERROR) |
	    (1 << MLXCX_EVENT_PORT_MODULE) |
	    (1 << MLXCX_EVENT_SENDQ_DRAIN) |
	    (1 << MLXCX_EVENT_LAST_WQE) |
	    (1 << MLXCX_EVENT_CQ_ERROR) |
	    (1 << MLXCX_EVENT_WQ_CATASTROPHE) |
	    (1 << MLXCX_EVENT_PAGE_FAULT) |
	    (1 << MLXCX_EVENT_WQ_INVALID_REQ) |
	    (1 << MLXCX_EVENT_WQ_ACCESS_VIOL) |
	    (1 << MLXCX_EVENT_NIC_VPORT) |
	    (1 << MLXCX_EVENT_DOORBELL_CONGEST);
	if (!mlxcx_cmd_create_eq(mlxp, mleq)) {
		/* mlxcx_teardown_eqs() will clean this up */
		mutex_exit(&mleq->mleq_mtx);
		return (B_FALSE);
	}
	if (ddi_intr_enable(mlxp->mlx_intr_handles[0]) != DDI_SUCCESS) {
		/*
		 * mlxcx_teardown_eqs() will handle calling cmd_destroy_eq and
		 * eq_rele_dma
		 */
		mutex_exit(&mleq->mleq_mtx);
		return (B_FALSE);
	}
	mleq->mleq_hascheck = B_TRUE;
	mleq->mleq_checktimer = ddi_periodic_add(mlxcx_eq_check, mleq,
	    MLXCX_CQ_CHECK_INTERVAL_NSEC, mlxp->mlx_intr_pri);
	mlxcx_arm_eq(mlxp, mleq);
	mutex_exit(&mleq->mleq_mtx);
	return (B_TRUE);
}

int
mlxcx_cq_compare(const void *arg0, const void *arg1)
{
	const mlxcx_completion_queue_t *left = arg0;
	const mlxcx_completion_queue_t *right = arg1;
	if (left->mlcq_num < right->mlcq_num) {
		return (-1);
	}
	if (left->mlcq_num > right->mlcq_num) {
		return (1);
	}
	return (0);
}

static boolean_t
mlxcx_setup_eqs(mlxcx_t *mlxp)
{
	uint_t i;
	mlxcx_event_queue_t *mleq;

	for (i = 1; i < mlxp->mlx_intr_count; ++i) {
		mleq = &mlxp->mlx_eqs[i];
		mutex_enter(&mleq->mleq_mtx);
		if (!mlxcx_eq_alloc_dma(mlxp, mleq)) {
			mutex_exit(&mleq->mleq_mtx);
			return (B_FALSE);
		}
		mleq->mleq_uar = &mlxp->mlx_uar;
		if (!mlxcx_cmd_create_eq(mlxp, mleq)) {
			/* mlxcx_teardown() will handle calling eq_rele_dma */
			mutex_exit(&mleq->mleq_mtx);
			return (B_FALSE);
		}
		if (mlxcx_intrmod_period_usec != 0 && !mlxcx_cmd_set_int_mod(
		    mlxp, i, mlxcx_intrmod_period_usec)) {
			mutex_exit(&mleq->mleq_mtx);
			return (B_FALSE);
		}
		if (ddi_intr_enable(mlxp->mlx_intr_handles[i]) != DDI_SUCCESS) {
			mutex_exit(&mleq->mleq_mtx);
			return (B_FALSE);
		}
		mlxcx_arm_eq(mlxp, mleq);
		mutex_exit(&mleq->mleq_mtx);
	}

	mlxp->mlx_next_eq = 1;

	return (B_TRUE);
}

/*
 * Snapshot all of the hardware capabilities that we care about and then modify
 * the HCA capabilities to get things moving..
 */
static boolean_t
mlxcx_init_caps(mlxcx_t *mlxp)
{
	mlxcx_caps_t *c;

	mlxp->mlx_caps = (c = kmem_zalloc(sizeof (mlxcx_caps_t), KM_SLEEP));

	if (!mlxcx_cmd_query_hca_cap(mlxp, MLXCX_HCA_CAP_GENERAL,
	    MLXCX_HCA_CAP_MODE_CURRENT, &c->mlc_hca_cur)) {
		mlxcx_warn(mlxp, "failed to obtain current HCA general caps");
	}

	if (!mlxcx_cmd_query_hca_cap(mlxp, MLXCX_HCA_CAP_GENERAL,
	    MLXCX_HCA_CAP_MODE_MAX, &c->mlc_hca_max)) {
		mlxcx_warn(mlxp, "failed to obtain maximum HCA general caps");
	}

	if (!mlxcx_cmd_query_hca_cap(mlxp, MLXCX_HCA_CAP_ETHERNET,
	    MLXCX_HCA_CAP_MODE_CURRENT, &c->mlc_ether_cur)) {
		mlxcx_warn(mlxp, "failed to obtain current HCA general caps");
	}

	if (!mlxcx_cmd_query_hca_cap(mlxp, MLXCX_HCA_CAP_ETHERNET,
	    MLXCX_HCA_CAP_MODE_MAX, &c->mlc_ether_max)) {
		mlxcx_warn(mlxp, "failed to obtain maximum HCA general caps");
	}

	if (!mlxcx_cmd_query_hca_cap(mlxp, MLXCX_HCA_CAP_NIC_FLOW,
	    MLXCX_HCA_CAP_MODE_CURRENT, &c->mlc_nic_flow_cur)) {
		mlxcx_warn(mlxp, "failed to obtain current HCA general caps");
	}

	if (!mlxcx_cmd_query_hca_cap(mlxp, MLXCX_HCA_CAP_NIC_FLOW,
	    MLXCX_HCA_CAP_MODE_MAX, &c->mlc_nic_flow_max)) {
		mlxcx_warn(mlxp, "failed to obtain maximum HCA general caps");
	}
	/*
	 * XXX It's not clear which caps we should or shouldn't keep for the
	 * moment, so we'll just use the default to get a bit further in the
	 * driver, though we'll really want to change a few of these.
	 */

	/*
	 * Check the caps meet our requirements.
	 */
	const mlxcx_hca_cap_general_caps_t *gen = &c->mlc_hca_cur.mhc_general;

	if (gen->mlcap_general_log_pg_sz != 12) {
		mlxcx_warn(mlxp, "!hardware has page size != 4k");
		goto err;
	}
	if (gen->mlcap_general_cqe_version != 1) {
		mlxcx_warn(mlxp, "!hardware does not support CQE v1");
		goto err;
	}
	if (gen->mlcap_general_port_type !=
	    MLXCX_CAP_GENERAL_PORT_TYPE_ETHERNET) {
		mlxcx_warn(mlxp, "!hardware has non-ethernet ports");
		goto err;
	}
	mlxp->mlx_nports = gen->mlcap_general_num_ports;
	mlxp->mlx_max_sdu = (1 << (gen->mlcap_general_log_max_msg & 0x1F));

	c->mlc_max_tir = (1 << gen->mlcap_general_log_max_tir);

	c->mlc_checksum = get_bit32(c->mlc_ether_cur.mhc_eth.mlcap_eth_flags,
	    MLXCX_ETH_CAP_CSUM_CAP);
	c->mlc_vxlan = get_bit32(c->mlc_ether_cur.mhc_eth.mlcap_eth_flags,
	    MLXCX_ETH_CAP_TUNNEL_STATELESS_VXLAN);

	c->mlc_max_lso_size = (1 << get_bits32(c->mlc_ether_cur.mhc_eth.
	    mlcap_eth_flags, MLXCX_ETH_CAP_MAX_LSO_CAP));
	if (c->mlc_max_lso_size == 1) {
		c->mlc_max_lso_size = 0;
		c->mlc_lso = B_FALSE;
	} else {
		c->mlc_lso = B_TRUE;
	}

	c->mlc_max_rqt_size = (1 << get_bits32(c->mlc_ether_cur.mhc_eth.
	    mlcap_eth_flags, MLXCX_ETH_CAP_RSS_IND_TBL_CAP));

	if (!get_bit32(c->mlc_nic_flow_cur.mhc_flow.mlcap_flow_nic_rx.
	    mlcap_flow_prop_flags, MLXCX_FLOW_CAP_PROPS_SUPPORT)) {
		mlxcx_warn(mlxp, "!hardware does not support rx flow tables");
		goto err;
	}
	if (!get_bit32(c->mlc_nic_flow_cur.mhc_flow.mlcap_flow_nic_rx.
	    mlcap_flow_prop_flags, MLXCX_FLOW_CAP_PROPS_MODIFY)) {
		mlxcx_warn(mlxp, "!hardware does not support modifying rx "
		    "flow table entries");
		goto err;
	}

	c->mlc_max_rx_ft_shift = c->mlc_nic_flow_cur.mhc_flow.mlcap_flow_nic_rx.
	    mlcap_flow_prop_log_max_ft_size;
	c->mlc_max_rx_flows = (1 << c->mlc_nic_flow_cur.mhc_flow.
	    mlcap_flow_nic_rx.mlcap_flow_prop_log_max_flow);
	c->mlc_max_rx_fe_dest = (1 << c->mlc_nic_flow_cur.mhc_flow.
	    mlcap_flow_nic_rx.mlcap_flow_prop_log_max_destination);

	return (B_TRUE);

err:
	kmem_free(mlxp->mlx_caps, sizeof (mlxcx_caps_t));
	return (B_FALSE);
}

static int
mlxcx_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	mlxcx_t *mlxp;

	if (cmd != DDI_DETACH)
		return (DDI_FAILURE);

	mlxp = ddi_get_driver_private(dip);
	if (mlxp == NULL) {
		mlxcx_warn(NULL, "asked to detach, but missing instance private "
		    "data");
		return (DDI_FAILURE);
	}

	if (mlxp->mlx_attach & MLXCX_ATTACH_MAC_HDL) {
		if (mac_unregister(mlxp->mlx_mac_hdl) != DDI_SUCCESS) {
			return (DDI_FAILURE);
		}
		mlxp->mlx_attach &= ~MLXCX_ATTACH_MAC_HDL;
	}

	mlxcx_teardown(mlxp);
	mlxcx_note(NULL, "detached driver (%p)", mlxp);
	return (DDI_SUCCESS);
}

static size_t
mlxcx_calc_rx_ngroups(mlxcx_t *mlxp)
{
	size_t ngroups = mlxcx_rx_ngroups_large + mlxcx_rx_ngroups_small;
	size_t tirlim, flowlim, gflowlim;

	tirlim = mlxp->mlx_caps->mlc_max_tir / MLXCX_TIRS_PER_GROUP;
	if (tirlim < ngroups) {
		mlxcx_note(mlxp, "limiting number of rx groups to %u based "
		    "on number of TIRs available", tirlim);
		ngroups = tirlim;
	}

	flowlim = (1 << mlxp->mlx_caps->mlc_max_rx_ft_shift) - 2;
	if (flowlim < ngroups) {
		mlxcx_note(mlxp, "limiting number of rx groups to %u based "
		    "on max size of RX flow tables", flowlim);
		ngroups = flowlim;
	}

	do {
		gflowlim = mlxp->mlx_caps->mlc_max_rx_flows - 16 * ngroups - 2;
		if (gflowlim < ngroups) {
			mlxcx_note(mlxp, "limiting number of rx groups to %u "
			    "based on max total RX flows", gflowlim);
			--ngroups;
		}
	} while (gflowlim < ngroups);

	return (ngroups);
}

static int
mlxcx_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	mlxcx_t *mlxp;
	uint_t i;

	if (cmd != DDI_ATTACH)
		return (DDI_FAILURE);

	mlxp = kmem_zalloc(sizeof (mlxcx_t), KM_SLEEP);
	mlxp->mlx_dip = dip;
	mlxp->mlx_inst = ddi_get_instance(dip);
	ddi_set_driver_private(dip, mlxp);

	mlxcx_load_props(mlxp);

	mlxcx_fm_init(mlxp);
	mlxp->mlx_attach |= MLXCX_ATTACH_FM;

	if (pci_config_setup(mlxp->mlx_dip, &mlxp->mlx_cfg_handle) !=
	    DDI_SUCCESS) {
		mlxcx_warn(mlxp, "failed to initial PCI config space");
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_PCI_CONFIG;

	if (!mlxcx_regs_map(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_REGS;

	if (!mlxcx_cmd_queue_init(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_CMD;

	if (!mlxcx_cmd_enable_hca(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_ENABLE_HCA;

	if (!mlxcx_check_issi(mlxp)) {
		goto err;
	}

	/*
	 * We have to get our interrupts now so we know what priority to
	 * create pagemtx with.
	 */
	if (!mlxcx_intr_setup(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_INTRS;

	mutex_init(&mlxp->mlx_pagemtx, "mlx_pagemtx", MUTEX_DRIVER,
	    DDI_INTR_PRI(mlxp->mlx_intr_pri));
	list_create(&mlxp->mlx_pages, sizeof (mlxcx_dev_page_t),
	    offsetof(mlxcx_dev_page_t, mxdp_list));
	mlxp->mlx_attach |= MLXCX_ATTACH_PAGE_LIST;

	if (!mlxcx_init_pages(mlxp, MLXCX_QUERY_PAGES_OPMOD_BOOT)) {
		goto err;
	}

	if (!mlxcx_init_caps(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_CAPS;

	if (!mlxcx_init_pages(mlxp, MLXCX_QUERY_PAGES_OPMOD_INIT)) {
		goto err;
	}

	if (!mlxcx_cmd_init_hca(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_INIT_HCA;

	if (!mlxcx_cmd_set_driver_version(mlxp, MLXCX_DRIVER_VERSION)) {
		goto err;
	}

	/*
	 * The User Access Region (UAR) is needed so we can ring EQ and CQ
	 * doorbells.
	 */
	if (!mlxcx_cmd_alloc_uar(mlxp, &mlxp->mlx_uar)) {
		goto err;
	}
	for (i = 0; i < MLXCX_BF_PER_UAR; ++i) {
		mutex_init(&mlxp->mlx_uar.mlu_bf[i].mbf_mtx, "mlx_uar_bf_mtx",
		    MUTEX_DRIVER, DDI_INTR_PRI(mlxp->mlx_intr_pri));
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_UAR_PD_TD;

	/*
	 * Set up event queue #0 -- it's special and only handles control
	 * type events, like PAGE_REQUEST (which we will probably get during
	 * the commands below).
	 *
	 * This will enable and arm the interrupt on EQ 0, too.
	 */
	if (!mlxcx_setup_eq0(mlxp)) {
		goto err;
	}

	/*
	 * Allocate a protection and transport domain. These don't really do
	 * anything for us (they're IB concepts), but we need to give their
	 * ID numbers in other commands.
	 */
	if (!mlxcx_cmd_alloc_pd(mlxp, &mlxp->mlx_pd)) {
		goto err;
	}
	if (!mlxcx_cmd_alloc_tdom(mlxp, &mlxp->mlx_tdom)) {
		goto err;
	}
	/*
	 * Fetch the "reserved" lkey that lets us give linear addresses in
	 * work queue entries, rather than having to mess with the NIC's
	 * internal MMU.
	 */
	if (!mlxcx_cmd_query_special_ctxs(mlxp)) {
		goto err;
	}

	/*
	 * Query our port information and current state, populate the
	 * mlxcx_port_t structs.
	 *
	 * This also sets up the root flow tables and flow groups.
	 */
	if (!mlxcx_setup_ports(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_PORTS;

	/*
	 * Set up, enable and arm the rest of the interrupt EQs which will
	 * service events from CQs.
	 *
	 * The MLXCX_ATTACH_INTRS flag covers checking if these need to be
	 * cleaned up.
	 */
	if (!mlxcx_setup_eqs(mlxp)) {
		goto err;
	}

	/* Completion queues */
	list_create(&mlxp->mlx_cqs, sizeof (mlxcx_completion_queue_t),
	    offsetof(mlxcx_completion_queue_t, mlcq_entry));
	mlxp->mlx_attach |= MLXCX_ATTACH_CQS;

	/* Work queues (send queues, receive queues) */
	list_create(&mlxp->mlx_wqs, sizeof (mlxcx_work_queue_t),
	    offsetof(mlxcx_work_queue_t, mlwq_entry));
	mlxp->mlx_attach |= MLXCX_ATTACH_WQS;

	/*
	 * Construct our arrays of mlxcx_ring_group_ts, which represent the
	 * "groups" we advertise to MAC.
	 */
	mlxp->mlx_rx_ngroups = mlxcx_calc_rx_ngroups(mlxp);
	mlxp->mlx_rx_groups_size = mlxp->mlx_rx_ngroups *
	    sizeof (mlxcx_ring_group_t);
	mlxp->mlx_rx_groups = kmem_zalloc(mlxp->mlx_rx_groups_size, KM_SLEEP);

	mlxp->mlx_tx_ngroups = mlxcx_tx_ngroups;
	mlxp->mlx_tx_groups_size = mlxp->mlx_tx_ngroups *
	    sizeof (mlxcx_ring_group_t);
	mlxp->mlx_tx_groups = kmem_zalloc(mlxp->mlx_tx_groups_size, KM_SLEEP);

	mlxp->mlx_attach |= MLXCX_ATTACH_GROUPS;

	/*
	 * Sets up the free/busy buffers list for keeping track of packet
	 * buffers.
	 */
	if (!mlxcx_setup_bufs(mlxp))
		goto err;
	mlxp->mlx_attach |= MLXCX_ATTACH_BUFS;

	/*
	 * Before we tell MAC about our rings/groups, we need to do enough
	 * setup on them to be sure about the numbers and configuration that
	 * we have. This will do basically everything short of allocating
	 * packet buffers and starting the rings up.
	 */
	for (i = 0; i < mlxp->mlx_tx_ngroups; ++i) {
		if (!mlxcx_tx_group_setup(mlxp, &mlxp->mlx_tx_groups[i]))
			goto err;
	}
	for (i = 0; i < mlxp->mlx_rx_ngroups; ++i) {
		if (!mlxcx_rx_group_setup(mlxp, &mlxp->mlx_rx_groups[i]))
			goto err;
	}

	/*
	 * Finally, tell MAC that we exist!
	 */
	if (!mlxcx_register_mac(mlxp)) {
		goto err;
	}
	mlxp->mlx_attach |= MLXCX_ATTACH_MAC_HDL;

	mlxcx_note(mlxp, "attached driver ok (%p)", mlxp);

	return (DDI_SUCCESS);

err:
	mlxcx_teardown(mlxp);
	return (DDI_FAILURE);
}

static struct cb_ops mlxcx_cb_ops = {
	.cb_open = nulldev,
	.cb_close = nulldev,
	.cb_strategy = nodev,
	.cb_print = nodev,
	.cb_dump = nodev,
	.cb_read = nodev,
	.cb_write = nodev,
	.cb_ioctl = nodev,
	.cb_devmap = nodev,
	.cb_mmap = nodev,
	.cb_segmap = nodev,
	.cb_chpoll = nochpoll,
	.cb_prop_op = ddi_prop_op,
	.cb_flag = D_MP,
	.cb_rev = CB_REV,
	.cb_aread = nodev,
	.cb_awrite = nodev
};

static struct dev_ops mlxcx_dev_ops = {
	.devo_rev = DEVO_REV,
	.devo_refcnt = 0,
	.devo_getinfo = NULL,
	.devo_identify = nulldev,
	.devo_probe = nulldev,
	.devo_attach = mlxcx_attach,
	.devo_detach = mlxcx_detach,
	.devo_reset = nodev,
	.devo_power = ddi_power,
	.devo_quiesce = ddi_quiesce_not_supported,
	.devo_cb_ops = &mlxcx_cb_ops
};

static struct modldrv mlxcx_modldrv = {
	.drv_modops = &mod_driverops,
	.drv_linkinfo = "Mellanox Connect-X 4/5/6",
	.drv_dev_ops = &mlxcx_dev_ops
};

static struct modlinkage mlxcx_modlinkage = {
	.ml_rev = MODREV_1,
	.ml_linkage = { &mlxcx_modldrv, NULL }
};

int
_init(void)
{
	int ret;

	mac_init_ops(&mlxcx_dev_ops, MLXCX_MODULE_NAME);

	if ((ret = mod_install(&mlxcx_modlinkage)) != DDI_SUCCESS) {
		mac_fini_ops(&mlxcx_dev_ops);
		return (ret);
	}

	return (DDI_SUCCESS);
}

int
_info(struct modinfo *modinfop)
{
	return (mod_info(&mlxcx_modlinkage, modinfop));
}

int
_fini(void)
{
	int ret;

	if ((ret = mod_remove(&mlxcx_modlinkage)) != DDI_SUCCESS) {
		return (ret);
	}

	mac_fini_ops(&mlxcx_dev_ops);
	return (DDI_SUCCESS);
}
