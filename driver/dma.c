// SPDX-License-Identifier: GPL-2.0-or-later
/*

  Broadcom B43 wireless driver

  DMA ringbuffer and descriptor allocation/management

  Copyright (c) 2005, 2006 Michael Buesch <m@bues.ch>

  Some code in this file is derived from the b44.c driver
  Copyright (C) 2002 David S. Miller
  Copyright (C) Pekka Pietikainen


*/

#include "b43.h"
#include "dma.h"
#include "main.h"
#include "debugfs.h"
#include "xmit.h"

#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/slab.h>
#include <asm/div64.h>


/* Required number of TX DMA slots per TX frame.
 * This currently is 2, because we put the header and the ieee80211 frame
 * into separate slots. */
#define TX_SLOTS_PER_FRAME	2

static u32 b43_dma_address(struct b43_dma *dma, dma_addr_t dmaaddr,
			   enum b43_addrtype addrtype)
{
	u32 addr;

	switch (addrtype) {
	case B43_DMA_ADDR_LOW:
		addr = lower_32_bits(dmaaddr);
		if (dma->translation_in_low) {
			addr &= ~SSB_DMA_TRANSLATION_MASK;
			addr |= dma->translation;
		}
		break;
	case B43_DMA_ADDR_HIGH:
		addr = upper_32_bits(dmaaddr);
		if (!dma->translation_in_low) {
			addr &= ~SSB_DMA_TRANSLATION_MASK;
			addr |= dma->translation;
		}
		break;
	case B43_DMA_ADDR_EXT:
		if (dma->translation_in_low)
			addr = lower_32_bits(dmaaddr);
		else
			addr = upper_32_bits(dmaaddr);
		addr &= SSB_DMA_TRANSLATION_MASK;
		addr >>= SSB_DMA_TRANSLATION_SHIFT;
		break;
	}

	return addr;
}

/* 32bit DMA ops. */
static
struct b43_dmadesc_generic *op32_idx2desc(struct b43_dmaring *ring,
					  int slot,
					  struct b43_dmadesc_meta **meta)
{
	struct b43_dmadesc32 *desc;

	*meta = &(ring->meta[slot]);
	desc = ring->descbase;
	desc = &(desc[slot]);

	return (struct b43_dmadesc_generic *)desc;
}

static void op32_fill_descriptor(struct b43_dmaring *ring,
				 struct b43_dmadesc_generic *desc,
				 dma_addr_t dmaaddr, u16 bufsize,
				 int start, int end, int irq)
{
	struct b43_dmadesc32 *descbase = ring->descbase;
	int slot;
	u32 ctl;
	u32 addr;
	u32 addrext;

	slot = (int)(&(desc->dma32) - descbase);
	B43_WARN_ON(!(slot >= 0 && slot < ring->nr_slots));

	addr = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_LOW);
	addrext = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_EXT);

	ctl = bufsize & B43_DMA32_DCTL_BYTECNT;
	if (slot == ring->nr_slots - 1)
		ctl |= B43_DMA32_DCTL_DTABLEEND;
	if (start)
		ctl |= B43_DMA32_DCTL_FRAMESTART;
	if (end)
		ctl |= B43_DMA32_DCTL_FRAMEEND;
	if (irq)
		ctl |= B43_DMA32_DCTL_IRQ;
	ctl |= (addrext << B43_DMA32_DCTL_ADDREXT_SHIFT)
	    & B43_DMA32_DCTL_ADDREXT_MASK;

	desc->dma32.control = cpu_to_le32(ctl);
	desc->dma32.address = cpu_to_le32(addr);
}

static void op32_poke_tx(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA32_TXINDEX,
		      (u32) (slot * sizeof(struct b43_dmadesc32)));
}

static void op32_tx_suspend(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA32_TXCTL, b43_dma_read(ring, B43_DMA32_TXCTL)
		      | B43_DMA32_TXSUSPEND);
}

static void op32_tx_resume(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA32_TXCTL, b43_dma_read(ring, B43_DMA32_TXCTL)
		      & ~B43_DMA32_TXSUSPEND);
}

static int op32_get_current_rxslot(struct b43_dmaring *ring)
{
	u32 val;

	val = b43_dma_read(ring, B43_DMA32_RXSTATUS);
	val &= B43_DMA32_RXDPTR;

	return (val / sizeof(struct b43_dmadesc32));
}

static void op32_set_current_rxslot(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA32_RXINDEX,
		      (u32) (slot * sizeof(struct b43_dmadesc32)));
}

static const struct b43_dma_ops dma32_ops = {
	.idx2desc = op32_idx2desc,
	.fill_descriptor = op32_fill_descriptor,
	.poke_tx = op32_poke_tx,
	.tx_suspend = op32_tx_suspend,
	.tx_resume = op32_tx_resume,
	.get_current_rxslot = op32_get_current_rxslot,
	.set_current_rxslot = op32_set_current_rxslot,
};

/* 64bit DMA ops. */
static
struct b43_dmadesc_generic *op64_idx2desc(struct b43_dmaring *ring,
					  int slot,
					  struct b43_dmadesc_meta **meta)
{
	struct b43_dmadesc64 *desc;

	*meta = &(ring->meta[slot]);
	desc = ring->descbase;
	desc = &(desc[slot]);

	return (struct b43_dmadesc_generic *)desc;
}

static void op64_fill_descriptor(struct b43_dmaring *ring,
				 struct b43_dmadesc_generic *desc,
				 dma_addr_t dmaaddr, u16 bufsize,
				 int start, int end, int irq)
{
	struct b43_dmadesc64 *descbase = ring->descbase;
	int slot;
	u32 ctl0 = 0, ctl1 = 0;
	u32 addrlo, addrhi;
	u32 addrext;

	slot = (int)(&(desc->dma64) - descbase);
	B43_WARN_ON(!(slot >= 0 && slot < ring->nr_slots));

	addrlo = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_LOW);
	addrhi = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_HIGH);
	addrext = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_EXT);

	if (slot == ring->nr_slots - 1)
		ctl0 |= B43_DMA64_DCTL0_DTABLEEND;
	if (start)
		ctl0 |= B43_DMA64_DCTL0_FRAMESTART;
	if (end)
		ctl0 |= B43_DMA64_DCTL0_FRAMEEND;
	if (irq)
		ctl0 |= B43_DMA64_DCTL0_IRQ;
	ctl1 |= bufsize & B43_DMA64_DCTL1_BYTECNT;
	ctl1 |= (addrext << B43_DMA64_DCTL1_ADDREXT_SHIFT)
	    & B43_DMA64_DCTL1_ADDREXT_MASK;

	desc->dma64.control0 = cpu_to_le32(ctl0);
	desc->dma64.control1 = cpu_to_le32(ctl1);
	desc->dma64.address_low = cpu_to_le32(addrlo);
	desc->dma64.address_high = cpu_to_le32(addrhi);
}

static void op64_poke_tx(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA64_TXINDEX, ring->index_base +
		      (u32) (slot * sizeof(struct b43_dmadesc64)));
}

static void op64_tx_suspend(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA64_TXCTL, b43_dma_read(ring, B43_DMA64_TXCTL)
		      | B43_DMA64_TXSUSPEND);
}

static void op64_tx_resume(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA64_TXCTL, b43_dma_read(ring, B43_DMA64_TXCTL)
		      & ~B43_DMA64_TXSUSPEND);
}

static int op64_get_current_rxslot(struct b43_dmaring *ring)
{
	u32 val;

	val = b43_dma_read(ring, B43_DMA64_RXSTATUS);
	/* hnddma _dma_getnextrxp: ((status0 & mask) - rcvptrbase) & mask */
	val = (val - ring->index_base) & B43_DMA64_RXSTATDPTR;

	return (val / sizeof(struct b43_dmadesc64));
}

static void op64_set_current_rxslot(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA64_RXINDEX, ring->index_base +
		      (u32) (slot * sizeof(struct b43_dmadesc64)));
}

static const struct b43_dma_ops dma64_ops = {
	.idx2desc = op64_idx2desc,
	.fill_descriptor = op64_fill_descriptor,
	.poke_tx = op64_poke_tx,
	.tx_suspend = op64_tx_suspend,
	.tx_resume = op64_tx_resume,
	.get_current_rxslot = op64_get_current_rxslot,
	.set_current_rxslot = op64_set_current_rxslot,
};

static inline int free_slots(struct b43_dmaring *ring)
{
	return (ring->nr_slots - ring->used_slots);
}

static inline int next_slot(struct b43_dmaring *ring, int slot)
{
	B43_WARN_ON(!(slot >= -1 && slot <= ring->nr_slots - 1));
	if (slot == ring->nr_slots - 1)
		return 0;
	return slot + 1;
}

static inline int prev_slot(struct b43_dmaring *ring, int slot)
{
	B43_WARN_ON(!(slot >= 0 && slot <= ring->nr_slots - 1));
	if (slot == 0)
		return ring->nr_slots - 1;
	return slot - 1;
}

#ifdef CPTCFG_B43_DEBUG
static void update_max_used_slots(struct b43_dmaring *ring,
				  int current_used_slots)
{
	if (current_used_slots <= ring->max_used_slots)
		return;
	ring->max_used_slots = current_used_slots;
	if (b43_debug(ring->dev, B43_DBG_DMAVERBOSE)) {
		b43dbg(ring->dev->wl,
		       "max_used_slots increased to %d on %s ring %d\n",
		       ring->max_used_slots,
		       ring->tx ? "TX" : "RX", ring->index);
	}
}
#else
static inline
    void update_max_used_slots(struct b43_dmaring *ring, int current_used_slots)
{
}
#endif /* DEBUG */

/* Request a slot for usage. */
static inline int request_slot(struct b43_dmaring *ring)
{
	int slot;

	B43_WARN_ON(!ring->tx);
	B43_WARN_ON(ring->stopped);
	B43_WARN_ON(free_slots(ring) == 0);

	slot = next_slot(ring, ring->current_slot);
	ring->current_slot = slot;
	ring->used_slots++;

	update_max_used_slots(ring, ring->used_slots);

	return slot;
}

static u16 b43_dmacontroller_base(enum b43_dmatype type, int controller_idx)
{
	static const u16 map64[] = {
		B43_MMIO_DMA64_BASE0,
		B43_MMIO_DMA64_BASE1,
		B43_MMIO_DMA64_BASE2,
		B43_MMIO_DMA64_BASE3,
		B43_MMIO_DMA64_BASE4,
		B43_MMIO_DMA64_BASE5,
	};
	static const u16 map32[] = {
		B43_MMIO_DMA32_BASE0,
		B43_MMIO_DMA32_BASE1,
		B43_MMIO_DMA32_BASE2,
		B43_MMIO_DMA32_BASE3,
		B43_MMIO_DMA32_BASE4,
		B43_MMIO_DMA32_BASE5,
	};

	if (type == B43_DMA_64BIT) {
		B43_WARN_ON(!(controller_idx >= 0 &&
			      controller_idx < ARRAY_SIZE(map64)));
		return map64[controller_idx];
	}
	B43_WARN_ON(!(controller_idx >= 0 &&
		      controller_idx < ARRAY_SIZE(map32)));
	return map32[controller_idx];
}

static inline
    dma_addr_t map_descbuffer(struct b43_dmaring *ring,
			      unsigned char *buf, size_t len, int tx)
{
	dma_addr_t dmaaddr;

	if (tx) {
		dmaaddr = dma_map_single(ring->dev->dev->dma_dev,
					 buf, len, DMA_TO_DEVICE);
	} else {
		dmaaddr = dma_map_single(ring->dev->dev->dma_dev,
					 buf, len, DMA_FROM_DEVICE);
	}

	return dmaaddr;
}

static inline
    void unmap_descbuffer(struct b43_dmaring *ring,
			  dma_addr_t addr, size_t len, int tx)
{
	if (tx) {
		dma_unmap_single(ring->dev->dev->dma_dev,
				 addr, len, DMA_TO_DEVICE);
	} else {
		dma_unmap_single(ring->dev->dev->dma_dev,
				 addr, len, DMA_FROM_DEVICE);
	}
}

static inline
    void sync_descbuffer_for_cpu(struct b43_dmaring *ring,
				 dma_addr_t addr, size_t len)
{
	B43_WARN_ON(ring->tx);
	dma_sync_single_for_cpu(ring->dev->dev->dma_dev,
				    addr, len, DMA_FROM_DEVICE);
}

static inline
    void sync_descbuffer_for_device(struct b43_dmaring *ring,
				    dma_addr_t addr, size_t len)
{
	B43_WARN_ON(ring->tx);
	dma_sync_single_for_device(ring->dev->dev->dma_dev,
				   addr, len, DMA_FROM_DEVICE);
}

static inline
    void free_descriptor_buffer(struct b43_dmaring *ring,
				struct b43_dmadesc_meta *meta)
{
	if (meta->skb) {
		if (ring->tx)
			ieee80211_free_txskb(ring->dev->wl->hw, meta->skb);
		else
			dev_kfree_skb_any(meta->skb);
		meta->skb = NULL;
	}
}

static int alloc_ringmemory(struct b43_dmaring *ring)
{
	/* The specs call for 4K buffers for 30- and 32-bit DMA with 4K
	 * alignment and 8K buffers for 64-bit DMA with 8K alignment.
	 * In practice we could use smaller buffers for the latter, but the
	 * alignment is really important because of the hardware bug. If bit
	 * 0x00001000 is used in DMA address, some hardware (like BCM4331)
	 * copies that bit into B43_DMA64_RXSTATUS and we get false values from
	 * B43_DMA64_RXSTATDPTR. Let's just use 8K buffers even if we don't use
	 * more than 256 slots for ring.
	 */
	u16 ring_mem_size = (ring->type == B43_DMA_64BIT) ?
				B43_DMA64_RINGMEMSIZE : B43_DMA32_RINGMEMSIZE;

	ring->descbase = dma_alloc_coherent(ring->dev->dev->dma_dev,
					    ring_mem_size, &(ring->dmabase),
					    GFP_KERNEL);
	if (!ring->descbase)
		return -ENOMEM;

	return 0;
}

static void free_ringmemory(struct b43_dmaring *ring)
{
	u16 ring_mem_size = (ring->type == B43_DMA_64BIT) ?
				B43_DMA64_RINGMEMSIZE : B43_DMA32_RINGMEMSIZE;
	dma_free_coherent(ring->dev->dev->dma_dev, ring_mem_size,
			  ring->descbase, ring->dmabase);
}

/* Reset the RX DMA channel */
static int b43_dmacontroller_rx_reset(struct b43_wldev *dev, u16 mmio_base,
				      enum b43_dmatype type)
{
	int i;
	u32 value;
	u16 offset;

	might_sleep();

	offset = (type == B43_DMA_64BIT) ? B43_DMA64_RXCTL : B43_DMA32_RXCTL;
	b43_write32(dev, mmio_base + offset, 0);
	for (i = 0; i < 10; i++) {
		offset = (type == B43_DMA_64BIT) ? B43_DMA64_RXSTATUS :
						   B43_DMA32_RXSTATUS;
		value = b43_read32(dev, mmio_base + offset);
		if (type == B43_DMA_64BIT) {
			value &= B43_DMA64_RXSTAT;
			if (value == B43_DMA64_RXSTAT_DISABLED) {
				i = -1;
				break;
			}
		} else {
			value &= B43_DMA32_RXSTATE;
			if (value == B43_DMA32_RXSTAT_DISABLED) {
				i = -1;
				break;
			}
		}
		msleep(1);
	}
	if (i != -1) {
		b43err(dev->wl, "DMA RX reset timed out\n");
		return -ENODEV;
	}

	return 0;
}

/* Reset the TX DMA channel */
static int b43_dmacontroller_tx_reset(struct b43_wldev *dev, u16 mmio_base,
				      enum b43_dmatype type)
{
	int i;
	u32 value;
	u16 offset;

	might_sleep();

	for (i = 0; i < 10; i++) {
		offset = (type == B43_DMA_64BIT) ? B43_DMA64_TXSTATUS :
						   B43_DMA32_TXSTATUS;
		value = b43_read32(dev, mmio_base + offset);
		if (type == B43_DMA_64BIT) {
			value &= B43_DMA64_TXSTAT;
			if (value == B43_DMA64_TXSTAT_DISABLED ||
			    value == B43_DMA64_TXSTAT_IDLEWAIT ||
			    value == B43_DMA64_TXSTAT_STOPPED)
				break;
		} else {
			value &= B43_DMA32_TXSTATE;
			if (value == B43_DMA32_TXSTAT_DISABLED ||
			    value == B43_DMA32_TXSTAT_IDLEWAIT ||
			    value == B43_DMA32_TXSTAT_STOPPED)
				break;
		}
		msleep(1);
	}
	offset = (type == B43_DMA_64BIT) ? B43_DMA64_TXCTL : B43_DMA32_TXCTL;
	b43_write32(dev, mmio_base + offset, 0);
	for (i = 0; i < 10; i++) {
		offset = (type == B43_DMA_64BIT) ? B43_DMA64_TXSTATUS :
						   B43_DMA32_TXSTATUS;
		value = b43_read32(dev, mmio_base + offset);
		if (type == B43_DMA_64BIT) {
			value &= B43_DMA64_TXSTAT;
			if (value == B43_DMA64_TXSTAT_DISABLED) {
				i = -1;
				break;
			}
		} else {
			value &= B43_DMA32_TXSTATE;
			if (value == B43_DMA32_TXSTAT_DISABLED) {
				i = -1;
				break;
			}
		}
		msleep(1);
	}
	if (i != -1) {
		b43err(dev->wl, "DMA TX reset timed out\n");
		return -ENODEV;
	}
	/* ensure the reset is completed. */
	msleep(1);

	return 0;
}

/* Check if a DMA mapping address is invalid. */
static bool b43_dma_mapping_error(struct b43_dmaring *ring,
				  dma_addr_t addr,
				  size_t buffersize, bool dma_to_device)
{
	if (unlikely(dma_mapping_error(ring->dev->dev->dma_dev, addr)))
		return true;

	switch (ring->type) {
	case B43_DMA_30BIT:
		if ((u64)addr + buffersize > (1ULL << 30))
			goto address_error;
		break;
	case B43_DMA_32BIT:
		if ((u64)addr + buffersize > (1ULL << 32))
			goto address_error;
		break;
	case B43_DMA_64BIT:
		/* Currently we can't have addresses beyond
		 * 64bit in the kernel. */
		break;
	}

	/* The address is OK. */
	return false;

address_error:
	/* We can't support this address. Unmap it again. */
	unmap_descbuffer(ring, addr, buffersize, dma_to_device);

	return true;
}

static bool b43_rx_buffer_is_poisoned(struct b43_dmaring *ring, struct sk_buff *skb)
{
	unsigned char *f = skb->data + ring->frameoffset;

	return ((f[0] & f[1] & f[2] & f[3] & f[4] & f[5] & f[6] & f[7]) == 0xFF);
}

static void b43_poison_rx_buffer(struct b43_dmaring *ring, struct sk_buff *skb)
{
	struct b43_rxhdr_fw4 *rxhdr;
	unsigned char *frame;

	/* This poisons the RX buffer to detect DMA failures. */

	rxhdr = (struct b43_rxhdr_fw4 *)(skb->data);
	rxhdr->frame_len = 0;

	B43_WARN_ON(ring->rx_buffersize < ring->frameoffset + sizeof(struct b43_plcp_hdr6) + 2);
	frame = skb->data + ring->frameoffset;
	memset(frame, 0xFF, sizeof(struct b43_plcp_hdr6) + 2 /* padding */);
}

static int setup_rx_descbuffer(struct b43_dmaring *ring,
			       struct b43_dmadesc_generic *desc,
			       struct b43_dmadesc_meta *meta, gfp_t gfp_flags)
{
	dma_addr_t dmaaddr;
	struct sk_buff *skb;

	B43_WARN_ON(ring->tx);

	skb = __dev_alloc_skb(ring->rx_buffersize, gfp_flags);
	if (unlikely(!skb))
		return -ENOMEM;
	b43_poison_rx_buffer(ring, skb);
	dmaaddr = map_descbuffer(ring, skb->data, ring->rx_buffersize, 0);
	if (b43_dma_mapping_error(ring, dmaaddr, ring->rx_buffersize, 0)) {
		/* ugh. try to realloc in zone_dma */
		gfp_flags |= GFP_DMA;

		dev_kfree_skb_any(skb);

		skb = __dev_alloc_skb(ring->rx_buffersize, gfp_flags);
		if (unlikely(!skb))
			return -ENOMEM;
		b43_poison_rx_buffer(ring, skb);
		dmaaddr = map_descbuffer(ring, skb->data,
					 ring->rx_buffersize, 0);
		if (b43_dma_mapping_error(ring, dmaaddr, ring->rx_buffersize, 0)) {
			b43err(ring->dev->wl, "RX DMA buffer allocation failed\n");
			dev_kfree_skb_any(skb);
			return -EIO;
		}
	}

	meta->skb = skb;
	meta->dmaaddr = dmaaddr;
	ring->ops->fill_descriptor(ring, desc, dmaaddr,
				   ring->rx_buffersize, 0, 0, 0);

	return 0;
}

/* Allocate the initial descbuffers.
 * This is used for an RX ring only.
 */
static int alloc_initial_descbuffers(struct b43_dmaring *ring)
{
	int i, err = -ENOMEM;
	struct b43_dmadesc_generic *desc;
	struct b43_dmadesc_meta *meta;

	for (i = 0; i < ring->nr_slots; i++) {
		desc = ring->ops->idx2desc(ring, i, &meta);

		err = setup_rx_descbuffer(ring, desc, meta, GFP_KERNEL);
		if (err) {
			b43err(ring->dev->wl,
			       "Failed to allocate initial descbuffers\n");
			goto err_unwind;
		}
	}
	mb();
	ring->used_slots = ring->nr_slots;
	err = 0;
      out:
	return err;

      err_unwind:
	for (i--; i >= 0; i--) {
		desc = ring->ops->idx2desc(ring, i, &meta);

		unmap_descbuffer(ring, meta->dmaaddr, ring->rx_buffersize, 0);
		dev_kfree_skb(meta->skb);
	}
	goto out;
}

/*
 * DMA64 burst / outstanding read / prefetch fields for the BCM4360 on PCIe
 * (core rev >= 40), as wl configures hnddma.
 *
 * wl.ko: wlc_bmac_attach fills the parameter table at .data 0x7fe40 when the
 * bus is PCI (sih->bustype == 1, 0x1647c4..0x164800), and
 * wlc_bmac_dma_param_set (0x15bf5c..0x15c064; core rev > 31, chip 0x4360
 * among others) passes it to hnddma: TX ids 0..3 = 1, 3, 3, 4; RX ids
 * 0x100..0x102 = 2, 0, 4.
 * Stock R6400 kernel hnddma: _dma_param_set (0xc02b54f4) stores TX id 0
 * (outstanding reads), 1 (prefetch control), 2 (prefetch threshold),
 * 3 (burst length) and RX 0x100 (prefetch control), 0x101 (prefetch
 * threshold), 0x102 (burst length). dma64_txinit (0xc02b5d04..0xc02b5d30)
 * puts them in the TX control word at bits 6..7 (MR), 21..23 (PC),
 * 24..25 (PT), 18..20 (BL); _dma_rxenable (0xc02b4a84..0xc02b4ab8) puts the
 * RX ones at 21..23, 24..25, 18..20.
 * b43 leaves all of them 0 (16 byte bursts, one outstanding read, no
 * prefetch).
 */
#define B43_DMA64_TXMR_SHIFT		6
#define B43_DMA64_TXBL_SHIFT		18
#define B43_DMA64_TXPC_SHIFT		21
#define B43_DMA64_TXPT_SHIFT		24
#define B43_DMA64_RXBL_SHIFT		18
#define B43_DMA64_RXPC_SHIFT		21
#define B43_DMA64_RXPT_SHIFT		24

static u32 b43_dma64_ctl_params_rev40(struct b43_dmaring *ring)
{
	if (ring->dev->dev->core_rev < 40)
		return 0;
	/*
	 * BCM53573 (core rev 56) sits on the SoC AXI bus, not PCI, so wl skips
	 * the PCI table above: its TX control words read 0x00080901 and RX
	 * 0x00080851 on the AC6 (burst length 2, no prefetch, TX bit 8 set).
	 * With the PCI values the ucode never saw a TX frame.
	 */
	if (ring->dev->dev->core_rev == 56) {
		if (ring->tx)
			return (2 << B43_DMA64_TXBL_SHIFT) | 0x100;
		return 2 << B43_DMA64_RXBL_SHIFT;
	}
	if (ring->tx)
		return (1 << B43_DMA64_TXMR_SHIFT) |
		       (4 << B43_DMA64_TXBL_SHIFT) |
		       (3 << B43_DMA64_TXPC_SHIFT) |
		       (3 << B43_DMA64_TXPT_SHIFT);
	return (4 << B43_DMA64_RXBL_SHIFT) |
	       (2 << B43_DMA64_RXPC_SHIFT) |
	       (0 << B43_DMA64_RXPT_SHIFT);
}

/* Do initial setup of the DMA controller.
 * Reset the controller, write the ring busaddress
 * and switch the "enable" bit on.
 */
static int dmacontroller_setup(struct b43_dmaring *ring)
{
	int err = 0;
	u32 value;
	u32 addrext;
	bool parity = ring->dev->dma.parity;
	u32 addrlo;
	u32 addrhi;

	if (ring->tx) {
		if (ring->type == B43_DMA_64BIT) {
			u64 ringbase = (u64) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);
			addrhi = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_HIGH);

			value = B43_DMA64_TXENABLE;
			value |= b43_dma64_ctl_params_rev40(ring);
			value |= (addrext << B43_DMA64_TXADDREXT_SHIFT)
			    & B43_DMA64_TXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA64_TXPARITYDISABLE;
			if (ring->desc_unaligned) {
				/* hnddma dma64_txinit: ring address before the
				 * enable bit on this engine type. */
				b43_dma_write(ring, B43_DMA64_TXRINGLO, addrlo);
				b43_dma_write(ring, B43_DMA64_TXRINGHI, addrhi);
				b43_dma_write(ring, B43_DMA64_TXCTL, value);
			} else {
				b43_dma_write(ring, B43_DMA64_TXCTL, value);
				b43_dma_write(ring, B43_DMA64_TXRINGLO, addrlo);
				b43_dma_write(ring, B43_DMA64_TXRINGHI, addrhi);
			}
		} else {
			u32 ringbase = (u32) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);

			value = B43_DMA32_TXENABLE;
			value |= (addrext << B43_DMA32_TXADDREXT_SHIFT)
			    & B43_DMA32_TXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA32_TXPARITYDISABLE;
			b43_dma_write(ring, B43_DMA32_TXCTL, value);
			b43_dma_write(ring, B43_DMA32_TXRING, addrlo);
		}
	} else {
		err = alloc_initial_descbuffers(ring);
		if (err)
			goto out;
		if (ring->type == B43_DMA_64BIT) {
			u64 ringbase = (u64) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);
			addrhi = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_HIGH);

			value = (ring->frameoffset << B43_DMA64_RXFROFF_SHIFT);
			value |= B43_DMA64_RXENABLE;
			value |= b43_dma64_ctl_params_rev40(ring);
			value |= (addrext << B43_DMA64_RXADDREXT_SHIFT)
			    & B43_DMA64_RXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA64_RXPARITYDISABLE;
			if (ring->desc_unaligned) {
				/* hnddma _dma_rxinit: ring address before the
				 * enable bit on this engine type. */
				b43_dma_write(ring, B43_DMA64_RXRINGLO, addrlo);
				b43_dma_write(ring, B43_DMA64_RXRINGHI, addrhi);
				b43_dma_write(ring, B43_DMA64_RXCTL, value);
			} else {
				b43_dma_write(ring, B43_DMA64_RXCTL, value);
				b43_dma_write(ring, B43_DMA64_RXRINGLO, addrlo);
				b43_dma_write(ring, B43_DMA64_RXRINGHI, addrhi);
			}
			b43_dma_write(ring, B43_DMA64_RXINDEX, ring->index_base +
				      ring->nr_slots * sizeof(struct b43_dmadesc64));
		} else {
			u32 ringbase = (u32) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);

			value = (ring->frameoffset << B43_DMA32_RXFROFF_SHIFT);
			value |= B43_DMA32_RXENABLE;
			value |= (addrext << B43_DMA32_RXADDREXT_SHIFT)
			    & B43_DMA32_RXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA32_RXPARITYDISABLE;
			b43_dma_write(ring, B43_DMA32_RXCTL, value);
			b43_dma_write(ring, B43_DMA32_RXRING, addrlo);
			b43_dma_write(ring, B43_DMA32_RXINDEX, ring->nr_slots *
				      sizeof(struct b43_dmadesc32));
		}
	}

out:
	return err;
}

/* Shutdown the DMA controller. */
static void dmacontroller_cleanup(struct b43_dmaring *ring)
{
	if (ring->tx) {
		b43_dmacontroller_tx_reset(ring->dev, ring->mmio_base,
					   ring->type);
		if (ring->type == B43_DMA_64BIT) {
			b43_dma_write(ring, B43_DMA64_TXRINGLO, 0);
			b43_dma_write(ring, B43_DMA64_TXRINGHI, 0);
		} else
			b43_dma_write(ring, B43_DMA32_TXRING, 0);
	} else {
		b43_dmacontroller_rx_reset(ring->dev, ring->mmio_base,
					   ring->type);
		if (ring->type == B43_DMA_64BIT) {
			b43_dma_write(ring, B43_DMA64_RXRINGLO, 0);
			b43_dma_write(ring, B43_DMA64_RXRINGHI, 0);
		} else
			b43_dma_write(ring, B43_DMA32_RXRING, 0);
	}
}

static void free_all_descbuffers(struct b43_dmaring *ring)
{
	struct b43_dmadesc_meta *meta;
	int i;

	if (!ring->used_slots)
		return;
	for (i = 0; i < ring->nr_slots; i++) {
		/* get meta - ignore returned value */
		ring->ops->idx2desc(ring, i, &meta);

		if (!meta->skb || b43_dma_ptr_is_poisoned(meta->skb)) {
			B43_WARN_ON(!ring->tx);
			continue;
		}
		if (ring->tx) {
			unmap_descbuffer(ring, meta->dmaaddr,
					 meta->skb->len, 1);
		} else {
			unmap_descbuffer(ring, meta->dmaaddr,
					 ring->rx_buffersize, 0);
		}
		free_descriptor_buffer(ring, meta);
	}
}

static enum b43_dmatype b43_engine_type(struct b43_wldev *dev)
{
	u32 tmp;
	u16 mmio_base;

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOST);
		if (tmp & BCMA_IOST_DMA64)
			return B43_DMA_64BIT;
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		tmp = ssb_read32(dev->dev->sdev, SSB_TMSHIGH);
		if (tmp & SSB_TMSHIGH_DMA64)
			return B43_DMA_64BIT;
		break;
#endif
	}

	mmio_base = b43_dmacontroller_base(0, 0);
	b43_write32(dev, mmio_base + B43_DMA32_TXCTL, B43_DMA32_TXADDREXT_MASK);
	tmp = b43_read32(dev, mmio_base + B43_DMA32_TXCTL);
	if (tmp & B43_DMA32_TXADDREXT_MASK)
		return B43_DMA_32BIT;
	return B43_DMA_30BIT;
}

/*
 * Probe a DMA64 engine the way hnddma's dma_attach does in the stock R6400
 * kernel (2.6.36.4brcmarm+, R6400-V1.0.1.78), which wl.ko uses for the
 * BCM4360 (core rev 42):
 *
 * - address mask: write 0xffffffff to the ring address low word; if its low
 *   12 bits read back as 0 the mask is 0x1fff, else it is the index register
 *   read back, ORed with 0xf (0xc02b6f14..0xc02b6f58 RX, 0xc02b6fdc..0xc02b7024
 *   TX). Logged only: b43's 8 KiB rings are 8 KiB aligned, so the 0x1fff
 *   status mask below gives the right slot either way.
 * - descriptor table alignment: write 0xff0 to the ring address low word; a
 *   non-zero read back means the engine needs no aligned table (aligndesc_4k
 *   false, 0xc02b7240..0xc02b72ac). For such an engine hnddma programs the
 *   ring address before the enable bit (_dma_rxinit 0xc02b5c50..0xc02b5c70,
 *   dma64_txinit 0xc02b5d54..0xc02b5db0) and adds the ring base low word to
 *   every index register write (_dma_ddtable_init 0xc02b4784..0xc02b4798,
 *   _dma_rxfill 0xc048c928..0xc048c934, _dma_getnextrxp 0xc048c4b8..0xc048c4cc).
 *
 * hnddma probes the TX registers of a controller when it has them; b43 sets
 * up TX ring 0 before RX ring 0, so each ring probes its own registers here
 * to leave a live ring address alone. The engine is disabled at this point.
 */
static void b43_dma64_probe(struct b43_dmaring *ring)
{
	u16 lo = ring->tx ? B43_DMA64_TXRINGLO : B43_DMA64_RXRINGLO;
	u16 idx = ring->tx ? B43_DMA64_TXINDEX : B43_DMA64_RXINDEX;
	u32 mask, align;

	b43_dma_write(ring, lo, 0xffffffff);
	mask = b43_dma_read(ring, lo);
	if (mask & 0xfff)
		mask = b43_dma_read(ring, idx) | 0xf;
	else
		mask = 0x1fff;

	b43_dma_write(ring, lo, 0xff0);
	align = b43_dma_read(ring, lo);
	b43_dma_write(ring, lo, 0);

	ring->desc_unaligned = (align != 0);
	ring->index_base = ring->desc_unaligned ?
			   lower_32_bits(ring->dmabase) : 0;

	if (ring->desc_unaligned || mask != 0x1fff ||
	    ring->dev->dev->core_rev >= 40)
		b43info(ring->dev->wl, "DMA64 %s ring %d: address mask 0x%04x, "
			"0xff0 reads back 0x%03x, ring base 0x%08x, index base 0x%08x\n",
			ring->tx ? "TX" : "RX", ring->index, mask, align,
			lower_32_bits(ring->dmabase), ring->index_base);
}

/* Main initialization function. */
static
struct b43_dmaring *b43_setup_dmaring(struct b43_wldev *dev,
				      int controller_index,
				      int for_tx,
				      enum b43_dmatype type)
{
	struct b43_dmaring *ring;
	int i, err;
	dma_addr_t dma_test;

	ring = kzalloc(sizeof(*ring), GFP_KERNEL);
	if (!ring)
		goto out;

	ring->nr_slots = B43_RXRING_SLOTS;
	if (for_tx)
		ring->nr_slots = B43_TXRING_SLOTS;

	ring->meta = kcalloc(ring->nr_slots, sizeof(struct b43_dmadesc_meta),
			     GFP_KERNEL);
	if (!ring->meta)
		goto err_kfree_ring;
	for (i = 0; i < ring->nr_slots; i++)
		ring->meta->skb = B43_DMA_PTR_POISON;

	ring->type = type;
	ring->dev = dev;
	ring->mmio_base = b43_dmacontroller_base(type, controller_index);
	ring->index = controller_index;
	if (type == B43_DMA_64BIT)
		ring->ops = &dma64_ops;
	else
		ring->ops = &dma32_ops;
	if (for_tx) {
		ring->tx = true;
		ring->current_slot = -1;
	} else {
		if (ring->index == 0) {
			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				if (dev->dev->core_rev >= 40) {
					ring->rx_buffersize = B43_DMA0_RX_FW598_REV40_BUFSIZE;
					ring->frameoffset = B43_DMA0_RX_FW598_FO_REV40;
				} else {
					ring->rx_buffersize = B43_DMA0_RX_FW598_BUFSIZE;
					ring->frameoffset = B43_DMA0_RX_FW598_FO;
				}
				break;
			case B43_FW_HDR_410:
			case B43_FW_HDR_351:
				ring->rx_buffersize = B43_DMA0_RX_FW351_BUFSIZE;
				ring->frameoffset = B43_DMA0_RX_FW351_FO;
				break;
			}
		} else
			B43_WARN_ON(1);
	}
#ifdef CPTCFG_B43_DEBUG
	ring->last_injected_overflow = jiffies;
#endif

	if (for_tx) {
		/* Assumption: B43_TXRING_SLOTS can be divided by TX_SLOTS_PER_FRAME */
		BUILD_BUG_ON(B43_TXRING_SLOTS % TX_SLOTS_PER_FRAME != 0);

		ring->txhdr_cache = kcalloc(ring->nr_slots / TX_SLOTS_PER_FRAME,
					    b43_txhdr_size(dev),
					    GFP_KERNEL);
		if (!ring->txhdr_cache)
			goto err_kfree_meta;

		/* test for ability to dma to txhdr_cache */
		dma_test = dma_map_single(dev->dev->dma_dev,
					  ring->txhdr_cache,
					  b43_txhdr_size(dev),
					  DMA_TO_DEVICE);

		if (b43_dma_mapping_error(ring, dma_test,
					  b43_txhdr_size(dev), 1)) {
			/* ugh realloc */
			kfree(ring->txhdr_cache);
			ring->txhdr_cache = kcalloc(ring->nr_slots / TX_SLOTS_PER_FRAME,
						    b43_txhdr_size(dev),
						    GFP_KERNEL | GFP_DMA);
			if (!ring->txhdr_cache)
				goto err_kfree_meta;

			dma_test = dma_map_single(dev->dev->dma_dev,
						  ring->txhdr_cache,
						  b43_txhdr_size(dev),
						  DMA_TO_DEVICE);

			if (b43_dma_mapping_error(ring, dma_test,
						  b43_txhdr_size(dev), 1)) {

				b43err(dev->wl,
				       "TXHDR DMA allocation failed\n");
				goto err_kfree_txhdr_cache;
			}
		}

		dma_unmap_single(dev->dev->dma_dev,
				 dma_test, b43_txhdr_size(dev),
				 DMA_TO_DEVICE);
	}

	err = alloc_ringmemory(ring);
	if (err)
		goto err_kfree_txhdr_cache;
	if (type == B43_DMA_64BIT)
		b43_dma64_probe(ring);
	err = dmacontroller_setup(ring);
	if (err)
		goto err_free_ringmemory;

      out:
	return ring;

      err_free_ringmemory:
	free_ringmemory(ring);
      err_kfree_txhdr_cache:
	kfree(ring->txhdr_cache);
      err_kfree_meta:
	kfree(ring->meta);
      err_kfree_ring:
	kfree(ring);
	ring = NULL;
	goto out;
}

#define divide(a, b)	({	\
	typeof(a) __a = a;	\
	do_div(__a, b);		\
	__a;			\
  })

#define modulo(a, b)	({	\
	typeof(a) __a = a;	\
	do_div(__a, b);		\
  })

/* Main cleanup function. */
static void b43_destroy_dmaring(struct b43_dmaring *ring,
				const char *ringname)
{
	if (!ring)
		return;

#ifdef CPTCFG_B43_DEBUG
	{
		/* Print some statistics. */
		u64 failed_packets = ring->nr_failed_tx_packets;
		u64 succeed_packets = ring->nr_succeed_tx_packets;
		u64 nr_packets = failed_packets + succeed_packets;
		u64 permille_failed = 0, average_tries = 0;

		if (nr_packets)
			permille_failed = divide(failed_packets * 1000, nr_packets);
		if (nr_packets)
			average_tries = divide(ring->nr_total_packet_tries * 100, nr_packets);

		b43dbg(ring->dev->wl, "DMA-%u %s: "
		       "Used slots %d/%d, Failed frames %llu/%llu = %llu.%01llu%%, "
		       "Average tries %llu.%02llu\n",
		       (unsigned int)(ring->type), ringname,
		       ring->max_used_slots,
		       ring->nr_slots,
		       (unsigned long long)failed_packets,
		       (unsigned long long)nr_packets,
		       (unsigned long long)divide(permille_failed, 10),
		       (unsigned long long)modulo(permille_failed, 10),
		       (unsigned long long)divide(average_tries, 100),
		       (unsigned long long)modulo(average_tries, 100));
	}
#endif /* DEBUG */

	/* Device IRQs are disabled prior entering this function,
	 * so no need to take care of concurrency with rx handler stuff.
	 */
	dmacontroller_cleanup(ring);
	free_all_descbuffers(ring);
	free_ringmemory(ring);

	kfree(ring->txhdr_cache);
	kfree(ring->meta);
	kfree(ring);
}

#define destroy_ring(dma, ring) do {				\
	b43_destroy_dmaring((dma)->ring, __stringify(ring));	\
	(dma)->ring = NULL;					\
    } while (0)

void b43_dma_free(struct b43_wldev *dev)
{
	struct b43_dma *dma;

	if (b43_using_pio_transfers(dev))
		return;
	dma = &dev->dma;

	destroy_ring(dma, rx_ring);
	destroy_ring(dma, tx_ring_AC_BK);
	destroy_ring(dma, tx_ring_AC_BE);
	destroy_ring(dma, tx_ring_AC_VI);
	destroy_ring(dma, tx_ring_AC_VO);
	destroy_ring(dma, tx_ring_mcast);
}

/* Some hardware with 64-bit DMA seems to be bugged and looks for translation
 * bit in low address word instead of high one.
 */
static bool b43_dma_translation_in_low_word(struct b43_wldev *dev,
					    enum b43_dmatype type)
{
	if (type != B43_DMA_64BIT)
		return true;

#ifdef CPTCFG_B43_SSB
	if (dev->dev->bus_type == B43_BUS_SSB &&
	    dev->dev->sdev->bus->bustype == SSB_BUSTYPE_PCI &&
	    !(pci_is_pcie(dev->dev->sdev->bus->host_pci) &&
	      ssb_read32(dev->dev->sdev, SSB_TMSHIGH) & SSB_TMSHIGH_DMA64))
			return true;
#endif
	return false;
}

int b43_dma_init(struct b43_wldev *dev)
{
	struct b43_dma *dma = &dev->dma;
	enum b43_dmatype type = b43_engine_type(dev);
	int err;

	err = dma_set_mask_and_coherent(dev->dev->dma_dev, DMA_BIT_MASK(type));
	if (err) {
		b43err(dev->wl, "The machine/kernel does not support "
		       "the required %u-bit DMA mask\n", type);
		return err;
	}

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		dma->translation = bcma_core_dma_translation(dev->dev->bdev);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		dma->translation = ssb_dma_translation(dev->dev->sdev);
		break;
#endif
	}
	dma->translation_in_low = b43_dma_translation_in_low_word(dev, type);

	dma->parity = true;
#ifdef CPTCFG_B43_BCMA
	/* TODO: find out which SSB devices need disabling parity */
	if (dev->dev->bus_type == B43_BUS_BCMA)
		dma->parity = false;
#endif

	err = -ENOMEM;
	/* setup TX DMA channels. */
	dma->tx_ring_AC_BK = b43_setup_dmaring(dev, 0, 1, type);
	if (!dma->tx_ring_AC_BK)
		goto out;

	dma->tx_ring_AC_BE = b43_setup_dmaring(dev, 1, 1, type);
	if (!dma->tx_ring_AC_BE)
		goto err_destroy_bk;

	dma->tx_ring_AC_VI = b43_setup_dmaring(dev, 2, 1, type);
	if (!dma->tx_ring_AC_VI)
		goto err_destroy_be;

	dma->tx_ring_AC_VO = b43_setup_dmaring(dev, 3, 1, type);
	if (!dma->tx_ring_AC_VO)
		goto err_destroy_vi;

	dma->tx_ring_mcast = b43_setup_dmaring(dev, 4, 1, type);
	if (!dma->tx_ring_mcast)
		goto err_destroy_vo;

	/* setup RX DMA channel. */
	dma->rx_ring = b43_setup_dmaring(dev, 0, 0, type);
	if (!dma->rx_ring)
		goto err_destroy_mcast;

	/* No support for the TX status DMA ring. */
	B43_WARN_ON(dev->dev->core_rev < 5);

	b43dbg(dev->wl, "%u-bit DMA initialized\n",
	       (unsigned int)type);
	err = 0;
out:
	return err;

err_destroy_mcast:
	destroy_ring(dma, tx_ring_mcast);
err_destroy_vo:
	destroy_ring(dma, tx_ring_AC_VO);
err_destroy_vi:
	destroy_ring(dma, tx_ring_AC_VI);
err_destroy_be:
	destroy_ring(dma, tx_ring_AC_BE);
err_destroy_bk:
	destroy_ring(dma, tx_ring_AC_BK);
	return err;
}

/* Generate a cookie for the TX header. */
static u16 generate_cookie(struct b43_dmaring *ring, int slot)
{
	u16 cookie;

	/* Use the upper 4 bits of the cookie as
	 * DMA controller ID and store the slot number
	 * in the lower 12 bits.
	 * Note that the cookie must never be 0, as this
	 * is a special value used in RX path.
	 * It can also not be 0xFFFF because that is special
	 * for multicast frames.
	 *
	 * The AC-PHY TX header (ucode 907) gets wl.ko's frame id layout
	 * instead: FIFO in bits 0..2, a counter in bits 5..14
	 * (wlc_d11hdrs 0x11b734..0x11b744; FIFO 4 is broadcast/multicast,
	 * 0x11b6d0..0x11b6f0). The ring index is the FIFO number and the slot
	 * takes the counter's place. Whether ucode 907 reads these bits is not
	 * known; matching wl leaves nothing for it to disagree with.
	 */
	if (b43_txhdr_is_ac(ring->dev)) {
		B43_WARN_ON(slot & ~(B43_TXH_AC_FID_SLOT >>
				     B43_TXH_AC_FID_SLOT_SHIFT));
		return ((u16)slot << B43_TXH_AC_FID_SLOT_SHIFT) |
		       ((u16)ring->index & B43_TXH_AC_FID_FIFO);
	}

	cookie = (((u16)ring->index + 1) << 12);
	B43_WARN_ON(slot & ~0x0FFF);
	cookie |= (u16)slot;

	return cookie;
}

/* Inspect a cookie and find out to which controller/slot it belongs. */
static
struct b43_dmaring *parse_cookie(struct b43_wldev *dev, u16 cookie, int *slot)
{
	struct b43_dma *dma = &dev->dma;
	struct b43_dmaring *ring = NULL;

	if (b43_txhdr_is_ac(dev)) {
		switch (cookie & B43_TXH_AC_FID_FIFO) {
		case 0:
			ring = dma->tx_ring_AC_BK;
			break;
		case 1:
			ring = dma->tx_ring_AC_BE;
			break;
		case 2:
			ring = dma->tx_ring_AC_VI;
			break;
		case 3:
			ring = dma->tx_ring_AC_VO;
			break;
		case 4:
			ring = dma->tx_ring_mcast;
			break;
		}
		*slot = (cookie & B43_TXH_AC_FID_SLOT) >>
			B43_TXH_AC_FID_SLOT_SHIFT;
		goto check;
	}

	switch (cookie & 0xF000) {
	case 0x1000:
		ring = dma->tx_ring_AC_BK;
		break;
	case 0x2000:
		ring = dma->tx_ring_AC_BE;
		break;
	case 0x3000:
		ring = dma->tx_ring_AC_VI;
		break;
	case 0x4000:
		ring = dma->tx_ring_AC_VO;
		break;
	case 0x5000:
		ring = dma->tx_ring_mcast;
		break;
	}
	*slot = (cookie & 0x0FFF);
check:
	if (unlikely(!ring || *slot < 0 || *slot >= ring->nr_slots)) {
		b43dbg(dev->wl, "TX-status contains "
		       "invalid cookie: 0x%04X\n", cookie);
		return NULL;
	}

	return ring;
}

static int dma_tx_fragment(struct b43_dmaring *ring,
			   struct sk_buff *skb)
{
	const struct b43_dma_ops *ops = ring->ops;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct b43_private_tx_info *priv_info = b43_get_priv_tx_info(info);
	u8 *header;
	int slot, old_top_slot, old_used_slots;
	int err;
	struct b43_dmadesc_generic *desc;
	struct b43_dmadesc_meta *meta;
	struct b43_dmadesc_meta *meta_hdr;
	u16 cookie;
	size_t hdrsize = b43_txhdr_size(ring->dev);

	/* Important note: If the number of used DMA slots per TX frame
	 * is changed here, the TX_SLOTS_PER_FRAME definition at the top of
	 * the file has to be updated, too!
	 */

	old_top_slot = ring->current_slot;
	old_used_slots = ring->used_slots;

	/* Get a slot for the header. */
	slot = request_slot(ring);
	desc = ops->idx2desc(ring, slot, &meta_hdr);
	memset(meta_hdr, 0, sizeof(*meta_hdr));

	header = &(ring->txhdr_cache[(slot / TX_SLOTS_PER_FRAME) * hdrsize]);
	cookie = generate_cookie(ring, slot);
	err = b43_generate_txhdr(ring->dev, header,
				 skb, info, cookie);
	if (unlikely(err)) {
		ring->current_slot = old_top_slot;
		ring->used_slots = old_used_slots;
		return err;
	}

	meta_hdr->dmaaddr = map_descbuffer(ring, (unsigned char *)header,
					   hdrsize, 1);
	if (b43_dma_mapping_error(ring, meta_hdr->dmaaddr, hdrsize, 1)) {
		ring->current_slot = old_top_slot;
		ring->used_slots = old_used_slots;
		return -EIO;
	}
	ops->fill_descriptor(ring, desc, meta_hdr->dmaaddr,
			     hdrsize, 1, 0, 0);

	/* Get a slot for the payload. */
	slot = request_slot(ring);
	desc = ops->idx2desc(ring, slot, &meta);
	memset(meta, 0, sizeof(*meta));

	meta->skb = skb;
	meta->is_last_fragment = true;
	priv_info->bouncebuffer = NULL;

	meta->dmaaddr = map_descbuffer(ring, skb->data, skb->len, 1);
	/* create a bounce buffer in zone_dma on mapping failure. */
	if (b43_dma_mapping_error(ring, meta->dmaaddr, skb->len, 1)) {
		priv_info->bouncebuffer = kmemdup(skb->data, skb->len,
						  GFP_ATOMIC | GFP_DMA);
		if (!priv_info->bouncebuffer) {
			ring->current_slot = old_top_slot;
			ring->used_slots = old_used_slots;
			err = -ENOMEM;
			goto out_unmap_hdr;
		}

		meta->dmaaddr = map_descbuffer(ring, priv_info->bouncebuffer, skb->len, 1);
		if (b43_dma_mapping_error(ring, meta->dmaaddr, skb->len, 1)) {
			kfree(priv_info->bouncebuffer);
			priv_info->bouncebuffer = NULL;
			ring->current_slot = old_top_slot;
			ring->used_slots = old_used_slots;
			err = -EIO;
			goto out_unmap_hdr;
		}
	}

	ops->fill_descriptor(ring, desc, meta->dmaaddr, skb->len, 0, 1, 1);

	if (info->flags & IEEE80211_TX_CTL_SEND_AFTER_DTIM) {
		/* Tell the firmware about the cookie of the last
		 * mcast frame, so it can clear the more-data bit in it. */
		b43_shm_write16(ring->dev, B43_SHM_SHARED,
				B43_SHM_SH_MCASTCOOKIE, cookie);
	}
	/* Now transfer the whole frame. */
	wmb();
	/*
	 * (acphy_ampdu bit 4): a frame of an aggregate burst waits on the
	 * ring until the burst is complete (b43_tx_work calls
	 * b43_dma_tx_poke_pending), so the ucode finds the whole burst
	 * pending when it looks for MPDUs to join (its wait at 0x397..0x3A2
	 * only covers frames the DMA already holds).
	 */
	if (b43_ac_ampdu(ring->dev, B43_AC_AMPDU_BURST) &&
	    b43_ac_ampdu_same_burst(skb, ring->dev->ac_ampdu_next)) {
		ring->poke_pending = true;
		ring->poke_slot = next_slot(ring, slot);
	} else {
		ring->poke_pending = false;
		ops->poke_tx(ring, next_slot(ring, slot));
	}
	return 0;

out_unmap_hdr:
	unmap_descbuffer(ring, meta_hdr->dmaaddr,
			 hdrsize, 1);
	return err;
}

static inline int should_inject_overflow(struct b43_dmaring *ring)
{
#ifdef CPTCFG_B43_DEBUG
	if (unlikely(b43_debug(ring->dev, B43_DBG_DMAOVERFLOW))) {
		/* Check if we should inject another ringbuffer overflow
		 * to test handling of this situation in the stack. */
		unsigned long next_overflow;

		next_overflow = ring->last_injected_overflow + HZ;
		if (time_after(jiffies, next_overflow)) {
			ring->last_injected_overflow = jiffies;
			b43dbg(ring->dev->wl,
			       "Injecting TX ring overflow on "
			       "DMA controller %d\n", ring->index);
			return 1;
		}
	}
#endif /* CPTCFG_B43_DEBUG */
	return 0;
}

/* Static mapping of mac80211's queues (priorities) to b43 DMA rings. */
static struct b43_dmaring *select_ring_by_priority(struct b43_wldev *dev,
						   u8 queue_prio)
{
	struct b43_dmaring *ring;

	if (dev->qos_enabled) {
		/* 0 = highest priority */
		switch (queue_prio) {
		default:
			B43_WARN_ON(1);
			fallthrough;
		case 0:
			ring = dev->dma.tx_ring_AC_VO;
			break;
		case 1:
			ring = dev->dma.tx_ring_AC_VI;
			break;
		case 2:
			ring = dev->dma.tx_ring_AC_BE;
			break;
		case 3:
			ring = dev->dma.tx_ring_AC_BK;
			break;
		}
	} else
		ring = dev->dma.tx_ring_AC_BE;

	return ring;
}

/*
 * start the DMA engine of every TX ring that holds a burst frame put
 * there without a poke (dma_tx_fragment). Called by b43_tx_work after each
 * queue and after a failed submit, so no frame stays on a ring unstarted.
 */
void b43_dma_tx_poke_pending(struct b43_wldev *dev)
{
	struct b43_dmaring *rings[] = {
		dev->dma.tx_ring_AC_BK, dev->dma.tx_ring_AC_BE,
		dev->dma.tx_ring_AC_VI, dev->dma.tx_ring_AC_VO,
		dev->dma.tx_ring_mcast,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rings); i++) {
		struct b43_dmaring *ring = rings[i];

		if (!ring || !ring->poke_pending)
			continue;
		ring->poke_pending = false;
		wmb();
		ring->ops->poke_tx(ring, ring->poke_slot);
	}
}

int b43_dma_tx(struct b43_wldev *dev, struct sk_buff *skb)
{
	struct b43_dmaring *ring;
	struct ieee80211_hdr *hdr;
	int err = 0;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	hdr = (struct ieee80211_hdr *)skb->data;
	if (info->flags & IEEE80211_TX_CTL_SEND_AFTER_DTIM) {
		/* The multicast ring will be sent after the DTIM */
		ring = dev->dma.tx_ring_mcast;
		/* Set the more-data bit. Ucode will clear it on
		 * the last frame for us. */
		hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_MOREDATA);
	} else {
		/* Decide by priority where to put this frame. */
		ring = select_ring_by_priority(
			dev, skb_get_queue_mapping(skb));
	}

	B43_WARN_ON(!ring->tx);

	if (unlikely(ring->stopped)) {
		/* We get here only because of a bug in mac80211.
		 * Because of a race, one packet may be queued after
		 * the queue is stopped, thus we got called when we shouldn't.
		 * For now, just refuse the transmit. */
		if (b43_debug(dev, B43_DBG_DMAVERBOSE))
			b43err(dev->wl, "Packet after queue stopped\n");
		err = -ENOSPC;
		goto out;
	}

	if (WARN_ON(free_slots(ring) < TX_SLOTS_PER_FRAME)) {
		/* If we get here, we have a real error with the queue
		 * full, but queues not stopped. */
		b43err(dev->wl, "DMA queue overflow\n");
		err = -ENOSPC;
		goto out;
	}

	/* Assign the queue number to the ring (if not already done before)
	 * so TX status handling can use it. The queue to ring mapping is
	 * static, so we don't need to store it per frame. */
	ring->queue_prio = skb_get_queue_mapping(skb);

	err = dma_tx_fragment(ring, skb);
	if (unlikely(err == -ENOKEY)) {
		/* Drop this packet, as we don't have the encryption key
		 * anymore and must not transmit it unencrypted. */
		ieee80211_free_txskb(dev->wl->hw, skb);
		err = 0;
		goto out;
	}
	if (unlikely(err)) {
		b43err(dev->wl, "DMA tx mapping failure\n");
		goto out;
	}
	if ((free_slots(ring) < TX_SLOTS_PER_FRAME) ||
	    should_inject_overflow(ring)) {
		/* This TX ring is full. */
		unsigned int skb_mapping = skb_get_queue_mapping(skb);
		b43_stop_queue(dev, skb_mapping);
		dev->wl->tx_queue_stopped[skb_mapping] = true;
		ring->stopped = true;
		if (b43_debug(dev, B43_DBG_DMAVERBOSE)) {
			b43dbg(dev->wl, "Stopped TX ring %d\n", ring->index);
		}
	}
out:

	return err;
}

/*
 * the AC TX header of the frame whose header slot is `slot`, from the
 * ring's header cache (dma_tx_fragment keeps one per frame), behind the
 * TOE prefix.
 */
static const struct b43_txhdr_ac *b43_dma_ac_txhdr(struct b43_dmaring *ring,
						   int slot)
{
	const u8 *hdr = &ring->txhdr_cache[(slot / TX_SLOTS_PER_FRAME) *
					   b43_txhdr_size(ring->dev)];

	return (const struct b43_txhdr_ac *)
		(hdr + (b43_txhdr_has_toe(ring->dev) ? B43_TXH_AC_TOE_LEN : 0));
}

static atomic_t b43_ampdutx_st_logged = ATOMIC_INIT(0);
static bool b43_ampdutx_fallback_logged;

/*
 * an A-MPDU TX status (acphy_ampdu bit 1) releases status->ampdu_cnt
 * MPDUs from the FIFO in order, one getnexttxp each (wlc_ampdu_dotxstatus
 * 0xe45c..0xe464, loop 0xe6d0..0xee2c), each acked by its bit in the
 * block ack map (t2 bits 0..31 for MPDU 0..31 at 0xe868..0xe898, t3 for
 * 32..63 at 0xe8a0..0xe8d8). wl decides "A-MPDU" from the TX header of
 * the packet at the head of the FIFO (wlc_txh_get_isAMPDU, 0x9880), not
 * from the status; so does this, from the header cache.
 *
 * Returns true and the number of frames to release when the ring holds
 * exactly that many consecutive aggregate-able frames of one epoch from
 * `slot` on. Otherwise false: the frame completes with its own status, as
 * before. A wrong walk would free the wrong slots and hang the ring, so a
 * count that does not match what is queued falls back to one frame per
 * status (logged once, AMPDUTX): the remaining MPDUs of the aggregate then
 * complete on later statuses, which at worst reports them late.
 */
static bool b43_dma_ampdu_status(struct b43_dmaring *ring, int slot,
				 const struct b43_txstatus *status,
				 int *nframes)
{
	struct b43_wldev *dev = ring->dev;
	struct b43_dmadesc_meta *meta;
	const struct b43_txhdr_ac *hdr;
	const u8 *toe;
	const char *why = NULL;
	unsigned int n = status->ampdu_cnt, i;
	u8 epoch = 0;
	int s = slot;
	bool mimo;

	if (!b43_ac_ampdu(dev, B43_AC_AMPDU_TX) || !b43_txhdr_has_toe(dev))
		return false;
	hdr = b43_dma_ac_txhdr(ring, slot);
	if (!(hdr->mac_ctl_lo & cpu_to_le16(B43_TXH_AC_MAC_AMPDU)))
		return false;
	/* the first acphy_mimo_log statuses of 2+ stream aggregates */
	mimo = b43_ac_mimo_log_hdr(dev, hdr);
	/* wlc_dotxstatus frees one packet for a count of 0 (0x93f4..0x9400) */
	if (n == 0)
		n = 1;
	/* acphy_mimo_unfl: this FIFO's TX underflow counter per aggregate */
	b43_ac_unfl_status(dev, status, hdr, n);
	if (n > 64)
		why = "count above 64";
	else if (n * TX_SLOTS_PER_FRAME > ring->used_slots)
		why = "count above the frames in the ring";
	for (i = 0; !why && i < n; i++) {
		if (s % TX_SLOTS_PER_FRAME) {
			why = "odd header slot";
			break;
		}
		ring->ops->idx2desc(ring, s, &meta);
		if (meta->skb || meta->is_last_fragment) {
			why = "header slot carries a frame";
			break;
		}
		ring->ops->idx2desc(ring, next_slot(ring, s), &meta);
		if (!meta->skb || b43_dma_ptr_is_poisoned(meta->skb) ||
		    !meta->is_last_fragment) {
			why = "frame slot without a frame";
			break;
		}
		hdr = b43_dma_ac_txhdr(ring, s);
		toe = (const u8 *)hdr - B43_TXH_AC_TOE_LEN;
		if (!(hdr->mac_ctl_lo & cpu_to_le16(B43_TXH_AC_MAC_AMPDU))) {
			why = "frame without the AMPDU bit";
			break;
		}
		if (i == 0)
			epoch = toe[2] & B43_TXH_AC_TOE_EPOCH;
		else if ((toe[2] & B43_TXH_AC_TOE_EPOCH) != epoch) {
			why = "frame of another epoch";
			break;
		}
		s = next_slot(ring, next_slot(ring, s));
	}
	/* QUIET: the first-8 log and its atomic skipped; a fallback is always logged */
	if (why || mimo || (!b43_ac_fastpath(dev, B43_AC_FASTPATH_QUIET) &&
			    atomic_inc_return(&b43_ampdutx_st_logged) <= 8))
		/*
		 * the four per-entry words (attempts low byte, ACKs high
		 * byte; with the fixed-rate bit word 0 = attempts, word 1 =
		 * ACKs) instead of their sum.
		 * "nss2" marks a 2+ stream aggregate logged by
		 * acphy_mimo_log.
		 */
	if (why) {
		if (!b43_ampdutx_fallback_logged) {
			b43_ampdutx_fallback_logged = true;
		}
		return false;
	}
	*nframes = n;
	return true;
}

static void b43_dma_ampdu_rc(struct b43_wldev *dev,
			     struct ieee80211_tx_info *info,
			     const struct b43_txstatus *status,
			     int idx, int nframes)
{
	struct ieee80211_tx_rate *r = info->status.rates;
	unsigned int k, mpdus = 0;

	if (nframes < 1)
		return;
	for (k = 0; k < IEEE80211_TX_MAX_RATES && r[k].idx >= 0 && r[k].count;
	     k++) {
		mpdus += r[k].count;
		r[k].count = DIV_ROUND_UP(r[k].count, nframes);
	}
}

static void b43_dma_ampdu_report(struct b43_wldev *dev,
				 struct ieee80211_tx_info *info,
				 const struct b43_txstatus *status,
				 int idx, int nframes)
{
	u64 mask = nframes >= 64 ? ~0ULL : (BIT_ULL(nframes) - 1);
	u64 ba = status->ampdu_ba & mask;
	u64 nack;

	/* The same rule as the per-MPDU ACK in b43_dma_handle_txstatus */
	if (nframes == 1 && status->acked)
		ba |= 1;
	nack = mask & ~ba;
	if (idx == 0) {
		info->flags |= IEEE80211_TX_STAT_AMPDU;
		info->status.ampdu_len = nframes;
		info->status.ampdu_ack_len = hweight64(ba);
	}
	if (nack && idx == fls64(nack) - 1)
		info->flags |= IEEE80211_TX_STAT_AMPDU_NO_BACK;
	/* MPDU counts to rounds, after ampdu_ack_len is known */
	if (b43_ac_ampdu(dev, B43_AC_AMPDU_RCFIX))
		b43_dma_ampdu_rc(dev, info, status, idx, nframes);
}

/*
 * acphy_ampdu bit 10: the status of a session frame that went out
 * plain (a rate probe, or a lone MPDU with bit 4) as minstrel_ht reads it.
 * mac80211 marks every frame of a TID with an operational block ack
 * session IEEE80211_TX_CTL_AMPDU, and minstrel_ht_tx_status drops a status
 * that carries that flag without IEEE80211_TX_STAT_AMPDU ("aggregated but
 * doesn't carry status info"); status.c ieee80211_tx_get_rates and
 * ieee80211_lost_packet do the same. b43 sends every rate probe plain
 * (b43_ac_ampdu_same_burst refuses a probe), so once the session was up no
 * probe ever reached minstrel_ht and the rate stayed where the one or two
 * probes before the session had put it: MCS 0 or 1 in the 2026-10-07
 * 22:55, 23:07 and 23:12 UTC loads (5..9 Mbit/s) and MCS 7 in the 22:31 and
 * 23:10 loads (30..37 Mbit/s), with every probe up to MCS 7 at 20 and 40 MHz
 * acked at the first attempt and every aggregate block-acked in full. The
 * convention of drivers whose hardware aggregates (mt7603 mac.c
 * mt7603_fill_txs, mt76_connac_mac.c) is a one-MPDU aggregate:
 * STAT_AMPDU, ampdu_len 1, ampdu_ack_len = acked, the rate entries' counts
 * as they are (attempts), which minstrel_ht books as attempts = count * 1
 * and success = ampdu_ack_len.
 */
static void b43_dma_plain_report(struct ieee80211_tx_info *info)
{
	if (!(info->flags & IEEE80211_TX_CTL_AMPDU))
		return;
	info->flags |= IEEE80211_TX_STAT_AMPDU;
	info->status.ampdu_len = 1;
	info->status.ampdu_ack_len = !!(info->flags & IEEE80211_TX_STAT_ACK);
}

void b43_dma_handle_txstatus(struct b43_wldev *dev,
			     const struct b43_txstatus *status)
{
	const struct b43_dma_ops *ops;
	struct b43_dmaring *ring;
	struct b43_dmadesc_meta *meta;
	static const struct b43_txstatus fake; /* filled with 0 */
	const struct b43_txstatus *txstat;
	struct b43_txstatus mpdu_stat;
	int slot, firstused;
	bool frame_succeed;
	int skip;
	/* frames this status releases (A-MPDU), index of the current one */
	int nframes = 1, frame = 0;
	bool ampdu = false;
	static u8 err_out1;

	ring = parse_cookie(dev, status->cookie, &slot);
	if (unlikely(!ring))
		return;
	B43_WARN_ON(!ring->tx);

	/* Sanity check: TX packets are processed in-order on one ring.
	 * Check if the slot deduced from the cookie really is the first
	 * used slot. */
	firstused = ring->current_slot - ring->used_slots + 1;
	if (firstused < 0)
		firstused = ring->nr_slots + firstused;

	skip = 0;
	if (unlikely(slot != firstused)) {
		/* This possibly is a firmware bug and will result in
		 * malfunction, memory leaks and/or stall of DMA functionality.
		 */
		if (slot == next_slot(ring, next_slot(ring, firstused))) {
			/* If a single header/data pair was missed, skip over
			 * the first two slots in an attempt to recover.
			 */
			slot = firstused;
			skip = 2;
			if (!err_out1) {
				/* Report the error once. */
				b43dbg(dev->wl,
				       "Skip on DMA ring %d slot %d.\n",
				       ring->index, slot);
				err_out1 = 1;
			}
		} else {
			/* More than a single header/data pair were missed.
			 * Report this error. If running with open-source
			 * firmware, then reset the controller to
			 * revive operation.
			 */
			b43dbg(dev->wl,
			       "Out of order TX status report on DMA ring %d. Expected %d, but got %d\n",
			       ring->index, firstused, slot);
			if (dev->fw.opensource)
				b43_controller_restart(dev, "Out of order TX");
			return;
		}
	}

	ops = ring->ops;
	/* one status for several MPDUs of an A-MPDU (never when skipping) */
	if (!skip)
		ampdu = b43_dma_ampdu_status(ring, slot, status, &nframes);
	/*
	 * acphy_mimo_unfl: a frame sent plain is checked only when the
	 * ucode marked it underflowed (suppress reason 6), no shm read per
	 * frame
	 */
	if (!skip && !ampdu && status->supp_reason == 6)
		b43_ac_unfl_status(dev, status, NULL, 1);
	while (1) {
		B43_WARN_ON(slot < 0 || slot >= ring->nr_slots);
		/* get meta - ignore returned value */
		ops->idx2desc(ring, slot, &meta);

		if (b43_dma_ptr_is_poisoned(meta->skb)) {
			b43dbg(dev->wl, "Poisoned TX slot %d (first=%d) "
			       "on ring %d\n",
			       slot, firstused, ring->index);
			break;
		}

		if (meta->skb) {
			struct b43_private_tx_info *priv_info =
			     b43_get_priv_tx_info(IEEE80211_SKB_CB(meta->skb));

			unmap_descbuffer(ring, meta->dmaaddr,
					 meta->skb->len, 1);
			kfree(priv_info->bouncebuffer);
			priv_info->bouncebuffer = NULL;
		} else {
			unmap_descbuffer(ring, meta->dmaaddr,
					 b43_txhdr_size(dev), 1);
		}

		if (meta->is_last_fragment) {
			struct ieee80211_tx_info *info;

			if (unlikely(!meta->skb)) {
				/* This is a scatter-gather fragment of a frame,
				 * so the skb pointer must not be NULL.
				 */
				b43dbg(dev->wl, "TX status unexpected NULL skb "
				       "at slot %d (first=%d) on ring %d\n",
				       slot, firstused, ring->index);
				break;
			}

			info = IEEE80211_SKB_CB(meta->skb);

			/*
			 * Call back to inform the ieee80211 subsystem about
			 * the status of the transmission. When skipping over
			 * a missed TX status report, use a status structure
			 * filled with zeros to indicate that the frame was not
			 * sent (frame_count 0) and not acknowledged
			 */
			if (unlikely(skip)) {
				txstat = &fake;
			} else if (ampdu) {
				/*
				 * this MPDU's ACK is its bit of the
				 * block ack map (wlc_ampdu_dotxstatus
				 * 0xe868..0xe8d8), not the status bit. A
				 * one-MPDU status also takes the status bit
				 * (s1 bit 15), in case the ucode answers a
				 * lone MPDU's ACK there and not in the map.
				 */
				mpdu_stat = *status;
				mpdu_stat.acked =
					!!(status->ampdu_ba & BIT_ULL(frame)) ||
					(nframes == 1 && status->acked);
				txstat = &mpdu_stat;
			} else {
				txstat = status;
			}

			frame_succeed = b43_fill_txstatus_report(dev, info,
								 txstat);
			if (ampdu) {
				b43_dma_ampdu_report(dev, info, status, frame,
						     nframes);
			} else if (!skip && b43_ac_ampdu(dev, B43_AC_AMPDU_TX) &&
				   (info->flags & IEEE80211_TX_CTL_AMPDU)) {
				/*
				 * the first 16 statuses of session
				 * frames (mac80211's IEEE80211_TX_CTL_AMPDU)
				 * that went out plain (lone, a rate probe, or
				 * acphy_ampdu bit 7 clear): are such ordinary
				 * frames acked while a session is up?
				 * a 2+ stream frame sent plain (a probe,
				 * or acphy_mimo_ampdu=1) counts against
				 * acphy_mimo_log, QUIET or not.
				 */

				/* visible to minstrel_ht as a one-MPDU aggregate */
				if (b43_ac_ampdu(dev, B43_AC_AMPDU_PLAINRC))
					b43_dma_plain_report(info);
			}
#ifdef CPTCFG_B43_DEBUG
			if (frame_succeed)
				ring->nr_succeed_tx_packets++;
			else
				ring->nr_failed_tx_packets++;
			ring->nr_total_packet_tries += status->frame_count;
#endif /* DEBUG */
			ieee80211_tx_status_skb(dev->wl->hw, meta->skb);

			/* skb will be freed by ieee80211_tx_status_skb().
			 * Poison our pointer. */
			meta->skb = B43_DMA_PTR_POISON;
		} else {
			/* No need to call free_descriptor_buffer here, as
			 * this is only the txhdr, which is not allocated.
			 */
			if (unlikely(meta->skb)) {
				b43dbg(dev->wl, "TX status unexpected non-NULL skb "
				       "at slot %d (first=%d) on ring %d\n",
				       slot, firstused, ring->index);
				break;
			}
		}

		/* Everything unmapped and free'd. So it's not used anymore. */
		ring->used_slots--;

		if (meta->is_last_fragment && !skip) {
			/* This is the last scatter-gather
			 * fragment of the frame. We are done, unless this
			 * status covers more frames (A-MPDU). */
			if (++frame >= nframes)
				break;
		}
		slot = next_slot(ring, slot);
		if (skip > 0)
			--skip;
	}
	if (ring->stopped) {
		B43_WARN_ON(free_slots(ring) < TX_SLOTS_PER_FRAME);
		ring->stopped = false;
	}

	if (dev->wl->tx_queue_stopped[ring->queue_prio]) {
		dev->wl->tx_queue_stopped[ring->queue_prio] = false;
		/* TXBATCH: the worker must drain this queue (main.c) */
		dev->ac_txs_kick = true;
	} else {
		/* If the driver queue is running wake the corresponding
		 * mac80211 queue. */
		b43_wake_queue(dev, ring->queue_prio);
		if (b43_debug(dev, B43_DBG_DMAVERBOSE)) {
			b43dbg(dev->wl, "Woke up TX ring %d\n", ring->index);
		}
	}
	/* Add work to the queue. TXBATCH: once per interrupt instead. */
	if (!b43_ac_fastpath(dev, B43_AC_FASTPATH_TXBATCH))
		ieee80211_queue_work(dev->wl->hw, &dev->wl->tx_work);
}

static void dma_rx(struct b43_dmaring *ring, int *slot)
{
	const struct b43_dma_ops *ops = ring->ops;
	struct b43_dmadesc_generic *desc;
	struct b43_dmadesc_meta *meta;
	struct b43_rxhdr_fw4 *rxhdr;
	struct sk_buff *skb;
	u16 len;
	int err;
	dma_addr_t dmaaddr;
	/*
	 * RXSYNC: sync the RX header and the poison bytes now, the frame
	 * once its length is known, and skip the unmap's full-buffer pass. On
	 * this SoC (Cortex-A9, PL310, DMA not coherent) every sync or unmap
	 * of the 4143 byte buffer is 130 L1 lines plus 130 outer-cache
	 * operations; most frames are 1500 bytes or a 100 byte TCP ACK.
	 */
	bool rxsync = b43_ac_fastpath(ring->dev, B43_AC_FASTPATH_RXSYNC);
	size_t hdrsync = ring->frameoffset + sizeof(struct b43_plcp_hdr6) + 2;

	desc = ops->idx2desc(ring, *slot, &meta);

	sync_descbuffer_for_cpu(ring, meta->dmaaddr,
				rxsync ? hdrsync : ring->rx_buffersize);
	skb = meta->skb;

	rxhdr = (struct b43_rxhdr_fw4 *)skb->data;
	len = le16_to_cpu(rxhdr->frame_len);
	if (len == 0) {
		int i = 0;

		do {
			udelay(2);
			barrier();
			/* the header line again: the device may have written it */
			if (rxsync)
				sync_descbuffer_for_cpu(ring, meta->dmaaddr,
							hdrsync);
			len = le16_to_cpu(rxhdr->frame_len);
		} while (len == 0 && i++ < 5);
		if (unlikely(len == 0)) {
			dmaaddr = meta->dmaaddr;
			goto drop_recycle_buffer;
		}
	}
	if (unlikely(b43_rx_buffer_is_poisoned(ring, skb))) {
		/* Something went wrong with the DMA.
		 * The device did not touch the buffer and did not overwrite the poison. */
		b43dbg(ring->dev->wl, "DMA RX: Dropping poisoned buffer.\n");
		dmaaddr = meta->dmaaddr;
		goto drop_recycle_buffer;
	}
	if (unlikely(len + ring->frameoffset > ring->rx_buffersize)) {
		/* The data did not fit into one descriptor buffer
		 * and is split over multiple buffers.
		 * This should never happen, as we try to allocate buffers
		 * big enough. So simply ignore this packet.
		 */
		int cnt = 0;
		s32 tmp = len;

		while (1) {
			desc = ops->idx2desc(ring, *slot, &meta);
			/* recycle the descriptor buffer. */
			b43_poison_rx_buffer(ring, meta->skb);
			sync_descbuffer_for_device(ring, meta->dmaaddr,
						   ring->rx_buffersize);
			*slot = next_slot(ring, *slot);
			cnt++;
			tmp -= ring->rx_buffersize;
			if (tmp <= 0)
				break;
		}
		b43err(ring->dev->wl, "DMA RX buffer too small "
		       "(len: %u, buffer: %u, nr-dropped: %d)\n",
		       len, ring->rx_buffersize, cnt);
		goto drop;
	}

	dmaaddr = meta->dmaaddr;
	err = setup_rx_descbuffer(ring, desc, meta, GFP_ATOMIC);
	if (unlikely(err)) {
		b43dbg(ring->dev->wl, "DMA RX: setup_rx_descbuffer() failed\n");
		goto drop_recycle_buffer;
	}

	if (rxsync) {
		/* the bytes the device wrote; then the unmap without its pass */
		sync_descbuffer_for_cpu(ring, dmaaddr, len + ring->frameoffset);
		dma_unmap_single_attrs(ring->dev->dev->dma_dev, dmaaddr,
				       ring->rx_buffersize, DMA_FROM_DEVICE,
				       DMA_ATTR_SKIP_CPU_SYNC);
	} else {
		unmap_descbuffer(ring, dmaaddr, ring->rx_buffersize, 0);
	}
	skb_put(skb, len + ring->frameoffset);
	skb_pull(skb, ring->frameoffset);

	b43_rx(ring->dev, skb, rxhdr);
drop:
	return;

drop_recycle_buffer:
	/* Poison and recycle the RX buffer. */
	b43_poison_rx_buffer(ring, skb);
	sync_descbuffer_for_device(ring, dmaaddr, ring->rx_buffersize);
}

void b43_dma_handle_rx_overflow(struct b43_dmaring *ring)
{
	int current_slot, previous_slot;

	B43_WARN_ON(ring->tx);

	/* Device has filled all buffers, drop all packets and let TCP
	 * decrease speed.
	 * Decrement RX index by one will let the device to see all slots
	 * as free again
	 */
	/*
	*TODO: How to increase rx_drop in mac80211?
	*/
	current_slot = ring->ops->get_current_rxslot(ring);
	previous_slot = prev_slot(ring, current_slot);
	ring->ops->set_current_rxslot(ring, previous_slot);
}

void b43_dma_rx(struct b43_dmaring *ring)
{
	const struct b43_dma_ops *ops = ring->ops;
	int slot, current_slot;
	int used_slots = 0;
	/*
	 * RXLIST: ieee80211_rx_napi's steps around the whole drain: the
	 * bottom halves off once, every frame through ieee80211_rx_list
	 * (b43_rx, xmit.c), the netif list delivered once. ieee80211_rx_ni
	 * does the same per frame.
	 */
	bool batch = b43_ac_fastpath(ring->dev, B43_AC_FASTPATH_RXLIST);
	LIST_HEAD(list);

	B43_WARN_ON(ring->tx);
	current_slot = ops->get_current_rxslot(ring);
	B43_WARN_ON(!(current_slot >= 0 && current_slot < ring->nr_slots));

	slot = ring->current_slot;
	if (batch) {
		local_bh_disable();
		ring->dev->ac_rx_list = &list;
	}
	for (; slot != current_slot; slot = next_slot(ring, slot)) {
		dma_rx(ring, &slot);
		update_max_used_slots(ring, ++used_slots);
	}
	if (batch) {
		ring->dev->ac_rx_list = NULL;
		netif_receive_skb_list(&list);
		local_bh_enable();
	}
	wmb();
	ops->set_current_rxslot(ring, slot);
	ring->current_slot = slot;
}

static void b43_dma_tx_suspend_ring(struct b43_dmaring *ring)
{
	B43_WARN_ON(!ring->tx);
	ring->ops->tx_suspend(ring);
}

static void b43_dma_tx_resume_ring(struct b43_dmaring *ring)
{
	B43_WARN_ON(!ring->tx);
	ring->ops->tx_resume(ring);
}

void b43_dma_tx_suspend(struct b43_wldev *dev)
{
	b43_power_saving_ctl_bits(dev, B43_PS_AWAKE);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_BK);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_BE);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_VI);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_VO);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_mcast);
}

void b43_dma_tx_resume(struct b43_wldev *dev)
{
	b43_dma_tx_resume_ring(dev->dma.tx_ring_mcast);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_VO);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_VI);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_BE);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_BK);
	b43_power_saving_ctl_bits(dev, 0);
}

static void direct_fifo_rx(struct b43_wldev *dev, enum b43_dmatype type,
			   u16 mmio_base, bool enable)
{
	u32 ctl;

	if (type == B43_DMA_64BIT) {
		ctl = b43_read32(dev, mmio_base + B43_DMA64_RXCTL);
		ctl &= ~B43_DMA64_RXDIRECTFIFO;
		if (enable)
			ctl |= B43_DMA64_RXDIRECTFIFO;
		b43_write32(dev, mmio_base + B43_DMA64_RXCTL, ctl);
	} else {
		ctl = b43_read32(dev, mmio_base + B43_DMA32_RXCTL);
		ctl &= ~B43_DMA32_RXDIRECTFIFO;
		if (enable)
			ctl |= B43_DMA32_RXDIRECTFIFO;
		b43_write32(dev, mmio_base + B43_DMA32_RXCTL, ctl);
	}
}

/* Enable/Disable Direct FIFO Receive Mode (PIO) on a RX engine.
 * This is called from PIO code, so DMA structures are not available. */
void b43_dma_direct_fifo_rx(struct b43_wldev *dev,
			    unsigned int engine_index, bool enable)
{
	enum b43_dmatype type;
	u16 mmio_base;

	type = b43_engine_type(dev);

	mmio_base = b43_dmacontroller_base(type, engine_index);
	direct_fifo_rx(dev, type, mmio_base, enable);
}
