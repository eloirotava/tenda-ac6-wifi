// SPDX-License-Identifier: GPL-2.0-or-later
/*

  Broadcom B43 wireless driver

  Transmission (TX/RX) related functions.

  Copyright (C) 2005 Martin Langer <martin-langer@gmx.de>
  Copyright (C) 2005 Stefano Brivio <stefano.brivio@polimi.it>
  Copyright (C) 2005, 2006 Michael Buesch <m@bues.ch>
  Copyright (C) 2005 Danny van Dyk <kugelfang@gentoo.org>
  Copyright (C) 2005 Andreas Jaggi <andreas.jaggi@waterwave.ch>


*/

#include <linux/module.h>

#include "xmit.h"
#include "phy_common.h"
#include "dma.h"
#include "pio.h"
#include "phy_ac.h"

static const struct b43_tx_legacy_rate_phy_ctl_entry b43_tx_legacy_rate_phy_ctl[] = {
	{ B43_CCK_RATE_1MB,	0x0,			0x0 },
	{ B43_CCK_RATE_2MB,	0x0,			0x1 },
	{ B43_CCK_RATE_5MB,	0x0,			0x2 },
	{ B43_CCK_RATE_11MB,	0x0,			0x3 },
	{ B43_OFDM_RATE_6MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_BPSK },
	{ B43_OFDM_RATE_9MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_BPSK },
	{ B43_OFDM_RATE_12MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_QPSK },
	{ B43_OFDM_RATE_18MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QPSK },
	{ B43_OFDM_RATE_24MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_QAM16 },
	{ B43_OFDM_RATE_36MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QAM16 },
	{ B43_OFDM_RATE_48MB,	B43_TXH_PHY1_CRATE_2_3,	B43_TXH_PHY1_MODUL_QAM64 },
	{ B43_OFDM_RATE_54MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QAM64 },
};

static const struct b43_tx_legacy_rate_phy_ctl_entry *
b43_tx_legacy_rate_phy_ctl_ent(u8 bitrate)
{
	const struct b43_tx_legacy_rate_phy_ctl_entry *e;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(b43_tx_legacy_rate_phy_ctl); i++) {
		e = &(b43_tx_legacy_rate_phy_ctl[i]);
		if (e->bitrate == bitrate)
			return e;
	}

	B43_WARN_ON(1);
	return NULL;
}

/* Extract the bitrate index out of a CCK PLCP header. */
static int b43_plcp_get_bitrate_idx_cck(struct b43_plcp_hdr6 *plcp)
{
	switch (plcp->raw[0]) {
	case 0x0A:
		return 0;
	case 0x14:
		return 1;
	case 0x37:
		return 2;
	case 0x6E:
		return 3;
	}
	return -1;
}

/* Extract the bitrate index out of an OFDM PLCP header. */
static int b43_plcp_get_bitrate_idx_ofdm(struct b43_plcp_hdr6 *plcp, bool ghz5)
{
	/* For 2 GHz band first OFDM rate is at index 4, see main.c */
	int base = ghz5 ? 0 : 4;

	switch (plcp->raw[0] & 0xF) {
	case 0xB:
		return base + 0;
	case 0xF:
		return base + 1;
	case 0xA:
		return base + 2;
	case 0xE:
		return base + 3;
	case 0x9:
		return base + 4;
	case 0xD:
		return base + 5;
	case 0x8:
		return base + 6;
	case 0xC:
		return base + 7;
	}
	return -1;
}

u8 b43_plcp_get_ratecode_cck(const u8 bitrate)
{
	switch (bitrate) {
	case B43_CCK_RATE_1MB:
		return 0x0A;
	case B43_CCK_RATE_2MB:
		return 0x14;
	case B43_CCK_RATE_5MB:
		return 0x37;
	case B43_CCK_RATE_11MB:
		return 0x6E;
	}
	B43_WARN_ON(1);
	return 0;
}

u8 b43_plcp_get_ratecode_ofdm(const u8 bitrate)
{
	switch (bitrate) {
	case B43_OFDM_RATE_6MB:
		return 0xB;
	case B43_OFDM_RATE_9MB:
		return 0xF;
	case B43_OFDM_RATE_12MB:
		return 0xA;
	case B43_OFDM_RATE_18MB:
		return 0xE;
	case B43_OFDM_RATE_24MB:
		return 0x9;
	case B43_OFDM_RATE_36MB:
		return 0xD;
	case B43_OFDM_RATE_48MB:
		return 0x8;
	case B43_OFDM_RATE_54MB:
		return 0xC;
	}
	B43_WARN_ON(1);
	return 0;
}

void b43_generate_plcp_hdr(struct b43_plcp_hdr4 *plcp,
			   const u16 octets, const u8 bitrate)
{
	__u8 *raw = plcp->raw;

	if (b43_is_ofdm_rate(bitrate)) {
		u32 d;

		d = b43_plcp_get_ratecode_ofdm(bitrate);
		B43_WARN_ON(octets & 0xF000);
		d |= (octets << 5);
		plcp->data = cpu_to_le32(d);
	} else {
		u32 plen;

		plen = octets * 16 / bitrate;
		if ((octets * 16 % bitrate) > 0) {
			plen++;
			if ((bitrate == B43_CCK_RATE_11MB)
			    && ((octets * 8 % 11) < 4)) {
				raw[1] = 0x84;
			} else
				raw[1] = 0x04;
		} else
			raw[1] = 0x04;
		plcp->data |= cpu_to_le32(plen << 16);
		raw[0] = b43_plcp_get_ratecode_cck(bitrate);
	}
}

/* TODO: verify if needed for SSLPN or LCN  */
static u16 b43_generate_tx_phy_ctl1(struct b43_wldev *dev, u8 bitrate)
{
	const struct b43_phy *phy = &dev->phy;
	const struct b43_tx_legacy_rate_phy_ctl_entry *e;
	u16 control = 0;
	u16 bw;

	if (phy->type == B43_PHYTYPE_LP)
		bw = B43_TXH_PHY1_BW_20;
	else /* FIXME */
		bw = B43_TXH_PHY1_BW_20;

	if (0) { /* FIXME: MIMO */
	} else if (b43_is_cck_rate(bitrate) && phy->type != B43_PHYTYPE_LP) {
		control = bw;
	} else {
		control = bw;
		e = b43_tx_legacy_rate_phy_ctl_ent(bitrate);
		if (e) {
			control |= e->coding_rate;
			control |= e->modulation;
		}
		control |= B43_TXH_PHY1_MODE_SISO;
	}

	return control;
}

static u8 b43_calc_fallback_rate(u8 bitrate, int gmode)
{
	switch (bitrate) {
	case B43_CCK_RATE_1MB:
		return B43_CCK_RATE_1MB;
	case B43_CCK_RATE_2MB:
		return B43_CCK_RATE_1MB;
	case B43_CCK_RATE_5MB:
		return B43_CCK_RATE_2MB;
	case B43_CCK_RATE_11MB:
		return B43_CCK_RATE_5MB;
	/*
	 * Don't just fallback to CCK; it may be in 5GHz operation
	 * and falling back to CCK won't work out very well.
	 */
	case B43_OFDM_RATE_6MB:
		if (gmode)
			return B43_CCK_RATE_5MB;
		else
			return B43_OFDM_RATE_6MB;
	case B43_OFDM_RATE_9MB:
		return B43_OFDM_RATE_6MB;
	case B43_OFDM_RATE_12MB:
		return B43_OFDM_RATE_9MB;
	case B43_OFDM_RATE_18MB:
		return B43_OFDM_RATE_12MB;
	case B43_OFDM_RATE_24MB:
		return B43_OFDM_RATE_18MB;
	case B43_OFDM_RATE_36MB:
		return B43_OFDM_RATE_24MB;
	case B43_OFDM_RATE_48MB:
		return B43_OFDM_RATE_36MB;
	case B43_OFDM_RATE_54MB:
		return B43_OFDM_RATE_48MB;
	}
	B43_WARN_ON(1);
	return 0;
}

/*
 * Legacy rate tables of wl.ko, in the order wlc_acphy_txctl2_calc searches
 * them (0x114a60..0x114ac0): `cck_rates` (.rodata 0xd7584: 4 rates, 2 4 11 22)
 * and `ofdm_rates` (.rodata 0xd746c: 8 rates, 12 18 24 36 48 72 96 108, basic
 * bit 0x80 masked off). Both are in 500 kbit/s, the unit of b43's hw_value.
 */
static const u8 b43_ac_cck_rates[] = {
	B43_CCK_RATE_1MB, B43_CCK_RATE_2MB, B43_CCK_RATE_5MB, B43_CCK_RATE_11MB,
};

static const u8 b43_ac_ofdm_rates[] = {
	B43_OFDM_RATE_6MB, B43_OFDM_RATE_9MB, B43_OFDM_RATE_12MB,
	B43_OFDM_RATE_18MB, B43_OFDM_RATE_24MB, B43_OFDM_RATE_36MB,
	B43_OFDM_RATE_48MB, B43_OFDM_RATE_54MB,
};

/*
 * TX core mask for the AC header and the ucode 907 core mask block.
 *
 * wl keeps one mask per rate class at stf+22+2i (i = 0 CCK, 1 OFDM, 2 + n
 * for n + 1 spatial streams, wlc_stf_txcore_get 0x191078..0x191100).
 * wlc_stf_init_txcore_default (0x190660) fills it from .rodata 0xd8060
 * (`txcore_default`, rows of three bytes {01 01 02} {01 01 02} {01 01 02}
 * {02 03 06} {03 07 07} {04 0f 0f}); with txchain 7 it takes the second
 * byte of each row (stf+20 = 1, 0x19066c..0x19067c), so CCK, OFDM and one
 * stream use core 0 (0x1), two streams 0x3, three 0x7.
 * wlc_stf_spatialpolicy_set_complete can widen the legacy entries to the
 * whole txchain (0x1918a8..0x1918c0) and drops a mask with more cores than
 * txstreams (0x19193c..0x191978); every path leaves a non-zero mask.
 *
 * acphy_txcore overrides the legacy mask (1 = core 0 as txcore_default,
 * 7 = all three cores as the spatial expansion policy); it is limited to
 * the SPROM txchain when that is set.
 */
static int b43_modparam_ac_txcore = 0x1;
module_param_named(acphy_txcore, b43_modparam_ac_txcore, int, 0444);
MODULE_PARM_DESC(acphy_txcore,
		 "AC-PHY TX core mask for legacy rates (default 1 = core 0)");

u8 b43_ac_txcore(struct b43_wldev *dev, unsigned int nss)
{
	u8 chain = dev->dev->bus_sprom->txchain & 0xF;
	u8 mask;

	if (!chain)
		chain = 0x7;	/* 3 cores, b43_phy_ac reads PHY 0x00b & 7 */
	if (nss <= 1)
		mask = b43_modparam_ac_txcore & 0xF;
	else if (nss == 2)
		mask = 0x3;
	else
		mask = 0x7;
	mask &= chain;
	if (!mask)
		mask = chain & -chain;	/* lowest present core, never 0 */
	return mask;
}

/*
 * wlc_stf_txcore_shmem_write (0x19118c) for PHY type 11, core rev >= 40:
 * four words, i = 0..3 of stf+22+2i (CCK, OFDM, 1 and 2 space-time
 * streams; 0x1911f0..0x191248), mask & 0xf with no spatial-map byte (that
 * is PHY type 7 only, 0x19120c..0x191224). Three streams have no word: the
 * frame's own PHY control 0 carries 0x7. The 0x5de word after it is for
 * vendor 0x106b boards only (0x19124c..0x191290).
 */
static int b43_ac_mimo_hdb[2];	/* acphy_mimo_hdb, defined below */

/*
 * Where the block lives depends on the ucode generation, not on the core
 * rev alone: wl 7.x (ucode 907, core rev 40/42) uses word base 0x2ea (byte
 * 0x5d4). wl 9.10.178.27 of the Tenda AC6 (ucode56, core rev 56) sets the
 * word base 0x1ca (byte 0x394) for core rev > 39 in wlc_attach
 * (0x1322c8..0x1322cc, wlc->pub+0xe0), and ac24initvals56 seeds it with
 * 0x0001 0x0001 0x0303 0x0003. On ucode56, bytes 0x5d4..0x5da are not the
 * core mask: initvals put 0x0000 0x0000 0x08ad 0x0000 there, inside a
 * block that runs from 0x5c0 (0x08a9, 0x01cd, 0x003c, ...).
 */
static u16 b43_ac_txcore_shm_base(struct b43_wldev *dev)
{
	return dev->dev->core_rev >= 56 ? B43_SHM_SH_AC_TXCORE_REV56 :
					  B43_SHM_SH_AC_TXCORE;
}

void b43_ac_txcore_shm_write(struct b43_wldev *dev)
{
	static const u8 nss_of_word[B43_AC_TXCORE_NWORDS] = { 1, 1, 1, 2 };
	u16 w[B43_AC_TXCORE_NWORDS];
	u16 base = b43_ac_txcore_shm_base(dev);
	unsigned int i;

	if (!b43_txhdr_is_ac(dev) || dev->dev->core_rev < 40)
		return;
	for (i = 0; i < B43_AC_TXCORE_NWORDS; i++) {
		w[i] = b43_ac_txcore(dev, nss_of_word[i]) & 0xF;
		b43_shm_write16(dev, B43_SHM_SHARED, base + 2 * i, w[i]);
	}
	/* the per-rate core table as written, for MIMO trials */
}

/*
 * HT on the AC-PHY: spatial streams offered (rx_mask in the band's ht_cap,
 * TX MCS limit). wl runs this 3x3 board (txchain = rxchain = 7) at up to
 * three streams; the default is two streams (MCS 0..15), the set verified
 * on the air. 0 turns HT off (legacy only, as before).
 */
static int b43_modparam_ac_htnss = 2;
module_param_named(acphy_htnss, b43_modparam_ac_htnss, int, 0444);
MODULE_PARM_DESC(acphy_htnss,
		 "AC-PHY 802.11n spatial streams, 0 = HT off, 1..3 (default 2)");

/*
 * VHT (802.11ac) on the AC-PHY: VHT capabilities (main.c),
 * VHT rate entries in the TX header and VHT rate reporting on RX. Needs HT
 * (acphy_htnss >= 1); the streams are acphy_htnss's. The 80 MHz width also
 * needs acphy_bw40 and acphy_bw80 (phy_ac.c). Untested on the radio.
 */
bool b43_modparam_ac_vht;
module_param_named(acphy_vht, b43_modparam_ac_vht, bool, 0444);
MODULE_PARM_DESC(acphy_vht,
		 "AC-PHY 802.11ac VHT: capabilities, VHT TX rate entries, VHT RX reporting (default 0 = off)");

/* Cores in a 4 bit chain mask; 0 (not in the SPROM) counts as 3 cores,
 * as b43_ac_txcore treats it. Open-coded: hweight8 imports __sw_hweight8. */
static unsigned int b43_ac_chain_cores(u8 chain)
{
	unsigned int i, n = 0;

	chain &= 0xF;
	if (!chain)
		chain = 0x7;
	for (i = 0; i < 4; i++)
		n += (chain >> i) & 1;
	return n;
}

unsigned int b43_ac_ht_nss(struct b43_wldev *dev)
{
	unsigned int nss;

	if (dev->phy.type != B43_PHYTYPE_AC || b43_modparam_ac_htnss <= 0)
		return 0;
	nss = min3((unsigned int)b43_modparam_ac_htnss,
		   b43_ac_chain_cores(dev->dev->bus_sprom->txchain),
		   b43_ac_chain_cores(dev->dev->bus_sprom->rxchain));
	return min(nss, 3U);
}

/*
 * TX STBC for single-stream MCS rates.
 * wl advertises TX STBC once it has more than one TX stream and sets the
 * band's STBC TX mode to auto (wlc_stf_attach 0x192e94..0x192f04: band
 * byte 0x4d = -1, HT cap |= 0x0080); wlc_d11hdrs then sends every MCS 0..7
 * (and MCS 32) rate as STBC to a peer that advertises RX STBC (0x11d108..
 * 0x11d1a4). An HT STBC rate has nss + 1 space-time streams
 * (wlc_ratespec_nsts 0x180e0c..0x180e18), so it takes the two-stream core
 * mask (0x3). Off by default: b43 has never sent on cores 1 and 2.
 * mac80211 sets IEEE80211_TX_CTL_STBC only for a peer whose RX STBC
 * survives our TX STBC capability (ht.c), and minstrel_ht sets it on every
 * rate; it is applied to single-stream rates only, as wl does.
 */
static int b43_modparam_ac_htstbc;
module_param_named(acphy_htstbc, b43_modparam_ac_htstbc, int, 0444);
MODULE_PARM_DESC(acphy_htstbc,
		 "AC-PHY: advertise and send HT TX STBC on MCS 0..7 (needs 2 TX cores; default 0)");

bool b43_ac_ht_tx_stbc(struct b43_wldev *dev)
{
	return b43_modparam_ac_htstbc > 0 && b43_ac_ht_nss(dev) &&
	       b43_ac_chain_cores(dev->dev->bus_sprom->txchain) > 1;
}

/*
 * Per-frame TX power offset for an HT frame sent on more than one core,
 * half-dB in PHY control 1 from bit 3, added to the
 * per-rate offset.
 *
 * wl takes it per rate from the offset ppr at stf+0x2c, indexed by
 * bandwidth, spatial streams, TX mode (STBC, CDD for one stream on more
 * cores, none) and the number of cores in the frame's core mask
 * (wlc_stf_get_204080_pwrs 0x193990..0x193bd0; wlc_d11hdrs puts the byte
 * << 3 into PHY control 1 at 0x11c8c8..0x11c8f0, wlc_acphy_txctl1_calc
 * 0x114254). The board part of that ppr is the same for every core count
 * (wlc_phy_ppr_set_mcs 0x529c4 copies one MCS array to 1, 2 and 3 cores);
 * a lower per-core power for more cores can only come from the CLM
 * regulatory table, which is not decoded. b43's TX power target is a
 * per-core limit from mac80211's regulatory power (b43_ac_txpwr), so with
 * every core at that target a 2-core frame would carry 3 dB and a 3-core
 * frame 4.8 dB more in total. Default: back off by 10 log10(cores),
 * rounded up to half-dB (6 = 3.0 dB for 2 cores, 10 = 5.0 dB for 3).
 * Port choice, not wl's value; 0,0 sends at the single-core target.
 * Legacy frames (acphy_txcore) are not changed.
 */
static int b43_ac_mimo_hdb[2] = { 6, 10 };
static int b43_ac_mimo_hdb_n;
module_param_array_named(acphy_mimo_hdb, b43_ac_mimo_hdb, int,
			 &b43_ac_mimo_hdb_n, 0444);
MODULE_PARM_DESC(acphy_mimo_hdb,
		 "AC-PHY HT TX power back-off in half-dB for frames on 2,3 cores (default 6,10)");

static unsigned int b43_ac_mimo_hdb_of(u8 coremask)
{
	unsigned int n = b43_ac_chain_cores(coremask);
	int hdb = n >= 3 ? b43_ac_mimo_hdb[1] : n == 2 ? b43_ac_mimo_hdb[0] : 0;

	/* s4.1 half-dB, 0..31 (wlc_stf_convert_to_s41_hdb 0x19016c) */
	return clamp(hdb, 0, 31);
}

/*
 * 2026-10-08: the 2-stream downlink. With acphy_htnss=2 (ch36, HT40,
 * PS off, hardware CCMP) the router receives MCS 15 at 164 Mbit/s, but its
 * own 2-stream downlink carries 0.68 Mbit/s while the laptop samples
 * "300.0 MBit/s MCS 15" on every read, drops nothing and the router's
 * station dump shows 8 failed of 32198 (bench/r6400-5g/assoc-
 * 20261008T062806Z.log; 3 streams: T063044Z, 0.55). One stream: 107..111.
 * With software CCMP the same 2-stream downlink carries 40.9 Mbit/s, the
 * software-CCMP ceiling (T063457Z), so the aggregates are small there.
 * Static reading of the ucode finds nothing in the
 * header or the ucode's per-frame crypto setup that depends on the stream
 * count; the candidates left are the hardware crypto path at the 2-stream
 * PHY rate and the size of 2-stream aggregates. Three knobs separate them:
 *
 * acphy_mimo_ampdu (0..64, default 0): the aggregate of an HT frame with
 *   2 or more streams at rate entry 0. 0 = as for one stream; 1 = such a
 *   frame is never aggregated (no AMPDU bit, no tail; under a session its
 *   status still reaches minstrel_ht); N = at most N MPDUs per
 *   aggregate (the header tail's max MPDUs, bytes 102/103).
 * acphy_mimo_log (default 0): the first 4 headers (MIMOTX hdr, the 124
 *   bytes) and the first N TX statuses (AMPDUTX nss2 status / plain) of
 *   such frames, logged even with acphy_fastpath bit 0 (QUIET). The
 *   status line carries the MPDU count, the block ack map and the per
 *   entry words, so it says whether the laptop block-acks a 2-stream
 *   aggregate in full and how long the aggregates are.
 * acphy_mimo_word9 (default 0): rate entry word 9 bits 0..5 = the PHY
 *   control 1 power offset, bit 6 = STBC, see b43_ac_ht_entry.
 *
 * v2 (07:37..07:48 UTC on the air): plain 2-stream keyed frames carry
 * data (acphy_mimo_ampdu=1: 34.0 Mbit/s down, every probe acked), so the
 * defect is in aggregated keyed frames at the 2-stream rate. The two
 * tail words the host owns for an aggregate get their own 2-stream
 * values: acphy_mimo_dur (default 0 = wl's 5416 us; the ucode turns the
 * maximum duration into a byte budget at the entry's rate, twice the
 * bytes at 300 Mbit/s: 2708 gives the 1-stream budget) and
 * acphy_mimo_dens (default -1 = the peer's MPDU density code; 7 = 16 us,
 * the widest spacing the AQM pads between MPDUs, which is the time the
 * per-MPDU key setup has at the 2-stream rate). Both only for frames
 * with MCS 8 and up at rate entry 0.
 */
static int b43_modparam_ac_mimo_dur;
module_param_named(acphy_mimo_dur, b43_modparam_ac_mimo_dur, int, 0644);
MODULE_PARM_DESC(acphy_mimo_dur,
		 "AC-PHY HT TX with 2+ streams: A-MPDU maximum duration in us in the header tail (default 0 = 5416 as for one stream)");

static int b43_modparam_ac_mimo_dens = -1;
module_param_named(acphy_mimo_dens, b43_modparam_ac_mimo_dens, int, 0644);
MODULE_PARM_DESC(acphy_mimo_dens,
		 "AC-PHY HT TX with 2+ streams: A-MPDU MPDU density code 0..7 in the header tail (default -1 = the peer's)");
static int b43_modparam_ac_mimo_ampdu = 16;
module_param_named(acphy_mimo_ampdu, b43_modparam_ac_mimo_ampdu, int, 0644);
MODULE_PARM_DESC(acphy_mimo_ampdu,
		 "AC-PHY HT TX with 2+ streams: 0 = aggregate as with one stream, 1 = never aggregate, N = at most N MPDUs per aggregate (default 16)");

static int b43_modparam_ac_mimo_log;
module_param_named(acphy_mimo_log, b43_modparam_ac_mimo_log, int, 0644);
MODULE_PARM_DESC(acphy_mimo_log,
		 "AC-PHY: log the first N TX statuses and 4 TX headers of HT frames with 2+ streams, also with acphy_fastpath bit 0 (default 0)");

static int b43_modparam_ac_mimo_word9;
module_param_named(acphy_mimo_word9, b43_modparam_ac_mimo_word9, int, 0444);
MODULE_PARM_DESC(acphy_mimo_word9,
		 "AC-PHY: 1 = rate entry word 9 = PHY control 1 power offset (bits 0..5) | STBC (bit 6), the place ucode 907 reads them from for a non-beamformed frame (default 0)");

#define B43_AC_MIMO_HDRLOGS	4
static atomic_t b43_ac_mimo_hdrlogs = ATOMIC_INIT(0);
/* v2: aggregates and plain frames each have the acphy_mimo_log budget */
static atomic_t b43_ac_mimo_agglogs = ATOMIC_INIT(0);
static atomic_t b43_ac_mimo_plainlogs = ATOMIC_INIT(0);

/* HT with MCS 8 and up at rate entry 0: 2 or more spatial streams */
static bool b43_ac_mimo_entry_is_nss2(const struct b43_txhdr_ac_rate *e)
{
	return (le16_to_cpu(e->phy_ctl0) & B43_TXH_AC_PHY0_FT) ==
	       B43_TXH_AC_PHY0_FT_HT &&
	       (le16_to_cpu(e->phy_ctl2) & 0x7F) >= 8;
}

static bool b43_ac_mimo_rate_is_nss2(const struct ieee80211_tx_rate *r)
{
	return (r->flags & IEEE80211_TX_RC_MCS) && r->idx >= 8;
}

bool b43_ac_mimo_log_hdr(struct b43_wldev *dev,
			 const struct b43_txhdr_ac *hdr)
{
	return b43_modparam_ac_mimo_log > 0 &&
	       b43_ac_mimo_entry_is_nss2(&hdr->rate[0]) &&
	       atomic_inc_return(&b43_ac_mimo_agglogs) <=
	       b43_modparam_ac_mimo_log;
}

bool b43_ac_mimo_log_info(struct b43_wldev *dev,
			  const struct ieee80211_tx_info *info)
{
	return b43_modparam_ac_mimo_log > 0 &&
	       b43_ac_mimo_rate_is_nss2(&info->status.rates[0]) &&
	       atomic_inc_return(&b43_ac_mimo_plainlogs) <=
	       b43_modparam_ac_mimo_log;
}

/*
 * 2026-10-08: the 2-stream aggregate length. On the air (ch36, HT40,
 * PS off, hardware CCMP, the router at MCS 8..15) acphy_mimo_ampdu=N gave
 * 34 / 71 / 105 / 148 / 151 Mbit/s down for N = 1 / 4 / 8 / 16 / 20, then
 * 2.4 for 24 and 0.6..1.3 uncapped (32); one stream (MCS 7, 150 Mbit/s)
 * carries 32-MPDU aggregates at 111. The broken loads report tx failed ~0:
 * the long aggregates are block-acked, yet their data never arrives.
 *
 * Two mechanisms from the open driver for the earlier d11 cores fit "breaks
 * past a length that depends on the PHY rate", and this patch gives each a
 * knob, both off by default:
 *
 * acphy_mimo_ffpld (Mbit/s, default 0 = off): brcmsmac's FIFO preload
 *   model (brcmsmac/ampdu.c, "ffpld"). It assumes the TX FIFO is filled by
 *   DMA at dmaxferrate while the PHY drains it at the PHY rate, after a
 *   preload of ampdu_pld_size bytes; an aggregate whose deficit outruns the
 *   preload underflows. brcms_c_ffpld_calc_mcs2ampdu_table (ampdu.c:312)
 *   turns that into a per-MCS MPDU limit (ampdu.c:332..340):
 *     mcs2ampdu[mcs] = pld * phy / ((phy - dma) * FFPLD_MPDU_SIZE) + 1
 *   for phy > dma, FFPLD_MPDU_SIZE = 1800 (ampdu.c:56), and
 *   brcms_c_sendampdu applies it only to rates at or above dmaxferrate
 *   (ampdu.c:570..575). There it is adaptive: brcms_c_ffpld_check_txfunfl
 *   (ampdu.c:350) reads the ucode's txfunfl[fifo] counter on a suppressed
 *   status, raises the preload by 1000 bytes per 10 underflows and, at the
 *   FIFO size less 2048 bytes (ampdu.c:380..383), lowers the limit at the
 *   top MCS and recomputes the table. wl 7.x on core rev 42 has none of
 *   this: rev > 39 aggregates in hardware (_wlc_sendampdu_aqm), wl.ko has
 *   no ffpld/mcs2ampdu/dmaxferrate symbol or string, and its MPDU limit is
 *   per peer stream count, not per rate (scb_ampdu_update_config 0x140b18:
 *   32 / 48 / 64 for 1 / 2 / 3 stream peers from wlc_ampdu_tx_attach
 *   0x143374..0x143398, 16 only for a Broadcom peer without the large
 *   aggregate flag, 0x140b6c..0x140ba4). So this is a static table with
 *   the formula and our measured numbers: acphy_mimo_ffpld = the fill rate,
 *   acphy_mimo_pld (bytes, default 16384) = the preload. 150 / 16384 gives
 *   19 MPDUs at 300 Mbit/s (MCS 15 SGI), 21 at 270, 25 at 240, 55 (so the
 *   32 cap) at 180 and no limit at 150 and below: it keeps one stream as
 *   it is and caps only the rates that broke. The rate is rate entry 0's
 *   own (b43_ac_ht_rate500 / b43_ac_vht_rate500 x 500 kbit/s), any HT or
 *   VHT aggregate.
 * acphy_mimo_unfl (default 0 = off): N > 0 logs up to N TX underflow
 *   events. ucode 907 counts a TX FIFO underflow in shm word 0x76 + FIFO
 *   (byte 0xEC + 2 * FIFO, L759 @0xD5B..0xD5D: off5 = 0x76 + [0x18]) and
 *   sets the frame's suppress reason to 6 when none is set yet (@0xD5F..
 *   0xD60, s1 bits 4..7: brcmsmac's TX_STATUS_SUPR_UF, d11.h). That block
 *   is wl's macstat: wlc_statsupd 0x117768 copies 128 bytes from shm 0xE0
 *   (0x117790..0x1177a8) and adds words 6..11 (0x1178e4..0x11793c) to its
 *   six txfunfl counters, and the ucode counts txallfrm..txbcnfrm in words
 *   0x70..0x75. With this knob every A-MPDU status (and every status with
 *   suppress reason 6) reads the counter of its FIFO and logs "TXUNFL" when
 *   it moved or the reason is 6, with the aggregate's count, map, rate and
 *   limit. An underflow mid-aggregate is the ffpld model's event; no
 *   TXUNFL line in a broken uncapped load rules the FIFO out, and the
 *   corruption is then past the FIFO (the MPDUs are block-acked, so their
 *   FCS is good on the air: their payload, MIC or PN is not).
 */
static unsigned int b43_modparam_ac_mimo_ffpld = 50;
module_param_named(acphy_mimo_ffpld, b43_modparam_ac_mimo_ffpld, uint, 0644);
MODULE_PARM_DESC(acphy_mimo_ffpld,
		 "AC-PHY A-MPDU TX: FIFO fill rate in Mbit/s for brcmsmac's per-rate MPDU limit, applied to rates above it (default 50)");

static unsigned int b43_modparam_ac_mimo_pld = 16384;
module_param_named(acphy_mimo_pld, b43_modparam_ac_mimo_pld, uint, 0644);
MODULE_PARM_DESC(acphy_mimo_pld,
		 "AC-PHY A-MPDU TX: FIFO preload in bytes for acphy_mimo_ffpld (default 16384)");

static int b43_modparam_ac_mimo_unfl;
module_param_named(acphy_mimo_unfl, b43_modparam_ac_mimo_unfl, int, 0644);
MODULE_PARM_DESC(acphy_mimo_unfl,
		 "AC-PHY: log up to N TX FIFO underflow events (ucode txfunfl counter, suppress reason 6) with the A-MPDU status (default 0)");

#define B43_AC_FFPLD_MPDU_SIZE		1800	/* brcmsmac FFPLD_MPDU_SIZE */
#define B43_AC_SHM_TXFUNFL		0x00EC	/* macstat 0xE0 + 12, 6 FIFOs */
#define B43_AC_TXFUNFL_NFIFO		6
#define B43_TXST40_SUPR_UF		6	/* ucode 907 @0xD60 */
#define B43_AC_FFPLD_LOGS		16

static atomic_t b43_ac_unfl_logs = ATOMIC_INIT(0);
/*
 * acphy_mimo_ffpld: the MPDU limit for an aggregate at rate entry 0,
 * brcms_c_ffpld_calc_mcs2ampdu_table's formula (brcmsmac/ampdu.c:332..340)
 * in kbit/s, at most 255 as there; 0 = no limit.
 */
static unsigned int b43_ac_ffpld_cap(const struct b43_txhdr_ac_rate *e)
{
	u64 phy = (u64)le16_to_cpu(e->rate) * 500;
	u64 dma = (u64)b43_modparam_ac_mimo_ffpld * 1000;
	u64 n;

	if (!b43_modparam_ac_mimo_ffpld || phy <= dma)
		return 0;
	n = div64_u64((u64)b43_modparam_ac_mimo_pld * phy,
		      (phy - dma) * B43_AC_FFPLD_MPDU_SIZE) + 1;
	return min_t(u64, n, 255);
}

/*
 * acphy_mimo_unfl, from the TX status path (dma.c): `hdr` is the
 * header of the first MPDU the status releases (NULL when unknown),
 * `nframes` the MPDUs it releases. Reads FIFO's txfunfl word, keeps it
 * per FIFO (dev->ac_ampdu_epoch[fifo].txfunfl, zeroed with the core init
 * as the ucode's shm) and logs a moved counter or a suppress reason 6.
 */
void b43_ac_unfl_status(struct b43_wldev *dev,
			const struct b43_txstatus *status,
			const struct b43_txhdr_ac *hdr, unsigned int nframes)
{
	unsigned int fifo = status->cookie & B43_TXH_AC_FID_FIFO;
	struct b43_ac_ampdu_epoch *e;
	u16 cur, delta;

	if (b43_modparam_ac_mimo_unfl <= 0 || dev->dev->core_rev < 40 ||
	    fifo >= B43_AC_TXFUNFL_NFIFO)
		return;
	e = &dev->ac_ampdu_epoch[fifo];
	cur = b43_shm_read16(dev, B43_SHM_SHARED,
			     B43_AC_SHM_TXFUNFL + 2 * fifo);
	/* the ucode's counter starts over when its shm is cleared */
	delta = cur >= e->txfunfl ? cur - e->txfunfl : cur;
	e->txfunfl = cur;
	if (!delta && status->supp_reason != B43_TXST40_SUPR_UF)
		return;
	if (atomic_inc_return(&b43_ac_unfl_logs) > b43_modparam_ac_mimo_unfl)
		return;
}

/*
 * wl's per-MCS modulation and coding (.rodata 0xd73b4, `wlc_mcs_info`,
 * read by wlc_rate_mcs2rate 0x180e40): bits per subcarrier, code rate
 * numerator, code rate denominator, for MCS 0..7 of each stream and the
 * VHT MCS 8 and 9 (256-QAM 3/4 and 5/6).
 */
static const u8 b43_ac_mcs_info[10][3] = {
	{ 1, 1, 2 }, { 2, 1, 2 }, { 2, 3, 4 }, { 4, 1, 2 },
	{ 4, 3, 4 }, { 6, 2, 3 }, { 6, 3, 4 }, { 6, 5, 6 },
	{ 8, 3, 4 }, { 8, 5, 6 },
};

/*
 * Rate of a VHT MCS in 500 kbit/s (wlc_rate_rspec2rate 0x180f50..0x180f68
 * -> wlc_rate_mcs2rate): data subcarriers 52 / 108 / 234 at 20 / 40 /
 * 80 MHz (0x180e6c..0x180e9c), short GI as for HT. MCS 9, one stream,
 * 80 MHz: 780, or 866 with short GI.
 */
static u16 b43_ac_vht_rate500(unsigned int mcs, unsigned int nss,
			      unsigned int sc, bool sgi)
{
	const u8 *m = b43_ac_mcs_info[mcs];
	unsigned int kbps;

	kbps = 250 * (m[0] * m[1] * nss * sc / m[2]);
	if (sgi)
		kbps = (kbps * 10 + 4) / 9;
	return kbps / 500;
}

/*
 * Rate of an HT20 or HT40 MCS in 500 kbit/s, as wl stores it in the rate
 * entry (wlc_rate_rspec2rate(rspec) / 500 at 0x11c330..0x11c360).
 * wlc_rate_mcs2rate (0x180e40..0x180eec): kbit/s = 250 * (bits * num * nss *
 * sc / den), sc = 52 data subcarriers at 20 MHz (0x180e74), 108 at 40 MHz
 * (0x180e7c..0x180e84); short GI (kbit/s * 10 + 4) / 9 (0x180ed8..0x180ee8).
 * MCS 7, one stream: 130, or 144 with short GI; at 40 MHz 270 / 300.
 */
static u16 b43_ac_ht_rate500(unsigned int mcs, bool sgi, bool bw40)
{
	const u8 *m = b43_ac_mcs_info[mcs & 7];
	unsigned int nss = mcs / 8 + 1;
	unsigned int kbps;

	kbps = 250 * (m[0] * m[1] * nss * (bw40 ? 108 : 52) / m[2]);
	if (sgi)
		kbps = (kbps * 10 + 4) / 9;
	return kbps / 500;
}

/*
 * The 802.11 Duration of a unicast HT data frame. mac80211 leaves it 0 for
 * MCS rates ("assume HW handles this", tx.c ieee80211_duration); wl writes
 * it into the 802.11 header itself (wlc_d11hdrs 0x11c4b0..0x11c4f0,
 * wlc_compute_frame_dur 0x11afa0): SIFS (16 us on 5 GHz, 10 on 2.4 GHz,
 * 0x11afbc..0x11afc4) plus the ACK time at the highest basic rate not above
 * the MCS's reference rate (wlc_calc_ack_time 0x11af6c,
 * wlc_rate_ht_basic_reference 0x180f80, table .rodata 0xd73d4: 6, 12, 18,
 * 24, 36, 48, 54, 54 Mbit/s for MCS & 7). OFDM frame time 20 + 4 *
 * ceil((22 + 8 * len) / (2 * rate500)) us, ACK len 14 (wlc_calc_frame_time
 * 0x11af18..0x11af3c, 0x11af94). Only the single-fragment case: a frame
 * with More Fragments or a non-zero Duration is left alone.
 */
static const u8 b43_ac_ht_ref_rate[10] = {
	B43_OFDM_RATE_6MB, B43_OFDM_RATE_12MB, B43_OFDM_RATE_18MB,
	B43_OFDM_RATE_24MB, B43_OFDM_RATE_36MB, B43_OFDM_RATE_48MB,
	B43_OFDM_RATE_54MB, B43_OFDM_RATE_54MB,
	/*
	 * VHT MCS 8 and 9: basic_ref_rate (.rodata 0xd73d4) entries 8 and 9,
	 * read by wlc_rate_vht_basic_reference 0x180fdc (index MCS & 15, 0
	 * above 9) and wlc_rate_rspec_reference_rate 0x180fa0
	 */
	B43_OFDM_RATE_54MB, B43_OFDM_RATE_54MB,
};

/* ref: MCS & 7 for HT (wlc_rate_ht_basic_reference 0x180f80), MCS for VHT */
/*
 * PHY control 1 power bits for a frame: the per-rate offset phy_ac.c keeps
 * for slot (acphy_rateppr, acphy_rateppr_htadd; 0 by default), as
 * wlc_d11hdrs passes the wlc_stf_get_204080_pwrs byte to
 * wlc_acphy_txctl1_calc (0x11c8c8..0x11c8f0).
 */
static u16 b43_ac_phyctl1_pwr(struct b43_wldev *dev, unsigned int slot,
			      bool ht)
{
#ifdef CPTCFG_B43_PHY_AC
	if (dev->phy.type == B43_PHYTYPE_AC)
		return (u16)b43_phy_ac_txpwroff(dev, slot, ht) <<
		       B43_TXH_AC_PHY1_PWR_SHIFT;
#endif
	return 0;
}

static void b43_ac_ht_set_duration(struct b43_wldev *dev,
				   struct ieee80211_tx_info *info,
				   struct ieee80211_hdr *hdr, unsigned int ref_idx)
{
	struct ieee80211_supported_band *sband;
	struct ieee80211_vif *vif = dev->wl->vif;
	u8 ref = b43_ac_ht_ref_rate[min(ref_idx, 9U)];
	u8 ack = B43_OFDM_RATE_6MB;
	unsigned int i, us;

	if (info->flags & IEEE80211_TX_CTL_NO_ACK)
		return;
	if (hdr->duration_id ||
	    ieee80211_has_morefrags(hdr->frame_control) ||
	    is_multicast_ether_addr(hdr->addr1))
		return;

	sband = dev->wl->hw->wiphy->bands[info->band];
	if (vif && sband) {
		for (i = 0; i < sband->n_bitrates; i++) {
			u8 r = sband->bitrates[i].hw_value;

			if ((vif->bss_conf.basic_rates & BIT(i)) &&
			    b43_is_ofdm_rate(r) && r <= ref && r > ack)
				ack = r;
		}
	}
	/* r is in 500 kbit/s; data bits per 4 us symbol = 2 * r */
	us = 20 + 4 * DIV_ROUND_UP(22 + 8 * 14, 2 * ack);
	us += (info->band == NL80211_BAND_5GHZ) ? 16 : 10;
	hdr->duration_id = cpu_to_le16(us);
}

/*
 * Fill rate entry 0 for an HT MCS rate selected by mac80211 (minstrel_ht),
 * as wl fills it for an HT rspec. 20 MHz, or
 * 40 MHz when mac80211 asks for it on a 40 MHz chanspec (acphy_bw40);
 * mixed-format preamble, one rate entry, STBC only with acphy_htstbc, no
 * LDPC (not advertised), no beamforming.
 */
static int b43_ac_ht_entry(struct b43_wldev *dev,
			   struct b43_txhdr_ac_rate *entry,
			   const struct ieee80211_tx_rate *r,
			   unsigned int frame_len, u16 chanspec, bool stbc)
{
	unsigned int nss_max = b43_ac_ht_nss(dev);
	bool sgi = !!(r->flags & IEEE80211_TX_RC_SHORT_GI);
	bool bw40 = !!(r->flags & IEEE80211_TX_RC_40_MHZ_WIDTH);
	unsigned int mcs, nss, nsts, pwr;
	u16 phy_ctl0;
	u8 coremask;
	u8 *plcp = entry->plcp.raw;

	/*
	 * A 40 MHz rate only on a 40 or 80 MHz chanspec: wl caps the rspec
	 * bandwidth at the chanspec's (0x11bba0..0x11bbc0, 0x11bc08..0x11bc1c).
	 * The 20 MHz duplicate format has no AC entry encoding here.
	 */
	if (unlikely(r->idx < 0 || r->idx >= 8 * nss_max ||
		     (bw40 && !b43_ac_chanspec_is40(chanspec) &&
		      !b43_ac_chanspec_is80(chanspec)) ||
		     (r->flags & IEEE80211_TX_RC_DUP_DATA))) {
		b43dbg(dev->wl, "AC TX: HT rate idx %d flags 0x%x chanspec 0x%04x refused\n",
		       r->idx, r->flags, chanspec);
		return -EINVAL;
	}
	mcs = r->idx;
	nss = mcs / 8 + 1;
	/*
	 * STBC on single-stream rates only (wl: MCS 0..7 or 32, 0x11d148..
	 * 0x11d1a4); nsts = nss + 1 (wlc_ratespec_nsts 0x180e0c..0x180e18)
	 * picks the core mask, as wlc_stf_txcore_get indexes by nsts + 1.
	 */
	stbc = stbc && nss == 1 && b43_ac_ht_tx_stbc(dev);
	nsts = nss + (stbc ? 1 : 0);
	coremask = b43_ac_txcore(dev, nsts);

	/*
	 * PHY control 0 (wlc_acphy_txctl0_calc): frame type 2 for an HT rspec
	 * (0x1135b8..0x1135c0), bit 2 (0x113660), TX core mask << 6 for the
	 * rspec's streams (wlc_stf_txcore_get index nsts + 1, 0x1910dc..
	 * 0x191100, mask << 6 at 0x191138), bandwidth bits 0 for 20 MHz, 1 for
	 * 40 MHz (0x11364c..0x113654). Bit 4 only for preamble 1 or 2 (0x113600..
	 * 0x113614); wl picks 4 (mixed format) for HT unless the peer and
	 * the band allow greenfield (0x11bdb8..0x11bf10). Greenfield is not
	 * advertised, so b43 always sends mixed format and bit 4 stays clear.
	 */
	phy_ctl0 = B43_TXH_AC_PHY0_FT_HT | B43_TXH_AC_PHY0_SET;
	phy_ctl0 |= ((u16)coremask << B43_TXH_AC_PHY0_ANT_SHIFT) &
		    B43_TXH_AC_PHY0_ANT;
	if (bw40)
		phy_ctl0 |= B43_TXH_AC_PHY0_BW_40;		/* 0x11364c..0x113650 */
	entry->phy_ctl0 = cpu_to_le16(phy_ctl0);
	/*
	 * PHY control 1: per-rate power backoff << 3 from
	 * wlc_stf_get_204080_pwrs (0x11c184), not decoded for MCS rates; zero
	 * as for legacy rates (no backoff). The sideband bits
	 * (wlc_acphy_txctl1_calc @0x114254): the control sideband for a
	 * 20 MHz rate in a 40 or 80 MHz channel; for a 40 MHz rate 0 in a
	 * 40 MHz channel, the 40 MHz half (sb >> 1) in an 80 MHz one.
	 */
	/*
	 * the per-rate offset plus the multi-core back-off acphy_mimo_hdb
	 * for a frame on 2 or 3 cores (b43_ac_mimo_hdb_of), the sum capped at
	 * 31 as wl caps its offsets (0x193ef4..0x193f10).
	 */
	pwr = (b43_ac_phyctl1_pwr(dev, (bw40 ? B43_AC_PWROFF_MCS40 :
					B43_AC_PWROFF_MCS) + (mcs & 7), true) >>
	       B43_TXH_AC_PHY1_PWR_SHIFT) + b43_ac_mimo_hdb_of(coremask);
	entry->phy_ctl1 = cpu_to_le16(b43_ac_phyctl1_sb(chanspec, bw40) |
				      (min(pwr, 31U) << B43_TXH_AC_PHY1_PWR_SHIFT));
	/*
	 * PHY control 2: the MCS byte (0x114a38), bit 6 for STBC (0x114ac4..
	 * 0x114ac8). No stream-count field for HT: the PHY takes it from the
	 * MCS (VHT puts nss - 1 in bits 4..6, 0x114a40..0x114a50).
	 */
	entry->phy_ctl2 = cpu_to_le16(mcs | (stbc ? B43_TXH_AC_PHY2_STBC : 0));
	/*
	 * acphy_mimo_word9: ucode 907 L206/L207 @0x37A..0x37D copies rate
	 * entry word 9 bits 0..5 into PHY control 1 bits 3..8 and bit 6 into
	 * PHY control 2 bit 6 (spr8A, spr8B) for every frame whose
	 * beamforming block ([0x00,off1] bit 8, cleared @0x362 when the
	 * block's address is not the frame's A1) is not valid, i.e. every
	 * frame of this port: the header's own power offset and
	 * STBC bit never reached the PHY. wl writes word 9 only on its
	 * beamforming path (0x11c204..0x11c23c: the pwrs byte | 0x40 for
	 * STBC) and sends non-beamformed frames with word 9 = 0 too. With
	 * this bit the port's offset and STBC bit go where the ucode reads
	 * them. Off by default: a power change on an already working link.
	 */
	if (b43_modparam_ac_mimo_word9)
		entry->txbf = cpu_to_le16((min(pwr, 31U) & 0x3F) |
					  (stbc ? 0x40 : 0));
	/*
	 * HT-SIG, wlc_compute_plcp 0x118ca8..0x118cfc; CBW 40 at
	 * 0x118cb8..0x118cd0; STBC field nsts - nss = 1 (0x118cdc)
	 */
	plcp[0] = mcs | (bw40 ? B43_AC_HTSIG1_CBW40 : 0);
	plcp[1] = frame_len & 0xFF;
	plcp[2] = (frame_len >> 8) & 0xFF;
	plcp[3] = B43_AC_HTSIG2_BASE | (sgi ? B43_AC_HTSIG2_SGI : 0) |
		  (stbc ? B43_AC_HTSIG2_STBC1 : 0);
	plcp[4] = 0;
	plcp[5] = 0;
	/*
	 * No fallback bandwidth. For a 40 MHz HT rate wl writes fb = 1 (a
	 * 20 MHz fallback) only while byte 0x106 of *wlc is set, else 4
	 * (0x11bf20..0x11bf9c); that byte is not traced, so b43 keeps 4 (3
	 * in the word), the state wl has with it clear.
	 */
	entry->fb_bw = cpu_to_le16(B43_TXH_AC_FBBW_NONE);
	entry->rate = cpu_to_le16(b43_ac_ht_rate500(mcs, sgi, bw40));
	entry->rts_ctl = cpu_to_le16(B43_TXH_AC_RTS_LAST);
	return 0;
}

/*
 * wl's valid VHT MCS per (bandwidth, streams), .rodata 0xd74f4 halfwords
 * indexed (bw - 1) * 8 + nss - 1 (0x18162c..0x181644), for 1..3 streams:
 * no MCS 9 at 20 MHz for 1 and 2 streams, no MCS 6 at 80 MHz for 3.
 */
static const u16 b43_ac_vht_mcs_valid[3][3] = {
	{ 0x1ff, 0x1ff, 0x3ff },	/* 20 MHz */
	{ 0x3ff, 0x3ff, 0x3ff },	/* 40 MHz */
	{ 0x3ff, 0x3ff, 0x3bf },	/* 80 MHz */
};

/*
 * Fill rate entry 0 for a VHT rate selected by mac80211 (acphy_vht), as wl
 * fills it for a VHT rspec: 20 MHz, 40 MHz on a 40 or
 * 80 MHz chanspec, 80 MHz on an 80 MHz chanspec; one rate entry, no STBC,
 * no LDPC (neither advertised), no beamforming, no fallback bandwidth (wl's
 * fb 4 with byte 0x106 of *wlc clear, 0x11bf20..0x11bfbc). Untested.
 */
static int b43_ac_vht_entry(struct b43_wldev *dev,
			    struct b43_txhdr_ac_rate *entry,
			    const struct ieee80211_tx_rate *r,
			    __le16 fctl, u16 chanspec)
{
	unsigned int nss_max = b43_ac_ht_nss(dev);
	unsigned int mcs = ieee80211_rate_get_vht_mcs(r);
	unsigned int nss = ieee80211_rate_get_vht_nss(r);
	bool sgi = !!(r->flags & IEEE80211_TX_RC_SHORT_GI);
	unsigned int bw = 0, sc = 52, gid;
	u16 phy_ctl0 = B43_TXH_AC_PHY0_FT_VHT | B43_TXH_AC_PHY0_SET;
	u32 a1, a2;
	u8 *plcp = entry->plcp.raw;

	if (r->flags & IEEE80211_TX_RC_80_MHZ_WIDTH) {
		bw = 2;
		sc = 234;
		phy_ctl0 |= B43_TXH_AC_PHY0_BW_80;	/* 0x113634..0x113654 */
	} else if (r->flags & IEEE80211_TX_RC_40_MHZ_WIDTH) {
		bw = 1;
		sc = 108;
		phy_ctl0 |= B43_TXH_AC_PHY0_BW_40;	/* 0x113634..0x113654 */
	}
	if (unlikely(!b43_modparam_ac_vht || !nss || nss > nss_max ||
		     mcs > 9 ||
		     !(b43_ac_vht_mcs_valid[bw][nss - 1] & BIT(mcs)) ||
		     (bw == 2 && !b43_ac_chanspec_is80(chanspec)) ||
		     (bw == 1 && !b43_ac_chanspec_is40(chanspec) &&
		      !b43_ac_chanspec_is80(chanspec)) ||
		     (r->flags & (IEEE80211_TX_RC_160_MHZ_WIDTH |
				  IEEE80211_TX_RC_DUP_DATA)))) {
		b43dbg(dev->wl, "AC TX: VHT rate idx 0x%x flags 0x%x chanspec 0x%04x refused\n",
		       r->idx, r->flags, chanspec);
		return -EINVAL;
	}

	/* PHY control 0: core mask << 6 for nss streams, as for HT */
	phy_ctl0 |= ((u16)b43_ac_txcore(dev, nss) << B43_TXH_AC_PHY0_ANT_SHIFT) &
		    B43_TXH_AC_PHY0_ANT;
	entry->phy_ctl0 = cpu_to_le16(phy_ctl0);
	/* PHY control 1: pwr 0, sideband >> (bw code - 1) (0x114254) */
	entry->phy_ctl1 = cpu_to_le16(b43_ac_phyctl1_sb_bw(chanspec, bw));
	/* PHY control 2: (nss - 1) << 4 | MCS (0x114a40..0x114a50) */
	entry->phy_ctl2 = cpu_to_le16(((nss - 1) << 4) | mcs);
	/*
	 * VHT-SIG-A (0x118c04..0x118c9c). Group ID 0 for a frame to an AP
	 * (FC & 0x300 == ToDS), else 63; NSTS = NSS without STBC.
	 */
	gid = (le16_to_cpu(fctl) & (IEEE80211_FCTL_TODS | IEEE80211_FCTL_FROMDS)) ==
	      IEEE80211_FCTL_TODS ? 0 : B43_AC_VHTSIGA1_GID_ALL;
	a1 = B43_AC_VHTSIGA1_BASE | bw |
	     (gid << B43_AC_VHTSIGA1_GID_SHIFT) |
	     ((nss - 1) << B43_AC_VHTSIGA1_NSTS_SHIFT);
	a2 = (sgi ? B43_AC_VHTSIGA2_SGI : 0) |
	     (mcs << B43_AC_VHTSIGA2_MCS_SHIFT) | B43_AC_VHTSIGA2_RES9;
	plcp[0] = a1 & 0xFF;
	plcp[1] = (a1 >> 8) & 0xFF;
	plcp[2] = (a1 >> 16) & 0xFF;
	plcp[3] = a2 & 0xFF;
	plcp[4] = (a2 >> 8) & 0xFF;
	plcp[5] = 0;
	entry->fb_bw = cpu_to_le16(B43_TXH_AC_FBBW_NONE);
	entry->rate = cpu_to_le16(b43_ac_vht_rate500(mcs, nss, sc, sgi));
	entry->rts_ctl = cpu_to_le16(B43_TXH_AC_RTS_LAST);
	return 0;
}

/*
 * acphy_ampdu (bitmask, default 0), A-MPDU on core rev >= 40.
 * Bit 0: RX block ack sessions
 * (main.c: ampdu_action, AMPDU_AGGREGATION; mac80211 reorders). Bit 1:
 * TX (b43_ac_ampdu_txhdr below, dma.c b43_dma_handle_txstatus and
 * main.c b43_op_ampdu_action): wl's rev 42 TX aggregation is the AQM path
 * (_wlc_sendampdu_aqm 0xd9a8), hardware aggregation of ordinary MPDUs
 * queued on the normal FIFO with the AMPDU bit, the A-MPDU limits in the
 * header tail and an epoch bit in the TSO prefix; one TX status covers
 * several MPDUs with a block ack map. Bit 2: wl's A-MPDU shm words
 * (main.c, b43_ac_ampdu_shm). Bit 3: 64 MPDUs per aggregate instead of 32.
 *
 * RX: the ucode answers an A-MPDU with a block ack by itself; wl keeps no
 * ucode state per block ack session (wlc_ampdu_recv_addba_req_resp
 * 0x146714 and scb_ampdu_rx_init 0x144f6c write no shm, AMT or register).
 * Every MPDU of an A-MPDU arrives as its own RX frame, but only the first
 * carries the PLCP: wl skips its RX rate statistics when PLCP bytes 0..2
 * are all zero (wlc_recvdata 0x89a8..0x89bc), and the PHY RX status is not
 * valid then (MAC status bit 24, wlc_phy_rssi_compute 0x644..0x65c). b43
 * would decode the zero PLCP as an invalid legacy rate and drop the MPDU,
 * so the rate, bandwidth, flags and signal of the last good frame with a
 * PLCP are reused. An HT PLCP is never zero in bytes 1..2 (HT-SIG length).
 */
unsigned int b43_modparam_acphy_ampdu = 0x6d3;
module_param_named(acphy_ampdu, b43_modparam_acphy_ampdu, uint, 0444);
MODULE_PARM_DESC(acphy_ampdu,
		 "AC-PHY A-MPDU: 1 = RX block ack sessions, 2 = TX (hardware AQM aggregation), 4 = wl's A-MPDU shm words, 8 = 64 MPDUs per aggregate, 16 = aggregate in bursts only, lone MPDUs non-aggregated, 32 = TX host flag, 64 = accept TX block ack sessions, 128 = A-MPDU header fields for session frames, 256 = ucode sequence numbers for session frames, 512 = A-MPDU status counts as rounds for rate control, 1024 = plain session frames (rate probes) visible to rate control as one-MPDU aggregates (default 0x6d3)");

/* AP power save trial hook, see xmit.h */
unsigned int b43_modparam_acphy_psfix = 23;
module_param_named(acphy_psfix, b43_modparam_acphy_psfix, uint, 0444);
MODULE_PARM_DESC(acphy_psfix,
		 "AC AP power save: 1 = PSFIX log lines, 2 = hardware PMQ (DISCPMQ clear, TX filtered status, ignore-PMQ bit), 4 = AP_LINK_PS from the PMQ, 8 = an AMT entry per station, 16 = hold PMQ entries while a station dozes (4, 8, 16 imply 2) (default 23)");

unsigned int b43_modparam_acphy_psamt = 0x8004;
module_param_named(acphy_psamt, b43_modparam_acphy_psamt, uint, 0444);
MODULE_PARM_DESC(acphy_psamt,
		 "AC AP power save: AMT attribute of a station's entry with acphy_psfix bit 8 (0x8000 valid, 0x8 A1, 0x4 A2, 0x2 A3; default 0x8004)");

/* hardware keys on ucode 907 (xmit.h, b43.h B43_AC_HWKEY_*) */
int b43_modparam_acphy_hwcrypto = 1;
module_param_named(acphy_hwcrypto, b43_modparam_acphy_hwcrypto, int, 0444);
MODULE_PARM_DESC(acphy_hwcrypto,
		 "AC-PHY hardware keys on core rev >= 40: 1 = CCMP (pairwise key in slot 4 + the station's AMT entry, group key in slot keyidx), 2 = WEP, 4 = leave the ucode's PMQ power save on for a keyed station (else its frames carry the ignore-PMQ bit unless acphy_psfix bit 8 is set); TKIP and anything else stay in software (default 1)");

/* per-frame CPU cost cuts on core rev >= 40, see xmit.h */
unsigned int b43_modparam_acphy_fastpath = 0x37;
module_param_named(acphy_fastpath, b43_modparam_acphy_fastpath, uint, 0444);
MODULE_PARM_DESC(acphy_fastpath,
		 "AC core per-frame CPU cuts: 1 = no per-frame diagnostics, 2 = TX worker kick per interrupt, 4 = re-poll interrupt reasons before unmasking, 8 = A-MSDU TX via mac80211, 16 = RX cache maintenance over the frame only, 32 = batched RX delivery (default 0x37)");

unsigned int b43_modparam_acphy_rxlazy;
module_param_named(acphy_rxlazy, b43_modparam_acphy_rxlazy, uint, 0444);
MODULE_PARM_DESC(acphy_rxlazy,
		 "AC core RX interrupt coalescing (d11 intrcvlazy 0x100): frames << 24 | timeout, 0 = b43's one frame (default 0)");

/*
 * the per-load HT TX death. Nothing the driver
 * logs at init separates a live load from a dead one; the only known
 * cure is a core re-init without a module reload (the 2026-10-07 15:11:59
 * load, phy53: dead on its first HT frame, two ucode-fatal controller
 * restarts re-drew dead and then live, 18.6/5.1 Mbit/s after). The
 * restart goes through the same b43_controller_restart / restart_work
 * (b43_chip_reset) as the ucode-fatal handler. mac80211 is not told
 * (no ieee80211_restart_hw): b43_chip_reset stops, exits, inits and starts
 * the core under wl->mutex, then replays b43_op_config and
 * b43_op_bss_info_changed (channel, BSSID, beacon interval and template),
 * so hostapd and the persistent AP survive; keys are mac80211's (software
 * crypto on the AC header, b43_op_set_key) and need no re-install; block
 * ack sessions stay in mac80211 (nothing per session in the hardware,
 * b43_op_ampdu_action). In the log the station re-associated by itself 1..2 s
 * after each "Controller restarted" (its auth/assoc/EAPOL acked), then
 * sent HT at MCS 7.
 */
int b43_modparam_acphy_htdead = 1;
module_param_named(acphy_htdead, b43_modparam_acphy_htdead, int, 0444);
MODULE_PARM_DESC(acphy_htdead,
		 "AC-PHY: 1 = restart the controller when a core init came up with HT TX dead (every HT frame fails all its attempts, none acked since the init) (default 1)");

static int b43_modparam_acphy_htdead_n = 3;
module_param_named(acphy_htdead_n, b43_modparam_acphy_htdead_n, int, 0644);
MODULE_PARM_DESC(acphy_htdead_n,
		 "AC-PHY: HT frames that must each fail all their attempts, with no HT ACK since the core init, before acphy_htdead restarts the controller (default 3)");

static int b43_modparam_acphy_htdead_max = 6;
module_param_named(acphy_htdead_max, b43_modparam_acphy_htdead_max, int, 0644);
MODULE_PARM_DESC(acphy_htdead_max,
		 "AC-PHY: controller restarts acphy_htdead may ask for per core init chain, until an HT frame is acked (default 6)");

/*
 * ucode 907's attempts at a frame that is never acked: "frames 7" in every
 * dead status of 2026-10-07 (one rate entry, the long retry limit;
 * acphy_retryfix=3 left it at 7). A status with fewer attempts and no ACK
 * (frame_count 0 = not sent, 1 = a single try) is not counted as dead.
 */
static int b43_modparam_acphy_htdead_tries = 7;
module_param_named(acphy_htdead_tries, b43_modparam_acphy_htdead_tries, int, 0644);
MODULE_PARM_DESC(acphy_htdead_tries,
		 "AC-PHY: attempts a status must show for acphy_htdead to count an unacked HT frame as failed at the retry limit (default 7)");

/* Core init: the counters of this init start over */
void b43_ac_htdead_core_init(struct b43_wldev *dev)
{
	dev->ac_ht_tx_frames = 0;
	dev->ac_ht_tx_acked = 0;
	dev->ac_ht_tx_dead = 0;
	dev->ac_htdead_pending = false;
}

/* mac80211 starts the hardware (b43_op_start): a new restart chain */
void b43_ac_htdead_chain_reset(struct b43_wldev *dev)
{
	dev->ac_htdead_restarts = 0;
	dev->ac_htdead_last = 0;
}

/*
 * One core rev >= 40 AC-header TX status, from b43_fill_txstatus_report
 * (once per MPDU of an aggregate), with the report's rate entries still
 * the header's: rates[0] with the MCS flag is an HT frame (VHT counted
 * alike). Runs in the interrupt thread; the restart is only queued here.
 */
void b43_ac_htdead_txstatus(struct b43_wldev *dev,
			    const struct ieee80211_tx_info *report,
			    const struct b43_txstatus *status)
{
	unsigned int tries, need, limit;

	if (!b43_modparam_acphy_htdead)
		return;
	if (!(report->status.rates[0].flags &
	      (IEEE80211_TX_RC_MCS | IEEE80211_TX_RC_VHT_MCS)))
		return;
	dev->ac_ht_tx_frames++;
	if (status->acked) {
		if (dev->ac_ht_tx_acked++ == 0) {
			dev->ac_htdead_restarts = 0;
		}
		return;
	}
	tries = max(b43_modparam_acphy_htdead_tries, 1);
	if (status->frame_count < tries)
		return;
	dev->ac_ht_tx_dead++;
	if (dev->ac_ht_tx_acked || dev->ac_htdead_pending)
		return;
	need = max(b43_modparam_acphy_htdead_n, 1);
	if (dev->ac_ht_tx_dead < need)
		return;
	limit = max(b43_modparam_acphy_htdead_max, 0);
	if (dev->ac_htdead_restarts >= limit) {
		return;
	}
	/* 1 s between restarts, counted from the queueing of the last one */
	if (dev->ac_htdead_restarts &&
	    time_before(jiffies, dev->ac_htdead_last + HZ))
		return;
	dev->ac_htdead_restarts++;
	dev->ac_htdead_last = jiffies;
	dev->ac_htdead_pending = true;
	b43_controller_restart(dev, "HT TX dead");
}

#define B43_AC_PSFIX_LOGS	160
static atomic_t b43_ac_psfix_logn = ATOMIC_INIT(0);

/* The PSFIX lines are capped per module load; state changes log uncapped */
bool b43_ac_psfix_log(void)
{
	return atomic_inc_return(&b43_ac_psfix_logn) <= B43_AC_PSFIX_LOGS;
}

#define B43_AC_AMPDU_DUMPS	8
static atomic_t b43_ac_ampdu_rxdumps = ATOMIC_INIT(0);
static atomic_t b43_ac_ampdu_rxn = ATOMIC_INIT(0);
/*
 * `next` (the frame behind `skb` in the TX worker's software queue,
 * NULL when none) joins an aggregate burst with `skb` when both are QoS
 * data of one station and TID that mac80211 marked IEEE80211_TX_CTL_AMPDU,
 * at the same primary rate and neither a rate probe: the epoch rules of
 * b43_ac_ampdu_epoch would keep them in one epoch, so the ucode's AQM can
 * join them. Same TID means the same mac80211 queue and so the same DMA
 * ring (b43_dma_tx, select_ring_by_priority).
 */
bool b43_ac_ampdu_same_burst(struct sk_buff *skb, struct sk_buff *next)
{
	struct ieee80211_hdr *h1, *h2;
	const struct ieee80211_tx_info *i1, *i2;

	if (!skb || !next)
		return false;
	h1 = (struct ieee80211_hdr *)skb->data;
	h2 = (struct ieee80211_hdr *)next->data;
	i1 = IEEE80211_SKB_CB(skb);
	i2 = IEEE80211_SKB_CB(next);
	if (!(i1->flags & IEEE80211_TX_CTL_AMPDU) ||
	    !(i2->flags & IEEE80211_TX_CTL_AMPDU))
		return false;
	if ((i1->flags | i2->flags) &
	    (IEEE80211_TX_CTL_RATE_CTRL_PROBE | IEEE80211_TX_CTL_SEND_AFTER_DTIM))
		return false;
	if (!ieee80211_is_data_qos(h1->frame_control) ||
	    !ieee80211_is_data_qos(h2->frame_control))
		return false;
	if (!ether_addr_equal(h1->addr1, h2->addr1) ||
	    ieee80211_get_tid(h1) != ieee80211_get_tid(h2))
		return false;
	return i1->control.rates[0].idx == i2->control.rates[0].idx &&
	       i1->control.rates[0].flags == i2->control.rates[0].flags;
}

/*
 * epoch bit for the MPDU being queued on FIFO `fifo`, the rules of
 * _wlc_sendampdu_aqm 0xdec4..0xe10c and 0xdd68..0xdda4 (xmit.h, "epoch
 * state per TX FIFO"). `entry` is rate entry 0 of the header just built.
 * Returns the epoch to write; updates the saved state as
 * wlc_ampdu_chgnsav_epoch does (reason -1 at 0xe018 saves without a
 * change, 0xe024..0xe034 neither changes nor saves).
 */
static u8 b43_ac_ampdu_epoch(struct b43_wldev *dev, unsigned int fifo,
			     bool ampdu, bool probe, const u8 *addr, u8 tid,
			     u8 ft, u16 seq,
			     const struct b43_txhdr_ac_rate *entry)
{
	struct b43_ac_ampdu_epoch *e =
		&dev->ac_ampdu_epoch[fifo % B43_AC_AMPDU_NFIFO];
	u16 phyctl0 = le16_to_cpu(entry->phy_ctl0);
	u16 phyctl1 = le16_to_cpu(entry->phy_ctl1);
	const u8 *plcp = entry->plcp.raw;
	bool change, save = true;

	if (!ampdu) {
		/* 0xdd68..0xdda4: once after aggregates (reason 2, NAGG) */
		if (e->prev_ft) {
			e->epoch ^= 1;
			e->prev_ft = 0;
		}
		return e->epoch;
	}
	if (probe) {
		/* 0xded0..0xdef8: frame id bit 15, rate probe (reason 1) */
		change = true;
	} else if (!ether_addr_equal(addr, e->prev_addr) ||
		   tid != e->prev_tid) {
		/* 0xdefc..0xdf20: station or TID (reason 4) */
		change = true;
	} else if (ft != e->prev_ft || phyctl0 != e->prev_phyctl0 ||
		   phyctl1 != e->prev_phyctl1 || plcp[0] != e->prev_plcp[0] ||
		   plcp[3] != e->prev_plcp[3] ||
		   (ft == B43_AC_AMPDU_FT_VHT &&
		    (plcp[1] != e->prev_plcp[1] || plcp[2] != e->prev_plcp[2] ||
		     plcp[4] != e->prev_plcp[4]))) {
		/*
		 * 0xdf40..0xe00c: frame type, PHY control, SIG (reason 0).
		 * Includes the first aggregate after a non-aggregate frame
		 * (prev_ft 0 differs from HT/VHT), so 0xe010..0xe01c's
		 * "prev_ft 0: save only" is never reached.
		 */
		change = true;
	} else if (seq == e->next_seq) {
		/* 0xe020..0xe034: the expected sequence number: as before */
		change = false;
		save = false;
	} else {
		/* 0xe038..0xe03c: out of sequence (reason 5) */
		change = true;
	}
	if (change)
		e->epoch ^= 1;
	if (save) {
		/* wlc_ampdu_chgnsav_epoch 0x141c60..0x141ce8 */
		ether_addr_copy(e->prev_addr, addr);
		e->prev_tid = tid;
		e->prev_ft = ft;
		e->prev_phyctl0 = phyctl0;
		e->prev_phyctl1 = phyctl1;
		memcpy(e->prev_plcp, plcp, sizeof(e->prev_plcp));
	}
	/* 0xe1b4..0xe1c4: the next MPDU of this station and TID */
	e->next_seq = (seq + 1) & (IEEE80211_SCTL_SEQ >> 4);
	return e->epoch;
}

/*
 * A-MPDU TX fields of an HT or VHT MPDU that mac80211 marks
 * IEEE80211_TX_CTL_AMPDU (an operational block ack session for its TID),
 * as _wlc_sendampdu_aqm writes them (0xde3c..0xdec0), then the epoch bit
 * in the TSO prefix (wlc_txh_set_epoch 0x2d04, call 0xe10c). Non-aggregate
 * frames only feed the epoch (0xdd68..0xdda4). The peer's MPDU density and
 * the negotiated window come from its ieee80211_sta (found by address 1
 * under RCU; the AP's vif is in the control block); without a station the
 * MPDU goes out non-aggregated, which the status side sees from the
 * header bit, not from the mac80211 flag.
 *
 * acphy_ampdu bit 4: an aggregate-able MPDU keeps the AMPDU bit only
 * as part of a burst: another MPDU of its burst follows it in the TX
 * worker's queue (dev->ac_ampdu_next), or the MPDU before it on this FIFO
 * was such a one for the same station and TID. A lone MPDU goes out as an
 * ordinary frame (no AMPDU bit, the epoch flips as for any non-aggregate
 * frame). On the air (2026-10-07) every MPDU the
 * AQM sent alone (status count 1) was transmitted 7 times and never
 * acked, while the same frames without the bit are acked; a burst was
 * never reached because the TCP SYN itself was such a lone MPDU.
 */
static void b43_ac_ampdu_txhdr(struct b43_wldev *dev, u8 *toe,
			       struct b43_txhdr_ac *txhdr,
			       struct sk_buff *skb,
			       struct ieee80211_tx_info *info, u16 cookie,
			       bool ht, bool vht)
{
	struct ieee80211_hdr *wlhdr = (struct ieee80211_hdr *)skb->data;
	struct ieee80211_sta *sta;
	struct b43_ac_sta *bsta;
	struct b43_ac_ampdu_epoch *e =
		&dev->ac_ampdu_epoch[(cookie & B43_TXH_AC_FID_FIFO) %
				     B43_AC_AMPDU_NFIFO];
	unsigned int fifo = cookie & B43_TXH_AC_FID_FIFO;
	unsigned int wsize = 0, maxpdu = 0, dens = 0;
	unsigned int dur = B43_AC_AMPDU_MAX_DUR;
	unsigned int ffcap;
	u16 seq = le16_to_cpu(wlhdr->seq_ctrl) >> 4;
	u8 tid = 0, ft = 0, epoch;
	bool ampdu = false, lone = false, more = false, cont = false;
	/* an HT frame with 2 or more streams at rate entry 0 */
	bool nss2 = ht && b43_ac_mimo_rate_is_nss2(&info->control.rates[0]);

	if (!b43_ac_ampdu(dev, B43_AC_AMPDU_TX) || !b43_txhdr_has_toe(dev))
		return;
	if (ieee80211_is_data_qos(wlhdr->frame_control))
		tid = ieee80211_get_tid(wlhdr);
	if ((info->flags & IEEE80211_TX_CTL_AMPDU) && (ht || vht) &&
	    ieee80211_is_data_qos(wlhdr->frame_control) &&
	    info->control.vif) {
		rcu_read_lock();
		sta = ieee80211_find_sta(info->control.vif, wlhdr->addr1);
		if (sta) {
			bsta = (struct b43_ac_sta *)sta->drv_priv;
			wsize = bsta->ampdu_wsize[tid];
			dens = sta->deflink.ht_cap.ampdu_density;
		}
		rcu_read_unlock();
		ampdu = wsize > 0;
	}
	/* the header fields only with bit 7; else the frame is plain */
	if (!b43_ac_ampdu(dev, B43_AC_AMPDU_TXHDR))
		ampdu = false;
	/* acphy_mimo_ampdu=1, a 2+ stream frame goes out plain */
	if (nss2 && b43_modparam_ac_mimo_ampdu == 1)
		ampdu = false;
	if (b43_ac_ampdu(dev, B43_AC_AMPDU_BURST)) {
		more = ampdu && b43_ac_ampdu_same_burst(skb, dev->ac_ampdu_next);
		cont = ampdu && e->burst && e->prev_ft &&
		       !(info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE) &&
		       ether_addr_equal(wlhdr->addr1, e->prev_addr) &&
		       tid == e->prev_tid;
		lone = ampdu && !more && !cont;
		if (lone)
			ampdu = false;
		e->burst = more;
	}
	if (ampdu) {
		ft = vht ? B43_AC_AMPDU_FT_VHT : B43_AC_AMPDU_FT_HT;
		maxpdu = b43_ac_ampdu(dev, B43_AC_AMPDU_TX64) ?
			 B43_AC_AMPDU_MAX_MPDU64 : B43_AC_AMPDU_MAX_MPDU;
		maxpdu = min(maxpdu, wsize);
		/* acphy_mimo_ampdu=N caps a 2+ stream aggregate at N */
		if (nss2 && b43_modparam_ac_mimo_ampdu >= 2)
			maxpdu = min_t(unsigned int, maxpdu,
				       b43_modparam_ac_mimo_ampdu);
		/*
		 * acphy_mimo_ffpld: brcmsmac's per-rate limit at rate
		 * entry 0's rate (filled before this, b43_generate_txhdr)
		 */
		ffcap = b43_ac_ffpld_cap(&txhdr->rate[0]);
		if (ffcap) {
			maxpdu = min(maxpdu, ffcap);
		}
		/* 0xde4c, 0xde50 */
		txhdr->mac_ctl_lo |= cpu_to_le16(B43_TXH_AC_MAC_AMPDU);
		txhdr->mac_ctl_hi &= ~cpu_to_le16(B43_TXH_AC_MACH_VHT);
		/* a 2+ stream aggregate's own duration and density */
		if (nss2 && b43_modparam_ac_mimo_dur > 0)
			dur = min_t(unsigned int, b43_modparam_ac_mimo_dur,
				    0xFFF0);
		if (nss2 && b43_modparam_ac_mimo_dens >= 0)
			dens = b43_modparam_ac_mimo_dens;
		/* 0xde8c..0xdec0 */
		txhdr->ampdu_max_mpdu[0] = maxpdu;
		txhdr->ampdu_max_mpdu[1] = maxpdu;
		txhdr->ampdu_dur_dens =
			cpu_to_le16((dur & ~B43_AC_AMPDU_DENS_MASK) |
				    (dens & B43_AC_AMPDU_DENS_MASK));
		txhdr->ampdu_wsize = wsize - 1;
	}
	epoch = b43_ac_ampdu_epoch(dev, fifo, ampdu,
				   !!(info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE),
				   wlhdr->addr1, tid, ft, seq, &txhdr->rate[0]);
	toe[2] = (toe[2] & ~B43_TXH_AC_TOE_EPOCH) |
		 (epoch ? B43_TXH_AC_TOE_EPOCH : 0);
	/*
	 * the first headers of 2+ stream frames as the DMA sends them,
	 * QUIET or not: the A-MPDU decision, the crypto bytes and the 124
	 * bytes, 4 lines of 32.
	 */
	if (nss2 && b43_modparam_ac_mimo_log > 0 &&
	    atomic_inc_return(&b43_ac_mimo_hdrlogs) <= B43_AC_MIMO_HDRLOGS) {

	}
	/* QUIET: no per-frame log atomics */
	if (b43_ac_fastpath(dev, B43_AC_FASTPATH_QUIET))
		return;
}
static struct {
	bool valid;
	bool signal_valid;
	u8 encoding;
	u8 bw;
	u8 enc_flags;
	int rate_idx;
	s8 signal;
} b43_ac_ampdu_last;

/*
 * called after b43_ac_ampdu_txhdr, so the bytes are what the DMA
 * sends (the bring-up trials' dumps showed MAC control 4080 where the header
 * had 40c0: the dump ran before the A-MPDU edit); `toe` is the 4 byte
 * TOE prefix in front of the header (NULL without one).
 */
/* PHY control word 2 for a legacy rate: its index in the table above. */
static u16 b43_ac_legacy_rate_idx(u8 bitrate)
{
	const u8 *tbl;
	unsigned int i, n;

	if (b43_is_ofdm_rate(bitrate)) {
		tbl = b43_ac_ofdm_rates;
		n = ARRAY_SIZE(b43_ac_ofdm_rates);
	} else {
		tbl = b43_ac_cck_rates;
		n = ARRAY_SIZE(b43_ac_cck_rates);
	}
	for (i = 0; i < n; i++) {
		if (tbl[i] == bitrate)
			return i;
	}
	/* wl's loop ends with the table size when nothing matches (0x114abc) */
	B43_WARN_ON(1);
	return n;
}

/*
 * Header byte 8: the 802.11 header length as wl computes it
 * (0x11b41c, 0x11b440..0x11b44c, 0x11b548..0x11b594): data frames 24, or 30
 * with ToDS and FromDS both set, plus 2 for QoS data; control frames 16;
 * anything else 24.
 */
static u8 b43_ac_hdr_len(__le16 fctl)
{
	u16 fc = le16_to_cpu(fctl);
	u8 len;

	switch (fc & IEEE80211_FCTL_FTYPE) {
	case IEEE80211_FTYPE_DATA:
		if ((fc & (IEEE80211_FCTL_TODS | IEEE80211_FCTL_FROMDS)) ==
		    (IEEE80211_FCTL_TODS | IEEE80211_FCTL_FROMDS))
			len = 30;
		else
			len = 24;
		if (fc & IEEE80211_STYPE_QOS_DATA)
			len += 2;
		break;
	case IEEE80211_FTYPE_CTL:
		len = 16;
		break;
	default:
		len = 24;
	}
	return len;
}

/*
 * Generate the 124 byte AC TX header (ucode 907.113, PHY type AC).
 *
 * The header is what wl.ko's `wlc_d11hdrs` @0x11b360 writes for a frame
 * that does not go through `wlc_scb_ratesel_gettxrate`: one rate entry
 * (sp+208 = 1 at 0x11b7a0), the other three zero. b43 transmits legacy CCK
 * and OFDM rates at 20 MHz only, so only those cases are filled in. Fields
 * the decode does not establish are left zero, as wl's __memzero leaves them
 * (0x11b48c).
 */
static int b43_generate_txhdr_ac(struct b43_wldev *dev,
				 u8 *_txhdr,
				 struct sk_buff *skb_frag,
				 struct ieee80211_tx_info *info,
				 u16 cookie)
{
	struct b43_txhdr_ac *txhdr;
	struct b43_txhdr_ac_rate *entry;
	const struct ieee80211_hdr *wlhdr =
	    (const struct ieee80211_hdr *)skb_frag->data;
	unsigned int frame_len = skb_frag->len + FCS_LEN;
	const struct ieee80211_key_conf *hwkey = info->control.hw_key;
	const struct b43_ac_hwkey *hk = NULL;
	const struct b43_phy *phy = &dev->phy;
	struct ieee80211_tx_rate *rates = info->control.rates;
	struct ieee80211_rate *txrate;
	u16 mac_ctl = 0;
	u16 phy_ctl0 = 0;
	u16 chanspec;
	u8 rate = 0;
	u8 *toe;
	bool ht = !!(rates[0].flags & IEEE80211_TX_RC_MCS);
	bool vht = !!(rates[0].flags & IEEE80211_TX_RC_VHT_MCS);
	int err;

	BUILD_BUG_ON(sizeof(struct b43_txhdr_ac_rate) != 20);
	BUILD_BUG_ON(sizeof(struct b43_txhdr_ac) != 124);

	/*
	 * a hardware key (acphy_hwcrypto, b43_op_set_key gave it its
	 * slot). wl adds the key's ICV length to the frame length
	 * (0x11b46c..0x11b478), so the PLCPs and byte 10 count the MIC the
	 * ucode appends; the IV is already in the frame (mac80211's
	 * GENERATE_IV; _wlc_txh_iv_upd 0x11456c..0x114570 leaves the header
	 * alone for a non-TKIP key on rev > 39). A key the slot no longer
	 * holds drops the frame, as the legacy path does (never send it
	 * unencrypted). Without acphy_hwcrypto b43_op_set_key refused every
	 * key and mac80211 encrypts in software, so hw_key is NULL here.
	 */
	if (hwkey) {
		if (!b43_ac_hwcrypto(dev, B43_AC_HWCRYPTO_CCMP |
					  B43_AC_HWCRYPTO_WEP) ||
		    hwkey->hw_key_idx >= B43_AC_HWKEY_SLOTS ||
		    dev->ac_hwkey[hwkey->hw_key_idx].keyconf != hwkey)
			return -ENOKEY;
		hk = &dev->ac_hwkey[hwkey->hw_key_idx];
		frame_len += hwkey->icv_len;
	}

	/* TOE prefix (wlc_toe_add_hdr 0x199ef4), then the AC header */
	memset(_txhdr, 0, b43_txhdr_size(dev));
	toe = _txhdr;
	if (b43_txhdr_has_toe(dev)) {
		_txhdr[0] = B43_TXH_AC_TOE_BYTE0;
		_txhdr += B43_TXH_AC_TOE_LEN;
	}
	txhdr = (struct b43_txhdr_ac *)_txhdr;
	/* bytes 100 and 101, wlc_d11hdrs 0x11cc18..0x11cc2c */
	if (hk) {
		txhdr->key_algo = hk->algo_hw << 4;
		txhdr->key_idx = hwkey->hw_key_idx;
	}
	entry = &txhdr->rate[0];

	/* ieee80211_get_tx_rate() indexes the legacy table; not for MCS */
	if (!ht && !vht) {
		txrate = ieee80211_get_tx_rate(dev->wl->hw, info);
		rate = txrate ? txrate->hw_value : B43_CCK_RATE_1MB;
	}

	/* Packet part, bytes 0..19 */
	if (!(info->flags & IEEE80211_TX_CTL_NO_ACK))
		mac_ctl |= B43_TXH_AC_MAC_ACK;
	if (info->flags & IEEE80211_TX_CTL_ASSIGN_SEQ)
		mac_ctl |= B43_TXH_AC_MAC_HWSEQ;
	/*
	 * acphy_ampdu bit 8 (diagnostic): QoS data of a TX block ack
	 * session numbered by the ucode (@0x5D7, one counter in shm 0x7A8)
	 * instead of mac80211's per-TID counter; the session's SSN then no
	 * longer matches, so only the ACK behaviour is of interest.
	 */
	if (b43_ac_ampdu(dev, B43_AC_AMPDU_HWSEQ) &&
	    (info->flags & IEEE80211_TX_CTL_AMPDU) &&
	    ieee80211_is_data_qos(wlhdr->frame_control))
		mac_ctl |= B43_TXH_AC_MAC_HWSEQ;
	if (info->flags & IEEE80211_TX_CTL_FIRST_FRAGMENT)
		mac_ctl |= B43_TXH_AC_MAC_STMSDU;
	/*
	 * a PS-Poll or U-APSD response and the first frame after a
	 * wake-up go out even if the PMQ still holds the station as dozing
	 * (ucode 907 @0x03D1 -> L232, wl 0x11b490..0x11b4c0).
	 */
	if (b43_ac_psfix(dev, B43_AC_PSFIX_PMQ) &&
	    (info->flags & (IEEE80211_TX_CTL_NO_PS_BUFFER |
			    IEEE80211_TX_CTL_CLEAR_PS_FILT)))
		mac_ctl |= B43_TXH_AC_MAC_IPMQ;
	/*
	 * the AMT entry a pairwise hardware key needs must not hand the
	 * station's power save to the ucode unless acphy_psfix bit 8 asked
	 * for that (xmit.h, acphy_hwcrypto bit 4): with the entry the A2
	 * lookup succeeds and the ucode suppresses unicast frames while its
	 * PMQ holds the station dozing, which mac80211 handles in software
	 * already (2026-10-08 03:15 UTC: downlink 15.6 Mbit/s against 36..40,
	 * no failure, no drop). The ignore-PMQ bit per frame keeps the ucode
	 * out of it, as with no entry at all.
	 */
	if (hk && hk->pairwise && !b43_ac_psfix(dev, B43_AC_PSFIX_AMT) &&
	    !(b43_modparam_acphy_hwcrypto & B43_AC_HWCRYPTO_PMQ))
		mac_ctl |= B43_TXH_AC_MAC_IPMQ;
	txhdr->mac_ctl_lo = cpu_to_le16(mac_ctl);
	/* Single rate entry; 0x0004 when the primary rate is VHT (0x11c5c0) */
	txhdr->mac_ctl_hi = cpu_to_le16(B43_TXH_AC_MACH_NORATESEL |
					(vht ? B43_TXH_AC_MACH_VHT : 0));

	/* Must equal shm 0xa0 (b43_switch_channel), see phy_common.c */
	chanspec = b43_ac_chanspec(dev, phy->channel);
	txhdr->chanspec = cpu_to_le16(chanspec);

	txhdr->hdr_len = b43_ac_hdr_len(wlhdr->frame_control);
	/* pkttotlen + 4 (0x11b458..0x11b464) */
	txhdr->frame_len = cpu_to_le16(frame_len);
	txhdr->frame_id = cpu_to_le16(cookie);
	txhdr->seq_ctl = wlhdr->seq_ctrl;
	/* unk16 (packet flag 0x200), unk18, key bytes and tail: zero */

	if (vht && dev->dev->core_rev == 56 &&
	    ieee80211_rate_get_vht_mcs(&rates[0]) > 7) {
		/*
		 * Core rev 56: VHT-MCS 8/9 never get through on TX yet and the
		 * header has a single VHT rate entry (no fallback), so minstrel's
		 * probes at 8/9 lose the frame; send them at MCS 7.
		 */
		ieee80211_rate_set_vht(&rates[0], 7,
				       ieee80211_rate_get_vht_nss(&rates[0]));
	}
	if (vht) {
		/* VHT MCS: rate entry 0 at byte 20, see b43_ac_vht_entry */
		err = b43_ac_vht_entry(dev, entry, &rates[0],
				       wlhdr->frame_control, chanspec);
		if (err)
			return err;
		b43_ac_ht_set_duration(dev, info,
				       (struct ieee80211_hdr *)skb_frag->data,
				       ieee80211_rate_get_vht_mcs(&rates[0]));
		goto retry_count;
	}
	if (ht) {
		/* HT MCS: rate entry 0 at byte 20, see b43_ac_ht_entry */
		err = b43_ac_ht_entry(dev, entry, &rates[0], frame_len,
				      chanspec,
				      !!(info->flags & IEEE80211_TX_CTL_STBC));
		if (err)
			return err;
		b43_ac_ht_set_duration(dev, info,
				       (struct ieee80211_hdr *)skb_frag->data,
				       rates[0].idx & 7);
		goto retry_count;
	}

	/* Rate entry 0 at byte 20 */
	if (b43_is_ofdm_rate(rate))
		phy_ctl0 |= B43_TXH_AC_PHY0_FT_OFDM;
	else
		phy_ctl0 |= B43_TXH_AC_PHY0_FT_CCK;
	phy_ctl0 |= B43_TXH_AC_PHY0_SET;
	/*
	 * wl passes preamble 1 (short) only for CCK above 1 Mbit/s and when the
	 * caller allows it (0x11c104..0x11c158); OFDM keeps preamble 0.
	 */
	if (!b43_is_ofdm_rate(rate) && rate != B43_CCK_RATE_1MB &&
	    (rates[0].flags & IEEE80211_TX_RC_USE_SHORT_PREAMBLE))
		phy_ctl0 |= B43_TXH_AC_PHY0_SHORTPRMBL;
	/*
	 * Antenna bits 6..9: TX core mask << 6, the value
	 * wlc_stf_d11hdrs_phyctl_txant returns for PHY type 11 (call 0x113630,
	 * 0x191124..0x19113c). Left zero, the PHY transmits on no core.
	 * Bandwidth bits 14..15 are 0 for 20 MHz. No beamforming.
	 */
	phy_ctl0 |= ((u16)b43_ac_txcore(dev, 1) << B43_TXH_AC_PHY0_ANT_SHIFT) &
		    B43_TXH_AC_PHY0_ANT;
	entry->phy_ctl0 = cpu_to_le16(phy_ctl0);
	/*
	 * PHY control 1 = (pwr << 3) | ((chanspec >> 8 & 7) >> (bw - 1))
	 * (0x114254..0x114268). A legacy rspec is 20 MHz in every chanspec
	 * (0x11bc44..0x11bc70), so bw = 1: the control sideband of a 40 MHz
	 * chanspec, 0 for a 20 MHz one.
	 * pwr is one byte of the per-bandwidth table that
	 * wlc_stf_get_204080_pwrs fills (0x11c184) from the stf+0x2c ppr,
	 * which wlc_update_txppr_offset builds as a backoff in half dB clamped
	 * to 0..31 (0x193ef4..0x193f10, wlc_stf_convert_to_s41_hdb 0x19016c):
	 * zero is no backoff, the PHY's own TX power. OFDM frames take their
	 * rate's offset (acphy_rateppr, 0 by default); CCK is never sent on
	 * the AC PHY's 5 GHz band.
	 */
	entry->phy_ctl1 = cpu_to_le16(b43_ac_phyctl1_sb(chanspec, false) |
		(b43_is_ofdm_rate(rate) ?
		 b43_ac_phyctl1_pwr(dev, B43_AC_PWROFF_OFDM +
				    b43_ac_legacy_rate_idx(rate), false) : 0));
	entry->phy_ctl2 = cpu_to_le16(b43_ac_legacy_rate_idx(rate));
	/*
	 * PLCP. wl's rate_info (.rodata 0xd73fc) holds the same rate codes as
	 * b43_plcp_get_ratecode_cck/ofdm (0x0a 0x14 0x37 0x6e for CCK, low
	 * nibble 0xb 0xf 0xa 0xe 0x9 0xd 0x8 0xc for OFDM). The rest of
	 * wlc_compute_plcp (call 0x11c328) is not compared with b43's
	 * generator.
	 */
	b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)&entry->plcp,
			      frame_len, rate);
	entry->fb_bw = cpu_to_le16(B43_TXH_AC_FBBW_NONE);
	/* wlc_rate_rspec2rate(rspec) / 500 (0x11c330..0x11c360) */
	entry->rate = cpu_to_le16(rate);
	/*
	 * RTS/CTS control: wl sets bits 0, 2..4 and 8..11 only when it chooses
	 * protection (0x11c5d8..0x11c7b0); which of bits 2 and 3 means RTS is
	 * open, so no protection frame is requested here. Bit 5 marks the last
	 * entry.
	 */
	entry->rts_ctl = cpu_to_le16(B43_TXH_AC_RTS_LAST);
	/* Beamforming word: written only when wlc_txbf_sel selects (0x11c1f0) */

retry_count:
	/* A-MPDU fields and the epoch bit, after rate entry 0 is known */
	b43_ac_ampdu_txhdr(dev, toe, txhdr, skb_frag, info, cookie, ht, vht);

	/* Same retry-count bookkeeping as the 598 path, for the status report */
	if (rates[0].count <= dev->wl->hw->conf.long_frame_max_tx_count)
		rates[0].count = dev->wl->hw->conf.long_frame_max_tx_count;
	else
		rates[0].count = dev->wl->hw->conf.short_frame_max_tx_count;

	return 0;
}

/* Generate a TX data header. */
int b43_generate_txhdr(struct b43_wldev *dev,
		       u8 *_txhdr,
		       struct sk_buff *skb_frag,
		       struct ieee80211_tx_info *info,
		       u16 cookie)
{
	const unsigned char *fragment_data = skb_frag->data;
	unsigned int fragment_len = skb_frag->len;
	struct b43_txhdr *txhdr = (struct b43_txhdr *)_txhdr;
	const struct b43_phy *phy = &dev->phy;
	const struct ieee80211_hdr *wlhdr =
	    (const struct ieee80211_hdr *)fragment_data;
	int use_encryption = !!info->control.hw_key;
	__le16 fctl = wlhdr->frame_control;
	struct ieee80211_rate *fbrate;
	u8 rate, rate_fb;
	int rate_ofdm, rate_fb_ofdm;
	unsigned int plcp_fragment_len;
	u32 mac_ctl = 0;
	u16 phy_ctl = 0;
	bool fill_phy_ctl1 = (phy->type == B43_PHYTYPE_LP ||
			      phy->type == B43_PHYTYPE_N ||
			      phy->type == B43_PHYTYPE_HT);
	u8 extra_ft = 0;
	struct ieee80211_rate *txrate;
	struct ieee80211_tx_rate *rates;

	if (b43_txhdr_is_ac(dev))
		return b43_generate_txhdr_ac(dev, _txhdr, skb_frag, info,
					     cookie);

	memset(txhdr, 0, sizeof(*txhdr));

	txrate = ieee80211_get_tx_rate(dev->wl->hw, info);
	rate = txrate ? txrate->hw_value : B43_CCK_RATE_1MB;
	rate_ofdm = b43_is_ofdm_rate(rate);
	fbrate = ieee80211_get_alt_retry_rate(dev->wl->hw, info, 0) ? : txrate;
	rate_fb = fbrate->hw_value;
	rate_fb_ofdm = b43_is_ofdm_rate(rate_fb);

	if (rate_ofdm)
		txhdr->phy_rate = b43_plcp_get_ratecode_ofdm(rate);
	else
		txhdr->phy_rate = b43_plcp_get_ratecode_cck(rate);
	txhdr->mac_frame_ctl = wlhdr->frame_control;
	memcpy(txhdr->tx_receiver, wlhdr->addr1, ETH_ALEN);

	/* Calculate duration for fallback rate */
	if ((rate_fb == rate) ||
	    (wlhdr->duration_id & cpu_to_le16(0x8000)) ||
	    (wlhdr->duration_id == cpu_to_le16(0))) {
		/* If the fallback rate equals the normal rate or the
		 * dur_id field contains an AID, CFP magic or 0,
		 * use the original dur_id field. */
		txhdr->dur_fb = wlhdr->duration_id;
	} else {
		txhdr->dur_fb = ieee80211_generic_frame_duration(
			dev->wl->hw, info->control.vif, info->band,
			fragment_len, fbrate);
	}

	plcp_fragment_len = fragment_len + FCS_LEN;
	if (use_encryption) {
		u8 key_idx = info->control.hw_key->hw_key_idx;
		struct b43_key *key;
		int wlhdr_len;
		size_t iv_len;

		B43_WARN_ON(key_idx >= ARRAY_SIZE(dev->key));
		key = &(dev->key[key_idx]);

		if (unlikely(!key->keyconf)) {
			/* This key is invalid. This might only happen
			 * in a short timeframe after machine resume before
			 * we were able to reconfigure keys.
			 * Drop this packet completely. Do not transmit it
			 * unencrypted to avoid leaking information. */
			return -ENOKEY;
		}

		/* Hardware appends ICV. */
		plcp_fragment_len += info->control.hw_key->icv_len;

		key_idx = b43_kidx_to_fw(dev, key_idx);
		mac_ctl |= (key_idx << B43_TXH_MAC_KEYIDX_SHIFT) &
			   B43_TXH_MAC_KEYIDX;
		mac_ctl |= (key->algorithm << B43_TXH_MAC_KEYALG_SHIFT) &
			   B43_TXH_MAC_KEYALG;
		wlhdr_len = ieee80211_hdrlen(fctl);
		if (key->algorithm == B43_SEC_ALGO_TKIP) {
			u16 phase1key[5];
			int i;
			/* we give the phase1key and iv16 here, the key is stored in
			 * shm. With that the hardware can do phase 2 and encryption.
			 */
			ieee80211_get_tkip_p1k(info->control.hw_key, skb_frag, phase1key);
			/* phase1key is in host endian. Copy to little-endian txhdr->iv. */
			for (i = 0; i < 5; i++) {
				txhdr->iv[i * 2 + 0] = phase1key[i];
				txhdr->iv[i * 2 + 1] = phase1key[i] >> 8;
			}
			/* iv16 */
			memcpy(txhdr->iv + 10, ((u8 *) wlhdr) + wlhdr_len, 3);
		} else {
			iv_len = min_t(size_t, info->control.hw_key->iv_len,
				     ARRAY_SIZE(txhdr->iv));
			memcpy(txhdr->iv, ((u8 *) wlhdr) + wlhdr_len, iv_len);
		}
	}
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_598.plcp),
				      plcp_fragment_len, rate);
		break;
	case B43_FW_HDR_351:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_351.plcp),
				      plcp_fragment_len, rate);
		break;
	case B43_FW_HDR_410:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_410.plcp),
				      plcp_fragment_len, rate);
		break;
	}
	b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->plcp_fb),
			      plcp_fragment_len, rate_fb);

	/* Extra Frame Types */
	if (rate_fb_ofdm)
		extra_ft |= B43_TXH_EFT_FB_OFDM;
	else
		extra_ft |= B43_TXH_EFT_FB_CCK;

	/* Set channel radio code. Note that the micrcode ORs 0x100 to
	 * this value before comparing it to the value in SHM, if this
	 * is a 5Ghz packet.
	 */
	txhdr->chan_radio_code = phy->channel;

	/* PHY TX Control word */
	if (rate_ofdm)
		phy_ctl |= B43_TXH_PHY_ENC_OFDM;
	else
		phy_ctl |= B43_TXH_PHY_ENC_CCK;
	if (info->control.rates[0].flags & IEEE80211_TX_RC_USE_SHORT_PREAMBLE)
		phy_ctl |= B43_TXH_PHY_SHORTPRMBL;

	switch (b43_ieee80211_antenna_sanitize(dev, 0)) {
	case 0: /* Default */
		phy_ctl |= B43_TXH_PHY_ANT01AUTO;
		break;
	case 1: /* Antenna 0 */
		phy_ctl |= B43_TXH_PHY_ANT0;
		break;
	case 2: /* Antenna 1 */
		phy_ctl |= B43_TXH_PHY_ANT1;
		break;
	case 3: /* Antenna 2 */
		phy_ctl |= B43_TXH_PHY_ANT2;
		break;
	case 4: /* Antenna 3 */
		phy_ctl |= B43_TXH_PHY_ANT3;
		break;
	default:
		B43_WARN_ON(1);
	}

	rates = info->control.rates;
	/* MAC control */
	if (!(info->flags & IEEE80211_TX_CTL_NO_ACK))
		mac_ctl |= B43_TXH_MAC_ACK;
	/* use hardware sequence counter as the non-TID counter */
	if (info->flags & IEEE80211_TX_CTL_ASSIGN_SEQ)
		mac_ctl |= B43_TXH_MAC_HWSEQ;
	if (info->flags & IEEE80211_TX_CTL_FIRST_FRAGMENT)
		mac_ctl |= B43_TXH_MAC_STMSDU;
	if (!phy->gmode)
		mac_ctl |= B43_TXH_MAC_5GHZ;

	/* Overwrite rates[0].count to make the retry calculation
	 * in the tx status easier. need the actual retry limit to
	 * detect whether the fallback rate was used.
	 */
	if ((rates[0].flags & IEEE80211_TX_RC_USE_RTS_CTS) ||
	    (rates[0].count <= dev->wl->hw->conf.long_frame_max_tx_count)) {
		rates[0].count = dev->wl->hw->conf.long_frame_max_tx_count;
		mac_ctl |= B43_TXH_MAC_LONGFRAME;
	} else {
		rates[0].count = dev->wl->hw->conf.short_frame_max_tx_count;
	}

	/* Generate the RTS or CTS-to-self frame */
	if ((rates[0].flags & IEEE80211_TX_RC_USE_RTS_CTS) ||
	    (rates[0].flags & IEEE80211_TX_RC_USE_CTS_PROTECT)) {
		unsigned int len;
		struct ieee80211_hdr *hdr;
		int rts_rate, rts_rate_fb;
		int rts_rate_ofdm, rts_rate_fb_ofdm;
		struct b43_plcp_hdr6 *plcp;
		struct ieee80211_rate *rts_cts_rate;

		rts_cts_rate = ieee80211_get_rts_cts_rate(dev->wl->hw, info);

		rts_rate = rts_cts_rate ? rts_cts_rate->hw_value : B43_CCK_RATE_1MB;
		rts_rate_ofdm = b43_is_ofdm_rate(rts_rate);
		rts_rate_fb = b43_calc_fallback_rate(rts_rate, phy->gmode);
		rts_rate_fb_ofdm = b43_is_ofdm_rate(rts_rate_fb);

		if (rates[0].flags & IEEE80211_TX_RC_USE_CTS_PROTECT) {
			struct ieee80211_cts *cts;

			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				cts = (struct ieee80211_cts *)
					(txhdr->format_598.rts_frame);
				break;
			case B43_FW_HDR_351:
				cts = (struct ieee80211_cts *)
					(txhdr->format_351.rts_frame);
				break;
			case B43_FW_HDR_410:
				cts = (struct ieee80211_cts *)
					(txhdr->format_410.rts_frame);
				break;
			}
			ieee80211_ctstoself_get(dev->wl->hw, info->control.vif,
						fragment_data, fragment_len,
						info, cts);
			mac_ctl |= B43_TXH_MAC_SENDCTS;
			len = sizeof(struct ieee80211_cts);
		} else {
			struct ieee80211_rts *rts;

			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				rts = (struct ieee80211_rts *)
					(txhdr->format_598.rts_frame);
				break;
			case B43_FW_HDR_351:
				rts = (struct ieee80211_rts *)
					(txhdr->format_351.rts_frame);
				break;
			case B43_FW_HDR_410:
				rts = (struct ieee80211_rts *)
					(txhdr->format_410.rts_frame);
				break;
			}
			ieee80211_rts_get(dev->wl->hw, info->control.vif,
					  fragment_data, fragment_len,
					  info, rts);
			mac_ctl |= B43_TXH_MAC_SENDRTS;
			len = sizeof(struct ieee80211_rts);
		}
		len += FCS_LEN;

		/* Generate the PLCP headers for the RTS/CTS frame */
		switch (dev->fw.hdr_format) {
		case B43_FW_HDR_598:
			plcp = &txhdr->format_598.rts_plcp;
			break;
		case B43_FW_HDR_351:
			plcp = &txhdr->format_351.rts_plcp;
			break;
		case B43_FW_HDR_410:
			plcp = &txhdr->format_410.rts_plcp;
			break;
		}
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)plcp,
				      len, rts_rate);
		plcp = &txhdr->rts_plcp_fb;
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)plcp,
				      len, rts_rate_fb);

		switch (dev->fw.hdr_format) {
		case B43_FW_HDR_598:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_598.rts_frame);
			break;
		case B43_FW_HDR_351:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_351.rts_frame);
			break;
		case B43_FW_HDR_410:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_410.rts_frame);
			break;
		}
		txhdr->rts_dur_fb = hdr->duration_id;

		if (rts_rate_ofdm) {
			extra_ft |= B43_TXH_EFT_RTS_OFDM;
			txhdr->phy_rate_rts =
			    b43_plcp_get_ratecode_ofdm(rts_rate);
		} else {
			extra_ft |= B43_TXH_EFT_RTS_CCK;
			txhdr->phy_rate_rts =
			    b43_plcp_get_ratecode_cck(rts_rate);
		}
		if (rts_rate_fb_ofdm)
			extra_ft |= B43_TXH_EFT_RTSFB_OFDM;
		else
			extra_ft |= B43_TXH_EFT_RTSFB_CCK;

		if (rates[0].flags & IEEE80211_TX_RC_USE_RTS_CTS &&
		    fill_phy_ctl1) {
			txhdr->phy_ctl1_rts = cpu_to_le16(
				b43_generate_tx_phy_ctl1(dev, rts_rate));
			txhdr->phy_ctl1_rts_fb = cpu_to_le16(
				b43_generate_tx_phy_ctl1(dev, rts_rate_fb));
		}
	}

	/* Magic cookie */
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		txhdr->format_598.cookie = cpu_to_le16(cookie);
		break;
	case B43_FW_HDR_351:
		txhdr->format_351.cookie = cpu_to_le16(cookie);
		break;
	case B43_FW_HDR_410:
		txhdr->format_410.cookie = cpu_to_le16(cookie);
		break;
	}

	if (fill_phy_ctl1) {
		txhdr->phy_ctl1 =
			cpu_to_le16(b43_generate_tx_phy_ctl1(dev, rate));
		txhdr->phy_ctl1_fb =
			cpu_to_le16(b43_generate_tx_phy_ctl1(dev, rate_fb));
	}

	/* Apply the bitfields */
	txhdr->mac_ctl = cpu_to_le32(mac_ctl);
	txhdr->phy_ctl = cpu_to_le16(phy_ctl);
	txhdr->extra_ft = extra_ft;

	return 0;
}

static s8 b43_rssi_postprocess(struct b43_wldev *dev,
			       u8 in_rssi, int ofdm,
			       int adjust_2053, int adjust_2050)
{
	struct b43_phy *phy = &dev->phy;
	struct b43_phy_g *gphy = phy->g;
	s32 tmp;

	switch (phy->radio_ver) {
	case 0x2050:
		if (ofdm) {
			tmp = in_rssi;
			if (tmp > 127)
				tmp -= 256;
			tmp *= 73;
			tmp /= 64;
			if (adjust_2050)
				tmp += 25;
			else
				tmp -= 3;
		} else {
			if (dev->dev->bus_sprom->
			    boardflags_lo & B43_BFL_RSSI) {
				if (in_rssi > 63)
					in_rssi = 63;
				B43_WARN_ON(phy->type != B43_PHYTYPE_G);
				tmp = gphy->nrssi_lt[in_rssi];
				tmp = 31 - tmp;
				tmp *= -131;
				tmp /= 128;
				tmp -= 57;
			} else {
				tmp = in_rssi;
				tmp = 31 - tmp;
				tmp *= -149;
				tmp /= 128;
				tmp -= 68;
			}
			if (phy->type == B43_PHYTYPE_G && adjust_2050)
				tmp += 25;
		}
		break;
	case 0x2060:
		if (in_rssi > 127)
			tmp = in_rssi - 256;
		else
			tmp = in_rssi;
		break;
	default:
		tmp = in_rssi;
		tmp -= 11;
		tmp *= 103;
		tmp /= 64;
		if (adjust_2053)
			tmp -= 109;
		else
			tmp -= 83;
	}

	return (s8) tmp;
}

/*
 * acphy_psfix, the station power save frames of an AP on core rev
 * >= 40. ucode 907 hands them up like any other frame to our address: a
 * PS-Poll is acked and passed (L843 @0x0F1B -> L501 @0x090A, no KEEP_CTL
 * needed), a null or QoS null data frame takes the data path (L749
 * @0x0D33: 28 bytes including the FCS is exactly a null frame), and the
 * PM bit is bit 12 of the frame control the ucode reads at [0x03,off1].
 * Bit 1 logs them with their PM bit, mac80211's only input for the
 * station's state without AP_LINK_PS. With bit 4 (AP_LINK_PS) mac80211
 * leaves PS-Poll and U-APSD trigger frames to the driver
 * (ieee80211_rx_h_uapsd_and_pspoll), so they are reported here the way
 * mac80211 itself would: ieee80211_sta_pspoll for a PS-Poll,
 * ieee80211_sta_uapsd_trigger for a QoS data or QoS null frame with the
 * PM bit set and no more fragments.
 */
static void b43_ac_psfix_rx(struct b43_wldev *dev, struct sk_buff *skb)
{
	struct b43_wl *wl = dev->wl;
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	__le16 fc = hdr->frame_control;
	struct ieee80211_sta *sta;
	bool pspoll = ieee80211_is_pspoll(fc);

	/* addr2 of a PS-Poll (its TA) ends at byte 16 */
	if (skb->len < 16 || !wl->vif || !b43_is_mode(wl, NL80211_IFTYPE_AP))
		return;
	if (!b43_ac_psfix(dev, B43_AC_PSFIX_LINKPS))
		return;
	if (!pspoll &&
	    (!ieee80211_is_data_qos(fc) || !ieee80211_has_pm(fc) ||
	     ieee80211_has_morefrags(fc) || skb->len < ieee80211_hdrlen(fc)))
		return;

	rcu_read_lock();
	sta = ieee80211_find_sta(wl->vif, hdr->addr2);
	if (sta) {
		local_bh_disable();
		if (pspoll)
			ieee80211_sta_pspoll(sta);
		else
			ieee80211_sta_uapsd_trigger(sta, ieee80211_get_tid(hdr));
		local_bh_enable();
	}
	rcu_read_unlock();
}

void b43_rx(struct b43_wldev *dev, struct sk_buff *skb, const void *_rxhdr)
{
	struct ieee80211_rx_status status;
	struct b43_plcp_hdr6 *plcp;
	struct ieee80211_hdr *wlhdr;
	const struct b43_rxhdr_fw4 *rxhdr = _rxhdr;
	__le16 fctl;
	u16 phystat0, phystat3;
	u16 chanstat, mactime;
	u32 macstat;
	u16 chanid;
	u8 phytype;
	bool band5;
	bool noplcp;
	/* QUIET: no per-frame counters, dumps or their atomics */
	bool quiet = b43_ac_fastpath(dev, B43_AC_FASTPATH_QUIET);
	int padding, rate_idx;

	memset(&status, 0, sizeof(status));

	/* Get metadata about the frame from the header. */
	phystat0 = le16_to_cpu(rxhdr->phy_status0);
	phystat3 = le16_to_cpu(rxhdr->phy_status3);
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		macstat = le32_to_cpu(rxhdr->format_598.mac_status);
		mactime = le16_to_cpu(rxhdr->format_598.mac_time);
		chanstat = le16_to_cpu(rxhdr->format_598.channel);
		break;
	case B43_FW_HDR_410:
	case B43_FW_HDR_351:
		macstat = le32_to_cpu(rxhdr->format_351.mac_status);
		mactime = le16_to_cpu(rxhdr->format_351.mac_time);
		chanstat = le16_to_cpu(rxhdr->format_351.channel);
		break;
	}

	if (unlikely(macstat & B43_RX_MAC_FCSERR)) {
		dev->wl->ieee_stats.dot11FCSErrorCount++;
		status.flag |= RX_FLAG_FAILED_FCS_CRC;
	}
	if (unlikely(phystat0 & (B43_RX_PHYST0_PLCPHCF | B43_RX_PHYST0_PLCPFV)))
		status.flag |= RX_FLAG_FAILED_PLCP_CRC;
	if ((phystat0 & B43_RX_PHYST0_SHORTPRMBL) &&
	    !(dev->dev->core_rev >= 40 &&
	      ((phystat0 & B43_RX_PHYST0_FTYPE) == B43_RX_PHYST0_AC_HT ||
	       (b43_modparam_ac_vht &&
		(phystat0 & B43_RX_PHYST0_FTYPE) == B43_RX_PHYST0_AC_VHT))))
		status.enc_flags |= RX_ENC_FLAG_SHORTPRE;
	if (macstat & B43_RX_MAC_DECERR) {
		/* Decryption with the given key failed.
		 * Drop the packet. We also won't be able to decrypt it with
		 * the key in software. */
		goto drop;
	}

	/* Skip PLCP and padding */
	padding = (macstat & B43_RX_MAC_PADDING) ? 2 : 0;
	if (unlikely(skb->len < (sizeof(struct b43_plcp_hdr6) + padding))) {
		b43dbg(dev->wl, "RX: Packet size underrun (1)\n");
		goto drop;
	}
	plcp = (struct b43_plcp_hdr6 *)(skb->data + padding);
	skb_pull(skb, sizeof(struct b43_plcp_hdr6) + padding);
	/* The skb contains the Wireless Header + payload data now */
	if (unlikely(skb->len < (2 + 2 + 6 /*minimum hdr */  + FCS_LEN))) {
		b43dbg(dev->wl, "RX: Packet size underrun (2)\n");
		goto drop;
	}
	wlhdr = (struct ieee80211_hdr *)(skb->data);
	fctl = wlhdr->frame_control;

	/*
	 * an MPDU after the first of an A-MPDU has no PLCP (see
	 * b43_modparam_acphy_ampdu). Without a valid PHY RX status, the PHY
	 * status flags taken above are not this MPDU's: the FCS decides.
	 */
	noplcp = b43_ac_ampdu(dev, B43_AC_AMPDU_RX) &&
		 !(plcp->raw[0] | plcp->raw[1] | plcp->raw[2]);
	if (noplcp && !(macstat & B43_RX_MAC_RXST_VALID)) {
		status.flag &= ~RX_FLAG_FAILED_PLCP_CRC;
		status.enc_flags &= ~RX_ENC_FLAG_SHORTPRE;
	}

	if ((macstat & B43_RX_MAC_DEC) && dev->dev->core_rev >= 40 &&
	    b43_txhdr_is_ac(dev)) {
		/*
		 * ucode 907 decrypted the frame with the slot it found
		 * (A2 match index + 4 @0x0AA8..0x0AAE, or a default slot); a
		 * DECERR frame was dropped above. IV and MIC stay in the frame
		 * and mac80211 strips them (RX_FLAG_DECRYPTED), as on the
		 * legacy path. The status key index (bits 5..10) is only
		 * logged: 6 bits cannot name every slot and nothing here
		 * depends on it.
		 */
		if (b43_ac_hwcrypto(dev, B43_AC_HWCRYPTO_CCMP |
					 B43_AC_HWCRYPTO_WEP)) {
			status.flag |= RX_FLAG_DECRYPTED;
			if (dev->ac_hwcrypto_rxlogs < B43_AC_HWCRYPTO_RXLOGS) {
				dev->ac_hwcrypto_rxlogs++;
			}
		}
	} else if (macstat & B43_RX_MAC_DEC) {
		unsigned int keyidx;
		int wlhdr_len;

		keyidx = ((macstat & B43_RX_MAC_KEYIDX)
			  >> B43_RX_MAC_KEYIDX_SHIFT);
		/* We must adjust the key index here. We want the "physical"
		 * key index, but the ucode passed it slightly different.
		 */
		keyidx = b43_kidx_to_raw(dev, keyidx);
		if (B43_WARN_ON(keyidx >= ARRAY_SIZE(dev->key)))
			goto drop;

		if (dev->key[keyidx].algorithm != B43_SEC_ALGO_NONE) {
			wlhdr_len = ieee80211_hdrlen(fctl);
			if (unlikely(skb->len < (wlhdr_len + 3))) {
				b43dbg(dev->wl,
				       "RX: Packet size underrun (3)\n");
				goto drop;
			}
			status.flag |= RX_FLAG_DECRYPTED;
		}
	}

	/*
	 * Core rev >= 40 (AC-PHY) ucode writes the channel word as a chanspec,
	 * not a chanstat: wl.ko skips its chanstat->chanspec conversion for
	 * corerev > 39 (bhi at 0x12874). A chanspec carries no PHY type, so
	 * the PHY-type decode below is skipped for it. The channel is the low
	 * byte; the band comes from the channel number because the 0xc000
	 * band bits (tested by wl.ko at 0x8050) are still undecoded.
	 */
	if (dev->dev->core_rev >= 40) {
		phytype = B43_PHYTYPE_AC;
		chanid = chanstat & B43_RX_CHANSPEC_CHAN;
		band5 = chanid > 14;
		/*
		 * A 40 or 80 MHz chanspec names its center channel; mac80211
		 * wants the control channel. wf_chspec_ctlchan @0x7840: center
		 * - 2 + 4 * sideband at 40 MHz, center - 6 + 4 * sideband at
		 * 80 MHz.
		 */
		chanid = b43_ac_chanspec_ctlchan(chanstat);
	} else {
		phytype = chanstat & B43_RX_CHAN_PHYTYPE;
		chanid = (chanstat & B43_RX_CHAN_ID) >> B43_RX_CHAN_ID_SHIFT;
		band5 = !!(chanstat & B43_RX_CHAN_5GHZ);
	}

	/* Link quality statistics */
	switch (phytype) {
	case B43_PHYTYPE_HT:
		/* TODO: is max the right choice? */
		status.signal = max_t(__s8,
			max(rxhdr->phy_ht_power0, rxhdr->phy_ht_power1),
			rxhdr->phy_ht_power2);
		break;
	case B43_PHYTYPE_N:
		/* Broadcom has code for min and avg, but always uses max */
		if (rxhdr->power0 == 16 || rxhdr->power0 == 32)
			status.signal = max(rxhdr->power1, rxhdr->power2);
		else
			status.signal = max(rxhdr->power0, rxhdr->power1);
		break;
	case B43_PHYTYPE_B:
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_LP:
		status.signal = b43_rssi_postprocess(dev, rxhdr->jssi,
						  (phystat0 & B43_RX_PHYST0_OFDM),
						  (phystat0 & B43_RX_PHYST0_GAINCTL),
						  (phystat3 & B43_RX_PHYST3_TRSTATE));
		break;
#ifdef CPTCFG_B43_PHY_AC
	case B43_PHYTYPE_AC: {
		/*
		 * Core rev >= 40: one signed byte per core at 9, 10, 11 (the
		 * HT-PHY bytes), valid only with the PHY RX status valid bit
		 * (wl.ko wlc_phy_rssi_compute 0x644..0x65c).
		 */
		u8 raw[3] = { rxhdr->phy_ht_power0, rxhdr->phy_ht_power1,
			      rxhdr->phy_ht_power2 };
		s8 dbm;

		if (dev->phy.type == B43_PHYTYPE_AC &&
		    (macstat & B43_RX_MAC_RXST_VALID) &&
		    !b43_phy_ac_rssi(dev, raw, &dbm))
			status.signal = dbm;
		else
			status.flag |= RX_FLAG_NO_SIGNAL_VAL;
		break;
	}
#endif
	}

	if (noplcp) {
		/* rate of the A-MPDU's first MPDU */
		rate_idx = -1;
		if (b43_ac_ampdu_last.valid) {
			rate_idx = b43_ac_ampdu_last.rate_idx;
			status.encoding = b43_ac_ampdu_last.encoding;
			status.bw = b43_ac_ampdu_last.bw;
			status.enc_flags = b43_ac_ampdu_last.enc_flags;
			if ((status.flag & RX_FLAG_NO_SIGNAL_VAL) &&
			    b43_ac_ampdu_last.signal_valid) {
				status.signal = b43_ac_ampdu_last.signal;
				status.flag &= ~RX_FLAG_NO_SIGNAL_VAL;
			}
		}
		if (!quiet)
			atomic_inc(&b43_ac_ampdu_rxn);
		if (!quiet && atomic_inc_return(&b43_ac_ampdu_rxdumps) <=
		    B43_AC_AMPDU_DUMPS)
			b43info(dev->wl, "AMPDURX %d physt0 %04x physt3 %04x macst %08x fc %04x seq %u len %u rate %d flag 0x%x\n",
				atomic_read(&b43_ac_ampdu_rxn), phystat0,
				phystat3, macstat, le16_to_cpu(fctl),
				skb->len >= 24 ?
				le16_to_cpu(wlhdr->seq_ctrl) >> 4 : 0,
				skb->len, rate_idx, status.flag);
	} else if (dev->dev->core_rev >= 40 &&
	    (phystat0 & B43_RX_PHYST0_FTYPE) == B43_RX_PHYST0_AC_HT) {
		/*
		 * HT frame on core rev >= 40, decoded as
		 * wlc_recv_compute_rspec does (0x148a0..0x148dc) from the
		 * HT-SIG in the 6 PLCP bytes: byte 0 bits 0..6 MCS, bit 7
		 * 40 MHz; bytes 3..4 bit 7 short GI, bit 6 LDPC, bits 4..5
		 * STBC.
		 */
		u8 sig2 = plcp->raw[3];

		rate_idx = plcp->raw[0] & 0x7F;
		if (rate_idx <= 76)
			status.encoding = RX_ENC_HT;
		else
			rate_idx = -1;
		status.bw = (plcp->raw[0] & B43_AC_HTSIG1_CBW40) ?
			    RATE_INFO_BW_40 : RATE_INFO_BW_20;
		if (sig2 & B43_AC_HTSIG2_SGI)
			status.enc_flags |= RX_ENC_FLAG_SHORT_GI;
		if (sig2 & B43_AC_HTSIG2_LDPC)
			status.enc_flags |= RX_ENC_FLAG_LDPC;
		status.enc_flags |= ((sig2 & B43_AC_HTSIG2_STBC) >> 4) <<
				    RX_ENC_FLAG_STBC_SHIFT;
	} else if (dev->dev->core_rev >= 40 && b43_modparam_ac_vht &&
		   (phystat0 & B43_RX_PHYST0_FTYPE) == B43_RX_PHYST0_AC_VHT) {
		/*
		 * VHT frame (PHY status 0 type 3), acphy_vht: VHT-SIG-A from
		 * the 6 PLCP bytes as wlc_vht_get_rspec_from_plcp 0x181358 and
		 * wlc_vht_get_rate_from_plcp 0x11979c read it: A1 bits 0..1
		 * bandwidth, bit 3 STBC, bits 10..12 NSTS - 1 (NSS = NSTS / 2
		 * with STBC); byte 3 bits 4..7 MCS, bit 0 short GI, bit 2 LDPC.
		 */
		u32 a1 = plcp->raw[0] | (plcp->raw[1] << 8) |
			 (plcp->raw[2] << 16);
		u8 sig2 = plcp->raw[3];
		bool stbc = a1 & B43_AC_VHTSIGA1_STBC;
		unsigned int nss = ((a1 >> B43_AC_VHTSIGA1_NSTS_SHIFT) & 7) + 1;
		static const u8 vht_bw[4] = {
			RATE_INFO_BW_20, RATE_INFO_BW_40, RATE_INFO_BW_80,
			RATE_INFO_BW_160,
		};

		if (stbc)
			nss /= 2;
		rate_idx = sig2 >> B43_AC_VHTSIGA2_MCS_SHIFT;
		if (rate_idx <= 9 && nss >= 1) {
			status.encoding = RX_ENC_VHT;
			status.nss = nss;
			status.bw = vht_bw[a1 & 3];
			if (sig2 & B43_AC_VHTSIGA2_SGI)
				status.enc_flags |= RX_ENC_FLAG_SHORT_GI;
			if (sig2 & B43_AC_VHTSIGA2_LDPC)
				status.enc_flags |= RX_ENC_FLAG_LDPC;
			if (stbc)
				status.enc_flags |= 1 << RX_ENC_FLAG_STBC_SHIFT;
		} else {
			rate_idx = -1;
		}
	} else if (phystat0 & B43_RX_PHYST0_OFDM)
		rate_idx = b43_plcp_get_bitrate_idx_ofdm(plcp, band5);
	else
		rate_idx = b43_plcp_get_bitrate_idx_cck(plcp);
	if (unlikely(rate_idx == -1)) {
		/* PLCP seems to be corrupted.
		 * Drop the frame, if we are not interested in corrupted frames. */
		if (!(dev->wl->filter_flags & FIF_PLCPFAIL))
			goto drop;
	}
	status.rate_idx = rate_idx;
	status.antenna = !!(phystat0 & B43_RX_PHYST0_ANT);
	/* remember the rate for the MPDUs that follow without a PLCP */
	if (b43_ac_ampdu(dev, B43_AC_AMPDU_RX) && !noplcp && rate_idx >= 0 &&
	    !(status.flag & (RX_FLAG_FAILED_FCS_CRC |
			     RX_FLAG_FAILED_PLCP_CRC))) {
		b43_ac_ampdu_last.rate_idx = rate_idx;
		b43_ac_ampdu_last.encoding = status.encoding;
		b43_ac_ampdu_last.bw = status.bw;
		b43_ac_ampdu_last.enc_flags = status.enc_flags;
		b43_ac_ampdu_last.signal = status.signal;
		b43_ac_ampdu_last.signal_valid =
			!(status.flag & RX_FLAG_NO_SIGNAL_VAL);
		b43_ac_ampdu_last.valid = true;
	}

	/*
	 * All frames on monitor interfaces and beacons always need a full
	 * 64-bit timestamp. Monitor interfaces need it for diagnostic
	 * purposes and beacons for IBSS merging.
	 * This code assumes we get to process the packet within 16 bits
	 * of timestamp, i.e. about 65 milliseconds after the PHY received
	 * the first symbol.
	 */
	if (ieee80211_is_beacon(fctl) || dev->wl->radiotap_enabled) {
		u16 low_mactime_now;

		b43_tsf_read(dev, &status.mactime);
		low_mactime_now = status.mactime;
		status.mactime = status.mactime & ~0xFFFFULL;
		status.mactime += mactime;
		if (low_mactime_now <= mactime)
			status.mactime -= 0x10000;
		status.flag |= RX_FLAG_MACTIME_START;
	}

	switch (phytype) {
	case B43_PHYTYPE_G:
		status.band = NL80211_BAND_2GHZ;
		/* Somewhere between 478.104 and 508.1084 firmware for G-PHY
		 * has been modified to be compatible with N-PHY and others.
		 */
		if (dev->fw.rev >= 508)
			status.freq = ieee80211_channel_to_frequency(chanid, status.band);
		else
			status.freq = chanid + 2400;
		break;
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LP:
	case B43_PHYTYPE_HT:
		/* chanid is the SHM channel cookie. Which is the plain
		 * channel number in b43. */
		if (band5)
			status.band = NL80211_BAND_5GHZ;
		else
			status.band = NL80211_BAND_2GHZ;
		status.freq =
			ieee80211_channel_to_frequency(chanid, status.band);
		break;
	case B43_PHYTYPE_AC:
		status.band = band5 ? NL80211_BAND_5GHZ : NL80211_BAND_2GHZ;
		status.freq =
			ieee80211_channel_to_frequency(chanid, status.band);
		break;
	default:
		B43_WARN_ON(1);
		goto drop;
	}

	if (b43_modparam_acphy_psfix && dev->dev->core_rev >= 40 &&
	    !(status.flag & (RX_FLAG_FAILED_FCS_CRC | RX_FLAG_FAILED_PLCP_CRC)))
		b43_ac_psfix_rx(dev, skb);

	memcpy(IEEE80211_SKB_RXCB(skb), &status, sizeof(status));
	if (dev->ac_rx_list) {
		/*
		 * RXLIST: inside b43_dma_rx's drain, bottom halves off;
		 * ieee80211_rx_napi's steps, the netif list delivered once per
		 * drain. The RCU section is the one ieee80211_rx_napi takes.
		 */
		rcu_read_lock();
		ieee80211_rx_list(dev->wl->hw, NULL, skb, dev->ac_rx_list);
		rcu_read_unlock();
	} else {
		ieee80211_rx_ni(dev->wl->hw, skb);
	}

#if B43_DEBUG
	dev->rx_count++;
#endif
	return;
drop:
	dev_kfree_skb_any(skb);
}

static int b43_modparam_ac56_txslog;
module_param_named(ac56_txslog, b43_modparam_ac56_txslog, int, 0644);
MODULE_PARM_DESC(ac56_txslog, "log the first N TX status reports (debug)");

void b43_handle_txstatus(struct b43_wldev *dev,
			 const struct b43_txstatus *status)
{
	b43_debugfs_log_txstat(dev, status);
	if (b43_modparam_ac56_txslog > 0) {
		b43_modparam_ac56_txslog--;
		b43info(dev->wl, "txstatus: cookie 0x%04x seq %u phy_stat 0x%02x frames %u rts %u supp %u acked %u inter %u ampdu %u\n",
			status->cookie, status->seq, status->phy_stat, status->frame_count,
			status->rts_count, status->supp_reason, status->acked,
			status->intermediate, status->for_ampdu);
	}

	if (status->intermediate)
		return;
	if (status->for_ampdu)
		return;
	if (!status->acked)
		dev->wl->ieee_stats.dot11ACKFailureCount++;
	if (status->rts_count) {
		if (status->rts_count == 0xF)	//FIXME
			dev->wl->ieee_stats.dot11RTSFailureCount++;
		else
			dev->wl->ieee_stats.dot11RTSSuccessCount++;
	}

	if (b43_using_pio_transfers(dev))
		b43_pio_handle_txstatus(dev, status);
	else
		b43_dma_handle_txstatus(dev, status);

	b43_phy_txpower_check(dev, 0);
}

/*
 * Decode a core rev >= 40 TX status (ucode 907.113) into b43's struct.
 * s[0..3] are the first package (0x170..0x17c), t[0..3] the second.
 * Field map from wlc_bmac_txstatus @0x12270, rev > 39 branch at 0x123a0.
 */
void b43_txstatus_parse_rev40(struct b43_wldev *dev, struct b43_txstatus *stat,
			      const u32 *s, const u32 *t)
{
	u16 lo = s[0] & 0xFFFF;
	unsigned int cnt, frames;

	memset(stat, 0, sizeof(*stat));

	/* Frame id: s1 >> 16 (0x12408, 0x12414) */
	stat->cookie = s[0] >> 16;
	/* Sequence: s2 & 0xffff (0x12464) */
	stat->seq = s[1] & 0xFFFF;
	/* PHY status: s2 bits 16..23 (0x12418..0x1241c) */
	stat->phy_stat = (s[1] >> 16) & 0xFF;
	stat->intermediate = !!(lo & B43_TXST40_S1_INTERMEDIATE);
	stat->pm_indicated = !!(lo & B43_TXST40_S1_PM_INDICATED);
	stat->supp_reason = (lo & B43_TXST40_S1_SUPP_REASON) >>
			    B43_TXST40_S1_SUPP_REASON_SHIFT;
	/*
	 * Acked: bit 15, also set when the 7 bit count in bits 8..14 is above
	 * 1 (0x12430..0x12450). Why the count implies an ACK is open; this
	 * copies wl.
	 */
	cnt = (lo & B43_TXST40_S1_CNT) >> B43_TXST40_S1_CNT_SHIFT;
	stat->acked = !!(lo & B43_TXST40_S1_ACKED) || cnt > 1;
	/*
	 * AMPDU: not in the status on rev > 39; wl asks its TX header
	 * (wlc_txh_get_isAMPDU, 0x9880). dma.c reads the header bit from
	 * its header cache; for_ampdu stays 0 so b43_handle_txstatus passes
	 * the status on. The count is the MPDUs this status releases
	 * (wlc_ampdu_dotxstatus 0xe45c..0xe464), the block ack map is t2:t3
	 * by position (0xe868..0xe8d8).
	 */
	stat->for_ampdu = 0;
	stat->ampdu_cnt = cnt;
	stat->ampdu_ba = (u64)t[1] | ((u64)t[2] << 32);
	/* Frame count: s3[7:0] + s4[7:0] + s3[23:16] + s4[23:16] (0x12454..0x12478) */
	stat->rate_cnt[0] = s[2] & 0xFF;
	stat->rate_cnt[1] = (s[2] >> 16) & 0xFF;
	stat->rate_cnt[2] = s[3] & 0xFF;
	stat->rate_cnt[3] = (s[3] >> 16) & 0xFF;
	frames = (s[2] & 0xFF) + (s[3] & 0xFF) +
		 ((s[2] >> 16) & 0xFF) + ((s[3] >> 16) & 0xFF);
	/*
	 * ucode 1058 (core rev 56): an MPDU sent once and acked reports
	 * s3 = 0x00010001 and the air shows a single copy without the retry
	 * bit (checked with a monitor on the AP), so bits 16..23 are not a
	 * second rate's TX count there. Count only the low bytes, or every
	 * frame looks retried once and minstrel backs off for nothing.
	 */
	if (dev->dev->core_rev == 56) {
		frames = (s[2] & 0xFF) + (s[3] & 0xFF);
		stat->rate_cnt[1] = 0;
		stat->rate_cnt[3] = 0;
	}
	stat->frame_count = min_t(unsigned int, frames, 0xFF);
	if (b43_modparam_ac56_txslog > 0)
		pr_info("b43 txstat raw: s %08x %08x %08x %08x t %08x %08x %08x\n",
			s[0], s[1], s[2], s[3], t[0], t[1], t[2]);
	/* RTS count: t1 bits 16..23 (0x12508, 0x12530) */
	stat->rts_count = (t[0] >> 16) & 0xFF;
	/*
	 * Not used: t1 bits 24..31 (txs+14, 0x1250c), the raw halves of s3,
	 * s4, t1..t4 (txs+18..+40, 0x124cc..0x12528) and the TSF that wl
	 * reads from 0x180 once per call (0x123b8). Their meanings are open.
	 */
}

/* Fill out the mac80211 TXstatus report based on the b43-specific
 * txstatus report data. This returns a boolean whether the frame was
 * successfully transmitted. */
bool b43_fill_txstatus_report(struct b43_wldev *dev,
			      struct ieee80211_tx_info *report,
			      const struct b43_txstatus *status)
{
	bool frame_success = true;
	int retry_limit;

	/* preserve the confiured retry limit before clearing the status
	 * The xmit function has overwritten the rc's value with the actual
	 * retry limit done by the hardware */
	retry_limit = report->status.rates[0].count;
	ieee80211_tx_info_clear_status(report);

	/*
	 * ucode 907 did not send the frame because the PMQ holds its
	 * station as dozing (suppress reason 1, L247 @0x0409). mac80211
	 * requeues a filtered frame into the station's power save buffer
	 * (ieee80211_handle_filtered_frame), as wl's
	 * wlc_apps_suppr_frame_enq 0x152b24 does.
	 */
	if (b43_ac_psfix(dev, B43_AC_PSFIX_PMQ) && b43_txhdr_is_ac(dev) &&
	    !status->acked && status->supp_reason == B43_TXST40_SUPR_PMQ) {
		report->flags |= IEEE80211_TX_STAT_TX_FILTERED;
		report->status.rates[0].count = 0;
		return false;
	}

	/* HT TX liveness of this core init, rev >= 40 AC header only */
	if (b43_txhdr_is_ac(dev) && b43_txstatus_is_rev40(dev))
		b43_ac_htdead_txstatus(dev, report, status);

	if (status->acked) {
		/* The frame was ACKed. */
		report->flags |= IEEE80211_TX_STAT_ACK;
	} else {
		/* The frame was not ACKed... */
		if (!(report->flags & IEEE80211_TX_CTL_NO_ACK)) {
			/* ...but we expected an ACK. */
			frame_success = false;
		}
	}
	if (status->frame_count == 0) {
		/* The frame was not transmitted at all. */
		report->status.rates[0].count = 0;
	} else if (b43_txhdr_is_ac(dev) &&
		   (report->status.rates[0].flags & IEEE80211_TX_RC_MCS)) {
		/*
		 * AC header with an MCS rate: one rate entry and no fallback
		 * (b43_generate_txhdr_ac), so every attempt was at rates[0].
		 * The rev >= 40 status carries no rate; wl's rate selection
		 * also takes only the frame count and the ACK bit from it
		 * (wlc_ratesel_upd_txstatus_normalack 0x18368c..0x1836d8, txs
		 * bytes 12 and 9) and the MCS from its own TX header
		 * (wlc_txh_info_get_mcs 0x1197bc, call 0xa334).
		 */
		report->status.rates[0].count = status->frame_count;
		report->status.rates[1].idx = -1;
	} else if (status->rts_count > dev->wl->hw->conf.short_frame_max_tx_count) {
		/*
		 * If the short retries (RTS, not data frame) have exceeded
		 * the limit, the hw will not have tried the selected rate,
		 * but will have used the fallback rate instead.
		 * Don't let the rate control count attempts for the selected
		 * rate in this case, otherwise the statistics will be off.
		 */
		report->status.rates[0].count = 0;
		report->status.rates[1].count = status->frame_count;
	} else {
		if (status->frame_count > retry_limit) {
			report->status.rates[0].count = retry_limit;
			report->status.rates[1].count = status->frame_count -
					retry_limit;

		} else {
			report->status.rates[0].count = status->frame_count;
			report->status.rates[1].idx = -1;
		}
	}

	return frame_success;
}

/* Stop any TX operation on the device (suspend the hardware queues) */
void b43_tx_suspend(struct b43_wldev *dev)
{
	if (b43_using_pio_transfers(dev))
		b43_pio_tx_suspend(dev);
	else
		b43_dma_tx_suspend(dev);
}

/* Resume any TX operation on the device (resume the hardware queues) */
void b43_tx_resume(struct b43_wldev *dev)
{
	if (b43_using_pio_transfers(dev))
		b43_pio_tx_resume(dev);
	else
		b43_dma_tx_resume(dev);
}
