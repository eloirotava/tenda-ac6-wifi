// SPDX-License-Identifier: GPL-2.0-or-later
/*

  Broadcom B43 wireless driver
  IEEE 802.11ac AC-PHY support (BCM4360, Radio 2069)

  Ported from Broadcom wl.ko (Netgear R6400 firmware, v1.0.4.85) —
  Register sequences in wlc_phy_switch_radio_acphy (radio 0x2069) and
  wlc_phy_radio20691_pwron_seq / _vcocal (radio 0x30b).

*/

#ifndef LINUX_B43_PHY_AC_H_
#define LINUX_B43_PHY_AC_H_

#include <linux/workqueue.h>

struct b43_phy_ac {
	/*
	 * RC cal results, the u16 fields wl keeps at pi_ac+0x372 and
	 * pi_ac+0x374 (rc_result[0], rc_result[1]). wlc_phy_attach_acphy
	 * presets both to 0x80 (0x45bc8, 0x45bd0); the 2069 RC cal in
	 * wlc_phy_switch_radio_acphy stores the same value in both (0x4f6d8,
	 * 0x4f6e0). Read by wlc_phy_set_tbl_on_reset_acphy into ACPHY table 7
	 * on the channel set's init pass.
	 */
	u16 rc_result[2];
	/* RC cal pass 2 result, the byte at pi_ac+0x376 (0x4f860): dacbuf cap */
	u8 rc_dacbuf;
	/* RC cal ran to the end (each poll may still have timed out) */
	bool rccal_done;
	/* PHY core count, wl's pi+196: PHY reg 0x00b & 7 (0x45dfc..0x45e08) */
	u8 ncores;
	/* core enable mask, wl's sh+147 = NVRAM rxchain */
	u8 coremask;
	/* analog power-on completed; wl's radio-on flag pi+0x4bdc */
	bool pwron_done;

	/*
	 * Channel-set state, wlc_phy_chanspec_set_acphy.
	 * chan_init is wl's pi_ac[0x368]: set after
	 * power-on, it makes the next set run the reset-time tables, femctrl,
	 * hwaci, band and bandwidth steps. A set that stops at a gap leaves it
	 * set, so the next attempt runs them again.
	 */
	bool chan_init;
	/* pi_ac[0x36a]: band of the last set, true for 2 GHz (0 at attach) */
	bool band_2g;
	/* pi_ac[0x36c]: chanspec & 0x3800 of the last set */
	u16 bw;
	/*
	 * [pi,#222]: the bandwidth state wlc_phy_bw_state_set @0x5544c keeps
	 * and wlc_phy_clk_bwbits turns into core IOCTL bits 6..7; 0x1000 at
	 * attach (0x68ed0), which matches b43's 20 MHz PHY reset.
	 */
	u16 bw_state;
	/* pi+218: current chanspec (wlc_phy_chanspec_radio_set @0x55490) */
	u16 chanspec;
	/*
	 * acphy_initorder bit 4: chanspec of the last set that reached the end
	 * of its tail; 0 after a core reset, a radio-off or a failed set.
	 */
	u16 done_chanspec;
	/* pi_ac[12]: wlc_phy_stay_in_carriersearch_acphy nesting count */
	u16 cs_count;

	/*
	 * RX gain state the channel set keeps per core, wl's pi_ac arrays
	 * (wlc_phy_rxgainctrl_set_gaintbls_acphy
	 * @0x27bd0). Stage s is eLNA, LNA1, LNA2, mixer, BIQ0, BIQ1.
	 */
	s8 rx_gain[3][6][10];	/* pi_ac[0x4ef + 120c + 10s + i], dB */
	u8 rx_gainbits[3][6][10]; /* pi_ac[0x52b + 120c + 10s + i] */
	u8 rx_count[6];		/* pi_ac[0x6cf + s], {2,6,7,10,8,8} at attach */
	u8 rx_limit[6];		/* pi_ac[0x6d5 + s], set by gainctrl */
	u8 rx_elna[3];		/* pi_ac[0x4df + 4c], eLNA gain */
	u8 rx_triso[3];		/* pi_ac[0x4e0 + 4c], TR isolation */
	u8 rx_trelnabyp[3];	/* pi_ac[0x4e1 + 4c] */
	u8 rx_lna1byp[3];	/* pi_ac[0x4e2 + 4c], boardflags2 bit 31 on 5 GHz */
	u8 rx_max_lna;		/* pi_ac[0x6e2], get_max_lna_index(pi, 0) */
	/*
	 * pi_ac[0x6ef..0x6f8], the total desense block that
	 * wlc_phy_desense_calc_total_acphy @0x171fc fills:
	 * [7] eLNA bypass (0x6f6), [8] low-sens offset (0x6f7), [9] desense on
	 * (0x6f8).
	 */
	u8 desense_total[10];
	/*
	 * pi_ac[0x6dd..0x6e0]: the LNA desense offsets last applied by
	 * upd_lna1_lna2_gains, copies of desense_total[2..5]
	 * that wlc_phy_desense_apply_acphy compares.
	 */
	u8 desense_lna[4];
	/*
	 * pi_ac[16 + c]: TX power index (table 0x20 row) last applied per core,
	 * 0 at attach. The channel set and
	 * adjust_txpower apply the row from the board's TX power target, not wl's 0.
	 */
	s8 txpwr_idx[3];
	/*
	 * acphy_rateppr: per-rate TX power offset in half dB below the row's
	 * target, the stf+0x2c ppr wlc_update_txppr_offset @0x193e20 builds
	 * and wlc_stf_get_204080_pwrs @0x193990 reads per frame into PHY
	 * control 1 bits 3..7. Indexed by B43_AC_PWROFF_*; all 0 unless
	 * acphy_rateppr is set and the last applied target computation
	 * succeeded. Written with the rows (adjust_txpower, channel set).
	 */
	u8 pwroff[28];
	/*
	 * pi_ac[24 + 4*sb + c]: stored noise level per 5 GHz sub-band and core
	 * that wlc_phy_crs_min_pwr_cal_acphy reads; -30 at attach for PHY rev
	 * <= 1 (0x463c4..0x463fc).
	 */
	s8 crs_nf[5][4];
	/*
	 * pi+0x136..0x14e: five RX gain error groups (2 GHz, then 5 GHz
	 * channels 15..48, 49..64, 65..128, above), cores 0..2 absolute in
	 * bytes 0..2, byte 3 unused, byte 4 the all -1 flag; filled at radio-on
	 * as wl's attach does.
	 */
	u8 rxgainerr[5][5];
	/* pi + 0x166 + c: RX gain error per core */
	u8 rx_gainerr[3];
	/*
	 * pi_ac[0x3f6 + 12c + 3sb + bw]: NVRAM rssicorrnorm5g_c<c>, an RSSI
	 * offset per core, 5 GHz sub-band (1..4 as sb 0..3) and bandwidth
	 * (20/40/80); 0 for a missing key.
	 */
	s8 rssicorr5g[3][4][3];
	/*
	 * Byte pi+0x4bf4, the TX power control mode (the channel set's step
	 * 12): 0 on every set in this wl.ko.
	 */
	u8 txpwrctrl;
	/*
	 * sh+108, the current interference mode:
	 * 0 from wlc_phy_attach_acphy (0x473d0) until the first init's channel
	 * set is done, then sh+116 (7 for PHY rev 1) from wlc_phy_init
	 * (0x56aac) and wlc_phy_chanspec_set (0x5d538).
	 */
	u8 intf_mode;
	/*
	 * Per-channel ACI/desense records, pi_ac+0x760 (2 GHz) and +0x850
	 * (5 GHz), three of 80 bytes each, kept as wl lays them out;
	 * aci_cur is pi_ac[0x940], aci_chan is
	 * [pi,#400].
	 */
	u8 aci_rec[2][3][80];
	u8 *aci_cur;
	u8 aci_chan;
	/*
	 * acphy_desense: wl's ACI/desense engine state (phy_ac.c, "ACI and
	 * hwaci engines"). pi_ac[0x6db], [0x6dc]: the OFDM / BPHY desense
	 * wlc_phy_desense_apply_acphy last applied (0x279a0, 0x2799c).
	 */
	u8 desense_last[2];
	/* pi_ac[66]: max CRS min-power threshold of the last cal (0x3ab3c) */
	u8 crs_thr_max;
	/*
	 * wlc_phy_watchdog @0x5e48c: the ucode MAC-stat words last read
	 * (pi[0x400], [0x274], [0x364], [0x276]: shm 0x10e, 0x10c, 0x158,
	 * 0x15e) and the per-second samples the ACI engine reads back,
	 * glitch + 2 * bad PLCP for OFDM [0] and BPHY [1] (0x40940, 0x40980).
	 */
	u16 aci_stat[4];
	bool aci_stat_valid;
	u32 aci_sample[2];
	u32 aci_avg[2];		/* the engine's avg-of-max, OFDM [0], BPHY [1] */
	/* pi_ac[0x9c4]: core index wlc_phy_watchdog_acphy rotates into PHY 0x520 */
	u8 aci_core;
	/* what the last desense apply set: init gain target, CRS min power */
	u8 desense_init_target;
	u8 desense_crs;
	u8 desense_crs_hi;
	/* the 1 s work that stands for wl's wlc_phy_watchdog */
	struct b43_wldev *dev;
	struct delayed_work desense_work;
	bool desense_work_on;
	u32 desense_ticks;
	int desense_mode;	/* acphy_desense value the work last acted on */
	/*
	 * acphy_initfix: op_init calls since the allocation (1 = the first
	 * init of this module load, 2 = after the first controller restart,
	 * ...). Never reset: the "INITFIX" lines carry it so a per-init
	 * outcome can be correlated with the init count.
	 */
	unsigned int init_seq;
	/*
	 * acphy_radiodump: core inits since the allocation (prepare_structs
	 * calls), this init's local_clock origin for the INITTIME lines and
	 * the init's one-shots (first TX, first HT TX, channel set dump).
	 */
	unsigned int inittime_seq;
	u64 inittime_t0;
	bool inittime_tx;
	bool inittime_httx;
	bool radiodump_set_done;

	/*
	 * TX IQ/LO cal state, wl's cal_info = [pi+0x4bb0]
	 * and two pi_ac fields. Only used with acphy_txcal=1.
	 *
	 * acphy_initfix bit 1 zeroes everything above `dev` and the fields
	 * from txcal_gain on at every core init (b43_phy_ac_state_reset);
	 * the six fields between them survive a re-init on purpose. A new
	 * field goes above `dev`, or into that function.
	 */
	/*
	 * cal_info+0x82+10c: the per-core pre-cal TX gain record that
	 * wlc_phy_precal_txgain_acphy fills: rad_gain, rad_gain_mi,
	 * rad_gain_hi, (not written), bbmult.
	 */
	u16 txcal_gain[3][5];
	/*
	 * Coefficient types 16..19 (cal_info+4, the results kept between
	 * cals) and 12..15 (cal_info+44, the working copy), indexed
	 * core * 5 + {0, 1: IQ a/b, 2: LO d, 3: e, 4: f}.
	 */
	u16 txiqlo_final[15];
	u16 txiqlo_mem[15];
	bool txiqlo_valid;	/* cal_info[0x54]: final results present */
	bool txcal_ladder[3];	/* cal_info[0x55 + c]: ladder written for core c */
	u16 txcal_chanspec;	/* cal_info[0xaa]: chanspec of the last cal */
	u16 tone_bbmult[3];	/* pi_ac[2 + 2c]: bbmult saved by the tone */
	bool tone_bbmult_saved;	/* pi_ac[10] */
};

struct b43_phy_operations;
extern const struct b43_phy_operations b43_phyops_ac;

struct b43_wldev;
int b43_phy_ac_rssi(struct b43_wldev *dev, const u8 *raw, s8 *dbm);

/* acphy_bw40 (default 0): 40 MHz channel sets */
extern bool b43_modparam_ac_bw40;
/* acphy_bw80 (default 0): 80 MHz channel sets */
extern bool b43_modparam_ac_bw80;
/* wl chanspec (channel, sideband, bandwidth, band) for a set to channel */
u16 b43_phy_ac_chanspec(struct b43_wldev *dev, unsigned int channel);

/*
 * acphy_rateppr (default 0): per-rate TX power offset, half dB, for PHY
 * control 1 bits 3..7.
 * Slots: OFDM 6..54 Mbit/s, MCS 0..9 at 20 MHz (20-in-40 on a 40 MHz
 * chanspec), MCS 0..9 at 40 MHz.
 */
#define B43_AC_PWROFF_OFDM	0
#define B43_AC_PWROFF_MCS	8
#define B43_AC_PWROFF_MCS40	18
u8 b43_phy_ac_txpwroff(struct b43_wldev *dev, unsigned int slot, bool ht);

/* acphy_initorder (default 0): wl's order around the PHY init, phy_ac.c */
extern unsigned int b43_modparam_ac_initorder;
#define B43_AC_INITORDER_PHYRESET	0x01	/* wl's PHY reset pulse */
#define B43_AC_INITORDER_MPCLK		0x02	/* IOCTL 0x10 before the radio-on */
#define B43_AC_INITORDER_ONESET		0x04	/* one set per chanspec */
/* wlc_bmac_phy_reset + wlc_bmac_core_phy_clk(1), for acphy_initorder bit 0 */
void b43_phy_ac_phy_reset_wl(struct b43_wldev *dev);

/*
 * acphy_initfix (bitmask, default 0): driver and ucode state that a
 * controller restart (core re-init without a module reload) inherits from
 * the previous init. phy_ac.c handles bits 1
 * and 2 and the PHY-side log; main.c bits 4 and 8 and the MAC-side
 * log. Every bit that acts logs an "INITFIX" line.
 */
extern unsigned int b43_modparam_acphy_initfix;
/* bytes of shared memory zeroed before the ucode upload with bit 8 */
extern unsigned int b43_modparam_acphy_initfix_shm;
#define B43_AC_INITFIX_PHYSTATE	0x01	/* phy_ac state back to attach values at every init */
#define B43_AC_INITFIX_INTF7	0x02	/* init set with intf_mode 7 at every init (wl's later inits) */
#define B43_AC_INITFIX_WLDEV	0x04	/* per-wldev AC counters and A-MPDU epochs zeroed at every init */
#define B43_AC_INITFIX_SHM	0x08	/* shared memory above 4 KB zeroed before the ucode upload */
#define B43_AC_INITFIX_LOG	0x10	/* log the inherited values at every init */

/*
 * acphy_radiodump (int, default 0): radio 0x2069 register dump (RADREG)
 * and init step timing (INITTIME), phy_ac.c; main.c and xmit.c call
 * the hooks. Level 1: dump after the first completed channel set of every
 * core init; 2: also at the init's first HT frame; +4: PHY 0xc00..0x15ff
 * and 0x1800..0x1bff too.
 */
extern int b43_modparam_acphy_radiodump;
#define B43_AC_RADIODUMP_LEVEL(v)	((v) & 3)
#define B43_AC_RADIODUMP_PHYWIDE	0x04
void b43_phy_ac_radiodump(struct b43_wldev *dev, const char *why);
void b43_phy_ac_radiodump_chanset(struct b43_wldev *dev, u16 chanspec);
void b43_phy_ac_inittime(struct b43_wldev *dev, const char *step, u32 arg);
void b43_phy_ac_inittime_tx(struct b43_wldev *dev, bool ht);
/* main.c: the MAC-side steps, AC-PHY devices only (the 2.4 GHz core has no phy.ac); in phy_ac.c */
void b43_ac_inittime(struct b43_wldev *dev, const char *step, u32 arg);

/* BCM53573 (Tenda AC6): radio 20693 on AC-PHY rev 24, in phy_ac_20693.c */
#define B43_RADIO_ID_20693		0x3eb
int b43_ac20693_init(struct b43_wldev *dev);
int b43_ac20693_switch_channel(struct b43_wldev *dev, unsigned int channel);

#endif /* LINUX_B43_PHY_AC_H_ */
