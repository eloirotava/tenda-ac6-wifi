// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Broadcom B43 wireless driver
 * AC-PHY rev 24 + radio 20693 rev 23 (BCM53573 / BCM47189, Tenda AC6)
 *
 * Radio 20693 rev 23 is major rev 2; the AC6 runs the PHY as 2x2 MIMO.
 * The power-on follows wl's wlc_phy_switch_radio_acphy for radio 20693:
 *   xtal power-up of core 1, tiny power-on sequence (with the preferred
 *   values), PMU overrides, mini PMU power-on, PLL power-up on core 0,
 *   (mini PMU cal: skipped by wl on BCM53573 chip rev 0), static R cal.
 * Register addresses come from radio_20693.h (generated for rev 23).
 */

#include <linux/delay.h>
#include "b43.h"
#include "phy_ac.h"
#include "radio_20693.h"
#include "radio_20693_pref.h"
#include "phy_ac_rev24_tbl.h"
#include "radio_20693_chan.h"
#include "phy_ac_wlreplay56.h"
#include "phy_ac_wlchan56.h"
#include "main.h"
#include "radio_20693_wlreplay.h"

/* PHY AC rev 24 registers used here (ACPHY_*(24)) */
#define AC24_RfctrlCmd			0x0408
#define AC24_Pllldo_resetCtrl		0x0415
#define AC24_RfctrlOverrideGlobalPus	0x0416
#define AC24_RfctrlCoreGlobalPus	0x0417
#define AC24_RfctrlCoreTxPus0		0x0728
#define AC24_RfctrlOverrideTxPus0	0x0720
#define AC24_Rfpll_resetCtrl		0x040e
#define AC24_Logen_AfeDiv_reset		0x040c
#define AC24_MinorVersion		0x0002

/*
 * Experiment: after tuning channel 36, write the radio and PHY values that
 * differ from wl 9.10 on the same channel (radio_20693_wlreplay.h, from the
 * phydump of both drivers). Narrows down what the 20693 / PHY rev 24 init
 * is missing; not a fix by itself.
 */
static int ac20693_replay = 0;	/* 1 aborta o SoC no canal 36 */
module_param_named(ac20693_replay, ac20693_replay, int, 0644);
MODULE_PARM_DESC(ac20693_replay, "ch 36: replay wl radio/PHY values (1 = radio+PHY, 2 = radio, 3 = PHY)");

static int ac20693_rcal = 8;
module_param_named(ac20693_rcal, ac20693_rcal, int, 0444);
MODULE_PARM_DESC(ac20693_rcal, "20693 static R cal code (wl: 8 UMC, 11 TSMC fab)");

#define R20693(reg, core)	((u16)(RF0_20693_##reg | ((core) << 9)))

#define R20693_MOD(dev, reg, core, fld, v)					\
	b43_radio_maskset(dev, R20693(reg, core),				\
			  (u16)~RF_20693_##reg##_##fld##_MASK,			\
			  ((v) << RF_20693_##reg##_##fld##_SHIFT) &		\
			  RF_20693_##reg##_##fld##_MASK)

#define R20693_FLD(dev, reg, core, fld)						\
	((b43_radio_read(dev, R20693(reg, core)) &				\
	  RF_20693_##reg##_##fld##_MASK) >> RF_20693_##reg##_##fld##_SHIFT)

#define AC20693_NCORES	2

/* phy_ac_radio_20693_xtal_pwrup, MIMO: core 1 (core 0 is the PMU's) */
static void b43_ac20693_xtal_pwrup(struct b43_wldev *dev, u8 core)
{
	R20693_MOD(dev, PLL_XTALLDO1, core, ldo_1p2_xtalldo1p2_vref_bias_reset, 1);
	R20693_MOD(dev, PLL_XTAL_OVR1, core, ovr_ldo_1p2_xtalldo1p2_vref_bias_reset, 1);
	R20693_MOD(dev, PLL_XTALLDO1, core, ldo_1p2_xtalldo1p2_BG_pu, 1);
	R20693_MOD(dev, PLL_XTAL_OVR1, core, ovr_ldo_1p2_xtalldo1p2_BG_pu, 1);
	udelay(300);
	R20693_MOD(dev, PLL_XTALLDO1, core, ldo_1p2_xtalldo1p2_vref_bias_reset, 0);
}

/* wlc_phy_radio20693_upd_prfd_values, rev 23 (MIMO: every entry) */
static void b43_ac20693_prefregs(struct b43_wldev *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(b43_20693_rev23_prefregs); i++)
		b43_radio_write(dev, b43_20693_rev23_prefregs[i][0],
				b43_20693_rev23_prefregs[i][1]);
}

/* wlc_phy_tiny_radio_pwron_seq (radio major rev != 3) */
static void b43_ac20693_tiny_pwron(struct b43_wldev *dev)
{
	b43_phy_write(dev, AC24_RfctrlCoreTxPus0, 0);
	b43_phy_write(dev, AC24_RfctrlOverrideTxPus0, 0x3ff);
	b43_phy_write(dev, AC24_Pllldo_resetCtrl, 0);
	b43_phy_write(dev, AC24_Rfpll_resetCtrl, 0);
	b43_phy_write(dev, AC24_Logen_AfeDiv_reset, 0x2000);
	b43_phy_write(dev, AC24_RfctrlCmd, 0);
	b43_phy_write(dev, AC24_RfctrlCoreGlobalPus, 0);
	b43_phy_write(dev, AC24_RfctrlOverrideGlobalPus, 0xd);
	b43_phy_write(dev, AC24_RfctrlCmd, 0x7);		/* radio/jtag reset */
	udelay(100);
	b43_phy_write(dev, AC24_RfctrlCmd, 0x6);
	b43_ac20693_prefregs(dev);
	b43_phy_write(dev, AC24_RfctrlCoreGlobalPus, 0xd);
	b43_phy_write(dev, AC24_RfctrlCmd, 0x2);
	b43_phy_write(dev, AC24_RfctrlCoreTxPus0, 0x180);
	udelay(100);
	b43_phy_write(dev, AC24_RfctrlCoreGlobalPus, 0x4);
	b43_phy_write(dev, AC24_RfctrlCoreTxPus0, 0x80);
	b43_phy_write(dev, AC24_RfctrlOverrideTxPus0, 0x180);
}

/* wlc_phy_radio20693_minipmu_pwron_seq (major rev != 3) */
static void b43_ac20693_minipmu_pwron(struct b43_wldev *dev)
{
	u8 c;

	for (c = 0; c < AC20693_NCORES; c++) {
		R20693_MOD(dev, PMU_OP, c, wlpmu_en, 1);
		R20693_MOD(dev, PMU_OP, c, wlpmu_TXldo_pu, 1);
	}
	udelay(110);
	for (c = 0; c < AC20693_NCORES; c++) {
		R20693_MOD(dev, VREG_CFG, c, vreg25_pu, 1);
		R20693_MOD(dev, BG_CFG1, c, bg_pu, 1);
	}
	udelay(10);
	for (c = 0; c < AC20693_NCORES; c++) {
		R20693_MOD(dev, PMU_OP, c, wlpmu_VCOldo_pu, 1);
		R20693_MOD(dev, PMU_OP, c, wlpmu_AFEldo_pu, 1);
		R20693_MOD(dev, PMU_OP, c, wlpmu_RXldo_pu, 1);
		R20693_MOD(dev, PMU_CFG4, c, wlpmu_ADCldo_pu, 1);
	}
	udelay(12);
	for (c = 0; c < AC20693_NCORES; c++) {
		R20693_MOD(dev, PLL_CFG1, c, synth_pwrsw_en, 1);
		R20693_MOD(dev, PMU_CFG5, c, wlpmu_spare, 0);
	}
}

/* wlc_phy_radio20693_pmu_pll_config, MIMO: core 0 only */
static void b43_ac20693_pll_pwrup(struct b43_wldev *dev)
{
	R20693_MOD(dev, PLL_CFG1, 0, rfpll_vco_pu, 1);
	R20693_MOD(dev, PLL_HVLDO1, 0, ldo_2p5_pu_ldo_VCO, 1);
	R20693_MOD(dev, PLL_CFG1, 0, rfpll_vco_buf_pu, 1);
	R20693_MOD(dev, PLL_CFG1, 0, rfpll_synth_pu, 1);
	R20693_MOD(dev, PLL_CP1, 0, rfpll_cp_pu, 1);
	R20693_MOD(dev, PLL_HVLDO1, 0, ldo_2p5_pu_ldo_CP, 1);
	R20693_MOD(dev, PLL_CFG1, 0, rfpll_monitor_pu, 1);
	R20693_MOD(dev, PLL_HVLDO1, 0, ldo_2p5_bias_reset_CP, 1);
	R20693_MOD(dev, PLL_HVLDO1, 0, ldo_2p5_bias_reset_VCO, 1);
	udelay(89);
	R20693_MOD(dev, PLL_HVLDO1, 0, ldo_2p5_bias_reset_CP, 0);
	R20693_MOD(dev, PLL_HVLDO1, 0, ldo_2p5_bias_reset_VCO, 0);
}

/* wlc_phy_radio_tiny_rcal, TINY_RCAL_MODE1_STATIC */
static void b43_ac20693_rcal_static(struct b43_wldev *dev)
{
	u8 c;

	for (c = 0; c < AC20693_NCORES; c++) {
		R20693_MOD(dev, BG_CFG1, c, bg_rcal_trim, ac20693_rcal & 0xf);
		R20693_MOD(dev, BG_OVR1, c, ovr_otp_rcal_sel, 0);
		R20693_MOD(dev, BG_OVR1, c, ovr_bg_rcal_trim, 1);
	}
}

/* wlc_phy_switch_radio_acphy(pi, ON), radio 20693, major rev 2 */
static void b43_ac20693_radio_on(struct b43_wldev *dev)
{
	u8 c;

	b43_ac20693_xtal_pwrup(dev, 1);
	b43_ac20693_tiny_pwron(dev);
	for (c = 0; c < AC20693_NCORES; c++) {
		R20693_MOD(dev, PMU_OVR, c, ovr_wlpmu_en, 1);
		R20693_MOD(dev, PMU_OVR, c, ovr_wlpmu_AFEldo_pu, 1);
		R20693_MOD(dev, PMU_OVR, c, ovr_wlpmu_TXldo_pu, 1);
		R20693_MOD(dev, PMU_OVR, c, ovr_wlpmu_VCOldo_pu, 1);
		R20693_MOD(dev, PMU_OVR, c, ovr_wlpmu_RXldo_pu, 1);
		R20693_MOD(dev, PMU_OVR, c, ovr_wlpmu_ADCldo_pu, 1);
		R20693_MOD(dev, PMU_OVR, c, ovr_wlpmu_selavg_lo, 1);
	}
	b43_ac20693_minipmu_pwron(dev);
	b43_ac20693_pll_pwrup(dev);
	/* wlc_phy_tiny_radio_minipmu_cal returns at once on BCM53573 rev 0 */
	b43_ac20693_rcal_static(dev);
}

/* wlc_phy_table_write_acphy: TableID 0x0d, TableOffset 0x0e, data 0x0f/0x10 */
static void b43_ac20693_tbl_write(struct b43_wldev *dev, u16 id, u32 len,
				  u16 offset, u8 width, const void *data)
{
	u16 held = b43_phy_read(dev, 0x19e) & 0x2;	/* RxFeCtrl1 disable_stalls */
	u32 i, v;

	b43_phy_maskset(dev, 0x19e, (u16)~0x2, 0x2);
	b43_phy_write(dev, 0x0d, id);
	b43_phy_write(dev, 0x0e, offset);
	for (i = 0; i < len; i++) {
		if (width == 8) {
			b43_phy_write(dev, 0x0f, data ? ((const u8 *)data)[i] : 0);
		} else if (width == 16) {
			b43_phy_write(dev, 0x0f, data ? ((const u16 *)data)[i] : 0);
		} else {
			v = data ? ((const u32 *)data)[i] : 0;
			b43_phy_write(dev, 0x10, v >> 16);
			b43_phy_write(dev, 0x0f, v & 0xffff);
		}
	}
	b43_phy_maskset(dev, 0x19e, (u16)~0x2, held);
}

/* wlc_phy_static_table_download_acphy, ACMAJORREV_4: acphytbl_info_rev12 + zero tables */
static void b43_ac20693_static_tables(struct b43_wldev *dev)
{
	b43_ac20693_tbl_write(dev, 2, ARRAY_SIZE(b43_ac24_tx_evm_tbl_rev3), 0, 8,
			      b43_ac24_tx_evm_tbl_rev3);
	b43_ac20693_tbl_write(dev, 25, ARRAY_SIZE(b43_ac24_NvAdjTbl_rev12), 0, 32,
			      b43_ac24_NvAdjTbl_rev12);
	b43_ac20693_tbl_write(dev, 5, ARRAY_SIZE(b43_ac24_phasetrack_tbl_rev12), 0, 32,
			      b43_ac24_phasetrack_tbl_rev12);
	b43_ac20693_tbl_write(dev, 6, ARRAY_SIZE(b43_ac24_sqthreshold_tbl_rev12), 0, 32,
			      b43_ac24_sqthreshold_tbl_rev12);
	b43_ac20693_tbl_write(dev, 35, ARRAY_SIZE(b43_ac24_sgi_adjust_tbl_rev12), 0, 32,
			      b43_ac24_sgi_adjust_tbl_rev12);
	b43_ac20693_tbl_write(dev, 4, 256, 0, 8, NULL);		/* RX EVM */
	b43_ac20693_tbl_write(dev, 3, 256, 0, 32, NULL);	/* RX noise shaping */
	b43_ac20693_tbl_write(dev, 129, 64, 0, 32, NULL);	/* est pwr shift */
}

static int ac56_slow;
module_param(ac56_slow, int, 0644);
#define AC56_MARK(dev, txt) do { if (ac56_slow > 0) { b43info((dev)->wl, "20693 marca: %s\n", txt); msleep(400); } } while (0)
MODULE_PARM_DESC(ac56_slow, "rev 56 replay: log the op index and sleep 400 ms every N ops (find where the SoC aborts)");

static int ac56_wlearly;
module_param(ac56_wlearly, int, 0644);
MODULE_PARM_DESC(ac56_wlearly, "rev 56: replay the first N recorded wl ops in the PHY init instead of b43's static tables + radio on (0 = off)");
static void b43_ac56_wlreplay_n(struct b43_wldev *dev, unsigned int n);
static bool ac56_wlreplay_done;
static bool ac56_cflags_bw;	/* let cflags through the IOCTL bandwidth bits 6..7 */

int b43_ac20693_init(struct b43_wldev *dev)
{
	u16 minor = b43_phy_read(dev, AC24_MinorVersion);

	b43info(dev->wl, "20693: AC-PHY rev %u minor %u (ROUTER_4349 = 1 or 3), radio rev %u\n",
		dev->phy.rev, minor & 0xf, dev->phy.radio_rev);
	/*
	 * Every PHY init (interface up after all were down) starts from b43's
	 * own state again, so the next channel set has to replay once more.
	 */
	ac56_wlreplay_done = false;
	if (ac56_wlearly) {
		b43_ac56_wlreplay_n(dev, ac56_wlearly);
		AC56_MARK(dev, "init: fim (replay)");
		return 0;
	}
	b43_ac20693_static_tables(dev);
	b43_ac20693_radio_on(dev);
	b43info(dev->wl, "20693: radio on (%zu preferred values, rcal %d), REV_ID 0x%04x PLL_CFG1 0x%04x\n",
		ARRAY_SIZE(b43_20693_rev23_prefregs), ac20693_rcal,
		b43_radio_read(dev, R20693(REV_ID, 0)),
		b43_radio_read(dev, R20693(PLL_CFG1, 0)));
	return 0;
}

/* wlc_phy_radio20693_pll_tune: the 17 registers in chan_info_radio20693_pll order */
static const u16 b43_20693_pll_regs[17] = {
	RF0_20693_PLL_VCOCAL1, RF0_20693_PLL_VCOCAL11, RF0_20693_PLL_VCOCAL12,
	RF0_20693_PLL_FRCT2, RF0_20693_PLL_FRCT3, RF0_20693_PLL_HVLDO1,
	RF0_20693_PLL_LF4, RF0_20693_PLL_LF5, RF0_20693_PLL_LF7,
	RF0_20693_PLL_LF2, RF0_20693_PLL_LF3, RF0_20693_SPARE_CFG1,
	RF0_20693_SPARE_CFG14, RF0_20693_SPARE_CFG13, RF0_20693_TXMIX2G_CFG5,
	RF0_20693_TXMIX5G_CFG6, RF0_20693_PA5G_CFG4,
};

/* wlc_phy_radio20693_rffe_tune (MIMO: the core's own registers) */
static const u16 b43_20693_rffe_regs[3] = {
	RF0_20693_LNA2G_TUNE, RF0_20693_LNA5G_TUNE, RF0_20693_PA2G_CFG2,
};

static const struct b43_20693_chan *b43_20693_find_chan(unsigned int channel)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(b43_20693_chans); i++)
		if (b43_20693_chans[i].chan == channel)
			return &b43_20693_chans[i];
	return NULL;
}

/* wlc_phy_radio_tiny_vcocal, radio 20693, MIMO: core 0 only (no vco_opt yet) */
static void b43_ac20693_vcocal(struct b43_wldev *dev)
{
	u16 ovr1 = b43_radio_read(dev, R20693(CLK_DIV_OVR1, 0));
	u16 cfg1 = b43_radio_read(dev, R20693(CLK_DIV_CFG1, 0));

	R20693_MOD(dev, CLK_DIV_OVR1, 0, ovr_afeclkdiv_6g_mimo_pu, 1);
	R20693_MOD(dev, CLK_DIV_OVR1, 0, ovr_afeclkdiv_12g_mimo_div2_pu, 1);
	R20693_MOD(dev, CLK_DIV_OVR1, 0, ovr_afeclkdiv_12g_mimo_pu, 1);
	R20693_MOD(dev, CLK_DIV_CFG1, 0, afe_clk_div_6g_mimo_pu, 0);
	R20693_MOD(dev, CLK_DIV_CFG1, 0, afe_clk_div_12g_mimo_div2_pu, 0);
	R20693_MOD(dev, CLK_DIV_CFG1, 0, afe_clk_div_12g_mimo_pu, 0);

	R20693_MOD(dev, PLL_VCOCAL10, 0, rfpll_vcocal_ovr_mode, 0);
	R20693_MOD(dev, PLL_CFG2, 0, rfpll_rst_n, 0);
	R20693_MOD(dev, PLL_VCOCAL13, 0, rfpll_vcocal_rst_n, 0);
	R20693_MOD(dev, PLL_VCOCAL1, 0, rfpll_vcocal_cal, 0);
	udelay(11);
	R20693_MOD(dev, PLL_CFG2, 0, rfpll_rst_n, 1);
	R20693_MOD(dev, PLL_VCOCAL13, 0, rfpll_vcocal_rst_n, 1);
	udelay(1);
	R20693_MOD(dev, PLL_VCOCAL1, 0, rfpll_vcocal_cal, 1);
	/* wl waits for the cal (ACPHY_SPINWAIT_VCOCAL) then restores the clocks */
	udelay(300);
	b43_radio_write(dev, R20693(CLK_DIV_OVR1, 0), ovr1);
	b43_radio_write(dev, R20693(CLK_DIV_CFG1, 0), cfg1);
}

/* wlc_phy_resetcca_acphy, generic branch: pulse BBConfig.resetCCA, wait reset2rx */
static void b43_ac20693_resetcca(struct b43_wldev *dev)
{
	unsigned int i;

	b43_phy_force_clock(dev, true);
	b43_phy_maskset(dev, 0x001, 0xffff, 0x4000);		/* BBConfig.resetCCA */
	udelay(1);
	b43_phy_mask(dev, 0x001, (u16)~0x4000);
	b43_phy_force_clock(dev, false);
	for (i = 0; i < 1000; i++) {
		if (!(b43_phy_read(dev, 0x403) & 0x20))	/* RfseqStatus0.reset2rx */
			break;
		udelay(1);
	}
	if (i == 1000)
		b43err(dev->wl, "20693: resetcca: reset2rx still set (RfseqStatus0 0x%04x)\n",
		       b43_phy_read(dev, 0x403));
}

/* phy_ac_radio_20693_chanspec_setup, major rev 2, MIMO: both cores, 20 MHz */
/*
 * ac56_wlreplay: the wl 9.10 PHY/radio write sequence recorded while it
 * brings the 5 GHz core up (wltrace, rastro/wl-5g-up.txt), applied once per
 * module load before b43's own tuning. b43 writes 21 PHY registers in its
 * init where wl writes 417.
 */
static int ac56_wlreplay;
module_param(ac56_wlreplay, int, 0644);
MODULE_PARM_DESC(ac56_wlreplay, "rev 56: replay the first N recorded wl PHY/radio ops once before tuning (0 = off)");

static int ac56_cflags_all;
module_param(ac56_cflags_all, int, 0644);
MODULE_PARM_DESC(ac56_cflags_all, "rev 56 replay: apply every recorded cflags mask to the d11 IOCTL (default: clock bits only)");

/*
 * Radio register the replay leaves alone. 0x219 is PLL_XTALLDO1 of radio
 * core 1: wl ops 61/62 clear its bandgap and set the LDO pull-down
 * (0x01e7), and the SoC resets silently right there (and again at ops
 * 16732/16733). -1 = skip nothing.
 */
static int ac56_skiprad = -1;
module_param(ac56_skiprad, int, 0644);
MODULE_PARM_DESC(ac56_skiprad, "rev 56 replay: skip writes to this radio register (-1 = none; 0x219 = core 1 PLL_XTALLDO1)");

static int ac56_replayonly;
module_param(ac56_replayonly, int, 0644);
MODULE_PARM_DESC(ac56_replayonly, "rev 56: with ac56_wlreplay, skip b43's own channel tuning after the replay");

static int ac56_macops;
module_param(ac56_macops, int, 0644);
MODULE_PARM_DESC(ac56_macops, "rev 56 replay: also replay wl's MAC suspend/enable (default off)");


static void b43_ac56_run_ops(struct b43_wldev *dev, const struct b43_ac56_op *ops,
			     unsigned int n)
{
	struct bcma_device *core = dev->dev->bdev;
	unsigned int i;
	u32 m, v, io;

	for (i = 0; i < n; i++) {
		const struct b43_ac56_op *o = &ops[i];

		if (ac56_slow > 0 && !(i % ac56_slow)) {
			b43info(dev->wl, "20693 replay: op %u (tipo %u reg 0x%x)\n", i, o->type, o->addr);
			msleep(400);
		}
		if (o->type >= 5 && o->type <= 8 && (int)o->addr == ac56_skiprad)
			continue;
		switch (o->type) {
		case 1: b43_phy_write(dev, o->addr, o->val); break;
		case 2: b43_phy_maskset(dev, o->addr, ~o->mask, o->val & o->mask); break;
		case 3: b43_phy_mask(dev, o->addr, o->val); break;
		case 4: b43_phy_set(dev, o->addr, o->val); break;
		case 5: b43_radio_write(dev, o->addr, o->val); break;
		case 6: b43_radio_maskset(dev, o->addr, ~o->mask, o->val & o->mask); break;
		case 7: b43_radio_mask(dev, o->addr, o->val); break;
		case 8: b43_radio_set(dev, o->addr, o->val); break;
		case 9:		/* osl_delay */
			v = ((u32)o->mask << 16) | o->val;
			if (v >= 1000)
				mdelay(v / 1000);
			udelay(v % 1000);
			break;
		case 10:	/* si_core_cflags on the d11 core: core IOCTL mask/val */
			m = ((u32)o->addr << 16) | o->mask;
			v = o->val;
			if (i + 1 < n && ops[i + 1].type == 15)
				v |= (u32)ops[++i].val << 16;
			/* si_core_cflags acts on the si's current core, not always the
			 * d11: keep only the d11 clock bits (FGC, PHY_CLKEN, MACPHYCLKEN)
			 * unless ac56_cflags_all */
			if (!ac56_cflags_all)
				m &= ac56_cflags_bw ? 0x00d6 : 0x0016;
			if (!m)
				break;
			io = bcma_aread32(core, BCMA_IOCTL);
			bcma_awrite32(core, BCMA_IOCTL, (io & ~m) | (v & m));
			break;
		case 11: b43_phy_force_clock(dev, o->val); break;
		case 12:	/* suspend/enable MAC: b43 keeps its own MAC state */
		case 13:
			if (ac56_macops) {
				if (o->type == 12)
					b43_mac_suspend(dev);
				else
					b43_mac_enable(dev);
			}
			break;
		case 14: b43_shm_write16(dev, B43_SHM_SHARED, o->addr, o->val); break;
		case 16: b43_write16(dev, B43_MMIO_PHY_DATA, o->val); break;	/* 48 bit table words 1, 2 */
		}
	}
}

/* channel the PHY/radio are tuned to by the replay or a wl channel window */
static u16 ac56_cur_chanspec;

static void b43_ac56_wlreplay_n(struct b43_wldev *dev, unsigned int n)
{
	struct bcma_device *core = dev->dev->bdev;

	n = min_t(unsigned int, n, ARRAY_SIZE(b43_ac56_wlops));
	b43_ac56_run_ops(dev, b43_ac56_wlops, n);
	ac56_cur_chanspec = 0x1024;	/* the recorded bring-up ends on channel 36, 20 MHz */
	b43info(dev->wl, "20693: wl replay, %u of %zu ops, IOCTL 0x%08x\n", n,
		ARRAY_SIZE(b43_ac56_wlops), bcma_aread32(core, BCMA_IOCTL));
	if (ac56_slow > 0)
		msleep(400);
}

int b43_ac20693_switch_channel(struct b43_wldev *dev, unsigned int channel)
{
	const struct b43_20693_chan *ci = b43_20693_find_chan(channel);
	unsigned int i;
	u8 c;

	if (!ci) {
		b43err(dev->wl, "20693: no tuning entry for channel %u\n", channel);
		return -EINVAL;
	}
	if (ac56_wlreplay && !ac56_wlreplay_done) {
		ac56_wlreplay_done = true;
		b43_ac56_wlreplay_n(dev, ac56_wlreplay);
	}
	/*
	 * The replay already leaves the radio/PHY as wl has them on the
	 * channel; b43's own tuning on top of it changed PLL_CFG1, the ADC,
	 * TIA and AFE clock divider registers away from wl's values.
	 */
	/*
	 * Hardware TX power control (PHY 0x070 bits 13..15) is left on by the
	 * replay, but b43 has no idle TSSI / target power for it: the loop
	 * drives the TX gain index to 0x0c (wl sits at 0x58) and every frame
	 * goes out saturated, so nobody decodes it. With the loop off the gain
	 * words the replay left (wl's) are used, and TX works.
	 */
	b43_phy_mask(dev, 0x070, (u16)~0xe000);
	if (ac56_wlreplay && ac56_replayonly) {
		/*
		 * Other channels: wl's own channel set for that channel, cut from
		 * the scan in the trace (tools/mkchan.py: from the MAC suspend
		 * that opens it to the enable that closes it).
		 */
		u16 cs = b43_phy_ac_chanspec(dev, channel) & 0x3fff;

		if (!(cs & 0x3800))
			cs |= 0x1000;			/* 20 MHz */
		if (cs != ac56_cur_chanspec) {
			for (i = 0; i < ARRAY_SIZE(b43_ac56_chansegs); i++)
				if ((b43_ac56_chansegs[i].chanspec & 0x3fff) == cs)
					break;
			if (i == ARRAY_SIZE(b43_ac56_chansegs)) {
				b43err(dev->wl, "20693: channel %u chanspec 0x%04x: no wl channel set recorded\n",
				       channel, cs);
				return -EINVAL;
			}
			ac56_cflags_bw = true;	/* the set also switches the IOCTL bandwidth bits */
			b43_ac56_run_ops(dev, b43_ac56_chanops + b43_ac56_chansegs[i].start,
					 b43_ac56_chansegs[i].len);
			ac56_cflags_bw = false;
			b43_phy_mask(dev, 0x070, (u16)~0xe000);
			b43info(dev->wl, "20693: channel %u chanspec 0x%04x: wl channel set, %u ops, IOCTL 0x%08x\n",
				channel, cs, b43_ac56_chansegs[i].len,
				bcma_aread32(dev->dev->bdev, BCMA_IOCTL));
			ac56_cur_chanspec = cs;
		}
		return 0;
	}
	AC56_MARK(dev, "sw: pll/rffe");
	for (c = 0; c < AC20693_NCORES; c++) {
		for (i = 0; i < ARRAY_SIZE(b43_20693_pll_regs); i++)
			b43_radio_write(dev, b43_20693_pll_regs[i] | (c << 9), ci->pll[i]);
		for (i = 0; i < ARRAY_SIZE(b43_20693_rffe_regs); i++)
			b43_radio_write(dev, b43_20693_rffe_regs[i] | (c << 9), ci->rffe[i]);
	}
	AC56_MARK(dev, "sw: vcocal");
	b43_ac20693_vcocal(dev);
	AC56_MARK(dev, "sw: bw regs");
	for (i = 0; i < ARRAY_SIZE(ci->bw); i++)
		b43_phy_write(dev, 0x371 + i, ci->bw[i]);	/* ACPHY_BW1a.. */
	if (ci->chan == 36 && ac20693_replay) {
		if (ac20693_replay != 3)
			for (i = 0; i < ARRAY_SIZE(b43_ac20693_wl_radio_ch36); i++)
				b43_radio_write(dev, b43_ac20693_wl_radio_ch36[i][0],
						b43_ac20693_wl_radio_ch36[i][1]);
		if (ac20693_replay != 2)
			for (i = 0; i < ARRAY_SIZE(b43_ac20693_wl_phy_ch36); i++)
				b43_phy_write(dev, b43_ac20693_wl_phy_ch36[i][0],
					      b43_ac20693_wl_phy_ch36[i][1]);
		b43info(dev->wl, "20693: ch 36 wl replay %d (%zu radio, %zu PHY)\n",
			ac20693_replay, ARRAY_SIZE(b43_ac20693_wl_radio_ch36),
			ARRAY_SIZE(b43_ac20693_wl_phy_ch36));
	}
	AC56_MARK(dev, "sw: resetcca");
	b43_ac20693_resetcca(dev);
	AC56_MARK(dev, "sw: fim");
	b43info(dev->wl, "20693: channel %u (%u MHz) tuned, VCOCAL1 0x%04x\n",
		ci->chan, ci->freq, b43_radio_read(dev, R20693(PLL_VCOCAL1, 0)));
	/* ucode state: MACCTL, MACCMD, IRQ reason, PSM debug (PC) at 0x154, IFS */
	b43err(dev->wl, "20693 MAC: MACCTL 0x%08x MACCMD 0x%08x IRQ 0x%08x PSMDBG 0x%08x IFSSTAT 0x%04x\n",
	       b43_read32(dev, B43_MMIO_MACCTL), b43_read32(dev, B43_MMIO_MACCMD),
	       b43_read32(dev, B43_MMIO_GEN_IRQ_REASON), b43_read32(dev, 0x154),
	       b43_read16(dev, B43_MMIO_IFSSTAT));
	return 0;
}
