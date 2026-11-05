// SPDX-License-Identifier: GPL-2.0-or-later
/*

  Broadcom B43 wireless driver

  Copyright (c) 2005 Martin Langer <martin-langer@gmx.de>
  Copyright (c) 2005 Stefano Brivio <stefano.brivio@polimi.it>
  Copyright (c) 2005-2009 Michael Buesch <m@bues.ch>
  Copyright (c) 2005 Danny van Dyk <kugelfang@gentoo.org>
  Copyright (c) 2005 Andreas Jaggi <andreas.jaggi@waterwave.ch>
  Copyright (c) 2010-2011 Rafał Miłecki <zajec5@gmail.com>

  SDIO support
  Copyright (c) 2009 Albert Herranz <albert_herranz@yahoo.es>

  Some parts of the code in this file are derived from the ipw2200
  driver  Copyright(c) 2003 - 2004 Intel Corporation.


*/

#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "b43.h"
#include "main.h"
#include "debugfs.h"
#include "phy_common.h"
#include "phy_g.h"
#include "phy_n.h"
#include "phy_ac.h"
#include "dma.h"
#include "pio.h"
#include "sysfs.h"
#include "xmit.h"
#include "lo.h"
#include "sdio.h"
#include <linux/mmc/sdio_func.h>

MODULE_DESCRIPTION("Broadcom B43 wireless driver");
MODULE_AUTHOR("Martin Langer");
MODULE_AUTHOR("Stefano Brivio");
MODULE_AUTHOR("Michael Buesch");
MODULE_AUTHOR("Gábor Stefanik");
MODULE_AUTHOR("Rafał Miłecki");
MODULE_LICENSE("GPL");

MODULE_FIRMWARE("b43/ucode11.fw");
MODULE_FIRMWARE("b43/ucode13.fw");
MODULE_FIRMWARE("b43/ucode14.fw");
MODULE_FIRMWARE("b43/ucode15.fw");
MODULE_FIRMWARE("b43/ucode16_lp.fw");
MODULE_FIRMWARE("b43/ucode16_mimo.fw");
MODULE_FIRMWARE("b43/ucode24_lcn.fw");
MODULE_FIRMWARE("b43/ucode25_lcn.fw");
MODULE_FIRMWARE("b43/ucode25_mimo.fw");
MODULE_FIRMWARE("b43/ucode26_mimo.fw");
MODULE_FIRMWARE("b43/ucode29_mimo.fw");
MODULE_FIRMWARE("b43/ucode33_lcn40.fw");
MODULE_FIRMWARE("b43/ucode30_mimo.fw");
MODULE_FIRMWARE("b43/ucode5.fw");
MODULE_FIRMWARE("b43/ucode40.fw");
MODULE_FIRMWARE("b43/ucode42.fw");
MODULE_FIRMWARE("b43/ucode9.fw");

static int modparam_gpiomask = 0x000F;
module_param_named(gpiomask, modparam_gpiomask, int, 0444);
MODULE_PARM_DESC(gpiomask,
         "GPIO mask for LED control (default 0x000F)");

static int modparam_bad_frames_preempt;
module_param_named(bad_frames_preempt, modparam_bad_frames_preempt, int, 0444);
MODULE_PARM_DESC(bad_frames_preempt,
		 "enable(1) / disable(0) Bad Frames Preemption");

static char modparam_fwpostfix[16];
module_param_string(fwpostfix, modparam_fwpostfix, 16, 0444);
MODULE_PARM_DESC(fwpostfix, "Postfix for the .fw files to load.");

static int modparam_hwpctl;
module_param_named(hwpctl, modparam_hwpctl, int, 0444);
MODULE_PARM_DESC(hwpctl, "Enable hardware-side power control (default off)");

static int modparam_nohwcrypt;
module_param_named(nohwcrypt, modparam_nohwcrypt, int, 0444);
MODULE_PARM_DESC(nohwcrypt, "Disable hardware encryption.");

static int modparam_hwtkip;
module_param_named(hwtkip, modparam_hwtkip, int, 0444);
MODULE_PARM_DESC(hwtkip, "Enable hardware tkip.");

static int modparam_qos = 1;
module_param_named(qos, modparam_qos, int, 0444);
MODULE_PARM_DESC(qos, "Enable QOS support (default on)");

static int modparam_btcoex = 1;
module_param_named(btcoex, modparam_btcoex, int, 0444);
MODULE_PARM_DESC(btcoex, "Enable Bluetooth coexistence (default on)");

int b43_modparam_verbose = B43_VERBOSITY_DEFAULT;
module_param_named(verbose, b43_modparam_verbose, int, 0644);
MODULE_PARM_DESC(verbose, "Log message verbosity: 0=error, 1=warn, 2=info(default), 3=debug");

static int b43_modparam_pio;
module_param_named(pio, b43_modparam_pio, int, 0644);
MODULE_PARM_DESC(pio, "Use PIO accesses by default: 0=DMA, 1=PIO");

static int modparam_allhwsupport = 1;
module_param_named(allhwsupport, modparam_allhwsupport, int, 0444);
MODULE_PARM_DESC(allhwsupport, "Enable support for all hardware (even it if overlaps with the brcmsmac driver)");

#ifdef CPTCFG_B43_BCMA
static const struct bcma_device_id b43_bcma_tbl[] = {
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x11, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x15, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x17, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x18, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x1C, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x1D, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x1E, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x28, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x2A, BCMA_ANY_CLASS),
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, 0x38, BCMA_ANY_CLASS),
	{},
};
MODULE_DEVICE_TABLE(bcma, b43_bcma_tbl);
#endif

#ifdef CPTCFG_B43_SSB
static const struct ssb_device_id b43_ssb_tbl[] = {
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 5),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 6),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 7),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 9),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 10),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 11),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 12),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 13),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 15),
	SSB_DEVICE(SSB_VENDOR_BROADCOM, SSB_DEV_80211, 16),
	{},
};
MODULE_DEVICE_TABLE(ssb, b43_ssb_tbl);
#endif

/* Channel and ratetables are shared for all devices.
 * They can't be const, because ieee80211 puts some precalculated
 * data in there. This data is the same for all devices, so we don't
 * get concurrency issues */
#define RATETAB_ENT(_rateid, _flags) \
	{								\
		.bitrate	= B43_RATE_TO_BASE100KBPS(_rateid),	\
		.hw_value	= (_rateid),				\
		.flags		= (_flags),				\
	}

/*
 * NOTE: When changing this, sync with xmit.c's
 *	 b43_plcp_get_bitrate_idx_* functions!
 */
static struct ieee80211_rate __b43_ratetable[] = {
	RATETAB_ENT(B43_CCK_RATE_1MB, 0),
	RATETAB_ENT(B43_CCK_RATE_2MB, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(B43_CCK_RATE_5MB, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(B43_CCK_RATE_11MB, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(B43_OFDM_RATE_6MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_9MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_12MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_18MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_24MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_36MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_48MB, 0),
	RATETAB_ENT(B43_OFDM_RATE_54MB, 0),
};

#define b43_a_ratetable		(__b43_ratetable + 4)
#define b43_a_ratetable_size	8
#define b43_b_ratetable		(__b43_ratetable + 0)
#define b43_b_ratetable_size	4
#define b43_g_ratetable		(__b43_ratetable + 0)
#define b43_g_ratetable_size	12

#define CHAN2G(_channel, _freq, _flags) {			\
	.band			= NL80211_BAND_2GHZ,		\
	.center_freq		= (_freq),			\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}
static struct ieee80211_channel b43_2ghz_chantable[] = {
	CHAN2G(1, 2412, 0),
	CHAN2G(2, 2417, 0),
	CHAN2G(3, 2422, 0),
	CHAN2G(4, 2427, 0),
	CHAN2G(5, 2432, 0),
	CHAN2G(6, 2437, 0),
	CHAN2G(7, 2442, 0),
	CHAN2G(8, 2447, 0),
	CHAN2G(9, 2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

/* No support for the last 3 channels (12, 13, 14) */
#define b43_2ghz_chantable_limited_size		11
#undef CHAN2G

#define CHAN4G(_channel, _flags) {				\
	.band			= NL80211_BAND_5GHZ,		\
	.center_freq		= 4000 + (5 * (_channel)),	\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}
#define CHAN5G(_channel, _flags) {				\
	.band			= NL80211_BAND_5GHZ,		\
	.center_freq		= 5000 + (5 * (_channel)),	\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}
static struct ieee80211_channel b43_5ghz_nphy_chantable[] = {
	CHAN4G(184, 0),		CHAN4G(186, 0),
	CHAN4G(188, 0),		CHAN4G(190, 0),
	CHAN4G(192, 0),		CHAN4G(194, 0),
	CHAN4G(196, 0),		CHAN4G(198, 0),
	CHAN4G(200, 0),		CHAN4G(202, 0),
	CHAN4G(204, 0),		CHAN4G(206, 0),
	CHAN4G(208, 0),		CHAN4G(210, 0),
	CHAN4G(212, 0),		CHAN4G(214, 0),
	CHAN4G(216, 0),		CHAN4G(218, 0),
	CHAN4G(220, 0),		CHAN4G(222, 0),
	CHAN4G(224, 0),		CHAN4G(226, 0),
	CHAN4G(228, 0),
	CHAN5G(32, 0),		CHAN5G(34, 0),
	CHAN5G(36, 0),		CHAN5G(38, 0),
	CHAN5G(40, 0),		CHAN5G(42, 0),
	CHAN5G(44, 0),		CHAN5G(46, 0),
	CHAN5G(48, 0),		CHAN5G(50, 0),
	CHAN5G(52, 0),		CHAN5G(54, 0),
	CHAN5G(56, 0),		CHAN5G(58, 0),
	CHAN5G(60, 0),		CHAN5G(62, 0),
	CHAN5G(64, 0),		CHAN5G(66, 0),
	CHAN5G(68, 0),		CHAN5G(70, 0),
	CHAN5G(72, 0),		CHAN5G(74, 0),
	CHAN5G(76, 0),		CHAN5G(78, 0),
	CHAN5G(80, 0),		CHAN5G(82, 0),
	CHAN5G(84, 0),		CHAN5G(86, 0),
	CHAN5G(88, 0),		CHAN5G(90, 0),
	CHAN5G(92, 0),		CHAN5G(94, 0),
	CHAN5G(96, 0),		CHAN5G(98, 0),
	CHAN5G(100, 0),		CHAN5G(102, 0),
	CHAN5G(104, 0),		CHAN5G(106, 0),
	CHAN5G(108, 0),		CHAN5G(110, 0),
	CHAN5G(112, 0),		CHAN5G(114, 0),
	CHAN5G(116, 0),		CHAN5G(118, 0),
	CHAN5G(120, 0),		CHAN5G(122, 0),
	CHAN5G(124, 0),		CHAN5G(126, 0),
	CHAN5G(128, 0),		CHAN5G(130, 0),
	CHAN5G(132, 0),		CHAN5G(134, 0),
	CHAN5G(136, 0),		CHAN5G(138, 0),
	CHAN5G(140, 0),		CHAN5G(142, 0),
	CHAN5G(144, 0),		CHAN5G(145, 0),
	CHAN5G(146, 0),		CHAN5G(147, 0),
	CHAN5G(148, 0),		CHAN5G(149, 0),
	CHAN5G(150, 0),		CHAN5G(151, 0),
	CHAN5G(152, 0),		CHAN5G(153, 0),
	CHAN5G(154, 0),		CHAN5G(155, 0),
	CHAN5G(156, 0),		CHAN5G(157, 0),
	CHAN5G(158, 0),		CHAN5G(159, 0),
	CHAN5G(160, 0),		CHAN5G(161, 0),
	CHAN5G(162, 0),		CHAN5G(163, 0),
	CHAN5G(164, 0),		CHAN5G(165, 0),
	CHAN5G(166, 0),		CHAN5G(168, 0),
	CHAN5G(170, 0),		CHAN5G(172, 0),
	CHAN5G(174, 0),		CHAN5G(176, 0),
	CHAN5G(178, 0),		CHAN5G(180, 0),
	CHAN5G(182, 0),
};

static struct ieee80211_channel b43_5ghz_nphy_chantable_limited[] = {
	CHAN5G(36, 0),		CHAN5G(40, 0),
	CHAN5G(44, 0),		CHAN5G(48, 0),
	CHAN5G(149, 0),		CHAN5G(153, 0),
	CHAN5G(157, 0),		CHAN5G(161, 0),
	CHAN5G(165, 0),
};

/*
 * AC-PHY (core rev >= 40) 5 GHz channels: the 20 MHz channels radio 0x2069
 * rev 4's tuning table covers (chan_tuning_2069rev4, chan-set gaps doc),
 * without the A-PHY table's 4.9 GHz entries 184..196 and 2..26, whose
 * 5000 + 5 * n frequencies are wrong and made cfg80211_chandef_valid() warn
 * at every scan (2026-10-06 stress run).
 */
static struct ieee80211_channel b43_5ghz_acphy_chantable[] = {
	CHAN5G(36, 0),	CHAN5G(40, 0),	CHAN5G(44, 0),	CHAN5G(48, 0),
	CHAN5G(52, 0),	CHAN5G(56, 0),	CHAN5G(60, 0),	CHAN5G(64, 0),
	CHAN5G(100, 0),	CHAN5G(104, 0),	CHAN5G(108, 0),	CHAN5G(112, 0),
	CHAN5G(116, 0),	CHAN5G(120, 0),	CHAN5G(124, 0),	CHAN5G(128, 0),
	CHAN5G(132, 0),	CHAN5G(136, 0),	CHAN5G(140, 0),	CHAN5G(144, 0),
	CHAN5G(149, 0),	CHAN5G(153, 0),	CHAN5G(157, 0),	CHAN5G(161, 0),
	CHAN5G(165, 0),
};

static struct ieee80211_channel b43_5ghz_aphy_chantable[] = {
	CHAN5G(34, 0),		CHAN5G(36, 0),
	CHAN5G(38, 0),		CHAN5G(40, 0),
	CHAN5G(42, 0),		CHAN5G(44, 0),
	CHAN5G(46, 0),		CHAN5G(48, 0),
	CHAN5G(52, 0),		CHAN5G(56, 0),
	CHAN5G(60, 0),		CHAN5G(64, 0),
	CHAN5G(100, 0),		CHAN5G(104, 0),
	CHAN5G(108, 0),		CHAN5G(112, 0),
	CHAN5G(116, 0),		CHAN5G(120, 0),
	CHAN5G(124, 0),		CHAN5G(128, 0),
	CHAN5G(132, 0),		CHAN5G(136, 0),
	CHAN5G(140, 0),		CHAN5G(149, 0),
	CHAN5G(153, 0),		CHAN5G(157, 0),
	CHAN5G(161, 0),		CHAN5G(165, 0),
	CHAN5G(184, 0),		CHAN5G(188, 0),
	CHAN5G(192, 0),		CHAN5G(196, 0),
	CHAN5G(200, 0),		CHAN5G(204, 0),
	CHAN5G(208, 0),		CHAN5G(212, 0),
	CHAN5G(216, 0),
};
#undef CHAN4G
#undef CHAN5G

static struct ieee80211_supported_band b43_band_5GHz_nphy = {
	.band		= NL80211_BAND_5GHZ,
	.channels	= b43_5ghz_nphy_chantable,
	.n_channels	= ARRAY_SIZE(b43_5ghz_nphy_chantable),
	.bitrates	= b43_a_ratetable,
	.n_bitrates	= b43_a_ratetable_size,
};

static struct ieee80211_supported_band b43_band_5GHz_nphy_limited = {
	.band		= NL80211_BAND_5GHZ,
	.channels	= b43_5ghz_nphy_chantable_limited,
	.n_channels	= ARRAY_SIZE(b43_5ghz_nphy_chantable_limited),
	.bitrates	= b43_a_ratetable,
	.n_bitrates	= b43_a_ratetable_size,
};

static struct ieee80211_supported_band b43_band_5GHz_aphy = {
	.band		= NL80211_BAND_5GHZ,
	.channels	= b43_5ghz_aphy_chantable,
	.n_channels	= ARRAY_SIZE(b43_5ghz_aphy_chantable),
	.bitrates	= b43_a_ratetable,
	.n_bitrates	= b43_a_ratetable_size,
};

/*
 * 5 GHz band of the AC-PHY: the A-PHY channels and rates plus HT20 when
 * b43_ac_ht_nss() allows it (b43_ac_setup_ht_cap fills ht_cap).
 */
static struct ieee80211_supported_band b43_band_5GHz_acphy = {
	.band		= NL80211_BAND_5GHZ,
	.channels	= b43_5ghz_acphy_chantable,
	.n_channels	= ARRAY_SIZE(b43_5ghz_acphy_chantable),
	.bitrates	= b43_a_ratetable,
	.n_bitrates	= b43_a_ratetable_size,
};

static struct ieee80211_supported_band b43_band_2GHz = {
	.band		= NL80211_BAND_2GHZ,
	.channels	= b43_2ghz_chantable,
	.n_channels	= ARRAY_SIZE(b43_2ghz_chantable),
	.bitrates	= b43_g_ratetable,
	.n_bitrates	= b43_g_ratetable_size,
};

/* N-PHY 2.4 GHz (BCM43217 on the AC6): same channels/rates, HT filled at
 * setup (b43_n_setup_ht_cap). Separate from b43_band_2GHz so G-PHY devices
 * are not advertised as HT by accident.
 */
static struct ieee80211_supported_band b43_band_2GHz_nphy = {
	.band		= NL80211_BAND_2GHZ,
	.channels	= b43_2ghz_chantable,
	.n_channels	= ARRAY_SIZE(b43_2ghz_chantable),
	.bitrates	= b43_g_ratetable,
	.n_bitrates	= b43_g_ratetable_size,
};

/* 2057 rev 9/14: channels 12–14 cut (same as b43_band_2ghz_limited). */
static struct ieee80211_supported_band b43_band_2GHz_nphy_limited = {
	.band		= NL80211_BAND_2GHZ,
	.channels	= b43_2ghz_chantable,
	.n_channels	= b43_2ghz_chantable_limited_size,
	.bitrates	= b43_g_ratetable,
	.n_bitrates	= b43_g_ratetable_size,
};

static struct ieee80211_supported_band b43_band_2ghz_limited = {
	.band		= NL80211_BAND_2GHZ,
	.channels	= b43_2ghz_chantable,
	.n_channels	= b43_2ghz_chantable_limited_size,
	.bitrates	= b43_g_ratetable,
	.n_bitrates	= b43_g_ratetable_size,
};

static void b43_wireless_core_exit(struct b43_wldev *dev);
static int b43_wireless_core_init(struct b43_wldev *dev);
static struct b43_wldev *b43_ac56_dumpdev;	/* phydump (debug) */
static struct b43_wldev * b43_wireless_core_stop(struct b43_wldev *dev);
static int b43_wireless_core_start(struct b43_wldev *dev);
static void b43_op_bss_info_changed(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    struct ieee80211_bss_conf *conf,
				    u64 changed);

static int b43_ratelimit(struct b43_wl *wl)
{
	if (!wl || !wl->current_dev)
		return 1;
	if (b43_status(wl->current_dev) < B43_STAT_STARTED)
		return 1;
	/* We are up and running.
	 * Ratelimit the messages to avoid DoS over the net. */
	return net_ratelimit();
}

void b43info(struct b43_wl *wl, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_INFO)
		return;
	if (!b43_ratelimit(wl))
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	printk(KERN_INFO "b43-%s: %pV",
	       (wl && wl->hw) ? wiphy_name(wl->hw->wiphy) : "wlan", &vaf);

	va_end(args);
}

void b43err(struct b43_wl *wl, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_ERROR)
		return;
	if (!b43_ratelimit(wl))
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	printk(KERN_ERR "b43-%s ERROR: %pV",
	       (wl && wl->hw) ? wiphy_name(wl->hw->wiphy) : "wlan", &vaf);

	va_end(args);
}

void b43warn(struct b43_wl *wl, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_WARN)
		return;
	if (!b43_ratelimit(wl))
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	printk(KERN_WARNING "b43-%s warning: %pV",
	       (wl && wl->hw) ? wiphy_name(wl->hw->wiphy) : "wlan", &vaf);

	va_end(args);
}

void b43dbg(struct b43_wl *wl, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (b43_modparam_verbose < B43_VERBOSITY_DEBUG)
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	printk(KERN_DEBUG "b43-%s debug: %pV",
	       (wl && wl->hw) ? wiphy_name(wl->hw->wiphy) : "wlan", &vaf);

	va_end(args);
}

static void b43_ram_write(struct b43_wldev *dev, u16 offset, u32 val)
{
	u32 macctl;

	B43_WARN_ON(offset % 4 != 0);

	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	if (macctl & B43_MACCTL_BE)
		val = swab32(val);

	b43_write32(dev, B43_MMIO_RAM_CONTROL, offset);
	b43_write32(dev, B43_MMIO_RAM_DATA, val);
}

static inline void b43_shm_control_word(struct b43_wldev *dev,
					u16 routing, u16 offset)
{
	u32 control;

	/* "offset" is the WORD offset. */
	control = routing;
	control <<= 16;
	control |= offset;
	b43_write32(dev, B43_MMIO_SHM_CONTROL, control);
}

u32 b43_shm_read32(struct b43_wldev *dev, u16 routing, u16 offset)
{
	u32 ret;

	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			ret = b43_read16(dev, B43_MMIO_SHM_DATA_UNALIGNED);
			b43_shm_control_word(dev, routing, (offset >> 2) + 1);
			ret |= ((u32)b43_read16(dev, B43_MMIO_SHM_DATA)) << 16;

			goto out;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	ret = b43_read32(dev, B43_MMIO_SHM_DATA);
out:
	return ret;
}

u16 b43_shm_read16(struct b43_wldev *dev, u16 routing, u16 offset)
{
	u16 ret;

	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			ret = b43_read16(dev, B43_MMIO_SHM_DATA_UNALIGNED);

			goto out;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	ret = b43_read16(dev, B43_MMIO_SHM_DATA);
out:
	return ret;
}

void b43_shm_write32(struct b43_wldev *dev, u16 routing, u16 offset, u32 value)
{
	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			b43_write16(dev, B43_MMIO_SHM_DATA_UNALIGNED,
				    value & 0xFFFF);
			b43_shm_control_word(dev, routing, (offset >> 2) + 1);
			b43_write16(dev, B43_MMIO_SHM_DATA,
				    (value >> 16) & 0xFFFF);
			return;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	b43_write32(dev, B43_MMIO_SHM_DATA, value);
}

void b43_shm_write16(struct b43_wldev *dev, u16 routing, u16 offset, u16 value)
{
	if (routing == B43_SHM_SHARED) {
		B43_WARN_ON(offset & 0x0001);
		if (offset & 0x0003) {
			/* Unaligned access */
			b43_shm_control_word(dev, routing, offset >> 2);
			b43_write16(dev, B43_MMIO_SHM_DATA_UNALIGNED, value);
			return;
		}
		offset >>= 2;
	}
	b43_shm_control_word(dev, routing, offset);
	b43_write16(dev, B43_MMIO_SHM_DATA, value);
}

/* Read HostFlags */
u64 b43_hf_read(struct b43_wldev *dev)
{
	u64 ret;

	ret = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF3);
	ret <<= 16;
	ret |= b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF2);
	ret <<= 16;
	ret |= b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF1);

	return ret;
}

/* Write HostFlags */
void b43_hf_write(struct b43_wldev *dev, u64 value)
{
	u16 lo, mi, hi;

	lo = (value & 0x00000000FFFFULL);
	mi = (value & 0x0000FFFF0000ULL) >> 16;
	hi = (value & 0xFFFF00000000ULL) >> 32;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF1, lo);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF2, mi);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_HOSTF3, hi);
}

/* Read the firmware capabilities bitmask (Opensource firmware only) */
static u16 b43_fwcapa_read(struct b43_wldev *dev)
{
	B43_WARN_ON(!dev->fw.opensource);
	return b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_FWCAPA);
}

void b43_tsf_read(struct b43_wldev *dev, u64 *tsf)
{
	u32 low, high;

	B43_WARN_ON(dev->dev->core_rev < 3);

	/* The hardware guarantees us an atomic read, if we
	 * read the low register first. */
	low = b43_read32(dev, B43_MMIO_REV3PLUS_TSF_LOW);
	high = b43_read32(dev, B43_MMIO_REV3PLUS_TSF_HIGH);

	*tsf = high;
	*tsf <<= 32;
	*tsf |= low;
}

static void b43_time_lock(struct b43_wldev *dev)
{
	b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_TBTTHOLD);
	/* Commit the write */
	b43_read32(dev, B43_MMIO_MACCTL);
}

static void b43_time_unlock(struct b43_wldev *dev)
{
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_TBTTHOLD, 0);
	/* Commit the write */
	b43_read32(dev, B43_MMIO_MACCTL);
}

static void b43_tsf_write_locked(struct b43_wldev *dev, u64 tsf)
{
	u32 low, high;

	B43_WARN_ON(dev->dev->core_rev < 3);

	low = tsf;
	high = (tsf >> 32);
	/* The hardware guarantees us an atomic write, if we
	 * write the low register first. */
	b43_write32(dev, B43_MMIO_REV3PLUS_TSF_LOW, low);
	b43_write32(dev, B43_MMIO_REV3PLUS_TSF_HIGH, high);
}

void b43_tsf_write(struct b43_wldev *dev, u64 tsf)
{
	b43_time_lock(dev);
	b43_tsf_write_locked(dev, tsf);
	b43_time_unlock(dev);
}

/*
 * Core rev >= 40: wlc_bmac_write_amt @0x15c964. One control write selects
 * object 0x304 at word index * 2 (0x15c9e8..0x15c9f0), then two data writes:
 * address bytes 0..3, then bytes 4..5 with the attribute in the high half
 * (0x15c9a0..0x15c9cc, 0x15ca1c, 0x15ca38). With the valid bit set, wl first
 * reads the entry back the same way and ORs the old attribute in
 * (wlc_bmac_read_amt @0x15b9fc, 0x15c96c..0x15c99c).
 */
static void b43_amt_write(struct b43_wldev *dev, unsigned int idx,
			  const u8 *mac, u16 attr)
{
	u32 lo, hi;

	if (attr & B43_AMT_ATTR_VALID) {
		b43_write32(dev, B43_MMIO_SHM_CONTROL,
			    (B43_SHM_AMT << 16) | (idx * 2));
		b43_read32(dev, B43_MMIO_SHM_DATA);
		attr |= b43_read32(dev, B43_MMIO_SHM_DATA) >> 16;
	}
	lo = mac[0] | (mac[1] << 8) | (mac[2] << 16) | ((u32)mac[3] << 24);
	hi = mac[4] | (mac[5] << 8) | ((u32)attr << 16);
	b43_write32(dev, B43_MMIO_SHM_CONTROL, (B43_SHM_AMT << 16) | (idx * 2));
	b43_write32(dev, B43_MMIO_SHM_DATA, lo);
	b43_write32(dev, B43_MMIO_SHM_DATA, hi);
}

static
void b43_macfilter_set(struct b43_wldev *dev, u16 offset, const u8 *mac)
{
	static const u8 zero_addr[ETH_ALEN] = { 0 };
	u16 data;

	if (!mac)
		mac = zero_addr;

	/*
	 * Core rev >= 40: wl never touches MMIO 0x420/0x422
	 * (wlc_bmac_set_addrmatch returns for core rev > 39, 0x15cb94..0x15cba4)
	 * and uses the AMT instead: our MAC in entry 63 (wlc_set_mac
	 * 0x115a78..0x115a84), the BSSID in entry 62 (wlc_set_bssid
	 * 0x115b78..0x115bd8), attribute 0 for an all-zero BSSID.
	 */
	if (dev->dev->core_rev >= 40) {
		u16 attr;

		if (offset == B43_MACFILTER_SELF) {
			b43_amt_write(dev, B43_AMT_IDX_SELF, mac,
				      B43_AMT_ATTR_SELF);
			return;
		}
		if (offset != B43_MACFILTER_BSSID)
			return;
		if (is_zero_ether_addr(mac))
			attr = 0;
		else if (b43_is_mode(dev->wl, NL80211_IFTYPE_STATION))
			attr = B43_AMT_ATTR_BSSID_STA;
		else
			attr = B43_AMT_ATTR_BSSID_AP;
		b43_amt_write(dev, B43_AMT_IDX_BSSID, mac, attr);
		return;
	}

	offset |= 0x0020;
	b43_write16(dev, B43_MMIO_MACFILTER_CONTROL, offset);

	data = mac[0];
	data |= mac[1] << 8;
	b43_write16(dev, B43_MMIO_MACFILTER_DATA, data);
	data = mac[2];
	data |= mac[3] << 8;
	b43_write16(dev, B43_MMIO_MACFILTER_DATA, data);
	data = mac[4];
	data |= mac[5] << 8;
	b43_write16(dev, B43_MMIO_MACFILTER_DATA, data);
}

static void b43_write_mac_bssid_templates(struct b43_wldev *dev)
{
	const u8 *mac;
	const u8 *bssid;
	u8 mac_bssid[ETH_ALEN * 2];
	int i;
	u32 tmp;

	bssid = dev->wl->bssid;
	mac = dev->wl->mac_addr;

	b43_macfilter_set(dev, B43_MACFILTER_BSSID, bssid);

	memcpy(mac_bssid, mac, ETH_ALEN);
	memcpy(mac_bssid + ETH_ALEN, bssid, ETH_ALEN);

	/* Write our MAC address and BSSID to template ram */
	for (i = 0; i < ARRAY_SIZE(mac_bssid); i += sizeof(u32)) {
		tmp = (u32) (mac_bssid[i + 0]);
		tmp |= (u32) (mac_bssid[i + 1]) << 8;
		tmp |= (u32) (mac_bssid[i + 2]) << 16;
		tmp |= (u32) (mac_bssid[i + 3]) << 24;
		b43_ram_write(dev, 0x20 + i, tmp);
	}
}

/*
 * acphy_ampdu bit 2, the shm words wl's wlc_ampdu_shm_upd 0x14658c
 * writes from wlc_init (0x12f048) when A-MPDU TX and RX are on (wlc_pub
 * bytes 0x49 and 0x4a) and the PHY type is N, HT or AC (11). With the RX
 * factor 64 KiB (rx_factor & 3 == 3, wlc_ampdu_rx_attach 0x1462e8) it
 * writes 0xffff to shm 0xba and 10 to shm 0x3c, otherwise 0x8000 and 5:
 * brcmsmac's M_MIMO_MAXSYM and M_WATCHDOG_8TU (brcms_c_ampdu_shm_upd),
 * which extend the ucode's RX watchdog for long A-MPDUs. b43's 64 KiB
 * factor (b43_ac_setup_ht_cap) takes the first pair. shm 0x3c is b43's
 * B43_SHM_SH_DEFAULTIV, which b43 never writes. wl writes no host flag
 * and no other shm word for A-MPDU RX.
 */
#define B43_SHM_SH_AC_MIMO_MAXSYM	0x00BA
#define B43_SHM_SH_AC_WATCHDOG_8TU	0x003C

static void b43_ac_ampdu_shm(struct b43_wldev *dev)
{
	u16 maxsym, wdog;

	if (!b43_ac_ampdu(dev, B43_AC_AMPDU_SHM) ||
	    dev->phy.type != B43_PHYTYPE_AC)
		return;
	maxsym = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_AC_MIMO_MAXSYM);
	wdog = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_AC_WATCHDOG_8TU);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_AC_MIMO_MAXSYM, 0xFFFF);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_AC_WATCHDOG_8TU, 10);
}

static void b43_upload_card_macaddress(struct b43_wldev *dev)
{
	b43_write_mac_bssid_templates(dev);
	b43_macfilter_set(dev, B43_MACFILTER_SELF, dev->wl->mac_addr);
}

static void b43_set_slot_time(struct b43_wldev *dev, u16 slot_time)
{
	/* slot_time is in usec. */
	/*
	 * Core rev >= 40 (AC): wlc_bmac_update_slot_timing writes the IFS
	 * slot register and shm 0x10 together (0x15ff98..0x15ffe4), and on
	 * 5 GHz always with short slot (wlc_bmac_bsinit 0x160420..0x160434).
	 */
	if (dev->phy.type == B43_PHYTYPE_AC && dev->dev->core_rev >= 40) {
		if (b43_current_band(dev->wl) == NL80211_BAND_5GHZ)
			slot_time = 9;
		b43_write16(dev, B43_MMIO_IFSSLOT, 510 + slot_time);
		b43_shm_write16(dev, B43_SHM_SHARED, 0x0010, slot_time);
		return;
	}
	/* This test used to exit for all but a G PHY. */
	if (b43_current_band(dev->wl) == NL80211_BAND_5GHZ)
		return;
	b43_write16(dev, B43_MMIO_IFSSLOT, 510 + slot_time);
	/* Shared memory location 0x0010 is the slot time and should be
	 * set to slot_time; however, this register is initially 0 and changing
	 * the value adversely affects the transmit rate for BCM4311
	 * devices. Until this behavior is unterstood, delete this step
	 *
	 * b43_shm_write16(dev, B43_SHM_SHARED, 0x0010, slot_time);
	 */
}

static void b43_short_slot_timing_enable(struct b43_wldev *dev)
{
	b43_set_slot_time(dev, 9);
}

static void b43_short_slot_timing_disable(struct b43_wldev *dev)
{
	b43_set_slot_time(dev, 20);
}

/* DummyTransmission function, as documented on
 * https://bcm-v4.sipsolutions.net/802.11/DummyTransmission
 */
void b43_dummy_transmission(struct b43_wldev *dev, bool ofdm, bool pa_on)
{
	struct b43_phy *phy = &dev->phy;
	unsigned int i, max_loop;
	u16 value;
	u32 buffer[5] = {
		0x00000000,
		0x00D40000,
		0x00000000,
		0x01000000,
		0x00000000,
	};

	if (ofdm) {
		max_loop = 0x1E;
		buffer[0] = 0x000201CC;
	} else {
		max_loop = 0xFA;
		buffer[0] = 0x000B846E;
	}

	for (i = 0; i < 5; i++)
		b43_ram_write(dev, i * 4, buffer[i]);

	b43_write16(dev, B43_MMIO_XMTSEL, 0x0000);

	if (dev->dev->core_rev < 11)
		b43_write16(dev, B43_MMIO_WEPCTL, 0x0000);
	else
		b43_write16(dev, B43_MMIO_WEPCTL, 0x0100);

	value = (ofdm ? 0x41 : 0x40);
	b43_write16(dev, B43_MMIO_TXE0_PHYCTL, value);
	if (phy->type == B43_PHYTYPE_N || phy->type == B43_PHYTYPE_LP ||
	    phy->type == B43_PHYTYPE_LCN)
		b43_write16(dev, B43_MMIO_TXE0_PHYCTL1, 0x1A02);

	b43_write16(dev, B43_MMIO_TXE0_WM_0, 0x0000);
	b43_write16(dev, B43_MMIO_TXE0_WM_1, 0x0000);

	b43_write16(dev, B43_MMIO_XMTTPLATETXPTR, 0x0000);
	b43_write16(dev, B43_MMIO_XMTTXCNT, 0x0014);
	b43_write16(dev, B43_MMIO_XMTSEL, 0x0826);
	b43_write16(dev, B43_MMIO_TXE0_CTL, 0x0000);

	if (!pa_on && phy->type == B43_PHYTYPE_N) {
		; /*b43_nphy_pa_override(dev, false) */
	}

	switch (phy->type) {
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LCN:
		b43_write16(dev, B43_MMIO_TXE0_AUX, 0x00D0);
		break;
	case B43_PHYTYPE_LP:
		b43_write16(dev, B43_MMIO_TXE0_AUX, 0x0050);
		break;
	default:
		b43_write16(dev, B43_MMIO_TXE0_AUX, 0x0030);
	}
	b43_read16(dev, B43_MMIO_TXE0_AUX);

	if (phy->radio_ver == 0x2050 && phy->radio_rev <= 0x5)
		b43_radio_write16(dev, 0x0051, 0x0017);
	for (i = 0x00; i < max_loop; i++) {
		value = b43_read16(dev, B43_MMIO_TXE0_STATUS);
		if (value & 0x0080)
			break;
		udelay(10);
	}
	for (i = 0x00; i < 0x0A; i++) {
		value = b43_read16(dev, B43_MMIO_TXE0_STATUS);
		if (value & 0x0400)
			break;
		udelay(10);
	}
	for (i = 0x00; i < 0x19; i++) {
		value = b43_read16(dev, B43_MMIO_IFSSTAT);
		if (!(value & 0x0100))
			break;
		udelay(10);
	}
	if (phy->radio_ver == 0x2050 && phy->radio_rev <= 0x5)
		b43_radio_write16(dev, 0x0051, 0x0037);
}

static void key_write(struct b43_wldev *dev,
		      u8 index, u8 algorithm, const u8 *key)
{
	unsigned int i;
	u32 offset;
	u16 value;
	u16 kidx;

	/* Key index/algo block */
	kidx = b43_kidx_to_fw(dev, index);
	value = ((kidx << 4) | algorithm);
	b43_shm_write16(dev, B43_SHM_SHARED,
			B43_SHM_SH_KEYIDXBLOCK + (kidx * 2), value);

	/* Write the key to the Key Table Pointer offset */
	offset = dev->ktp + (index * B43_SEC_KEYSIZE);
	for (i = 0; i < B43_SEC_KEYSIZE; i += 2) {
		value = key[i];
		value |= (u16) (key[i + 1]) << 8;
		b43_shm_write16(dev, B43_SHM_SHARED, offset + i, value);
	}
}

static void keymac_write(struct b43_wldev *dev, u8 index, const u8 *addr)
{
	u32 addrtmp[2] = { 0, 0, };
	u8 pairwise_keys_start = B43_NR_GROUP_KEYS * 2;

	if (b43_new_kidx_api(dev))
		pairwise_keys_start = B43_NR_GROUP_KEYS;

	B43_WARN_ON(index < pairwise_keys_start);
	/* We have four default TX keys and possibly four default RX keys.
	 * Physical mac 0 is mapped to physical key 4 or 8, depending
	 * on the firmware version.
	 * So we must adjust the index here.
	 */
	index -= pairwise_keys_start;
	B43_WARN_ON(index >= B43_NR_PAIRWISE_KEYS);

	if (addr) {
		addrtmp[0] = addr[0];
		addrtmp[0] |= ((u32) (addr[1]) << 8);
		addrtmp[0] |= ((u32) (addr[2]) << 16);
		addrtmp[0] |= ((u32) (addr[3]) << 24);
		addrtmp[1] = addr[4];
		addrtmp[1] |= ((u32) (addr[5]) << 8);
	}

	/* Receive match transmitter address (RCMTA) mechanism */
	b43_shm_write32(dev, B43_SHM_RCMTA,
			(index * 2) + 0, addrtmp[0]);
	b43_shm_write16(dev, B43_SHM_RCMTA,
			(index * 2) + 1, addrtmp[1]);
}

/* The ucode will use phase1 key with TEK key to decrypt rx packets.
 * When a packet is received, the iv32 is checked.
 * - if it doesn't the packet is returned without modification (and software
 *   decryption can be done). That's what happen when iv16 wrap.
 * - if it does, the rc4 key is computed, and decryption is tried.
 *   Either it will success and B43_RX_MAC_DEC is returned,
 *   either it fails and B43_RX_MAC_DEC|B43_RX_MAC_DECERR is returned
 *   and the packet is not usable (it got modified by the ucode).
 * So in order to never have B43_RX_MAC_DECERR, we should provide
 * a iv32 and phase1key that match. Because we drop packets in case of
 * B43_RX_MAC_DECERR, if we have a correct iv32 but a wrong phase1key, all
 * packets will be lost without higher layer knowing (ie no resync possible
 * until next wrap).
 *
 * NOTE : this should support 50 key like RCMTA because
 * (B43_SHM_SH_KEYIDXBLOCK - B43_SHM_SH_TKIPTSCTTAK)/14 = 50
 */
static void rx_tkip_phase1_write(struct b43_wldev *dev, u8 index, u32 iv32,
		u16 *phase1key)
{
	unsigned int i;
	u32 offset;
	u8 pairwise_keys_start = B43_NR_GROUP_KEYS * 2;

	if (!modparam_hwtkip)
		return;

	if (b43_new_kidx_api(dev))
		pairwise_keys_start = B43_NR_GROUP_KEYS;

	B43_WARN_ON(index < pairwise_keys_start);
	/* We have four default TX keys and possibly four default RX keys.
	 * Physical mac 0 is mapped to physical key 4 or 8, depending
	 * on the firmware version.
	 * So we must adjust the index here.
	 */
	index -= pairwise_keys_start;
	B43_WARN_ON(index >= B43_NR_PAIRWISE_KEYS);

	if (b43_debug(dev, B43_DBG_KEYS)) {
		b43dbg(dev->wl, "rx_tkip_phase1_write : idx 0x%x, iv32 0x%x\n",
				index, iv32);
	}
	/* Write the key to the  RX tkip shared mem */
	offset = B43_SHM_SH_TKIPTSCTTAK + index * (10 + 4);
	for (i = 0; i < 10; i += 2) {
		b43_shm_write16(dev, B43_SHM_SHARED, offset + i,
				phase1key ? phase1key[i / 2] : 0);
	}
	b43_shm_write16(dev, B43_SHM_SHARED, offset + i, iv32);
	b43_shm_write16(dev, B43_SHM_SHARED, offset + i + 2, iv32 >> 16);
}

static void b43_op_update_tkip_key(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
				   struct ieee80211_key_conf *keyconf,
				   struct ieee80211_sta *sta,
				   u32 iv32, u16 *phase1key)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;
	int index = keyconf->hw_key_idx;

	if (B43_WARN_ON(!modparam_hwtkip))
		return;

	/* This is only called from the RX path through mac80211, where
	 * our mutex is already locked. */
	B43_WARN_ON(!mutex_is_locked(&wl->mutex));
	dev = wl->current_dev;
	B43_WARN_ON(!dev || b43_status(dev) < B43_STAT_INITIALIZED);

	keymac_write(dev, index, NULL);	/* First zero out mac to avoid race */

	rx_tkip_phase1_write(dev, index, iv32, phase1key);
	/* only pairwise TKIP keys are supported right now */
	if (WARN_ON(!sta))
		return;
	keymac_write(dev, index, sta->addr);
}

static void do_key_write(struct b43_wldev *dev,
			 u8 index, u8 algorithm,
			 const u8 *key, size_t key_len, const u8 *mac_addr)
{
	u8 buf[B43_SEC_KEYSIZE] = { 0, };
	u8 pairwise_keys_start = B43_NR_GROUP_KEYS * 2;

	if (b43_new_kidx_api(dev))
		pairwise_keys_start = B43_NR_GROUP_KEYS;

	B43_WARN_ON(index >= ARRAY_SIZE(dev->key));
	B43_WARN_ON(key_len > B43_SEC_KEYSIZE);

	if (index >= pairwise_keys_start)
		keymac_write(dev, index, NULL);	/* First zero out mac. */
	if (algorithm == B43_SEC_ALGO_TKIP) {
		/*
		 * We should provide an initial iv32, phase1key pair.
		 * We could start with iv32=0 and compute the corresponding
		 * phase1key, but this means calling ieee80211_get_tkip_key
		 * with a fake skb (or export other tkip function).
		 * Because we are lazy we hope iv32 won't start with
		 * 0xffffffff and let's b43_op_update_tkip_key provide a
		 * correct pair.
		 */
		rx_tkip_phase1_write(dev, index, 0xffffffff, (u16*)buf);
	} else if (index >= pairwise_keys_start) /* clear it */
		rx_tkip_phase1_write(dev, index, 0, NULL);
	if (key)
		memcpy(buf, key, key_len);
	key_write(dev, index, algorithm, buf);
	if (index >= pairwise_keys_start)
		keymac_write(dev, index, mac_addr);

	dev->key[index].algorithm = algorithm;
}

static int b43_key_write(struct b43_wldev *dev,
			 int index, u8 algorithm,
			 const u8 *key, size_t key_len,
			 const u8 *mac_addr,
			 struct ieee80211_key_conf *keyconf)
{
	int i;
	int pairwise_keys_start;

	/* For ALG_TKIP the key is encoded as a 256-bit (32 byte) data block:
	 * 	- Temporal Encryption Key (128 bits)
	 * 	- Temporal Authenticator Tx MIC Key (64 bits)
	 * 	- Temporal Authenticator Rx MIC Key (64 bits)
	 *
	 * 	Hardware only store TEK
	 */
	if (algorithm == B43_SEC_ALGO_TKIP && key_len == 32)
		key_len = 16;
	if (key_len > B43_SEC_KEYSIZE)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(dev->key); i++) {
		/* Check that we don't already have this key. */
		B43_WARN_ON(dev->key[i].keyconf == keyconf);
	}
	if (index < 0) {
		/* Pairwise key. Get an empty slot for the key. */
		if (b43_new_kidx_api(dev))
			pairwise_keys_start = B43_NR_GROUP_KEYS;
		else
			pairwise_keys_start = B43_NR_GROUP_KEYS * 2;
		for (i = pairwise_keys_start;
		     i < pairwise_keys_start + B43_NR_PAIRWISE_KEYS;
		     i++) {
			B43_WARN_ON(i >= ARRAY_SIZE(dev->key));
			if (!dev->key[i].keyconf) {
				/* found empty */
				index = i;
				break;
			}
		}
		if (index < 0) {
			b43warn(dev->wl, "Out of hardware key memory\n");
			return -ENOSPC;
		}
	} else
		B43_WARN_ON(index > 3);

	do_key_write(dev, index, algorithm, key, key_len, mac_addr);
	if ((index <= 3) && !b43_new_kidx_api(dev)) {
		/* Default RX key */
		B43_WARN_ON(mac_addr);
		do_key_write(dev, index + 4, algorithm, key, key_len, NULL);
	}
	keyconf->hw_key_idx = index;
	dev->key[index].keyconf = keyconf;

	return 0;
}

static int b43_key_clear(struct b43_wldev *dev, int index)
{
	if (B43_WARN_ON((index < 0) || (index >= ARRAY_SIZE(dev->key))))
		return -EINVAL;
	do_key_write(dev, index, B43_SEC_ALGO_NONE,
		     NULL, B43_SEC_KEYSIZE, NULL);
	if ((index <= 3) && !b43_new_kidx_api(dev)) {
		do_key_write(dev, index + 4, B43_SEC_ALGO_NONE,
			     NULL, B43_SEC_KEYSIZE, NULL);
	}
	dev->key[index].keyconf = NULL;

	return 0;
}

static void b43_clear_keys(struct b43_wldev *dev)
{
	int i, count;

	if (b43_new_kidx_api(dev))
		count = B43_NR_GROUP_KEYS + B43_NR_PAIRWISE_KEYS;
	else
		count = B43_NR_GROUP_KEYS * 2 + B43_NR_PAIRWISE_KEYS;
	for (i = 0; i < count; i++)
		b43_key_clear(dev, i);
}

static void b43_dump_keymemory(struct b43_wldev *dev)
{
	unsigned int i, index, count, offset, pairwise_keys_start;
	u8 mac[ETH_ALEN];
	u16 algo;
	u32 rcmta0;
	u16 rcmta1;
	u64 hf;
	struct b43_key *key;

	if (!b43_debug(dev, B43_DBG_KEYS))
		return;

	hf = b43_hf_read(dev);
	b43dbg(dev->wl, "Hardware key memory dump:  USEDEFKEYS=%u\n",
	       !!(hf & B43_HF_USEDEFKEYS));
	if (b43_new_kidx_api(dev)) {
		pairwise_keys_start = B43_NR_GROUP_KEYS;
		count = B43_NR_GROUP_KEYS + B43_NR_PAIRWISE_KEYS;
	} else {
		pairwise_keys_start = B43_NR_GROUP_KEYS * 2;
		count = B43_NR_GROUP_KEYS * 2 + B43_NR_PAIRWISE_KEYS;
	}
	for (index = 0; index < count; index++) {
		key = &(dev->key[index]);
		printk(KERN_DEBUG "Key slot %02u: %s",
		       index, (key->keyconf == NULL) ? " " : "*");
		offset = dev->ktp + (index * B43_SEC_KEYSIZE);
		for (i = 0; i < B43_SEC_KEYSIZE; i += 2) {
			u16 tmp = b43_shm_read16(dev, B43_SHM_SHARED, offset + i);
			printk("%02X%02X", (tmp & 0xFF), ((tmp >> 8) & 0xFF));
		}

		algo = b43_shm_read16(dev, B43_SHM_SHARED,
				      B43_SHM_SH_KEYIDXBLOCK + (index * 2));
		printk("   Algo: %04X/%02X", algo, key->algorithm);

		if (index >= pairwise_keys_start) {
			if (key->algorithm == B43_SEC_ALGO_TKIP) {
				printk("   TKIP: ");
				offset = B43_SHM_SH_TKIPTSCTTAK + (index - 4) * (10 + 4);
				for (i = 0; i < 14; i += 2) {
					u16 tmp = b43_shm_read16(dev, B43_SHM_SHARED, offset + i);
					printk("%02X%02X", (tmp & 0xFF), ((tmp >> 8) & 0xFF));
				}
			}
			rcmta0 = b43_shm_read32(dev, B43_SHM_RCMTA,
						((index - pairwise_keys_start) * 2) + 0);
			rcmta1 = b43_shm_read16(dev, B43_SHM_RCMTA,
						((index - pairwise_keys_start) * 2) + 1);
			*((__le32 *)(&mac[0])) = cpu_to_le32(rcmta0);
			*((__le16 *)(&mac[4])) = cpu_to_le16(rcmta1);
			printk("   MAC: %pM", mac);
		} else
			printk("   DEFAULT KEY");
		printk("\n");
	}
}

void b43_power_saving_ctl_bits(struct b43_wldev *dev, unsigned int ps_flags)
{
	u32 macctl;
	u16 ucstat;
	bool hwps;
	bool awake;
	int i;

	B43_WARN_ON((ps_flags & B43_PS_ENABLED) &&
		    (ps_flags & B43_PS_DISABLED));
	B43_WARN_ON((ps_flags & B43_PS_AWAKE) && (ps_flags & B43_PS_ASLEEP));

	if (ps_flags & B43_PS_ENABLED) {
		hwps = true;
	} else if (ps_flags & B43_PS_DISABLED) {
		hwps = false;
	} else {
		//TODO: If powersave is not off and FIXME is not set and we are not in adhoc
		//      and thus is not an AP and we are associated, set bit 25
	}
	if (ps_flags & B43_PS_AWAKE) {
		awake = true;
	} else if (ps_flags & B43_PS_ASLEEP) {
		awake = false;
	} else {
		//TODO: If the device is awake or this is an AP, or we are scanning, or FIXME,
		//      or we are associated, or FIXME, or the latest PS-Poll packet sent was
		//      successful, set bit26
	}

/* FIXME: For now we force awake-on and hwps-off */
	hwps = false;
	awake = true;

	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	if (hwps)
		macctl |= B43_MACCTL_HWPS;
	else
		macctl &= ~B43_MACCTL_HWPS;
	if (awake)
		macctl |= B43_MACCTL_AWAKE;
	else
		macctl &= ~B43_MACCTL_AWAKE;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);
	/* Commit write */
	b43_read32(dev, B43_MMIO_MACCTL);
	if (awake && dev->dev->core_rev >= 5) {
		/* Wait for the microcode to wake up. */
		for (i = 0; i < 100; i++) {
			ucstat = b43_shm_read16(dev, B43_SHM_SHARED,
						B43_SHM_SH_UCODESTAT);
			if (ucstat != B43_SHM_SH_UCODESTAT_SLEEP)
				break;
			udelay(10);
		}
	}
}

/* https://bcm-v4.sipsolutions.net/802.11/PHY/BmacCorePllReset */
void b43_wireless_core_phy_pll_reset(struct b43_wldev *dev)
{
	struct bcma_drv_cc *bcma_cc __maybe_unused;
	struct ssb_chipcommon *ssb_cc __maybe_unused;

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_cc = &dev->dev->bdev->bus->drv_cc;

		bcma_cc_write32(bcma_cc, BCMA_CC_PMU_CHIPCTL_ADDR, 0);
		bcma_cc_mask32(bcma_cc, BCMA_CC_PMU_CHIPCTL_DATA, ~0x4);
		bcma_cc_set32(bcma_cc, BCMA_CC_PMU_CHIPCTL_DATA, 0x4);
		bcma_cc_mask32(bcma_cc, BCMA_CC_PMU_CHIPCTL_DATA, ~0x4);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		ssb_cc = &dev->dev->sdev->bus->chipco;

		chipco_write32(ssb_cc, SSB_CHIPCO_CHIPCTL_ADDR, 0);
		chipco_mask32(ssb_cc, SSB_CHIPCO_CHIPCTL_DATA, ~0x4);
		chipco_set32(ssb_cc, SSB_CHIPCO_CHIPCTL_DATA, 0x4);
		chipco_mask32(ssb_cc, SSB_CHIPCO_CHIPCTL_DATA, ~0x4);
		break;
#endif
	}
}

#ifdef CPTCFG_B43_BCMA
static void b43_bcma_phy_reset(struct b43_wldev *dev)
{
	u32 flags;

#ifdef CPTCFG_B43_PHY_AC
	/*
	 * acphy_initorder 1: wl's PHY reset pulse for PHY type 11
	 * (wlc_bmac_phy_reset + wlc_bmac_core_phy_clk(1), phy_ac.c). Not on
	 * the first attach reset: phy.type is set by b43_phy_versioning.
	 */
	if (dev->phy.type == B43_PHYTYPE_AC &&
	    (b43_modparam_ac_initorder & B43_AC_INITORDER_PHYRESET)) {
		b43_phy_ac_phy_reset_wl(dev);
		return;
	}
#endif

	/* Put PHY into reset */
	flags = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
	flags |= B43_BCMA_IOCTL_PHY_RESET;
	/*
	 * The AC PHY's 40 MHz channel set writes 0x80 into the bandwidth
	 * field (wlc_bmac_bw_reset); OR-ing 0x40 onto that
	 * would give 0xc0 (80 MHz), so clear the field first, as wl's
	 * si_core_cflags replaces it (0x15d334..0x15d354).
	 */
	if (dev->phy.type == B43_PHYTYPE_AC)
		flags &= ~B43_BCMA_IOCTL_PHY_BW;
	flags |= B43_BCMA_IOCTL_PHY_BW_20MHZ; /* Make 20 MHz def */
	bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, flags);
	udelay(2);

	b43_phy_take_out_of_reset(dev);
}

/*
 * BCM4360 chip rev >= 3: the PMU resource setup bcma has no case for
 * ("PMU resource config unknown or not needed for device 0x4360"). Values
 * from Broadcom's si_pmu_res_init in the stock R6400 kernel (2.6.36.4brcmarm):
 * up/down timer table bcm4360_res_updown {6, 0x00100001}, no dependency table,
 * PLL control 6 and 14 = 0x09048560 for chip rev <= 3, max resource mask
 * 0x1ff, min left 0. All of it only when chip status bit 5 is clear; with it
 * set the stock kernel leaves both masks 0 and writes no PLL fix.
 *
 * Every write is gated on a read showing it is needed, so a chip that is
 * already set up is left alone and a second reset writes nothing. Selecting
 * the table entry (0x620) and the PLL control index (0x660) are writes too;
 * the values behind them cannot be read without them.
 */
/*
 * BCM53573 (d11 rev 56): bcma has no PMU resource table for this chip
 * ("PMU resource config unknown ... 0xD145"), and after a cold boot the
 * PMU max resource mask is 0x000BFFFF. Resources 18 and 20 are then missing, the
 * d11 PHY PLL request (CLKCTLST 0x200) never completes ("PLL enable
 * timeout", CLKCTLST 0x050b0342 instead of 0x070b0342) and the microcode
 * does not respond. wl 9.10 leaves the mask at 0x001FFFFF; it survives a
 * warm reboot, which is why b43 only worked after wl had run. Adding
 * resources 18 and 20 to the max mask is enough (checked with devmem on
 * a cold boot; 20 alone is not).
 */
#define B43_53573_PMU_RES	0x00140000	/* resources 18 and 20 */
static void b43_bcma_53573_pmu_res(struct b43_wldev *dev)
{
	struct bcma_bus *bus = dev->dev->bdev->bus;
	struct bcma_drv_cc *cc = &bus->drv_cc;
	u32 maxres;

	if (bus->chipinfo.id != BCMA_CHIP_ID_BCM53573)
		return;
	if (!(cc->capabilities & BCMA_CC_CAP_PMU) || !cc->pmu.core)
		return;
	maxres = bcma_pmu_read32(cc, BCMA_CC_PMU_MAXRES_MSK);
	if ((maxres & B43_53573_PMU_RES) == B43_53573_PMU_RES)
		return;
	bcma_pmu_write32(cc, BCMA_CC_PMU_MAXRES_MSK, maxres | B43_53573_PMU_RES);
	b43info(dev->wl, "53573 PMU: max_res_mask 0x%08x -> 0x%08x (resources 18 and 20 for the d11 PHY PLL)\n",
		maxres, bcma_pmu_read32(cc, BCMA_CC_PMU_MAXRES_MSK));
}

static void b43_bcma_4360_pmu_res(struct b43_wldev *dev)
{
	struct bcma_bus *bus = dev->dev->bdev->bus;
	struct bcma_drv_cc *cc = &bus->drv_cc;
	static const u32 pll_regs[] = { 6, 14 };
	u32 chipst, updn, maxres, val;
	bool pll_upd = false, wrote = false;
	int i;

	if (bus->chipinfo.id != BCMA_CHIP_ID_BCM4360 || bus->chipinfo.rev < 3)
		return;
	if (!(cc->capabilities & BCMA_CC_CAP_PMU) || !cc->pmu.core)
		return;

	chipst = bcma_cc_read32(cc, BCMA_CC_CHIPSTAT);
	if (chipst & 0x20) {
		b43info(dev->wl, "4360 PMU: chipst 0x%08x bit 5 set, nothing written\n",
			chipst);
		return;
	}

	/* Up/down timer of resource 6 (RES_TABSEL 0x620, RES_UPDNTM 0x628) */
	bcma_pmu_write32(cc, BCMA_CC_PMU_RES_TABSEL, 6);
	updn = bcma_pmu_read32(cc, BCMA_CC_PMU_RES_UPDNTM);
	if (updn != 0x00100001) {
		bcma_pmu_write32(cc, BCMA_CC_PMU_RES_UPDNTM, 0x00100001);
		wrote = true;
	}

	/* PLL control 6 and 14, chip rev <= 3 only, then PLL_UPD */
	if (bus->chipinfo.rev <= 3) {
		for (i = 0; i < ARRAY_SIZE(pll_regs); i++) {
			val = bcma_chipco_pll_read(cc, pll_regs[i]);
			if (val == 0x09048560)
				continue;
			b43info(dev->wl, "4360 PMU: PLL ctl %u 0x%08x -> 0x09048560\n",
				pll_regs[i], val);
			bcma_chipco_pll_write(cc, pll_regs[i], 0x09048560);
			pll_upd = true;
		}
		if (pll_upd)
			bcma_pmu_set32(cc, BCMA_CC_PMU_CTL,
				       BCMA_CC_PMU_CTL_PLL_UPD);
	}

	/* Max resource mask: raise to include 0x1ff, never clear a bit */
	maxres = bcma_pmu_read32(cc, BCMA_CC_PMU_MAXRES_MSK);
	if ((maxres & 0x1ff) != 0x1ff) {
		bcma_pmu_write32(cc, BCMA_CC_PMU_MAXRES_MSK, maxres | 0x1ff);
		wrote = true;
	}

	b43info(dev->wl, "4360 PMU: chipst 0x%08x updn6 0x%08x maxres 0x%08x%s\n",
		chipst, updn, maxres,
		(wrote || pll_upd) ? ", written" : ", already set");

	/* Let resources come up, as bcma_pmu_resources_init does */
	if (wrote || pll_upd)
		usleep_range(2000, 2500);
}

/*
 * R6400 BCM4360 (core rev 42, PHY type 11): the PMU and GPIO parts of
 * wl's TX front-end setup that b43 and bcma do not do.
 *
 * acphy_fem bit 0: RF LDO on. wl's wlc_bmac_init calls si_pmu_rfldo(sih, 1)
 *   for chips 0x4360/0x4352/0xaa06 (wl.ko 0x1625a4..0x1625d0), before the
 *   ucode download; the stock kernel's si_pmu_rfldo (0xc02c9ac4) is
 *   si_pmu_regcontrol(sih, 0, 0x2, on ? 0 : 0x2): PMU regulator control 0
 *   bit 1 cleared. wl sets it again at the end of attach (0x164e10) and in
 *   wlc_bmac_hw_down (0x15eb90). Live 2026-10-06 14:35 UTC: regctl 0 reads
 *   0x00000004 (bit 1 already 0), so on our unit this is a no-op kept for
 *   parity with wl.
 * acphy_fem bit 1: GPIOs back to ChipCommon. wl's wlc_phy_init_acphy calls
 *   si_gpiocontrol(sih, 0xffff, 0, 0) (0x50080..0x5009c), ChipCommon 0x6c
 *   bits 0..15 = 0 (kernel 0xc02c6bac, PCI bus: no reservation mask), after
 *   wlc_bmac_init's own gpiocontrol, which is (0, 0) on this board
 *   (0x162744..0x16294c: no antsel GPIOs, boardflags bit 1 clear).
 *   b43_gpio_init gives GPIOs 0..3 to the d11 core instead.
 */
static int b43_modparam_ac56_rx07 = 1;
module_param_named(ac56_rx07, b43_modparam_ac56_rx07, int, 0444);
MODULE_PARM_DESC(ac56_rx07, "core rev 56: advertise VHT RX MCS 0..7 only (default 1)");

static int modparam_acphy_fem = 0x3;
module_param_named(acphy_fem, modparam_acphy_fem, int, 0444);
MODULE_PARM_DESC(acphy_fem, "BCM4360 core rev >= 40: bit 0 RF LDO on (PMU regctl 0 bit 1 = 0), bit 1 GPIO control 0..15 to ChipCommon");

static bool b43_ac_fem_chip(struct b43_wldev *dev)
{
	struct bcma_bus *bus;

	if (dev->dev->bus_type != B43_BUS_BCMA || dev->dev->core_rev < 40 ||
	    dev->phy.type != B43_PHYTYPE_AC)
		return false;
	bus = dev->dev->bdev->bus;
	return bus->chipinfo.id == BCMA_CHIP_ID_BCM4360 ||
	       bus->chipinfo.id == 0x4352 || bus->chipinfo.id == 0xaa06;
}

/* wl 0x1625d0 -> si_pmu_rfldo(sih, 1), kernel 0xc02c9af0..0xc02c9b04 */
static void b43_ac_rfldo_on(struct b43_wldev *dev)
{
	struct bcma_drv_cc *cc;
	u32 before;

	if (!(modparam_acphy_fem & 0x1) || !b43_ac_fem_chip(dev))
		return;
	cc = &dev->dev->bdev->bus->drv_cc;
	if (!(cc->capabilities & BCMA_CC_CAP_PMU) || !cc->pmu.core)
		return;
	bcma_pmu_write32(cc, BCMA_CC_PMU_REGCTL_ADDR, 0);
	before = bcma_pmu_read32(cc, BCMA_CC_PMU_REGCTL_DATA);
	bcma_chipco_regctl_maskset(cc, 0, ~0x2, 0);
	b43info(dev->wl, "FEM: PMU regctl 0 0x%08x -> 0x%08x (RF LDO on, wl 0x1625d0)\n",
		before, before & ~0x2);
}

/* wl 0x5009c -> si_gpiocontrol(sih, 0xffff, 0, 0) */
static void b43_ac_gpio_to_cc(struct b43_wldev *dev)
{
	struct bcma_drv_cc *cc;
	u32 before;

	if (!(modparam_acphy_fem & 0x2) || !b43_ac_fem_chip(dev))
		return;
	cc = &dev->dev->bdev->bus->drv_cc;
	before = bcma_cc_read32(cc, BCMA_CC_GPIOCTL);
	bcma_chipco_gpio_control(cc, 0xffff, 0);
	b43info(dev->wl, "FEM: ChipCommon gpiocontrol 0x%08x -> 0x%08x (wl 0x5009c)\n",
		before, before & ~0xffff);
}

/*
 * PMU regulator control 0 bits 20..24, wlc_phy_init_acphy 0x4ffec..0x50048:
 * n = PHY 0x000 & 0xf; n == 0: v = f, or 5 when f is 0; n == 1: v = f - 3
 * when f > 3, else 0; n > 1: no write. f is OTP word 16 bits 8..12
 * (wlc_phy_attach_acphy otp_read_word 0x472d8, ubfx 0x47300), which b43
 * cannot read, so it is a parameter; -1 (default) writes nothing. wl
 * writes this after the radio power-on and before the first channel set;
 * here it runs just before b43_phy_init (before the radio power-on).
 * Live 2026-10-06 14:35 UTC: regctl 0 = 0x00000004 with our b43 up, so
 * the field is 0 where wl writes 5 for a zero OTP field on nibble 0.
 */
static int modparam_acphy_otp16 = -1;
module_param_named(acphy_otp16, modparam_acphy_otp16, int, 0444);
MODULE_PARM_DESC(acphy_otp16, "BCM4360: OTP word 16 bits 8..12 for PMU regctl 0 bits 20..24 (wl 0x50048); -1 skips");

static void b43_ac_regctl0_paldo(struct b43_wldev *dev)
{
	struct bcma_drv_cc *cc;
	u32 f, v, before;
	u16 n;

	if (modparam_acphy_otp16 < 0 || !b43_ac_fem_chip(dev))
		return;
	cc = &dev->dev->bdev->bus->drv_cc;
	if (!(cc->capabilities & BCMA_CC_CAP_PMU) || !cc->pmu.core)
		return;
	n = b43_phy_read(dev, 0x000) & 0xf;			/* 0x4fff4 */
	if (n > 1) {						/* 0x50000 */
		b43info(dev->wl, "FEM: PHY 0x000 nibble %u, regctl 0 bits 20..24 not written (wl 0x50000)\n", n);
		return;
	}
	f = modparam_acphy_otp16 & 0x1f;
	if (n == 0)
		v = f ? f : 5;					/* 0x5000c..0x50018 */
	else
		v = f > 3 ? f - 3 : 0;				/* 0x50020..0x50030 */
	bcma_pmu_write32(cc, BCMA_CC_PMU_REGCTL_ADDR, 0);
	before = bcma_pmu_read32(cc, BCMA_CC_PMU_REGCTL_DATA);
	bcma_chipco_regctl_maskset(cc, 0, ~0x01f00000, v << 20);	/* 0x50048 */
	b43info(dev->wl, "FEM: PMU regctl 0 0x%08x, bits 20..24 <- %u (PHY 0x000 nibble %u, OTP field %u)\n",
		before, v, n, f);
}

static void b43_bcma_wireless_core_reset(struct b43_wldev *dev, bool gmode)
{
	u32 req = B43_BCMA_CLKCTLST_80211_PLL_REQ |
		  B43_BCMA_CLKCTLST_PHY_PLL_REQ;
	u32 status = B43_BCMA_CLKCTLST_80211_PLL_ST |
		     B43_BCMA_CLKCTLST_PHY_PLL_ST;
	u32 flags;

	b43_bcma_4360_pmu_res(dev);
	b43_bcma_53573_pmu_res(dev);

	flags = B43_BCMA_IOCTL_PHY_CLKEN;
	if (gmode)
		flags |= B43_BCMA_IOCTL_GMODE;
	b43_device_enable(dev, flags);

	if (dev->phy.type == B43_PHYTYPE_AC) {
		u16 tmp;

		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
		tmp &= ~B43_BCMA_IOCTL_DAC;
		tmp |= 0x100;
		bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, tmp);

		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
		tmp &= ~B43_BCMA_IOCTL_PHY_CLKEN;
		bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, tmp);

		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
		tmp |= B43_BCMA_IOCTL_PHY_CLKEN;
		bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, tmp);
	}

	bcma_core_set_clockmode(dev->dev->bdev, BCMA_CLKMODE_FAST);
	b43_bcma_phy_reset(dev);
	bcma_core_pll_ctl(dev->dev->bdev, req, status, true);

	/*
	 * BCM53573 (d11 rev 56): two d11 cores share the 2x2 PHY. wl resets
	 * them together (ai_d11rsdb_core_reset in the AC6 wl.ko: the same
	 * IOCTL on both wrappers, both out of reset). Bring unit 1 out of
	 * reset with unit 0's final IOCTL.
	 */
	if (dev->dev->core_rev == 56) {
		struct bcma_device *c0 = dev->dev->bdev;
		struct bcma_device *c1 = bcma_find_core_unit(c0->bus,
							     BCMA_CORE_80211, 1);

		if (c1) {
			if (!bcma_core_is_enabled(c1))
				bcma_core_enable(c1, 0);
			bcma_awrite32(c1, BCMA_IOCTL, bcma_aread32(c0, BCMA_IOCTL));
			b43info(dev->wl, "rev56: d11 unit 1 IOCTL 0x%08x RESET_CTL 0x%08x (unit 0 IOCTL 0x%08x)\n",
				bcma_aread32(c1, BCMA_IOCTL),
				bcma_aread32(c1, BCMA_RESET_CTL),
				bcma_aread32(c0, BCMA_IOCTL));
		} else {
			b43warn(dev->wl, "rev56: d11 unit 1 not found\n");
		}
	}
}
#endif

#ifdef CPTCFG_B43_SSB
static void b43_ssb_wireless_core_reset(struct b43_wldev *dev, bool gmode)
{
	u32 flags = 0;

	if (gmode)
		flags |= B43_TMSLOW_GMODE;
	flags |= B43_TMSLOW_PHYCLKEN;
	flags |= B43_TMSLOW_PHYRESET;
	if (dev->phy.type == B43_PHYTYPE_N)
		flags |= B43_TMSLOW_PHY_BANDWIDTH_20MHZ; /* Make 20 MHz def */
	b43_device_enable(dev, flags);
	msleep(2);		/* Wait for the PLL to turn on. */

	b43_phy_take_out_of_reset(dev);
}
#endif

void b43_wireless_core_reset(struct b43_wldev *dev, bool gmode)
{
	u32 macctl;

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		b43_bcma_wireless_core_reset(dev, gmode);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		b43_ssb_wireless_core_reset(dev, gmode);
		break;
#endif
	}

	/* Turn Analog ON, but only if we already know the PHY-type.
	 * This protects against very early setup where we don't know the
	 * PHY-type, yet. wireless_core_reset will be called once again later,
	 * when we know the PHY-type. */
	if (dev->phy.ops)
		dev->phy.ops->switch_analog(dev, 1);

	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	macctl &= ~B43_MACCTL_GMODE;
	if (gmode)
		macctl |= B43_MACCTL_GMODE;
	macctl |= B43_MACCTL_IHR_ENABLED;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);
}

/*
 * Core rev >= 40 (ucode 907.113): each status is two four-word packages,
 * read from 0x170..0x17c twice, in wl.ko's order (wlc_bmac_txstatus: valid
 * bit of 0x170 at 0x12570..0x12584, 0xffffffff check at 0x123d8, first
 * package 0x123e0..0x12400, second 0x1247c..0x124ac). A second package
 * without its valid bit ends the call and drops the first, as wl does
 * (0x124c0..0x124c8, return at 0x125d0).
 */
static bool b43_read_txstatus_rev40(struct b43_wldev *dev, u32 *s, u32 *t)
{
	s[0] = b43_read32(dev, B43_MMIO_XMITSTAT_0);
	if (!(s[0] & B43_TXST40_S1_VALID))
		return false;
	if (s[0] == 0xFFFFFFFF)
		return false;
	s[1] = b43_read32(dev, B43_MMIO_XMITSTAT_1);
	s[2] = b43_read32(dev, B43_MMIO_XMITSTAT_2);
	s[3] = b43_read32(dev, B43_MMIO_XMITSTAT_3);
	t[0] = b43_read32(dev, B43_MMIO_XMITSTAT_0);
	t[1] = b43_read32(dev, B43_MMIO_XMITSTAT_1);
	t[2] = b43_read32(dev, B43_MMIO_XMITSTAT_2);
	t[3] = b43_read32(dev, B43_MMIO_XMITSTAT_3);
	if (!(t[0] & B43_TXST40_S1_VALID)) {
		b43dbg(dev->wl, "TX status second package not valid "
		       "(0x%08X), first package 0x%08X dropped\n", t[0], s[0]);
		return false;
	}
	return true;
}

static void handle_irq_transmit_status(struct b43_wldev *dev)
{
	u32 v0, v1;
	u16 tmp;
	struct b43_txstatus stat;

	if (b43_txstatus_is_rev40(dev)) {
		u32 s[B43_TXST40_WORDS], t[B43_TXST40_WORDS];
		/* QUIET: no per-status dump checks or log atomics */
		bool quiet = b43_ac_fastpath(dev, B43_AC_FASTPATH_QUIET);

		dev->ac_txs_kick = false;
		while (b43_read_txstatus_rev40(dev, s, t)) {
			b43_txstatus_parse_rev40(dev, &stat, s, t);
			if (quiet) {
				b43_handle_txstatus(dev, &stat);
				continue;
			}
			b43_handle_txstatus(dev, &stat);
		}
		/*
		 * TXBATCH: b43_dma_handle_txstatus kicked the TX worker
		 * after every status (a kworker wake-up, the mutex and four
		 * empty queues when nothing waits). With the bit it only notes
		 * when a status cleared tx_queue_stopped, the one case where a
		 * frame sits in the software queue without a kick of its own
		 * (b43_op_tx kicks unless the queue is stopped), and the worker
		 * runs once per interrupt.
		 */
		if (dev->ac_txs_kick)
			ieee80211_queue_work(dev->wl->hw, &dev->wl->tx_work);
		return;
	}

	while (1) {
		v0 = b43_read32(dev, B43_MMIO_XMITSTAT_0);
		if (!(v0 & 0x00000001))
			break;
		v1 = b43_read32(dev, B43_MMIO_XMITSTAT_1);

		stat.cookie = (v0 >> 16);
		stat.seq = (v1 & 0x0000FFFF);
		stat.phy_stat = ((v1 & 0x00FF0000) >> 16);
		tmp = (v0 & 0x0000FFFF);
		stat.frame_count = ((tmp & 0xF000) >> 12);
		stat.rts_count = ((tmp & 0x0F00) >> 8);
		stat.supp_reason = ((tmp & 0x001C) >> 2);
		stat.pm_indicated = !!(tmp & 0x0080);
		stat.intermediate = !!(tmp & 0x0040);
		stat.for_ampdu = !!(tmp & 0x0020);
		stat.acked = !!(tmp & 0x0002);

		b43_handle_txstatus(dev, &stat);
	}
}

static void drain_txstatus_queue(struct b43_wldev *dev)
{
	u32 dummy;

	if (dev->dev->core_rev < 5)
		return;
	/* Read all entries from the microcode TXstatus FIFO
	 * and throw them away.
	 */
	if (b43_txstatus_is_rev40(dev)) {
		u32 s[B43_TXST40_WORDS], t[B43_TXST40_WORDS];

		while (b43_read_txstatus_rev40(dev, s, t))
			;
		return;
	}
	while (1) {
		dummy = b43_read32(dev, B43_MMIO_XMITSTAT_0);
		if (!(dummy & 0x00000001))
			break;
		dummy = b43_read32(dev, B43_MMIO_XMITSTAT_1);
	}
}

static u32 b43_jssi_read(struct b43_wldev *dev)
{
	u32 val = 0;

	val = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI1);
	val <<= 16;
	val |= b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI0);

	return val;
}

static void b43_jssi_write(struct b43_wldev *dev, u32 jssi)
{
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI0,
			(jssi & 0x0000FFFF));
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_JSSI1,
			(jssi & 0xFFFF0000) >> 16);
}

static void b43_generate_noise_sample(struct b43_wldev *dev)
{
	b43_jssi_write(dev, 0x7F7F7F7F);
	b43_write32(dev, B43_MMIO_MACCMD,
		    b43_read32(dev, B43_MMIO_MACCMD) | B43_MACCMD_BGNOISE);
}

static void b43_calculate_link_quality(struct b43_wldev *dev)
{
	/* Top half of Link Quality calculation. */

	if (dev->phy.type != B43_PHYTYPE_G)
		return;
	if (dev->noisecalc.calculation_running)
		return;
	dev->noisecalc.calculation_running = true;
	dev->noisecalc.nr_samples = 0;

	b43_generate_noise_sample(dev);
}

static void handle_irq_noise(struct b43_wldev *dev)
{
	struct b43_phy_g *phy = dev->phy.g;
	u16 tmp;
	u8 noise[4];
	u8 i, j;
	s32 average;

	/* Bottom half of Link Quality calculation. */

	if (dev->phy.type != B43_PHYTYPE_G)
		return;

	/* Possible race condition: It might be possible that the user
	 * changed to a different channel in the meantime since we
	 * started the calculation. We ignore that fact, since it's
	 * not really that much of a problem. The background noise is
	 * an estimation only anyway. Slightly wrong results will get damped
	 * by the averaging of the 8 sample rounds. Additionally the
	 * value is shortlived. So it will be replaced by the next noise
	 * calculation round soon. */

	B43_WARN_ON(!dev->noisecalc.calculation_running);
	*((__le32 *)noise) = cpu_to_le32(b43_jssi_read(dev));
	if (noise[0] == 0x7F || noise[1] == 0x7F ||
	    noise[2] == 0x7F || noise[3] == 0x7F)
		goto generate_new;

	/* Get the noise samples. */
	B43_WARN_ON(dev->noisecalc.nr_samples >= 8);
	i = dev->noisecalc.nr_samples;
	noise[0] = clamp_val(noise[0], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	noise[1] = clamp_val(noise[1], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	noise[2] = clamp_val(noise[2], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	noise[3] = clamp_val(noise[3], 0, ARRAY_SIZE(phy->nrssi_lt) - 1);
	dev->noisecalc.samples[i][0] = phy->nrssi_lt[noise[0]];
	dev->noisecalc.samples[i][1] = phy->nrssi_lt[noise[1]];
	dev->noisecalc.samples[i][2] = phy->nrssi_lt[noise[2]];
	dev->noisecalc.samples[i][3] = phy->nrssi_lt[noise[3]];
	dev->noisecalc.nr_samples++;
	if (dev->noisecalc.nr_samples == 8) {
		/* Calculate the Link Quality by the noise samples. */
		average = 0;
		for (i = 0; i < 8; i++) {
			for (j = 0; j < 4; j++)
				average += dev->noisecalc.samples[i][j];
		}
		average /= (8 * 4);
		average *= 125;
		average += 64;
		average /= 128;
		tmp = b43_shm_read16(dev, B43_SHM_SHARED, 0x40C);
		tmp = (tmp / 128) & 0x1F;
		if (tmp >= 8)
			average += 2;
		else
			average -= 25;
		if (tmp == 8)
			average -= 72;
		else
			average -= 48;

		dev->stats.link_noise = average;
		dev->noisecalc.calculation_running = false;
		return;
	}
generate_new:
	b43_generate_noise_sample(dev);
}

static void handle_irq_tbtt_indication(struct b43_wldev *dev)
{
	if (b43_is_mode(dev->wl, NL80211_IFTYPE_AP)) {
		///TODO: PS TBTT
	} else {
		if (1 /*FIXME: the last PSpoll frame was sent successfully */ )
			b43_power_saving_ctl_bits(dev, 0);
	}
	if (b43_is_mode(dev->wl, NL80211_IFTYPE_ADHOC))
		dev->dfq_valid = true;
}

static void handle_irq_atim_end(struct b43_wldev *dev)
{
	if (dev->dfq_valid) {
		b43_write32(dev, B43_MMIO_MACCMD,
			    b43_read32(dev, B43_MMIO_MACCMD)
			    | B43_MACCMD_DFQ_VALID);
		dev->dfq_valid = false;
	}
}

/*
 * acphy_psfix bit 2, the hardware power management queue (PMQ) of
 * core rev >= 40. ucode 907 looks every received frame's A2 up in the PMQ
 * (@0x0A34..0x0A38) and, for a data or management frame to our address
 * from a station past authentication (@0x0C2D..0x0C30 skip assoc/auth),
 * queues (A2, PM bit) when the bit differs from the queued state
 * (@0x0C31..0x0C49), raising B43_IRQ_PMQ once more than shm 0x2A entries
 * wait (@0x0C4A..0x0C4C; b43 zeroes shm, so on the first). Entries are
 * only queued while MACCTL DISCPMQ is clear when the PSM (re)starts:
 * @0x0DE3..0x0DE5 derives the queue command word [0x16] from it, so
 * b43_adjust_opmode cycles the MAC after flipping the bit. A deauth or
 * disassoc to our address queues a delete (L845 @0x0F20). wl.ko drains
 * the queue in wlc_bmac_processpmq 0x12180: while 0x140 bit 3 is set it
 * reads 0x144 (A2 bytes 0..3), 0x148 (bytes 4..5), takes PM on/off from
 * 0x140 bits 17/16, skips group addresses (0x121f8) and calls
 * wlc_apps_process_ps_switch 0x11fa0; wlc_bmac_process_ps_switch 0x1210c
 * then writes 2 (delete up to the read pointer) to 0x140. While an entry
 * with PM on waits, the ucode TX path suppresses unicast frames to that
 * station with reason 1 (@0x03D1..0x03DC -> L247 @0x0409) unless the
 * header's MAC control bit 9 is set; xmit.c reports those as
 * IEEE80211_TX_STAT_TX_FILTERED. With bit 4 the entries drive
 * ieee80211_sta_ps_transition (AP_LINK_PS).
 */
#define B43_AC_PMQ_MAX	64

/*
 * acphy_psfix bit 16, the dozing stations as the PMQ entries report
 * them, so the processed entries can stay in the queue (the ucode's
 * lookups find them) until nobody dozes. Index 0 is unused.
 */
#define B43_AC_PMQ_STAS	62
static struct {
	u8 mac[ETH_ALEN];
	bool used;
	bool dozing;
} b43_ac_pmq_sta[B43_AC_PMQ_STAS];

/* Returns the number of dozing stations after the update */
static unsigned int b43_ac_pmq_sta_update(const u8 *mac, bool dozing,
					  bool gone)
{
	unsigned int i, free = B43_AC_PMQ_STAS, n = 0;

	for (i = 0; i < B43_AC_PMQ_STAS; i++) {
		if (!b43_ac_pmq_sta[i].used) {
			if (free == B43_AC_PMQ_STAS)
				free = i;
			continue;
		}
		if (ether_addr_equal(b43_ac_pmq_sta[i].mac, mac)) {
			b43_ac_pmq_sta[i].dozing = dozing;
			b43_ac_pmq_sta[i].used = !gone;
			free = i;
		}
		if (b43_ac_pmq_sta[i].used && b43_ac_pmq_sta[i].dozing)
			n++;
	}
	if (!gone && free < B43_AC_PMQ_STAS && !b43_ac_pmq_sta[free].used) {
		memcpy(b43_ac_pmq_sta[free].mac, mac, ETH_ALEN);
		b43_ac_pmq_sta[free].used = true;
		b43_ac_pmq_sta[free].dozing = dozing;
		if (dozing)
			n++;
	}
	return n;
}

static void b43_ac_pmq_drain(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	struct ieee80211_sta *sta;
	unsigned int n, dozing = 0;
	bool hold = b43_ac_psfix(dev, B43_AC_PSFIX_HOLD), flush = false;
	u32 reg, lo, hi;
	u8 mac[ETH_ALEN];

	for (n = 0; n < B43_AC_PMQ_MAX; n++) {
		reg = b43_read32(dev, B43_MMIO_PS_STATUS);
		if (!(reg & B43_AC_PMQ_NOT_EMPTY))
			break;
		lo = b43_read32(dev, B43_MMIO_AC_PMQ_PATL);
		hi = b43_read32(dev, B43_MMIO_AC_PMQ_PATH);
		mac[0] = lo;
		mac[1] = lo >> 8;
		mac[2] = lo >> 16;
		mac[3] = lo >> 24;
		mac[4] = hi;
		mac[5] = hi >> 8;
		if (reg & (B43_AC_PMQ_OFLO | B43_AC_PMQ_DASAT))
			flush = true;
		if (mac[0] & 1)
			continue;
		if (reg & B43_AC_PMQ_DASAT)
			dozing = b43_ac_pmq_sta_update(mac, false, true);
		if (!(reg & (B43_AC_PMQ_PMON | B43_AC_PMQ_PMOFF)))
			continue;
		dozing = b43_ac_pmq_sta_update(mac, !!(reg & B43_AC_PMQ_PMON),
					       false);
		if (!b43_ac_psfix(dev, B43_AC_PSFIX_LINKPS) || !wl->vif)
			continue;
		/*
		 * With AP_LINK_PS set, mac80211 does not track the PM bit itself:
		 * the driver must report every power-save transition, or it never
		 * buffers for a dozing station and keeps transmitting to it (every
		 * frame fails its retries). The PMQ gives us exactly that edge.
		 */
		rcu_read_lock();
		sta = ieee80211_find_sta(wl->vif, mac);
		if (sta)
			ieee80211_sta_ps_transition(sta,
						    !!(reg & B43_AC_PMQ_PMON));
		rcu_read_unlock();
	}
	if (n == B43_AC_PMQ_MAX) {
		flush = true;
	}
	if (!n)
		return;
	if (hold && dozing && !flush) {
		return;
	}
	b43_write16(dev, B43_MMIO_PS_STATUS, B43_AC_PMQ_DEL_MULT);
}

static void handle_irq_pmq(struct b43_wldev *dev)
{
	u32 tmp;

	if (b43_ac_psfix(dev, B43_AC_PSFIX_PMQ)) {
		b43_ac_pmq_drain(dev);
		return;
	}

	//TODO: AP mode.

	while (1) {
		tmp = b43_read32(dev, B43_MMIO_PS_STATUS);
		if (!(tmp & 0x00000008))
			break;
	}
	/* 16bit write is odd, but correct. */
	b43_write16(dev, B43_MMIO_PS_STATUS, 0x0002);
}

static void b43_write_template_common(struct b43_wldev *dev,
				      const u8 *data, u16 size,
				      u16 ram_offset,
				      u16 shm_size_offset, u8 rate)
{
	u32 i, tmp;
	struct b43_plcp_hdr4 plcp;

	plcp.data = 0;
	b43_generate_plcp_hdr(&plcp, size + FCS_LEN, rate);
	b43_ram_write(dev, ram_offset, le32_to_cpu(plcp.data));
	ram_offset += sizeof(u32);
	/* The PLCP is 6 bytes long, but we only wrote 4 bytes, yet.
	 * So leave the first two bytes of the next write blank.
	 */
	tmp = (u32) (data[0]) << 16;
	tmp |= (u32) (data[1]) << 24;
	b43_ram_write(dev, ram_offset, tmp);
	ram_offset += sizeof(u32);
	for (i = 2; i < size; i += sizeof(u32)) {
		tmp = (u32) (data[i + 0]);
		if (i + 1 < size)
			tmp |= (u32) (data[i + 1]) << 8;
		if (i + 2 < size)
			tmp |= (u32) (data[i + 2]) << 16;
		if (i + 3 < size)
			tmp |= (u32) (data[i + 3]) << 24;
		b43_ram_write(dev, ram_offset + i - 2, tmp);
	}
	b43_shm_write16(dev, B43_SHM_SHARED, shm_size_offset,
			size + sizeof(struct b43_plcp_hdr6));
}

/*
 * Core rev >= 40 beacon template: a 12 byte PHY header area, then the frame.
 * wlc_bcn_prb_template @0x1256b4 zeroes pub+0xbc = 12 bytes in front of the
 * 802.11 header (0x125714..0x125750), and for a beacon on core rev > 39 puts
 * the PLCP at byte 3 for OFDM and HT, 6 for CCK, 0 for VHT (0x1257c0..0x125808),
 * built by wlc_compute_plcp @0x118be8 for frame length + FCS (bodylen + 28,
 * 0x125814). wlc_bmac_write_template_ram @0x15cc60 sets MMIO 0x130 once and
 * streams 32 bit words into 0x134, rounded up to 4 bytes (0x15fa98..0x15faa0);
 * the length word is the exact byte count, prefix included (0x15fab8..0x15fac4).
 * b43 only generates legacy rates; its 4 byte CCK PLCP leaves bytes 4..5 of the
 * CCK PLCP zero, as b43's pre-rev-40 path does.
 */
static void b43_write_template_ac(struct b43_wldev *dev,
				  const u8 *data, u16 size,
				  u16 ram_offset,
				  u16 shm_size_offset, u8 rate)
{
	u8 phyhdr[B43_AC_TPL_PHYHDR] = { 0 };
	struct b43_plcp_hdr4 plcp;
	unsigned int i, k, j, total;
	u32 tmp;

	plcp.data = 0;
	b43_generate_plcp_hdr(&plcp, size + FCS_LEN, rate);
	memcpy(phyhdr + (b43_is_cck_rate(rate) ? B43_AC_TPL_PLCP_CCK :
					      B43_AC_TPL_PLCP_OFDM),
	       plcp.raw, sizeof(plcp.raw));

	total = B43_AC_TPL_PHYHDR + size;
	for (i = 0; i < total; i += sizeof(u32)) {
		tmp = 0;
		for (k = 0; k < sizeof(u32); k++) {
			j = i + k;
			if (j >= total)
				break;
			tmp |= (u32)(j < B43_AC_TPL_PHYHDR ? phyhdr[j] :
				     data[j - B43_AC_TPL_PHYHDR]) << (8 * k);
		}
		b43_ram_write(dev, ram_offset + i, tmp);
	}
	b43_shm_write16(dev, B43_SHM_SHARED, shm_size_offset, total);
}

/*
 * PHY control word 2 for a legacy rate: index in wl's cck_rates or ofdm_rates
 * (wlc_acphy_txctl2_calc @0x114a24, 0x114a60..0x114ac0).
 */
static u16 b43_ac_bcn_rate_idx(u8 rate)
{
	switch (rate) {
	case B43_CCK_RATE_1MB:
	case B43_OFDM_RATE_6MB:
		return 0;
	case B43_CCK_RATE_2MB:
	case B43_OFDM_RATE_9MB:
		return 1;
	case B43_CCK_RATE_5MB:
	case B43_OFDM_RATE_12MB:
		return 2;
	case B43_CCK_RATE_11MB:
	case B43_OFDM_RATE_18MB:
		return 3;
	case B43_OFDM_RATE_24MB:
		return 4;
	case B43_OFDM_RATE_36MB:
		return 5;
	case B43_OFDM_RATE_48MB:
		return 6;
	case B43_OFDM_RATE_54MB:
		return 7;
	}
	B43_WARN_ON(1);
	return 0;
}

/*
 * Beacon TSF offset (shm BTSFOFF 0x1c), wlc_compute_bcntsfoff @0x11ee64 as
 * wlc_write_hw_bcnparams calls it (long preamble, PHY delay on; 0x11f1e0..
 * 0x11f20c): OFDM 22 us plus the airtime of SERVICE + 24 byte header (26
 * bytes, 4 us symbols, 0x114824..0x114838); CCK 195 us plus 24 bytes at the
 * CCK rate (0x114840..0x114890). b43 never wrote this word.
 */
static u16 b43_ac_bcn_tsfoff(u8 rate)
{
	if (b43_is_cck_rate(rate))
		return 195 + (24 * 16) / rate;
	return 22 + 4 * DIV_ROUND_UP(26 * 8, 2 * rate);
}

/* Check if the use of the antenna that ieee80211 told us to
 * use is possible. This will fall back to DEFAULT.
 * "antenna_nr" is the antenna identifier we got from ieee80211. */
u8 b43_ieee80211_antenna_sanitize(struct b43_wldev *dev,
				  u8 antenna_nr)
{
	u8 antenna_mask;

	if (antenna_nr == 0) {
		/* Zero means "use default antenna". That's always OK. */
		return 0;
	}

	/* Get the mask of available antennas. */
	if (dev->phy.gmode)
		antenna_mask = dev->dev->bus_sprom->ant_available_bg;
	else
		antenna_mask = dev->dev->bus_sprom->ant_available_a;

	if (!(antenna_mask & (1 << (antenna_nr - 1)))) {
		/* This antenna is not available. Fall back to default. */
		return 0;
	}

	return antenna_nr;
}

/* Convert a b43 antenna number value to the PHY TX control value. */
static u16 b43_antenna_to_phyctl(int antenna)
{
	switch (antenna) {
	case B43_ANTENNA0:
		return B43_TXH_PHY_ANT0;
	case B43_ANTENNA1:
		return B43_TXH_PHY_ANT1;
	case B43_ANTENNA2:
		return B43_TXH_PHY_ANT2;
	case B43_ANTENNA3:
		return B43_TXH_PHY_ANT3;
	case B43_ANTENNA_AUTO0:
	case B43_ANTENNA_AUTO1:
		return B43_TXH_PHY_ANT01AUTO;
	}
	B43_WARN_ON(1);
	return 0;
}

static void b43_write_beacon_template(struct b43_wldev *dev,
				      u16 ram_offset,
				      u16 shm_size_offset)
{
	unsigned int i, len, variable_len;
	const struct ieee80211_mgmt *bcn;
	const u8 *ie;
	bool tim_found = false;
	unsigned int rate;
	u16 ctl;
	int antenna;
	struct ieee80211_tx_info *info;
	unsigned long flags;
	struct sk_buff *beacon_skb;
	bool ac = dev->dev->core_rev >= 40;
	unsigned int phyhdr_len;

	spin_lock_irqsave(&dev->wl->beacon_lock, flags);
	info = IEEE80211_SKB_CB(dev->wl->current_beacon);
	rate = ieee80211_get_tx_rate(dev->wl->hw, info)->hw_value;
	/* Clone the beacon, so it cannot go away, while we write it to hw. */
	beacon_skb = skb_clone(dev->wl->current_beacon, GFP_ATOMIC);
	spin_unlock_irqrestore(&dev->wl->beacon_lock, flags);

	if (!beacon_skb) {
		b43dbg(dev->wl, "Could not upload beacon. "
		       "Failed to clone beacon skb.");
		return;
	}

	bcn = (const struct ieee80211_mgmt *)(beacon_skb->data);
	if (ac) {
		len = min_t(size_t, beacon_skb->len,
			    B43_AC_TPL_SIZE - B43_AC_TPL_PHYHDR);
		b43_write_template_ac(dev, (const u8 *)bcn,
				      len, ram_offset, shm_size_offset, rate);
		phyhdr_len = B43_AC_TPL_PHYHDR;
	} else {
		len = min_t(size_t, beacon_skb->len,
			    0x200 - sizeof(struct b43_plcp_hdr6));
		b43_write_template_common(dev, (const u8 *)bcn,
					  len, ram_offset, shm_size_offset,
					  rate);
		phyhdr_len = sizeof(struct b43_plcp_hdr6);
	}

	if (ac) {
		/*
		 * wlc_beacon_phytxctl @0x118f84, core rev > 39 branch: the rate
		 * forced to 20 MHz (0x118fa0..0x118fa8), word 0 =
		 * wlc_acphy_txctl0_calc(rspec, preamble 0): frame type, bit 2,
		 * core mask << 6 from wlc_stf_d11hdrs_phyctl_txant, bandwidth 0
		 * (0x1135a0..0x113664); wlc_beacon_phytxctl_txant_upd @0x11641c
		 * rewrites bits 6..13 of 0xcc with the same core mask
		 * (0x116444, 0x116478..0x1164b0). Word 1 =
		 * wlc_acphy_txctl1_calc(chanspec, pwr, the 20 MHz rspec): power
		 * backoff << 3 | control sideband (0x118fc4..0x118ff4,
		 * 0x114254..0x114268): 0 at full power on a 20 MHz chanspec, the
		 * sideband on a 40 MHz one. Word 2 = the legacy rate index
		 * (0x118ff8..0x119008). BEACPHYCTL (0x54) and EXTNPHYCTL (0xb0)
		 * are not written on this branch.
		 */
		ctl = b43_is_cck_rate(rate) ? B43_TXH_AC_PHY0_FT_CCK :
					      B43_TXH_AC_PHY0_FT_OFDM;
		ctl |= B43_TXH_AC_PHY0_SET;
		ctl |= (b43_ac_txcore(dev, 1) << B43_TXH_AC_PHY0_ANT_SHIFT) &
		       B43_TXH_AC_PHY0_ANT;
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_AC_BCNPHYCTL0, ctl);
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_AC_BCNPHYCTL1,
				b43_ac_phyctl1_sb(b43_ac_chanspec(dev,
						dev->phy.channel), false));
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_AC_BCNPHYCTL2,
				b43_ac_bcn_rate_idx(rate));
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_BTSFOFF,
				b43_ac_bcn_tsfoff(rate));
		goto tim;
	}

	/* Write the PHY TX control parameters. */
	antenna = dev->tx_antenna;
	antenna = b43_antenna_to_phyctl(antenna);
	ctl = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_BEACPHYCTL);
	/* We can't send beacons with short preamble. Would get PHY errors. */
	ctl &= ~B43_TXH_PHY_SHORTPRMBL;
	ctl &= ~B43_TXH_PHY_ANT;
	ctl &= ~B43_TXH_PHY_ENC;
	ctl |= antenna;
	if (b43_is_cck_rate(rate))
		ctl |= B43_TXH_PHY_ENC_CCK;
	else
		ctl |= B43_TXH_PHY_ENC_OFDM;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_BEACPHYCTL, ctl);

tim:
	/* Find the position of the TIM and the DTIM_period value
	 * and write them to SHM. */
	ie = bcn->u.beacon.variable;
	variable_len = len - offsetof(struct ieee80211_mgmt, u.beacon.variable);
	for (i = 0; i < variable_len - 2; ) {
		uint8_t ie_id, ie_len;

		ie_id = ie[i];
		ie_len = ie[i + 1];
		if (ie_id == 5) {
			u16 tim_position;
			u16 dtim_period;
			/* This is the TIM Information Element */

			/* Check whether the ie_len is in the beacon data range. */
			if (variable_len < ie_len + 2 + i)
				break;
			/* A valid TIM is at least 4 bytes long. */
			if (ie_len < 4)
				break;
			tim_found = true;

			/* From the template start, PHY header area included
			 * (wlc_write_hw_bcnparams 0x11f238..0x11f270). */
			tim_position = phyhdr_len;
			tim_position += offsetof(struct ieee80211_mgmt, u.beacon.variable);
			tim_position += i;

			dtim_period = ie[i + 3];

			b43_shm_write16(dev, B43_SHM_SHARED,
					B43_SHM_SH_TIMBPOS, tim_position);
			b43_shm_write16(dev, B43_SHM_SHARED,
					B43_SHM_SH_DTIMPER, dtim_period);
			break;
		}
		i += ie_len + 2;
	}
	if (!tim_found) {
		/*
		 * If ucode wants to modify TIM do it behind the beacon, this
		 * will happen, for example, when doing mesh networking.
		 */
		b43_shm_write16(dev, B43_SHM_SHARED,
				B43_SHM_SH_TIMBPOS,
				len + phyhdr_len);
		b43_shm_write16(dev, B43_SHM_SHARED,
				B43_SHM_SH_DTIMPER, 0);
	}
	b43dbg(dev->wl, "Updated beacon template at 0x%x\n", ram_offset);

	dev_kfree_skb_any(beacon_skb);
}

static void b43_upload_beacon0(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;

	if (wl->beacon0_uploaded)
		return;
	b43_write_beacon_template(dev, dev->dev->core_rev >= 40 ?
				  B43_AC_TPL_BCN0 : B43_SHM_SH_BT_BASE0,
				  B43_SHM_SH_BTL0);
	wl->beacon0_uploaded = true;
}

static void b43_upload_beacon1(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;

	if (wl->beacon1_uploaded)
		return;
	b43_write_beacon_template(dev, dev->dev->core_rev >= 40 ?
				  B43_AC_TPL_BCN1 : B43_SHM_SH_BT_BASE1,
				  B43_SHM_SH_BTL1);
	wl->beacon1_uploaded = true;
}

static void handle_irq_beacon(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	u32 cmd, beacon0_valid, beacon1_valid;

	if (!b43_is_mode(wl, NL80211_IFTYPE_AP) &&
	    !b43_is_mode(wl, NL80211_IFTYPE_MESH_POINT) &&
	    !b43_is_mode(wl, NL80211_IFTYPE_ADHOC))
		return;

	/* This is the bottom half of the asynchronous beacon update. */

	/* Ignore interrupt in the future. */
	dev->irq_mask &= ~B43_IRQ_BEACON;

	cmd = b43_read32(dev, B43_MMIO_MACCMD);
	beacon0_valid = (cmd & B43_MACCMD_BEACON0_VALID);
	beacon1_valid = (cmd & B43_MACCMD_BEACON1_VALID);

	/*
	 * acphy_psfix bit 1. ucode 907 sends from the template its
	 * r20 bit 9 names (@0x0518: 0x202 / 0x482), and when both valid bits
	 * are set (@0x0109) writes those bits back to MACCMD (@0x01C1) and
	 * raises IRQ 0x2 (@0x01C2). MACCMD is write-1-to-clear for the ucode
	 * (@0x0110, @0x1018, @0x123E, @0x1254 acknowledge host commands that
	 * way), so the bit of the template just sent clears and the read-back
	 * (@0x01C3..0x01C4) switches r20 to the other one: the alternation
	 * b43 expects. A TIM change (set_tim) lands in the next free template
	 * and goes on air one beacon later. This line shows which bit the
	 * ucode cleared; "both valid" only ever means the IRQ is re-armed.
	 */

	/* Schedule interrupt manually, if busy. */
	if (beacon0_valid && beacon1_valid) {
		b43_write32(dev, B43_MMIO_GEN_IRQ_REASON, B43_IRQ_BEACON);
		dev->irq_mask |= B43_IRQ_BEACON;
		return;
	}

	if (unlikely(wl->beacon_templates_virgin)) {
		/* We never uploaded a beacon before.
		 * Upload both templates now, but only mark one valid. */
		wl->beacon_templates_virgin = false;
		b43_upload_beacon0(dev);
		b43_upload_beacon1(dev);
		cmd = b43_read32(dev, B43_MMIO_MACCMD);
		cmd |= B43_MACCMD_BEACON0_VALID;
		b43_write32(dev, B43_MMIO_MACCMD, cmd);
	} else {
		if (!beacon0_valid) {
			b43_upload_beacon0(dev);
			cmd = b43_read32(dev, B43_MMIO_MACCMD);
			cmd |= B43_MACCMD_BEACON0_VALID;
			b43_write32(dev, B43_MMIO_MACCMD, cmd);
		} else if (!beacon1_valid) {
			b43_upload_beacon1(dev);
			cmd = b43_read32(dev, B43_MMIO_MACCMD);
			cmd |= B43_MACCMD_BEACON1_VALID;
			b43_write32(dev, B43_MMIO_MACCMD, cmd);
		}
	}
}

static void b43_do_beacon_update_trigger_work(struct b43_wldev *dev)
{
	u32 old_irq_mask = dev->irq_mask;

	/* update beacon right away or defer to irq */
	handle_irq_beacon(dev);
	if (old_irq_mask != dev->irq_mask) {
		/* The handler updated the IRQ mask. */
		B43_WARN_ON(!dev->irq_mask);
		if (b43_read32(dev, B43_MMIO_GEN_IRQ_MASK)) {
			b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, dev->irq_mask);
		} else {
			/* Device interrupts are currently disabled. That means
			 * we just ran the hardirq handler and scheduled the
			 * IRQ thread. The thread will write the IRQ mask when
			 * it finished, so there's nothing to do here. Writing
			 * the mask _here_ would incorrectly re-enable IRQs. */
		}
	}
}

static void b43_beacon_update_trigger_work(struct work_struct *work)
{
	struct b43_wl *wl = container_of(work, struct b43_wl,
					 beacon_update_trigger);
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (likely(dev && (b43_status(dev) >= B43_STAT_INITIALIZED))) {
		if (b43_bus_host_is_sdio(dev->dev)) {
			/* wl->mutex is enough. */
			b43_do_beacon_update_trigger_work(dev);
		} else {
			spin_lock_irq(&wl->hardirq_lock);
			b43_do_beacon_update_trigger_work(dev);
			spin_unlock_irq(&wl->hardirq_lock);
		}
	}
	mutex_unlock(&wl->mutex);
}

/* Asynchronously update the packet templates in template RAM. */
static void b43_update_templates(struct b43_wl *wl)
{
	struct sk_buff *beacon, *old_beacon;
	unsigned long flags;

	/* This is the top half of the asynchronous beacon update.
	 * The bottom half is the beacon IRQ.
	 * Beacon update must be asynchronous to avoid sending an
	 * invalid beacon. This can happen for example, if the firmware
	 * transmits a beacon while we are updating it. */

	/* We could modify the existing beacon and set the aid bit in
	 * the TIM field, but that would probably require resizing and
	 * moving of data within the beacon template.
	 * Simply request a new beacon and let mac80211 do the hard work. */
	beacon = ieee80211_beacon_get(wl->hw, wl->vif, 0);
	if (unlikely(!beacon))
		return;

	spin_lock_irqsave(&wl->beacon_lock, flags);
	old_beacon = wl->current_beacon;
	wl->current_beacon = beacon;
	wl->beacon0_uploaded = false;
	wl->beacon1_uploaded = false;
	spin_unlock_irqrestore(&wl->beacon_lock, flags);

	ieee80211_queue_work(wl->hw, &wl->beacon_update_trigger);

	if (old_beacon)
		dev_kfree_skb_any(old_beacon);
}

static void b43_set_beacon_int(struct b43_wldev *dev, u16 beacon_int)
{
	b43_time_lock(dev);
	if (dev->dev->core_rev >= 3) {
		b43_write32(dev, B43_MMIO_TSF_CFP_REP, (beacon_int << 16));
		b43_write32(dev, B43_MMIO_TSF_CFP_START, (beacon_int << 10));
	} else {
		b43_write16(dev, 0x606, (beacon_int >> 6));
		b43_write16(dev, 0x610, beacon_int);
	}
	b43_time_unlock(dev);
	b43dbg(dev->wl, "Set beacon interval to %u\n", beacon_int);
}

static void b43_handle_firmware_panic(struct b43_wldev *dev)
{
	u16 reason;

	/* Read the register that contains the reason code for the panic. */
	reason = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_FWPANIC_REASON_REG);
	b43err(dev->wl, "Whoopsy, firmware panic! Reason: %u\n", reason);

	switch (reason) {
	default:
		b43dbg(dev->wl, "The panic reason is unknown.\n");
		fallthrough;
	case B43_FWPANIC_DIE:
		/* Do not restart the controller or firmware.
		 * The device is nonfunctional from now on.
		 * Restarting would result in this panic to trigger again,
		 * so we avoid that recursion. */
		break;
	case B43_FWPANIC_RESTART:
		b43_controller_restart(dev, "Microcode panic");
		break;
	}
}

static void handle_irq_ucode_debug(struct b43_wldev *dev)
{
	unsigned int i, cnt;
	u16 reason, marker_id, marker_line;
	__le16 *buf;

	/* The proprietary firmware doesn't have this IRQ. */
	if (!dev->fw.opensource)
		return;

	/* Read the register that contains the reason code for this IRQ. */
	reason = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_DEBUGIRQ_REASON_REG);

	switch (reason) {
	case B43_DEBUGIRQ_PANIC:
		b43_handle_firmware_panic(dev);
		break;
	case B43_DEBUGIRQ_DUMP_SHM:
		if (!B43_DEBUG)
			break; /* Only with driver debugging enabled. */
		buf = kmalloc(4096, GFP_ATOMIC);
		if (!buf) {
			b43dbg(dev->wl, "SHM-dump: Failed to allocate memory\n");
			goto out;
		}
		for (i = 0; i < 4096; i += 2) {
			u16 tmp = b43_shm_read16(dev, B43_SHM_SHARED, i);
			buf[i / 2] = cpu_to_le16(tmp);
		}
		b43info(dev->wl, "Shared memory dump:\n");
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET,
			       16, 2, buf, 4096, 1);
		kfree(buf);
		break;
	case B43_DEBUGIRQ_DUMP_REGS:
		if (!B43_DEBUG)
			break; /* Only with driver debugging enabled. */
		b43info(dev->wl, "Microcode register dump:\n");
		for (i = 0, cnt = 0; i < 64; i++) {
			u16 tmp = b43_shm_read16(dev, B43_SHM_SCRATCH, i);
			if (cnt == 0)
				printk(KERN_INFO);
			printk("r%02u: 0x%04X  ", i, tmp);
			cnt++;
			if (cnt == 6) {
				printk("\n");
				cnt = 0;
			}
		}
		printk("\n");
		break;
	case B43_DEBUGIRQ_MARKER:
		if (!B43_DEBUG)
			break; /* Only with driver debugging enabled. */
		marker_id = b43_shm_read16(dev, B43_SHM_SCRATCH,
					   B43_MARKER_ID_REG);
		marker_line = b43_shm_read16(dev, B43_SHM_SCRATCH,
					     B43_MARKER_LINE_REG);
		b43info(dev->wl, "The firmware just executed the MARKER(%u) "
			"at line number %u\n",
			marker_id, marker_line);
		break;
	default:
		b43dbg(dev->wl, "Debug-IRQ triggered for unknown reason: %u\n",
		       reason);
	}
out:
	/* Acknowledge the debug-IRQ, so the firmware can continue. */
	b43_shm_write16(dev, B43_SHM_SCRATCH,
			B43_DEBUGIRQ_REASON_REG, B43_DEBUGIRQ_ACK);
}

/*
 * Core rev >= 40: ucode 907 raises MACINTSTATUS bit 13 (b43's TIMER0, wl's
 * GP0) on a fatal error. wl's default MAC interrupt mask 0xb0e7a860
 * (wlc_bmac_info_init literal 0x15c484, wlc_bmac_attach 0x163c08) includes
 * it; wlc_dpc then calls wlc_dump_ucode_fatal and reinitialises the driver
 * (wl_init) (0x12db0..0x12dd8). b43's mask lacks it, so a ucode crash
 * was silent. Restart the controller as wl does, at most
 * B43_UCODE_FATAL_RESTARTS times.
 */
#define B43_IRQ_UCODE_FATAL_REV40	B43_IRQ_TIMER0
#define B43_UCODE_FATAL_RESTARTS	3
static atomic_t b43_ucode_fatal_count = ATOMIC_INIT(0);

/*
 * acphy_gp0 bit 1 logs the fatal and goes on without the controller
 * restart (2.4 s of silence: every laptop beacon loss of 2026-10-08
 * followed one), to see whether ucode 907 keeps running after its watchdog
 * fired; the interrupt is acknowledged by the top half either way.
 */
static int b43_modparam_acphy_gp0;
module_param_named(acphy_gp0, b43_modparam_acphy_gp0, int, 0644);
MODULE_PARM_DESC(acphy_gp0,
		 "AC-PHY ucode fatal (MACINTSTATUS bit 13) on core rev >= 40: 1 = log only, no controller restart (default 0)");

/* true when a controller restart was queued (the caller then returns
 * without touching the IRQ mask); false past the limit, the caller goes on
 * and restores the mask, else the core stays silent until the next load. */
static bool b43_handle_ucode_fatal_rev40(struct b43_wldev *dev)
{
	unsigned int n;

	n = atomic_inc_return(&b43_ucode_fatal_count);
	if (b43_modparam_acphy_gp0 & 1) {
		b43err(dev->wl, "ucode fatal %u, no controller restart (acphy_gp0)\n", n);
		return false;
	}
	if (n <= B43_UCODE_FATAL_RESTARTS) {
		b43_controller_restart(dev, "ucode fatal");
		return true;
	}
	b43err(dev->wl, "ucode fatal %u times, not restarting\n", n);
	return false;
}

/*
 * The register part of the interrupt top half: read the DMA reasons for
 * `reason` (already masked with dev->irq_mask), acknowledge everything and
 * save it for the thread. Caller holds wl->hardirq_lock.
 */
static void b43_irq_ack_reasons(struct b43_wldev *dev, u32 reason)
{
	dev->dma_reason[0] = b43_read32(dev, B43_MMIO_DMA0_REASON)
	    & 0x0001FC00;
	dev->dma_reason[1] = b43_read32(dev, B43_MMIO_DMA1_REASON)
	    & 0x0000DC00;
	dev->dma_reason[2] = b43_read32(dev, B43_MMIO_DMA2_REASON)
	    & 0x0000DC00;
	dev->dma_reason[3] = b43_read32(dev, B43_MMIO_DMA3_REASON)
	    & 0x0001DC00;
	dev->dma_reason[4] = b43_read32(dev, B43_MMIO_DMA4_REASON)
	    & 0x0000DC00;
/* Unused ring
	dev->dma_reason[5] = b43_read32(dev, B43_MMIO_DMA5_REASON)
	    & 0x0000DC00;
*/

	/* ACK the interrupt. */
	b43_write32(dev, B43_MMIO_GEN_IRQ_REASON, reason);
	b43_write32(dev, B43_MMIO_DMA0_REASON, dev->dma_reason[0]);
	b43_write32(dev, B43_MMIO_DMA1_REASON, dev->dma_reason[1]);
	b43_write32(dev, B43_MMIO_DMA2_REASON, dev->dma_reason[2]);
	b43_write32(dev, B43_MMIO_DMA3_REASON, dev->dma_reason[3]);
	b43_write32(dev, B43_MMIO_DMA4_REASON, dev->dma_reason[4]);
/* Unused ring
	b43_write32(dev, B43_MMIO_DMA5_REASON, dev->dma_reason[5]);
*/

	/* Save the reason bitmasks for the IRQ thread handler. */
	dev->irq_reason = reason;
}

/*
 * IRQPOLL: at the end of a thread pass, with the device still masked,
 * look for reasons that arrived while the pass ran. True (and the reasons
 * read and acknowledged as the top half would) when there are any, so the
 * thread handles them in this pass instead of after a hard interrupt and a
 * wake-up. Under hardirq_lock, the lock of the top half (a shared line
 * could run it meanwhile; whichever side takes the reasons handles them).
 */
static bool b43_ac_irq_repoll(struct b43_wldev *dev)
{
	unsigned long flags;
	u32 reason;
	bool more;

	spin_lock_irqsave(&dev->wl->hardirq_lock, flags);
	reason = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
	more = reason != 0xffffffff && (reason & dev->irq_mask);
	if (more)
		b43_irq_ack_reasons(dev, reason & dev->irq_mask);
	spin_unlock_irqrestore(&dev->wl->hardirq_lock, flags);
	return more;
}

static void b43_do_interrupt_thread(struct b43_wldev *dev)
{
	u32 reason;
	u32 dma_reason[ARRAY_SIZE(dev->dma_reason)];
	u32 merged_dma_reason = 0;
	unsigned int round = 0;
	int i;

again:
	if (unlikely(b43_status(dev) != B43_STAT_STARTED))
		return;

	reason = dev->irq_reason;
	merged_dma_reason = 0;
	for (i = 0; i < ARRAY_SIZE(dma_reason); i++) {
		dma_reason[i] = dev->dma_reason[i];
		merged_dma_reason |= dma_reason[i];
	}

	if (unlikely(reason & B43_IRQ_MAC_TXERR))
		b43err(dev->wl, "MAC transmission error\n");

	if (unlikely(reason & B43_IRQ_PHY_TXERR) &&
	    dev->dev->core_rev >= 40 && dev->phy.type == B43_PHYTYPE_AC) {
		/*
		 * Core rev >= 40 / AC-PHY: wl only counts PHY TX errors
		 * (wlc_dpc 0x12c78..0x12c9c); its reinit on a threshold is for
		 * PHY type 6 only (0x12ca0..0x12cc8). Frames are retried and
		 * delivered, so count and log rate-limited instead of
		 * restarting the controller after B43_PHY_TX_BADNESS_LIMIT.
		 */
		dev->phy_txerr_total++;
		if (b43_ratelimit(dev->wl))
			b43dbg(dev->wl, "PHY transmission error (%u so far)\n",
			       dev->phy_txerr_total);
		reason &= ~B43_IRQ_PHY_TXERR;
	}
	if (unlikely(reason & B43_IRQ_PHY_TXERR)) {
		b43err(dev->wl, "PHY transmission error\n");
		rmb();
		if (unlikely(atomic_dec_and_test(&dev->phy.txerr_cnt))) {
			atomic_set(&dev->phy.txerr_cnt,
				   B43_PHY_TX_BADNESS_LIMIT);
			b43err(dev->wl, "Too many PHY TX errors, "
					"restarting the controller\n");
			b43_controller_restart(dev, "PHY TX errors");
		}
	}

	if (unlikely(merged_dma_reason & (B43_DMAIRQ_FATALMASK))) {
		b43err(dev->wl,
			"Fatal DMA error: 0x%08X, 0x%08X, 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
			dma_reason[0], dma_reason[1],
			dma_reason[2], dma_reason[3],
			dma_reason[4], dma_reason[5]);
#ifdef CPTCFG_B43_PIO
		b43err(dev->wl, "This device does not support DMA "
			       "on your system. It will now be switched to PIO.\n");
		/* Fall back to PIO transfers if we get fatal DMA errors! */
		dev->use_pio = true;
#endif
		b43_controller_restart(dev, "DMA error");
		return;
	}

	if (unlikely(dev->dev->core_rev >= 40 &&
		     (reason & B43_IRQ_UCODE_FATAL_REV40))) {
		if (b43_handle_ucode_fatal_rev40(dev))
			return;	/* the restart re-enables interrupts */
		/* past the restart limit: keep serving interrupts */
	}
	if (unlikely(reason & B43_IRQ_UCODE_DEBUG))
		handle_irq_ucode_debug(dev);
	if (reason & B43_IRQ_TBTT_INDI)
		handle_irq_tbtt_indication(dev);
	if (reason & B43_IRQ_ATIM_END)
		handle_irq_atim_end(dev);
	if (reason & B43_IRQ_BEACON)
		handle_irq_beacon(dev);
	if (reason & B43_IRQ_PMQ)
		handle_irq_pmq(dev);
	if (reason & B43_IRQ_TXFIFO_FLUSH_OK) {
		;/* TODO */
	}
	if (reason & B43_IRQ_NOISESAMPLE_OK)
		handle_irq_noise(dev);

	/* Check the DMA reason registers for received data. */
	if (dma_reason[0] & B43_DMAIRQ_RDESC_UFLOW) {
		if (B43_DEBUG)
			b43warn(dev->wl, "RX descriptor underrun\n");
		b43_dma_handle_rx_overflow(dev->dma.rx_ring);
	}
	if (dma_reason[0] & B43_DMAIRQ_RX_DONE) {
		if (b43_using_pio_transfers(dev))
			b43_pio_rx(dev->pio.rx_queue);
		else
			b43_dma_rx(dev->dma.rx_ring);
	}
	B43_WARN_ON(dma_reason[1] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[2] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[3] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[4] & B43_DMAIRQ_RX_DONE);
	B43_WARN_ON(dma_reason[5] & B43_DMAIRQ_RX_DONE);

	if (reason & B43_IRQ_TX_OK)
		handle_irq_transmit_status(dev);

	/*
	 * wlc_dpc checks MACCTL after the DMA interrupt and logs when neither
	 * EN_MAC nor PSM_RUN is set (tst #3 at 0x12d34..0x12d64).
	 */
	if (dev->dev->core_rev >= 40 && (reason & B43_IRQ_DMA) &&
	    !b43_ac_fastpath(dev, B43_AC_FASTPATH_QUIET)) {
		static unsigned int psm_stopped_logged;
		u32 macctl = b43_read32(dev, B43_MMIO_MACCTL);

		if (!(macctl & 3) && psm_stopped_logged++ < 10)
			b43err(dev->wl, "PSM not running, MACCTL 0x%08x\n",
			       macctl);
	}

	/* IRQPOLL: another pass for what arrived meanwhile, bounded */
	if (b43_ac_fastpath(dev, B43_AC_FASTPATH_IRQPOLL) &&
	    ++round < B43_AC_IRQPOLL_ROUNDS && b43_ac_irq_repoll(dev))
		goto again;

	/* Re-enable interrupts on the device by restoring the current interrupt mask. */
	b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, dev->irq_mask);

#if B43_DEBUG
	if (b43_debug(dev, B43_DBG_VERBOSESTATS)) {
		dev->irq_count++;
		for (i = 0; i < ARRAY_SIZE(dev->irq_bit_count); i++) {
			if (reason & (1 << i))
				dev->irq_bit_count[i]++;
		}
	}
#endif
}

/* Interrupt thread handler. Handles device interrupts in thread context. */
static irqreturn_t b43_interrupt_thread_handler(int irq, void *dev_id)
{
	struct b43_wldev *dev = dev_id;

	mutex_lock(&dev->wl->mutex);
	b43_do_interrupt_thread(dev);
	mutex_unlock(&dev->wl->mutex);

	return IRQ_HANDLED;
}

static irqreturn_t b43_do_interrupt(struct b43_wldev *dev)
{
	u32 reason;

	/* This code runs under wl->hardirq_lock, but _only_ on non-SDIO busses.
	 * On SDIO, this runs under wl->mutex. */

	reason = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
	if (reason == 0xffffffff)	/* shared IRQ */
		return IRQ_NONE;
	reason &= dev->irq_mask;
	if (!reason)
		return IRQ_NONE;

	/* DMA reasons, the ACKs and the save for the thread (shared with the fast path) */
	b43_irq_ack_reasons(dev, reason);

	/* Disable IRQs on the device. The IRQ thread handler will re-enable them. */
	b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, 0);

	return IRQ_WAKE_THREAD;
}

/* Interrupt handler top-half. This runs with interrupts disabled. */
static irqreturn_t b43_interrupt_handler(int irq, void *dev_id)
{
	struct b43_wldev *dev = dev_id;
	irqreturn_t ret;

	if (unlikely(b43_status(dev) < B43_STAT_STARTED))
		return IRQ_NONE;

	spin_lock(&dev->wl->hardirq_lock);
	ret = b43_do_interrupt(dev);
	spin_unlock(&dev->wl->hardirq_lock);

	return ret;
}

/* SDIO interrupt handler. This runs in process context. */
static void b43_sdio_interrupt_handler(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	irqreturn_t ret;

	mutex_lock(&wl->mutex);

	ret = b43_do_interrupt(dev);
	if (ret == IRQ_WAKE_THREAD)
		b43_do_interrupt_thread(dev);

	mutex_unlock(&wl->mutex);
}

void b43_do_release_fw(struct b43_firmware_file *fw)
{
	release_firmware(fw->data);
	fw->data = NULL;
	fw->filename = NULL;
}

static void b43_release_firmware(struct b43_wldev *dev)
{
	complete(&dev->fw_load_complete);
	b43_do_release_fw(&dev->fw.ucode);
	b43_do_release_fw(&dev->fw.pcm);
	b43_do_release_fw(&dev->fw.initvals);
	b43_do_release_fw(&dev->fw.initvals_band);
}

static void b43_print_fw_helptext(struct b43_wl *wl, bool error)
{
	const char text[] =
		"You must go to " \
		"https://wireless.docs.kernel.org/en/latest/en/users/drivers/b43/developers.html#list-of-firmware " \
		"and download the correct firmware for this driver version. " \
		"Please carefully read all instructions on this website.\n";

	if (error)
		b43err(wl, text);
	else
		b43warn(wl, text);
}

static void b43_fw_cb(const struct firmware *firmware, void *context)
{
	struct b43_request_fw_context *ctx = context;

	ctx->blob = firmware;
	complete(&ctx->dev->fw_load_complete);
}

int b43_do_request_fw(struct b43_request_fw_context *ctx,
		      const char *name,
		      struct b43_firmware_file *fw, bool async)
{
	struct b43_fw_header *hdr;
	u32 size;
	int err;

	if (!name) {
		/* Don't fetch anything. Free possibly cached firmware. */
		/* FIXME: We should probably keep it anyway, to save some headache
		 * on suspend/resume with multiband devices. */
		b43_do_release_fw(fw);
		return 0;
	}
	if (fw->filename) {
		if ((fw->type == ctx->req_type) &&
		    (strcmp(fw->filename, name) == 0))
			return 0; /* Already have this fw. */
		/* Free the cached firmware first. */
		/* FIXME: We should probably do this later after we successfully
		 * got the new fw. This could reduce headache with multiband devices.
		 * We could also redesign this to cache the firmware for all possible
		 * bands all the time. */
		b43_do_release_fw(fw);
	}

	switch (ctx->req_type) {
	case B43_FWTYPE_PROPRIETARY:
		snprintf(ctx->fwname, sizeof(ctx->fwname),
			 "b43%s/%s.fw",
			 modparam_fwpostfix, name);
		break;
	case B43_FWTYPE_OPENSOURCE:
		snprintf(ctx->fwname, sizeof(ctx->fwname),
			 "b43-open%s/%s.fw",
			 modparam_fwpostfix, name);
		break;
	default:
		B43_WARN_ON(1);
		return -ENOSYS;
	}
	if (async) {
		/* do this part asynchronously */
		init_completion(&ctx->dev->fw_load_complete);
		err = request_firmware_nowait(THIS_MODULE, 1, ctx->fwname,
					      ctx->dev->dev->dev, GFP_KERNEL,
					      ctx, b43_fw_cb);
		if (err < 0) {
			pr_err("Unable to load firmware\n");
			return err;
		}
		wait_for_completion(&ctx->dev->fw_load_complete);
		if (ctx->blob)
			goto fw_ready;
	/* On some ARM systems, the async request will fail, but the next sync
	 * request works. For this reason, we fall through here
	 */
	}
	err = request_firmware(&ctx->blob, ctx->fwname,
			       ctx->dev->dev->dev);
	if (err == -ENOENT) {
		snprintf(ctx->errors[ctx->req_type],
			 sizeof(ctx->errors[ctx->req_type]),
			 "Firmware file \"%s\" not found\n",
			 ctx->fwname);
		return err;
	} else if (err) {
		snprintf(ctx->errors[ctx->req_type],
			 sizeof(ctx->errors[ctx->req_type]),
			 "Firmware file \"%s\" request failed (err=%d)\n",
			 ctx->fwname, err);
		return err;
	}
fw_ready:
	if (ctx->blob->size < sizeof(struct b43_fw_header))
		goto err_format;
	hdr = (struct b43_fw_header *)(ctx->blob->data);
	switch (hdr->type) {
	case B43_FW_TYPE_UCODE:
	case B43_FW_TYPE_PCM:
		size = be32_to_cpu(hdr->size);
		if (size != ctx->blob->size - sizeof(struct b43_fw_header))
			goto err_format;
		fallthrough;
	case B43_FW_TYPE_IV:
		if (hdr->ver != 1)
			goto err_format;
		break;
	default:
		goto err_format;
	}

	fw->data = ctx->blob;
	fw->filename = name;
	fw->type = ctx->req_type;

	return 0;

err_format:
	snprintf(ctx->errors[ctx->req_type],
		 sizeof(ctx->errors[ctx->req_type]),
		 "Firmware file \"%s\" format error.\n", ctx->fwname);
	release_firmware(ctx->blob);

	return -EPROTO;
}

/* https://bcm-v4.sipsolutions.net/802.11/Init/Firmware */
static int b43_try_request_fw(struct b43_request_fw_context *ctx)
{
	struct b43_wldev *dev = ctx->dev;
	struct b43_firmware *fw = &ctx->dev->fw;
	struct b43_phy *phy = &dev->phy;
	const u8 rev = ctx->dev->dev->core_rev;
	const char *filename;
	int err;

	/* Get microcode */
	filename = NULL;
	switch (rev) {
	case 56:
		if (phy->type == B43_PHYTYPE_AC)
			filename = "ucode56";
		break;
	case 42:
		if (phy->type == B43_PHYTYPE_AC)
			filename = "ucode42";
		break;
	case 40:
		if (phy->type == B43_PHYTYPE_AC)
			filename = "ucode40";
		break;
	case 33:
		if (phy->type == B43_PHYTYPE_LCN40)
			filename = "ucode33_lcn40";
		break;
	case 30:
		if (phy->type == B43_PHYTYPE_N)
			filename = "ucode30_mimo";
		break;
	case 29:
		if (phy->type == B43_PHYTYPE_HT)
			filename = "ucode29_mimo";
		break;
	case 26:
		if (phy->type == B43_PHYTYPE_HT)
			filename = "ucode26_mimo";
		break;
	case 28:
	case 25:
		if (phy->type == B43_PHYTYPE_N)
			filename = "ucode25_mimo";
		else if (phy->type == B43_PHYTYPE_LCN)
			filename = "ucode25_lcn";
		break;
	case 24:
		if (phy->type == B43_PHYTYPE_LCN)
			filename = "ucode24_lcn";
		break;
	case 23:
		if (phy->type == B43_PHYTYPE_N)
			filename = "ucode16_mimo";
		break;
	case 16 ... 19:
		if (phy->type == B43_PHYTYPE_N)
			filename = "ucode16_mimo";
		else if (phy->type == B43_PHYTYPE_LP)
			filename = "ucode16_lp";
		break;
	case 15:
		filename = "ucode15";
		break;
	case 14:
		filename = "ucode14";
		break;
	case 13:
		filename = "ucode13";
		break;
	case 11 ... 12:
		filename = "ucode11";
		break;
	case 5 ... 10:
		filename = "ucode5";
		break;
	}
	if (!filename)
		goto err_no_ucode;
	err = b43_do_request_fw(ctx, filename, &fw->ucode, true);
	if (err)
		goto err_load;

	/* Get PCM code */
	if ((rev >= 5) && (rev <= 10))
		filename = "pcm5";
	else if (rev >= 11)
		filename = NULL;
	else
		goto err_no_pcm;
	fw->pcm_request_failed = false;
	err = b43_do_request_fw(ctx, filename, &fw->pcm, false);
	if (err == -ENOENT) {
		/* We did not find a PCM file? Not fatal, but
		 * core rev <= 10 must do without hwcrypto then. */
		fw->pcm_request_failed = true;
	} else if (err)
		goto err_load;

	/* Get initvals */
	filename = NULL;
	switch (dev->phy.type) {
	case B43_PHYTYPE_G:
		if (rev == 13)
			filename = "b0g0initvals13";
		else if (rev >= 5 && rev <= 10)
			filename = "b0g0initvals5";
		break;
	case B43_PHYTYPE_N:
		if (rev == 30)
			filename = "n16initvals30";
		else if (rev == 28 || rev == 25)
			filename = "n0initvals25";
		else if (rev == 24)
			filename = "n0initvals24";
		else if (rev == 23)
			filename = "n0initvals16"; /* What about n0initvals22? */
		else if (rev >= 16 && rev <= 18)
			filename = "n0initvals16";
		else if (rev >= 11 && rev <= 12)
			filename = "n0initvals11";
		break;
	case B43_PHYTYPE_LP:
		if (rev >= 16 && rev <= 18)
			filename = "lp0initvals16";
		else if (rev == 15)
			filename = "lp0initvals15";
		else if (rev == 14)
			filename = "lp0initvals14";
		else if (rev == 13)
			filename = "lp0initvals13";
		break;
	case B43_PHYTYPE_HT:
		if (rev == 29)
			filename = "ht0initvals29";
		else if (rev == 26)
			filename = "ht0initvals26";
		break;
	case B43_PHYTYPE_LCN:
		if (rev == 24)
			filename = "lcn0initvals24";
		break;
	case B43_PHYTYPE_LCN40:
		if (rev == 33)
			filename = "lcn400initvals33";
		break;
	case B43_PHYTYPE_AC:
		if (rev == 56)
			filename = "ac24initvals56";
		else if (rev == 42)
			filename = "ac1initvals42";
		else if (rev == 40)
			filename = "ac0initvals40";
		break;
	}
	if (!filename)
		goto err_no_initvals;
	err = b43_do_request_fw(ctx, filename, &fw->initvals, false);
	if (err)
		goto err_load;

	/* Get bandswitch initvals */
	filename = NULL;
	switch (dev->phy.type) {
	case B43_PHYTYPE_G:
		if (rev == 13)
			filename = "b0g0bsinitvals13";
		else if (rev >= 5 && rev <= 10)
			filename = "b0g0bsinitvals5";
		break;
	case B43_PHYTYPE_N:
		if (rev == 30)
			filename = "n16bsinitvals30";
		else if (rev == 28 || rev == 25)
			filename = "n0bsinitvals25";
		else if (rev == 24)
			filename = "n0bsinitvals24";
		else if (rev == 23)
			filename = "n0bsinitvals16"; /* What about n0bsinitvals22? */
		else if (rev >= 16 && rev <= 18)
			filename = "n0bsinitvals16";
		else if (rev >= 11 && rev <= 12)
			filename = "n0bsinitvals11";
		break;
	case B43_PHYTYPE_LP:
		if (rev >= 16 && rev <= 18)
			filename = "lp0bsinitvals16";
		else if (rev == 15)
			filename = "lp0bsinitvals15";
		else if (rev == 14)
			filename = "lp0bsinitvals14";
		else if (rev == 13)
			filename = "lp0bsinitvals13";
		break;
	case B43_PHYTYPE_HT:
		if (rev == 29)
			filename = "ht0bsinitvals29";
		else if (rev == 26)
			filename = "ht0bsinitvals26";
		break;
	case B43_PHYTYPE_LCN:
		if (rev == 24)
			filename = "lcn0bsinitvals24";
		break;
	case B43_PHYTYPE_LCN40:
		if (rev == 33)
			filename = "lcn400bsinitvals33";
		break;
	case B43_PHYTYPE_AC:
		if (rev == 56)
			filename = "ac24bsinitvals56";
		else if (rev == 42)
			filename = "ac1bsinitvals42";
		else if (rev == 40)
			filename = "ac0bsinitvals40";
		break;
	}
	if (!filename)
		goto err_no_initvals;
	err = b43_do_request_fw(ctx, filename, &fw->initvals_band, false);
	if (err)
		goto err_load;

	fw->opensource = (ctx->req_type == B43_FWTYPE_OPENSOURCE);

	return 0;

err_no_ucode:
	err = ctx->fatal_failure = -EOPNOTSUPP;
	b43err(dev->wl, "The driver does not know which firmware (ucode) "
	       "is required for your device (wl-core rev %u)\n", rev);
	goto error;

err_no_pcm:
	err = ctx->fatal_failure = -EOPNOTSUPP;
	b43err(dev->wl, "The driver does not know which firmware (PCM) "
	       "is required for your device (wl-core rev %u)\n", rev);
	goto error;

err_no_initvals:
	err = ctx->fatal_failure = -EOPNOTSUPP;
	b43err(dev->wl, "The driver does not know which firmware (initvals) "
	       "is required for your device (wl-core rev %u)\n", rev);
	goto error;

err_load:
	/* We failed to load this firmware image. The error message
	 * already is in ctx->errors. Return and let our caller decide
	 * what to do. */
	goto error;

error:
	b43_release_firmware(dev);
	return err;
}

static int b43_one_core_attach(struct b43_bus_dev *dev, struct b43_wl *wl);
static void b43_one_core_detach(struct b43_bus_dev *dev);
static int b43_rng_init(struct b43_wl *wl);

static void b43_request_firmware(struct work_struct *work)
{
	struct b43_wl *wl = container_of(work,
			    struct b43_wl, firmware_load);
	struct b43_wldev *dev = wl->current_dev;
	struct b43_request_fw_context *ctx;
	unsigned int i;
	int err;
	const char *errmsg;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return;
	ctx->dev = dev;

	ctx->req_type = B43_FWTYPE_PROPRIETARY;
	err = b43_try_request_fw(ctx);
	if (!err)
		goto start_ieee80211; /* Successfully loaded it. */
	/* Was fw version known? */
	if (ctx->fatal_failure)
		goto out;

	/* proprietary fw not found, try open source */
	ctx->req_type = B43_FWTYPE_OPENSOURCE;
	err = b43_try_request_fw(ctx);
	if (!err)
		goto start_ieee80211; /* Successfully loaded it. */
	if(ctx->fatal_failure)
		goto out;

	/* Could not find a usable firmware. Print the errors. */
	for (i = 0; i < B43_NR_FWTYPES; i++) {
		errmsg = ctx->errors[i];
		if (strlen(errmsg))
			b43err(dev->wl, "%s", errmsg);
	}
	b43_print_fw_helptext(dev->wl, 1);
	goto out;

start_ieee80211:
	wl->hw->queues = B43_QOS_QUEUE_NUM;
	if (!modparam_qos || dev->fw.opensource ||
	    dev->dev->chip_id == BCMA_CHIP_ID_BCM4331)
		wl->hw->queues = 1;

	err = ieee80211_register_hw(wl->hw);
	if (err)
		goto out;
	wl->hw_registered = true;
	b43_leds_register(wl->current_dev);

	/* Register HW RNG driver */
	b43_rng_init(wl);

out:
	kfree(ctx);
}

/*
 * acphy_initfix bits 8 and 16 (phy_ac.c)
 * at the ucode upload of a core rev >= 40 AC core. b43 zeroes 4 KB of
 * shared memory; ucode 907's initvals write from 0x1000 up (0x10ca,
 * 0x17d2, 0x17e4..) and the ucode keeps words of its own there (0x17f4..
 * 0x180e the PHY control read-back, 0x1816 and 0x1822..0x1832 the packet
 * engine), which a controller restart, and a module
 * reload with the PMU resources never dropped, inherit. Bit 16 logs the
 * words before anything touches them (the fresh-load value on the first
 * init, the previous init's on a restart); bit 8 zeroes 0x1000 up to
 * acphy_initfix_shm bytes (default 0x2000) with the 4 KB, before the
 * initvals. Addresses past the core's shared memory wrap (the control
 * word's offset is masked), so a too-large end only re-zeroes the low
 * part; bounded at 32 KB.
 */
static void b43_ac_initfix_shm(struct b43_wldev *dev)
{
	static const u16 words[] = {
		0x10CA, 0x17D2, 0x17D4, 0x17F4, 0x17F6, 0x17F8, 0x17FA,
		0x1816, 0x1822, 0x1824, 0x1826, 0x1832,
	};
	unsigned int bits = b43_modparam_acphy_initfix;
	unsigned int i, end;
	char line[160];
	int len = 0;

	if (!(bits & (B43_AC_INITFIX_SHM | B43_AC_INITFIX_LOG)) ||
	    !b43_txhdr_is_ac(dev))
		return;
	if (bits & B43_AC_INITFIX_LOG) {
		for (i = 0; i < ARRAY_SIZE(words); i++)
			len += scnprintf(line + len, sizeof(line) - len,
					 " %04x:%04x", words[i],
					 b43_shm_read16(dev, B43_SHM_SHARED,
							words[i]));
	}
	if (!(bits & B43_AC_INITFIX_SHM))
		return;
	end = clamp(b43_modparam_acphy_initfix_shm, 0x1000U, 0x8000U) & ~1U;
	for (i = 0x1000; i < end; i += 2)
		b43_shm_write16(dev, B43_SHM_SHARED, i, 0);
}

/*
 * acphy_initfix bits 4 and 16 at the end of b43_wireless_core_init:
 * the per-wldev AC state that outlives a core re-init. The PHY TX error
 * count only grows; the A-MPDU epoch state is zeroed at init only with
 * TX aggregation on; the HT-dead restart count is this chain's and is logged,
 * not touched.
 */
static void b43_ac_initfix_wldev(struct b43_wldev *dev)
{
	unsigned int bits = b43_modparam_acphy_initfix;

	if (!(bits & (B43_AC_INITFIX_WLDEV | B43_AC_INITFIX_LOG)) ||
	    !b43_txhdr_is_ac(dev))
		return;
	if (!(bits & B43_AC_INITFIX_WLDEV))
		return;
	dev->phy_txerr_total = 0;
	memset(dev->ac_ampdu_epoch, 0, sizeof(dev->ac_ampdu_epoch));
}

static int b43_upload_microcode(struct b43_wldev *dev)
{
	struct wiphy *wiphy = dev->wl->hw->wiphy;
	const size_t hdr_len = sizeof(struct b43_fw_header);
	const __be32 *data;
	unsigned int i, len;
	u16 fwrev, fwpatch, fwdate, fwtime;
	u32 tmp, macctl;
	int err = 0;

	/* Jump the microcode PSM to offset 0 */
	macctl = b43_read32(dev, B43_MMIO_MACCTL);
	B43_WARN_ON(macctl & B43_MACCTL_PSM_RUN);
	macctl |= B43_MACCTL_PSM_JMP0;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);
	/* what the ucode's memory above 4 KB holds before this init */
	b43_ac_initfix_shm(dev);
	/* Zero out all microcode PSM registers and shared memory. */
	for (i = 0; i < 64; i++)
		b43_shm_write16(dev, B43_SHM_SCRATCH, i, 0);
	for (i = 0; i < 4096; i += 2)
		b43_shm_write16(dev, B43_SHM_SHARED, i, 0);

	/* Upload Microcode. */
	data = (__be32 *) (dev->fw.ucode.data->data + hdr_len);
	len = (dev->fw.ucode.data->size - hdr_len) / sizeof(__be32);
	b43_shm_control_word(dev, B43_SHM_UCODE | B43_SHM_AUTOINC_W, 0x0000);
	for (i = 0; i < len; i++) {
		b43_write32(dev, B43_MMIO_SHM_DATA, be32_to_cpu(data[i]));
		udelay(10);
	}

	if (dev->fw.pcm.data) {
		/* Upload PCM data. */
		data = (__be32 *) (dev->fw.pcm.data->data + hdr_len);
		len = (dev->fw.pcm.data->size - hdr_len) / sizeof(__be32);
		b43_shm_control_word(dev, B43_SHM_HW, 0x01EA);
		b43_write32(dev, B43_MMIO_SHM_DATA, 0x00004000);
		/* No need for autoinc bit in SHM_HW */
		b43_shm_control_word(dev, B43_SHM_HW, 0x01EB);
		for (i = 0; i < len; i++) {
			b43_write32(dev, B43_MMIO_SHM_DATA, be32_to_cpu(data[i]));
			udelay(10);
		}
	}

	b43_write32(dev, B43_MMIO_GEN_IRQ_REASON, B43_IRQ_ALL);

	/* Start the microcode PSM */
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_PSM_JMP0,
		      B43_MACCTL_PSM_RUN);

	/* Wait for the microcode to load and respond */
	i = 0;
	while (1) {
		tmp = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
		if (tmp == B43_IRQ_MAC_SUSPENDED)
			break;
		i++;
		if (i >= 20) {
			b43err(dev->wl, "Microcode not responding\n");
			b43_print_fw_helptext(dev->wl, 1);
			err = -ENODEV;
			goto error;
		}
		msleep(50);
	}
	b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);	/* dummy read */

	/* Get and check the revisions. */
	fwrev = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODEREV);
	fwpatch = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODEPATCH);
	fwdate = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODEDATE);
	fwtime = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_UCODETIME);

	if (fwrev <= 0x128) {
		b43err(dev->wl, "YOUR FIRMWARE IS TOO OLD. Firmware from "
		       "binary drivers older than version 4.x is unsupported. "
		       "You must upgrade your firmware files.\n");
		b43_print_fw_helptext(dev->wl, 1);
		err = -EOPNOTSUPP;
		goto error;
	}
	dev->fw.rev = fwrev;
	dev->fw.patch = fwpatch;
	if (dev->fw.rev >= 598)
		dev->fw.hdr_format = B43_FW_HDR_598;
	else if (dev->fw.rev >= 410)
		dev->fw.hdr_format = B43_FW_HDR_410;
	else
		dev->fw.hdr_format = B43_FW_HDR_351;
	WARN_ON(dev->fw.opensource != (fwdate == 0xFFFF));

	dev->qos_enabled = dev->wl->hw->queues > 1;
	/* Default to firmware/hardware crypto acceleration. */
	dev->hwcrypto_enabled = true;

	if (dev->fw.opensource) {
		u16 fwcapa;

		/* Patchlevel info is encoded in the "time" field. */
		dev->fw.patch = fwtime;
		b43info(dev->wl, "Loading OpenSource firmware version %u.%u\n",
			dev->fw.rev, dev->fw.patch);

		fwcapa = b43_fwcapa_read(dev);
		if (!(fwcapa & B43_FWCAPA_HWCRYPTO) || dev->fw.pcm_request_failed) {
			b43info(dev->wl, "Hardware crypto acceleration not supported by firmware\n");
			/* Disable hardware crypto and fall back to software crypto. */
			dev->hwcrypto_enabled = false;
		}
		/* adding QoS support should use an offline discovery mechanism */
		WARN(fwcapa & B43_FWCAPA_QOS, "QoS in OpenFW not supported\n");
	} else {
		b43info(dev->wl, "Loading firmware version %u.%u "
			"(20%.2i-%.2i-%.2i %.2i:%.2i:%.2i)\n",
			fwrev, fwpatch,
			(fwdate >> 12) & 0xF, (fwdate >> 8) & 0xF, fwdate & 0xFF,
			(fwtime >> 11) & 0x1F, (fwtime >> 5) & 0x3F, fwtime & 0x1F);
		if (dev->fw.pcm_request_failed) {
			b43warn(dev->wl, "No \"pcm5.fw\" firmware file found. "
				"Hardware accelerated cryptography is disabled.\n");
			b43_print_fw_helptext(dev->wl, 0);
		}
	}

	snprintf(wiphy->fw_version, sizeof(wiphy->fw_version), "%u.%u",
			dev->fw.rev, dev->fw.patch);
	wiphy->hw_version = dev->dev->core_id;

	if (dev->fw.hdr_format == B43_FW_HDR_351) {
		/* We're over the deadline, but we keep support for old fw
		 * until it turns out to be in major conflict with something new. */
		b43warn(dev->wl, "You are using an old firmware image. "
			"Support for old firmware will be removed soon "
			"(official deadline was July 2008).\n");
		b43_print_fw_helptext(dev->wl, 0);
	}

	return 0;

error:
	/* Stop the microcode PSM. */
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_PSM_RUN,
		      B43_MACCTL_PSM_JMP0);

	return err;
}

static int b43_write_initvals(struct b43_wldev *dev,
			      const struct b43_iv *ivals,
			      size_t count,
			      size_t array_size)
{
	const struct b43_iv *iv;
	u16 offset;
	size_t i;
	bool bit32;

	BUILD_BUG_ON(sizeof(struct b43_iv) != 6);
	iv = ivals;
	for (i = 0; i < count; i++) {
		if (array_size < sizeof(iv->offset_size))
			goto err_format;
		array_size -= sizeof(iv->offset_size);
		offset = be16_to_cpu(iv->offset_size);
		bit32 = !!(offset & B43_IV_32BIT);
		offset &= B43_IV_OFFSET_MASK;
		if (offset >= 0x1000)
			goto err_format;
		if (bit32) {
			u32 value;

			if (array_size < sizeof(iv->data.d32))
				goto err_format;
			array_size -= sizeof(iv->data.d32);

			value = get_unaligned_be32(&iv->data.d32);
			b43_write32(dev, offset, value);

			iv = (const struct b43_iv *)((const uint8_t *)iv +
							sizeof(__be16) +
							sizeof(__be32));
		} else {
			u16 value;

			if (array_size < sizeof(iv->data.d16))
				goto err_format;
			array_size -= sizeof(iv->data.d16);

			value = be16_to_cpu(iv->data.d16);
			b43_write16(dev, offset, value);

			iv = (const struct b43_iv *)((const uint8_t *)iv +
							sizeof(__be16) +
							sizeof(__be16));
		}
	}
	if (array_size)
		goto err_format;

	return 0;

err_format:
	b43err(dev->wl, "Initial Values Firmware file-format error.\n");
	b43_print_fw_helptext(dev->wl, 1);

	return -EPROTO;
}

static int b43_upload_initvals(struct b43_wldev *dev)
{
	const size_t hdr_len = sizeof(struct b43_fw_header);
	const struct b43_fw_header *hdr;
	struct b43_firmware *fw = &dev->fw;
	const struct b43_iv *ivals;
	size_t count;

	hdr = (const struct b43_fw_header *)(fw->initvals.data->data);
	ivals = (const struct b43_iv *)(fw->initvals.data->data + hdr_len);
	count = be32_to_cpu(hdr->size);
	return b43_write_initvals(dev, ivals, count,
				 fw->initvals.data->size - hdr_len);
}

static int b43_upload_initvals_band(struct b43_wldev *dev)
{
	const size_t hdr_len = sizeof(struct b43_fw_header);
	const struct b43_fw_header *hdr;
	struct b43_firmware *fw = &dev->fw;
	const struct b43_iv *ivals;
	size_t count;

	if (!fw->initvals_band.data)
		return 0;

	hdr = (const struct b43_fw_header *)(fw->initvals_band.data->data);
	ivals = (const struct b43_iv *)(fw->initvals_band.data->data + hdr_len);
	count = be32_to_cpu(hdr->size);
	return b43_write_initvals(dev, ivals, count,
				  fw->initvals_band.data->size - hdr_len);
}

/* Initialize the GPIOs
 * https://bcm-specs.sipsolutions.net/GPIO
 */

#ifdef CPTCFG_B43_SSB
static struct ssb_device *b43_ssb_gpio_dev(struct b43_wldev *dev)
{
	struct ssb_bus *bus = dev->dev->sdev->bus;

#ifdef CONFIG_SSB_DRIVER_PCICORE
	return (bus->chipco.dev ? bus->chipco.dev : bus->pcicore.dev);
#else
	return bus->chipco.dev;
#endif
}
#endif

static int b43_gpio_init(struct b43_wldev *dev)
{
#ifdef CPTCFG_B43_SSB
	struct ssb_device *gpiodev;
#endif
	u32 mask, set;

	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_GPOUTSMSK, 0);
	b43_maskset16(dev, B43_MMIO_GPIO_MASK, ~0, modparam_gpiomask);

	mask = 0x0000001F;
	set = modparam_gpiomask;
	if (dev->dev->chip_id == 0x4301) {
		mask |= 0x0060;
		set |= 0x0060;
	} else if (dev->dev->chip_id == 0x5354) {
		/* Don't allow overtaking buttons GPIOs */
		set &= 0x2; /* 0x2 is LED GPIO on BCM5354 */
	} else if (dev->dev->chip_id == BCMA_CHIP_ID_BCM4716 || 
		   dev->dev->chip_id == BCMA_CHIP_ID_BCM47162 ||
		   dev->dev->chip_id == BCMA_CHIP_ID_BCM5356 ||
		   dev->dev->chip_id == BCMA_CHIP_ID_BCM5357 ||
		   dev->dev->chip_id == BCMA_CHIP_ID_BCM53572) {
		/* just use gpio 0 and 1 for 2.4 GHz wifi led */
		set &= 0x3;
		mask &= 0x3;
	}

	if (0 /* FIXME: conditional unknown */ ) {
		b43_write16(dev, B43_MMIO_GPIO_MASK,
			    b43_read16(dev, B43_MMIO_GPIO_MASK)
			    | 0x0100);
		/* BT Coexistance Input */
		mask |= 0x0080;
		set |= 0x0080;
		/* BT Coexistance Out */
		mask |= 0x0100;
		set |= 0x0100;
	}
	if (dev->dev->bus_sprom->boardflags_lo & B43_BFL_PACTRL) {
		/* PA is controlled by gpio 9, let ucode handle it */
		b43_write16(dev, B43_MMIO_GPIO_MASK,
			    b43_read16(dev, B43_MMIO_GPIO_MASK)
			    | 0x0200);
		mask |= 0x0200;
		set |= 0x0200;
	}

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_chipco_gpio_control(&dev->dev->bdev->bus->drv_cc, mask, set);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		gpiodev = b43_ssb_gpio_dev(dev);
		if (gpiodev)
			ssb_write32(gpiodev, B43_GPIO_CONTROL,
				    (ssb_read32(gpiodev, B43_GPIO_CONTROL)
				    & ~mask) | set);
		break;
#endif
	}

	return 0;
}

/* Turn off all GPIO stuff. Call this on module unload, for example. */
static void b43_gpio_cleanup(struct b43_wldev *dev)
{
#ifdef CPTCFG_B43_SSB
	struct ssb_device *gpiodev;
#endif

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_chipco_gpio_control(&dev->dev->bdev->bus->drv_cc, ~0, 0);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		gpiodev = b43_ssb_gpio_dev(dev);
		if (gpiodev)
			ssb_write32(gpiodev, B43_GPIO_CONTROL, 0);
		break;
#endif
	}
}

/* http://bcm-specs.sipsolutions.net/EnableMac */
void b43_mac_enable(struct b43_wldev *dev)
{
	if (b43_debug(dev, B43_DBG_FIRMWARE)) {
		u16 fwstate;

		fwstate = b43_shm_read16(dev, B43_SHM_SHARED,
					 B43_SHM_SH_UCODESTAT);
		if ((fwstate != B43_SHM_SH_UCODESTAT_SUSP) &&
		    (fwstate != B43_SHM_SH_UCODESTAT_SLEEP)) {
			b43err(dev->wl, "b43_mac_enable(): The firmware "
			       "should be suspended, but current state is %u\n",
			       fwstate);
		}
	}

	dev->mac_suspended--;
	B43_WARN_ON(dev->mac_suspended < 0);
	if (dev->mac_suspended == 0) {
		b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_ENABLED);
		b43_write32(dev, B43_MMIO_GEN_IRQ_REASON,
			    B43_IRQ_MAC_SUSPENDED);
		/* Commit writes */
		b43_read32(dev, B43_MMIO_MACCTL);
		b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
		b43_power_saving_ctl_bits(dev, 0);
	}
}

/* https://bcm-specs.sipsolutions.net/SuspendMAC */
void b43_mac_suspend(struct b43_wldev *dev)
{
	int i;
	u32 tmp;

	might_sleep();
	B43_WARN_ON(dev->mac_suspended < 0);

	if (dev->mac_suspended == 0) {
		b43_power_saving_ctl_bits(dev, B43_PS_AWAKE);
		b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_ENABLED, 0);
		/* force pci to flush the write */
		b43_read32(dev, B43_MMIO_MACCTL);
		for (i = 35; i; i--) {
			tmp = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
			if (tmp & B43_IRQ_MAC_SUSPENDED)
				goto out;
			udelay(10);
		}
		/* Hm, it seems this will take some time. Use msleep(). */
		for (i = 40; i; i--) {
			tmp = b43_read32(dev, B43_MMIO_GEN_IRQ_REASON);
			if (tmp & B43_IRQ_MAC_SUSPENDED)
				goto out;
			msleep(1);
		}
		b43err(dev->wl, "MAC suspend failed\n");
	}
out:
	dev->mac_suspended++;
}

/* https://bcm-v4.sipsolutions.net/802.11/PHY/N/MacPhyClkSet */
void b43_mac_phy_clock_set(struct b43_wldev *dev, bool on)
{
	u32 tmp;

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
		if (on)
			tmp |= B43_BCMA_IOCTL_MACPHYCLKEN;
		else
			tmp &= ~B43_BCMA_IOCTL_MACPHYCLKEN;
		bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, tmp);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		tmp = ssb_read32(dev->dev->sdev, SSB_TMSLOW);
		if (on)
			tmp |= B43_TMSLOW_MACPHYCLKEN;
		else
			tmp &= ~B43_TMSLOW_MACPHYCLKEN;
		ssb_write32(dev->dev->sdev, SSB_TMSLOW, tmp);
		break;
#endif
	}
}

/* brcms_b_switch_macfreq */
void b43_mac_switch_freq(struct b43_wldev *dev, u8 spurmode)
{
	u16 chip_id = dev->dev->chip_id;

	if (chip_id == BCMA_CHIP_ID_BCM4331) {
		switch (spurmode) {
		case 2: /* 168 Mhz: 2^26/168 = 0x61862 */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x1862);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x6);
			break;
		case 1: /* 164 Mhz: 2^26/164 = 0x63e70 */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x3e70);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x6);
			break;
		default: /* 160 Mhz: 2^26/160 = 0x66666 */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x6666);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x6);
			break;
		}
	} else if (chip_id == BCMA_CHIP_ID_BCM43131 ||
	    chip_id == BCMA_CHIP_ID_BCM43217 ||
	    chip_id == BCMA_CHIP_ID_BCM43222 ||
	    chip_id == BCMA_CHIP_ID_BCM43224 ||
	    chip_id == BCMA_CHIP_ID_BCM43225 ||
	    chip_id == BCMA_CHIP_ID_BCM43227 ||
	    chip_id == BCMA_CHIP_ID_BCM43228) {
		switch (spurmode) {
		case 2: /* 126 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x2082);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x8);
			break;
		case 1: /* 123 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x5341);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x8);
			break;
		default: /* 120 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x8889);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0x8);
			break;
		}
	} else if (dev->phy.type == B43_PHYTYPE_LCN) {
		switch (spurmode) {
		case 1: /* 82 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0x7CE0);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0xC);
			break;
		default: /* 80 Mhz */
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_LOW, 0xCCCD);
			b43_write16(dev, B43_MMIO_TSF_CLK_FRAC_HIGH, 0xC);
			break;
		}
	}
}

static void b43_adjust_opmode(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	u32 ctl;
	u16 cfp_pretbtt;

	ctl = b43_read32(dev, B43_MMIO_MACCTL);
	/* Reset status to STA infrastructure mode. */
	ctl &= ~B43_MACCTL_AP;
	ctl &= ~B43_MACCTL_KEEP_CTL;
	ctl &= ~B43_MACCTL_KEEP_BADPLCP;
	ctl &= ~B43_MACCTL_KEEP_BAD;
	ctl &= ~B43_MACCTL_PROMISC;
	ctl &= ~B43_MACCTL_BEACPROMISC;
	ctl |= B43_MACCTL_INFRA;

	if (b43_is_mode(wl, NL80211_IFTYPE_AP) ||
	    b43_is_mode(wl, NL80211_IFTYPE_MESH_POINT))
		ctl |= B43_MACCTL_AP;
	else if (b43_is_mode(wl, NL80211_IFTYPE_ADHOC))
		ctl &= ~B43_MACCTL_INFRA;

	if (wl->filter_flags & FIF_CONTROL)
		ctl |= B43_MACCTL_KEEP_CTL;
	if (wl->filter_flags & FIF_FCSFAIL)
		ctl |= B43_MACCTL_KEEP_BAD;
	if (wl->filter_flags & FIF_PLCPFAIL)
		ctl |= B43_MACCTL_KEEP_BADPLCP;
	if (wl->filter_flags & FIF_BCN_PRBRESP_PROMISC)
		ctl |= B43_MACCTL_BEACPROMISC;

	/* Workaround: On old hardware the HW-MAC-address-filter
	 * doesn't work properly, so always run promisc in filter
	 * it in software. */
	if (dev->dev->core_rev <= 4)
		ctl |= B43_MACCTL_PROMISC;

	b43_write32(dev, B43_MMIO_MACCTL, ctl);

	cfp_pretbtt = 2;
	if ((ctl & B43_MACCTL_INFRA) && !(ctl & B43_MACCTL_AP)) {
		if (dev->dev->chip_id == 0x4306 &&
		    dev->dev->chip_rev == 3)
			cfp_pretbtt = 100;
		else
			cfp_pretbtt = 50;
	}
	b43_write16(dev, 0x612, cfp_pretbtt);

	/* FIXME: We don't currently implement the PMQ mechanism,
	 *        so always disable it. If we want to implement PMQ,
	 *        we need to enable it here (clear DISCPMQ) in AP mode.
	 */
	/*
	 * acphy_psfix bit 2 enables the PMQ for an AP on core rev >= 40
	 * (b43_ac_pmq_drain), as wl clears DISCPMQ for an AP. ucode 907 reads
	 * the bit only when the PSM (re)starts (@0x0DE3..0x0DE5, L785 from
	 * the MAC enable wait @0x0DC0), so a change while the MAC runs takes
	 * effect through a suspend / enable cycle.
	 */
	if ((ctl & B43_MACCTL_AP) && b43_ac_psfix(dev, B43_AC_PSFIX_PMQ)) {
		bool was_set = b43_read32(dev, B43_MMIO_MACCTL) &
			       B43_MACCTL_DISCPMQ;

		b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_DISCPMQ, 0);
		if (was_set && dev->mac_suspended == 0) {
			b43_mac_suspend(dev);
			b43_mac_enable(dev);
		}
	} else {
		b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_DISCPMQ);
	}
}

/*
 * Buffer memory and TX FIFO setup for core rev 40..42 (ucode 907).
 *
 * wl.ko wlc_bmac_init runs this right after the core rev 42 initvals
 * (wlc_write_inits, call 0x162b14) when core rev > 39 (0x162b20), and skips
 * the legacy shm FIFO size words SIZE01..67 (0x162e60 -> 0x163258). Neither
 * initvals42 nor bsinitvals42 writes 0x520..0x550, and b43 never touches
 * them, so without this the TX FIFOs own no buffers: TX DMA can hand frames
 * to the MAC but the ucode has nothing to transmit from and returns no TX
 * status, while RX keeps working.
 * Register names are guesses; the values and their order are wl's:
 * - buffers = MAC_HW_CAP (0x15c) bits 3..12, times 4 (0x162b44..0x162b5c),
 *   into 0x542 (0x162b90); 5 into 0x540, poll bit 0 clear, 20 x 10 us
 *   (0x162bac..0x162be8);
 * - per FIFO in the order 7, 0, 1, 2, 3, 4, 5 (.rodata 0xd69f8; loop
 *   0x162c28..0x162d48): 0x54a, 0x54c, 0x520, 0x54e, 0x550, then 0x548 =
 *   fifo | 0x10. FIFO 7: 42, 42, 42, 42 | 38 << 8, 0x740c. FIFOs 0..5:
 *   buffers - 42, minbuf[rev - 40][fifo] (.rodata 0xd699c + 0x78,
 *   0x162c44..0x162c5c), 11, 22 | 18 << 8, 0x740c;
 * - for i = 0..41 (0x162d6c..0x162e58): 0x534 = i, 0x536 = hi =
 *   min(i + 2, 41), 0x532 = hi + 1 - i, 0x530 = i << 4 | 0x8007
 *   (literal 0x1632bc), poll 0x530 until 0, 20 x 10 us.
 */
/*
 * Buffer manager (BMC) init for core rev 56 (BCM53573 5 GHz, wl 9.10.178.27):
 * the core rev 56 branch of wlc_coreinit (cmp #56 at 0x166430), single MAC
 * unit (wlc_hw->macunits <= 1, the path b43 takes on unit 0). Parameters from
 * the rev 56 table at .rodata+0x7f9a4 of the Tenda wl.ko: FIFO order
 * {7, 0, 1, 2, 3, 4, 5, 6, 8}, then { 1, 0x80, 0x80, 1, 1, 0,
 * minbuf[9] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x20, 0x00, 0x0a } }.
 * 0x166c2c..0x16743c; b43_bmc_init_rev40 is the rev 40..42 version.
 */
static void b43_bmc_init_rev56(struct b43_wldev *dev)
{
	static const u8 fifos[] = { 7, 0, 1, 2, 3, 4, 5, 6, 8 };
	static const u16 minbuf[9] = {
		0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x20, 0x00, 0x0a,
	};
	const u16 tpl = 84;	/* template buffers, one MAC unit */
	const u16 a = 0x80, b = 0x80;
	u16 bufs, v, hi;
	unsigned int i, k, fails = 0;
	u32 cap;

	b43_write32(dev, B43_MMIO_MACCTL,
		    b43_read32(dev, B43_MMIO_MACCTL) | 0x400);
	cap = b43_read32(dev, B43_MMIO_MAC_HW_CAP);
	bufs = cap & 0xff8;
	b43info(dev->wl, "BMC rev56: MAC_HW_CAP 0x%08x, %u buffers\n", cap, bufs);

	b43_write16(dev, 0x42c, 0);
	b43_write16(dev, 0x42e, (a << 7) - 1);
	b43_write16(dev, 0x43a, a << 7);
	b43_write16(dev, 0x43c, ((a + b) << 7) - 1);
	b43_write16(dev, 0x406, 0x101);
	b43_write16(dev, 0x406, 1);

	b43_write16(dev, 0x542, bufs);
	b43_write16(dev, 0x540, 1);
	for (k = 0; k < 21 && (b43_read16(dev, 0x540) & 1); k++)
		udelay(10);
	if (b43_read16(dev, 0x540) & 1)
		b43warn(dev->wl, "BMC rev56: 0x540 bit 0 still set\n");

	for (i = 0; i < ARRAY_SIZE(fifos); i++) {
		u8 f = fifos[i];
		u16 maxb, minb, rsv, ctl;

		if (f == 7) {
			maxb = minb = rsv = tpl;
			ctl = (tpl | ((tpl - 4) << 11)) & 0xffff;	/* wl: uxth */
		} else {
			if (f == 6 || f == 8) {
				u16 m = minbuf[f] << 1;

				b43_write16(dev, 0xb00, m);
				maxb = minb = m + 3;
			} else {
				maxb = bufs - tpl;
				minb = minbuf[f] << 1;
			}
			rsv = 1;
			ctl = 2 | (2 << 11);
		}
		b43_write16(dev, 0x54a, maxb);
		b43_write16(dev, 0x54c, minb);
		b43_write16(dev, 0x520, rsv);
		b43_write16(dev, 0x54e, ctl);
		b43_write16(dev, 0xb04, 8);
		b43_write16(dev, 0x550, 0x740c);
		b43_write16(dev, 0x548, f | 0x10);
		if (f == 7) {
			/* wl: wait for the template FIFO count, 0xb30 == 84 */
			for (k = 0; k < 20 && b43_read16(dev, 0xb30) != tpl; k++)
				udelay(10);
			v = b43_read16(dev, 0xb30);
			if (v != tpl)
				b43warn(dev->wl, "BMC rev56: 0xb30 reads %u, "
					"not %u\n", v, tpl);
		}
	}

	for (i = 0; i < tpl; i++) {
		hi = min_t(u16, i + 3, tpl - 1);
		b43_write16(dev, 0x534, i);
		b43_write16(dev, 0x536, hi);
		b43_write16(dev, 0x532, hi + 1 - i);
		b43_write16(dev, 0x530, (i << 4) | 0x8007);
		/* wl: 21 x 10 us for the whole register to read 0 */
		for (k = 0; k < 21 && b43_read16(dev, 0x530); k++)
			udelay(10);
		if ((b43_read16(dev, 0x530) & 0x8000) && !fails++)
			b43warn(dev->wl, "BMC rev56: entry %u, 0x530 0x%04x\n",
				i, b43_read16(dev, 0x530));
	}
	if (fails)
		b43warn(dev->wl, "BMC rev56: %u of %u entries did not complete\n",
			fails, tpl);
}

static void b43_bmc_init_rev40(struct b43_wldev *dev)
{
	static const u8 fifos[] = { 7, 0, 1, 2, 3, 4, 5 };
	static const u16 minbuf[3][6] = {
		{ 0x20, 0x20, 0x20, 0x20, 0x20, 0x08 },	/* core rev 40 */
		{ 0x20, 0x20, 0x20, 0x20, 0x20, 0x00 },	/* core rev 41 */
		{ 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 },	/* core rev 42 */
	};
	u8 rev = dev->dev->core_rev;
	u16 bufs, a, b, c, d, v;
	unsigned int i, k, fails = 0;
	u32 cap;

	if (rev == 56) {
		b43_bmc_init_rev56(dev);
		return;
	}
	if (rev < 40 || rev > 42)
		return;

	cap = b43_read32(dev, B43_MMIO_MAC_HW_CAP);
	bufs = ((cap >> 3) & 0x3FF) << 2;
	b43info(dev->wl, "BMC: MAC_HW_CAP 0x%08x, %u buffers, MACCTL 0x%08x\n",
		cap, bufs, b43_read32(dev, B43_MMIO_MACCTL));
	b43_write16(dev, 0x542, bufs);
	b43_write16(dev, 0x540, 5);
	for (k = 0; k < 20 && (b43_read16(dev, 0x540) & 1); k++)
		udelay(10);
	if (b43_read16(dev, 0x540) & 1)
		b43warn(dev->wl, "BMC: 0x540 bit 0 still set\n");

	for (i = 0; i < ARRAY_SIZE(fifos); i++) {
		u8 f = fifos[i];

		if (f == 7) {
			a = b = c = d = 42;
		} else {
			a = bufs - 42;
			b = minbuf[rev - 40][f];
			c = 11;
			d = 22;
		}
		b43_write16(dev, 0x54A, a);
		b43_write16(dev, 0x54C, b);
		b43_write16(dev, 0x520, c);
		b43_write16(dev, 0x54E, d | ((d - 4) << 8));
		b43_write16(dev, 0x550, 0x740C);
		b43_write16(dev, 0x548, f | 0x10);
	}

	for (i = 0; i < 42; i++) {
		u16 hi = min_t(u16, i + 2, 41);

		b43_write16(dev, 0x534, i);
		b43_write16(dev, 0x536, hi);
		b43_write16(dev, 0x532, hi + 1 - i);
		b43_write16(dev, 0x530, (i << 4) | 0x8007);
		/*
		 * wl waits for the whole register to read 0 (ldrh / cmp #0 /
		 * beq at 0x162e18..0x162e2c), 20 x 10 us, and never checks
		 * the result. On hardware (2026-10-06 12:24 UTC) bit 15 clears
		 * and the command bits stay (0x8007 -> 0x0007), as bit 0 of
		 * 0x540 clears and 0x4 stays; wl then just spends the full
		 * 200 us per entry. Poll bit 15, the bit that clears.
		 */
		for (k = 0; k < 20 && (b43_read16(dev, 0x530) & 0x8000); k++)
			udelay(10);
		v = b43_read16(dev, 0x530);
		if ((v & 0x8000) && !fails++)
			b43warn(dev->wl, "BMC: entry %u, 0x530 reads 0x%04x "
				"(0x540 0x%04x, MACCTL 0x%08x)\n", i, v,
				b43_read16(dev, 0x540),
				b43_read32(dev, B43_MMIO_MACCTL));
	}
	if (fails)
		b43warn(dev->wl, "BMC: %u of 42 entries did not complete\n",
			fails);
	b43dbg(dev->wl, "BMC: %u buffers, 0x540 0x%04x\n", bufs,
	       b43_read16(dev, 0x540));
}

static void b43_rate_memory_write(struct b43_wldev *dev, u16 rate, int is_ofdm)
{
	u16 offset;

	if (is_ofdm) {
		offset = 0x480;
		offset += (b43_plcp_get_ratecode_ofdm(rate) & 0x000F) * 2;
	} else {
		offset = 0x4C0;
		offset += (b43_plcp_get_ratecode_cck(rate) & 0x000F) * 2;
	}
	b43_shm_write16(dev, B43_SHM_SHARED, offset + 0x20,
			b43_shm_read16(dev, B43_SHM_SHARED, offset));
}

/*
 * Core rev >= 40 (ucode 907): wl copies each basic rate's entry of the direct
 * rate maps (OFDM at shm 0x1c0, CCK at 0x200, read by wlc_init
 * 0x12ec28..0x12ec74) into the basic rate maps at 0x1e0 / 0x220 + 2 * code
 * (wlc_set_ratetable 0x11ebc0..0x11ebd4, from wlc_bsinit 0x120d1c). Same
 * scheme as b43_rate_memory_write with the new bases. b43 had no AC case and
 * hit the B43_WARN_ON below.
 */
static void b43_rate_memory_write_rev40(struct b43_wldev *dev, u16 rate,
					int is_ofdm)
{
	u16 code, src;

	if (is_ofdm) {
		code = b43_plcp_get_ratecode_ofdm(rate) & 0x000F;
		src = 0x1C0 + code * 2;
	} else {
		code = b43_plcp_get_ratecode_cck(rate) & 0x000F;
		src = 0x200 + code * 2;
	}
	b43_shm_write16(dev, B43_SHM_SHARED, src + 0x20,
			b43_shm_read16(dev, B43_SHM_SHARED, src));
}

static void b43_rate_memory_init(struct b43_wldev *dev)
{
	if (dev->phy.type == B43_PHYTYPE_AC && dev->dev->core_rev >= 40) {
		static const u16 ofdm[] = {
			B43_OFDM_RATE_6MB, B43_OFDM_RATE_9MB,
			B43_OFDM_RATE_12MB, B43_OFDM_RATE_18MB,
			B43_OFDM_RATE_24MB, B43_OFDM_RATE_36MB,
			B43_OFDM_RATE_48MB, B43_OFDM_RATE_54MB,
		};
		static const u16 cck[] = {
			B43_CCK_RATE_1MB, B43_CCK_RATE_2MB,
			B43_CCK_RATE_5MB, B43_CCK_RATE_11MB,
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(ofdm); i++)
			b43_rate_memory_write_rev40(dev, ofdm[i], 1);
		for (i = 0; i < ARRAY_SIZE(cck); i++)
			b43_rate_memory_write_rev40(dev, cck[i], 0);
		return;
	}

	switch (dev->phy.type) {
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LP:
	case B43_PHYTYPE_HT:
	case B43_PHYTYPE_LCN:
		b43_rate_memory_write(dev, B43_OFDM_RATE_6MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_9MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_12MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_18MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_24MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_36MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_48MB, 1);
		b43_rate_memory_write(dev, B43_OFDM_RATE_54MB, 1);
		fallthrough;
	case B43_PHYTYPE_B:
		b43_rate_memory_write(dev, B43_CCK_RATE_1MB, 0);
		b43_rate_memory_write(dev, B43_CCK_RATE_2MB, 0);
		b43_rate_memory_write(dev, B43_CCK_RATE_5MB, 0);
		b43_rate_memory_write(dev, B43_CCK_RATE_11MB, 0);
		break;
	default:
		B43_WARN_ON(1);
	}
}

/* Set the default values for the PHY TX Control Words. */
static void b43_set_phytxctl_defaults(struct b43_wldev *dev)
{
	u16 ctl = 0;

	/*
	 * Core rev > 39: wl writes none of these words in the legacy format.
	 * wlc_ucode_txant_set returns before shm 0x188 / 0x22 (0x15f9c4) and
	 * wlc_beacon_phytxctl uses shm 0xcc/0xce/0xd0 instead of 0x54
	 * (0x118f98..0x119008). What ucode 907 keeps at 0x22, 0x54 and 0x188
	 * is not decoded, so leave them as the initvals set them.
	 */
	if (dev->dev->core_rev >= 40)
		return;

	ctl |= B43_TXH_PHY_ENC_CCK;
	ctl |= B43_TXH_PHY_ANT01AUTO;
	ctl |= B43_TXH_PHY_TXPWR;

	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_BEACPHYCTL, ctl);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_ACKCTSPHYCTL, ctl);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRPHYCTL, ctl);
}

/* Set the TX-Antenna for management frames sent by firmware. */
static void b43_mgmtframe_txantenna(struct b43_wldev *dev, int antenna)
{
	u16 ant;
	u16 tmp;

	/* See b43_set_phytxctl_defaults: not on core rev >= 40 (0x15f9c4) */
	if (dev->dev->core_rev >= 40)
		return;

	ant = b43_antenna_to_phyctl(antenna);

	/* For ACK/CTS */
	tmp = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_ACKCTSPHYCTL);
	tmp = (tmp & ~B43_TXH_PHY_ANT) | ant;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_ACKCTSPHYCTL, tmp);
	/* For Probe Resposes */
	tmp = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_PRPHYCTL);
	tmp = (tmp & ~B43_TXH_PHY_ANT) | ant;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRPHYCTL, tmp);
}

/* This is the opposite of b43_chip_init() */
static void b43_chip_exit(struct b43_wldev *dev)
{
	b43_phy_exit(dev);
	b43_gpio_cleanup(dev);
	/* firmware is released later */
}

/* Initialize the chip
 * https://bcm-specs.sipsolutions.net/ChipInit
 */
static int b43_chip_init(struct b43_wldev *dev)
{
	struct b43_phy *phy = &dev->phy;
	int err;
	u32 macctl;
	u16 value16;

	/* Initialize the MAC control */
	macctl = B43_MACCTL_IHR_ENABLED | B43_MACCTL_SHM_ENABLED;
	if (dev->phy.gmode)
		macctl |= B43_MACCTL_GMODE;
	macctl |= B43_MACCTL_INFRA;
	/*
	 * Core rev >= 40: wl's MAC control from here to the end of
	 * wlc_bmac_init. wlc_bmac_init sets it to WAKE | IHR_EN | PSM_JMP_0
	 * (mctrl(~0, 0x04000404), 0x162620..0x162630) before the ucode
	 * download (0x16270c), starts the PSM with WAKE | INFRA | IHR_EN |
	 * PSM_RUN (wlc_bmac_wowlucode_start, mctrl(~0, 0x04020402),
	 * 0x15dbf8..0x15dc08) and keeps WAKE set through the initvals and the
	 * buffer memory setup (0x162b14..0x162e60). No SHM_EN, no GMODE.
	 * b43 started the ucode without AWAKE, and the first b43 path that
	 * sets it (b43_power_saving_ctl_bits) runs after b43_chip_init; on
	 * hardware the buffer memory commands at 0x530 then never completed.
	 */
	if (dev->dev->core_rev >= 40)
		macctl = B43_MACCTL_IHR_ENABLED | B43_MACCTL_AWAKE |
			 B43_MACCTL_INFRA;
	b43_write32(dev, B43_MMIO_MACCTL, macctl);

	/* wl's si_pmu_rfldo(sih, 1) precedes its ucode download */
	b43_ac_rfldo_on(dev);

	err = b43_upload_microcode(dev);
	if (err)
		goto out;	/* firmware is released later */

	err = b43_gpio_init(dev);
	if (err)
		goto out;	/* firmware is released later */
	/* after b43's GPIO handoff, wl's si_gpiocontrol(0xffff, 0) */
	b43_ac_gpio_to_cc(dev);

	err = b43_upload_initvals(dev);
	if (err)
		goto err_gpio_clean;

	err = b43_upload_initvals_band(dev);
	if (err)
		goto err_gpio_clean;

	/* wl's order: right after the initvals (wlc_bmac_init 0x162b14) */
	b43_bmc_init_rev40(dev);
	if (dev->dev->core_rev == 56)
		b43_ac56_dumpdev = dev;

	/* Turn the Analog on and initialize the PHY. */
	phy->ops->switch_analog(dev, 1);
	/* wl 0x50048, PMU regctl 0 bits 20..24 (acphy_otp16) */
	b43_ac_regctl0_paldo(dev);
	err = b43_phy_init(dev);
	if (err)
		goto err_gpio_clean;

	/* Disable Interference Mitigation. */
	if (phy->ops->interf_mitigation)
		phy->ops->interf_mitigation(dev, B43_INTERFMODE_NONE);

	/* Select the antennae */
	if (phy->ops->set_rx_antenna)
		phy->ops->set_rx_antenna(dev, dev->rx_antenna);
	b43_mgmtframe_txantenna(dev, dev->tx_antenna);

	if (phy->type == B43_PHYTYPE_B) {
		value16 = b43_read16(dev, 0x005E);
		value16 |= 0x0004;
		b43_write16(dev, 0x005E, value16);
	}
	/*
	 * Receive-lazy interrupt control of DMA engine 0: one frame per RX
	 * interrupt. acphy_rxlazy sets another frame count and timeout
	 * on core rev >= 40 (the RX ring holds B43_RXRING_SLOTS frames).
	 */
	if (dev->dev->core_rev >= 40 && b43_modparam_acphy_rxlazy)
		b43_write32(dev, B43_MMIO_INTRCVLAZY0, b43_modparam_acphy_rxlazy);
	else
		b43_write32(dev, B43_MMIO_INTRCVLAZY0, 0x01000000);
	if (dev->dev->core_rev < 5)
		b43_write32(dev, 0x010C, 0x01000000);

	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_INFRA, 0);
	b43_maskset32(dev, B43_MMIO_MACCTL, ~0, B43_MACCTL_INFRA);

	/* Probe Response Timeout value */
	/* FIXME: Default to 0, has to be set by ioctl probably... :-/ */
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRMAXTIME, 0);

	/* Initially set the wireless operation mode. */
	b43_adjust_opmode(dev);

	if (dev->dev->core_rev < 3) {
		b43_write16(dev, 0x060E, 0x0000);
		b43_write16(dev, 0x0610, 0x8000);
		b43_write16(dev, 0x0604, 0x0000);
		b43_write16(dev, 0x0606, 0x0200);
	} else {
		b43_write32(dev, 0x0188, 0x80000000);
		b43_write32(dev, 0x018C, 0x02000000);
	}
	b43_write32(dev, B43_MMIO_GEN_IRQ_REASON, 0x00004000);
	b43_write32(dev, B43_MMIO_DMA0_IRQ_MASK, 0x0001FC00);
	b43_write32(dev, B43_MMIO_DMA1_IRQ_MASK, 0x0000DC00);
	b43_write32(dev, B43_MMIO_DMA2_IRQ_MASK, 0x0000DC00);
	b43_write32(dev, B43_MMIO_DMA3_IRQ_MASK, 0x0001DC00);
	b43_write32(dev, B43_MMIO_DMA4_IRQ_MASK, 0x0000DC00);
	b43_write32(dev, B43_MMIO_DMA5_IRQ_MASK, 0x0000DC00);

	b43_mac_phy_clock_set(dev, true);

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		/* FIXME: 0xE74 is quite common, but should be read from CC */
		b43_write16(dev, B43_MMIO_POWERUP_DELAY, 0xE74);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		b43_write16(dev, B43_MMIO_POWERUP_DELAY,
			    dev->dev->sdev->bus->chipco.fast_pwrup_delay);
		break;
#endif
	}

	err = 0;
	b43dbg(dev->wl, "Chip initialized\n");
out:
	return err;

err_gpio_clean:
	b43_gpio_cleanup(dev);
	return err;
}

static void b43_periodic_every60sec(struct b43_wldev *dev)
{
	const struct b43_phy_operations *ops = dev->phy.ops;

	if (ops->pwork_60sec)
		ops->pwork_60sec(dev);

	/* Force check the TX power emission now. */
	b43_phy_txpower_check(dev, B43_TXPWR_IGNORE_TIME);
}

static void b43_periodic_every30sec(struct b43_wldev *dev)
{
	/* Update device statistics. */
	b43_calculate_link_quality(dev);
}

static void b43_periodic_every15sec(struct b43_wldev *dev)
{
	struct b43_phy *phy = &dev->phy;
	u16 wdr;

	if (dev->fw.opensource) {
		/* Check if the firmware is still alive.
		 * It will reset the watchdog counter to 0 in its idle loop. */
		wdr = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_WATCHDOG_REG);
		if (unlikely(wdr)) {
			b43err(dev->wl, "Firmware watchdog: The firmware died!\n");
			b43_controller_restart(dev, "Firmware watchdog");
			return;
		} else {
			b43_shm_write16(dev, B43_SHM_SCRATCH,
					B43_WATCHDOG_REG, 1);
		}
	}

	if (phy->ops->pwork_15sec)
		phy->ops->pwork_15sec(dev);

	atomic_set(&phy->txerr_cnt, B43_PHY_TX_BADNESS_LIMIT);
	wmb();

#if B43_DEBUG
	if (b43_debug(dev, B43_DBG_VERBOSESTATS)) {
		unsigned int i;

		b43dbg(dev->wl, "Stats: %7u IRQs/sec, %7u TX/sec, %7u RX/sec\n",
		       dev->irq_count / 15,
		       dev->tx_count / 15,
		       dev->rx_count / 15);
		dev->irq_count = 0;
		dev->tx_count = 0;
		dev->rx_count = 0;
		for (i = 0; i < ARRAY_SIZE(dev->irq_bit_count); i++) {
			if (dev->irq_bit_count[i]) {
				b43dbg(dev->wl, "Stats: %7u IRQ-%02u/sec (0x%08X)\n",
				       dev->irq_bit_count[i] / 15, i, (1 << i));
				dev->irq_bit_count[i] = 0;
			}
		}
	}
#endif
}

static void do_periodic_work(struct b43_wldev *dev)
{
	unsigned int state;

	state = dev->periodic_state;
	if (state % 4 == 0)
		b43_periodic_every60sec(dev);
	if (state % 2 == 0)
		b43_periodic_every30sec(dev);
	b43_periodic_every15sec(dev);
}

/* Periodic work locking policy:
 * 	The whole periodic work handler is protected by
 * 	wl->mutex. If another lock is needed somewhere in the
 * 	pwork callchain, it's acquired in-place, where it's needed.
 */
static void b43_periodic_work_handler(struct work_struct *work)
{
	struct b43_wldev *dev = container_of(work, struct b43_wldev,
					     periodic_work.work);
	struct b43_wl *wl = dev->wl;
	unsigned long delay;

	mutex_lock(&wl->mutex);

	if (unlikely(b43_status(dev) != B43_STAT_STARTED))
		goto out;
	if (b43_debug(dev, B43_DBG_PWORK_STOP))
		goto out_requeue;

	do_periodic_work(dev);

	dev->periodic_state++;
out_requeue:
	if (b43_debug(dev, B43_DBG_PWORK_FAST))
		delay = msecs_to_jiffies(50);
	else
		delay = round_jiffies_relative(HZ * 15);
	ieee80211_queue_delayed_work(wl->hw, &dev->periodic_work, delay);
out:
	mutex_unlock(&wl->mutex);
}

static void b43_periodic_tasks_setup(struct b43_wldev *dev)
{
	struct delayed_work *work = &dev->periodic_work;

	dev->periodic_state = 0;
	INIT_DELAYED_WORK(work, b43_periodic_work_handler);
	ieee80211_queue_delayed_work(dev->wl->hw, work, 0);
}

/* Check if communication with the device works correctly. */
static int b43_validate_chipaccess(struct b43_wldev *dev)
{
	u32 v, backup0, backup4;

	backup0 = b43_shm_read32(dev, B43_SHM_SHARED, 0);
	backup4 = b43_shm_read32(dev, B43_SHM_SHARED, 4);

	/* Check for read/write and endianness problems. */
	b43_shm_write32(dev, B43_SHM_SHARED, 0, 0x55AAAA55);
	if (b43_shm_read32(dev, B43_SHM_SHARED, 0) != 0x55AAAA55)
		goto error;
	b43_shm_write32(dev, B43_SHM_SHARED, 0, 0xAA5555AA);
	if (b43_shm_read32(dev, B43_SHM_SHARED, 0) != 0xAA5555AA)
		goto error;

	/* Check if unaligned 32bit SHM_SHARED access works properly.
	 * However, don't bail out on failure, because it's noncritical. */
	b43_shm_write16(dev, B43_SHM_SHARED, 0, 0x1122);
	b43_shm_write16(dev, B43_SHM_SHARED, 2, 0x3344);
	b43_shm_write16(dev, B43_SHM_SHARED, 4, 0x5566);
	b43_shm_write16(dev, B43_SHM_SHARED, 6, 0x7788);
	if (b43_shm_read32(dev, B43_SHM_SHARED, 2) != 0x55663344)
		b43warn(dev->wl, "Unaligned 32bit SHM read access is broken\n");
	b43_shm_write32(dev, B43_SHM_SHARED, 2, 0xAABBCCDD);
	if (b43_shm_read16(dev, B43_SHM_SHARED, 0) != 0x1122 ||
	    b43_shm_read16(dev, B43_SHM_SHARED, 2) != 0xCCDD ||
	    b43_shm_read16(dev, B43_SHM_SHARED, 4) != 0xAABB ||
	    b43_shm_read16(dev, B43_SHM_SHARED, 6) != 0x7788)
		b43warn(dev->wl, "Unaligned 32bit SHM write access is broken\n");

	b43_shm_write32(dev, B43_SHM_SHARED, 0, backup0);
	b43_shm_write32(dev, B43_SHM_SHARED, 4, backup4);

	if ((dev->dev->core_rev >= 3) && (dev->dev->core_rev <= 10)) {
		/* The 32bit register shadows the two 16bit registers
		 * with update sideeffects. Validate this. */
		b43_write16(dev, B43_MMIO_TSF_CFP_START, 0xAAAA);
		b43_write32(dev, B43_MMIO_TSF_CFP_START, 0xCCCCBBBB);
		if (b43_read16(dev, B43_MMIO_TSF_CFP_START_LOW) != 0xBBBB)
			goto error;
		if (b43_read16(dev, B43_MMIO_TSF_CFP_START_HIGH) != 0xCCCC)
			goto error;
	}
	b43_write32(dev, B43_MMIO_TSF_CFP_START, 0);

	v = b43_read32(dev, B43_MMIO_MACCTL);
	v |= B43_MACCTL_GMODE;
	if (v != (B43_MACCTL_GMODE | B43_MACCTL_IHR_ENABLED))
		goto error;

	return 0;
error:
	b43err(dev->wl, "Failed to validate the chipaccess\n");
	return -ENODEV;
}

/*
 * acphy_psfix bit 8, one AMT entry per associated station on core rev
 * >= 40 (see xmit.h). Entries 0..61 are free: b43 keeps the BSSID in 62 and
 * our MAC in 63 (wlc_set_bssid 0x115b78.., wlc_set_mac 0x115a78..). wl
 * writes an entry through wlc_bmac_write_amt 0x15c964 (b43_amt_write
 * mirrors it) and clears it with attribute 0; the attribute is
 * acphy_psamt. The index lives in the station's drv_priv (b43_ac_sta.amt_idx
 * + 1). Only AP interfaces get entries.
 */
#define B43_AC_AMT_STA_MAX	62
static DECLARE_BITMAP(b43_ac_amt_used, B43_AC_AMT_STA_MAX);

/*
 * a station's AMT entry is shared by acphy_psfix bit 8 and its
 * pairwise hardware key (B43_AC_AMT_USER_*): the first user allocates the
 * entry, every user writes its attribute (b43_amt_write ORs the old one in
 * when the valid bit is set, as wlc_write_amt 0x115514..0x115540), the
 * last user clears it. A key needs an entry below B43_AC_HWKEY_AMT_MAX
 * (slot 4 + entry < wl's 61 keys); the bitmap fills from 0, so only a
 * crowd of stations pushes one above that. Returns the entry or -ENOSPC.
 * wl->mutex held; the station has a b43_ac_sta (hw->sta_data_size).
 */
static int b43_ac_amt_sta_get(struct b43_wldev *dev,
			      struct ieee80211_sta *sta, u8 user, u16 attr)
{
	struct b43_ac_sta *acs = (struct b43_ac_sta *)sta->drv_priv;
	unsigned int idx, max;
	u32 lo, hi;

	max = user == B43_AC_AMT_USER_KEY ? B43_AC_HWKEY_AMT_MAX :
					    B43_AC_AMT_STA_MAX;
	if (acs->amt_idx) {
		idx = acs->amt_idx - 1;
		if (idx >= max)
			return -ENOSPC;
	} else {
		idx = find_first_zero_bit(b43_ac_amt_used, max);
		if (idx >= max)
			return -ENOSPC;
		set_bit(idx, b43_ac_amt_used);
		acs->amt_idx = idx + 1;
	}
	acs->amt_users |= user;
	b43_amt_write(dev, idx, sta->addr, attr);
	/* Read the entry back the way wlc_bmac_read_amt 0x15b9fc does */
	b43_write32(dev, B43_MMIO_SHM_CONTROL, (B43_SHM_AMT << 16) | (idx * 2));
	lo = b43_read32(dev, B43_MMIO_SHM_DATA);
	hi = b43_read32(dev, B43_MMIO_SHM_DATA);
	return idx;
}

/* Drop users of the station's entry; the last one clears it (wl->mutex held). */
static void b43_ac_amt_sta_put(struct b43_wldev *dev,
			       struct ieee80211_sta *sta, u8 users)
{
	struct b43_ac_sta *acs = (struct b43_ac_sta *)sta->drv_priv;
	static const u8 zero_addr[ETH_ALEN] = { 0 };
	unsigned int idx;

	acs->amt_users &= ~users;
	if (!acs->amt_idx || acs->amt_users)
		return;
	idx = acs->amt_idx - 1;
	clear_bit(idx, b43_ac_amt_used);
	acs->amt_idx = 0;
	if (dev && b43_status(dev) >= B43_STAT_INITIALIZED) {
		b43_amt_write(dev, idx, zero_addr, 0);
	}
}

/*
 * acphy_hwcrypto, ucode 907's key table (b43.h B43_AC_HWKEY_*,
 * xmit.h). A slot is its index/algorithm word at 0x5e0 + 2 * slot and 16
 * bytes of material at ktp + 16 * slot (wlc_key_hw_init 0x1730a4..
 * 0x17310c). The material goes in before the word that enables the slot
 * and the word is cleared before the material, so the ucode never works
 * with half a key; wl writes them the other way round (0x1730bc, 0x1730ec)
 * and clears a slot to (slot << 4) | 0 where wlc_key_hw_init_all
 * (0x1745a8) writes 0: both mean "off".
 */
static void b43_ac_hwkey_slot_write(struct b43_wldev *dev, unsigned int slot,
				    u8 algo_hw, const u8 *key)
{
	unsigned int i;
	u16 value;

	if (!algo_hw)
		b43_shm_write16(dev, B43_SHM_SHARED,
				B43_SHM_SH_AC_KEYIDXBLOCK + slot * 2, 0);
	for (i = 0; i < B43_AC_HWKEY_SIZE; i += 2) {
		value = key ? key[i] | (u16)key[i + 1] << 8 : 0;
		b43_shm_write16(dev, B43_SHM_SHARED,
				dev->ktp + slot * B43_AC_HWKEY_SIZE + i, value);
	}
	if (algo_hw)
		b43_shm_write16(dev, B43_SHM_SHARED,
				B43_SHM_SH_AC_KEYIDXBLOCK + slot * 2,
				(slot << 4) | algo_hw);
}

static bool b43_ac_hwkey_sta_ok(struct b43_wldev *dev,
				struct ieee80211_sta *sta)
{
	return sta && dev->wl->hw->sta_data_size >= sizeof(struct b43_ac_sta);
}

/* USEDEFKEYS for WEP default keys, as wlc_key_defkeyflag 0x172df4 and the
 * legacy path; off with any other key in the table. */
static void b43_ac_hwkey_defkeys(struct b43_wldev *dev)
{
	unsigned int slot;
	bool wep = false;

	for (slot = 0; slot < B43_AC_HWKEY_GROUP_SLOTS; slot++)
		if (dev->ac_hwkey[slot].keyconf &&
		    dev->ac_hwkey[slot].algo_hw != B43_AC_WSEC_ALGO_AES)
			wep = true;
	for (slot = B43_AC_HWKEY_GROUP_SLOTS; slot < B43_AC_HWKEY_SLOTS; slot++)
		if (dev->ac_hwkey[slot].keyconf)
			wep = false;
	if (wep)
		b43_hf_write(dev, b43_hf_read(dev) | B43_HF_USEDEFKEYS);
	else
		b43_hf_write(dev, b43_hf_read(dev) & ~B43_HF_USEDEFKEYS);
}

static int b43_ac_hwkey_set(struct b43_wldev *dev, struct ieee80211_vif *vif,
			    struct ieee80211_sta *sta,
			    struct ieee80211_key_conf *key)
{
	static const u8 zero_addr[ETH_ALEN] = { 0 };
	bool pairwise = !!(key->flags & IEEE80211_KEY_FLAG_PAIRWISE);
	struct b43_ac_hwkey *hk;
	u8 algo_hw, buf[B43_AC_HWKEY_SIZE] = { 0 };
	unsigned int i;
	int slot, amt = -1;

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_CCMP:
		if (!b43_ac_hwcrypto(dev, B43_AC_HWCRYPTO_CCMP))
			return -EOPNOTSUPP;
		algo_hw = B43_AC_WSEC_ALGO_AES;
		break;
	case WLAN_CIPHER_SUITE_WEP40:
		if (!b43_ac_hwcrypto(dev, B43_AC_HWCRYPTO_WEP))
			return -EOPNOTSUPP;
		algo_hw = B43_AC_WSEC_ALGO_WEP1;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		if (!b43_ac_hwcrypto(dev, B43_AC_HWCRYPTO_WEP))
			return -EOPNOTSUPP;
		algo_hw = B43_AC_WSEC_ALGO_WEP128;
		break;
	default:
		/* TKIP (software in wl too), GCMP, the CMAC group key of MFP */
		return -EOPNOTSUPP;
	}
	if (key->keylen > B43_AC_HWKEY_SIZE || key->keyidx < 0 ||
	    key->keyidx >= B43_AC_HWKEY_GROUP_SLOTS)
		return -EINVAL;
	if (pairwise) {
		if (!b43_ac_hwkey_sta_ok(dev, sta))
			return -EOPNOTSUPP;
		amt = b43_ac_amt_sta_get(dev, sta, B43_AC_AMT_USER_KEY,
					 B43_AMT_ATTR_VALID | B43_AMT_ATTR_A2);
		if (amt < 0) {
			return -ENOSPC;
		}
		slot = B43_AC_HWKEY_GROUP_SLOTS + amt;
	} else {
		slot = key->keyidx;
	}
	/*
	 * The same key again (a replay) or the slot in use (a PTK rekey:
	 * mac80211 installs the new key before it disables the old one, and
	 * the old one's DISABLE_KEY then finds its slot taken, below): clear
	 * first. A stale pairwise record at another slot leaves its own AMT
	 * entry behind; cleared only if no station holds it (the station's
	 * current entry is the one b43_ac_amt_sta_get just returned).
	 */
	for (i = 0; i < B43_AC_HWKEY_SLOTS; i++) {
		hk = &dev->ac_hwkey[i];
		if (!hk->keyconf || (hk->keyconf != key && i != slot))
			continue;
		if (hk->pairwise && i != slot &&
		    !test_bit(i - B43_AC_HWKEY_GROUP_SLOTS, b43_ac_amt_used))
			b43_amt_write(dev, i - B43_AC_HWKEY_GROUP_SLOTS,
				      zero_addr, 0);
		b43_ac_hwkey_slot_write(dev, i, 0, NULL);
		memset(hk, 0, sizeof(*hk));
	}
	memcpy(buf, key->key, key->keylen);
	b43_ac_hwkey_slot_write(dev, slot, algo_hw, buf);
	hk = &dev->ac_hwkey[slot];
	hk->keyconf = key;
	hk->algo_hw = algo_hw;
	hk->pairwise = pairwise;
	memcpy(hk->mac, sta ? sta->addr : zero_addr, ETH_ALEN);
	memcpy(hk->key, buf, B43_AC_HWKEY_SIZE);
	key->hw_key_idx = slot;
	/* The host writes the IV into the frame, as wl's does (xmit.h) */
	key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;
	if (algo_hw == B43_AC_WSEC_ALGO_AES)
		key->flags |= IEEE80211_KEY_FLAG_SW_MGMT_TX;
	b43_ac_hwkey_defkeys(dev);
	return 0;
}

/* The core may be down (mac80211 drops keys around b43_op_stop): the
 * record always goes, the hardware only while it is up. */
static int b43_ac_hwkey_disable(struct b43_wldev *dev,
				struct ieee80211_sta *sta,
				struct ieee80211_key_conf *key)
{
	unsigned int slot = key->hw_key_idx;
	struct b43_ac_hwkey *hk;

	if (slot >= B43_AC_HWKEY_SLOTS || dev->ac_hwkey[slot].keyconf != key) {
		return 0;
	}
	hk = &dev->ac_hwkey[slot];
	/* wl's order: the AMT entry first (0x172f04..0x172f2c), then the slot */
	if (hk->pairwise && b43_ac_hwkey_sta_ok(dev, sta))
		b43_ac_amt_sta_put(dev, sta, B43_AC_AMT_USER_KEY);
	if (b43_status(dev) >= B43_STAT_INITIALIZED) {
		b43_ac_hwkey_slot_write(dev, slot, 0, NULL);
		b43_ac_hwkey_defkeys(dev);
	}
	memset(hk, 0, sizeof(*hk));
	return 0;
}

static int b43_ac_hwkey_op(struct b43_wldev *dev, enum set_key_cmd cmd,
			   struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			   struct ieee80211_key_conf *key)
{
	switch (cmd) {
	case SET_KEY:
		return b43_ac_hwkey_set(dev, vif, sta, key);
	case DISABLE_KEY:
		return b43_ac_hwkey_disable(dev, sta, key);
	default:
		B43_WARN_ON(1);
		return -EINVAL;
	}
}

/*
 * from b43_security_init on the AC path. A clean table (every slot
 * and index word, and the AMT rows a key may use that no station holds:
 * wlc_key_hw_init_all 0x174594..0x1745ec), then the keys mac80211 still
 * holds, because a controller restart re-inits the core without
 * telling mac80211 (xmit.c, acphy_htdead) and ucode 907's key material sits
 * above the 4 KiB b43_upload_microcode zeroes (ktp 0x10f4 on 907).
 */
static void b43_ac_hwcrypto_core_init(struct b43_wldev *dev)
{
	static const u8 zero_addr[ETH_ALEN] = { 0 };
	struct b43_ac_hwkey *hk;
	unsigned int slot, n = 0;
	u16 attr;

	if (!b43_ac_hwcrypto(dev, B43_AC_HWCRYPTO_CCMP | B43_AC_HWCRYPTO_WEP))
		return;
	for (slot = 0; slot < B43_AC_HWKEY_SLOTS; slot++)
		b43_ac_hwkey_slot_write(dev, slot, 0, NULL);
	for (slot = B43_AC_HWKEY_SLOTS; slot < B43_AC_KEYIDXBLOCK_WORDS; slot++)
		b43_shm_write16(dev, B43_SHM_SHARED,
				B43_SHM_SH_AC_KEYIDXBLOCK + slot * 2, 0);
	for (slot = 0; slot < B43_AC_HWKEY_AMT_MAX; slot++)
		if (!test_bit(slot, b43_ac_amt_used))
			b43_amt_write(dev, slot, zero_addr, 0);
	dev->ac_hwcrypto_rxlogs = 0;
	attr = B43_AMT_ATTR_VALID | B43_AMT_ATTR_A2;
	if (b43_ac_psfix(dev, B43_AC_PSFIX_AMT))
		attr |= b43_modparam_acphy_psamt;
	for (slot = 0; slot < B43_AC_HWKEY_SLOTS; slot++) {
		hk = &dev->ac_hwkey[slot];
		if (!hk->keyconf)
			continue;
		b43_ac_hwkey_slot_write(dev, slot, hk->algo_hw, hk->key);
		if (hk->pairwise)
			b43_amt_write(dev, slot - B43_AC_HWKEY_GROUP_SLOTS,
				      hk->mac, attr);
		n++;
	}
	b43_ac_hwkey_defkeys(dev);
}

static void b43_security_init(struct b43_wldev *dev)
{
	dev->ktp = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_KTP);
	/* KTP is a word address, but we address SHM bytewise.
	 * So multiply by two.
	 */
	dev->ktp *= 2;
	/*
	 * Core rev >= 40 with the AC TX header: ucode 907's shared memory is
	 * laid out differently. b43's key index block B43_SHM_SH_KEYIDXBLOCK
	 * (byte 0x5d4, "v4 firmware") is wl's TX core mask block there:
	 * wlc_attach sets the word base 0x2ea for core rev > 39
	 * (0x13d180..0x13d184) and wlc_stf_txcore_shmem_write writes byte
	 * offsets (0x2ea + i) * 2 = 0x5d4.. (0x1911e8..0x191248). Clearing
	 * the keys wrote (kidx << 4) | ALGO_NONE from 0x5d4 upward, after the TX
	 * enable had written the core masks: on hardware (2026-10-06 15:22 UTC)
	 * 0x5d4..0x5da read 0x0000 0x0010 0x0020 0x0030 and the ucode's own
	 * frames went out with PHY control 0x0405 (no core), a PHY TX error
	 * each. b43_op_set_key already refuses keys on the AC header,
	 * so there is nothing to clear; the ucode 907 key table layout is not
	 * decoded. wl writes MMIO 0x43c (RCMTA count) on no core rev > 39
	 * path, so leave it too.
	 */
	if (b43_txhdr_is_ac(dev) && dev->dev->core_rev >= 40) {
		/* ucode 907's own table, when acphy_hwcrypto asks */
		b43_ac_hwcrypto_core_init(dev);
		return;
	}
	/* Number of RCMTA address slots */
	b43_write16(dev, B43_MMIO_RCMTA_COUNT, B43_NR_PAIRWISE_KEYS);
	/* Clear the key memory. */
	b43_clear_keys(dev);
}

#ifdef CPTCFG_B43_HWRNG
static int b43_rng_read(struct hwrng *rng, u32 *data)
{
	struct b43_wl *wl = (struct b43_wl *)rng->priv;
	struct b43_wldev *dev;
	int count = -ENODEV;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (likely(dev && b43_status(dev) >= B43_STAT_INITIALIZED)) {
		*data = b43_read16(dev, B43_MMIO_RNG);
		count = sizeof(u16);
	}
	mutex_unlock(&wl->mutex);

	return count;
}
#endif /* CPTCFG_B43_HWRNG */

static void b43_rng_exit(struct b43_wl *wl)
{
#ifdef CPTCFG_B43_HWRNG
	if (wl->rng_initialized)
		hwrng_unregister(&wl->rng);
#endif /* CPTCFG_B43_HWRNG */
}

static int b43_rng_init(struct b43_wl *wl)
{
	int err = 0;

#ifdef CPTCFG_B43_HWRNG
	snprintf(wl->rng_name, ARRAY_SIZE(wl->rng_name),
		 "%s_%s", KBUILD_MODNAME, wiphy_name(wl->hw->wiphy));
	wl->rng.name = wl->rng_name;
	wl->rng.data_read = b43_rng_read;
	wl->rng.priv = (unsigned long)wl;
	wl->rng_initialized = true;
	err = hwrng_register(&wl->rng);
	if (err) {
		wl->rng_initialized = false;
		b43err(wl, "Failed to register the random "
		       "number generator (%d)\n", err);
	}
#endif /* CPTCFG_B43_HWRNG */

	return err;
}

static void b43_tx_work(struct work_struct *work)
{
	struct b43_wl *wl = container_of(work, struct b43_wl, tx_work);
	struct b43_wldev *dev;
	struct sk_buff *skb;
	int queue_num;
	int err = 0;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (unlikely(!dev || b43_status(dev) < B43_STAT_STARTED)) {
		mutex_unlock(&wl->mutex);
		return;
	}

	for (queue_num = 0; queue_num < B43_QOS_QUEUE_NUM; queue_num++) {
		while (skb_queue_len(&wl->tx_queue[queue_num])) {
			skb = skb_dequeue(&wl->tx_queue[queue_num]);
			/*
			 * (acphy_ampdu bit 4): the frame behind this one,
			 * for the A-MPDU burst rules (xmit.c
			 * b43_ac_ampdu_txhdr, dma.c dma_tx_fragment).
			 */
			dev->ac_ampdu_next = b43_ac_ampdu(dev, B43_AC_AMPDU_BURST) ?
				skb_peek(&wl->tx_queue[queue_num]) : NULL;
			if (b43_using_pio_transfers(dev))
				err = b43_pio_tx(dev, skb);
			else
				err = b43_dma_tx(dev, skb);
			dev->ac_ampdu_next = NULL;
			if (err == -ENOSPC) {
				wl->tx_queue_stopped[queue_num] = true;
				b43_stop_queue(dev, queue_num);
				skb_queue_head(&wl->tx_queue[queue_num], skb);
				break;
			}
			if (unlikely(err))
				ieee80211_free_txskb(wl->hw, skb);
			err = 0;
		}
		/* a burst left unstarted by dma_tx_fragment, or its tail
		 * after an error, is started here */
		if (!b43_using_pio_transfers(dev))
			b43_dma_tx_poke_pending(dev);

		if (!err)
			wl->tx_queue_stopped[queue_num] = false;
	}

#if B43_DEBUG
	dev->tx_count++;
#endif
	mutex_unlock(&wl->mutex);
}

static void b43_op_tx(struct ieee80211_hw *hw,
		      struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	u16 skb_queue_mapping;

	if (unlikely(skb->len < 2 + 2 + 6)) {
		/* Too short, this can't be a valid frame. */
		ieee80211_free_txskb(hw, skb);
		return;
	}
	B43_WARN_ON(skb_shinfo(skb)->nr_frags);

	skb_queue_mapping = skb_get_queue_mapping(skb);
	skb_queue_tail(&wl->tx_queue[skb_queue_mapping], skb);
	if (!wl->tx_queue_stopped[skb_queue_mapping])
		ieee80211_queue_work(wl->hw, &wl->tx_work);
	else
		b43_stop_queue(wl->current_dev, skb_queue_mapping);
}

static void b43_qos_params_upload(struct b43_wldev *dev,
				  const struct ieee80211_tx_queue_params *p,
				  u16 shm_offset)
{
	u16 params[B43_NR_QOSPARAMS];
	int bslots, tmp;
	unsigned int i;

	if (!dev->qos_enabled)
		return;

	bslots = b43_read16(dev, B43_MMIO_RNG) & p->cw_min;

	memset(&params, 0, sizeof(params));

	params[B43_QOSPARAM_TXOP] = p->txop * 32;
	params[B43_QOSPARAM_CWMIN] = p->cw_min;
	params[B43_QOSPARAM_CWMAX] = p->cw_max;
	params[B43_QOSPARAM_CWCUR] = p->cw_min;
	params[B43_QOSPARAM_AIFS] = p->aifs;
	params[B43_QOSPARAM_BSLOTS] = bslots;
	params[B43_QOSPARAM_REGGAP] = bslots + p->aifs;

	for (i = 0; i < ARRAY_SIZE(params); i++) {
		if (i == B43_QOSPARAM_STATUS) {
			tmp = b43_shm_read16(dev, B43_SHM_SHARED,
					     shm_offset + (i * 2));
			/* Mark the parameters as updated. */
			tmp |= 0x100;
			b43_shm_write16(dev, B43_SHM_SHARED,
					shm_offset + (i * 2),
					tmp);
		} else {
			b43_shm_write16(dev, B43_SHM_SHARED,
					shm_offset + (i * 2),
					params[i]);
		}
	}
}

/* Mapping of mac80211 queue numbers to b43 QoS SHM offsets. */
static const u16 b43_qos_shm_offsets[] = {
	/* [mac80211-queue-nr] = SHM_OFFSET, */
	[0] = B43_QOS_VOICE,
	[1] = B43_QOS_VIDEO,
	[2] = B43_QOS_BESTEFFORT,
	[3] = B43_QOS_BACKGROUND,
};

/* Update all QOS parameters in hardware. */
static void b43_qos_upload_all(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	struct b43_qos_params *params;
	unsigned int i;

	if (!dev->qos_enabled)
		return;

	BUILD_BUG_ON(ARRAY_SIZE(b43_qos_shm_offsets) !=
		     ARRAY_SIZE(wl->qos_params));

	b43_mac_suspend(dev);
	for (i = 0; i < ARRAY_SIZE(wl->qos_params); i++) {
		params = &(wl->qos_params[i]);
		b43_qos_params_upload(dev, &(params->p),
				      b43_qos_shm_offsets[i]);
	}
	b43_mac_enable(dev);
}

static void b43_qos_clear(struct b43_wl *wl)
{
	struct b43_qos_params *params;
	unsigned int i;

	/* Initialize QoS parameters to sane defaults. */

	BUILD_BUG_ON(ARRAY_SIZE(b43_qos_shm_offsets) !=
		     ARRAY_SIZE(wl->qos_params));

	for (i = 0; i < ARRAY_SIZE(wl->qos_params); i++) {
		params = &(wl->qos_params[i]);

		switch (b43_qos_shm_offsets[i]) {
		case B43_QOS_VOICE:
			params->p.txop = 0;
			params->p.aifs = 2;
			params->p.cw_min = 0x0001;
			params->p.cw_max = 0x0001;
			break;
		case B43_QOS_VIDEO:
			params->p.txop = 0;
			params->p.aifs = 2;
			params->p.cw_min = 0x0001;
			params->p.cw_max = 0x0001;
			break;
		case B43_QOS_BESTEFFORT:
			params->p.txop = 0;
			params->p.aifs = 3;
			params->p.cw_min = 0x0001;
			params->p.cw_max = 0x03FF;
			break;
		case B43_QOS_BACKGROUND:
			params->p.txop = 0;
			params->p.aifs = 7;
			params->p.cw_min = 0x0001;
			params->p.cw_max = 0x03FF;
			break;
		default:
			B43_WARN_ON(1);
		}
	}
}

/* Initialize the core's QOS capabilities */
static void b43_qos_init(struct b43_wldev *dev)
{
	if (!dev->qos_enabled) {
		/* Disable QOS support. */
		b43_hf_write(dev, b43_hf_read(dev) & ~B43_HF_EDCF);
		b43_write16(dev, B43_MMIO_IFSCTL,
			    b43_read16(dev, B43_MMIO_IFSCTL)
			    & ~B43_MMIO_IFSCTL_USE_EDCF);
		b43dbg(dev->wl, "QoS disabled\n");
		return;
	}

	/* Upload the current QOS parameters. */
	b43_qos_upload_all(dev);

	/* Enable QOS support. */
	b43_hf_write(dev, b43_hf_read(dev) | B43_HF_EDCF);
	b43_write16(dev, B43_MMIO_IFSCTL,
		    b43_read16(dev, B43_MMIO_IFSCTL)
		    | B43_MMIO_IFSCTL_USE_EDCF);
	b43dbg(dev->wl, "QoS enabled\n");
}

static int b43_op_conf_tx(struct ieee80211_hw *hw,
			  struct ieee80211_vif *vif,
			  unsigned int link_id, u16 _queue,
			  const struct ieee80211_tx_queue_params *params)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;
	unsigned int queue = (unsigned int)_queue;
	int err = -ENODEV;

	if (queue >= ARRAY_SIZE(wl->qos_params)) {
		/* Queue not available or don't support setting
		 * params on this queue. Return success to not
		 * confuse mac80211. */
		return 0;
	}
	BUILD_BUG_ON(ARRAY_SIZE(b43_qos_shm_offsets) !=
		     ARRAY_SIZE(wl->qos_params));

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (unlikely(!dev || (b43_status(dev) < B43_STAT_INITIALIZED)))
		goto out_unlock;

	memcpy(&(wl->qos_params[queue].p), params, sizeof(*params));
	b43_mac_suspend(dev);
	b43_qos_params_upload(dev, &(wl->qos_params[queue].p),
			      b43_qos_shm_offsets[queue]);
	b43_mac_enable(dev);
	err = 0;

out_unlock:
	mutex_unlock(&wl->mutex);

	return err;
}

static int b43_op_get_stats(struct ieee80211_hw *hw,
			    struct ieee80211_low_level_stats *stats)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);

	mutex_lock(&wl->mutex);
	memcpy(stats, &wl->ieee_stats, sizeof(*stats));
	mutex_unlock(&wl->mutex);

	return 0;
}

static u64 b43_op_get_tsf(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;
	u64 tsf;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;

	if (dev && (b43_status(dev) >= B43_STAT_INITIALIZED))
		b43_tsf_read(dev, &tsf);
	else
		tsf = 0;

	mutex_unlock(&wl->mutex);

	return tsf;
}

static void b43_op_set_tsf(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif, u64 tsf)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;

	if (dev && (b43_status(dev) >= B43_STAT_INITIALIZED))
		b43_tsf_write(dev, tsf);

	mutex_unlock(&wl->mutex);
}

static const char *band_to_string(enum nl80211_band band)
{
	switch (band) {
	case NL80211_BAND_5GHZ:
		return "5";
	case NL80211_BAND_2GHZ:
		return "2.4";
	default:
		break;
	}
	B43_WARN_ON(1);
	return "";
}

/* Expects wl->mutex locked */
static int b43_switch_band(struct b43_wldev *dev,
			   struct ieee80211_channel *chan)
{
	struct b43_phy *phy = &dev->phy;
	bool gmode;
	u32 tmp;

	switch (chan->band) {
	case NL80211_BAND_5GHZ:
		gmode = false;
		break;
	case NL80211_BAND_2GHZ:
		gmode = true;
		break;
	default:
		B43_WARN_ON(1);
		return -EINVAL;
	}

	if (!((gmode && phy->supports_2ghz) ||
	      (!gmode && phy->supports_5ghz))) {
		b43err(dev->wl, "This device doesn't support %s-GHz band\n",
		       band_to_string(chan->band));
		return -ENODEV;
	}

	if (!!phy->gmode == !!gmode) {
		/* This device is already running. */
		return 0;
	}

	b43dbg(dev->wl, "Switching to %s GHz band\n",
	       band_to_string(chan->band));

	/* Some new devices don't need disabling radio for band switching */
	if (!(phy->type == B43_PHYTYPE_N && phy->rev >= 3))
		b43_software_rfkill(dev, true);

	phy->gmode = gmode;
	b43_phy_put_into_reset(dev);
	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOCTL);
		if (gmode)
			tmp |= B43_BCMA_IOCTL_GMODE;
		else
			tmp &= ~B43_BCMA_IOCTL_GMODE;
		bcma_awrite32(dev->dev->bdev, BCMA_IOCTL, tmp);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		tmp = ssb_read32(dev->dev->sdev, SSB_TMSLOW);
		if (gmode)
			tmp |= B43_TMSLOW_GMODE;
		else
			tmp &= ~B43_TMSLOW_GMODE;
		ssb_write32(dev->dev->sdev, SSB_TMSLOW, tmp);
		break;
#endif
	}
	b43_phy_take_out_of_reset(dev);

	b43_upload_initvals_band(dev);

	b43_phy_init(dev);

	return 0;
}

static void b43_set_beacon_listen_interval(struct b43_wldev *dev, u16 interval)
{
	interval = min_t(u16, interval, (u16)0xFF);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_BCN_LI, interval);
}

/* Write the short and long frame retry limit values. */
static void b43_set_retry_limits(struct b43_wldev *dev,
				 unsigned int short_retry,
				 unsigned int long_retry)
{
	/* The retry limit is a 4-bit counter. Enforce this to avoid overflowing
	 * the chip-internal counter. */
	short_retry = min(short_retry, (unsigned int)0xF);
	long_retry = min(long_retry, (unsigned int)0xF);

	b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_SRLIMIT,
			short_retry);
	b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_LRLIMIT,
			long_retry);
}

static int b43_op_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev = wl->current_dev;
	struct b43_phy *phy = &dev->phy;
	struct ieee80211_conf *conf = &hw->conf;
	int err = 0;

	mutex_lock(&wl->mutex);
	b43_mac_suspend(dev);

	if (changed & IEEE80211_CONF_CHANGE_LISTEN_INTERVAL)
		b43_set_beacon_listen_interval(dev, conf->listen_interval);

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		phy->chandef = &conf->chandef;
		phy->channel = conf->chandef.chan->hw_value;

		/* Switch the band (if necessary). */
		err = b43_switch_band(dev, conf->chandef.chan);
		if (err)
			goto out_mac_enable;

		/* Switch to the requested channel.
		 * The firmware takes care of races with the TX handler.
		 * The AC PHY reads the width and center from phy->chandef
		 * (b43_phy_ac_chanspec); mac80211 flags a width
		 * change as a channel change too.
		 */
		b43_switch_channel(dev, phy->channel);
	}

	if (changed & IEEE80211_CONF_CHANGE_RETRY_LIMITS)
		b43_set_retry_limits(dev, conf->short_frame_max_tx_count,
					  conf->long_frame_max_tx_count);
	changed &= ~IEEE80211_CONF_CHANGE_RETRY_LIMITS;
	if (!changed)
		goto out_mac_enable;

	dev->wl->radiotap_enabled = !!(conf->flags & IEEE80211_CONF_MONITOR);

	/* Adjust the desired TX power level. */
	if (conf->power_level != 0) {
		if (conf->power_level != phy->desired_txpower) {
			phy->desired_txpower = conf->power_level;
			b43_phy_txpower_check(dev, B43_TXPWR_IGNORE_TIME |
						   B43_TXPWR_IGNORE_TSSI);
		}
	}

	/* Antennas for RX and management frame TX. */
	b43_mgmtframe_txantenna(dev, dev->tx_antenna);
	if (phy->ops->set_rx_antenna)
		phy->ops->set_rx_antenna(dev, dev->rx_antenna);

	if (wl->radio_enabled != phy->radio_on) {
		if (wl->radio_enabled) {
			b43_software_rfkill(dev, false);
			b43info(dev->wl, "Radio turned on by software\n");
			if (!dev->radio_hw_enable) {
				b43info(dev->wl, "The hardware RF-kill button "
					"still turns the radio physically off. "
					"Press the button to turn it on.\n");
			}
		} else {
			b43_software_rfkill(dev, true);
			b43info(dev->wl, "Radio turned off by software\n");
		}
	}

out_mac_enable:
	b43_mac_enable(dev);
	mutex_unlock(&wl->mutex);

	return err;
}

static void b43_update_basic_rates(struct b43_wldev *dev, u32 brates)
{
	struct ieee80211_supported_band *sband =
		dev->wl->hw->wiphy->bands[b43_current_band(dev->wl)];
	const struct ieee80211_rate *rate;
	int i;
	u16 basic, direct, offset, basic_offset, rateptr;

	for (i = 0; i < sband->n_bitrates; i++) {
		rate = &sband->bitrates[i];

		if (b43_is_cck_rate(rate->hw_value)) {
			direct = B43_SHM_SH_CCKDIRECT;
			basic = B43_SHM_SH_CCKBASIC;
			offset = b43_plcp_get_ratecode_cck(rate->hw_value);
			offset &= 0xF;
		} else {
			direct = B43_SHM_SH_OFDMDIRECT;
			basic = B43_SHM_SH_OFDMBASIC;
			offset = b43_plcp_get_ratecode_ofdm(rate->hw_value);
			offset &= 0xF;
		}

		rate = ieee80211_get_response_rate(sband, brates, rate->bitrate);

		if (b43_is_cck_rate(rate->hw_value)) {
			basic_offset = b43_plcp_get_ratecode_cck(rate->hw_value);
			basic_offset &= 0xF;
		} else {
			basic_offset = b43_plcp_get_ratecode_ofdm(rate->hw_value);
			basic_offset &= 0xF;
		}

		/*
		 * Get the pointer that we need to point to
		 * from the direct map
		 */
		rateptr = b43_shm_read16(dev, B43_SHM_SHARED,
					 direct + 2 * basic_offset);
		/* and write it to the basic map */
		b43_shm_write16(dev, B43_SHM_SHARED, basic + 2 * offset,
				rateptr);
	}
}

static void b43_op_bss_info_changed(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    struct ieee80211_bss_conf *conf,
				    u64 changed)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);

	dev = wl->current_dev;
	if (!dev || b43_status(dev) < B43_STAT_STARTED)
		goto out_unlock_mutex;

	B43_WARN_ON(wl->vif != vif);

	if (changed & BSS_CHANGED_BSSID) {
		if (conf->bssid)
			memcpy(wl->bssid, conf->bssid, ETH_ALEN);
		else
			eth_zero_addr(wl->bssid);
	}

	if (b43_status(dev) >= B43_STAT_INITIALIZED) {
		if (changed & BSS_CHANGED_BEACON &&
		    (b43_is_mode(wl, NL80211_IFTYPE_AP) ||
		     b43_is_mode(wl, NL80211_IFTYPE_MESH_POINT) ||
		     b43_is_mode(wl, NL80211_IFTYPE_ADHOC)))
			b43_update_templates(wl);

		if (changed & BSS_CHANGED_BSSID)
			b43_write_mac_bssid_templates(dev);
	}

	b43_mac_suspend(dev);

	/* Update templates for AP/mesh mode. */
	if (changed & BSS_CHANGED_BEACON_INT &&
	    (b43_is_mode(wl, NL80211_IFTYPE_AP) ||
	     b43_is_mode(wl, NL80211_IFTYPE_MESH_POINT) ||
	     b43_is_mode(wl, NL80211_IFTYPE_ADHOC)) &&
	    conf->beacon_int)
		b43_set_beacon_int(dev, conf->beacon_int);

	if (changed & BSS_CHANGED_BASIC_RATES)
		b43_update_basic_rates(dev, conf->basic_rates);

	if (changed & BSS_CHANGED_ERP_SLOT) {
		if (conf->use_short_slot)
			b43_short_slot_timing_enable(dev);
		else
			b43_short_slot_timing_disable(dev);
	}

	b43_mac_enable(dev);
out_unlock_mutex:
	mutex_unlock(&wl->mutex);
}

static int b43_op_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			  struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			  struct ieee80211_key_conf *key)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;
	u8 algorithm;
	u8 index;
	int err;
	static const u8 bcast_addr[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

	if (modparam_nohwcrypt)
		return -ENOSPC; /* User disabled HW-crypto */

	if ((vif->type == NL80211_IFTYPE_ADHOC ||
	     vif->type == NL80211_IFTYPE_MESH_POINT) &&
	    (key->cipher == WLAN_CIPHER_SUITE_TKIP ||
	     key->cipher == WLAN_CIPHER_SUITE_CCMP) &&
	    !(key->flags & IEEE80211_KEY_FLAG_PAIRWISE)) {
		/*
		 * For now, disable hw crypto for the RSN IBSS group keys. This
		 * could be optimized in the future, but until that gets
		 * implemented, use of software crypto for group addressed
		 * frames is a acceptable to allow RSN IBSS to be used.
		 */
		return -EOPNOTSUPP;
	}

	mutex_lock(&wl->mutex);

	dev = wl->current_dev;
	/* a slot record outlives a core that is down; drop it anyway */
	if (dev && cmd == DISABLE_KEY && dev->dev->core_rev >= 40 &&
	    b43_status(dev) < B43_STAT_INITIALIZED) {
		err = b43_ac_hwkey_disable(dev, sta, key);
		mutex_unlock(&wl->mutex);
		return err;
	}
	err = -ENODEV;
	if (!dev || b43_status(dev) < B43_STAT_INITIALIZED)
		goto out_unlock;

	if (b43_txhdr_is_ac(dev)) {
		/*
		 * ucode 907's key table with acphy_hwcrypto (b43.h
		 * B43_AC_HWKEY_*, xmit.h); refused (software crypto, as
		 * before) without it. Nothing below applies to this ucode.
		 */
		if (!b43_ac_hwcrypto(dev, B43_AC_HWCRYPTO_CCMP |
					  B43_AC_HWCRYPTO_WEP))
			err = -EOPNOTSUPP;
		else
			err = b43_ac_hwkey_op(dev, cmd, vif, sta, key);
		mutex_unlock(&wl->mutex);
		return err;
	}

	if (dev->fw.pcm_request_failed || !dev->hwcrypto_enabled) {
		/* We don't have firmware for the crypto engine.
		 * Must use software-crypto. */
		err = -EOPNOTSUPP;
		goto out_unlock;
	}

	err = -EINVAL;
	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		algorithm = B43_SEC_ALGO_WEP40;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		algorithm = B43_SEC_ALGO_WEP104;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		algorithm = B43_SEC_ALGO_TKIP;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		algorithm = B43_SEC_ALGO_AES;
		break;
	default:
		B43_WARN_ON(1);
		goto out_unlock;
	}
	index = (u8) (key->keyidx);
	if (index > 3)
		goto out_unlock;

	switch (cmd) {
	case SET_KEY:
		if (algorithm == B43_SEC_ALGO_TKIP &&
		    (!(key->flags & IEEE80211_KEY_FLAG_PAIRWISE) ||
		    !modparam_hwtkip)) {
			/* We support only pairwise key */
			err = -EOPNOTSUPP;
			goto out_unlock;
		}

		if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE) {
			if (WARN_ON(!sta)) {
				err = -EOPNOTSUPP;
				goto out_unlock;
			}
			/* Pairwise key with an assigned MAC address. */
			err = b43_key_write(dev, -1, algorithm,
					    key->key, key->keylen,
					    sta->addr, key);
		} else {
			/* Group key */
			err = b43_key_write(dev, index, algorithm,
					    key->key, key->keylen, NULL, key);
		}
		if (err)
			goto out_unlock;

		if (algorithm == B43_SEC_ALGO_WEP40 ||
		    algorithm == B43_SEC_ALGO_WEP104) {
			b43_hf_write(dev, b43_hf_read(dev) | B43_HF_USEDEFKEYS);
		} else {
			b43_hf_write(dev,
				     b43_hf_read(dev) & ~B43_HF_USEDEFKEYS);
		}
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;
		if (algorithm == B43_SEC_ALGO_TKIP)
			key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIC;
		break;
	case DISABLE_KEY: {
		err = b43_key_clear(dev, key->hw_key_idx);
		if (err)
			goto out_unlock;
		break;
	}
	default:
		B43_WARN_ON(1);
	}

out_unlock:
	if (!err) {
		b43dbg(wl, "%s hardware based encryption for keyidx: %d, "
		       "mac: %pM\n",
		       cmd == SET_KEY ? "Using" : "Disabling", key->keyidx,
		       sta ? sta->addr : bcast_addr);
		b43_dump_keymemory(dev);
	}
	mutex_unlock(&wl->mutex);

	return err;
}

static void b43_op_configure_filter(struct ieee80211_hw *hw,
				    unsigned int changed, unsigned int *fflags,
				    u64 multicast)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (!dev) {
		*fflags = 0;
		goto out_unlock;
	}

	*fflags &= FIF_ALLMULTI |
		  FIF_FCSFAIL |
		  FIF_PLCPFAIL |
		  FIF_CONTROL |
		  FIF_OTHER_BSS |
		  FIF_BCN_PRBRESP_PROMISC;

	changed &= FIF_ALLMULTI |
		   FIF_FCSFAIL |
		   FIF_PLCPFAIL |
		   FIF_CONTROL |
		   FIF_OTHER_BSS |
		   FIF_BCN_PRBRESP_PROMISC;

	wl->filter_flags = *fflags;

	if (changed && b43_status(dev) >= B43_STAT_INITIALIZED)
		b43_adjust_opmode(dev);

out_unlock:
	mutex_unlock(&wl->mutex);
}

/* Locking: wl->mutex
 * Returns the current dev. This might be different from the passed in dev,
 * because the core might be gone away while we unlocked the mutex. */
static struct b43_wldev * b43_wireless_core_stop(struct b43_wldev *dev)
{
	struct b43_wl *wl;
	struct b43_wldev *orig_dev;
	u32 mask;
	int queue_num;

	if (!dev)
		return NULL;
	wl = dev->wl;
redo:
	if (!dev || b43_status(dev) < B43_STAT_STARTED)
		return dev;

	/* Cancel work. Unlock to avoid deadlocks. */
	mutex_unlock(&wl->mutex);
	cancel_delayed_work_sync(&dev->periodic_work);
	cancel_work_sync(&wl->tx_work);
	b43_leds_stop(dev);
	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (!dev || b43_status(dev) < B43_STAT_STARTED) {
		/* Whoops, aliens ate up the device while we were unlocked. */
		return dev;
	}

	/* Disable interrupts on the device. */
	b43_set_status(dev, B43_STAT_INITIALIZED);
	if (b43_bus_host_is_sdio(dev->dev)) {
		/* wl->mutex is locked. That is enough. */
		b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, 0);
		b43_read32(dev, B43_MMIO_GEN_IRQ_MASK);	/* Flush */
	} else {
		spin_lock_irq(&wl->hardirq_lock);
		b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, 0);
		b43_read32(dev, B43_MMIO_GEN_IRQ_MASK);	/* Flush */
		spin_unlock_irq(&wl->hardirq_lock);
	}
	/* Synchronize and free the interrupt handlers. Unlock to avoid deadlocks. */
	orig_dev = dev;
	mutex_unlock(&wl->mutex);
	if (b43_bus_host_is_sdio(dev->dev))
		b43_sdio_free_irq(dev);
	else
		free_irq(dev->dev->irq, dev);
	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (!dev)
		return dev;
	if (dev != orig_dev) {
		if (b43_status(dev) >= B43_STAT_STARTED)
			goto redo;
		return dev;
	}
	mask = b43_read32(dev, B43_MMIO_GEN_IRQ_MASK);
	B43_WARN_ON(mask != 0xFFFFFFFF && mask);

	/* Drain all TX queues. */
	for (queue_num = 0; queue_num < B43_QOS_QUEUE_NUM; queue_num++) {
		while (skb_queue_len(&wl->tx_queue[queue_num])) {
			struct sk_buff *skb;

			skb = skb_dequeue(&wl->tx_queue[queue_num]);
			ieee80211_free_txskb(wl->hw, skb);
		}
	}

	b43_mac_suspend(dev);
	b43_leds_exit(dev);
	b43dbg(wl, "Wireless interface stopped\n");

	return dev;
}

/* Locking: wl->mutex */
static int b43_wireless_core_start(struct b43_wldev *dev)
{
	int err;

	B43_WARN_ON(b43_status(dev) != B43_STAT_INITIALIZED);

	drain_txstatus_queue(dev);
	if (b43_bus_host_is_sdio(dev->dev)) {
		err = b43_sdio_request_irq(dev, b43_sdio_interrupt_handler);
		if (err) {
			b43err(dev->wl, "Cannot request SDIO IRQ\n");
			goto out;
		}
	} else {
		err = request_threaded_irq(dev->dev->irq, b43_interrupt_handler,
					   b43_interrupt_thread_handler,
					   IRQF_SHARED, KBUILD_MODNAME, dev);
		if (err) {
			b43err(dev->wl, "Cannot request IRQ-%d\n",
			       dev->dev->irq);
			goto out;
		}
	}

	/* We are ready to run. */
	ieee80211_wake_queues(dev->wl->hw);
	b43_set_status(dev, B43_STAT_STARTED);

	/* Start data flow (TX/RX). */
	b43_mac_enable(dev);
	b43_write32(dev, B43_MMIO_GEN_IRQ_MASK, dev->irq_mask);

	/* Start maintenance work */
	b43_periodic_tasks_setup(dev);

	b43_leds_init(dev);

	b43dbg(dev->wl, "Wireless interface started\n");
out:
	return err;
}

static char *b43_phy_name(struct b43_wldev *dev, u8 phy_type)
{
	switch (phy_type) {
	case B43_PHYTYPE_A:
		return "A";
	case B43_PHYTYPE_B:
		return "B";
	case B43_PHYTYPE_G:
		return "G";
	case B43_PHYTYPE_N:
		return "N";
	case B43_PHYTYPE_LP:
		return "LP";
	case B43_PHYTYPE_SSLPN:
		return "SSLPN";
	case B43_PHYTYPE_HT:
		return "HT";
	case B43_PHYTYPE_LCN:
		return "LCN";
	case B43_PHYTYPE_LCNXN:
		return "LCNXN";
	case B43_PHYTYPE_LCN40:
		return "LCN40";
	case B43_PHYTYPE_AC:
		return "AC";
	}
	return "UNKNOWN";
}

/* Get PHY and RADIO versioning numbers */
static int b43_phy_versioning(struct b43_wldev *dev)
{
	struct b43_phy *phy = &dev->phy;
	const u8 core_rev = dev->dev->core_rev;
	u32 tmp;
	u8 analog_type;
	u8 phy_type;
	u8 phy_rev;
	u16 radio_manuf;
	u16 radio_id;
	u16 radio_rev;
	u8 radio_ver;
	int unsupported = 0;

	/* Get PHY versioning */
	tmp = b43_read16(dev, B43_MMIO_PHY_VER);
	analog_type = (tmp & B43_PHYVER_ANALOG) >> B43_PHYVER_ANALOG_SHIFT;
	phy_type = (tmp & B43_PHYVER_TYPE) >> B43_PHYVER_TYPE_SHIFT;
	phy_rev = (tmp & B43_PHYVER_VERSION);

	/* LCNXN is continuation of N which run out of revisions */
	if (phy_type == B43_PHYTYPE_LCNXN) {
		phy_type = B43_PHYTYPE_N;
		phy_rev += 16;
	}

	switch (phy_type) {
#ifdef CPTCFG_B43_PHY_G
	case B43_PHYTYPE_G:
		if (phy_rev > 9)
			unsupported = 1;
		break;
#endif
#ifdef CPTCFG_B43_PHY_N
	case B43_PHYTYPE_N:
		if (phy_rev >= 19)
			unsupported = 1;
		break;
#endif
#ifdef CPTCFG_B43_PHY_LP
	case B43_PHYTYPE_LP:
		if (phy_rev > 2)
			unsupported = 1;
		break;
#endif
#ifdef CPTCFG_B43_PHY_HT
	case B43_PHYTYPE_HT:
		if (phy_rev > 1)
			unsupported = 1;
		break;
#endif
#ifdef CPTCFG_B43_PHY_LCN
	case B43_PHYTYPE_LCN:
		if (phy_rev > 1)
			unsupported = 1;
		break;
#endif
#ifdef CPTCFG_B43_PHY_AC
	case B43_PHYTYPE_AC:
		if (phy_rev > 1 && !(core_rev == 56 && phy_rev == 24))
			unsupported = 1;
		break;
#endif
	default:
		unsupported = 1;
	}
	if (unsupported) {
		b43err(dev->wl, "FOUND UNSUPPORTED PHY (Analog %u, Type %d (%s), Revision %u)\n",
		       analog_type, phy_type, b43_phy_name(dev, phy_type),
		       phy_rev);
		return -EOPNOTSUPP;
	}
	b43info(dev->wl, "Found PHY: Analog %u, Type %d (%s), Revision %u\n",
		analog_type, phy_type, b43_phy_name(dev, phy_type), phy_rev);

	/* Get RADIO versioning */
	if (core_rev == 40 || core_rev == 42 || core_rev == 56) {
		/*
		 * Core rev 56: wl's phy_ac_radio_query_idcode (0xc509c in the
		 * AC6 wl.ko) reads the same two words, id 0x3eb = 20693.
		 *
		 * The RADIO24 window reaches the radio, which needs the core's
		 * HT clock. On the R6400 BCM4360 bcma logs "HT force timeout"
		 * and "PLL enable timeout" at boot, and reading the radio then
		 * hung the router (2026-10-04, 2026-10-05 20:43 UTC). CLKCTLST
		 * is in the core's own register space, which bcma itself polls
		 * in bcma_core_set_clockmode() without hanging, so checking it
		 * here is safe. Refuse instead of reading the radio.
		 */
		if (dev->dev->bus_type == B43_BUS_BCMA) {
			u32 clkst = bcma_read32(dev->dev->bdev, BCMA_CLKCTLST);

			b43info(dev->wl, "core rev %u CLKCTLST 0x%08x\n",
				core_rev, clkst);
			if (!(clkst & BCMA_CLKCTLST_HAVEHT)) {
				b43err(dev->wl, "no HT clock, not reading radio\n");
				return -ENODEV;
			}
		}
		radio_manuf = 0x17F;

		b43_write16f(dev, B43_MMIO_RADIO24_CONTROL, 0);
		radio_rev = b43_read16(dev, B43_MMIO_RADIO24_DATA);

		b43_write16f(dev, B43_MMIO_RADIO24_CONTROL, 1);
		radio_id = b43_read16(dev, B43_MMIO_RADIO24_DATA);

		radio_ver = 0; /* Is there version somewhere? */
	} else if (core_rev >= 24) {
		u16 radio24[3];

		for (tmp = 0; tmp < 3; tmp++) {
			b43_write16f(dev, B43_MMIO_RADIO24_CONTROL, tmp);
			radio24[tmp] = b43_read16(dev, B43_MMIO_RADIO24_DATA);
		}

		radio_manuf = 0x17F;
		radio_id = (radio24[2] << 8) | radio24[1];
		radio_rev = (radio24[0] & 0xF);
		radio_ver = (radio24[0] & 0xF0) >> 4;
	} else {
		if (dev->dev->chip_id == 0x4317) {
			if (dev->dev->chip_rev == 0)
				tmp = 0x3205017F;
			else if (dev->dev->chip_rev == 1)
				tmp = 0x4205017F;
			else
				tmp = 0x5205017F;
		} else {
			b43_write16f(dev, B43_MMIO_RADIO_CONTROL,
				     B43_RADIOCTL_ID);
			tmp = b43_read16(dev, B43_MMIO_RADIO_DATA_LOW);
			b43_write16f(dev, B43_MMIO_RADIO_CONTROL,
				     B43_RADIOCTL_ID);
			tmp |= b43_read16(dev, B43_MMIO_RADIO_DATA_HIGH) << 16;
		}
		radio_manuf = (tmp & 0x00000FFF);
		radio_id = (tmp & 0x0FFFF000) >> 12;
		radio_rev = (tmp & 0xF0000000) >> 28;
		radio_ver = 0; /* Probably not available on old hw */
	}

	if (radio_manuf != 0x17F /* Broadcom */)
		unsupported = 1;
	switch (phy_type) {
	case B43_PHYTYPE_B:
		if ((radio_id & 0xFFF0) != 0x2050)
			unsupported = 1;
		break;
	case B43_PHYTYPE_G:
		if (radio_id != 0x2050)
			unsupported = 1;
		break;
	case B43_PHYTYPE_N:
		if (radio_id != 0x2055 && radio_id != 0x2056 &&
		    radio_id != 0x2057)
			unsupported = 1;
		if (radio_id == 0x2057 &&
		    !(radio_rev == 9 || radio_rev == 14))
			unsupported = 1;
		break;
	case B43_PHYTYPE_LP:
		if (radio_id != 0x2062 && radio_id != 0x2063)
			unsupported = 1;
		break;
	case B43_PHYTYPE_HT:
		if (radio_id != 0x2059)
			unsupported = 1;
		break;
	case B43_PHYTYPE_LCN:
		if (radio_id != 0x2064)
			unsupported = 1;
		break;
	case B43_PHYTYPE_AC:
		if (radio_id != 0x2069 && radio_id != 0x3eb)
			unsupported = 1;
		break;
	default:
		B43_WARN_ON(1);
	}
	if (unsupported) {
		b43err(dev->wl,
		       "FOUND UNSUPPORTED RADIO (Manuf 0x%X, ID 0x%X, Revision %u, Version %u)\n",
		       radio_manuf, radio_id, radio_rev, radio_ver);
		return -EOPNOTSUPP;
	}
	b43info(dev->wl,
		"Found Radio: Manuf 0x%X, ID 0x%X, Revision %u, Version %u\n",
		radio_manuf, radio_id, radio_rev, radio_ver);

	/* FIXME: b43 treats "id" as "ver" and ignores the real "ver" */
	phy->radio_manuf = radio_manuf;
	phy->radio_ver = radio_id;
	phy->radio_rev = radio_rev;

	phy->analog = analog_type;
	phy->type = phy_type;
	phy->rev = phy_rev;

	return 0;
}

static void setup_struct_phy_for_init(struct b43_wldev *dev,
				      struct b43_phy *phy)
{
	phy->hardware_power_control = !!modparam_hwpctl;
	phy->next_txpwr_check_time = jiffies;
	/* PHY TX errors counter. */
	atomic_set(&phy->txerr_cnt, B43_PHY_TX_BADNESS_LIMIT);

#if B43_DEBUG
	phy->phy_locked = false;
	phy->radio_locked = false;
#endif
}

static void setup_struct_wldev_for_init(struct b43_wldev *dev)
{
	dev->dfq_valid = false;

	/* Assume the radio is enabled. If it's not enabled, the state will
	 * immediately get fixed on the first periodic work run. */
	dev->radio_hw_enable = true;

	/* Stats */
	memset(&dev->stats, 0, sizeof(dev->stats));

	setup_struct_phy_for_init(dev, &dev->phy);

	/* IRQ related flags */
	dev->irq_reason = 0;
	memset(dev->dma_reason, 0, sizeof(dev->dma_reason));
	dev->irq_mask = B43_IRQ_MASKTEMPLATE;
	if (b43_modparam_verbose < B43_VERBOSITY_DEBUG)
		dev->irq_mask &= ~B43_IRQ_PHY_TXERR;
	/* wl's defmacintmask carries bit 13 (ucode fatal) on this ucode */
	if (dev->dev->core_rev >= 40)
		dev->irq_mask |= B43_IRQ_UCODE_FATAL_REV40;

	dev->mac_suspended = 1;

	/* Noise calculation context */
	memset(&dev->noisecalc, 0, sizeof(dev->noisecalc));
}

static void b43_bluetooth_coext_enable(struct b43_wldev *dev)
{
	struct ssb_sprom *sprom = dev->dev->bus_sprom;
	u64 hf;

	if (!modparam_btcoex)
		return;
	if (!(sprom->boardflags_lo & B43_BFL_BTCOEXIST))
		return;
	if (dev->phy.type != B43_PHYTYPE_B && !dev->phy.gmode)
		return;

	hf = b43_hf_read(dev);
	if (sprom->boardflags_lo & B43_BFL_BTCMOD)
		hf |= B43_HF_BTCOEXALT;
	else
		hf |= B43_HF_BTCOEX;
	b43_hf_write(dev, hf);
}

static void b43_bluetooth_coext_disable(struct b43_wldev *dev)
{
	if (!modparam_btcoex)
		return;
	//TODO
}

static void b43_imcfglo_timeouts_workaround(struct b43_wldev *dev)
{
	struct ssb_bus *bus;
	u32 tmp;

#ifdef CPTCFG_B43_SSB
	if (dev->dev->bus_type != B43_BUS_SSB)
		return;
#else
	return;
#endif

	bus = dev->dev->sdev->bus;

	if ((bus->chip_id == 0x4311 && bus->chip_rev == 2) ||
	    (bus->chip_id == 0x4312)) {
		tmp = ssb_read32(dev->dev->sdev, SSB_IMCFGLO);
		tmp &= ~SSB_IMCFGLO_REQTO;
		tmp &= ~SSB_IMCFGLO_SERTO;
		tmp |= 0x3;
		ssb_write32(dev->dev->sdev, SSB_IMCFGLO, tmp);
		ssb_commit_settings(bus);
	}
}

static void b43_set_synth_pu_delay(struct b43_wldev *dev, bool idle)
{
	u16 pu_delay;

	/* The time value is in microseconds. */
	pu_delay = 1050;
	if (b43_is_mode(dev->wl, NL80211_IFTYPE_ADHOC) || idle)
		pu_delay = 500;
	if ((dev->phy.radio_ver == 0x2050) && (dev->phy.radio_rev == 8))
		pu_delay = max(pu_delay, (u16)2400);

	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_SPUWKUP, pu_delay);
}

/* Set the TSF CFP pre-TargetBeaconTransmissionTime. */
static void b43_set_pretbtt(struct b43_wldev *dev)
{
	u16 pretbtt;

	/* The time value is in microseconds. */
	if (b43_is_mode(dev->wl, NL80211_IFTYPE_ADHOC))
		pretbtt = 2;
	else
		pretbtt = 250;
	/*
	 * wlc_pretbtt_calc @0x1164b4: PHY type 11, no MBSS, AP -> 2 us
	 * (0x116554..0x11656c), written to MMIO 0x612 and shm 0x96
	 * (wlc_pretbtt_set 0x1165fc..0x116618).
	 */
	if (dev->dev->core_rev >= 40 &&
	    b43_is_mode(dev->wl, NL80211_IFTYPE_AP))
		pretbtt = 2;
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRETBTT, pretbtt);
	b43_write16(dev, B43_MMIO_TSF_CFP_PRETBTT, pretbtt);
}

/* Shutdown a wireless core */
/* Locking: wl->mutex */
static void b43_wireless_core_exit(struct b43_wldev *dev)
{
	B43_WARN_ON(dev && b43_status(dev) > B43_STAT_INITIALIZED);
	if (!dev || b43_status(dev) != B43_STAT_INITIALIZED)
		return;

	b43_set_status(dev, B43_STAT_UNINIT);

	/* Stop the microcode PSM. */
	b43_maskset32(dev, B43_MMIO_MACCTL, ~B43_MACCTL_PSM_RUN,
		      B43_MACCTL_PSM_JMP0);

	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_host_pci_down(dev->dev->bdev->bus);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		/* TODO */
		break;
#endif
	}

	b43_dma_free(dev);
	b43_pio_free(dev);
	b43_chip_exit(dev);
	dev->phy.ops->switch_analog(dev, 0);
	if (dev->wl->current_beacon) {
		dev_kfree_skb_any(dev->wl->current_beacon);
		dev->wl->current_beacon = NULL;
	}

	b43_device_disable(dev, 0);
	b43_bus_may_powerdown(dev);
}

/*
 * acphy_retryfix (bitmask, default 0), per-frame transmit retry
 * ceilings on core rev >= 40.
 *
 * ucode 907.113 takes its retry inputs where b43 already writes them:
 * the per-frame short/long retry limits from SCRATCH 0x6/0x7
 * (b43_set_retry_limits == wl's wlc_bmac_retrylimit_upd, which writes
 * objmem routing 0x0002 offsets 0x0006/0x0007); the per-queue EDCF
 * parameters from 0x240 + 32*fifo (b43_qos_params_upload ==
 * wl's wlc_edcf_acp_set_one, same 16-word block, STATUS word |= 0x100);
 * and the per-rate fallback counts from SFFBLIM 0x44 / LFFBLIM 0x46.
 * All offsets and the EDCF layout match wl.ko byte for byte.
 *
 * The one value divergence is the fallback counts: b43 hardcodes 3/2,
 * while the AC header b43 builds carries a single rate entry with
 * MACH_NORATESEL and no fallback entries. A single-entry frame is
 * therefore attempted only up to the fallback count before it runs out
 * of rate entries, so AC_BE data frames give up well short of the
 * global short/long retry limit that small management frames reach.
 * This hook raises those ceilings (bit 1), re-asserts the retry limits
 * (bit 0) and the EDCF STATUS mark (bit 2), logging the old and new
 * shared-memory words so a trial can confirm what the ucode reads. It
 * changes nothing when the parameter is 0.
 */
static unsigned int b43_modparam_acphy_retryfix = 3;
module_param_named(acphy_retryfix, b43_modparam_acphy_retryfix, uint, 0444);
MODULE_PARM_DESC(acphy_retryfix,
		 "AC-PHY retry ceilings (core rev >= 40): 1 = max short/long retry limits, 2 = max fallback counts, 4 = re-assert EDCF status (default 3)");

#define B43_AC_RETRYFIX_SRL	0x1
#define B43_AC_RETRYFIX_FBLIM	0x2
#define B43_AC_RETRYFIX_EDCF	0x4
#define B43_AC_RETRY_MAX	0xF	/* the ucode retry counters are 4-bit */

static void b43_ac_retryfix(struct b43_wldev *dev)
{
	unsigned int fix = b43_modparam_acphy_retryfix;
	u16 old, cur;
	int q;

	if (!fix || dev->dev->core_rev < 40)
		return;

	if (fix & B43_AC_RETRYFIX_SRL) {
		old = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_SHM_SC_SRLIMIT);
		b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_SRLIMIT,
				B43_AC_RETRY_MAX);
		cur = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_SHM_SC_SRLIMIT);

		old = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_SHM_SC_LRLIMIT);
		b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_LRLIMIT,
				B43_AC_RETRY_MAX);
		cur = b43_shm_read16(dev, B43_SHM_SCRATCH, B43_SHM_SC_LRLIMIT);
	}

	if (fix & B43_AC_RETRYFIX_FBLIM) {
		old = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_SFFBLIM);
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_SFFBLIM,
				B43_AC_RETRY_MAX);
		cur = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_SFFBLIM);

		old = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_LFFBLIM);
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_LFFBLIM,
				B43_AC_RETRY_MAX);
		cur = b43_shm_read16(dev, B43_SHM_SHARED, B43_SHM_SH_LFFBLIM);
	}

	if (fix & B43_AC_RETRYFIX_EDCF) {
		for (q = 0; q < 4; q++) {
			u16 off = B43_QOS_PARAMS(q) + B43_QOSPARAM_STATUS * 2;

			old = b43_shm_read16(dev, B43_SHM_SHARED, off);
			b43_shm_write16(dev, B43_SHM_SHARED, off, old | 0x100);
			cur = b43_shm_read16(dev, B43_SHM_SHARED, off);
		}
	}
}

/* Initialize a wireless core */
static int b43_wireless_core_init(struct b43_wldev *dev)
{
	struct ssb_sprom *sprom = dev->dev->bus_sprom;
	struct b43_phy *phy = &dev->phy;
	int err;
	u64 hf;

	B43_WARN_ON(b43_status(dev) != B43_STAT_UNINIT);

	err = b43_bus_powerup(dev, 0);
	if (err)
		goto out;
	if (!b43_device_is_enabled(dev))
		b43_wireless_core_reset(dev, phy->gmode);

	/* Reset all data structures. */
	setup_struct_wldev_for_init(dev);
	phy->ops->prepare_structs(dev);

	/* Enable IRQ routing to this device. */
	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		bcma_host_pci_irq_ctl(dev->dev->bdev->bus,
				      dev->dev->bdev, true);
		bcma_host_pci_up(dev->dev->bdev->bus);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		ssb_pcicore_dev_irqvecs_enable(&dev->dev->sdev->bus->pcicore,
					       dev->dev->sdev);
		break;
#endif
	}

	b43_imcfglo_timeouts_workaround(dev);
	b43_bluetooth_coext_disable(dev);
	if (phy->ops->prepare_hardware) {
		err = phy->ops->prepare_hardware(dev);
		if (err)
			goto err_busdown;
	}
	err = b43_chip_init(dev);
	if (err)
		goto err_busdown;
	b43_shm_write16(dev, B43_SHM_SHARED,
			B43_SHM_SH_WLCOREREV, dev->dev->core_rev);
	hf = b43_hf_read(dev);
	if (phy->type == B43_PHYTYPE_G) {
		hf |= B43_HF_SYMW;
		if (phy->rev == 1)
			hf |= B43_HF_GDCW;
		if (sprom->boardflags_lo & B43_BFL_PACTRL)
			hf |= B43_HF_OFDMPABOOST;
	}
	if (phy->radio_ver == 0x2050) {
		if (phy->radio_rev == 6)
			hf |= B43_HF_4318TSSI;
		if (phy->radio_rev < 6)
			hf |= B43_HF_VCORECALC;
	}
	if (sprom->boardflags_lo & B43_BFL_XTAL_NOSLOW)
		hf |= B43_HF_DSCRQ; /* Disable slowclock requests from ucode. */
#if defined(CPTCFG_B43_SSB) && defined(CONFIG_SSB_DRIVER_PCICORE)
	if (dev->dev->bus_type == B43_BUS_SSB &&
	    dev->dev->sdev->bus->bustype == SSB_BUSTYPE_PCI &&
	    dev->dev->sdev->bus->pcicore.dev->id.revision <= 10)
		hf |= B43_HF_PCISCW; /* PCI slow clock workaround. */
#endif
	hf &= ~B43_HF_SKCFPUP;
	/*
	 * ucode 907: hostapd answers probe requests, so turn ucode probe
	 * responses off the way wl does in its host probe response mode
	 * (wlc_config_ucode_probe_resp 0x11351c..0x11355c). Without the bit
	 * the ucode answers from the probe response template at 0x700
	 * (wlc_bss_update_probe_resp 0x128428..0x128444), which b43 never
	 * writes; PRMAXTIME = 1 below was b43's only guard.
	 */
	if (dev->dev->core_rev >= 40)
		hf |= B43_HF_AC_NOPRBRESP;
	/*
	 * acphy_ampdu bit 1, the A-MPDU TX host flag wl sets with its
	 * AQM mode (wlc_ampdu_tx_set 0x143314..0x14332c); the epoch state of
	 * every TX FIFO starts over with the ucode (xmit.c b43_ac_ampdu_epoch).
	 */
	if (b43_ac_ampdu(dev, B43_AC_AMPDU_TX)) {
		/*
		 * the flag only with bit 5. Ucode 907 tests shm 0x62
		 * (its word [0x31]) bits 2, 3, 4, 5, 7, 8, 9 and 12 and never
		 * bit 6, so on this ucode the flag should be inert.
		 */
		if (b43_ac_ampdu(dev, B43_AC_AMPDU_HF))
			hf |= B43_HF_AC_AMPDUTX;
		memset(dev->ac_ampdu_epoch, 0, sizeof(dev->ac_ampdu_epoch));
	}
	/* the per-frame cuts in force for this core init, once */
	b43_hf_write(dev, hf);
	/* acphy_ampdu bit 2, wl's A-MPDU shm words */
	b43_ac_ampdu_shm(dev);

	/* tell the ucode MAC capabilities */
	if (dev->dev->core_rev >= 13) {
		u32 mac_hw_cap = b43_read32(dev, B43_MMIO_MAC_HW_CAP);

		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_MACHW_L,
				mac_hw_cap & 0xffff);
		b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_MACHW_H,
				(mac_hw_cap >> 16) & 0xffff);
	}

	b43_set_retry_limits(dev, B43_DEFAULT_SHORT_RETRY_LIMIT,
			     B43_DEFAULT_LONG_RETRY_LIMIT);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_SFFBLIM, 3);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_LFFBLIM, 2);
	b43_ac_retryfix(dev);

	/* Disable sending probe responses from firmware.
	 * Setting the MaxTime to one usec will always trigger
	 * a timeout, so we never send any probe resp.
	 * A timeout of zero is infinite. */
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PRMAXTIME, 1);

	b43_rate_memory_init(dev);
	b43_set_phytxctl_defaults(dev);
	/* ucode 907 TX core mask block, wlc_init 0x12f04c..0x12f070 */
	b43_ac_txcore_shm_write(dev);
	/*
	 * TOE prefix on: wlc_up clears bit 0 of MMIO 0xa00 when set
	 * (0x129bdc..0x129c2c) for core rev > 39 (wlc+0x7c6, wlc_attach
	 * 0x13d4fc..0x13d504), and wl then adds the 4 byte prefix to every
	 * TX header (see b43_txhdr_has_toe). Meaning of the register beyond
	 * that bit is not decoded.
	 */
	if (b43_txhdr_has_toe(dev)) {
		u16 toe = b43_read16(dev, 0xA00);

		b43info(dev->wl, "TOE: MMIO 0xa00 = 0x%04x\n", toe);
		if (toe & 1)
			b43_write16(dev, 0xA00, toe & ~1);
	}
	/*
	 * wlc_bmac_ifsctl_vht_set(hw, 1) for PHY type 11 from
	 * wlc_bmac_up_finish (0x15f9a8) and wlc_set_chanspec (0x120fa8): at
	 * 20 MHz MMIO 0x6c6 = 0x0f0f (0x15f798) and shm 0x5a = 0x0f0f
	 * (0x15f7a0). Meaning a guess (CRS select per 20 MHz subband for the
	 * IFS engine); no initvals entry writes 0x6c6. b43 reads 0x5a as the
	 * high half of TSSI_CCK, only on G-PHY paths.
	 */
	if (dev->phy.type == B43_PHYTYPE_AC && dev->dev->core_rev >= 40) {
		b43_write16(dev, 0x6C6, 0x0F0F);
		b43_shm_write16(dev, B43_SHM_SHARED, 0x005A, 0x0F0F);
		/*
		 * The same wlc_bmac_ifsctl_vht_set(hw, 1) ends in
		 * wlc_acphy_set_scramb_dyn_bw_en(pi, 1) (tail call 0x15f888):
		 * PHY 0x042 bit 15 set (0x4305c..0x43098). Name from the wl
		 * symbol: scrambler dynamic bandwidth enable.
		 */
		b43_phy_set(dev, 0x042, 0x8000);
	}

	/*
	 * wl's MAC timing on core rev >= 40 that b43 leaves at the
	 * initvals or at 0.
	 * - wlc_bmac_init clears IFS_CTL bits 12..15 (0x163618..0x163634) and
	 *   writes MMIO 0x69c = 1 (0x163650..0x163658).
	 * - wlc_bmac_bsinit calls wlc_bmac_update_slot_timing with short
	 *   slot forced on for the 5 GHz band (band type 1, 0x160420..
	 *   0x160434): MMIO 0x684 (IFS slot) = 0x207 and shm 0x10 (slot
	 *   time) = 9 (0x15ff98..0x15ffb0; long slot 0x212 / 20). b43's
	 *   b43_set_slot_time returns on 5 GHz and never writes shm 0x10, and
	 *   b43 zeroes shm before the ucode upload, so the ucode's slot time
	 *   was 0.
	 */
	if (dev->phy.type == B43_PHYTYPE_AC && dev->dev->core_rev >= 40) {
		b43_write16(dev, B43_MMIO_IFSCTL,
			    b43_read16(dev, B43_MMIO_IFSCTL) & 0x0FFF);
		b43_write16(dev, 0x69C, 1);
		if (b43_current_band(dev->wl) == NL80211_BAND_5GHZ) {
			b43_write16(dev, B43_MMIO_IFSSLOT, 0x207);
			b43_shm_write16(dev, B43_SHM_SHARED, 0x0010, 9);
		}
	}

	/* Minimum Contention Window */
	if (phy->type == B43_PHYTYPE_B)
		b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_MINCONT, 0x1F);
	else
		b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_MINCONT, 0xF);
	/* Maximum Contention Window */
	b43_shm_write16(dev, B43_SHM_SCRATCH, B43_SHM_SC_MAXCONT, 0x3FF);

	/* write phytype and phyvers */
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PHYTYPE, phy->type);
	b43_shm_write16(dev, B43_SHM_SHARED, B43_SHM_SH_PHYVER, phy->rev);

	if (b43_bus_host_is_pcmcia(dev->dev) ||
	    b43_bus_host_is_sdio(dev->dev)) {
		dev->__using_pio_transfers = true;
		err = b43_pio_init(dev);
	} else if (dev->use_pio) {
		b43warn(dev->wl, "Forced PIO by use_pio module parameter. "
			"This should not be needed and will result in lower "
			"performance.\n");
		dev->__using_pio_transfers = true;
		err = b43_pio_init(dev);
	} else {
		dev->__using_pio_transfers = false;
		err = b43_dma_init(dev);
	}
	if (err)
		goto err_chip_exit;
	b43_qos_init(dev);
	b43_set_synth_pu_delay(dev, 1);
	b43_bluetooth_coext_enable(dev);

	b43_bus_powerup(dev, !(sprom->boardflags_lo & B43_BFL_XTAL_NOSLOW));
	b43_upload_card_macaddress(dev);
	b43_security_init(dev);
	/* the core mask block, after anything that wrote shm above */
	b43_ac_txcore_shm_write(dev);
	/* this init's HT TX counters start over (xmit.c) */
	b43_ac_htdead_core_init(dev);
	/* acphy_initfix, the per-wldev AC state a re-init inherits */
	b43_ac_initfix_wldev(dev);

	ieee80211_wake_queues(dev->wl->hw);

	b43_set_status(dev, B43_STAT_INITIALIZED);

out:
	return err;

err_chip_exit:
	b43_chip_exit(dev);
err_busdown:
	b43_bus_may_powerdown(dev);
	B43_WARN_ON(b43_status(dev) != B43_STAT_UNINIT);
	return err;
}

static int b43_op_add_interface(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;
	int err = -EOPNOTSUPP;

	/* TODO: allow AP devices to coexist */

	if (vif->type != NL80211_IFTYPE_AP &&
	    vif->type != NL80211_IFTYPE_MESH_POINT &&
	    vif->type != NL80211_IFTYPE_STATION &&
	    vif->type != NL80211_IFTYPE_ADHOC)
		return -EOPNOTSUPP;

	mutex_lock(&wl->mutex);
	if (wl->operating)
		goto out_mutex_unlock;

	b43dbg(wl, "Adding Interface type %d\n", vif->type);

	dev = wl->current_dev;
	wl->operating = true;
	wl->vif = vif;
	wl->if_type = vif->type;
	memcpy(wl->mac_addr, vif->addr, ETH_ALEN);

	b43_adjust_opmode(dev);
	b43_set_pretbtt(dev);
	b43_set_synth_pu_delay(dev, 0);
	b43_upload_card_macaddress(dev);

	err = 0;
 out_mutex_unlock:
	mutex_unlock(&wl->mutex);

	if (err == 0)
		b43_op_bss_info_changed(hw, vif, &vif->bss_conf, ~0);

	return err;
}

static void b43_op_remove_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev = wl->current_dev;

	b43dbg(wl, "Removing Interface type %d\n", vif->type);

	mutex_lock(&wl->mutex);

	B43_WARN_ON(!wl->operating);
	B43_WARN_ON(wl->vif != vif);
	wl->vif = NULL;

	wl->operating = false;

	b43_adjust_opmode(dev);
	eth_zero_addr(wl->mac_addr);
	b43_upload_card_macaddress(dev);

	mutex_unlock(&wl->mutex);
}

static int b43_op_start(struct ieee80211_hw *hw)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev = wl->current_dev;
	int did_init = 0;
	int err = 0;

	/* Kill all old instance specific information to make sure
	 * the card won't use it in the short timeframe between start
	 * and mac80211 reconfiguring it. */
	eth_zero_addr(wl->bssid);
	eth_zero_addr(wl->mac_addr);
	wl->filter_flags = 0;
	wl->radiotap_enabled = false;
	b43_qos_clear(wl);
	/* the ucode-fatal restart budget is per interface start,
	 * not per module: a controller restart's core init keeps it (so a
	 * restart loop still ends), a fresh start gets a new one. */
	atomic_set(&b43_ucode_fatal_count, 0);
	wl->beacon0_uploaded = false;
	wl->beacon1_uploaded = false;
	wl->beacon_templates_virgin = true;
	wl->radio_enabled = true;

	mutex_lock(&wl->mutex);

	/* a new HT-dead restart chain with every mac80211 start */
	b43_ac_htdead_chain_reset(dev);
	if (b43_status(dev) < B43_STAT_INITIALIZED) {
		err = b43_wireless_core_init(dev);
		if (err)
			goto out_mutex_unlock;
		did_init = 1;
	}

	if (b43_status(dev) < B43_STAT_STARTED) {
		err = b43_wireless_core_start(dev);
		if (err) {
			if (did_init)
				b43_wireless_core_exit(dev);
			goto out_mutex_unlock;
		}
	}

	/* XXX: only do if device doesn't support rfkill irq */
	wiphy_rfkill_start_polling(hw->wiphy);

 out_mutex_unlock:
	mutex_unlock(&wl->mutex);

	/*
	 * Configuration may have been overwritten during initialization.
	 * Reload the configuration, but only if initialization was
	 * successful. Reloading the configuration after a failed init
	 * may hang the system.
	 */
	if (!err)
		b43_op_config(hw, -1, ~0);

	return err;
}

static void b43_op_stop(struct ieee80211_hw *hw, bool suspend)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev = wl->current_dev;

	cancel_work_sync(&(wl->beacon_update_trigger));

	if (!dev)
		goto out;

	mutex_lock(&wl->mutex);
	if (b43_status(dev) >= B43_STAT_STARTED) {
		dev = b43_wireless_core_stop(dev);
		if (!dev)
			goto out_unlock;
	}
	b43_wireless_core_exit(dev);
	wl->radio_enabled = false;

out_unlock:
	mutex_unlock(&wl->mutex);
out:
	cancel_work_sync(&(wl->txpower_adjust_work));
}

static int b43_op_beacon_set_tim(struct ieee80211_hw *hw,
				 struct ieee80211_sta *sta, bool set)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);

	/* acphy_psfix bit 1, mac80211's TIM request (sta_info_recalc_tim) */

	b43_update_templates(wl);

	return 0;
}

static void b43_op_sta_notify(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      enum sta_notify_cmd notify_cmd,
			      struct ieee80211_sta *sta)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);

	B43_WARN_ON(!vif || wl->vif != vif);

	/*
	 * acphy_psfix bit 1, mac80211's view of the station (sta_ps_start
	 * / sta_ps_end from the PM bit of received frames; not called with
	 * AP_LINK_PS, where the PMQ lines show it instead).
	 */
}

/*
 * acphy_psfix bit 8 (the AMT comment sits with b43_ac_amt_sta_get).
 * the entry is taken through the allocator shared with the hardware
 * key, and drv_priv is only touched when hw->sta_data_size made room for
 * a b43_ac_sta (it is 0 without the bits that need one).
 */
static int b43_op_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			  struct ieee80211_sta *sta)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_ac_sta *acs = (struct b43_ac_sta *)sta->drv_priv;
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (hw->sta_data_size < sizeof(*acs))
		goto out;
	acs->amt_idx = 0;
	acs->amt_users = 0;
	if (!dev || b43_status(dev) < B43_STAT_INITIALIZED ||
	    !b43_ac_psfix(dev, B43_AC_PSFIX_AMT) ||
	    vif->type != NL80211_IFTYPE_AP)
		goto out;
out:
	mutex_unlock(&wl->mutex);
	return 0;
}

static int b43_op_sta_remove(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct ieee80211_sta *sta)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_ac_sta *acs = (struct b43_ac_sta *)sta->drv_priv;
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	/* mac80211 drops the station's keys first; every user goes anyway */
	if (hw->sta_data_size >= sizeof(*acs) && acs->amt_idx)
		b43_ac_amt_sta_put(dev, sta,
				   B43_AC_AMT_USER_PS | B43_AC_AMT_USER_KEY);
	mutex_unlock(&wl->mutex);
	return 0;
}

static void b43_op_sw_scan_start_notifier(struct ieee80211_hw *hw,
					  struct ieee80211_vif *vif,
					  const u8 *mac_addr)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (dev && (b43_status(dev) >= B43_STAT_INITIALIZED)) {
		/* Disable CFP update during scan on other channels. */
		b43_hf_write(dev, b43_hf_read(dev) | B43_HF_SKCFPUP);
	}
	mutex_unlock(&wl->mutex);
}

static void b43_op_sw_scan_complete_notifier(struct ieee80211_hw *hw,
					     struct ieee80211_vif *vif)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (dev && (b43_status(dev) >= B43_STAT_INITIALIZED)) {
		/* Re-enable CFP update. */
		b43_hf_write(dev, b43_hf_read(dev) & ~B43_HF_SKCFPUP);
	}
	mutex_unlock(&wl->mutex);
}

static int b43_op_get_survey(struct ieee80211_hw *hw, int idx,
			     struct survey_info *survey)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev = wl->current_dev;
	struct ieee80211_conf *conf = &hw->conf;

	if (idx != 0)
		return -ENOENT;

	survey->channel = conf->chandef.chan;
	survey->filled = SURVEY_INFO_NOISE_DBM;
	survey->noise = dev->stats.link_noise;

	return 0;
}

static int b43_op_set_antenna(struct ieee80211_hw *hw, int radio_idx,
			      u32 tx_ant, u32 rx_ant)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev = wl->current_dev;

	if (tx_ant == 1 && rx_ant == 1) {
		dev->tx_antenna = B43_ANTENNA0;
		dev->rx_antenna = B43_ANTENNA0;
	}
	else if (tx_ant == 2 && rx_ant == 2) {
		dev->tx_antenna = B43_ANTENNA1;
		dev->rx_antenna = B43_ANTENNA1;
	}
	else if ((tx_ant & 3) == 3 && (rx_ant & 3) == 3) {
		dev->tx_antenna = B43_ANTENNA_DEFAULT;
		dev->rx_antenna = B43_ANTENNA_DEFAULT;
	}
	else {
		return -EINVAL;
	}

	return 0;
}


static int b43_op_get_antenna(struct ieee80211_hw *hw, int radio_idx,
			      u32 *tx_ant, u32 *rx_ant)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev = wl->current_dev;

	switch (dev->tx_antenna) {
	case B43_ANTENNA0:
		*tx_ant = 1; *rx_ant = 1; break;
	case B43_ANTENNA1:
		*tx_ant = 2; *rx_ant = 2; break;
	case B43_ANTENNA_DEFAULT:
		*tx_ant = 3; *rx_ant = 3; break;
	}
	return 0;
}

/*
 * block ack sessions on core rev >= 40 (acphy_ampdu, xmit.c).
 * RX needs nothing from the driver:
 * wl answers the ADDBA request in software (wlc_send_addba_resp 0x146638)
 * and keeps the session on the host (scb_ampdu_rx_init 0x144f6c,
 * wlc_ampdu_recvdata 0x10d4c reorders), with no shm, AMT or register write
 * for it; the ucode sends the block ack. mac80211 does the reordering here.
 *
 * acphy_ampdu bit 1, TX sessions (wl's AQM path, _wlc_sendampdu_aqm
 * 0xd9a8 in .text.fastpath): the ucode needs no per-session state either;
 * wl's per-TID initiator (ini) only feeds the header tail of each MPDU
 * (xmit.c b43_ac_ampdu_txhdr). TX_START is granted at once; TX_OPERATIONAL
 * stores the negotiated window in the station's drv_priv, from which the
 * header takes the block ack window and the MPDU limit; the STOP variants
 * clear it, so the frames queued afterwards go out non-aggregated.
 */
static int b43_op_ampdu_action(struct ieee80211_hw *hw,
			       struct ieee80211_vif *vif,
			       struct ieee80211_ampdu_params *params)
{
	struct b43_wl *wl = hw_to_b43_wl(hw);
	struct b43_wldev *dev;
	struct b43_ac_sta *bsta = (struct b43_ac_sta *)params->sta->drv_priv;
	int err = -EOPNOTSUPP;

	mutex_lock(&wl->mutex);
	dev = wl->current_dev;
	if (!dev || dev->phy.type != B43_PHYTYPE_AC ||
	    !b43_ac_ampdu(dev, B43_AC_AMPDU_RX | B43_AC_AMPDU_TX))
		goto out_unlock;
	switch (params->action) {
	case IEEE80211_AMPDU_RX_START:
		if (!b43_ac_ampdu(dev, B43_AC_AMPDU_RX))
			break;
		err = 0;
		break;
	case IEEE80211_AMPDU_RX_STOP:
		if (!b43_ac_ampdu(dev, B43_AC_AMPDU_RX))
			break;
		err = 0;
		break;
	case IEEE80211_AMPDU_TX_START:
		/* sessions only with bit 6 (and bit 1) */
		if (!b43_ac_ampdu(dev, B43_AC_AMPDU_TX) ||
		    !b43_ac_ampdu(dev, B43_AC_AMPDU_SESSION) ||
		    params->tid >= IEEE80211_NUM_TIDS)
			break;
		err = IEEE80211_AMPDU_TX_START_IMMEDIATE;
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		if (!b43_ac_ampdu(dev, B43_AC_AMPDU_TX) ||
		    !b43_ac_ampdu(dev, B43_AC_AMPDU_SESSION) ||
		    params->tid >= IEEE80211_NUM_TIDS)
			break;
		bsta->ampdu_wsize[params->tid] =
			clamp_t(u16, params->buf_size, 1, B43_AC_AMPDU_WSIZE_DEF);
		err = 0;
		break;
	case IEEE80211_AMPDU_TX_STOP_CONT:
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		if (!b43_ac_ampdu(dev, B43_AC_AMPDU_TX) ||
		    !b43_ac_ampdu(dev, B43_AC_AMPDU_SESSION) ||
		    params->tid >= IEEE80211_NUM_TIDS)
			break;
		bsta->ampdu_wsize[params->tid] = 0;
		/* Queued frames go out unaggregated; nothing to flush */
		if (params->action == IEEE80211_AMPDU_TX_STOP_CONT)
			ieee80211_stop_tx_ba_cb_irqsafe(vif, params->sta->addr,
							params->tid);
		err = 0;
		break;
	default:
		break;
	}
out_unlock:
	mutex_unlock(&wl->mutex);
	return err;
}

static const struct ieee80211_ops b43_hw_ops = {
	.add_chanctx = ieee80211_emulate_add_chanctx,
	.remove_chanctx = ieee80211_emulate_remove_chanctx,
	.change_chanctx = ieee80211_emulate_change_chanctx,
	.switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx,
	.tx			= b43_op_tx,
	.wake_tx_queue		= ieee80211_handle_wake_tx_queue,
	.conf_tx		= b43_op_conf_tx,
	.add_interface		= b43_op_add_interface,
	.remove_interface	= b43_op_remove_interface,
	.config			= b43_op_config,
	.bss_info_changed	= b43_op_bss_info_changed,
	.configure_filter	= b43_op_configure_filter,
	.set_key		= b43_op_set_key,
	.update_tkip_key	= b43_op_update_tkip_key,
	.get_stats		= b43_op_get_stats,
	.get_tsf		= b43_op_get_tsf,
	.set_tsf		= b43_op_set_tsf,
	.start			= b43_op_start,
	.stop			= b43_op_stop,
	.set_tim		= b43_op_beacon_set_tim,
	.sta_notify		= b43_op_sta_notify,
	.sta_add		= b43_op_sta_add,
	.sta_remove		= b43_op_sta_remove,
	.sw_scan_start		= b43_op_sw_scan_start_notifier,
	.sw_scan_complete	= b43_op_sw_scan_complete_notifier,
	.get_survey		= b43_op_get_survey,
	.rfkill_poll		= b43_rfkill_poll,
	.set_antenna		= b43_op_set_antenna,
	.get_antenna		= b43_op_get_antenna,
	.ampdu_action		= b43_op_ampdu_action,
};

/* Hard-reset the chip. Do not call this directly.
 * Use b43_controller_restart()
 */
static void b43_chip_reset(struct work_struct *work)
{
	struct b43_wldev *dev =
	    container_of(work, struct b43_wldev, restart_work);
	struct b43_wl *wl = dev->wl;
	int err = 0;
	int prev_status;

	mutex_lock(&wl->mutex);

	prev_status = b43_status(dev);
	/* Bring the device down... */
	if (prev_status >= B43_STAT_STARTED) {
		dev = b43_wireless_core_stop(dev);
		if (!dev) {
			err = -ENODEV;
			goto out;
		}
	}
	if (prev_status >= B43_STAT_INITIALIZED)
		b43_wireless_core_exit(dev);

	/* ...and up again. */
	if (prev_status >= B43_STAT_INITIALIZED) {
		err = b43_wireless_core_init(dev);
		if (err)
			goto out;
	}
	if (prev_status >= B43_STAT_STARTED) {
		err = b43_wireless_core_start(dev);
		if (err) {
			b43_wireless_core_exit(dev);
			goto out;
		}
	}
out:
	if (err)
		wl->current_dev = NULL; /* Failed to init the dev. */
	mutex_unlock(&wl->mutex);

	if (err) {
		b43err(wl, "Controller restart FAILED\n");
		return;
	}

	/* reload configuration */
	b43_op_config(wl->hw, -1, ~0);
	if (wl->vif)
		b43_op_bss_info_changed(wl->hw, wl->vif, &wl->vif->bss_conf, ~0);

	b43info(wl, "Controller restarted\n");
}

/*
 * HT capabilities of the AC-PHY 5 GHz band.
 * wl.ko starts from 0x181e (wlc_attach 0x13e390..0x13e39c:
 * 20/40, SM power save disabled, greenfield, max A-MSDU 7935, DSSS/CCK 40)
 * and adds short GI 20 and 40 (PHY cap bit 2, 0x13e47c..0x13e4a4,
 * wlc_ht_update_sgi_rx 0x12855c), RX STBC 1 (PHY cap bit 1,
 * wlc_stf_stbc_rx_set 0x13e4e8, bits 8..9 at 0x1905fc..0x19060c), TX STBC
 * with more than one TX stream (0x13e4f4..0x13e520) and LDPC (PHY cap
 * bit 4, 0x13e3ac); wlc_phy_ac_caps 0x2a174 gives this PHY rev the STBC,
 * SGI and LDPC bits. A-MPDU: factor 3 (64 KiB, 0x1462e8), density 5 (4 us,
 * 0x14628c), wlc_ampdu_update_ie_param 0x146138.
 *
 * b43 advertises the subset it implements: 20 MHz only, SM power save
 * disabled (all chains on, as wl), short GI 20 (TX in HT-SIG, RX decoded),
 * RX STBC 1 (the PHY decodes it; b43_rx reports it). Not advertised:
 * 40 MHz and DSSS/CCK 40 (HT20 only), greenfield (b43 always sends mixed
 * format; greenfield RX unverified), max A-MSDU 7935 (RX buffer sized for
 * 3839), LDPC (TX not implemented; wl's wlc_phy_update_rxldpc_acphy not
 * ported), TX STBC (single core by default). A-MPDU parameters are wl's;
 * mac80211 declines every block ack session unless acphy_ampdu bit 0
 * accepts RX sessions (b43_op_ampdu_action) or bit 1 TX sessions
 * (hardware AQM aggregation).
 */
/*
 * BCM43217 (N-PHY, 2x2 on the AC6): advertise HT20 so hostapd/mac80211 can
 * run ieee80211n. Without this, phy1 has no ht_cap and OpenWrt's HT20 mode
 * fails ("Driver does not support configured HT capability"). HT40 left off
 * for now — earlier tries hit extension-channel setup failures on this chip.
 */
static void b43_n_setup_ht_cap(struct b43_wldev *dev,
			       struct ieee80211_sta_ht_cap *ht_cap)
{
	memset(ht_cap, 0, sizeof(*ht_cap));
	ht_cap->ht_supported = true;
	ht_cap->cap = IEEE80211_HT_CAP_SM_PS |
		      IEEE80211_HT_CAP_SGI_20 |
		      IEEE80211_HT_CAP_TX_STBC |
		      (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
	ht_cap->ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
	ht_cap->ampdu_density = IEEE80211_HT_MPDU_DENSITY_4;
	ht_cap->mcs.rx_mask[0] = 0xFF;
	ht_cap->mcs.rx_mask[1] = 0xFF;
	ht_cap->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
	b43info(dev->wl, "N-PHY HT20: 2 streams, MCS 0..15\n");
}

static void b43_ac_setup_ht_cap(struct b43_wldev *dev,
				struct ieee80211_sta_ht_cap *ht_cap)
{
	unsigned int nss = b43_ac_ht_nss(dev), i;

	memset(ht_cap, 0, sizeof(*ht_cap));
	if (!nss)
		return;
	ht_cap->ht_supported = true;
	ht_cap->cap = IEEE80211_HT_CAP_SM_PS |
		      IEEE80211_HT_CAP_SGI_20 |
		      (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
	/*
	 * acphy_bw40: 20/40 MHz and short GI at 40 MHz, as wl
	 * advertises them for this chip (0x181e at 0x13e390..0x13e39c,
	 * wlc_ht_update_sgi_rx(wlc, 3) at 0x13e47c..0x13e4a4). DSSS/CCK 40
	 * stays off: 5 GHz only.
	 */
	if (b43_ac_bw40_enabled())
		ht_cap->cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
			       IEEE80211_HT_CAP_SGI_40;
	/*
	 * acphy_htstbc: TX STBC as wl advertises it with
	 * more than one TX stream (wlc_stf_attach 0x192eb4..0x192f04)
	 */
	if (b43_ac_ht_tx_stbc(dev))
		ht_cap->cap |= IEEE80211_HT_CAP_TX_STBC;
	ht_cap->ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
	ht_cap->ampdu_density = IEEE80211_HT_MPDU_DENSITY_4;
	/*
	 * RX MCS set as wlc_rateset_mcs_build (0x181b44) makes it: .rodata
	 * 0xd75e9 (ff ff ff ff, MCS 0..31, no MCS 32) with bytes nss..3
	 * cleared (0x181b68..0x181b7c). TX set = RX set, so TX_DEFINED alone.
	 */
	for (i = 0; i < nss; i++)
		ht_cap->mcs.rx_mask[i] = 0xFF;
	ht_cap->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
	b43info(dev->wl, "AC-PHY HT20%s: %u stream(s), MCS 0..%u%s\n",
		b43_ac_bw40_enabled() ? "/40" : "", nss, 8 * nss - 1,
		b43_ac_ht_tx_stbc(dev) ? ", TX STBC" : "");
}

/*
 * VHT capabilities of the AC-PHY band (acphy_vht). wl on
 * this chip (wlc_vht_init_defaults 0x19c4c4, wlc_up 0x129adc..0x129b30)
 * advertises 0x038001b2 without beamforming: max MPDU 11454, 80 MHz only
 * (no 160 / 80+80), RX LDPC, short GI 80, TX STBC (txstreams > 1), RX STBC
 * 1, A-MPDU exponent 7; RX and TX MCS maps 0xffea (MCS 0..9 on 3 streams);
 * highest rates 0. Beamforming bits only with wl's txbf on.
 *
 * b43 advertises the subset it implements, as for HT (b43_ac_setup_ht_cap):
 * max MPDU 3895 (RX buffers sized for 3839), RX STBC 1, short GI 80 only
 * with the 80 MHz width, A-MPDU exponent 3 (64 KiB, the HT factor; b43 has
 * no ampdu_action). Not advertised: RX LDPC (wlc_phy_update_rxldpc_acphy
 * not ported), TX STBC, beamforming, 160 MHz. MCS 0..9 on the streams b43
 * transmits (acphy_htnss, the TX limit), RX and TX alike. 80 MHz needs
 * acphy_bw40 and acphy_bw80 (mac80211 uses 80 MHz only with HT 20/40);
 * without them every channel gets IEEE80211_CHAN_NO_80MHZ. Untested.
 */
static void b43_ac_setup_vht_cap(struct b43_wldev *dev,
				 struct ieee80211_supported_band *band)
{
	struct ieee80211_sta_vht_cap *vht_cap = &band->vht_cap;
	unsigned int nss = b43_ac_ht_nss(dev), i;
	bool bw80 = b43_ac_bw40_enabled() && b43_ac_bw80_enabled();
	u16 map = 0;

	memset(vht_cap, 0, sizeof(*vht_cap));
	if (!b43_modparam_ac_vht || !nss)
		return;
	vht_cap->vht_supported = true;
	vht_cap->cap = IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_3895 |
		       IEEE80211_VHT_CAP_RXSTBC_1 |
		       (3 << IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_SHIFT);
	if (bw80)
		vht_cap->cap |= IEEE80211_VHT_CAP_SHORT_GI_80;
	for (i = 0; i < 8; i++)
		map |= (i < nss ? IEEE80211_VHT_MCS_SUPPORT_0_9 :
				  IEEE80211_VHT_MCS_NOT_SUPPORTED) << (2 * i);
	vht_cap->vht_mcs.rx_mcs_map = cpu_to_le16(map);
	/*
	 * BCM53573 (core rev 56): VHT-MCS 8 and 9 (256-QAM) do not get through
	 * on TX yet (every attempt fails at 80 MHz, 1 or 2 streams, while MCS
	 * 0..7 go out without retries); RX of MCS 9 works. Advertise TX 0..7 so
	 * mac80211 does not pick them.
	 */
	if (dev->dev->core_rev == 56) {
		map = 0;
		for (i = 0; i < 8; i++)
			map |= (i < nss ? IEEE80211_VHT_MCS_SUPPORT_0_7 :
					  IEEE80211_VHT_MCS_NOT_SUPPORTED) << (2 * i);
		/* RX of MCS 8/9 under load: the AP's aggregates crawl (~1 Mbit/s) */
		if (b43_modparam_ac56_rx07)
			vht_cap->vht_mcs.rx_mcs_map = cpu_to_le16(map);
	}
	vht_cap->vht_mcs.tx_mcs_map = cpu_to_le16(map);
	for (i = 0; i < band->n_channels; i++) {
		band->channels[i].flags |= IEEE80211_CHAN_NO_160MHZ;
		if (!bw80)
			band->channels[i].flags |= IEEE80211_CHAN_NO_80MHZ;
	}
	b43info(dev->wl, "AC-PHY VHT%s: %u stream(s), MCS 0..9, map 0x%04x (untested)\n",
		bw80 ? "20/40/80" : (b43_ac_bw40_enabled() ? "20/40" : "20"),
		nss, map);
}

static int b43_setup_bands(struct b43_wldev *dev,
			   bool have_2ghz_phy, bool have_5ghz_phy)
{
	struct ieee80211_hw *hw = dev->wl->hw;
	struct b43_phy *phy = &dev->phy;
	bool limited_2g;
	bool limited_5g;

	/* We don't support all 2 GHz channels on some devices */
	limited_2g = phy->radio_ver == 0x2057 &&
		     (phy->radio_rev == 9 || phy->radio_rev == 14);
	limited_5g = phy->radio_ver == 0x2057 &&
		     phy->radio_rev == 9;

	if (have_2ghz_phy) {
		if (dev->phy.type == B43_PHYTYPE_N && !limited_2g) {
			b43_n_setup_ht_cap(dev, &b43_band_2GHz_nphy.ht_cap);
			hw->wiphy->bands[NL80211_BAND_2GHZ] =
				&b43_band_2GHz_nphy;
		} else {
			hw->wiphy->bands[NL80211_BAND_2GHZ] = limited_2g ?
				&b43_band_2ghz_limited : &b43_band_2GHz;
		}
	}
	if (dev->phy.type == B43_PHYTYPE_N) {
		if (have_5ghz_phy)
			hw->wiphy->bands[NL80211_BAND_5GHZ] = limited_5g ?
				&b43_band_5GHz_nphy_limited :
				&b43_band_5GHz_nphy;
	} else if (dev->phy.type == B43_PHYTYPE_AC) {
		if (have_5ghz_phy) {
			b43_ac_setup_ht_cap(dev, &b43_band_5GHz_acphy.ht_cap);
			b43_ac_setup_vht_cap(dev, &b43_band_5GHz_acphy);
			hw->wiphy->bands[NL80211_BAND_5GHZ] =
				&b43_band_5GHz_acphy;
		}
	} else {
		if (have_5ghz_phy)
			hw->wiphy->bands[NL80211_BAND_5GHZ] = &b43_band_5GHz_aphy;
	}

	dev->phy.supports_2ghz = have_2ghz_phy;
	dev->phy.supports_5ghz = have_5ghz_phy;

	return 0;
}

static void b43_wireless_core_detach(struct b43_wldev *dev)
{
	/* We release firmware that late to not be required to re-request
	 * is all the time when we reinit the core. */
	b43_release_firmware(dev);
	b43_phy_free(dev);
}

static void b43_supported_bands(struct b43_wldev *dev, bool *have_2ghz_phy,
				bool *have_5ghz_phy)
{
	u16 dev_id = 0;

#ifdef CPTCFG_B43_BCMA
	if (dev->dev->bus_type == B43_BUS_BCMA &&
	    dev->dev->bdev->bus->hosttype == BCMA_HOSTTYPE_PCI)
		dev_id = dev->dev->bdev->bus->host_pci->device;
#endif
#ifdef CPTCFG_B43_SSB
	if (dev->dev->bus_type == B43_BUS_SSB &&
	    dev->dev->sdev->bus->bustype == SSB_BUSTYPE_PCI)
		dev_id = dev->dev->sdev->bus->host_pci->device;
#endif
	/* Override with SPROM value if available */
	if (dev->dev->bus_sprom->dev_id)
		dev_id = dev->dev->bus_sprom->dev_id;

	/*
	 * BCM53573 d11 rev 56 (SoC core, no PCI id; NVRAM sb/1/devid 0x43b6):
	 * on the AC6 this core is the 5 GHz radio, the 2.4 GHz one is the PCIe
	 * BCM43217. The IOST band bits read back 0 on it.
	 */
	if (dev->dev->core_rev == 56) {
		*have_2ghz_phy = false;
		*have_5ghz_phy = true;
		return;
	}

	/* Note: below IDs can be "virtual" (not maching e.g. real PCI ID) */
	switch (dev_id) {
	case 0x4324: /* BCM4306 */
	case 0x4312: /* BCM4311 */
	case 0x4319: /* BCM4318 */
	case 0x4328: /* BCM4321 */
	case 0x432b: /* BCM4322 */
	case 0x4350: /* BCM43222 */
	case 0x4353: /* BCM43224 */
	case 0x0576: /* BCM43224 */
	case 0x435f: /* BCM6362 */
	case 0x4331: /* BCM4331 */
	case 0x4359: /* BCM43228 */
	case 0x43a0: /* BCM4360 */
	case 0x43b1: /* BCM4352 */
		/* Dual band devices */
		*have_2ghz_phy = true;
		*have_5ghz_phy = true;
		return;
	case 0x4321: /* BCM4306 */
		/* There are 14e4:4321 PCI devs with 2.4 GHz BCM4321 (N-PHY) */
		if (dev->phy.type != B43_PHYTYPE_G)
			break;
		fallthrough;
	case 0x4313: /* BCM4311 */
	case 0x431a: /* BCM4318 */
	case 0x432a: /* BCM4321 */
	case 0x432d: /* BCM4322 */
	case 0x4352: /* BCM43222 */
	case 0x435a: /* BCM43228 */
	case 0x4333: /* BCM4331 */
	case 0x43a2: /* BCM4360 */
	case 0x43b3: /* BCM4352 */
		/* 5 GHz only devices */
		*have_2ghz_phy = false;
		*have_5ghz_phy = true;
		return;
	}

	/* As a fallback, try to guess using PHY type */
	switch (dev->phy.type) {
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LP:
	case B43_PHYTYPE_HT:
	case B43_PHYTYPE_LCN:
		*have_2ghz_phy = true;
		*have_5ghz_phy = false;
		return;
	}

	B43_WARN_ON(1);
}

static int b43_wireless_core_attach(struct b43_wldev *dev)
{
	struct b43_wl *wl = dev->wl;
	struct b43_phy *phy = &dev->phy;
	int err;
	u32 tmp;
	bool have_2ghz_phy = false, have_5ghz_phy = false;

	/* Do NOT do any device initialization here.
	 * Do it in wireless_core_init() instead.
	 * This function is for gathering basic information about the HW, only.
	 * Also some structs may be set up here. But most likely you want to have
	 * that in core_init(), too.
	 */

	err = b43_bus_powerup(dev, 0);
	if (err) {
		b43err(wl, "Bus powerup failed\n");
		goto out;
	}

	phy->do_full_init = true;

	/* Try to guess supported bands for the first init needs */
	switch (dev->dev->bus_type) {
#ifdef CPTCFG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOST);
		have_2ghz_phy = !!(tmp & B43_BCMA_IOST_2G_PHY);
		have_5ghz_phy = !!(tmp & B43_BCMA_IOST_5G_PHY);
		break;
#endif
#ifdef CPTCFG_B43_SSB
	case B43_BUS_SSB:
		if (dev->dev->core_rev >= 5) {
			tmp = ssb_read32(dev->dev->sdev, SSB_TMSHIGH);
			have_2ghz_phy = !!(tmp & B43_TMSHIGH_HAVE_2GHZ_PHY);
			have_5ghz_phy = !!(tmp & B43_TMSHIGH_HAVE_5GHZ_PHY);
		} else
			B43_WARN_ON(1);
		break;
#endif
	}

	dev->phy.gmode = have_2ghz_phy;
	b43_wireless_core_reset(dev, dev->phy.gmode);

	/* Get the PHY type. */
	err = b43_phy_versioning(dev);
	if (err)
		goto err_powerdown;

	/* Get real info about supported bands */
	b43_supported_bands(dev, &have_2ghz_phy, &have_5ghz_phy);

	/* We don't support 5 GHz on some PHYs yet */
	if (have_5ghz_phy) {
		switch (dev->phy.type) {
		case B43_PHYTYPE_G:
		case B43_PHYTYPE_LP:
		case B43_PHYTYPE_HT:
			b43warn(wl, "5 GHz band is unsupported on this PHY\n");
			have_5ghz_phy = false;
		}
	}

	if (!have_2ghz_phy && !have_5ghz_phy) {
		b43err(wl, "b43 can't support any band on this device\n");
		err = -EOPNOTSUPP;
		goto err_powerdown;
	}

	err = b43_phy_allocate(dev);
	if (err)
		goto err_powerdown;

	dev->phy.gmode = have_2ghz_phy;
	b43_wireless_core_reset(dev, dev->phy.gmode);

	err = b43_validate_chipaccess(dev);
	if (err)
		goto err_phy_free;
	err = b43_setup_bands(dev, have_2ghz_phy, have_5ghz_phy);
	if (err)
		goto err_phy_free;

	/* Now set some default "current_dev" */
	if (!wl->current_dev)
		wl->current_dev = dev;
	INIT_WORK(&dev->restart_work, b43_chip_reset);

	dev->phy.ops->switch_analog(dev, 0);
	b43_device_disable(dev, 0);
	b43_bus_may_powerdown(dev);

out:
	return err;

err_phy_free:
	b43_phy_free(dev);
err_powerdown:
	b43_bus_may_powerdown(dev);
	return err;
}

static void b43_one_core_detach(struct b43_bus_dev *dev)
{
	struct b43_wldev *wldev;

	/* Do not cancel ieee80211-workqueue based work here.
	 * See comment in b43_remove(). */

	wldev = b43_bus_get_wldev(dev);
	b43_debugfs_remove_device(wldev);
	b43_wireless_core_detach(wldev);
	list_del(&wldev->list);
	b43_bus_set_wldev(dev, NULL);
	kfree(wldev);
}

static int b43_one_core_attach(struct b43_bus_dev *dev, struct b43_wl *wl)
{
	struct b43_wldev *wldev;
	int err = -ENOMEM;

	wldev = kzalloc(sizeof(*wldev), GFP_KERNEL);
	if (!wldev)
		goto out;

	wldev->rx_antenna = B43_ANTENNA_DEFAULT;
	wldev->tx_antenna = B43_ANTENNA_DEFAULT;
	wldev->use_pio = b43_modparam_pio;
	wldev->dev = dev;
	wldev->wl = wl;
	b43_set_status(wldev, B43_STAT_UNINIT);
	wldev->bad_frames_preempt = modparam_bad_frames_preempt;
	INIT_LIST_HEAD(&wldev->list);

	err = b43_wireless_core_attach(wldev);
	if (err)
		goto err_kfree_wldev;

	b43_bus_set_wldev(dev, wldev);
	b43_debugfs_add_device(wldev);

      out:
	return err;

      err_kfree_wldev:
	kfree(wldev);
	return err;
}

#define IS_PDEV(pdev, _vendor, _device, _subvendor, _subdevice)		( \
	(pdev->vendor == PCI_VENDOR_ID_##_vendor) &&			\
	(pdev->device == _device) &&					\
	(pdev->subsystem_vendor == PCI_VENDOR_ID_##_subvendor) &&	\
	(pdev->subsystem_device == _subdevice)				)

#ifdef CPTCFG_B43_SSB
static void b43_sprom_fixup(struct ssb_bus *bus)
{
	struct pci_dev *pdev;

	/* boardflags workarounds */
	if (bus->boardinfo.vendor == SSB_BOARDVENDOR_DELL &&
	    bus->chip_id == 0x4301 && bus->sprom.board_rev == 0x74)
		bus->sprom.boardflags_lo |= B43_BFL_BTCOEXIST;
	if (bus->boardinfo.vendor == PCI_VENDOR_ID_APPLE &&
	    bus->boardinfo.type == 0x4E && bus->sprom.board_rev > 0x40)
		bus->sprom.boardflags_lo |= B43_BFL_PACTRL;
	if (bus->bustype == SSB_BUSTYPE_PCI) {
		pdev = bus->host_pci;
		if (IS_PDEV(pdev, BROADCOM, 0x4318, ASUSTEK, 0x100F) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320,    DELL, 0x0003) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320,      HP, 0x12f8) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, LINKSYS, 0x0015) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, LINKSYS, 0x0014) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, LINKSYS, 0x0013) ||
		    IS_PDEV(pdev, BROADCOM, 0x4320, MOTOROLA, 0x7010))
			bus->sprom.boardflags_lo &= ~B43_BFL_BTCOEXIST;
	}
}

static void b43_wireless_exit(struct b43_bus_dev *dev, struct b43_wl *wl)
{
	struct ieee80211_hw *hw = wl->hw;

	ssb_set_devtypedata(dev->sdev, NULL);
	ieee80211_free_hw(hw);
}
#endif

static struct b43_wl *b43_wireless_init(struct b43_bus_dev *dev)
{
	struct ssb_sprom *sprom = dev->bus_sprom;
	struct ieee80211_hw *hw;
	struct b43_wl *wl;
	char chip_name[6];
	int queue_num;

	hw = ieee80211_alloc_hw(sizeof(*wl), &b43_hw_ops);
	if (!hw) {
		b43err(NULL, "Could not allocate ieee80211 device\n");
		return ERR_PTR(-ENOMEM);
	}
	wl = hw_to_b43_wl(hw);

	/* fill hw info */
	ieee80211_hw_set(hw, RX_INCLUDES_FCS);
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, MFP_CAPABLE);
	/*
	 * acphy_ampdu bit 0, RX block ack sessions. The PHY type is not
	 * known yet; core rev >= 40 is the AC core. The reorder buffer is
	 * capped at the RX ring (B43_RXRING_SLOTS, 32 after OpenWrt's 813)
	 * where wl offers 64 (wlc_ampdu_rx_attach 0x146258..0x146288), so a
	 * burst cannot outrun the ring.
	 */
	if ((b43_modparam_acphy_ampdu & (B43_AC_AMPDU_RX | B43_AC_AMPDU_TX)) &&
	    dev->core_rev >= 40) {
		ieee80211_hw_set(hw, AMPDU_AGGREGATION);
		hw->max_rx_aggregation_subframes = B43_RXRING_SLOTS;
	}
	/*
	 * acphy_fastpath bit 3, A-MSDU TX. mac80211 aggregates MSDUs of a
	 * station and TID into one MPDU on its fast-xmit path (tx.c
	 * ieee80211_amsdu_aggregate; up to the peer's HT max A-MSDU length)
	 * and, as b43 lacks TX_FRAG_LIST, linearizes the result before
	 * b43_op_tx (ieee80211_tx_dequeue). The TX DMA descriptor takes up to
	 * 8191 bytes; the AC header's frame length is 16 bits. No effect with
	 * software crypto: fast-xmit refuses it until hardware CCMP exists.
	 */
	if ((b43_modparam_acphy_fastpath & B43_AC_FASTPATH_AMSDU) &&
	    dev->core_rev >= 40)
		ieee80211_hw_set(hw, TX_AMSDU);
	/*
	 * acphy_psfix bit 4: the hardware PMQ reports a station's power
	 * save state (b43_ac_pmq_drain), so mac80211 does not track the PM
	 * bit of received frames and leaves PS-Poll and U-APSD trigger frames
	 * to b43_rx (b43_ac_psfix_rx). wl's model (wlc_bmac_processpmq).
	 */
	if ((b43_modparam_acphy_psfix & B43_AC_PSFIX_LINKPS) &&
	    dev->core_rev >= 40)
		ieee80211_hw_set(hw, AP_LINK_PS);
	/*
	 * acphy_ampdu bit 1, TX block ack sessions: the negotiated
	 * window per TID lives in the station's drv_priv (b43_op_ampdu_action,
	 * xmit.c b43_ac_ampdu_txhdr). mac80211 caps an aggregate at its
	 * default of 64 subframes; the header tail carries b43's own limit.
	 */
	if ((((b43_modparam_acphy_ampdu & B43_AC_AMPDU_TX) &&
	      (b43_modparam_acphy_ampdu & B43_AC_AMPDU_SESSION)) ||
	     (b43_modparam_acphy_psfix & B43_AC_PSFIX_AMT) ||
	     b43_modparam_acphy_hwcrypto) &&	/* the key's AMT entry */
	    dev->core_rev >= 40)
		hw->sta_data_size = sizeof(struct b43_ac_sta);
	hw->wiphy->interface_modes =
		BIT(NL80211_IFTYPE_AP) |
		BIT(NL80211_IFTYPE_MESH_POINT) |
		BIT(NL80211_IFTYPE_STATION) |
		BIT(NL80211_IFTYPE_ADHOC);

	hw->wiphy->flags |= WIPHY_FLAG_IBSS_RSN;

	wiphy_ext_feature_set(hw->wiphy, NL80211_EXT_FEATURE_CQM_RSSI_LIST);

	hw->wiphy->available_antennas_rx = 0x3;
	hw->wiphy->available_antennas_tx = 0x3;

	wl->hw_registered = false;
	hw->max_rates = 2;
	SET_IEEE80211_DEV(hw, dev->dev);
	if (is_valid_ether_addr(sprom->et1mac))
		SET_IEEE80211_PERM_ADDR(hw, sprom->et1mac);
	else
		SET_IEEE80211_PERM_ADDR(hw, sprom->il0mac);

	/* Initialize struct b43_wl */
	wl->hw = hw;
	mutex_init(&wl->mutex);
	spin_lock_init(&wl->hardirq_lock);
	spin_lock_init(&wl->beacon_lock);
	INIT_WORK(&wl->beacon_update_trigger, b43_beacon_update_trigger_work);
	INIT_WORK(&wl->txpower_adjust_work, b43_phy_txpower_adjust_work);
	INIT_WORK(&wl->tx_work, b43_tx_work);

	/* Initialize queues and flags. */
	for (queue_num = 0; queue_num < B43_QOS_QUEUE_NUM; queue_num++) {
		skb_queue_head_init(&wl->tx_queue[queue_num]);
		wl->tx_queue_stopped[queue_num] = false;
	}

	snprintf(chip_name, ARRAY_SIZE(chip_name),
		 (dev->chip_id > 0x9999) ? "%d" : "%04X", dev->chip_id);
	b43info(wl, "Broadcom %s WLAN found (core revision %u)\n", chip_name,
		dev->core_rev);
	return wl;
}

#ifdef CPTCFG_B43_BCMA
/*
 * BCM53573 (Tenda AC6): d11 core rev 56, AC-PHY rev 24, radio 20693 (id
 * 0x3eb) rev 23. Port in progress: the core is only touched with ac56=1,
 * so a hang never turns into a boot loop. Unit 1 is the second MAC of the
 * same 2x2 PHY and is never attached on its own.
 */
/*
 * Debug: read AC-PHY rev 24 / radio 20693 state on the BCM53573 5 GHz core,
 * to diff against the same reads under wl (casca). Reading a PHY address
 * that does not exist on this core is a bus error (external abort), so the
 * addresses come from userspace, one line per call:
 *   echo "P 001 002 ..."   PHY registers   -> "PD P <addr> <val>"
 *   echo "R 001 002 ..."   radio registers -> "PD R <addr> <val>"
 *   echo "T <id> <off> <n>" PHY table (16 bit) -> "PD T<id> <off> <val>"
 * to /sys/module/b43/parameters/phydump (hex). Each line is printed with
 * pr_warn right after its read, so after an abort the last line on the
 * serial console names the address before the bad one.
 */
static void b43_ac56_phydump(struct b43_wldev *dev, const char *val)
{
	char kind = *val;
	unsigned long a, id, off, n;
	const char *p = val + 1;
	char *end;

	if (kind == 'T' || kind == 'W') {
		id = simple_strtoul(skip_spaces(p), &end, 16);
		off = simple_strtoul(skip_spaces(end), &end, 16);
		n = simple_strtoul(skip_spaces(end), &end, 16);
		{
			/* MAC suspended, PHY clock forced, RxFeCtrl1 disable_stalls:
			 * the tables in use by the hardware (RFSEQ, FEM, TX gain)
			 * bus-fault when read from outside with the MAC running. */
			u16 s19e, lo, hi;

			b43_mac_suspend(dev);
			b43_phy_force_clock(dev, true);
			s19e = b43_phy_read(dev, 0x19e);
			b43_phy_write(dev, 0x19e, s19e | 0x2);
			for (a = 0; a < n; a++) {
				b43_phy_write(dev, 0x0d, id);
				b43_phy_write(dev, 0x0e, off + a);
				lo = b43_phy_read(dev, 0x0f);
				hi = kind == 'W' ? b43_phy_read(dev, 0x10) : 0;
				pr_warn("PD T%lu %03lx %04x %04x\n", id, off + a, lo, hi);
			}
			b43_phy_write(dev, 0x19e, s19e);
			b43_phy_force_clock(dev, false);
			b43_mac_enable(dev);
		}
		return;
	}
	if (kind != 'P' && kind != 'R')
		return;
	for (;;) {
		while (*p == ' ')
			p++;
		if (!isxdigit(*p))
			break;
		a = simple_strtoul(p, &end, 16);
		p = end;
		if (kind == 'P')
			pr_warn("PD P %03lx %04x\n", a, b43_phy_read(dev, a));
		else
			pr_warn("PD R %03lx %04x\n", a, b43_radio_read(dev, a));
	}
}

static int b43_phydump_set(const char *val, const struct kernel_param *kp)
{
	struct b43_wldev *dev = b43_ac56_dumpdev;

	if (!dev)
		return -ENODEV;
	mutex_lock(&dev->wl->mutex);
	if (b43_status(dev) >= B43_STAT_INITIALIZED) {
		b43_mac_suspend(dev);
		b43_ac56_phydump(dev, val);
		b43_mac_enable(dev);
	}
	mutex_unlock(&dev->wl->mutex);
	return 0;
}

static const struct kernel_param_ops b43_phydump_ops = {
	.set = b43_phydump_set,
};
module_param_cb(phydump, &b43_phydump_ops, NULL, 0200);
MODULE_PARM_DESC(phydump, "write \"P|R <hex addr>...\" or \"T <id> <off> <n>\": print rev 56 PHY/radio/table values");

static int modparam_ac56;
module_param_named(ac56, modparam_ac56, int, 0444);
MODULE_PARM_DESC(ac56, "BCM53573 d11 rev 56 (5 GHz): attach (port in progress)");

static int b43_bcma_probe(struct bcma_device *core)
{
	struct b43_bus_dev *dev;
	struct b43_wl *wl;
	int err;

	if (core->id.rev == 0x38) {
		if (!modparam_ac56 || core->core_unit != 0)
			return -ENODEV;
		pr_info("b43: BCM53573 d11 rev 56 unit 0 (ac56=1)\n");
	}

	if (!modparam_allhwsupport &&
	    (core->id.rev == 0x17 || core->id.rev == 0x18)) {
		pr_err("Support for cores revisions 0x17 and 0x18 disabled by module param allhwsupport=0. Try b43.allhwsupport=1\n");
		return -ENOTSUPP;
	}

	dev = b43_bus_dev_bcma_init(core);
	if (!dev)
		return -ENODEV;

	wl = b43_wireless_init(dev);
	if (IS_ERR(wl)) {
		err = PTR_ERR(wl);
		goto bcma_out;
	}

	err = b43_one_core_attach(dev, wl);
	if (err)
		goto bcma_err_wireless_exit;

	/* setup and start work to load firmware */
	INIT_WORK(&wl->firmware_load, b43_request_firmware);
	schedule_work(&wl->firmware_load);

	return err;

bcma_err_wireless_exit:
	ieee80211_free_hw(wl->hw);
bcma_out:
	kfree(dev);
	return err;
}

static void b43_bcma_remove(struct bcma_device *core)
{
	struct b43_wldev *wldev = bcma_get_drvdata(core);
	struct b43_wl *wl = wldev->wl;

	/* We must cancel any work here before unregistering from ieee80211,
	 * as the ieee80211 unreg will destroy the workqueue. */
	cancel_work_sync(&wldev->restart_work);
	cancel_work_sync(&wl->firmware_load);

	B43_WARN_ON(!wl);
	if (!wldev->fw.ucode.data)
		return;			/* NULL if firmware never loaded */
	if (wl->current_dev == wldev && wl->hw_registered) {
		b43_leds_stop(wldev);
		ieee80211_unregister_hw(wl->hw);
	}

	b43_one_core_detach(wldev->dev);

	/* Unregister HW RNG driver */
	b43_rng_exit(wl);

	b43_leds_unregister(wl);
	ieee80211_free_hw(wl->hw);
	kfree(wldev->dev);
}

static struct bcma_driver b43_bcma_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= b43_bcma_tbl,
	.probe		= b43_bcma_probe,
	.remove		= b43_bcma_remove,
};
#endif

#ifdef CPTCFG_B43_SSB
static
int b43_ssb_probe(struct ssb_device *sdev, const struct ssb_device_id *id)
{
	struct b43_bus_dev *dev;
	struct b43_wl *wl;
	int err;

	dev = b43_bus_dev_ssb_init(sdev);
	if (!dev)
		return -ENOMEM;

	wl = ssb_get_devtypedata(sdev);
	if (wl) {
		b43err(NULL, "Dual-core devices are not supported\n");
		err = -ENOTSUPP;
		goto err_ssb_kfree_dev;
	}

	b43_sprom_fixup(sdev->bus);

	wl = b43_wireless_init(dev);
	if (IS_ERR(wl)) {
		err = PTR_ERR(wl);
		goto err_ssb_kfree_dev;
	}
	ssb_set_devtypedata(sdev, wl);
	B43_WARN_ON(ssb_get_devtypedata(sdev) != wl);

	err = b43_one_core_attach(dev, wl);
	if (err)
		goto err_ssb_wireless_exit;

	/* setup and start work to load firmware */
	INIT_WORK(&wl->firmware_load, b43_request_firmware);
	schedule_work(&wl->firmware_load);

	return err;

err_ssb_wireless_exit:
	b43_wireless_exit(dev, wl);
err_ssb_kfree_dev:
	kfree(dev);
	return err;
}

static void b43_ssb_remove(struct ssb_device *sdev)
{
	struct b43_wl *wl = ssb_get_devtypedata(sdev);
	struct b43_wldev *wldev = ssb_get_drvdata(sdev);
	struct b43_bus_dev *dev = wldev->dev;

	/* We must cancel any work here before unregistering from ieee80211,
	 * as the ieee80211 unreg will destroy the workqueue. */
	cancel_work_sync(&wldev->restart_work);
	cancel_work_sync(&wl->firmware_load);

	B43_WARN_ON(!wl);
	if (!wldev->fw.ucode.data)
		return;			/* NULL if firmware never loaded */
	if (wl->current_dev == wldev && wl->hw_registered) {
		b43_leds_stop(wldev);
		ieee80211_unregister_hw(wl->hw);
	}

	b43_one_core_detach(dev);

	/* Unregister HW RNG driver */
	b43_rng_exit(wl);

	b43_leds_unregister(wl);
	b43_wireless_exit(dev, wl);
	kfree(dev);
}

static struct ssb_driver b43_ssb_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= b43_ssb_tbl,
	.probe		= b43_ssb_probe,
	.remove		= b43_ssb_remove,
};
#endif /* CPTCFG_B43_SSB */

/* Perform a hardware reset. This can be called from any context. */
void b43_controller_restart(struct b43_wldev *dev, const char *reason)
{
	/* Must avoid requeueing, if we are in shutdown. */
	if (b43_status(dev) < B43_STAT_INITIALIZED)
		return;
	b43info(dev->wl, "Controller RESET (%s) ...\n", reason);
	ieee80211_queue_work(dev->wl->hw, &dev->restart_work);
}

static void b43_print_driverinfo(void)
{
	const char *feat_pci = "", *feat_nphy = "",
		   *feat_leds = "", *feat_sdio = "";

#ifdef CPTCFG_B43_PCI_AUTOSELECT
	feat_pci = "P";
#endif
#ifdef CPTCFG_B43_PHY_N
	feat_nphy = "N";
#endif
#ifdef CPTCFG_B43_LEDS
	feat_leds = "L";
#endif
#ifdef CPTCFG_B43_SDIO
	feat_sdio = "S";
#endif
	printk(KERN_INFO "Broadcom 43xx driver loaded "
	       "[ Features: %s%s%s%s ]\n",
	       feat_pci, feat_nphy, feat_leds, feat_sdio);
}

void b43trace_init(void);
void b43trace_exit(void);

static int __init b43_init(void)
{
	b43trace_init();
	int err;

	b43_debugfs_init();
	err = b43_sdio_init();
	if (err)
		goto err_dfs_exit;
#ifdef CPTCFG_B43_BCMA
	err = bcma_driver_register(&b43_bcma_driver);
	if (err)
		goto err_sdio_exit;
#endif
#ifdef CPTCFG_B43_SSB
	err = ssb_driver_register(&b43_ssb_driver);
	if (err)
		goto err_bcma_driver_exit;
#endif
	b43_print_driverinfo();

	return err;

#ifdef CPTCFG_B43_SSB
err_bcma_driver_exit:
#endif
#ifdef CPTCFG_B43_BCMA
	bcma_driver_unregister(&b43_bcma_driver);
err_sdio_exit:
#endif
	b43_sdio_exit();
err_dfs_exit:
	b43_debugfs_exit();
	return err;
}

static void __exit b43_exit(void)
{
	b43trace_exit();
#ifdef CPTCFG_B43_SSB
	ssb_driver_unregister(&b43_ssb_driver);
#endif
#ifdef CPTCFG_B43_BCMA
	bcma_driver_unregister(&b43_bcma_driver);
#endif
	b43_sdio_exit();
	b43_debugfs_exit();
}

module_init(b43_init)
module_exit(b43_exit)
