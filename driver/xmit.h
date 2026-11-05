/* SPDX-License-Identifier: GPL-2.0 */
#ifndef B43_XMIT_H_
#define B43_XMIT_H_

#include "main.h"
#include "phy_ac.h"
#include <net/mac80211.h>


#define _b43_declare_plcp_hdr(size) \
	struct b43_plcp_hdr##size {		\
		union {				\
			__le32 data;		\
			__u8 raw[size];		\
		} __packed;	\
	} __packed

/* struct b43_plcp_hdr4 */
_b43_declare_plcp_hdr(4);
/* struct b43_plcp_hdr6 */
_b43_declare_plcp_hdr(6);

#undef _b43_declare_plcp_hdr

/* TX header for v4 firmware */
struct b43_txhdr {
	__le32 mac_ctl;			/* MAC TX control */
	__le16 mac_frame_ctl;		/* Copy of the FrameControl field */
	__le16 tx_fes_time_norm;	/* TX FES Time Normal */
	__le16 phy_ctl;			/* PHY TX control */
	__le16 phy_ctl1;		/* PHY TX control word 1 */
	__le16 phy_ctl1_fb;		/* PHY TX control word 1 for fallback rates */
	__le16 phy_ctl1_rts;		/* PHY TX control word 1 RTS */
	__le16 phy_ctl1_rts_fb;		/* PHY TX control word 1 RTS for fallback rates */
	__u8 phy_rate;			/* PHY rate */
	__u8 phy_rate_rts;		/* PHY rate for RTS/CTS */
	__u8 extra_ft;			/* Extra Frame Types */
	__u8 chan_radio_code;		/* Channel Radio Code */
	__u8 iv[16];			/* Encryption IV */
	__u8 tx_receiver[6];		/* TX Frame Receiver address */
	__le16 tx_fes_time_fb;		/* TX FES Time Fallback */
	struct b43_plcp_hdr6 rts_plcp_fb; /* RTS fallback PLCP header */
	__le16 rts_dur_fb;		/* RTS fallback duration */
	struct b43_plcp_hdr6 plcp_fb;	/* Fallback PLCP header */
	__le16 dur_fb;			/* Fallback duration */
	__le16 mimo_modelen;		/* MIMO mode length */
	__le16 mimo_ratelen_fb;		/* MIMO fallback rate length */
	__le32 timeout;			/* Timeout */

	union {
		/* Tested with 598.314, 644.1001 and 666.2 */
		struct {
			__le16 mimo_antenna;            /* MIMO antenna select */
			__le16 preload_size;            /* Preload size */
			PAD_BYTES(2);
			__le16 cookie;                  /* TX frame cookie */
			__le16 tx_status;               /* TX status */
			__le16 max_n_mpdus;
			__le16 max_a_bytes_mrt;
			__le16 max_a_bytes_fbr;
			__le16 min_m_bytes;
			struct b43_plcp_hdr6 rts_plcp;  /* RTS PLCP header */
			__u8 rts_frame[16];             /* The RTS frame (if used) */
			PAD_BYTES(2);
			struct b43_plcp_hdr6 plcp;      /* Main PLCP header */
		} format_598 __packed;

		/* Tested with 410.2160, 478.104 and 508.* */
		struct {
			__le16 mimo_antenna;		/* MIMO antenna select */
			__le16 preload_size;		/* Preload size */
			PAD_BYTES(2);
			__le16 cookie;			/* TX frame cookie */
			__le16 tx_status;		/* TX status */
			struct b43_plcp_hdr6 rts_plcp;	/* RTS PLCP header */
			__u8 rts_frame[16];		/* The RTS frame (if used) */
			PAD_BYTES(2);
			struct b43_plcp_hdr6 plcp;	/* Main PLCP header */
		} format_410 __packed;

		/* Tested with 351.126 */
		struct {
			PAD_BYTES(2);
			__le16 cookie;			/* TX frame cookie */
			__le16 tx_status;		/* TX status */
			struct b43_plcp_hdr6 rts_plcp;	/* RTS PLCP header */
			__u8 rts_frame[16];		/* The RTS frame (if used) */
			PAD_BYTES(2);
			struct b43_plcp_hdr6 plcp;	/* Main PLCP header */
		} format_351 __packed;

	} __packed;
} __packed;

struct b43_tx_legacy_rate_phy_ctl_entry {
	u8 bitrate;
	u16 coding_rate;
	u16 modulation;
};

/* MAC TX control */
#define B43_TXH_MAC_RTS_FB_SHORTPRMBL	0x80000000 /* RTS fallback preamble */
#define B43_TXH_MAC_RTS_SHORTPRMBL	0x40000000 /* RTS main rate preamble */
#define B43_TXH_MAC_FB_SHORTPRMBL	0x20000000 /* Main fallback preamble */
#define B43_TXH_MAC_USEFBR		0x10000000 /* Use fallback rate for this AMPDU */
#define B43_TXH_MAC_KEYIDX		0x0FF00000 /* Security key index */
#define B43_TXH_MAC_KEYIDX_SHIFT	20
#define B43_TXH_MAC_ALT_TXPWR		0x00080000 /* Use alternate txpwr defined at loc. M_ALT_TXPWR_IDX */
#define B43_TXH_MAC_KEYALG		0x00070000 /* Security key algorithm */
#define B43_TXH_MAC_KEYALG_SHIFT	16
#define B43_TXH_MAC_AMIC		0x00008000 /* AMIC */
#define B43_TXH_MAC_RIFS		0x00004000 /* Use RIFS */
#define B43_TXH_MAC_LIFETIME		0x00002000 /* Lifetime */
#define B43_TXH_MAC_FRAMEBURST		0x00001000 /* Frameburst */
#define B43_TXH_MAC_SENDCTS		0x00000800 /* Send CTS-to-self */
#define B43_TXH_MAC_AMPDU		0x00000600 /* AMPDU status */
#define  B43_TXH_MAC_AMPDU_MPDU		0x00000000 /* Regular MPDU, not an AMPDU */
#define  B43_TXH_MAC_AMPDU_FIRST	0x00000200 /* First MPDU or AMPDU */
#define  B43_TXH_MAC_AMPDU_INTER	0x00000400 /* Intermediate MPDU or AMPDU */
#define  B43_TXH_MAC_AMPDU_LAST		0x00000600 /* Last (or only) MPDU of AMPDU */
#define B43_TXH_MAC_40MHZ		0x00000100 /* Use 40 MHz bandwidth */
#define B43_TXH_MAC_5GHZ		0x00000080 /* 5GHz band */
#define B43_TXH_MAC_DFCS		0x00000040 /* DFCS */
#define B43_TXH_MAC_IGNPMQ		0x00000020 /* Ignore PMQ */
#define B43_TXH_MAC_HWSEQ		0x00000010 /* Use Hardware Sequence Number */
#define B43_TXH_MAC_STMSDU		0x00000008 /* Start MSDU */
#define B43_TXH_MAC_SENDRTS		0x00000004 /* Send RTS */
#define B43_TXH_MAC_LONGFRAME		0x00000002 /* Long frame */
#define B43_TXH_MAC_ACK			0x00000001 /* Immediate ACK */

/* Extra Frame Types */
#define B43_TXH_EFT_FB			0x03 /* Data frame fallback encoding */
#define  B43_TXH_EFT_FB_CCK		0x00 /* CCK */
#define  B43_TXH_EFT_FB_OFDM		0x01 /* OFDM */
#define  B43_TXH_EFT_FB_HT		0x02 /* HT */
#define  B43_TXH_EFT_FB_VHT		0x03 /* VHT */
#define B43_TXH_EFT_RTS			0x0C /* RTS/CTS encoding */
#define  B43_TXH_EFT_RTS_CCK		0x00 /* CCK */
#define  B43_TXH_EFT_RTS_OFDM		0x04 /* OFDM */
#define  B43_TXH_EFT_RTS_HT		0x08 /* HT */
#define  B43_TXH_EFT_RTS_VHT		0x0C /* VHT */
#define B43_TXH_EFT_RTSFB		0x30 /* RTS/CTS fallback encoding */
#define  B43_TXH_EFT_RTSFB_CCK		0x00 /* CCK */
#define  B43_TXH_EFT_RTSFB_OFDM		0x10 /* OFDM */
#define  B43_TXH_EFT_RTSFB_HT		0x20 /* HT */
#define  B43_TXH_EFT_RTSFB_VHT		0x30 /* VHT */

/* PHY TX control word */
#define B43_TXH_PHY_ENC			0x0003 /* Data frame encoding */
#define  B43_TXH_PHY_ENC_CCK		0x0000 /* CCK */
#define  B43_TXH_PHY_ENC_OFDM		0x0001 /* OFDM */
#define  B43_TXH_PHY_ENC_HT		0x0002 /* HT */
#define  B43_TXH_PHY_ENC_VHT		0x0003 /* VHT */
#define B43_TXH_PHY_SHORTPRMBL		0x0010 /* Use short preamble */
#define B43_TXH_PHY_ANT			0x03C0 /* Antenna selection */
#define  B43_TXH_PHY_ANT0		0x0000 /* Use antenna 0 */
#define  B43_TXH_PHY_ANT1		0x0040 /* Use antenna 1 */
#define  B43_TXH_PHY_ANT01AUTO		0x00C0 /* Use antenna 0/1 auto */
#define  B43_TXH_PHY_ANT2		0x0100 /* Use antenna 2 */
#define  B43_TXH_PHY_ANT3		0x0200 /* Use antenna 3 */
#define B43_TXH_PHY_TXPWR		0xFC00 /* TX power */
#define B43_TXH_PHY_TXPWR_SHIFT		10

/* PHY TX control word 1 */
#define B43_TXH_PHY1_BW			0x0007 /* Bandwidth */
#define  B43_TXH_PHY1_BW_10		0x0000 /* 10 MHz */
#define  B43_TXH_PHY1_BW_10U		0x0001 /* 10 MHz upper */
#define  B43_TXH_PHY1_BW_20		0x0002 /* 20 MHz */
#define  B43_TXH_PHY1_BW_20U		0x0003 /* 20 MHz upper */
#define  B43_TXH_PHY1_BW_40		0x0004 /* 40 MHz */
#define  B43_TXH_PHY1_BW_40DUP		0x0005 /* 40 MHz duplicate */
#define B43_TXH_PHY1_MODE		0x0038 /* Mode */
#define  B43_TXH_PHY1_MODE_SISO		0x0000 /* SISO */
#define  B43_TXH_PHY1_MODE_CDD		0x0008 /* CDD */
#define  B43_TXH_PHY1_MODE_STBC		0x0010 /* STBC */
#define  B43_TXH_PHY1_MODE_SDM		0x0018 /* SDM */
#define B43_TXH_PHY1_CRATE		0x0700 /* Coding rate */
#define  B43_TXH_PHY1_CRATE_1_2		0x0000 /* 1/2 */
#define  B43_TXH_PHY1_CRATE_2_3		0x0100 /* 2/3 */
#define  B43_TXH_PHY1_CRATE_3_4		0x0200 /* 3/4 */
#define  B43_TXH_PHY1_CRATE_4_5		0x0300 /* 4/5 */
#define  B43_TXH_PHY1_CRATE_5_6		0x0400 /* 5/6 */
#define  B43_TXH_PHY1_CRATE_7_8		0x0600 /* 7/8 */
#define B43_TXH_PHY1_MODUL		0x3800 /* Modulation scheme */
#define  B43_TXH_PHY1_MODUL_BPSK	0x0000 /* BPSK */
#define  B43_TXH_PHY1_MODUL_QPSK	0x0800 /* QPSK */
#define  B43_TXH_PHY1_MODUL_QAM16	0x1000 /* QAM16 */
#define  B43_TXH_PHY1_MODUL_QAM64	0x1800 /* QAM64 */
#define  B43_TXH_PHY1_MODUL_QAM256	0x2000 /* QAM256 */


/*
 * AC-PHY TX header (ucode 907.113, d11ucode42), 124 bytes.
 *
 * Decoded from wl.ko 6.37.15.24 `wlc_d11hdrs` @0x11b360, AC path (PHY type
 * 11 at 0x11b384). wl pushes and
 * zeroes 124 bytes (skb_push/__memzero 0x7c at 0x11b480/0x11b48c): a 20 byte
 * packet part, four 20 byte rate entries (primary first, then fallbacks;
 * stride 20 at 0x11bb74) and a 24 byte tail. Fields named "unk" or covered by
 * PAD_BYTES have no writer in the decode and stay zero.
 */
struct b43_txhdr_ac_rate {
	__le16 phy_ctl0;	/* +0  wlc_acphy_txctl0_calc @0x1135a0, store 0x11c8bc */
	__le16 phy_ctl1;	/* +2  wlc_acphy_txctl1_calc @0x114254, store 0x11c964 */
	__le16 phy_ctl2;	/* +4  wlc_acphy_txctl2_calc @0x114a24, store 0x11c978 */
	struct b43_plcp_hdr6 plcp; /* +6 wlc_compute_plcp, call 0x11c328 */
	__le16 fb_bw;		/* +12 fallback bandwidth word, 0x11bfa0..0x11bfbc */
	__le16 rate;		/* +14 rate in 500 kbit/s, 0x11c330..0x11c360 */
	__le16 rts_ctl;		/* +16 RTS/CTS control, 0x11bb94 and 0x11c5d8.. */
	__le16 txbf;		/* +18 beamforming word, 0x11c204..0x11c23c */
} __packed;

struct b43_txhdr_ac {
	PAD_BYTES(2);		/* 0..1: no writer in the AC path */
	__le16 mac_ctl_lo;	/* 2..3: 0x11cafc, 0x11cb08 */
	__le16 mac_ctl_hi;	/* 4..5: 0x11cb10, 0x11cb18 */
	__le16 chanspec;	/* 6..7: wlc+0x548, 0x11b53c/0x11b544 */
	__u8 hdr_len;		/* 8: 802.11 header length, 0x11b594 */
	PAD_BYTES(1);		/* 9: no writer */
	__le16 frame_len;	/* 10..11: frame length + FCS, 0x11b5a0/0x11b5a8 */
	__le16 frame_id;	/* 12..13: 0x11cb20/0x11cb28 */
	__le16 seq_ctl;		/* 14..15: 802.11 seq control, 0x11b754/0x11b75c */
	__le16 unk16;		/* 16..17: only with wl packet flag 0x200 (0x11b760..0x11b780) */
	__le16 unk18;		/* 18..19: zeroed at 0x11b798/0x11b79c */
	struct b43_txhdr_ac_rate rate[4]; /* 20..99 */
	__u8 key_algo;		/* 100: key byte +12 << 4, 0x11cc18/0x11cc20 */
	__u8 key_idx;		/* 101: key byte +6, 0x11cc24/0x11cc2c */
	/*
	 * the A-MPDU tail, written by _wlc_sendampdu_aqm for an
	 * aggregate-able MPDU with the full 124 byte header (0xde78..0xdec0),
	 * read by ucode 907 @0x38F..0x3A3 only with mac_ctl_lo bit 6 set.
	 */
	__u8 ampdu_max_mpdu[2];	/* 102, 103: both scb_ampdu max_pdu (0xde8c..0xde98) */
	__le16 ampdu_dur_dens;	/* 104..105: max duration & 0xfff0 | density (0xde9c..0xdeb4) */
	__u8 ampdu_wsize;	/* 106: block ack window - 1 (0xdeb8..0xdec0) */
	PAD_BYTES(17);		/* 107..123: no writer in wlc_d11hdrs */
} __packed;

/* AC MAC control, low word (bytes 2..3), accumulated at wl sp+24 */
#define B43_TXH_AC_MAC_AMPDU		0x0040 /* aggregate-able MPDU, the AQM joins
						* consecutive ones of equal epoch
						* (_wlc_sendampdu_aqm 0xde4c set, 0xdd38
						* clear; wlc_txh_get_isAMPDU 0x2cdc) */
#define B43_TXH_AC_MAC_ACK		0x0080 /* Unicast, ACK expected (0x11b4dc..0x11b530) */
#define B43_TXH_AC_MAC_IPMQ		0x0200 /* ignore the PMQ, send to a dozing station
						* (wl: beacons and PS-Poll / APSD responses,
						* 0x11b490..0x11b4c0; ucode 907 @0x03D1 skips
						* the suppression L247 with it set) */
#define B43_TXH_AC_MAC_HWSEQ		0x0800 /* ucode assigns the sequence number (0x11b688..0x11b69c) */
#define B43_TXH_AC_MAC_STMSDU		0x4000 /* Fragment number 0 (0x11b4c4..0x11b4d8) */
/*
 * Other bits wl sets here, left clear by b43: 0x0200 for beacons or an scb
 * flag (0x11b490..0x11b4c0); 0x1000 with packet flag 0x200 (0x11b784);
 * 0x8400 on the frame-burst path (wlc_d11hdrs 0x11c364..0x11c418, gated by
 * wlc byte 0x201; _wlc_sendampdu_aqm clears 0x0400 again at 0xe08c..
 * 0xe0ac); 0x0400 frameburst-related (0x11c558..0x11c5bc).
 */

/* AC MAC control, high word (bytes 4..5), accumulated at wl sp+32 */
#define B43_TXH_AC_MACH_NORATESEL	0x0002 /* Set on every frame, cleared only when
						* wlc_scb_ratesel_gettxrate filled 2 or 4
						* rate entries (0x11b9b0..0x11b9cc) */
#define B43_TXH_AC_MACH_VHT		0x0004 /* Primary rate is VHT (0x11c5c0..0x11c5d4).
						* _wlc_sendampdu_aqm sets it on a
						* non-aggregated MPDU (0xdd40) and clears
						* it on an aggregate-able one (0xde50): a
						* lone VHT MPDU goes out as a one-MPDU
						* A-MPDU, a real aggregate does not */

/* AC per-rate PHY control word 0 (wlc_acphy_txctl0_calc @0x1135a0) */
#define B43_TXH_AC_PHY0_FT		0x0003 /* Frame type (0x1135a8..0x1135fc) */
#define  B43_TXH_AC_PHY0_FT_CCK		0x0000
#define  B43_TXH_AC_PHY0_FT_OFDM	0x0001
#define  B43_TXH_AC_PHY0_FT_HT		0x0002
#define  B43_TXH_AC_PHY0_FT_VHT		0x0003
#define B43_TXH_AC_PHY0_SET		0x0004 /* Always set (0x113660) */
#define B43_TXH_AC_PHY0_TXBF		0x0008 /* Beamforming (0x11c850) */
#define B43_TXH_AC_PHY0_SHORTPRMBL	0x0010 /* Preamble 1 or 2 (0x113600..0x113614) */
/*
 * TX core mask << 6. For PHY type 11 wlc_stf_d11hdrs_phyctl_txant (0x193284)
 * tail-calls _wlc_stf_phytxchain_sel, which returns wlc_stf_txcore_get() << 6
 * (0x191124..0x19113c): the per rate class core mask (stf+22+2i, i = 0 CCK,
 * 1 OFDM, 2.. by stream count; 0x191078..0x191100). Zero selects no core.
 */
#define B43_TXH_AC_PHY0_ANT		0x03C0
#define B43_TXH_AC_PHY0_ANT_SHIFT	6
#define B43_TXH_AC_PHY0_BW		0xC000 /* TX bandwidth (0x11362c..0x113654) */
#define  B43_TXH_AC_PHY0_BW_20		0x0000
#define  B43_TXH_AC_PHY0_BW_40		0x4000
#define  B43_TXH_AC_PHY0_BW_80		0x8000
#define  B43_TXH_AC_PHY0_BW_160		0xC000

/*
 * AC PHY control 1 (wlc_acphy_txctl1_calc @0x114254): control sideband in
 * bits 0..2, the frame's TX power offset below the target in half dB from
 * bit 3 (0..31, wlc_update_txppr_offset 0x193f04..0x193f0c).
 */
#define B43_TXH_AC_PHY1_PWR_SHIFT	3

/* AC per-rate fallback bandwidth word: bits 0..1 = (fb - 1) & 3, fb = 4
 * (no fallback bandwidth) for every 20 MHz rate (0x11bf20..0x11bfbc) */
#define B43_TXH_AC_FBBW_NONE		0x0003

/* AC frame id (header bytes 12..13): `((n << 5) & 0x7fe0) | (fifo & 7)`
 * (0x11b734..0x11b744); wlc_dotxstatus takes the FIFO from bits 0..2
 * (0x927c..0x9284). b43's DMA ring index is the FIFO number. */
#define B43_TXH_AC_FID_FIFO		0x0007
#define B43_TXH_AC_FID_SLOT		0x7FE0
#define B43_TXH_AC_FID_SLOT_SHIFT	5

/* AC per-rate RTS/CTS control word */
#define B43_TXH_AC_RTS_LAST		0x0020 /* Last rate entry (0x11cac8..0x11caf4) */

/*
 * HT (802.11n) rate in an AC rate entry, from wl.ko's rspec (bits 24..25 = 1
 * for HT, low byte = MCS, bit 20 STBC, bit 22 LDPC, bit 23 short GI):
 * - PHY control 2 = MCS byte, | 0x40 with STBC (wlc_acphy_txctl2_calc
 *   0x114a30..0x114a38, 0x114ac4..0x114ac8).
 * - PLCP = HT-SIG (wlc_compute_plcp HT branch 0x118ca0..0x118cfc):
 *   byte 0 MCS, | 0x80 for 40 MHz or MCS 32; bytes 1..2 the length (frame +
 *   FCS, sp+72 at 0x11c318); byte 3 = 0x07 (smoothing, not sounding,
 *   reserved), | 0x10 STBC, | 0x40 LDPC, | 0x80 short GI; bytes 4..5 zero.
 */
#define B43_TXH_AC_PHY2_STBC		0x0040 /* 0x114ac8 */
#define B43_AC_HTSIG1_CBW40		0x80	/* 0x118cc4..0x118cd0 */
#define B43_AC_HTSIG2_BASE		0x07	/* 0x118cd8 */
#define B43_AC_HTSIG2_STBC1		0x10	/* 0x118cdc */
#define B43_AC_HTSIG2_STBC		0x30	/* wlc_txh_get_isSTBC 0x2de0, recv 0x148d4 */
#define B43_AC_HTSIG2_LDPC		0x40	/* 0x118ce4 */
#define B43_AC_HTSIG2_SGI		0x80	/* 0x118cec */

/*
 * VHT (802.11ac) rate in an AC rate entry, wl's rspec with bits 24..25 = 2:
 * MCS bits 0..3, NSS bits 4..7 (1 based, wlc_ratespec_nss 0x180da4),
 * bandwidth bits 16..18 (1/2/3 = 20/40/80 MHz), STBC 20, LDPC 22, short GI
 * 23.
 * - PHY control 0: frame type 3 (0x1135ac..0x1135b0); VHT forces preamble 4
 *   (0x11bf1c), so bit 4 stays clear; bandwidth 0x8000 at 80 MHz
 *   (0x113634..0x113654).
 * - PHY control 2 = (NSS - 1) << 4 | MCS, | 0x40 with STBC
 *   (wlc_acphy_txctl2_calc 0x114a40..0x114a50).
 * - MAC control high word | 0x0004 (0x11c5c0..0x11c5d4).
 * - PLCP = VHT-SIG-A as wlc_compute_plcp builds it (0x118c04..0x118c9c):
 *   A1 in bytes 0..2 (little endian) = 0x800004 | BW (0..3) | STBC << 3 |
 *   group ID << 4 | (NSTS - 1) << 10, group ID 0 when the frame goes to an
 *   AP (FC ToDS only), else 63; partial AID and TXOP_PS 0. A2 in bytes 3..4
 *   = short GI | LDPC << 2 | MCS << 4 | 0x200 (reserved bit 9). CRC and tail
 *   are left 0 (byte 4 bits 2..7, byte 5): wl does not compute them.
 */
#define B43_AC_VHTSIGA1_BASE		0x800004	/* 0x118c08..0x118c3c */
#define B43_AC_VHTSIGA1_STBC		0x000008	/* 0x118c40..0x118c48 */
#define B43_AC_VHTSIGA1_GID_SHIFT	4		/* 0x118c4c */
#define B43_AC_VHTSIGA1_GID_ALL		63
#define B43_AC_VHTSIGA1_NSTS_SHIFT	10		/* 0x118c50..0x118c5c */
#define B43_AC_VHTSIGA2_SGI		0x0001		/* 0x118c70 */
#define B43_AC_VHTSIGA2_LDPC		0x0004		/* 0x118c78 */
#define B43_AC_VHTSIGA2_MCS_SHIFT	4		/* 0x118c74, 0x118c80 */
#define B43_AC_VHTSIGA2_RES9		0x0200		/* 0x118c84 */

/* acphy_vht (xmit.c, default 0): VHT caps, VHT TX and VHT RX reporting */
extern bool b43_modparam_ac_vht;

/*
 * Spatial streams b43 offers for HT on the AC-PHY (0 = HT off), limited
 * by the acphy_htnss module parameter and the SPROM tx/rx chains.
 */
unsigned int b43_ac_ht_nss(struct b43_wldev *dev);
/* HT TX STBC advertised and sent (acphy_htstbc) */
bool b43_ac_ht_tx_stbc(struct b43_wldev *dev);
/*
 * acphy_mimo_log (xmit.c): true for the first N TX statuses of HT
 * frames with 2 or more streams (MCS 8 and up at rate entry 0), by the
 * header at the head of the FIFO or by the frame's control block; the
 * dma.c status lines then print even with acphy_fastpath bit 0.
 */
bool b43_ac_mimo_log_hdr(struct b43_wldev *dev,
			 const struct b43_txhdr_ac *hdr);
bool b43_ac_mimo_log_info(struct b43_wldev *dev,
			  const struct ieee80211_tx_info *info);
/*
 * acphy_mimo_unfl (xmit.c): the TX FIFO underflow counter of the
 * status' FIFO (ucode 907 shm 0xEC + 2 * FIFO) and suppress reason 6,
 * logged as TXUNFL with the status of the aggregate.
 */
struct b43_txstatus;
void b43_ac_unfl_status(struct b43_wldev *dev,
			const struct b43_txstatus *status,
			const struct b43_txhdr_ac *hdr, unsigned int nframes);

/*
 * acphy_ampdu (xmit.c, bitmask, default 0), A-MPDU on core rev >= 40:
 * bit 0 RX block ack sessions, bit 1
 * TX (hardware AQM aggregation), bit 2 wl's A-MPDU shm words, bit 3
 * 64 MPDUs per aggregate instead of 32, bit 4 aggregate only
 * in bursts: a lone MPDU goes out non-aggregated and the DMA of a burst
 * is started once the whole burst is on the ring (xmit.c
 * b43_ac_ampdu_txhdr, dma.c dma_tx_fragment, main.c b43_tx_work).
 */
#define B43_AC_AMPDU_RX		0x1
#define B43_AC_AMPDU_TX		0x2
#define B43_AC_AMPDU_SHM	0x4
#define B43_AC_AMPDU_TX64	0x8
#define B43_AC_AMPDU_BURST	0x10
/*
 * what bit 1 used to do unconditionally, each on its own bit, after
 * the 2026-10-07 16:58/17:10 trials (TX mode broke ordinary unicast
 * frames with no aggregate ever sent):
 * bit 5 the host flag (B43_HF_AC_AMPDUTX, shm 0x62 bit 6), bit 6 accept
 * TX block ack sessions (and the station drv_priv), bit 7 the A-MPDU
 * header fields (AMPDU bit, tail, epoch) for session frames, bit 8 the
 * ucode's own sequence numbers (MAC control low bit 11) for QoS data of a
 * session instead of mac80211's, a diagnostic. Bit 1 alone keeps the
 * AMPDU_AGGREGATION hw flag, the status-side code and the logs.
 */
#define B43_AC_AMPDU_HF		0x20
#define B43_AC_AMPDU_SESSION	0x40
#define B43_AC_AMPDU_TXHDR	0x80
#define B43_AC_AMPDU_HWSEQ	0x100
/*
 * bit 9, the rate entries' counts of an A-MPDU status reported to
 * minstrel_ht as transmission rounds (mac80211's convention) instead of
 * MPDUs transmitted (the ucode's words), which booked a 19-MPDU aggregate
 * as 361 attempts (dma.c b43_dma_ampdu_rc).
 */
#define B43_AC_AMPDU_RCFIX	0x200
/*
 * bit 10, the status of a session frame sent plain (every rate probe,
 * and a lone MPDU with bit 4) reported as a one-MPDU aggregate
 * (IEEE80211_TX_STAT_AMPDU, ampdu_len 1), as mt76 does; without it
 * minstrel_ht drops the status (IEEE80211_TX_CTL_AMPDU without
 * STAT_AMPDU) and never learns a rate above the one the pre-session
 * probes hit (dma.c b43_dma_plain_report).
 */
#define B43_AC_AMPDU_PLAINRC	0x400
extern unsigned int b43_modparam_acphy_ampdu;

static inline bool b43_ac_ampdu(struct b43_wldev *dev, unsigned int bits)
{
	return (b43_modparam_acphy_ampdu & bits) && dev->dev->core_rev >= 40;
}

/* would `next` join an aggregate burst with `skb` (xmit.c)? */
bool b43_ac_ampdu_same_burst(struct sk_buff *skb, struct sk_buff *next);

/*
 * acphy_psfix (bitmask, default 0), AP power save on core rev >= 40.
 * 1 = "PSFIX" log lines for mac80211's view (sta_notify sleep/awake,
 *     set_tim, PS-Poll / null data frames received with their PM bit);
 * 2 = the hardware power management queue (PMQ): MACCTL DISCPMQ cleared
 *     for an AP as wl does, B43_IRQ_PMQ drained and logged, a frame the
 *     ucode suppressed for a dozing station (reason 1) reported as
 *     IEEE80211_TX_STAT_TX_FILTERED, and IEEE80211_TX_CTL_NO_PS_BUFFER /
 *     CLEAR_PS_FILT frames sent with the header's ignore-PMQ bit;
 * 4 = AP_LINK_PS: the PMQ entries drive ieee80211_sta_ps_transition and
 *     b43_rx hands PS-Poll and U-APSD trigger frames to mac80211 itself
 *     (wl's model: wlc_bmac_processpmq 0x12180 -> wlc_apps_process_ps_switch
 *     0x11fa0). Implies 2.
 */
#define B43_AC_PSFIX_LOG	0x1
#define B43_AC_PSFIX_PMQ	0x2
#define B43_AC_PSFIX_LINKPS	0x4
/*
 * 8 = an address match table (AMT) entry per associated station
 *     (b43_op_sta_add / b43_op_sta_remove, indices 0..61; 62 and 63 are the
 *     BSSID and our MAC), attribute acphy_psamt (default 0x8004 =
 *     valid | match A2, the entry wl gives a station's hardware key:
 *     ucode 907 @0x0AA8..0x0AAE turns an A2 match index into a key slot).
 *     ucode 907 looks a received frame's A2 up (@0x0A34..0x0A38) and queues
 *     a PMQ entry for every frame while the lookup says "not found"
 *     (@0x0C40), and its TX suppression (@0x03D4..0x03D6) is keyed on the
 *     lookup result too; without an entry no frame was ever suppressed
 *     (2026-10-07 16:28 runs). Implies 2.
 */
#define B43_AC_PSFIX_AMT	0x8
/*
 * 16 = hold the PMQ entries while a station dozes (stage 3, after the
 *     2026-10-07 17:28 runs with bit 8: still one PMQ entry per received
 *     frame and no suppression). The lookup the ucode makes for every
 *     received frame (@0x0A34..0x0A38, command 4 / pattern type 0x13 on
 *     the PMQ registers spr1F0..spr1F7) and the one its TX path relies on
 *     (@0x03D4..0x03D6) search the PMQ's own entries, not the AMT: an
 *     entry answers "found" only while it is in the queue, and b43 deleted
 *     every entry right after reading it (as wl does only once its data
 *     FIFO drained: wlc_bmac_process_ps_switch 0x1210c writes the delete
 *     for ps_on < 0, and wlc_apps_process_ps_switch 0x120e4..0x120f8 makes
 *     ps_on negative only when wlc byte 0x384 bit 3, the "PS pending"
 *     state, is clear). With the entries deleted the next frame is "not
 *     found" again (@0x0C40 -> L685 queues it) and the TX path never sees
 *     a dozing station. This bit deletes the processed entries only when
 *     no station is dozing (per the entries themselves), on a deauth
 *     entry, or when the queue overflows. Implies 2.
 */
#define B43_AC_PSFIX_HOLD	0x10
extern unsigned int b43_modparam_acphy_psfix;
extern unsigned int b43_modparam_acphy_psamt;
bool b43_ac_psfix_log(void);

static inline bool b43_ac_psfix(struct b43_wldev *dev, unsigned int bits)
{
	if (bits & (B43_AC_PSFIX_LINKPS | B43_AC_PSFIX_AMT | B43_AC_PSFIX_HOLD))
		bits |= B43_AC_PSFIX_PMQ;
	return (b43_modparam_acphy_psfix & bits) == bits &&
	       dev->dev->core_rev >= 40;
}

/* ucode 907 TX status suppress reason (s1 bits 4..7): PMQ, L247 @0x0409 */
#define B43_TXST40_SUPR_PMQ	1

/*
 * acphy_hwcrypto (bitmask, default 0): keys in the hardware on core
 * rev >= 40 with the AC header (b43.h, B43_AC_HWKEY_*). Every frame was
 * CCMP'd in software until now, and the router's CPU is the ~35 Mbit/s
 * ceiling (76 % system time on an iperf run, 2026-10-08).
 * 1 = CCMP: a pairwise key in slot 4 + its station's AMT entry (shared
 *     with acphy_psfix bit 8, b43_ac_amt_sta_get), a group key in slot
 *     keyidx (0..3). mac80211 writes the CCMP header into the frame
 *     (IEEE80211_KEY_FLAG_GENERATE_IV, as wl's host does: wlc_dofrag ->
 *     wlc_key_iv_update; _wlc_txh_iv_upd 0x11456c returns without touching
 *     the header for a non-TKIP key on rev > 39), the ucode encrypts and
 *     appends the MIC (frame length + icv_len, wlc_d11hdrs 0x11b46c..
 *     0x11b478), header bytes 100 = algo_hw << 4 and 101 = slot
 *     (0x11cc18..0x11cc2c). Received frames the ucode decrypted (mac
 *     status DEC without DECERR) keep their IV and MIC and are flagged
 *     RX_FLAG_DECRYPTED, as on the legacy path. Protected management
 *     frames stay in software (IEEE80211_KEY_FLAG_SW_MGMT_TX).
 * 2 = WEP40/WEP104 the same way (slot keyidx, host flag USEDEFKEYS as
 *     wlc_key_defkeyflag 0x172df4 sets it for WEP default keys).
 * 4 = leave ucode 907's PMQ power save on for a keyed station. Without
 *     it, a unicast frame to a station whose AMT entry exists for its key
 *     alone (acphy_psfix bit 8 not asked for) carries the ignore-PMQ bit:
 *     the entry makes the ucode's A2 lookup succeed, and from then on its
 *     TX path suppresses frames while the PMQ holds the station dozing
 *     (@0x03D4..0x03D6, acphy_psfix), power save mac80211 is handling in
 *     software at the same time. On air (2026-10-08 03:11..03:16 UTC,
 *     the laptop toggling PM every second or two: 19 doze/wake pairs in
 *     35 s) that cut the downlink to 15.6 Mbit/s with no MAC failure and
 *     no drop at the laptop, against 36..40 with software CCMP and no
 *     entry; the uplink went from 49 to 76 Mbit/s.
 * TKIP always stays in software: this wl never gives it to the hardware
 * (wlc_key_hw_init 0x172e4c..0x172e5c). Anything else: -EOPNOTSUPP,
 * mac80211 encrypts in software as before. Lines "HWCRYPTO ..." report
 * each slot written or cleared (slot, algo, AMT entry) and the first
 * decrypted frames (mac status key index, A2).
 */
#define B43_AC_HWCRYPTO_CCMP	0x1
#define B43_AC_HWCRYPTO_WEP	0x2
#define B43_AC_HWCRYPTO_PMQ	0x4
#define B43_AC_HWCRYPTO_RXLOGS	16
extern int b43_modparam_acphy_hwcrypto;

static inline bool b43_ac_hwcrypto(struct b43_wldev *dev, int bits)
{
	/* b43_txhdr_is_ac (below), spelled out: it is defined after this */
	return (b43_modparam_acphy_hwcrypto & bits) &&
	       dev->dev->core_rev >= 40 && dev->phy.type == B43_PHYTYPE_AC;
}

/* Users of a station's AMT entry (b43_ac_sta.amt_users) */
#define B43_AC_AMT_USER_PS	0x1	/* acphy_psfix bit 8 */
#define B43_AC_AMT_USER_KEY	0x2	/* its pairwise hardware key */

/*
 * acphy_fastpath (xmit.c, bitmask, default 0), the per-frame CPU cost
 * of the core rev >= 40 path (the ~40 % system time per 40 Mbit/s that is
 * not software CCMP, 2026-10-08). Each bit is one independent cut:
 * 1 QUIET: no per-frame diagnostics: the capped TXSUB/TXHDR, TXST40, HTTX,
 *     HTRX, BADRX, AMPDURX, AMPDUTX, RATESEL lines and their atomics, the
 *     RXFT counters with their line every 512 frames (a 100-character line
 *     on a 115200 baud serial console is ~9 ms of busy wait), the TXST40
 *     PSM dump and the per-interrupt MACCTL read ("PSM not running").
 *     acphy_regdump=1 (first HT TX) is part of the HTTX dump and so is off.
 * 2 TXBATCH: one TX worker kick per interrupt instead of one per TX
 *     status, and only when a status unblocked a software queue (every
 *     other frame already has its own kick from b43_op_tx).
 * 4 IRQPOLL: before the interrupt mask is restored, re-read and
 *     acknowledge the interrupt reasons under hardirq_lock and handle them
 *     in the same thread pass (up to B43_AC_IRQPOLL_ROUNDS): under load
 *     events arrive while the pass runs, and each would otherwise cost a
 *     hard interrupt (6 PCIe reads, 7 writes) and a thread wake-up.
 * 8 AMSDU: IEEE80211_HW_TX_AMSDU, mac80211 builds A-MSDUs (up to the
 *     peer's 3839 or 7935 bytes) on its fast-xmit path and linearizes
 *     them for a driver without TX_FRAG_LIST; fewer MPDUs per byte. The
 *     fast-xmit path refuses software crypto (tx.c "don't handle software
 *     crypto"), so this bit only acts with hardware CCMP or no encryption.
 * 16 RXSYNC: cache maintenance of an RX buffer over the received bytes
 *     only. b43 maps 4143 bytes (B43_DMA0_RX_FW598_REV40_BUFSIZE) at setup,
 *     then syncs and unmaps the whole buffer per frame: three passes of
 *     130 L1 lines and 130 PL310 outer-cache operations on this SoC. With
 *     the bit: the RX header, then len + frameoffset, and the unmap skips
 *     its pass (DMA_ATTR_SKIP_CPU_SYNC), the idiom of the ethernet drivers.
 * 32 RXLIST: ieee80211_rx_napi's steps around the whole ring drain: bottom
 *     halves off once, ieee80211_rx_list per frame, one
 *     netif_receive_skb_list for the batch (ieee80211_rx_ni does this per
 *     frame).
 */
#define B43_AC_FASTPATH_QUIET	0x01
#define B43_AC_FASTPATH_TXBATCH	0x02
#define B43_AC_FASTPATH_IRQPOLL	0x04
#define B43_AC_FASTPATH_AMSDU	0x08
#define B43_AC_FASTPATH_RXSYNC	0x10
#define B43_AC_FASTPATH_RXLIST	0x20
/* IRQPOLL: passes per interrupt before the mask is restored regardless */
#define B43_AC_IRQPOLL_ROUNDS	8
extern unsigned int b43_modparam_acphy_fastpath;
/*
 * acphy_rxlazy (uint, default 0 = b43's 0x01000000, one frame per RX
 * interrupt): the value for B43_MMIO_INTRCVLAZY0 at chip init, frames << 24
 * | timeout. The timeout unit is not established (core clocks or
 * microseconds): 0x04000080 = 4 frames or 128 units is the first trial.
 */
extern unsigned int b43_modparam_acphy_rxlazy;

static inline bool b43_ac_fastpath(struct b43_wldev *dev, unsigned int bits)
{
	return (b43_modparam_acphy_fastpath & bits) && dev->dev->core_rev >= 40;
}

/*
 * acphy_htdead (int, default 0). About two loads in three come up
 * with HT TX dead: every HT frame the core sends fails all its attempts
 * while legacy frames are acked, the state is drawn inside
 * b43_wireless_core_init and a controller restart re-draws it with
 * P(live) ~ 1/3. With 1, the TX
 * status path counts HT frames per core init and asks for
 * b43_controller_restart when acphy_htdead_n HT frames each failed all
 * their attempts and none was acked since the init, at most
 * acphy_htdead_max times per chain, 1 s apart. xmit.c.
 */
extern int b43_modparam_acphy_htdead;
void b43_ac_htdead_txstatus(struct b43_wldev *dev,
			    const struct ieee80211_tx_info *report,
			    const struct b43_txstatus *status);
void b43_ac_htdead_core_init(struct b43_wldev *dev);
void b43_ac_htdead_chain_reset(struct b43_wldev *dev);

/*
 * A-MPDU TX limits as wl's AQM path writes them into the header tail
 * (_wlc_sendampdu_aqm 0xde84..0xdec0). Max MPDUs per aggregate: wl's
 * tunables for core rev > 39 are 32, 48 or 64 (wlc_ampdu_tx_attach
 * 0x143374..0x143398), picked per peer by scb_ampdu_update_config
 * 0x140b18 from capability bits it does not name; b43 takes 32, or 64 with
 * acphy_ampdu bit 3, capped by the negotiated window. Max duration 5416 us
 * (ampdu_tx+0x2a, 0x1434fc), its low nibble replaced by the peer's MPDU
 * density code (0xdea4..0xdea8). Block ack window: the negotiated one, wl's
 * default 64 (ampdu_tx+0x13, 0x1434a4..0x1434bc).
 */
#define B43_AC_AMPDU_MAX_MPDU		32
#define B43_AC_AMPDU_MAX_MPDU64		64
#define B43_AC_AMPDU_MAX_DUR		5416
#define B43_AC_AMPDU_DENS_MASK		0x000F
#define B43_AC_AMPDU_WSIZE_DEF		64

/*
 * per station A-MPDU TX state (ieee80211_sta drv_priv, allocated
 * with acphy_ampdu bit 1): the negotiated block ack window per TID from
 * IEEE80211_AMPDU_TX_OPERATIONAL, 0 while no session is operational. wl
 * keeps it in ini[tid] byte 1 (0xdeb8).
 */
struct b43_ac_sta {
	u8 ampdu_wsize[IEEE80211_NUM_TIDS];
	u8 amt_idx;	/* AMT entry + 1, 0 = none (acphy_psfix bit 8) */
	u8 amt_users;	/* B43_AC_AMT_USER_* holding the entry */
};

/*
 * epoch state per TX FIFO (wl: ampdu_tx + 0x278 + 28 * fifo). The
 * AQM joins consecutive aggregate-able MPDUs of one FIFO while their epoch
 * bit (TSO prefix byte 2 bit 1, wlc_txh_set_epoch 0x2d04) is equal, so
 * the host toggles the bit (wlc_ampdu_change_epoch 0x141bc4, mode 3:
 * 0x141bec..0x141c00) whenever the next MPDU must not join: a rate probe
 * (0xded0..0xdef8), another station or TID (0xdefc..0xdf20), another frame
 * type, PHY control word or HT-SIG/VHT-SIG (0xdf40..0xe00c), a sequence
 * number that is not the one expected (0xe020..0xe03c), and once for a
 * non-aggregate frame after aggregates (0xdd68..0xdda4). What
 * wlc_ampdu_chgnsav_epoch 0x141c20 saves (0x141c60..0x141ce8) is kept
 * in struct b43_ac_ampdu_epoch (b43.h, one per FIFO in struct b43_wldev):
 * station (+4), TID (+8), frame type 1 HT / 2 VHT / 0 none (+9), PHY
 * control 0 and 1 (+10, +12), PLCP bytes 0 and 3 (+14, +15; VHT also 1, 2,
 * 4 at +17, +16, +18).
 */
#define B43_AC_AMPDU_FT_HT		1	/* 0x141c7c..0x141c84 */
#define B43_AC_AMPDU_FT_VHT		2
#define B43_TXH_AC_TOE_EPOCH		0x02	/* prefix byte 2 (0x2d0c..0x2d1c) */

/* AC chanspec (wlc+0x548): channel in bits 0..7 (the center channel at
 * 40 and 80 MHz), sideband 0x0700 (40 MHz: 0x0100 = control channel in the
 * upper half; 80 MHz: 0..3 from the lowest 20 MHz, wf_chspec_ctlchan
 * @0x7840: control = center - (bw - 20) / 10 + 4 * sb), bandwidth 0x3800
 * (0x1000 = 20 MHz, 0x1800 = 40 MHz, 0x2000 = 80 MHz, 0x11b39c..0x11b3c4),
 * band 0xc000 (0 = 2 GHz, 0xc000 = 5 GHz, 0x4b9b8) */
#define B43_AC_CHANSPEC_BW		0x3800
#define B43_AC_CHANSPEC_BW_20		0x1000
#define B43_AC_CHANSPEC_BW_40		0x1800
#define B43_AC_CHANSPEC_BW_80		0x2000
#define B43_AC_CHANSPEC_SB		0x0700
#define B43_AC_CHANSPEC_SB_SHIFT	8
#define B43_AC_CHANSPEC_BAND_2G		0x0000
#define B43_AC_CHANSPEC_BAND_5G		0xC000

/* 40 MHz on the AC PHY: the acphy_bw40 module parameter of phy_ac.c */
static inline bool b43_ac_bw40_enabled(void)
{
#ifdef CPTCFG_B43_PHY_AC
	return b43_modparam_ac_bw40;
#else
	return false;
#endif
}

static inline bool b43_ac_chanspec_is40(u16 chanspec)
{
	return (chanspec & B43_AC_CHANSPEC_BW) == B43_AC_CHANSPEC_BW_40;
}

/* 80 MHz on the AC PHY: acphy_bw80 of phy_ac.c */
static inline bool b43_ac_bw80_enabled(void)
{
#ifdef CPTCFG_B43_PHY_AC
	return b43_modparam_ac_bw80;
#else
	return false;
#endif
}

static inline bool b43_ac_chanspec_is80(u16 chanspec)
{
	return (chanspec & B43_AC_CHANSPEC_BW) == B43_AC_CHANSPEC_BW_80;
}

/*
 * The control channel of an AC chanspec, wf_chspec_ctlchan @0x7840:
 * center + 4 * sb - (bw_mhz - 20) / 10, bw_mhz from .rodata 0x3038
 * (0x7884..0x78b4): 20 MHz the channel byte, 40 MHz center - 2 + 4 sb,
 * 80 MHz center - 6 + 4 sb.
 */
static inline unsigned int b43_ac_chanspec_ctlchan(u16 chanspec)
{
	unsigned int ch = chanspec & 0xFF;
	unsigned int sb = (chanspec & B43_AC_CHANSPEC_SB) >>
			  B43_AC_CHANSPEC_SB_SHIFT;

	if (b43_ac_chanspec_is40(chanspec))
		return ch - 2 + 4 * sb;
	if (b43_ac_chanspec_is80(chanspec))
		return ch - 6 + 4 * sb;
	return ch;
}

/*
 * PHY control word 1 sideband bits, wlc_acphy_txctl1_calc.clone.18
 * @0x114254: ((chanspec >> 8) & 7) >> (bw - 1), bw the rate's bandwidth code
 * (1 = 20, 2 = 40, 3 = 80 MHz). A 20 MHz frame in a 40 or 80 MHz channel
 * carries the control sideband, a 40 MHz frame in an 80 MHz channel the
 * 40 MHz half (sb >> 1), a frame as wide as the channel 0.
 * rate40 is b43_ac_phyctl1_sb_bw(chanspec, 1).
 */
static inline u16 b43_ac_phyctl1_sb_bw(u16 chanspec, unsigned int shift)
{
	return ((chanspec & B43_AC_CHANSPEC_SB) >> B43_AC_CHANSPEC_SB_SHIFT) >>
	       shift;
}

static inline u16 b43_ac_phyctl1_sb(u16 chanspec, bool rate40)
{
	return b43_ac_phyctl1_sb_bw(chanspec, rate40 ? 1 : 0);
}

/*
 * The chanspec as wl keeps it at wlc+0x548 for a channel in the current
 * band: the AC PHY's (20 MHz, 40 MHz with acphy_bw40 and a 40 MHz chandef,
 * 80 MHz with acphy_bw80 and an 80 MHz chandef, b43_phy_ac_chanspec in
 * phy_ac.c), else 20 MHz.
 */
static inline u16 b43_ac_chanspec(struct b43_wldev *dev, unsigned int channel)
{
	u16 chanspec;

#ifdef CPTCFG_B43_PHY_AC
	if (dev->phy.type == B43_PHYTYPE_AC)
		return b43_phy_ac_chanspec(dev, channel);
#endif
	chanspec = (channel & 0xFF) | B43_AC_CHANSPEC_BW_20;

	if (b43_current_band(dev->wl) == NL80211_BAND_5GHZ)
		chanspec |= B43_AC_CHANSPEC_BAND_5G;
	else
		chanspec |= B43_AC_CHANSPEC_BAND_2G;
	return chanspec;
}

/*
 * Header format choice. wl.ko builds the AC header when the PHY type is 11
 * (cmp r3, #11 at 0x11b384) and its legacy 6 + 112 byte header otherwise
 * (0x11cc98), so the TX header follows the PHY type, not the core or
 * firmware rev. b43 only gives PHY type AC to core rev 40 and 42.
 */
static inline bool b43_txhdr_is_ac(struct b43_wldev *dev)
{
	return dev->phy.type == B43_PHYTYPE_AC;
}

/*
 * TX status format choice. wlc_bmac_txstatus keys on core rev > 39
 * (cmp r3, #39 / bhi at 0x12288), as the RX header change does.
 */
static inline bool b43_txstatus_is_rev40(struct b43_wldev *dev)
{
	return dev->dev->core_rev >= 40;
}

/*
 * ucode 907 TX core mask block, core rev >= 40. wlc_attach stores word base
 * 0x2ea in pub+0xd0 for core rev > 39 (0x13d180..0x13d184); wlc_init copies it
 * to stf+0x2a and calls wlc_stf_txcore_shmem_write(wlc, 1) when N mode is on
 * (0x12f04c..0x12f070), which writes four words at byte offset
 * (0x2ea + i) * 2, i = 0 CCK, 1 OFDM, 2 one stream, 3 two streams, each the
 * core mask & 0xf (0x1911e8..0x191248; no spatial map for PHY type 11).
 * b43 zeroes shared memory before the ucode upload, so without this block
 * the ucode sees core mask 0 for every rate class.
 */
#define B43_SHM_SH_AC_TXCORE		0x05D4	/* 4 words: CCK, OFDM, 1SS, 2SS */
#define B43_SHM_SH_AC_TXCORE_REV56	0x0394	/* same block on ucode56 (wl 9.10) */
#define B43_AC_TXCORE_NWORDS		4

u8 b43_ac_txcore(struct b43_wldev *dev, unsigned int nss);
void b43_ac_txcore_shm_write(struct b43_wldev *dev);

/*
 * TOE (TSO) prefix in front of the AC TX header, core rev > 39.
 *
 * wlc_attach sets wlc+0x7c6 = 1 and wlc+0x7c5 = 0 for core rev > 39
 * (0x13d4fc..0x13d50c). wlc_up clears bit 0 of d11 MMIO 0xa00 when it is set
 * and leaves wlc+0x7c5 = that bit = 0 (0x129bcc..0x129c3c). With 0x7c6 set
 * and 0x7c5 clear, wlc_d11hdrs ends every AC header with
 * wlc_toe_add_hdr (0x11cc54..0x11cc90), which pushes 4 more bytes in front
 * of the 124 byte header: byte 0 = 0x02, bytes 1..3 = 0 (0x199ef4..0x199f30).
 * Every reader skips it: wlc_get_txh_info takes the header at
 * data + wlc_tso_hdr_length(data) for core rev > 39 (wlc_pkt_get_vht_hdr
 * 0x2d50..0x2d78), and wlc_tso_hdr_length is 4, or 16 when byte 0 bit 0 is
 * set (0x199f44..0x199f50). So the hardware takes the first 4 bytes of each
 * TX buffer as this prefix. Without it, the ucode read the 124 byte header
 * 4 bytes late: frame id from our bytes 16..17 and sequence from 18..19,
 * both 0, which is what every TX status reported on hardware.
 */
#define B43_TXH_AC_TOE_LEN		4
#define B43_TXH_AC_TOE_BYTE0		0x02	/* 0x199f1c */

static inline bool b43_txhdr_has_toe(struct b43_wldev *dev)
{
	return b43_txhdr_is_ac(dev) && dev->dev->core_rev >= 40;
}

static inline
size_t b43_txhdr_size(struct b43_wldev *dev)
{
	if (b43_txhdr_is_ac(dev))
		return sizeof(struct b43_txhdr_ac) +
		       (b43_txhdr_has_toe(dev) ? B43_TXH_AC_TOE_LEN : 0);

	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		return 112 + sizeof(struct b43_plcp_hdr6);
	case B43_FW_HDR_410:
		return 104 + sizeof(struct b43_plcp_hdr6);
	case B43_FW_HDR_351:
		return 100 + sizeof(struct b43_plcp_hdr6);
	}
	return 0;
}


int b43_generate_txhdr(struct b43_wldev *dev,
		       u8 * txhdr,
		       struct sk_buff *skb_frag,
		       struct ieee80211_tx_info *txctl, u16 cookie);

/* Transmit Status */
struct b43_txstatus {
	u16 cookie;		/* The cookie from the txhdr */
	u16 seq;		/* Sequence number */
	u8 phy_stat;		/* PHY TX status */
	u8 frame_count;		/* Frame transmit count */
	u8 rts_count;		/* RTS transmit count */
	u8 supp_reason;		/* Suppression reason */
	/* flags */
	u8 pm_indicated;	/* PM mode indicated to AP */
	u8 intermediate;	/* Intermediate status notification (not final) */
	u8 for_ampdu;		/* Status is for an AMPDU (afterburner) */
	u8 acked;		/* Wireless ACK received */
	/*
	 * core rev >= 40 only: MPDUs this status releases from the FIFO
	 * (s1 bits 8..14, wlc_ampdu_dotxstatus 0xe45c..0xe464), the block ack
	 * map by position (t2 for MPDU 0..31, t3 for 32..63, 0xe868..0xe8d8)
	 * and the TX count of each rate entry (s3/s4 bytes 0 and 2).
	 */
	u8 ampdu_cnt;
	u8 rate_cnt[4];
	u64 ampdu_ba;
};

/*
 * Core rev >= 40 TX status: two packages of four words each, read in the
 * order 0x170, 0x174, 0x178, 0x17c, 0x170, 0x174, 0x178, 0x17c
 * (wlc_bmac_txstatus 0x12570, 0x123e0..0x12400, 0x1247c..0x124ac).
 */
#define B43_TXST40_WORDS		4
#define B43_TXST40_S1_VALID		0x00000001 /* 0x12580, second package 0x124c0 */
#define B43_TXST40_S1_INTERMEDIATE	0x00000004 /* 0x12420 */
#define B43_TXST40_S1_PM_INDICATED	0x00000008 /* 0x12428 */
#define B43_TXST40_S1_SUPP_REASON	0x000000F0 /* 0x12440 */
#define B43_TXST40_S1_SUPP_REASON_SHIFT	4
#define B43_TXST40_S1_CNT		0x00007F00 /* 0x12430 */
#define B43_TXST40_S1_CNT_SHIFT		8
#define B43_TXST40_S1_ACKED		0x00008000 /* 0x12438 */

void b43_txstatus_parse_rev40(struct b43_wldev *dev, struct b43_txstatus *stat,
			      const u32 *s, const u32 *t);

/* txstatus supp_reason values */
enum {
	B43_TXST_SUPP_NONE,	/* Not suppressed */
	B43_TXST_SUPP_PMQ,	/* Suppressed due to PMQ entry */
	B43_TXST_SUPP_FLUSH,	/* Suppressed due to flush request */
	B43_TXST_SUPP_PREV,	/* Previous fragment failed */
	B43_TXST_SUPP_CHAN,	/* Channel mismatch */
	B43_TXST_SUPP_LIFE,	/* Lifetime expired */
	B43_TXST_SUPP_UNDER,	/* Buffer underflow */
	B43_TXST_SUPP_ABNACK,	/* Afterburner NACK */
};

/* Receive header for v4 firmware. */
struct b43_rxhdr_fw4 {
	__le16 frame_len;	/* Frame length */
	 PAD_BYTES(2);
	__le16 phy_status0;	/* PHY RX Status 0 */
	union {
		/* RSSI for A/B/G-PHYs */
		struct {
			__u8 jssi;	/* PHY RX Status 1: JSSI */
			__u8 sig_qual;	/* PHY RX Status 1: Signal Quality */
		} __packed;

		/* RSSI for N-PHYs */
		struct {
			__s8 power0;	/* PHY RX Status 1: Power 0 */
			__s8 power1;	/* PHY RX Status 1: Power 1 */
		} __packed;
	} __packed;
	union {
		/* HT-PHY */
		struct {
			PAD_BYTES(1);
			__s8 phy_ht_power0;
		} __packed;

		/* RSSI for N-PHYs */
		struct {
			__s8 power2;
			PAD_BYTES(1);
		} __packed;

		__le16 phy_status2;	/* PHY RX Status 2 */
	} __packed;
	union {
		/* HT-PHY */
		struct {
			__s8 phy_ht_power1;
			__s8 phy_ht_power2;
		} __packed;

		__le16 phy_status3;	/* PHY RX Status 3 */
	} __packed;
	union {
		/* Tested with 598.314, 644.1001 and 666.2 */
		struct {
			__le16 phy_status4;	/* PHY RX Status 4 */
			__le16 phy_status5;	/* PHY RX Status 5 */
			__le32 mac_status;	/* MAC RX status */
			__le16 mac_time;
			__le16 channel;
		} format_598 __packed;

		/* Tested with 351.126, 410.2160, 478.104 and 508.* */
		struct {
			__le32 mac_status;	/* MAC RX status */
			__le16 mac_time;
			__le16 channel;
		} format_351 __packed;
	} __packed;
} __packed;

/* PHY RX Status 0 */
#define B43_RX_PHYST0_GAINCTL		0x4000 /* Gain Control */
#define B43_RX_PHYST0_PLCPHCF		0x0200
#define B43_RX_PHYST0_PLCPFV		0x0100
#define B43_RX_PHYST0_SHORTPRMBL	0x0080 /* Received with Short Preamble */
#define B43_RX_PHYST0_LCRS		0x0040
#define B43_RX_PHYST0_ANT		0x0020 /* Antenna */
#define B43_RX_PHYST0_UNSRATE		0x0010
#define B43_RX_PHYST0_CLIP		0x000C
#define B43_RX_PHYST0_CLIP_SHIFT	2
#define B43_RX_PHYST0_FTYPE		0x0003 /* Frame type */
#define  B43_RX_PHYST0_CCK		0x0000 /* Frame type: CCK */
#define  B43_RX_PHYST0_OFDM		0x0001 /* Frame type: OFDM */
#define  B43_RX_PHYST0_PRE_N		0x0002 /* Pre-standard N-PHY frame */
#define  B43_RX_PHYST0_STD_N		0x0003 /* Standard N-PHY frame */
/* Core rev >= 40 (AC-PHY) frame type values: wlc_recv_compute_rspec
 * 0x14854..0x14870 decodes 0 CCK, 1 OFDM, 2 HT, 3 VHT */
#define  B43_RX_PHYST0_AC_HT		0x0002
#define  B43_RX_PHYST0_AC_VHT		0x0003

/* PHY RX Status 2 */
#define B43_RX_PHYST2_LNAG		0xC000 /* LNA Gain */
#define B43_RX_PHYST2_LNAG_SHIFT	14
#define B43_RX_PHYST2_PNAG		0x3C00 /* PNA Gain */
#define B43_RX_PHYST2_PNAG_SHIFT	10
#define B43_RX_PHYST2_FOFF		0x03FF /* F offset */

/* PHY RX Status 3 */
#define B43_RX_PHYST3_DIGG		0x1800 /* DIG Gain */
#define B43_RX_PHYST3_DIGG_SHIFT	11
#define B43_RX_PHYST3_TRSTATE		0x0400 /* TR state */

/* MAC RX Status */
#define B43_RX_MAC_RXST_VALID		0x01000000 /* PHY RXST valid */
#define B43_RX_MAC_TKIP_MICERR		0x00100000 /* TKIP MIC error */
#define B43_RX_MAC_TKIP_MICATT		0x00080000 /* TKIP MIC attempted */
#define B43_RX_MAC_AGGTYPE		0x00060000 /* Aggregation type */
#define B43_RX_MAC_AGGTYPE_SHIFT	17
#define B43_RX_MAC_AMSDU		0x00010000 /* A-MSDU mask */
#define B43_RX_MAC_BEACONSENT		0x00008000 /* Beacon sent flag */
#define B43_RX_MAC_KEYIDX		0x000007E0 /* Key index */
#define B43_RX_MAC_KEYIDX_SHIFT		5
#define B43_RX_MAC_DECERR		0x00000010 /* Decrypt error */
#define B43_RX_MAC_DEC			0x00000008 /* Decryption attempted */
#define B43_RX_MAC_PADDING		0x00000004 /* Pad bytes present */
#define B43_RX_MAC_RESP			0x00000002 /* Response frame transmitted */
#define B43_RX_MAC_FCSERR		0x00000001 /* FCS error */

/* RX channel */
#define B43_RX_CHAN_40MHZ		0x1000 /* 40 Mhz channel width */
#define B43_RX_CHAN_5GHZ		0x0800 /* 5 Ghz band */
#define B43_RX_CHAN_ID			0x07F8 /* Channel ID */
#define B43_RX_CHAN_ID_SHIFT		3
#define B43_RX_CHAN_PHYTYPE		0x0007 /* PHY type */
/* Core rev >= 40 writes a chanspec instead of a chanstat */
#define B43_RX_CHANSPEC_CHAN		0x00FF /* Channel number */


u8 b43_plcp_get_ratecode_cck(const u8 bitrate);
u8 b43_plcp_get_ratecode_ofdm(const u8 bitrate);

void b43_generate_plcp_hdr(struct b43_plcp_hdr4 *plcp,
			   const u16 octets, const u8 bitrate);

void b43_rx(struct b43_wldev *dev, struct sk_buff *skb, const void *_rxhdr);

void b43_handle_txstatus(struct b43_wldev *dev,
			 const struct b43_txstatus *status);
bool b43_fill_txstatus_report(struct b43_wldev *dev,
			      struct ieee80211_tx_info *report,
			      const struct b43_txstatus *status);

void b43_tx_suspend(struct b43_wldev *dev);
void b43_tx_resume(struct b43_wldev *dev);


/* Helper functions for converting the key-table index from "firmware-format"
 * to "raw-format" and back. The firmware API changed for this at some revision.
 * We need to account for that here. */
static inline int b43_new_kidx_api(struct b43_wldev *dev)
{
	/* FIXME: Not sure the change was at rev 351 */
	return (dev->fw.rev >= 351);
}
static inline u8 b43_kidx_to_fw(struct b43_wldev *dev, u8 raw_kidx)
{
	u8 firmware_kidx;
	if (b43_new_kidx_api(dev)) {
		firmware_kidx = raw_kidx;
	} else {
		if (raw_kidx >= 4)	/* Is per STA key? */
			firmware_kidx = raw_kidx - 4;
		else
			firmware_kidx = raw_kidx;	/* TX default key */
	}
	return firmware_kidx;
}
static inline u8 b43_kidx_to_raw(struct b43_wldev *dev, u8 firmware_kidx)
{
	u8 raw_kidx;
	if (b43_new_kidx_api(dev))
		raw_kidx = firmware_kidx;
	else
		raw_kidx = firmware_kidx + 4;	/* RX default keys or per STA keys */
	return raw_kidx;
}

/* struct b43_private_tx_info - TX info private to b43.
 * The structure is placed in (struct ieee80211_tx_info *)->rate_driver_data
 *
 * @bouncebuffer: DMA Bouncebuffer (if used)
 */
struct b43_private_tx_info {
	void *bouncebuffer;
};

static inline struct b43_private_tx_info *
b43_get_priv_tx_info(struct ieee80211_tx_info *info)
{
	BUILD_BUG_ON(sizeof(struct b43_private_tx_info) >
		     sizeof(info->rate_driver_data));
	return (struct b43_private_tx_info *)info->rate_driver_data;
}

#endif /* B43_XMIT_H_ */
