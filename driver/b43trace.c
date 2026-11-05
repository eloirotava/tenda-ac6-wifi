// SPDX-License-Identifier: GPL-2.0
/*
 * b43trace.c - rastro das escritas de PHY/radio do b43 (core rev 56).
 * Mesmo formato do wltrace do wl (wphy/wrad), para comparar os dois.
 * /proc/b43trace: leitura = registros; escrita "clear", "on", "off".
 */
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

enum { T_WPHY = 1, T_MPHY, T_APHY, T_OPHY, T_WRAD, T_MRAD, T_ARAD, T_ORAD, T_TBL };
void b43trace_init(void);
void b43trace_exit(void);
void b43trace_add(u16 type, void *pi, u32 addr, u32 mask, u32 val, u32 x, u32 y);

static const char *const tname[] = { "?", "wphy", "mphy", "aphy", "ophy",
	"wrad", "mrad", "arad", "orad", "tbl" };

struct btrec {
	u32 pi;
	u16 type, addr, mask, val;
	u32 x;		/* tabela: id << 16 | width; senao 0 */
	u32 y;		/* tabela: offset << 16 | len */
};

#define BT_N	65536
static struct btrec *bt_buf;
static unsigned int bt_head, bt_count;
static bool bt_on = true;
module_param_named(b43trace_on, bt_on, bool, 0644);
static DEFINE_SPINLOCK(bt_lock);

void b43trace_add(u16 type, void *pi, u32 addr, u32 mask, u32 val, u32 x, u32 y)
{
	unsigned long fl;
	struct btrec *r;

	if (!bt_on || !bt_buf)
		return;
	spin_lock_irqsave(&bt_lock, fl);
	r = &bt_buf[bt_head];
	r->pi = (u32)(unsigned long)pi;
	r->type = type;
	r->addr = addr;
	r->mask = mask;
	r->val = val;
	r->x = x;
	r->y = y;
	bt_head = (bt_head + 1) % BT_N;
	if (bt_count < BT_N)
		bt_count++;
	spin_unlock_irqrestore(&bt_lock, fl);
}

static void *bt_start(struct seq_file *s, loff_t *pos)
{
	return *pos < bt_count ? pos : NULL;
}
static void *bt_next(struct seq_file *s, void *v, loff_t *pos)
{
	(*pos)++;
	return *pos < bt_count ? pos : NULL;
}
static void bt_stop(struct seq_file *s, void *v) { }
static int bt_show(struct seq_file *s, void *v)
{
	loff_t i = *(loff_t *)v;
	struct btrec *r = &bt_buf[(bt_head + BT_N - bt_count + i) % BT_N];

	if (r->type == T_TBL)
		seq_printf(s, "%s %08x id %u w %u off %u len %u\n", tname[r->type], r->pi,
			   r->x >> 16, r->x & 0xffff, r->y >> 16, r->y & 0xffff);
	else
		seq_printf(s, "%s %08x %04x %04x %04x\n", tname[r->type < 10 ? r->type : 0],
			   r->pi, r->addr, r->mask, r->val);
	return 0;
}
static const struct seq_operations bt_sops = {
	.start = bt_start, .next = bt_next, .stop = bt_stop, .show = bt_show,
};
static int bt_open(struct inode *i, struct file *f)
{
	return seq_open(f, &bt_sops);
}
static ssize_t bt_write(struct file *f, const char __user *ub, size_t n, loff_t *p)
{
	char b[8] = { 0 };

	if (copy_from_user(b, ub, min(n, sizeof(b) - 1)))
		return -EFAULT;
	if (!strncmp(b, "clear", 5)) {
		spin_lock_irq(&bt_lock);
		bt_head = bt_count = 0;
		spin_unlock_irq(&bt_lock);
	} else if (!strncmp(b, "on", 2)) {
		bt_on = true;
	} else if (!strncmp(b, "off", 3)) {
		bt_on = false;
	}
	return n;
}
static const struct proc_ops bt_pops = {
	.proc_open = bt_open, .proc_read = seq_read, .proc_lseek = seq_lseek,
	.proc_release = seq_release, .proc_write = bt_write,
};

void b43trace_init(void)
{
	bt_buf = vzalloc(BT_N * sizeof(*bt_buf));
	proc_create("b43trace", 0600, NULL, &bt_pops);
}
void b43trace_exit(void)
{
	remove_proc_entry("b43trace", NULL);
	vfree(bt_buf);
	bt_buf = NULL;
}

