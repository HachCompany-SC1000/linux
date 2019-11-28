#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/releasenum.h>

static int releasenum_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", LINUX_KERNEL_RELEASE_NUM);
	return 0;
}

static int releasenum_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, releasenum_proc_show, NULL);
}

static const struct file_operations releasenum_proc_fops = {
	.open		= releasenum_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_releasenum_init(void)
{
	proc_create("releasenum", 0, NULL, &releasenum_proc_fops);
	return 0;
}
module_init(proc_releasenum_init);
