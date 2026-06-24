// daq.c — RPMsg telemetry + gains driver for STM32MP1 A7
// /dev/daq0 : read telemetry (rpmsg_telem_t structs, 16 bytes each)
// /dev/daq1 : write gains   (float Kp, Ki, Kd — 12 bytes)

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rpmsg.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/string.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Atul");
MODULE_DESCRIPTION("STM32MP1 DAQ telemetry + gains RPMsg driver");
MODULE_VERSION("0.4");

/* ------------------------------------------------------------------ */
/* Packet struct — must match M4 exactly                               */
/* ------------------------------------------------------------------ */

struct rpmsg_telem_t {
    uint32_t counter;
    uint32_t timestamp_cycles;
    uint16_t adc_raw;
    uint8_t  padding[6];
} __attribute__((packed));

static_assert(sizeof(struct rpmsg_telem_t) == 16,
              "rpmsg_telem_t size mismatch — check M4 struct");

/* ------------------------------------------------------------------ */
/* kfifo + wait queue for telemetry                                    */
/* ------------------------------------------------------------------ */

#define FIFO_DEPTH 64
DEFINE_KFIFO(telem_fifo, struct rpmsg_telem_t, FIFO_DEPTH);
static DEFINE_SPINLOCK(fifo_lock);
static DECLARE_WAIT_QUEUE_HEAD(daq_wq);

/* ------------------------------------------------------------------ */
/* Globals                                                             */
/* ------------------------------------------------------------------ */

static struct rpmsg_device *telem_rpdev;   /* set at probe, cleared at remove */
static struct rpmsg_device *gains_rpdev;

static dev_t          daq_devno;
static struct cdev    daq0_cdev;
static struct cdev    daq1_cdev;
static struct class  *daq_class;

/* ------------------------------------------------------------------ */
/* File operations — /dev/daq0  (read telemetry)                       */
/* ------------------------------------------------------------------ */

static int daq_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t daq_read(struct file *filp, char __user *buf,
                        size_t count, loff_t *ppos)
{
    size_t max_pkts = count / sizeof(struct rpmsg_telem_t);
    struct rpmsg_telem_t *tmp;
    unsigned int n = 0;
    size_t bytes;

    if (max_pkts == 0)
        return -EINVAL;

    if (wait_event_interruptible(daq_wq, !kfifo_is_empty(&telem_fifo)))
        return -ERESTARTSYS;

    tmp = kmalloc(FIFO_DEPTH * sizeof(struct rpmsg_telem_t), GFP_KERNEL);
    if (!tmp)
        return -ENOMEM;

    while (n < max_pkts) {
        unsigned int got = kfifo_out_spinlocked(&telem_fifo,
                                                &tmp[n], 1,
                                                &fifo_lock);
        if (got == 0)
            break;
        n++;
    }

    if (n == 0) {
        kfree(tmp);
        return 0;
    }

    bytes = n * sizeof(struct rpmsg_telem_t);
    if (copy_to_user(buf, tmp, bytes)) {
        kfree(tmp);
        return -EFAULT;
    }

    kfree(tmp);
    return bytes;
}

static const struct file_operations daq0_fops = {
    .owner = THIS_MODULE,
    .open  = daq_open,
    .read  = daq_read,
};

/* ------------------------------------------------------------------ */
/* File operations — /dev/daq1  (write gains)                          */
/* ------------------------------------------------------------------ */

static ssize_t gains_write(struct file *filp,
                           const char __user *buf,
                           size_t count,
                           loff_t *ppos)
{
    char kbuf[64];
    int  ret;

    if (!gains_rpdev)
        return -ENODEV;

    if (count != 12)
        return -EINVAL;

    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;

    pr_info("daq: gains_write count=%zu\n", count);

    print_hex_dump(KERN_INFO,
                "daq tx: ",
                DUMP_PREFIX_NONE,
                16,
                1,
                kbuf,
                count,
                false);

    pr_info("daq: gains_rpdev src=%u dst=%u\n",
            gains_rpdev->src,
            gains_rpdev->dst);

    if (gains_rpdev->ept)
        pr_info("daq: ept=%p\n", gains_rpdev->ept);

    ret = rpmsg_send(gains_rpdev->ept, kbuf, count);

    pr_info("daq: rpmsg_send ret=%d\n", ret);
    if (ret)
        return -EIO;

    return count;
}

static const struct file_operations daq1_fops = {
    .owner = THIS_MODULE,
    .open  = daq_open,
    .write = gains_write,
};

/* ------------------------------------------------------------------ */
/* RPMsg RX callback — only handles rpmsg-telem packets                */
/* ------------------------------------------------------------------ */

static int daq_rpmsg_rx_cb(struct rpmsg_device *rpdev,
                           void *data, int len,
                           void *priv, u32 src)
{
    struct rpmsg_telem_t pkt;

    /* Ignore anything that isn't the telemetry channel */
    if (strcmp(rpdev->id.name, "rpmsg-telem"))
        return 0;

    if (len != sizeof(struct rpmsg_telem_t)) {
        dev_warn(&rpdev->dev,
                 "Bad packet len: got %d, expected %zu\n",
                 len, sizeof(struct rpmsg_telem_t));
        return 0;
    }

    memcpy(&pkt, data, sizeof(pkt));

    if (kfifo_in_spinlocked(&telem_fifo, &pkt, 1, &fifo_lock) == 0)
        dev_warn_ratelimited(&rpdev->dev, "kfifo full — packet dropped\n");

    wake_up_interruptible(&daq_wq);
    return 0;
}

/* ------------------------------------------------------------------ */
/* RPMsg probe / remove                                                */
/* ------------------------------------------------------------------ */

static int daq_rpmsg_probe(struct rpmsg_device *rpdev)
{
    dev_info(&rpdev->dev, "channel %s src=%u dst=%u\n",
             rpdev->id.name, rpdev->src, rpdev->dst);

    if (!strcmp(rpdev->id.name, "rpmsg-telem")) {
        telem_rpdev = rpdev;
        rpmsg_send(rpdev->ept, "RDY", 3);
        dev_info(&rpdev->dev, "telemetry channel ready\n");
    } else if (!strcmp(rpdev->id.name, "rpmsg-gains")) {
        gains_rpdev = rpdev;
        rpmsg_send(rpdev->ept, "RDY", 3);
        dev_info(&rpdev->dev, "gains channel ready\n");
    }

    return 0;
}

static void daq_rpmsg_remove(struct rpmsg_device *rpdev)
{
    dev_info(&rpdev->dev, "channel %s removed\n", rpdev->id.name);

    if (!strcmp(rpdev->id.name, "rpmsg-telem"))
        telem_rpdev = NULL;
    else if (!strcmp(rpdev->id.name, "rpmsg-gains"))
        gains_rpdev = NULL;
}

/* ------------------------------------------------------------------ */
/* RPMsg driver table                                                  */
/* ------------------------------------------------------------------ */

static const struct rpmsg_device_id daq_rpmsg_id_table[] = {
    { .name = "rpmsg-telem" },
    { .name = "rpmsg-gains" },
    {},
};
MODULE_DEVICE_TABLE(rpmsg, daq_rpmsg_id_table);

static struct rpmsg_driver daq_rpmsg_driver = {
    .drv = {
        .name  = "daq_rpmsg",
        .owner = THIS_MODULE,
    },
    .id_table = daq_rpmsg_id_table,
    .probe    = daq_rpmsg_probe,
    .remove   = daq_rpmsg_remove,
    .callback = daq_rpmsg_rx_cb,
};

/* ------------------------------------------------------------------ */
/* Module init / exit — char devices created ONCE here, not in probe  */
/* ------------------------------------------------------------------ */

static int __init daq_init(void)
{
    int ret;

    /* 1. Allocate major + 2 minors */
    ret = alloc_chrdev_region(&daq_devno, 0, 2, "daq");
    if (ret < 0) {
        pr_err("daq: alloc_chrdev_region failed: %d\n", ret);
        return ret;
    }

    /* 2. /dev/daq0 — telemetry read */
    cdev_init(&daq0_cdev, &daq0_fops);
    daq0_cdev.owner = THIS_MODULE;
    ret = cdev_add(&daq0_cdev, MKDEV(MAJOR(daq_devno), 0), 1);
    if (ret < 0) {
        pr_err("daq: cdev_add daq0 failed: %d\n", ret);
        goto err_region;
    }

    /* 3. /dev/daq1 — gains write */
    cdev_init(&daq1_cdev, &daq1_fops);
    daq1_cdev.owner = THIS_MODULE;
    ret = cdev_add(&daq1_cdev, MKDEV(MAJOR(daq_devno), 1), 1);
    if (ret < 0) {
        pr_err("daq: cdev_add daq1 failed: %d\n", ret);
        goto err_cdev0;
    }

    /* 4. sysfs class → /dev/daqX nodes appear */
    daq_class = class_create("daq");
    if (IS_ERR(daq_class)) {
        ret = PTR_ERR(daq_class);
        pr_err("daq: class_create failed: %d\n", ret);
        goto err_cdev1;
    }

    device_create(daq_class, NULL, MKDEV(MAJOR(daq_devno), 0), NULL, "daq0");
    device_create(daq_class, NULL, MKDEV(MAJOR(daq_devno), 1), NULL, "daq1");

    /* 5. Register RPMsg driver — probe fires when M4 announces channels */
    ret = register_rpmsg_driver(&daq_rpmsg_driver);
    if (ret < 0) {
        pr_err("daq: register_rpmsg_driver failed: %d\n", ret);
        goto err_class;
    }

    pr_info("daq: loaded — /dev/daq0 (telem read) /dev/daq1 (gains write)\n");
    return 0;

err_class:
    device_destroy(daq_class, MKDEV(MAJOR(daq_devno), 1));
    device_destroy(daq_class, MKDEV(MAJOR(daq_devno), 0));
    class_destroy(daq_class);
err_cdev1:
    cdev_del(&daq1_cdev);
err_cdev0:
    cdev_del(&daq0_cdev);
err_region:
    unregister_chrdev_region(daq_devno, 2);
    return ret;
}

static void __exit daq_exit(void)
{
    unregister_rpmsg_driver(&daq_rpmsg_driver);

    device_destroy(daq_class, MKDEV(MAJOR(daq_devno), 1));
    device_destroy(daq_class, MKDEV(MAJOR(daq_devno), 0));
    class_destroy(daq_class);

    cdev_del(&daq1_cdev);
    cdev_del(&daq0_cdev);

    unregister_chrdev_region(daq_devno, 2);

    pr_info("daq: unloaded\n");
}

module_init(daq_init);
module_exit(daq_exit);