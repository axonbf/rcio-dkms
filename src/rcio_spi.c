#include <linux/delay.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/spi/spi.h>

#include "rcio.h"
#include "protocol.h"

static struct IOPacket *buffer;

static int wait_complete(struct spi_device *spi)
{
    int ret;

    buffer->crc = 0;
    buffer->crc = crc_packet(buffer);

    usleep_range(120, 150);

    ret = spi_write_then_read(spi, (char *) buffer, sizeof(struct IOPacket), NULL, 0);

    if (ret < 0)
        return ret;

    usleep_range(120, 150);
    ret = spi_write_then_read(spi, NULL, 0, (char *) buffer, sizeof(struct IOPacket));

    if (ret < 0)
        return ret;

    return 0;
}

static int rcio_spi_write(struct rcio_adapter *state, u16 address, const char *data, size_t count)
{
    int result;
    struct spi_device *spi = state->client;
    u16 *values = (u16 *) data;
    u8 page = address >> 8;
    u8 offset = address & 0xff;

    if (count > PKT_MAX_REGS)
        return -EINVAL;
    mutex_lock(&state->lock);

    buffer->count_code = count | PKT_CODE_WRITE;
    buffer->page = page;
    buffer->offset = offset;

    memcpy(&buffer->regs[0], (void *)values, (2 * count));
    for (unsigned i = count; i < PKT_MAX_REGS; i++)
        buffer->regs[i] = 0x55aa;

    result = wait_complete(spi);

    if (result == 0) {
        uint8_t crc = buffer->crc;
        buffer->crc = 0;

        if (crc != crc_packet(buffer)) {
            result = -EIO;
        } else if (PKT_CODE(*buffer) == PKT_CODE_ERROR) {
            result = -EINVAL;
        }
    }

    if (result == 0)
        result = count;

    mutex_unlock(&state->lock);

    return result;
}

static int rcio_spi_read(struct rcio_adapter *state, u16 address, char *data, size_t count)
{
    int result;
    struct spi_device *spi = state->client;
    u16 *values = (u16 *) data;
    u8 page = address >> 8;
    u8 offset = address & 0xff;

    if (count > PKT_MAX_REGS)
        return -EINVAL;

    mutex_lock(&state->lock);

    buffer->count_code = count | PKT_CODE_READ;
    buffer->page = page;
    buffer->offset = offset;

    result = wait_complete(spi);

    if (result == 0) {
        uint8_t crc = buffer->crc;
        buffer->crc = 0;

        if (crc != crc_packet(buffer)) {
            result = -EIO;
            dev_warn(&spi->dev,
                     "rcio_spi_read crc error page=%u offset=%u count=%zu code=0x%02x\n",
                     page, offset, count, PKT_CODE(*buffer));
        } else if (PKT_CODE(*buffer) == PKT_CODE_ERROR) {
            result = -EINVAL;
            dev_warn(&spi->dev,
                     "rcio_spi_read error reply page=%u offset=%u count=%zu\n",
                     page, offset, count);
        } else if (PKT_COUNT(*buffer) != count) {
            result = -EIO;
            dev_warn(&spi->dev,
                     "rcio_spi_read count mismatch page=%u offset=%u expected=%zu got=%u\n",
                     page, offset, count, PKT_COUNT(*buffer));
        } else {
            memcpy(values, &buffer->regs[0], (2 * count));
        }
    }

    if (result == 0)
        result = count;
    mutex_unlock(&state->lock);
    return result;
}

struct rcio_adapter st;

static int rcio_spi_probe(struct spi_device *spi)
{
    int ret;
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 4000000;

    ret = spi_setup(spi);

    if (ret < 0)
        return ret;

    st.client = spi;
    st.dev = &spi->dev;
    st.write = rcio_spi_write;
    st.read = rcio_spi_read;

    buffer = kmalloc(sizeof(struct IOPacket), GFP_DMA | GFP_KERNEL);

    if (buffer == NULL) {
        printk(KERN_INFO "No memory\n");
        return -ENOMEM;
    }

    ret = rcio_probe(&st);
    if (ret < 0) {
        kfree(buffer);
        return ret;
    }

    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0)
static void rcio_spi_remove(struct spi_device *spi)
{
    int ret = rcio_remove(&st);

    if (ret < 0) {
        dev_err(&spi->dev, "rcio_remove=%d", ret);
        return;
    }

    kfree(buffer);
}
#else
static int rcio_spi_remove(struct spi_device *spi)
{
    int ret = rcio_remove(&st);

    if (ret < 0) {
        dev_err(&spi->dev, "rcio_remove=%d", ret);
        return ret;
    }

    kfree(buffer);
    return ret;
}
#endif

static const struct spi_device_id rcio_id[] = {
	{ "rcio", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, rcio_id);

static struct spi_driver rcio_driver = {
	.driver = {
		.name = "rcio",
		.owner = THIS_MODULE,
	},
	.id_table = rcio_id,
	.probe = rcio_spi_probe,
	.remove = rcio_spi_remove,
};
module_spi_driver(rcio_driver);

MODULE_AUTHOR("Georgii Staroselskii <georgii.staroselskii@emlid.com>");
MODULE_DESCRIPTION("RCIO spi driver");
MODULE_LICENSE("GPL v2");
