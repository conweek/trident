#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define OUTPUT_PRIORITY 1
#define SENSOR_PRIORITY 2
#define OUTPUT_THREAD_STACK 512
#define SENSOR_THREAD_STACK 512

#define WHO_AM_I 0

#define SPI_NODE DT_NODELABEL(imuspi)

#define SPI_CONFIG \
    (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER)

K_MSGQ_DEFINE(accel_data, sizeof(uint16_t), 1, 1);

const struct gpio_dt_spec output = GPIO_DT_SPEC_GET(DT_NODELABEL(out), gpios);
const struct gpio_dt_spec pwr = GPIO_DT_SPEC_GET(DT_NODELABEL(imupwr), gpios);

/* Get the SPI Device */
const struct device* spi_dev = DEVICE_DT_GET(SPI_NODE);

const struct spi_cs_control cs_ctrl = {
		.gpio = GPIO_DT_SPEC_GET(SPI_NODE, cs_gpios),
		.delay = 0u,
};

struct spi_config cfg = {
    .frequency = 32000,
    .operation = SPI_CONFIG,
    .slave = 0,
    .cs = cs_ctrl
};


int inv_spi_single_write(uint8_t reg, uint8_t *data)
{
	int result;

	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1,
		},
		{
			.buf = data,
			.len = 1,
		}
	};

//    const struct spi_buf rx_buf[2];

	const struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2,
	};

//    const struct spi_buf_set rx = {
//        .buffers = rx_buf,
//        .count = 2
//    };

    result = spi_transceive(spi_dev, &cfg, &tx, NULL);

	if (result) {
		return result;
	}

	return 0;
}

int inv_spi_read(uint8_t reg, uint8_t *data, size_t len)
{
	int result;

	unsigned char tx_buffer[2] = { 0, };

	tx_buffer[0] = 0x80 | reg;

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf[2] = {
		{
			.buf = tx_buffer,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};

	result = spi_transceive(spi_dev, &cfg, &tx, &rx);

	if (result) {
		return result;
	}

	return 0;
}

void output_thread()
{
    while (1) {
        
        uint16_t xVal = 0;

        /* Grab the data out of the message queue */
        if (k_msgq_get(&accel_data, &xVal, K_FOREVER)) {
            gpio_pin_set_dt(&output, 1);
            k_msleep(250);
            gpio_pin_set_dt(&output, 0);
        }

    }
}

void init_imu()
{

    int ret = 0;
    uint8_t value = 0x80;

    /* Soft reset the device */
    inv_spi_single_write(0x06, &value);

    k_msleep(50);

    value = 0x10;

    /* Enable SPI not I2C */
    inv_spi_single_write(0x03, &value);

    k_msleep(50);
    ret = inv_spi_read(WHO_AM_I, &value, 1);
    printk("Who-Am-I passed : 0x%x\r\n", value);

    k_msleep(50);
    
    value = 0x01;

    /* Enable PLL Clock or interal osc */
    inv_spi_single_write(0x06, &value);

    k_msleep(50);

    value = (3 << 1);

    //inv_spi_single_write(((2 << 7) | 0x14), &value);
    //printk("Accel config: 0x%x", value);

    k_msleep(50);

    inv_spi_read(((2 << 7) | 0x14), &value, 1);
    printk("Accel config: 0x%x", value);

}

void sensor_thread()
{

    if (!gpio_is_ready_dt(&pwr)) {
        printk("IMU Power not ready!\r\n");
        k_panic();
    }

    gpio_pin_set_dt(&pwr, 1);

    /* Check the device is ready */
    if (!device_is_ready(spi_dev)) {
        printk("Dead SPI!\r\n");
        k_panic();
    }

    init_imu();
    float accelRes = 2.0f / 32768.0f;  // 2g full scale
    uint8_t rawData[2];
    int16_t accel_raw;
    float accel_m_s2;

    while (1) {

        // Read high and low bytes of Y-axis
        inv_spi_read(0x2F, &rawData[0], 1);  // ACCEL_YOUT_H
        inv_spi_read(0x30, &rawData[1], 1);  // ACCEL_YOUT_L

        // Combine bytes into signed 16-bit value
        accel_raw = ((int16_t)rawData[0] << 8) | rawData[1];

        // Convert to g using full-scale resolution
        accel_m_s2 = (float)accel_raw * accelRes;

        // Convert to m/s²
        accel_m_s2 *= 9.81f;

        if (accel_m_s2 > 12.f) {
            k_msgq_purge(&accel_data);
            k_msgq_put(&accel_data, &accel_m_s2, K_NO_WAIT);
        }
        k_msleep(5);   
    }

}


K_THREAD_DEFINE(output_tid, OUTPUT_THREAD_STACK, output_thread, 
        NULL, NULL, NULL, OUTPUT_PRIORITY, 0, 0);

K_THREAD_DEFINE(sensor_tid, SENSOR_THREAD_STACK, sensor_thread, 
        NULL, NULL, NULL, SENSOR_PRIORITY, 0, 0);
