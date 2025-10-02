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

struct spi_cs_control cs_ctrl = (struct spi_cs_control) {
		.gpio = GPIO_DT_SPEC_GET(SPI_NODE, cs_gpios),
		.delay = 0u,
};

struct spi_config cfg = (struct spi_config) {
    .frequency = 10000,
    .operation = SPI_CONFIG,
    .slave = 0,
    .cs = &cs_ctrl 
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

    printk("About to read ts\r\n");
	result = spi_transceive(spi_dev, &cfg, &tx, &rx);
    printk("Read ts\r\n");

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
            k_msleep(50);
            gpio_pin_set_dt(&output, 0);
        }

    }
}

void init_imu()
{


    printk("Turning on the shit\r\n");

    k_msleep(10);

    int ret = 0;
    uint8_t value = 0;

    printk("Abt to read!!\r\n");
    ret = inv_spi_read(WHO_AM_I, &value, 1);
    __ASSERT(value == 0xE0, "Expected Who-am-i of 0xE0, got 0x%x", value);

    if (ret) {
        printk("Failed to read SPI\n");
        k_panic();
    }

    printk("Who-Am-I passed : 0x%x\r\n", value);

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
    
    while (1) {
        k_msleep(1);   
    }

}


K_THREAD_DEFINE(output_tid, OUTPUT_THREAD_STACK, output_thread, 
        NULL, NULL, NULL, OUTPUT_PRIORITY, 0, 0);

K_THREAD_DEFINE(sensor_tid, SENSOR_THREAD_STACK, sensor_thread, 
        NULL, NULL, NULL, SENSOR_PRIORITY, 0, 0);
