#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

/* Thread priorities */
#define OUTPUT_PRIORITY 1
#define SENSOR_PRIORITY 2

/* Thread stack sizes */
#define OUTPUT_THREAD_STACK 512
#define SENSOR_THREAD_STACK 512

/* Accelerometer Registers */
#define WHO_AM_I 0
#define PWR_MGMT_1 0x06
#define USR_CTRL 0x03
#define YOUT_HIGH 0x2F
#define YOUT_LOW 0x30

/* Accelerometer Commands */
#define CLK_PLL_ENA 0x01
#define SOFT_RESET 0x08
#define I2C_DIS 0x10

/* Delay Values */
#define ACCEL_INIT_DELAY 50
#define SENSOR_THREAD_DELAY 5
#define FLIPPER_HOLD_DELAY 250

/* Gravity (used to convert accelerometer reading to m/s^2) */
#define GRAVITY 9.81

/* "Stab" threshold - how hard to stab to trigger the flipper */
#define STAB_THRESH 12.f

/* Accelerometer full scale range and resolution */
#define FULL_SCALE_RANGE  2.f
#define RESOLUTION 32678.f
#define ACCEL_RES (FULL_SCALE_RANGE / RESOLUTION)

/* Accelerometer Byte Defines */
#define BYTE_OFFSET 8
#define BYTES_USED 2
#define LOW_BYTE 1
#define HIGH_BYTE 0

#define SPI_NODE DT_NODELABEL(imuspi)

/* SPI Mode Configuration */
#define SPI_CONFIG \
    (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER)


K_MSGQ_DEFINE(accel_data, sizeof(uint16_t), 1, 1);

/* GPIOs */
const struct gpio_dt_spec output = GPIO_DT_SPEC_GET(DT_NODELABEL(out), gpios);
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec pwr = GPIO_DT_SPEC_GET(DT_NODELABEL(imupwr), gpios);

/* Get the SPI Device */
const struct device* spi_dev = DEVICE_DT_GET(SPI_NODE);

/* Create the CS struct for use in the SPI Config */
const struct spi_cs_control cs_ctrl = {
		.gpio = GPIO_DT_SPEC_GET(SPI_NODE, cs_gpios),
		.delay = 0u,
};

/* Configuration used for the SPI */
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

	const struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2,
	};

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

    /* Set highest byte to do read operation */
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
            
            /* Enable the flipper signal pin & LED if "stab" completed */
            gpio_pin_set_dt(&output, 1);
            gpio_pin_set_dt(&led, 1);

            /* Hold the flipper up to make game playable */
            k_msleep(FLIPPER_HOLD_DELAY);
            
            /* Disable the flipper signal and LED */
            gpio_pin_set_dt(&output, 0);
            gpio_pin_set_dt(&led, 0);
        }

    }
}

void init_imu()
{

    int ret = 0;
    uint8_t value = SOFT_RESET;

    /* Soft reset the device */
    inv_spi_single_write(PWR_MGMT_1, &value);

    k_msleep(ACCEL_INIT_DELAY);

    value = I2C_DIS;

    /* Enable SPI not I2C */
    inv_spi_single_write(USR_CTRL, &value);

    k_msleep(ACCEL_INIT_DELAY);
    ret = inv_spi_read(WHO_AM_I, &value, 1);
    printk("Who-Am-I : 0x%x\r\n", value);

    k_msleep(ACCEL_INIT_DELAY);
    
    value = CLK_PLL_ENA;

    /* Enable PLL Clock or interal osc */
    inv_spi_single_write(PWR_MGMT_1, &value);

    k_msleep(ACCEL_INIT_DELAY);

}

void sensor_thread()
{

    if (!gpio_is_ready_dt(&pwr)) {
        printk("ICM Power not ready!\r\n");
        k_panic();
    }

    /* Turn on the power to the accelerometer */
    gpio_pin_set_dt(&pwr, 1);

    if (!gpio_is_ready_dt(&led)) {
        printk("LED not ready!\r\n");
        k_panic();
    }

    /* Check the device is ready */
    if (!device_is_ready(spi_dev)) {
        printk("Dead SPI!\r\n");
        k_panic();
    }

    /* Initialise the accelerometer */
    init_imu();

    float accelRes = ACCEL_RES;
    uint8_t rawData[BYTES_USED];
    int16_t accel_raw;
    float accel_m_s2;

    while (1) {

        /* Read high and low bytes of Y-axis */
        inv_spi_read(YOUT_HIGH, &rawData[HIGH_BYTE], 1);
        inv_spi_read(YOUT_LOW, &rawData[LOW_BYTE], 1);

        /* Combine bytes into signed 16-bit value */
        accel_raw = (((int16_t)rawData[HIGH_BYTE] << BYTE_OFFSET)
                | rawData[LOW_BYTE]);

        /* Convert to g w/ full scale range */
        accel_m_s2 = (float)accel_raw * accelRes;

        /* Convert to m/s^2 */
        accel_m_s2 *= GRAVITY;

        /* Check we're over the "stab threshold" */
        if (accel_m_s2 > STAB_THRESH) {
            k_msgq_purge(&accel_data);
            k_msgq_put(&accel_data, &accel_m_s2, K_NO_WAIT);
        }

        k_msleep(SENSOR_THREAD_DELAY);
    }

}


K_THREAD_DEFINE(output_tid, OUTPUT_THREAD_STACK, output_thread, 
        NULL, NULL, NULL, OUTPUT_PRIORITY, 0, 0);

K_THREAD_DEFINE(sensor_tid, SENSOR_THREAD_STACK, sensor_thread, 
        NULL, NULL, NULL, SENSOR_PRIORITY, 0, 0);
