#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>

#define  AD5060_CS 		GPIO_NUM_5
#define  AD5060_MISO 	GPIO_NUM_19
#define  AD5060_MOSI 	GPIO_NUM_23
#define  AD5060_SCLK 	GPIO_NUM_18
#define  AD5060_HOST    VSPI_HOST
#define  DMA_CHAN		2

float refVal_cali = 1.0171;
spi_device_handle_t spi1;

//uninitalised pointers to SPI objects


// Defines
// DAC Maximum value
#define MAXVAL 4095
// Maximum number of offset table entries
#define MAX_OFFSETS 10
// DAC Slave Select
#define DACSS 5
// Mode values
// send refVal to DAC after adding correction factor
#define CORRECTED true
// send raw refVal numbers to the DAC w/o correcting
#define RAW false

uint32_t refVal = 0;
uint8_t sdata;
uint16_t DACreg;
uint8_t offset = 0;
uint16_t Vref = 4020;

bool mode = CORRECTED; // when mode = CORRECTED then offset value is used


void app_main(void) {
	esp_err_t ret;
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	int level = 0;

	spi_bus_config_t buscfg = {
			.mosi_io_num = AD5060_MOSI,
			.miso_io_num = AD5060_MISO,
			.sclk_io_num = AD5060_SCLK,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 32,
	};

	ret = spi_bus_initialize(AD5060_HOST, &buscfg, DMA_CHAN);
	printf("init SPI BUS return = %d\r\n", ret);
	ESP_ERROR_CHECK(ret);

	// from https://github.com/espressif/esp-idf/issues/1148
	spi_device_interface_config_t devcfg = {
			.clock_speed_hz = 16*1000*1000,  	// clock out at 16 MHz
			.mode = 1, 							// SPI Mode 3
			.queue_size = 1,					//We want to be able to queue 1 transactions at a time
			.spics_io_num = AD5060_CS,

	};

	ret = spi_bus_add_device(AD5060_HOST, &devcfg, &spi1);
	printf("init SPI device return = %d\r\n", ret);
	ESP_ERROR_CHECK(ret);
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 24;
	t.flags = SPI_TRANS_USE_TXDATA;
	t.tx_data[0] = 0x00;
	t.tx_data[1] = 0x80;
	t.tx_data[2] = 0xff;
	t.tx_data[3] = 0x00;

	refVal = 20000;

	if (mode == CORRECTED) {
		printf("offset = %d\r\n", offset);
		DACreg = (uint16_t) (((refVal + 2.68710725) / 6.24904004)) + offset;
	}
	else
	{
		DACreg = (uint16_t) ((refVal + 2.68710725) / 6.24904004);
	}

	printf("DACreg = %x\r\n", DACreg);

	t.tx_data[1] = (DACreg & 0xFF00) >> 8;
	t.tx_data[2] = (DACreg & 0xFF);

	printf("tx_data = %x %x %x\r\n", t.tx_data[0], t.tx_data[1], t.tx_data[2]);

	// write data to AD5060
	ret = spi_device_acquire_bus(spi1, portMAX_DELAY);
	ESP_ERROR_CHECK(ret);
	ret = spi_device_transmit(spi1, &t);
	printf("SPI device transmit return = %d\r\n", ret);
	ESP_ERROR_CHECK(ret);
	spi_device_release_bus(spi1);

	printf("Hello World.\r\n");

	while (true) {
		gpio_set_level(GPIO_NUM_2, level);
		level = !level;
		vTaskDelay(500 / portTICK_PERIOD_MS);



		refVal += 1000;

		if (mode == CORRECTED) {
			printf("offset = %d\r\n", offset);
			DACreg = (uint16_t) (((refVal + 2.68710725) / 6.24904004)) + offset;
		}
		else
		{
			DACreg = (uint16_t) ((refVal + 2.68710725) / 6.24904004);
		}

		printf("DACreg = %x\r\n", DACreg);

		t.tx_data[1] = (DACreg & 0xFF00) >> 8;
		t.tx_data[2] = (DACreg & 0xFF);

		printf("tx_data = %x %x %x\r\n", t.tx_data[0], t.tx_data[1], t.tx_data[2]);

		// write data to AD5060
		ret = spi_device_acquire_bus(spi1, portMAX_DELAY);
		ESP_ERROR_CHECK(ret);
		ret = spi_device_transmit(spi1, &t);
		printf("SPI device transmit return = %d\r\n", ret);
		ESP_ERROR_CHECK(ret);
		spi_device_release_bus(spi1);



	}
}

