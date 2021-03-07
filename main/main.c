#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define  AD5060_CS 		GPIO_NUM_5
#define  AD5060_MISO 	GPIO_NUM_19
#define  AD5060_MOSI 	GPIO_NUM_23
#define  AD5060_SCLK 	GPIO_NUM_18
#define  AD5060_HOST    HSPI_HOST
#define  DMA_CHAN		2

float refVal_cali = 1.0171;

//uninitalised pointers to SPI objects

void app_main(void) {
	esp_err_t ret;
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
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
	ESP_ERROR_CHECK(ret);





	printf("Hello World.\r\n");

	while (true) {
		gpio_set_level(GPIO_NUM_2, level);
		level = !level;
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

