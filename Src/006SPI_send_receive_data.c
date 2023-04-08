/*
 * 006SPI_receive_data_from_slave.c
 *
 *  Created on: 28-Dec-2022
 *      Author: dsour
 */


#include "../drivers/Inc/stm32f446xx.h"
#include <string.h>

/*
 * Specifications
 * 		->mode SPI2- MASTER
 * 		->SCLK- 2Mz
 * 		->DFF=0
 * 		->Full duplex
 * 		->Hardware slave management(SSM=0)
 *
 * When built-in button pressed, send data to slave, slave gives response.
 */

/*
 * MOSI-> PB15
 * MISO-> PB14
 * SCLK-> PB13
 * NSS -> PB12
 * Alternate fn mode-> 5
 *
 * Button-> PC13
 */

int main()
{
	//;
	return 0;
}
