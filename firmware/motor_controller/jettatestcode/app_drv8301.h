/*
 * drv8301.h
 *
 * Created: 1/21/2015 9:30:36 PM
 *  Author: Marko
 */ 
#include "Platform.h"
#include <stdint.h>

#ifndef DRV8301_H_
#define DRV8301_H_

#define SLAVE_SELECT_PIN EXT1_PIN_SPI_SS_0

#define DRV8301_RW_BITS      (1)  // SPI word [15]
#define DRV8301_ADDRESS_BITS (4)  // SPI word [14:11]
#define DRV8301_DATA_BITS    (11) // SPI word [10:0]

#define DRV8301_WRITE_CMD    (0)
#define DRV8301_READ_CMD     (1)

#define DRV8301_STATUS_REG_1  (0x00)
#define DRV8301_STATUS_REG_2  (0x01)
#define DRV8301_CONTROL_REG_1 (0x02)
#define DRV8301_CONTROL_REG_2 (0x03)

#define DRV8301_GATE_CURRENT 0
#define DRV8301_GATE_RESET   2
#define DRV8301_PWM_MODE     3
#define DRV8301_OC_MODE      4

extern struct spi_module spi_master_instance;
extern struct spi_slave_inst slave;

void init_DRV8301(void);
void configure_DRV8301(void);
uint16_t getStatus_DRV8301(void);
uint16_t getCtrlReg_DRV8301(uint8_t Reg);

uint16_t sendData(uint16_t Data);


#endif /* DRV8301_H_ */
