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

#define DRV8301_RW_BIT_POS      (15)
#define DRV8301_ADDRESS_BIT_POS (11)
#define DRV8301_DATA_BITS_POS   (0)

#define DRV8301_WRITE_CMD    (0)
#define DRV8301_READ_CMD     (1)

#define DRV8301_STATUS_REG_1  (0x00)
#define DRV8301_STATUS_REG_2  (0x01)
#define DRV8301_CONTROL_REG_1 (0x02)
#define DRV8301_CONTROL_REG_2 (0x03)

#define DRV8301_GATE_CURRENT_POS 0
#define DRV8301_GATE_RESET_POS   2
#define DRV8301_PWM_MODE_POS     3
#define DRV8301_OC_MODE_POS      4
#define DRV8301_OC_ADJ_SET_POS   6

#define DRV8301_GATE_PEAK_CURRENT_1700mA (0x00)
#define DRV8301_GATE_PEAK_CURRENT_700mA  (0x01)
#define DRV8301_GATE_PEAK_CURRENT_250mA  (0x02)

#define DRV8301_GATE_RESET (0x01)

#define DRV8301_PWM_MODE_6_INPUTS (0x00)
#define DRV8301_PWM_MODE_3_INPUTS (0x01)

#define DRV8301_OC_MODE_CURRENT_LIMIT (0x00)
#define DRV8301_OC_MODE_SHUT_DOWN     (0x01)
#define DRV8301_OC_MODE_REPORT_ONLY   (0x02)
#define DRV8301_OC_MODE_DISABLED      (0x03)

#define DRV8301_OC_ADJ_SET_VDS_60mV   (0x00)
#define DRV8301_OC_ADJ_SET_VDS_68mV   (0x01)
#define DRV8301_OC_ADJ_SET_VDS_76mV   (0x02)
#define DRV8301_OC_ADJ_SET_VDS_86mV   (0x03)
#define DRV8301_OC_ADJ_SET_VDS_97mV   (0x04)
#define DRV8301_OC_ADJ_SET_VDS_109mV  (0x05)
#define DRV8301_OC_ADJ_SET_VDS_123mV  (0x06)
#define DRV8301_OC_ADJ_SET_VDS_138mV  (0x07)
#define DRV8301_OC_ADJ_SET_VDS_155mV  (0x08)
#define DRV8301_OC_ADJ_SET_VDS_175mV  (0x09)
#define DRV8301_OC_ADJ_SET_VDS_197mV  (0x0A)
#define DRV8301_OC_ADJ_SET_VDS_222mV  (0x0B)
#define DRV8301_OC_ADJ_SET_VDS_250mV  (0x0C)
#define DRV8301_OC_ADJ_SET_VDS_282mV  (0x0D)
#define DRV8301_OC_ADJ_SET_VDS_317mV  (0x0E)
#define DRV8301_OC_ADJ_SET_VDS_358mV  (0x0F)
#define DRV8301_OC_ADJ_SET_VDS_403mV  (0x10)
#define DRV8301_OC_ADJ_SET_VDS_454mV  (0x11)
#define DRV8301_OC_ADJ_SET_VDS_511mV  (0x12)
#define DRV8301_OC_ADJ_SET_VDS_576mV  (0x13)
#define DRV8301_OC_ADJ_SET_VDS_648mV  (0x14)
#define DRV8301_OC_ADJ_SET_VDS_730mV  (0x15)
#define DRV8301_OC_ADJ_SET_VDS_822mV  (0x16)
#define DRV8301_OC_ADJ_SET_VDS_926mV  (0x17)
#define DRV8301_OC_ADJ_SET_VDS_1043mV (0x18)
#define DRV8301_OC_ADJ_SET_VDS_1175mV (0x19)
#define DRV8301_OC_ADJ_SET_VDS_1324mV (0x1A)
#define DRV8301_OC_ADJ_SET_VDS_1491mV (0x1B)
#define DRV8301_OC_ADJ_SET_VDS_1679mV (0x1C)
#define DRV8301_OC_ADJ_SET_VDS_1892mV (0x1D)
#define DRV8301_OC_ADJ_SET_VDS_2131mV (0x1E)
#define DRV8301_OC_ADJ_SET_VDS_2400mV (0x1F)

#define DRV8301_STATUS_REG1  (0x00)
#define DRV8301_STATUS_REG2  (0x01)
#define DRV8301_CONTROL_REG1 (0x02)
#define DRV8301_CONTROL_REG2 (0x03)

// Status flags (actually error flags, status reg1 only reports errors...)
#define DRV8301_FAULT_FLAG    (1 << 10)	// fault detected, other flags specify what whas the problem...
#define DRV8301_GVDD_UV_FLAG  (1 << 9)
#define DRV8301_PVDD_UV_FLAG  (1 << 8)
#define DRV8301_OTSD_FLAG     (1 << 7)
#define DRV8301_OTW_FLAG      (1 << 6)
#define DRV8301_FETHA_OC_FLAG (1 << 5)  // MOSFET on channel A, high side overcurrent
#define DRV8301_FETLA_OC_FLAG (1 << 4)  // MOSFET on channel A, low side overcurrent
#define DRV8301_FETHB_OC_FLAG (1 << 3)  // MOSFET on channel B, high side overcurrent
#define DRV8301_FETLB_OC_FLAG (1 << 2)  // MOSFET on channel B, low side overcurrent
#define DRV8301_FETHC_OC_FLAG (1 << 1)  // MOSFET on channel C, high side overcurrent
#define DRV8301_FETLC_OC_FLAG (1 << 0)  // MOSFET on channel C, low side overcurrent

extern struct spi_module spi_master_instance;
extern struct spi_slave_inst slave;

void init_DRV8301(void);
void configure_DRV8301(void);
uint16_t getReg_DRV8301(uint8_t Reg);

#endif /* DRV8301_H_ */
