/*
 * MAX30100_Registers.h
 *
 *  Created on: Nov 29, 2019
 *      Author: armando
 */

#ifndef MAX30100_REGISTERS_H_
#define MAX30100_REGISTERS_H_

#define MAX30100_WRITE 		0xAE
#define MAX30100_READ  		0xAF

// Interrupt status register (RO)
#define MAX30100_REG_INTERRUPT_STATUS           0x00
#define MAX30100_IS_PWR_RDY                     (1 << 0)
#define MAX30100_IS_SPO2_RDY                    (1 << 4)
#define MAX30100_IS_HR_RDY                      (1 << 5)
#define MAX30100_IS_TEMP_RDY                    (1 << 6)
#define MAX30100_IS_A_FULL                      (1 << 7)

// Interrupt enable register
#define MAX30100_REG_INTERRUPT_ENABLE           0x01
#define MAX30100_IE_ENB_SPO2_RDY                (1 << 4)
#define MAX30100_IE_ENB_HR_RDY                  (1 << 5)
#define MAX30100_IE_ENB_TEMP_RDY                (1 << 6)
#define MAX30100_IE_ENB_A_FULL                  (1 << 7)

// FIFO control and data registers
#define MAX30100_REG_FIFO_WRITE_POINTER         0x02
#define MAX30100_REG_FIFO_OVERFLOW_COUNTER      0x03
#define MAX30100_REG_FIFO_READ_POINTER          0x04
#define MAX30100_REG_FIFO_DATA                  0x05

// Mode Configuration register
#define MAX30100_REG_MODE_CONFIGURATION         0x06
#define MAX30100_MC_TEMP_EN                     (1 << 3)
#define MAX30100_MC_RESET                       (1 << 6)
#define MAX30100_MC_SHDN                        (1 << 7)

typedef enum Mode {
    MAX30100_MODE_HRONLY    = 0x02,
    MAX30100_MODE_SPO2_HR   = 0x03
} Mode;

// SpO2 Configuration register
// Check tables 8 and 9, p19 of the MAX30100 datasheet to see the permissible
// combinations of sampling rates and pulse widths
#define MAX30100_REG_SPO2_CONFIGURATION         0x07
#define MAX30100_SPC_SPO2_HI_RES_EN             (1 << 6)


typedef enum SamplingRate {
    MAX30100_SAMPRATE_50HZ,
	MAX30100_SAMPRATE_100HZ,
	MAX30100_SAMPRATE_167HZ,
	MAX30100_SAMPRATE_200HZ,
	MAX30100_SAMPRATE_400HZ,
	MAX30100_SAMPRATE_600HZ,
	MAX30100_SAMPRATE_800HZ,
	MAX30100_SAMPRATE_1000HZ
} SamplingRate;

typedef enum LEDPulseWidth {
    MAX30100_SPC_PW_200US_13BITS,
    MAX30100_SPC_PW_400US_14BITS,
    MAX30100_SPC_PW_800US_15BITS,
    MAX30100_SPC_PW_1600US_16BITS
} LEDPulseWidth;

// LED Configuration register
#define MAX30100_REG_LED_CONFIGURATION          0x09

typedef enum LEDCurrent {
	MAX30100_LED_CURR_0MA,
	MAX30100_LED_CURR_4_4MA,
	MAX30100_LED_CURR_7_6MA,
	MAX30100_LED_CURR_11MA,
	MAX30100_LED_CURR_14_2MA,
	MAX30100_LED_CURR_17_4MA,
	MAX30100_LED_CURR_20_8MA,
	MAX30100_LED_CURR_24MA,
	MAX30100_LED_CURR_27_1MA,
	MAX30100_LED_CURR_30_6MA,
	MAX30100_LED_CURR_33_8MA,
	MAX30100_LED_CURR_37MA,
	MAX30100_LED_CURR_40_2MA,
	MAX30100_LED_CURR_43_6MA,
	MAX30100_LED_CURR_46_8MA,
	MAX30100_LED_CURR_50MA
} LEDCurrent;

// Temperature integer part register
#define MAX30100_REG_TEMPERATURE_DATA_INT       0x16
// Temperature fractional part register
#define MAX30100_REG_TEMPERATURE_DATA_FRAC      0x17

// Revision ID register (RO)
#define MAX30100_REG_REVISION_ID                0xfe
// Part ID register
#define MAX30100_REG_PART_ID                    0xff

#endif /* MAX30100_REGISTERS_H_ */
