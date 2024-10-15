/**
 * *******************************************************
 * 
 * @file L3G4200D_HAL_SPI.h
 * @brief Provide APIs to write/read data from L3G4200D for STM32 family
 * @author MinhChi
 * 
 * *******************************************************
*/

#ifndef __L3G4200D_HAL_SPI_H
#define __L3G4200D_HAL_SPI_H

/**
 * Change this header file if you use another STM32 MCU, or another driver
 */
#include "stm32f1xx_hal.h"   /* Need for SPI*/
#include <stdint.h>

/**
 * *************************************************************
 * 
 * @brief Define register address sensor's
 * @page 27
 * 
 *    The device contains a set of registers which are used to control its behavior and to retrieve
 *    acceleration data. The registers address, made of 7 bits, is used to identify them and to
 *    write the data through serial interface
 * 
 * *************************************************************
 */
#define WHO_AM_I         0x0F
#define CTRL_REG1        0x20
#define CTRL_REG2        0x21
#define CTRL_REG3        0x22
#define CTRL_REG4        0x23
#define CTRL_REG5        0x24
#define REFERENCE        0x25
#define OUT_TEMP         0x26
#define STATUS_REG       0x27
#define OUT_X_L          0x28
#define OUT_X_H          0x29
#define OUT_Y_L          0x2A
#define OUT_Y_H          0x2B
#define OUT_Z_L          0x2C
#define OUT_Z_H          0x2D
#define FIFO_CTRL_REG    0x2E
#define FIFO_SRC_REG     0x2F
#define INT1_CFG         0x30
#define INT1_SRC         0x31
#define INT1_THS_XH      0x32
#define INT1_THS_XL      0x33
#define INT1_THS_YH      0x34
#define INT1_THS_YL      0x35
#define INT1_THS_ZH      0x36
#define INT1_THS_ZL      0x37
#define INT1_DURATION    0x38

/**
 * Device address
 * @note If the SDO pin (PIN 4) is connected to the voltage supply, LSb is ‘1’ (address 1101001b).
 *       Otherwise, if the SDO pin (PIN 4) is connected to ground, the LSb value is ‘0’ (address 1101000b).
 * @page 22
 */
//#define L3G4200D_ADDRESS        0x69               /* Pin 4 connecting to the voltage supply */
#define L3G4200D_ADDRESS        0x68              /* Pin 4 connecting to ground */
#define DEFAULT_VALUE           0x00u
#define SPI_READ_REQUEST        0x80u
#define SPI_WRITE_REQUEST       0x3Fu
#define SPI_DUMMY               0x00u
/**
 * Define write/read register default value
 */
#define L3G4200D_IDENTIFY             0xD3
#define CTRL_REG1_DF_VALUE            0x07u
#define CTRL_REG2_DF_VALUE            DEFAULT_VALUE
#define CTRL_REG3_DF_VALUE            DEFAULT_VALUE
#define CTRL_REG4_DF_VALUE            DEFAULT_VALUE
#define CTRL_REG5_DF_VALUE            DEFAULT_VALUE
#define REFERENCE_DF_VALUE            DEFAULT_VALUE
#define FIFO_CTRL_REG_DF_VALUE        DEFAULT_VALUE
#define INT1_CFG_DF_VALUE             DEFAULT_VALUE
#define INT1_THS_XH_DF_VALUE          DEFAULT_VALUE
#define INT1_THS_XL_DF_VALUE          DEFAULT_VALUE
#define INT1_THS_YH_DF_VALUE          DEFAULT_VALUE
#define INT1_THS_YL_DF_VALUE          DEFAULT_VALUE
#define INT1_THS_ZH_DF_VALUE          DEFAULT_VALUE
#define INT1_THS_ZL_DF_VALUE          DEFAULT_VALUE
#define INT1_DURATION_DF_VALUE        DEFAULT_VALUE

/**
 * *********************************************************
 * 
 * @brief Define operation mode to control device, reference on CTRL_REG1
 *          DR[7:6]: Output Data Rate
 *          BW[5:4]: Bandwidth
 *          PD[3]: Power down mode
 *          X, Y, X Enable[2:0]: Enable three axis
 * 
 * @page 29
 * 
 * **********************************************************
 */
#define OUTPUT_DATA_RATE_100     DEFAULT_VALUE
#define OUTPUT_DATA_RATE_200     0x01u
#define OUTPUT_DATA_RATE_400     0x02u
#define OUTPUT_DATA_RATE_800     0x03u
#define BANDWIDTH_1              DEFAULT_VALUE
#define BANDWIDTH_2              0x01u
#define BANDWIDTH_3              0x02u
#define BANDWIDTH_4              0x03u
#define PWR_DOWN_MODE            DEFAULT_VALUE
#define PWR_NORMAL_MODE          0x01u
#define PWR_SLEEP_MODE           PWR_NORMAL_MODE
#define DISABLE_Z_AXIS           0x00u
#define ENABLE_Z_AXIS            0x01u
#define DISABLE_Y_AXIS           0x00u
#define ENABLE_Y_AXIS            0x01u
#define DISABLE_X_AXIS           0x00u
#define ENABLE_X_AXIS            0x01u

/**
 * *********************************************************
 * 
 * @brief Define high-pass filter to control device, reference on CTRL_REG2
 *          HPM[5:4]: High Pass filter Mode
 *          HPCF[3:0]: High Pass filter Cut Off frequency 
 * 
 * @page 30
 * 
 * **********************************************************
 */
#define NORMAL_MODE_RESET                        DEFAULT_VALUE
#define REF_SIG_FOR_FILTER                       0x01u
#define NORMAL_MODE                              0x02u
#define AUTORESET_ON_ITR                         0x03u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_1     DEFAULT_VALUE
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_2     0x01u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_3     0x02u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_4     0x03u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_5     0x04u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_6     0x05u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_7     0x06u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_8     0x07u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_9     0x08u
#define HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_10    0x09u

/**
 * *********************************************************
 * 
 * @brief Define interrupt mode to control device, reference on CTRL_REG3
 *          I1_Int1[7]: Interrupt enable on INT1 pin
 *          I1_Boot[6]: Boot status available on INT1
 *          H_Lactive[5]: Interrupt active configuration on INT1
 *          PP_OD[4]: Push- Pull / Open drain
 *          I2_DRDY[3]: Data Ready on DRDY/INT2
 *          I2_WTM[2]: FIFO Watermark interrupt on DRDY/INT2
 *          I2_ORun[1]: FIFO Overrun interrupt on DRDY/INT2
 *          I2_Empty[0]: FIFO Empty interrupt on DRDY/INT2
 * 
 * @page 31
 * 
 * **********************************************************
 */
#define ENABLE_ITR_ON_INT1              0x01u           /* Interrupt enable on INT1 pin */
#define DISABLE_ITR_ON_INT1             DEFAULT_VALUE
#define ENABLE_BOOT_STATUS_ON_INT1      0x01u           /* Boot status available on INT1 */
#define DISABLE_BOOT_STATUS_ON_INT1     DEFAULT_VALUE
#define ITR_ACTIVE_ON_INT1_HIGH         DEFAULT_VALUE   /* Interrupt active configuration on INT1 */
#define ITR_ACTIVE_ON_INT1_LOW          0x01u           /* Interrupt active configuration on INT1 */
#define PUSH_PULL_ENABLE                DEFAULT_VALUE   /* Push- Pull / Open drain */
#define OPEN_DRAIN_ENABLE               0x01u           /* Push- Pull / Open drain */
#define ENABLE_DATA_RDY_ON_INT2         0x01u           /* Data Ready on DRDY/INT2 */
#define DISABLE_DATA_RDY_ON_INT2        DEFAULT_VALUE
#define ENABLE_ITR_FIFO_WTM_ON_INT2     0x01u           /* FIFO Watermark interrupt on DRDY/INT2 */
#define DISABLE_ITR_FIFO_WTM_ON_INT2    DEFAULT_VALUE
#define ENABLE_ITR_FIFO_ORUN_ON_INT2    0x01u           /* FIFO Overrun interrupt on DRDY/INT2 */
#define DISABLE_ITR_FIFO_ORUN_ON_INT2   DEFAULT_VALUE
#define ENABLE_ITR_FIFO_EMPTY_ON_INT2   0x01u           /* FIFO Empty interrupt on DRDY/INT2 */
#define DISABLE_ITR_FIFO_EMPTY_ON_INT2  DEFAULT_VALUE

/**
 * *********************************************************
 * 
 * @brief Define measuring range, SPI interface and mode to control device, reference on CTRL_REG4
 *          BDU[7]: Block Data Update
 *          BLE[6]: Big/Little Endian Data Selection
 *          FS[5:4]: Full Scale selection
 *          ST[2:1]: Self Test Enable
 *          SIM[0]: SPI Serial Interface Mode selection
 * 
 * @page 32
 * 
 * **********************************************************
 */
#define DBU_CONTINUOUS_UPDATE          DEFAULT_VALUE
#define DBU_OUTPUT_REG_NOT_UPDATE      0x01u
#define BLE_DATA_LSB_LOW_ADD           DEFAULT_VALUE
#define BLE_DATA_MSB_LOW_ADD           0x01u
#define FULL_SCALE_MODE_1              DEFAULT_VALUE    /* 250 dps */
#define FULL_SCALE_MODE_2              0x01u            /* 500 dps */
#define FULL_SCALE_MODE_3              0x02u            /* 2000 dps */
#define FULL_SCALE_MODE_4              0x03u            /* 2000 dps */
#define SELF_TEST_NORMAL_MODE          DEFAULT_VALUE
#define SELF_TEST_MODE_0               0x01u
#define SELF_TEST_MODE_UNKOWN          0x02u
#define SELF_TEST_MODE_1               0x03u
#define SPI_INTERFACE_3WIRE_MODE       0x01u
#define SPI_INTERFACE_4WIRE_MODE       DEFAULT_VALUE

/**
 * *********************************************************
 * 
 * @brief Define turn on/off FIFO and high-pass filter to control device, reference on CTRL_REG5
 *          BOOT[7]: Block Data Update
 *          FIFO[6]: Big/Little Endian Data Selection
 *          HPEN[4]: Full Scale selection
 *          INT1_SEL[3:2]: Self Test Enable
 *          OUT_SEL[1:0]: SPI Serial Interface Mode selection
 * 
 * @note Visit table 35, 36 page 33 for the right option for your project
 * 
 * @page 32
 * 
 * **********************************************************
 */
#define BOOT_NORMAL_MODE              DEFAULT_VALUE
#define BOOT_MEMORY_CONTENT_MODE      0x01u
#define ENABLE_FIFO                   0x01u
#define DISABLE_FIFO                  DEFAULT_VALUE
#define ENABLE_HIGH_PASS_FILTER       0x01u
#define DISABLE_HIGH_PASS_FILTER      DEFAULT_VALUE
/* Read the @note above */
#define INT1_SELECT_MODE_1           DEFAULT_VALUE          /* Non-high-pass-filtered data are used for interrupt generation */
#define INT1_SELECT_MODE_2           0x01u                  /* High-pass-filtered data are used for interrupt generation */
#define INT1_SELECT_MODE_3           0x02u                  /* Low-pass-filtered data are used for interrupt generation */
#define INT1_SELECT_MODE_4           0x03u                  /* High-pass and low-pass-filtered data are used for interrupt generation */
/* Read the @note above */
#define OUT_SELECT_MODE_1            DEFAULT_VALUE          /* Data in DataReg and FIFO are non-highpass-filtered */
#define OUT_SELECT_MODE_2            0x01u                  /* Data in DataReg and FIFO are high-passfiltered */
#define OUT_SELECT_MODE_3            0x02u                  /* Data in DataReg and FIFO are low-passfiltered by LPF2 */
#define OUT_SELECT_MODE_4            0x03u                  /* Data in DataReg and FIFO are high-pass and low-pass-filtered by LPF2*/

/**
 * *********************************************************
 * 
 * @brief Define configure and operation mode of FIFO to control device, reference on FIFO_CTRL_REG
 *          FM[7:5]: FIFO mode selection
 *          WTM[4:0]: FIFO threshold. Watermark level setting
 * @page 35
 * 
 * **********************************************************
 */
#define FIFO_BYPASS_MODE             DEFAULT_VALUE
#define FIFO_FIFO_MODE               0x01u
#define FIFO_STREAM_MODE             0x02u
#define FIFO_STREAM_TO_FIFO_MODE     0x03u
#define FIFO_STREAM_TO_BYPASS_MODE   0x04u
#define WTM_THS_LEVEL_DEFAULT        DEFAULT_VALUE        /* Set by user. Value: [0:31] */

/**
 * *********************************************************
 * 
 * @brief Define configure interrupt events on INT1 pin to control device, reference on INT1_CFG
 *          AND/OR[7]: AND/OR combination of Interrupt events
 *          LIR[6]:  Latch Interrupt Request
 *          ZHIE[5]: Enable interrupt generation on Z high event
 *          ZLIE[4]: Enable interrupt generation on Z low event
 *          YHIE[3]: Enable interrupt generation on Y high event
 *          YLIE[2]: Enable interrupt generation on Y low event
 *          XHIE[1]: Enable interrupt generation on X high event
 *          XLIE[0]: Enable interrupt generation on X low event
 * 
 * @page 36
 * 
 * **********************************************************
 */
#define ENABLE_OR_COMBINATION_INTERRUPT_EVENT              DEFAULT_VALUE
#define ENABLE_AND_COMBINATION_INTERRUPT_EVENT             0x01u
#define ENABLE_INTERRUPT_REQUEST_LATCH                     0x01u
#define DISABLE_INTERRUPT_REQUEST_LATCH                    DEFAULT_VALUE
#define ENABLE_INTERRUPT_Z_HIGH_EVENT                      0x01u
#define DISABLE_INTERRUPT_Z_HIGH_EVENT                     DEFAULT_VALUE
#define ENABLE_INTERRUPT_Z_LOW_EVENT                       0x01u
#define DISABLE_INTERRUPT_Z_LOW_EVENT                      DEFAULT_VALUE
#define ENABLE_INTERRUPT_Y_HIGH_EVENT                      0x01u
#define DISABLE_INTERRUPT_Y_HIGH_EVENT                     DEFAULT_VALUE
#define ENABLE_INTERRUPT_Y_LOW_EVENT                       0x01u
#define DISABLE_INTERRUPT_Y_LOW_EVENT                      DEFAULT_VALUE
#define ENABLE_INTERRUPT_X_HIGH_EVENT                      0x01u
#define DISABLE_INTERRUPT_X_HIGH_EVENT                     DEFAULT_VALUE
#define ENABLE_INTERRUPT_X_LOW_EVENT                       0x01u
#define DISABLE_INTERRUPT_X_LOW_EVENT                      DEFAULT_VALUE

/**
 * *********************************************************
 * 
 * @brief Define wait time to control device, reference on INT1_DURATION
 *          WAIT[7]: WAIT enable
 *          D[6:0]: Duration value. This value set by user. Value: [127:0]
 * 
 * @page 38
 * 
 * **********************************************************
 */
#define ENABLE_WAIT_MODE           0x01u
#define DISABLE_WAIT_MODE          DEFAULT_VALUE

/**
 * **************************************************
 * 
 * @brief These structures are used every time the user wants to customize other operating modes for the device.
 *        Users can edit by assigning corresponding values ​​to suit the project.
 *        The L3G4200D_Init_Custom_Mode function is called for these changes to take effect
 *        Must not edit this structs
 * 
 * **************************************************
 */
/* Struct custom mode register CTRL_REG1 */
typedef union
{
    uint8_t CTRL_REG1_Val;
    struct
    {
        uint8_t x_axis_status : 1;
        uint8_t y_axis_status : 1;
        uint8_t z_axis_status : 1;
        uint8_t pwr_mode : 1;
        uint8_t bandwidth : 2;
        uint8_t data_rate : 2;
    }CTRL_REG1_Bits;
}CTRL_REG1_Typedef;
/* Struct custom mode register CTRL_REG2 */
typedef union
{
    uint8_t CTRL_REG2_Val;
    struct
    {
        uint8_t cutoff_frequency : 4;
        uint8_t high_pass_filter_mode : 2;
        uint8_t : 2;
    }CTRL_REG2_Bits;
}CTRL_REG2_Typedef;
/* Struct custom mode register CTRL_REG3 */
typedef union
{
    uint8_t CTRL_REG3_Val;
    struct
    {
        uint8_t itr_fifo_empty_int2_pin_status : 1;
        uint8_t itr_fifo_overun_int2_pin_status : 1;
        uint8_t itr_fifo_wtm_int2_pin_status : 1;
        uint8_t data_rdy_int2_pin_status : 1;
        uint8_t pushpull_opendrain_mode : 1;
        uint8_t itr_active_int1_pin_level : 1;
        uint8_t boot_available_int1_pin_status : 1;
        uint8_t itr_int1_pin_status : 1;
    }CTRL_REG3_Bits;
}CTRL_REG3_Typedef;
/* Struct custom mode register CTRL_REG4 */
typedef union
{
    uint8_t CTRL_REG4_Val;
    struct
    {
        uint8_t spi_interface_mode : 1;
        uint8_t self_test_mode : 2;
        uint8_t : 1;
        uint8_t full_scale_value : 2;
        uint8_t sort_data_mode : 1;
        uint8_t data_update_mode : 1;
    }CTRL_REG4_Bits;
}CTRL_REG4_Typedef;
/* Struct custom mode register CTRL_REG5 */
typedef union
{
    uint8_t CTRL_REG5_Val;
    struct
    {
        uint8_t out_mode : 2;
        uint8_t int1_pin_mode : 2;
        uint8_t high_pass_filter_status : 1;
        uint8_t : 1;
        uint8_t fifo_status : 1;
        uint8_t reboot_mode : 1;
    }CTRL_REG5_Bits;
}CTRL_REG5_Typedef;
/* Struct custom mode register REFERENCE */
typedef struct {
    uint8_t ref_value;
}REFERENCE_Typedef;
/* Struct custom mode register FIFO_CTRL_REG */
typedef union
{
    uint8_t FIFO_CTRL_REG_Val;
    struct
    {
        uint8_t wtm_level : 5;
        uint8_t fifo_mode : 3;
    }FIFO_CTRL_REG_Bits;
}FIFO_CTRL_REG_Typedef;
/* Struct custom mode register INT1_CFG */
typedef union
{
    uint8_t INT1_CFG_Val;
    struct
    {
        uint8_t itr_X_low_event_status : 1;
        uint8_t itr_X_high_event_status : 1;
        uint8_t itr_Y_low_event_status : 1;
        uint8_t itr_Y_high_event_status : 1;
        uint8_t itr_Z_low_event_status : 1;
        uint8_t itr_Z_high_event_status : 1;
        uint8_t latch_itr_status : 1;
        uint8_t AND_OR_mode_select : 1;
    }INT1_CFG_Bits;
}INT1_CFG_Typedef;
/* Struct custom mode register INT1_DURATION */
typedef union
{
    uint8_t INT1_DURATION_Val;
    struct
    {
        uint8_t wait_time : 7;
        uint8_t wait_status : 1;
    }INT1_DURATION_Bits;
}INT1_DURATION_Typedef;
/* Struct custom mode register INT1_THS_XH */
typedef struct
{
    uint8_t value;           /* Value: [0:127] */
}INT1_THS_XH_Typedef;
/* Struct custom mode register INT1_THS_XH */
typedef struct
{
    uint8_t value;          /* Value: [0:255] */
}INT1_THS_XL_Typedef;
/* Struct custom mode register INT1_THS_YH */
typedef struct
{
    uint8_t value;          /* Value: [0:127] */
}INT1_THS_YH_Typedef;

/* Struct custom mode register NT1_THS_YL */
typedef struct
{
    uint8_t value;          /* Value: [0:255] */
}INT1_THS_YL_Typedef;

/* Struct custom mode register NT1_THS_ZH */
typedef struct
{
    uint8_t value;         /* Value: [0:127] */
}INT1_THS_ZH_Typedef;

/* Struct custom mode register NT1_THS_ZL */
typedef struct
{
    uint8_t value;        /* Value: [0:255] */
}INT1_THS_ZL_Typedef;
/* Struct is to contain all modifiable function registers */
typedef struct
{
    CTRL_REG1_Typedef reg1;
    CTRL_REG2_Typedef reg2;
    CTRL_REG3_Typedef reg3;
    CTRL_REG4_Typedef reg4;
    CTRL_REG5_Typedef reg5;
    REFERENCE_Typedef ref_conf;
    FIFO_CTRL_REG_Typedef fifo_ctr_conf;
    INT1_CFG_Typedef int1_conf;
    INT1_THS_XH_Typedef ths_xh;
    INT1_THS_XL_Typedef ths_xl;
    INT1_THS_YH_Typedef ths_yh;
    INT1_THS_YL_Typedef ths_yl;
    INT1_THS_ZH_Typedef ths_zh;
    INT1_THS_ZL_Typedef ths_zl;
    INT1_DURATION_Typedef int1_dura;
}DevMod_Typedef;

/**
 * @brief This struct to modify features and it is edited by user. User should be copy to main.c file,
 *        Here cannot create a variable of data type Device_ModifyTypedef here.
 */
// DevMod_Typedef User_Mod = {
//    .reg1 = {
//        .CTRL_REG1_Bits.data_rate = OUTPUT_DATA_RATE_100,
//        .CTRL_REG1_Bits.bandwidth = BANDWIDTH_1,
//        .CTRL_REG1_Bits.pwr_mode = PWR_NORMAL_MODE,
//        .CTRL_REG1_Bits.x_axis_status = ENABLE_X_AXIS,
//        .CTRL_REG1_Bits.y_axis_status = ENABLE_Y_AXIS,
//        .CTRL_REG1_Bits.z_axis_status = ENABLE_Z_AXIS
//    },
//    .reg2 = {
//        .CTRL_REG2_Bits.cutoff_frequency = HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_5,
//        .CTRL_REG2_Bits.high_pass_filter_mode = NORMAL_MODE
//    },
//    .reg3 = {
//        .CTRL_REG3_Bits.boot_available_int1_pin_status = DISABLE_BOOT_STATUS_ON_INT1,
//        .CTRL_REG3_Bits.data_rdy_int2_pin_status = DISABLE_DATA_RDY_ON_INT2,
//        .CTRL_REG3_Bits.itr_active_int1_pin_level = ITR_ACTIVE_ON_INT1_HIGH,
//        .CTRL_REG3_Bits.itr_fifo_empty_int2_pin_status = DISABLE_ITR_FIFO_EMPTY_ON_INT2,
//        .CTRL_REG3_Bits.itr_fifo_overun_int2_pin_status = DISABLE_ITR_FIFO_ORUN_ON_INT2,
//        .CTRL_REG3_Bits.itr_fifo_wtm_int2_pin_status = DISABLE_ITR_FIFO_WTM_ON_INT2,
//        .CTRL_REG3_Bits.itr_int1_pin_status = DISABLE_ITR_ON_INT1,
//        .CTRL_REG3_Bits.pushpull_opendrain_mode = PUSH_PULL_ENABLE
//    },
//    .reg4 = {
//        .CTRL_REG4_Bits.data_update_mode = DBU_CONTINUOUS_UPDATE,
//        .CTRL_REG4_Bits.full_scale_value = FULL_SCALE_MODE_1,
//        .CTRL_REG4_Bits.self_test_mode = SELF_TEST_NORMAL_MODE,
//        .CTRL_REG4_Bits.sort_data_mode = BLE_DATA_LSB_LOW_ADD,
//        .CTRL_REG4_Bits.spi_interface_mode = SPI_INTERFACE_4WIRE_MODE
//    },
//    .reg5 = {
//        .CTRL_REG5_Bits.fifo_status = DISABLE_FIFO,
//        .CTRL_REG5_Bits.high_pass_filter_status = DISABLE_HIGH_PASS_FILTER,
//        .CTRL_REG5_Bits.int1_pin_mode = INT1_SELECT_MODE_1,
//        .CTRL_REG5_Bits.out_mode = OUT_SELECT_MODE_1,
//        .CTRL_REG5_Bits.reboot_mode = BOOT_NORMAL_MODE
//    },
//    .fifo_ctr_conf = {
//        .FIFO_CTRL_REG_Bits.fifo_mode = FIFO_BYPASS_MODE,
//        .FIFO_CTRL_REG_Bits.wtm_level = DEFAULT_VALUE          /* Set by user. Value: [0:31] */
//    },
//    .int1_conf = {
//        .INT1_CFG_Bits.AND_OR_mode_select = ENABLE_OR_COMBINATION_INTERRUPT_EVENT,
//        .INT1_CFG_Bits.itr_X_high_event_status = DISABLE_INTERRUPT_X_HIGH_EVENT,
//        .INT1_CFG_Bits.itr_X_low_event_status = DISABLE_INTERRUPT_X_LOW_EVENT,
//        .INT1_CFG_Bits.itr_Y_high_event_status = DISABLE_INTERRUPT_Y_HIGH_EVENT,
//        .INT1_CFG_Bits.itr_Y_low_event_status = DISABLE_INTERRUPT_Y_LOW_EVENT,
//        .INT1_CFG_Bits.itr_Z_high_event_status = DISABLE_INTERRUPT_Z_HIGH_EVENT,
//        .INT1_CFG_Bits.itr_Z_low_event_status = DISABLE_INTERRUPT_Z_LOW_EVENT,
//        .INT1_CFG_Bits.latch_itr_status = DISABLE_INTERRUPT_REQUEST_LATCH
//    },
//    .int1_dura = {
//        .INT1_DURATION_Bits.wait_status = DISABLE_WAIT_MODE,
//        .INT1_DURATION_Bits.wait_time = DEFAULT_VALUE         /* Set by user. Value: [0:127] */
//    },
//    .ref_conf = {
//        .ref_value = DEFAULT_VALUE         /* Set by user. Value: [0:255] */
//    },
//    .ths_xh = {
//        .value = DEFAULT_VALUE             /* Set by user. Value: [0:127] */
//    },
//    .ths_xl = {
//        .value = DEFAULT_VALUE             /* Set by user. Value: [0:255] */
//    },
//    .ths_yh = {
//        .value = DEFAULT_VALUE             /* Set by user. Value: [0:127] */
//    },
//    .ths_yl = {
//        .value = DEFAULT_VALUE             /* Set by user. Value: [0:255] */
//    },
//    .ths_zh = {
//        .value = DEFAULT_VALUE             /* Set by user. Value: [0:127] */
//    },
//    .ths_zl = {
//        .value = DEFAULT_VALUE             /* Set by user. Value: [0:255] */
//    }
// };

/**
 * @brief Sensor struct
 */
typedef struct
{
    /* Get Device ID */
    uint8_t ID;
    /* I2C handle */
    SPI_HandleTypeDef *spiHandle;
    /* Acceleration data (X, Y, Z) */
    volatile float Acc_X;
    volatile float Acc_Y;
    volatile float Acc_Z;
    /* Temperature data in deg */
    volatile float Temperature;
    /* GPIO NSS/CC */
    GPIO_TypeDef *GPIOPort;
    uint16_t GPIO_Pin;

}L3G4200D_SPI4W_Typedef;

/**
 * @brief This APIs to help user to control device use I2C protocol
 */
uint8_t L3G4200D_SPI_4WIRE_Init(L3G4200D_SPI4W_Typedef *dev, SPI_HandleTypeDef *spiHandle, DevMod_Typedef *device_mod, GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_X);
uint8_t L3G4200D_SPI_4WIRE_DeInit(L3G4200D_SPI4W_Typedef *dev);
uint8_t L3G4200D_SPI_4WIRE_GetTemperature(L3G4200D_SPI4W_Typedef *dev);
uint8_t L3G4200D_SPI_4WIRE_Get_X_Axis_Accelerations(L3G4200D_SPI4W_Typedef *dev);
uint8_t L3G4200D_SPI_4WIRE_Get_Y_Axis_Accelerations(L3G4200D_SPI4W_Typedef *dev);
uint8_t L3G4200D_SPI_4WIRE_Get_Z_Axis_Accelerations(L3G4200D_SPI4W_Typedef *dev);
uint8_t L3G4200D_SPI_4WIRE_ReadRegister(L3G4200D_SPI4W_Typedef *dev, uint8_t reg_add, uint8_t *data_buf);
uint8_t L3G4200D_SPI_4WIRE_WriteRegister(L3G4200D_SPI4W_Typedef *dev, uint8_t reg_add, uint8_t *pTxData);

#endif    //#ifndef __L3G4200D_H