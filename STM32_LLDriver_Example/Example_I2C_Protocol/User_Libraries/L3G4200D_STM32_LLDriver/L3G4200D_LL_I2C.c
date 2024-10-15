/**
  ******************************************************************************
  * @file    L3G4200D_LL_I2C.c
  * @author  MinhChi
  * @brief   L3G4200D sensor driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the L3G4200D
  *
  ******************************************************************************
*/

#include "L3G4200D_LL_I2C.h"

/**
 * ***********************************************************************************************
 * @brief This function initializes everything necessary about the sensor in custom mode by user
 *        to be able to read the values ​​of the 3 axes and the temperature.
 * @param dev: As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @param device_add Device address
 * @param i2cHandle: Contains I2C configuration information that has been previously configured by the user using the HAL Driver
 * @param device_mod: Contains all config mode behavior device by user
 * @retval 255: Device identify different from L3G4200D sensor
 * @retval 200: The pointer passed in is a NULL pointer
 * ***********************************************************************************************
 */
uint8_t L3G4200D_I2C_Init(L3G4200D_I2C_Typedef *dev, I2C_TypeDef *i2cChanel, DevMod_Typedef *device_mod, uint8_t dev_add)
{
    /* Check NULL pointer */
    if (dev == NULL || i2cChanel == NULL || device_mod == NULL)
    {
        return 200;
    }
    LL_I2C_Disable(i2cChanel);
    /* Reset L3G4200D struct parameter */
    dev->i2cChanel = i2cChanel;
    dev->Acc_X = 0.0f;
    dev->Acc_Y = 0.0f;
    dev->Acc_Z = 0.0f;
    dev->Temperature = 0.0f;
    dev->ID = 0;
    dev->DEV_ADD = dev_add;
    typedef struct
    {
        uint8_t Reg_add;
        uint8_t Reg_val;

    }Register_Config;
    /* Check device ID */
    L3G4200D_I2C_ReadRegister(dev, WHO_AM_I, &dev->ID);
    if(dev->ID != L3G4200D_IDENTIFY)
        return 255;
    /* Init data to write into register to enable mode config by user */
    Register_Config Config[] = {
        {CTRL_REG1, device_mod->reg1.CTRL_REG1_Val},
        {CTRL_REG2, device_mod->reg2.CTRL_REG2_Val},
        {CTRL_REG3, device_mod->reg3.CTRL_REG3_Val},
        {CTRL_REG4, device_mod->reg4.CTRL_REG4_Val},
        {CTRL_REG5, device_mod->reg5.CTRL_REG5_Val},
        {REFERENCE, device_mod->ref_conf.ref_value},
        {FIFO_CTRL_REG, device_mod->fifo_ctr_conf.FIFO_CTRL_REG_Val},
        {INT1_CFG, device_mod->int1_conf.INT1_CFG_Val},
        {INT1_THS_XH, device_mod->ths_xh.value},
        {INT1_THS_XL, device_mod->ths_xl.value},
        {INT1_THS_YH, device_mod->ths_yh.value},
        {INT1_THS_YL, device_mod->ths_yl.value},
        {INT1_THS_ZH, device_mod->ths_zh.value},
        {INT1_THS_ZL, device_mod->ths_zl.value},
        {INT1_DURATION, device_mod->int1_dura.INT1_DURATION_Val}
    };
    for(uint8_t i = 0; i < (sizeof(Config)/sizeof(Config[0])); i++)
    {
        L3G4200D_I2C_WriteRegister(dev, Config[i].Reg_add, &Config[i].Reg_val);
    }
    return 0;
}

/**
 * ***********************************************************************************************
 * @brief This function reset and delete everything parameter necessary about the sensor in default mode 
 *        to be able to read the values ​​of the 3 axes and the temperature.
 * @param dev: As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @retval 200: The pointer passed in is a NULL pointer
 * ***********************************************************************************************
 */
uint8_t L3G4200D_I2C_DeInit(L3G4200D_I2C_Typedef *dev)
{
    /* Check NULL pointer */
    if (dev == NULL)
    {
        return 200;
    }
    /* Reset parameter L3G4200D struct */
    dev->i2cChanel = NULL;
    dev->Acc_X = 0.0f;
    dev->Acc_Y = 0.0f;
    dev->Acc_Z = 0.0f;
    dev->Temperature = 0.0f;
    dev->ID = 0;
    dev->DEV_ADD = 0;
    return 0;
}

/**
 * **************************************************************
 * @brief Read the temperature value of an L3G4200D sensor
 * @param dev As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @retval 200: The pointer passed in is a NULL pointer
 * **************************************************************
 */
uint8_t L3G4200D_I2C_GetTemperature(L3G4200D_I2C_Typedef *dev)
{
    /* Check NULL pointer */
    if (dev == NULL)
        return 200;
    uint8_t Temp;
    /* Read register value OUT_TEMP */
    L3G4200D_I2C_ReadRegister(dev, OUT_TEMP, &Temp);
    dev->Temperature = (float)Temp;
    return 0;
}

/**
 * **************************************************************
 * @brief Read X axis accelerations value of L3G4200D sensor
 * @param dev As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @retval 200: The pointer passed in is a NULL pointer
 * **************************************************************
 */
uint8_t L3G4200D_I2C_Get_X_Axis_Accelerations(L3G4200D_I2C_Typedef *dev)
{
    /* Check NULL pointer */
    if (dev == NULL)
        return 200;
    uint8_t _rawdata_H, _rawdata_L, _fs_val, _ble_val;
    int16_t _raw_X_axis;
    float _sensitivity;
    /* Check bit BLE in CTRL_REG4 register */
    L3G4200D_I2C_ReadRegister(dev, CTRL_REG4, &_ble_val);
    if ((_ble_val & 0x40) == 0)
    {
        /* Read register value OUT_X_L */
        L3G4200D_I2C_ReadRegister(dev, OUT_X_L, &_rawdata_L);
        /* Read register value OUT_X_H */
        L3G4200D_I2C_ReadRegister(dev, OUT_X_H, &_rawdata_H);
    }
    else
    {
        /* Read register value OUT_X_H */
        L3G4200D_I2C_ReadRegister(dev, OUT_X_H, &_rawdata_H);
        /* Read register value OUT_X_L */
        L3G4200D_I2C_ReadRegister(dev, OUT_X_L, &_rawdata_L);
    }
    /* Merge register value OUT_X_H and register value OUT_X_L */
    _raw_X_axis = (int16_t)(_rawdata_H << 8 | _rawdata_L);
    /* Check full scale value in CTRL_REG4 to convert sensitivity */
    L3G4200D_I2C_ReadRegister(dev, CTRL_REG4, &_fs_val);
    if ((_fs_val & 0x30) == 0)
    {
        _sensitivity = 0.00875;
    }
    else if ((_fs_val & 0x30) == 0x10)
    {
        _sensitivity = 0.0175;
    }
    else
    {
        _sensitivity = 0.07;
    }
    /* Real X axis accelerations value */
    dev->Acc_X = _raw_X_axis * _sensitivity;     /* Unit: dps */
    return 0;
}

/**
 * **************************************************************
 * @brief Read Y axis accelerations value of L3G4200D sensor
 * @param dev As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @retval 200: The pointer passed in is a NULL pointer
 * @retval i2c_status: If i2c_status != 0, I2C transmission is failed
 * **************************************************************
 */
uint8_t L3G4200D_I2C_Get_Y_Axis_Accelerations(L3G4200D_I2C_Typedef *dev)
{
    /* Check NULL pointer */
    if (dev == NULL)
        return 200;
    uint8_t _rawdata_H, _rawdata_L, _fs_val, _ble_val;
    int16_t _raw_Y_axis;
    float _sensitivity;
    /* Check bit BLE in CTRL_REG4 register */
    L3G4200D_I2C_ReadRegister(dev, CTRL_REG4, &_ble_val);
    if ((_ble_val & 0x40) == 0)
    {
        /* Read register value OUT_Y_L */
        L3G4200D_I2C_ReadRegister(dev, OUT_Y_L, &_rawdata_L);
        /* Read register value OUT_Y_H */
        L3G4200D_I2C_ReadRegister(dev, OUT_Y_H, &_rawdata_H);
    }
    else
    {
        /* Read register value OUT_Y_H */
        L3G4200D_I2C_ReadRegister(dev, OUT_Y_H, &_rawdata_H);
        /* Read register value OUT_Y_L */
        L3G4200D_I2C_ReadRegister(dev, OUT_Y_L, &_rawdata_L);
    }
    /* Merge register value OUT_Y_H and register value OUT_Y_L */
    _raw_Y_axis = (int16_t)(_rawdata_H << 8 | _rawdata_L);
    /* Check full scale value in CTRL_REG4 to convert sensitivity */
    L3G4200D_I2C_ReadRegister(dev, CTRL_REG4, &_fs_val);
    if ((_fs_val & 0x30) == 0)
        _sensitivity = 0.00875;
    else if ((_fs_val & 0x30) == 0x10)
        _sensitivity = 0.0175;
    else
        _sensitivity = 0.07;
    /* Real Y axis accelerations value */
    dev->Acc_Y = _raw_Y_axis * _sensitivity;     /* Unit: dps */
    return 0;
}

/**
 * **************************************************************
 * @brief Read Z axis accelerations value of L3G4200D sensor
 * @param dev As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @retval 200: The pointer passed in is a NULL pointer
 * @retval i2c_status: If i2c_status != 0, I2C transmission is failed
 * **************************************************************
 */
uint8_t L3G4200D_I2C_Get_Z_Axis_Accelerations(L3G4200D_I2C_Typedef *dev)
{
    /* Check NULL pointer */
    if (dev == NULL)
        return 200;
    uint8_t _rawdata_H, _rawdata_L, _fs_val, _ble_val;
    int16_t _raw_Z_axis;
    float _sensitivity;
    /* Check bit BLE in CTRL_REG4 register */
    L3G4200D_I2C_ReadRegister(dev, CTRL_REG4, &_ble_val);
    if ((_ble_val & 0x40) == 0)
    {
        /* Read register value OUT_Z_L */
        L3G4200D_I2C_ReadRegister(dev, OUT_Z_L, &_rawdata_L);
        /* Read register value OUT_Z_H */
        L3G4200D_I2C_ReadRegister(dev, OUT_Z_H, &_rawdata_H);
    }
    else
    {
        /* Read register value OUT_Z_H */
        L3G4200D_I2C_ReadRegister(dev, OUT_Z_H, &_rawdata_H);
        /* Read register value OUT_Z_L */
        L3G4200D_I2C_ReadRegister(dev, OUT_Z_L, &_rawdata_L);
    }
    /* Merge register value OUT_Z_H and register value OUT_Z_L */
    _raw_Z_axis = (int16_t)(_rawdata_H << 8 | _rawdata_L);
    /* Check full scale value in CTRL_REG4 to convert sensitivity */
    L3G4200D_I2C_ReadRegister(dev, CTRL_REG4, &_fs_val);
    if ((_fs_val & 0x30) == 0)
        _sensitivity = 0.00875;
    else if ((_fs_val & 0x30) == 0x10)
        _sensitivity = 0.0175;
    else
        _sensitivity = 0.07;
    /* Real Z axis accelerations value */
    dev->Acc_Z = _raw_Z_axis * _sensitivity;     /* Unit: dps */
    return 0;
}

/**
 * **************************************************************
 * @brief Read the value of an 8-bit register L3G4200D sensor
 * @param dev As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @param reg_add Register address
 * @param data_buffer Buffer to store data from register
 * @retval 200: The pointer passed in is a NULL pointer
 * **************************************************************
 */
uint8_t L3G4200D_I2C_ReadRegister(L3G4200D_I2C_Typedef *dev, uint8_t reg_add, uint8_t *data_buf)
{
    /* Check NULL pointer */
    if (data_buf == NULL || dev == NULL)
        return 200;
    uint8_t device_add, write_request;
    if(dev->DEV_ADD == 0x68)
    {
        device_add = 0xD1;
        write_request = 0xD0;
    }
    else
    {
        device_add = 0xD3;
        write_request = 0xD2;
    }
    /* Check Bus BUSY */
    while(LL_I2C_IsActiveFlag_BUSY(dev->i2cChanel));
    /* Enable I2C */
    LL_I2C_Enable(dev->i2cChanel);
    /* Send start bit (SB) to slave */
    LL_I2C_GenerateStartCondition(dev->i2cChanel);
    /* Wait until SB flag was set */
    while (!LL_I2C_IsActiveFlag_SB(dev->i2cChanel));
    /* Send device address with write request */
    LL_I2C_TransmitData8(dev->i2cChanel, write_request);
    /* Wait slave respones match address */
    while (!LL_I2C_IsActiveFlag_ADDR(dev->i2cChanel));
    /* Clear match address flag */
    LL_I2C_ClearFlag_ADDR(dev->i2cChanel);
    /* Wait TX buffer is empty */
    while (!LL_I2C_IsActiveFlag_TXE(dev->i2cChanel));
    /* Send register address to read */
    LL_I2C_TransmitData8(dev->i2cChanel, reg_add);
    /* Send repeat start (SR) to slave */
    LL_I2C_GenerateStartCondition(dev->i2cChanel);
    /* Wait until SB flag was set again */
    while (!LL_I2C_IsActiveFlag_SB(dev->i2cChanel));
    /* Send device address with read request */
    LL_I2C_TransmitData8(dev->i2cChanel, device_add);
    /* Wait slave respones match address */
    while (!LL_I2C_IsActiveFlag_ADDR(dev->i2cChanel));
    /* Clear match address flag */
    LL_I2C_ClearFlag_ADDR(dev->i2cChanel);
    /* Wait until Rx not empty */
    while (!LL_I2C_IsActiveFlag_RXNE(dev->i2cChanel));
    /* Read data from internal buffer */
    *data_buf = LL_I2C_ReceiveData8(dev->i2cChanel);
    /* Send stop bit (SP) to slave */
    LL_I2C_GenerateStopCondition(dev->i2cChanel);
    LL_I2C_Disable(dev->i2cChanel);
    return 0;
}

/**
 * **************************************************************
 * @brief Write data into 8-bit register L3G4200D sensor
 * @param dev As an initialization variable to store protocol information, the buffer stores data from the sensor.
 * @param reg_add Register address
 * @param data_send Data buffer to write into register
 * @retval 200: The pointer passed in is a NULL pointer
 * **************************************************************
 */
uint8_t L3G4200D_I2C_WriteRegister(L3G4200D_I2C_Typedef *dev, uint8_t reg_add, uint8_t *data_send)
{
    /* Check NULL pointer */
    if (data_send == NULL || dev == NULL)
        return 200;
    uint8_t device_add;
    if(dev->DEV_ADD == 0x68)
        device_add = 0xD0;
    else
        device_add = 0xD2;
    /* Check Bus BUSY */
    while(LL_I2C_IsActiveFlag_BUSY(dev->i2cChanel));
    /* Enable I2C */
    LL_I2C_Enable(dev->i2cChanel);
    /* Send start bit (SB) to slave */
    LL_I2C_GenerateStartCondition(dev->i2cChanel);
    /* Wait until SB flag was set */
    while (!LL_I2C_IsActiveFlag_SB(dev->i2cChanel));
    /* Send device address with write request */
    LL_I2C_TransmitData8(dev->i2cChanel, device_add);
    /* Wait slave respones match address */
    while (!LL_I2C_IsActiveFlag_ADDR(dev->i2cChanel));
    /* Clear match address flash */
    LL_I2C_ClearFlag_ADDR(dev->i2cChanel);
    /* Wait TXE flag empty */
    while (!LL_I2C_IsActiveFlag_TXE(dev->i2cChanel));
    /* Send register address to be write*/
    LL_I2C_TransmitData8(dev->i2cChanel, reg_add);
    /* Wait TXE flag empty */
    while (!LL_I2C_IsActiveFlag_TXE(dev->i2cChanel));
    /* Send data to write into register address */
    LL_I2C_TransmitData8(dev->i2cChanel, *data_send);
    /* Wait transmission successful */
    while (!LL_I2C_IsActiveFlag_BTF(dev->i2cChanel));
    /* Send stop bit (SP) to slave */
    LL_I2C_GenerateStopCondition(dev->i2cChanel);
    LL_I2C_Disable(dev->i2cChanel);
    return 0;
}