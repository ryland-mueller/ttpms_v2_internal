#include "I2C_Functions.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>

/* includes for debugging/temporary */
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(ttpms);

const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0)); // is something like "chosen" in devicetree for I2C (see how CAN device is done)


int I2C_Init()  // this doesn't actually init anything lol
{   
    if(!device_is_ready(i2c_dev)) {
        return -1;
    } else {
        return 0;
    }
    //i2c_configure should be uneccessary since we set 400KHz in devicetree
}

int I2C_GeneralReset(void)  // this function is originally for MLX one-shot mode, will not be used
{    
    int ack;
    uint8_t cmd = 0x06;   
 
    // writing 0x06 to address 0x00 will reset all devices on I2C
    ack = i2c_write(i2c_dev, &cmd, 1, 0x00);

    if (ack != 0x00)
    {
        LOG_ERR("Error sending I2C reset message");
        return -1;
    }         

    return 0;
}

int MLX_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{                          
    int ack = 0;                               
    int cnt = 0;
    int i = 0;
    char cmd[2] = {0,0};
    char i2cData[1664] = {0};
    uint16_t *p;

    p = data;
    cmd[0] = startAddress >> 8;
    cmd[1] = startAddress & 0x00FF;

    ack = i2c_write_read(i2c_dev, slaveAddr, cmd, 2, i2cData, 2*nMemAddressRead);

    if (ack != 0x00)
    {
        LOG_ERR("Error reading from MLX over I2C");
        return -1;
    }

    for(cnt=0; cnt < nMemAddressRead; cnt++)    // this just turns uint8_t[2*nMemAddressRead] into uint16_t[nMemAddressRead]
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i]*256 + (uint16_t)i2cData[i+1];
    }

    return 0;   
} 

int MLX_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    int ack = 0;
    char cmd[4] = {0,0,0,0};
    static uint16_t dataCheck;

    cmd[0] = writeAddress >> 8;
    cmd[1] = writeAddress & 0x00FF;
    cmd[2] = data >> 8;
    cmd[3] = data & 0x00FF;

    //k_sleep(K_USEC(5));

    ack = i2c_write(i2c_dev, cmd, 4, slaveAddr);

    if (ack != 0x00)
    {
        //LOG_ERR("Error writing to MLX over I2C");
        return -1;
    }         

    MLX_I2CRead(slaveAddr, writeAddress, 1, &dataCheck);

    if (dataCheck != data)
    {
        //LOG_ERR("Error writing to MLX, readback failed");
        //LOG_ERR("dataCheck: %d  data: %d", dataCheck, data);
        return -2;
    }    

    return 0;
}