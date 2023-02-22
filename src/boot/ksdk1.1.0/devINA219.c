#include <stdint.h>
#include <stdlib.h>
 #include <math.h>

#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"

#define INA219_DEBUG 0

extern volatile WarpI2CDeviceState    deviceINA219State;
extern volatile uint32_t        gWarpI2cBaudRateKbps;
extern volatile uint32_t        gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t        gWarpSupplySettlingDelayMilliseconds;

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kINA219PinSCL		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kINA219PinSDA		= GPIO_MAKE_PIN(HW_GPIOA, 9),
};

void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure I2C to be on PTA8 and PTA9 for SCL and SDA respectively.
	 */
    
    deviceINA219State.i2cAddress            = i2cAddress;
    deviceINA219State.operatingVoltageMillivolts    = operatingVoltageMillivolts;
    
    
    devINA219writeRegister(kINA219RegisterConfig, config.raw_val);
    devINA219writeRegister(kINA219RegisterCalibration, INA219_CAL);

    OSA_TimeDelay(100);

    devINA219writeRegisterPointer(kINA219RegisterCurrent);

    
    PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt2);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt2);
    
    return;
}

WarpStatus
devINA219writeRegister(INA219Register deviceRegister, uint16_t payload)
{
    uint8_t        payloadByte[2], commandByte[1];
    i2c_status_t    status;

    if (deviceRegister > 0x05) {
        return kWarpStatusBadDeviceCommand;
    }

    i2c_device_t slave =
    {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    commandByte[0] = deviceRegister;
    payloadByte[0] = (uint8_t) (payload >> 8);
    payloadByte[1] = (uint8_t) (payload & 0xFF);
    warpEnableI2Cpins();

    status = I2C_DRV_MasterSendDataBlocking(
                            0 /* I2C instance */,
                            &slave,
                            commandByte,
                            1,
                            payloadByte,
                            2,
                            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
devINA219writeRegisterPointer(INA219Register deviceRegister)
{
    uint8_t        commandByte[1];
    i2c_status_t    status;

    if (deviceRegister > 0x05) {
        return kWarpStatusBadDeviceCommand;
    }

    i2c_device_t slave =
    {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    commandByte[0] = deviceRegister;
    warpEnableI2Cpins();

    status = I2C_DRV_MasterSendDataBlocking(
                            0 /* I2C instance */,
                            &slave,
                            commandByte,
                            1,
                            NULL,
                            0,
                            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
devINA219read(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t        cmdBuf[1] = {0xFF};
    i2c_status_t    status;

    USED(2);

    i2c_device_t slave =
    {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    cmdBuf[0] = deviceRegister;
    warpEnableI2Cpins();

    status = I2C_DRV_MasterReceiveDataBlocking(
                            0 /* I2C instance */,
                            &slave,
                            cmdBuf,
                            0,
                            (uint8_t *)deviceINA219State.i2cBuffer,
                            2,
                            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

#if (INA219_DEBUG)
    extern void warpPrint(const char *fmt, ...);
#endif

int
devINA219getCurrent(void)
{
    uint16_t current_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterCurrent);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read(kINA219RegisterCurrent,2);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    current_raw = (int16_t) (
        deviceINA219State.i2cBuffer[1] |
        deviceINA219State.i2cBuffer[0] << 8
    );

    return (int) (current_raw * INA219_CURRENT_LSB);
}

unsigned int
devINA219getBusVoltage(void)
{
    ina219_reg_bus_voltage_t voltage_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterBusVoltage);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read(kINA219RegisterBusVoltage,2);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    voltage_raw.lsb = deviceINA219State.i2cBuffer[1];
    voltage_raw.msb = deviceINA219State.i2cBuffer[0];

    /* LSB = 4mV */
    return (unsigned int) (voltage_raw.bd * 4);
}

int
devINA219getShuntVoltage(void)
{
    int16_t voltage_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterShuntVoltage);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read(kINA219RegisterShuntVoltage,2);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    voltage_raw = (int16_t) (
        deviceINA219State.i2cBuffer[1] |
        deviceINA219State.i2cBuffer[0] << 8
    );

    /* LSB = 10uV */
    return (int) (voltage_raw * 10);
}


unsigned int
devINA219getPower(void)
{
    uint16_t power_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterPower);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read(kINA219RegisterPower,2);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    power_raw = (uint16_t) (
        deviceINA219State.i2cBuffer[1] |
        deviceINA219State.i2cBuffer[0] << 8
    );

    /* LSB = 4mV */
    return (unsigned int) (power_raw * INA219_POWER_LSB);
}
