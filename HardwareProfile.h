/* 
 * File:   HardwareProfile.h
 * Author: jecraig
 *
 * Created on January 13, 2014, 8:41 AM
 */

#ifndef HARDWAREPROFILE_H
#define	HARDWAREPROFILE_H

#include <xc.h>
#include <pic18.h>
#include <plib.h>
#include <i2c_interface.h>

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef INPUT_PIN
#define INPUT_PIN 1
#endif
#ifndef OUTPUT_PIN
#define OUTPUT_PIN 0
#endif

#define SYSTEM_CLOCK    (_XTAL_FREQ << 2)
#define _XTAL_FREQ      16000000ul
#define I2C_FREQ        400000
#define UART_BAUD_RATE  9600
#define USE_AND_MASK
    
#define INA219_I2C_BUS          I2CBUS1
#define INA219_I2C_ADDR_SERVO0  0x40
#define INA219_I2C_ADDR_SERVO1  0x41
#define INA219_I2C_ADDR_SERVO2  0x44
    
#define EEPROM_I2C_BUS          I2CBUS1
#define EEPROM_I2C_ADDR         0x50


#ifdef	__cplusplus
}
#endif

#endif	/* HARDWAREPROFILE_H */

