#include <xc.h>
#include <plib.h>
#include <HardwareProfile.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <i2c_interface.h>
#include <CurrentSensor_INA219.h>
#include <eeprom_interface.h>


// PIC18F26K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = ON      // 4X PLL Enable (Oscillator multiplied by 4)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 285       // Brown Out Reset Voltage bits (VBOR set to 2.85 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stuck status)
#pragma config T3CMX = PORTB5   // Timer3 Clock input mux bit (T3CKI is on RC5)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#define SERVO_0_BASE_ADDR   0x00
#define SERVO_1_BASE_ADDR   0x40
#define SERVO_2_BASE_ADDR   0x80


#define MAX_PULSE_WIDTH_ERROR 20
#define SERVO_NEUTRAL   0
#define SERVO_CW        1
#define SERVO_CCW       2
#define MAX_ANGLE   180
#define MIN_ANGLE   0
#define DEFAULT_PULSE_WIDTH 2300

void SerialPrint(char* buffer);
unsigned int GetServoSetting(unsigned char servo, double angle);
int SetServoSetting(unsigned char servo, double angle, unsigned int pulse_width);
void SetServoAngle(unsigned char servo, double angle);
void DisableServo(unsigned char servo);
void SetPWM(unsigned char servo, unsigned int pulse_width);
unsigned int ReadADC(unsigned char pin, unsigned int tad, unsigned int num_samples);

unsigned char GetControlRegister(unsigned char addr);
void SetControlRegister(unsigned char addr, unsigned char data);

//Used by the I2C slave code
volatile unsigned int i2c_request_addr = 0x00;
volatile unsigned int i2c_byte_count = 0;

typedef struct ServoControl_struct {
    volatile unsigned int on_ticks;
    volatile unsigned int off_ticks;
    volatile unsigned int pending_on_ticks;
    volatile unsigned int pending_off_ticks;
    volatile unsigned char pin_output;
    volatile unsigned int pulse_count;
    volatile unsigned int total_pulse_count;
    double target_angle;
    double current_angle;
    double previous_current_angle;
    double max_angle;
    double min_angle;
    unsigned int current;
    unsigned int current_adc_value;
    unsigned int cache_value;
    unsigned int max_current;
    volatile unsigned char status_reg1_shadow;
    volatile unsigned char status_reg2_shadow;
    volatile unsigned char active;
    volatile unsigned char active_pending;
    volatile unsigned char active_change_pending;
    volatile unsigned char update_pending;
    volatile double target_angle_pending;
    volatile unsigned char target_angle_change_pending;
    unsigned char scanning;
    volatile unsigned char scanning_enabled;
    unsigned char locked;
    unsigned char stuck;
    unsigned char over_current;
    unsigned char cache_valid;
    unsigned char cache_written;
    unsigned char current_sense_i2c_addr;
    unsigned int min_pulse;
    unsigned int max_pulse;
    unsigned int adc_offset;
    double adc_conversion;
} ServoControl;

ServoControl servo_control[4]; //only 3 are used, the fourth is to avoid i2c requests crashing the part if the servo select is invalid

int main(void) {
    double current;
    char buffer[256];
    double servo_angle;
    unsigned char servo;
    unsigned long long int loop_counter = 1;
    unsigned char i2c_slave_addr;


    OSCCONbits.IRCF = 0x7; //16 Mhz Internal Clock (with 4x PLL mult))


    TRISAbits.TRISA3 = INPUT_PIN;
    TRISAbits.TRISA4 = INPUT_PIN;
    TRISAbits.TRISA5 = INPUT_PIN;

    LATCbits.LATC2 = 0;
    TRISCbits.TRISC2 = INPUT_PIN;
    LATCbits.LATC1 = 0;
    TRISCbits.TRISC1 = INPUT_PIN;
    LATCbits.LATC0 = 0;
    TRISCbits.TRISC0 = INPUT_PIN;

    //Disable pins used for I2C and UART as analog inputs
    ANSA3 = 0;
    ANSA5 = 0;
    ANSB5 = 0;
    ANSB2 = 0;
    ANSB1 = 0;
    ANSC2 = 0;
    ANSC3 = 0;
    ANSC4 = 0;
    ANSC6 = 0;
    ANSC7 = 0;

    //Enable pins used as analog inputs
    ANSA0 = 1;
    ANSA1 = 1;
    ANSA2 = 1;

    //Disable Comparators
    C1ON = 0;
    C2ON = 0;

    //Disable External Interrupts
    INT0IE = 0;
    INT1IE = 0;
    INT2IE = 0;

    //TRISBbits.TRISB1 = INPUT_PIN;
    //TRISBbits.TRISB2 = INPUT_PIN;

    //Disable Port B Pull-Ups
    INTCON2bits.RBPU = 1;



    for (servo = 0; servo < 3; ++servo) {
        servo_control[servo].active = 0;
        servo_control[servo].active_pending = 0;
        servo_control[servo].active_change_pending = 0;
        servo_control[servo].update_pending = 0;
        servo_control[servo].target_angle_change_pending = 0;
        servo_control[servo].scanning = 0;
        servo_control[servo].scanning_enabled = 1;
        servo_control[servo].locked = 0;
        servo_control[servo].stuck = 0;
        servo_control[servo].over_current = 0;
        servo_control[servo].cache_valid = 0;
        servo_control[servo].cache_written = 0;
        servo_control[servo].min_pulse = 935;
        servo_control[servo].max_pulse = 4700;
        servo_control[servo].adc_offset = 25;
        servo_control[servo].adc_conversion = 2.64;
        servo_control[servo].max_angle = MAX_ANGLE;
        servo_control[servo].min_angle = MIN_ANGLE;
        servo_control[servo].max_current = 850;
        servo_control[servo].on_ticks = DEFAULT_PULSE_WIDTH;
        servo_control[servo].pending_on_ticks = DEFAULT_PULSE_WIDTH;
        servo_control[servo].pulse_count = 0;
        servo_control[servo].total_pulse_count = 0;
        servo_control[servo].status_reg1_shadow = 0x00;
        servo_control[servo].status_reg2_shadow = 0x00;
    }

    servo_control[0].current_sense_i2c_addr = INA219_I2C_ADDR_SERVO0;
    servo_control[1].current_sense_i2c_addr = INA219_I2C_ADDR_SERVO1;
    servo_control[2].current_sense_i2c_addr = INA219_I2C_ADDR_SERVO2;

    //Enable I2C Bus 1 in Master Mode
    OpenI2C1(MASTER, SLEW_OFF);
    SSP1ADD = ((_XTAL_FREQ / I2C_FREQ) / 4) - 1;

    //Configure I2C slave on I2C2
    i2c_slave_addr = 0x28 /*| ((PORTAbits.RA5 & 0x01) << 2) | ((PORTAbits.RA4 & 0x01) << 1) | (PORTAbits.RA3 & 0x01)*/;
    SSP2ADD = (i2c_slave_addr << 1); //Select slave 8-bit address (0x28 is 7-bit addr when A3, A4, and A5 are tied to GND) 
    SSP2CON1 = 0x36; // SSPEN: Synchronous Serial Port Enable bit - Enables the serial port and configures the SDA and SCL pins as the serial port pins
    // CKP: SCK Release Control bit              - Release clock
    // SSPM3:SSPM0: SSP Mode Select bits         - 0110 = I2C Slave mode, 7-bit address    
    SSP2STAT = 0x00;
    SSP2CON2 = 0x00; // GCEN: General Call address (00h) (Slave mode only) 0 = General call address disabled
    //SSP2CON3bits.BOEN = 1;
    //SetPriorityIntI2C2(0); //Set to low priority
    PIR3bits.SSP2IF = 0;
    IPR3bits.SSP2IP = 0;
    PIE3bits.SSP2IE = 1; //Enable interrupts from MSSP 2

    Open1USART(USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, ((_XTAL_FREQ / UART_BAUD_RATE) / 16) - 1);
    PIR1bits.RC1IF = 0; //reset RX pin interrupt flag
    IPR1bits.RC1IP = 0; //Not high priority
    PIE1bits.RC1IE = 0; //Disable RX interrupts

    //Init peripherals for servo 0
    PMD0bits.TMR1MD = 0; //Enable Timer Peripheral
    PMD1bits.CCP1MD = 0; //Enable the CCP Peripheral
    T1CONbits.TMR1ON = 0;
    CCP1CONbits.CCP1M = 0xA;
    CCPTMRS0bits.C1TSEL = 0x0; //CCP1 uses Timer 1
    PIR1bits.CCP1IF = 0;
    IPR1bits.CCP1IP = 1; //High priority
    PIE1bits.CCP1IE = 1;
    T1GCONbits.TMR1GE = 0;
    T1CONbits.TMR1CS = 0x0;
    T1CONbits.T1CKPS = 0x1; //Prescale 0x0 is 1:1, 0x1 is 1:2, 0x2 is 1:4, 0x3 is 1:8
    T1CONbits.T1RD16 = 1;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 0;

    //Init peripherals for servo 1
    PMD0bits.TMR3MD = 0; //Enable Timer Peripheral
    PMD1bits.CCP2MD = 0; //Enable the CCP Peripheral
    T3CONbits.TMR3ON = 0;
    CCP2CONbits.CCP2M = 0xA;
    CCPTMRS0bits.C2TSEL = 0x1; //CCP2 uses Timer 3
    PIR2bits.CCP2IF = 0;
    IPR2bits.CCP2IP = 1; //High priority
    PIE2bits.CCP2IE = 1;
    T3GCONbits.TMR3GE = 0;
    T3CONbits.TMR3CS = 0x0;
    T3CONbits.T3CKPS = 0x1; //Prescale 0x0 is 1:1, 0x1 is 1:2, 0x2 is 1:4, 0x3 is 1:8
    T3CONbits.T3RD16 = 1;
    PIR2bits.TMR3IF = 0;
    PIE2bits.TMR3IE = 0;

    //Init peripherals for servo 2
    PMD0bits.TMR5MD = 0; //Enable Timer Peripheral
    PMD1bits.CCP3MD = 0; //Enable the CCP Peripheral
    T5CONbits.TMR5ON = 0;
    CCP3CONbits.CCP3M = 0xA;
    CCPTMRS0bits.C3TSEL = 0x2; //CCP3 uses Timer 5
    PIR4bits.CCP3IF = 0;
    IPR4bits.CCP3IP = 1; //High priority
    PIE4bits.CCP3IE = 1;
    T5GCONbits.TMR5GE = 0;
    T5CONbits.TMR5CS = 0x0;
    T5CONbits.T5CKPS = 0x1; //Prescale 0x0 is 1:1, 0x1 is 1:2, 0x2 is 1:4, 0x3 is 1:8
    T5CONbits.T5RD16 = 1;
    PIR5bits.TMR5IF = 0;
    PIE5bits.TMR5IE = 0;

    RCONbits.IPEN = 1; //Enable priority levels on interrupts
    INTCONbits.PEIE = 1; //Enable peripheral interrupt (serial port is a peripheral)
    INTCONbits.GIE = 1; //Enable interrupts

    INA219_Init(INA219_I2C_BUS, INA219_I2C_ADDR_SERVO0);
    INA219_Init(INA219_I2C_BUS, INA219_I2C_ADDR_SERVO1);
    INA219_Init(INA219_I2C_BUS, INA219_I2C_ADDR_SERVO2);

    while (1) {
        for (servo = 0; servo < 3; ++servo) {
            ClrWdt();
            if (INA219_GetCurrent(INA219_I2C_BUS, servo_control[servo].current_sense_i2c_addr, &current)) {
                servo_control[servo].current = (unsigned int) current;
            }

            servo_control[servo].current_adc_value = ReadADC(servo, 3, 32);
            servo_angle = (servo_control[servo].current_adc_value - servo_control[servo].adc_offset);
            servo_angle = servo_angle / servo_control[servo].adc_conversion;
            servo_control[servo].current_angle = servo_angle;

            if (servo_control[servo].active) {
                if (servo_control[servo].current > servo_control[servo].max_current) {
                    servo_control[servo].over_current = 1;
                    servo_control[servo].stuck = 1;
                    DisableServo(servo);
                }
                if ((servo_control[servo].total_pulse_count > 250 && servo_control[servo].current > 150) || servo_control[servo].current > 300) {
                    servo_control[servo].stuck = 1;
                }
            }

            if (servo_control[servo].update_pending) {
                servo_control[servo].update_pending = 0;
                if (servo_control[servo].target_angle_change_pending && servo_control[servo].active) {
                    servo_control[servo].target_angle_change_pending = 0;
                    SetServoAngle(servo, servo_control[servo].target_angle_pending);
                }
                if (servo_control[servo].active_change_pending) {
                    servo_control[servo].active_change_pending = 0;
                    if (servo_control[servo].active_pending) {
                        SetServoAngle(servo, servo_control[servo].target_angle);
                    } else {
                        DisableServo(servo);
                    }
                }
            } else if (servo_control[servo].active) {
                //sprintf(buffer, "Servo %d: %0.2fmA %0.2f %d %d (Scan: %d)\r\n", servo, current, servo_control[servo].current_angle, adc_result, servo_control[servo].on_ticks, servo_control[servo].scanning);
                //SerialPrint(buffer);
                if (servo_control[servo].pulse_count >= 48) {
                    if (servo_control[servo].scanning) {
                        if (servo_control[servo].current_angle < (servo_control[servo].target_angle - 1.0)) {
                            //int angle_diff = (int) (servo_control[servo].target_angle - servo_control[servo].current_angle);
                            //angle_diff = max(1, angle_diff);
                            SetPWM(servo, servo_control[servo].pending_on_ticks + 1);
                        } else if (servo_control[servo].current_angle > (servo_control[servo].target_angle + 1.0)) {
                            //int angle_diff = (int) (servo_control[servo].current_angle - servo_control[servo].target_angle);
                            //angle_diff = max(1, angle_diff);
                            SetPWM(servo, servo_control[servo].pending_on_ticks - 1);
                        } else {
                            servo_control[servo].locked = 1;
                            servo_control[servo].scanning = 0;
                            if (SetServoSetting(servo, servo_control[servo].target_angle, servo_control[servo].pending_on_ticks)) {
                                servo_control[servo].cache_written = 1;
                            }
                        }
                    } else if (servo_control[servo].scanning_enabled) {
                        if (servo_control[servo].current_angle < (servo_control[servo].target_angle - 2.0) ||
                                servo_control[servo].current_angle > (servo_control[servo].target_angle + 2.0)) {
                            servo_control[servo].scanning = 1;
                        }
                    }
                }
            }
            servo_control[servo].previous_current_angle = servo_control[servo].current_angle;
            servo_control[servo].status_reg1_shadow = servo_control[servo].active |
                    (servo_control[servo].scanning << 1) |
                    (servo_control[servo].locked << 2) |
                    (servo_control[servo].over_current << 3) |
                    (servo_control[servo].stuck << 4) |
                    (servo_control[servo].update_pending << 5) |
                    (servo_control[servo].cache_valid << 6) |
                    (servo_control[servo].cache_written << 7);

        }
        ++loop_counter;
    }
}

unsigned int GetServoSetting(unsigned char servo, double angle) {
    unsigned int addr, data;
    unsigned int rounded_angle = (unsigned int) angle;

    if (servo > 2) servo = 0;
    addr = (364 * servo) + (rounded_angle << 1);

    if (!EEPROMReadU16(addr, &data)) {
        data = 0xFFFF;
    }

    return data;
}

int SetServoSetting(unsigned char servo, double angle, unsigned int pulse_width) {
    int rc = 0;
    unsigned int addr, eeprom_pulse_width;
    unsigned int rounded_angle = (unsigned int) angle;

    eeprom_pulse_width = GetServoSetting(servo, angle);

    if (eeprom_pulse_width != pulse_width) {
        if (servo > 2) servo = 0;
        addr = (364 * servo) + (rounded_angle << 1);


        rc = EEPROMWriteU16(addr, pulse_width);
    }

    return rc;
}

void SetServoAngle(unsigned char servo, double angle) {
    unsigned int pulse_width;

    if (angle > servo_control[servo].max_angle) angle = servo_control[servo].max_angle;
    if (angle < servo_control[servo].min_angle) angle = servo_control[servo].min_angle;
    servo_control[servo].target_angle = angle;
    servo_control[servo].cache_value = GetServoSetting(servo, angle);
    if (servo_control[servo].cache_value == 0xFFFF) {
        pulse_width = servo_control[servo].pending_on_ticks; //default to where the servo is now and start scanning from there
        servo_control[servo].cache_valid = 0;
        servo_control[servo].scanning = 1;
    } else {
        pulse_width = servo_control[servo].cache_value;
        servo_control[servo].cache_valid = 1;
        servo_control[servo].scanning = servo_control[servo].scanning_enabled;
    }
    servo_control[servo].cache_written = 0;
    servo_control[servo].locked = 0;
    servo_control[servo].over_current = 0;
    servo_control[servo].stuck = 0;
    servo_control[servo].total_pulse_count = 0;
    SetPWM(servo, pulse_width);
}

void DisableServo(unsigned char servo) {
    servo_control[servo].active = 0;
    servo_control[servo].scanning = 0;
    servo_control[servo].locked = 0;
    servo_control[servo].stuck = 0;
    servo_control[servo].over_current = 0;
    servo_control[servo].update_pending = 0;
    servo_control[servo].cache_valid = 0;
    servo_control[servo].cache_written = 0;
    servo_control[servo].pin_output = 0;
    servo_control[servo].pulse_count = 0;
    if (servo == 0) {
        T1CONbits.TMR1ON = 0;
        TRISCbits.TRISC2 = INPUT_PIN;
    } else if (servo == 1) {
        T3CONbits.TMR3ON = 0;
        TRISCbits.TRISC1 = INPUT_PIN;
    } else if (servo == 2) {
        T5CONbits.TMR5ON = 0;
        TRISCbits.TRISC0 = INPUT_PIN;
    }
}

void SetPWM(unsigned char servo, unsigned int pulse_width) {
    servo_control[servo].pending_on_ticks = min(servo_control[servo].max_pulse, max(pulse_width, servo_control[servo].min_pulse));
    servo_control[servo].pending_off_ticks = (40000 - servo_control[servo].pending_on_ticks);
    servo_control[servo].pulse_count = 0;

    if (!servo_control[servo].active) {
        servo_control[servo].active = 1;
        servo_control[servo].on_ticks = servo_control[servo].pending_on_ticks;
        servo_control[servo].off_ticks = servo_control[servo].pending_off_ticks;
        if (servo == 0) {
            if (T1CONbits.TMR1ON == 0) {
                CCPR1L = (servo_control[servo].on_ticks & 0xFF); // 1 = 0.0005
                CCPR1H = ((servo_control[servo].on_ticks >> 8) & 0xFF);

                TMR1L = 0x00;
                TMR1H = 0x00;

                servo_control[servo].pin_output = 1;
                LATCbits.LATC2 = 1;
                TRISCbits.TRISC2 = OUTPUT_PIN;
                T1CONbits.TMR1ON = 1;
            }
        } else if (servo == 1) {
            if (T3CONbits.TMR3ON == 0) {
                CCPR2L = (servo_control[servo].on_ticks & 0xFF); // 1 = 0.0005
                CCPR2H = ((servo_control[servo].on_ticks >> 8) & 0xFF);

                TMR3L = 0x00;
                TMR3H = 0x00;

                servo_control[servo].pin_output = 1;
                LATCbits.LATC1 = 1;
                TRISCbits.TRISC1 = OUTPUT_PIN;
                T3CONbits.TMR3ON = 1;
            }
        } else if (servo == 2) {
            if (T5CONbits.TMR5ON == 0) {
                CCPR3L = (servo_control[servo].on_ticks & 0xFF); // 1 = 0.0005
                CCPR3H = ((servo_control[servo].on_ticks >> 8) & 0xFF);

                TMR5L = 0x00;
                TMR5H = 0x00;

                servo_control[servo].pin_output = 1;
                LATCbits.LATC0 = 1;
                TRISCbits.TRISC0 = OUTPUT_PIN;
                T5CONbits.TMR5ON = 1;
            }
        }
    }
}

void interrupt InterruptHandler() {

    if (PIE1bits.CCP1IE && PIR1bits.CCP1IF) {
        if (servo_control[0].active) {
            T1CONbits.TMR1ON = 0;
            TMR1L = 0x00;
            TMR1H = 0x00;
            if (servo_control[0].pin_output) {
                CCPR1L = (servo_control[0].off_ticks & 0xFF); // 1 = 0.0005
                CCPR1H = ((servo_control[0].off_ticks >> 8) & 0xFF);
                servo_control[0].pin_output = 0;
                LATCbits.LATC2 = 0;
            } else {
                servo_control[0].on_ticks = servo_control[0].pending_on_ticks;
                servo_control[0].off_ticks = servo_control[0].pending_off_ticks;
                CCPR1L = (servo_control[0].on_ticks & 0xFF); // 1 = 0.0005
                CCPR1H = ((servo_control[0].on_ticks >> 8) & 0xFF);
                servo_control[0].pin_output = 1;
                if (servo_control[0].pulse_count < 0xFFFF) ++servo_control[0].pulse_count; //A complete control pulse has been sent
                if (servo_control[0].total_pulse_count < 0xFFFF) ++servo_control[0].total_pulse_count;
                LATCbits.LATC2 = 1;
            }
            T1CONbits.TMR1ON = 1;
        }
        PIR1bits.CCP1IF = 0;
    }
    if (PIE2bits.CCP2IE && PIR2bits.CCP2IF) {
        if (servo_control[1].active) {
            T3CONbits.TMR3ON = 0;
            TMR3L = 0x00;
            TMR3H = 0x00;
            if (servo_control[1].pin_output) {
                CCPR2L = (servo_control[1].off_ticks & 0xFF); // 1 = 0.0005
                CCPR2H = ((servo_control[1].off_ticks >> 8) & 0xFF);
                servo_control[1].pin_output = 0;
                LATCbits.LATC1 = 0;
            } else {
                servo_control[1].on_ticks = servo_control[1].pending_on_ticks;
                servo_control[1].off_ticks = servo_control[1].pending_off_ticks;
                CCPR2L = (servo_control[1].on_ticks & 0xFF); // 1 = 0.0005
                CCPR2H = ((servo_control[1].on_ticks >> 8) & 0xFF);
                servo_control[1].pin_output = 1;
                if (servo_control[1].pulse_count < 0xFFFF) ++servo_control[1].pulse_count; //A complete control pulse has been sent
                if (servo_control[1].total_pulse_count < 0xFFFF) ++servo_control[1].total_pulse_count;
                LATCbits.LATC1 = 1;
            }
            T3CONbits.TMR3ON = 1;
        }
        PIR2bits.CCP2IF = 0;
    }
    if (PIE4bits.CCP3IE && PIR4bits.CCP3IF) {
        if (servo_control[2].active) {
            T5CONbits.TMR5ON = 0;
            TMR5L = 0x00;
            TMR5H = 0x00;
            if (servo_control[2].pin_output) {
                CCPR3L = (servo_control[2].off_ticks & 0xFF);
                CCPR3H = ((servo_control[2].off_ticks >> 8) & 0xFF);
                servo_control[2].pin_output = 0;
                LATCbits.LATC0 = 0;
            } else {
                servo_control[2].on_ticks = servo_control[2].pending_on_ticks;
                servo_control[2].off_ticks = servo_control[2].pending_off_ticks;
                CCPR3L = (servo_control[2].on_ticks & 0xFF);
                CCPR3H = ((servo_control[2].on_ticks >> 8) & 0xFF);
                servo_control[2].pin_output = 1;
                if (servo_control[2].pulse_count < 0xFFFF) ++servo_control[2].pulse_count; //A complete control pulse has been sent
                if (servo_control[2].total_pulse_count < 0xFFFF) ++servo_control[2].total_pulse_count;
                LATCbits.LATC0 = 1;
            }
            T5CONbits.TMR5ON = 1;
        }
        PIR4bits.CCP3IF = 0;
    }
}

void interrupt low_priority LowInterruptHandler() {
    unsigned char data;

    if (PIE3bits.SSP2IE && PIR3bits.SSP2IF) {
        if (SSP2STATbits.D_NOT_A) {
            // Data bytes 
            i2c_byte_count++;

            if (SSP2STATbits.BF) {
                data = SSP2BUF; // Clear BF
            }

            if (SSP2STATbits.R_NOT_W) {
                // Multi-byte read - advance to next address
                SSP2CON1bits.WCOL = 0;
                SSP2BUF = GetControlRegister(i2c_request_addr + i2c_byte_count);
            } else {

                if (i2c_byte_count == 1) {
                    // First write byte is register address
                    i2c_request_addr = data;
                } else {
                    SetControlRegister(i2c_request_addr + (i2c_byte_count - 2), data);
                }
            }

        } else {
            //
            // Slave Address 
            //
            i2c_byte_count = 0;

            if (SSP2STATbits.BF) {
                // Discard slave address 
                data = SSP2BUF; // Clear BF
            }

            if (SSP2STATbits.R_NOT_W) {
                // Reading - read from register map
                SSP2CON1bits.WCOL = 0;
                SSP2BUF = GetControlRegister(i2c_request_addr);
            }
        }

        // Finally
        PIR3bits.SSP2IF = 0; // Clear MSSP interrupt flag 
        SSP2CON1bits.CKP = 1; // Release clock
    }
}

unsigned char GetControlRegister(unsigned char addr) {
    volatile unsigned char servo = (addr >> 6);
    volatile unsigned char reg_addr = (addr & 0x3F);
    volatile unsigned char data = 0xE8;

    if (reg_addr == 0x00) {//addr reserved for any global control, like a software reset...
        data = 0xA5;
    } else if (reg_addr == 0x01) {//status reg
        data = servo_control[servo].status_reg1_shadow; //shadow reg used for performance to reduce the computation done inside the interrupt handler
    } else if (reg_addr == 0x02) {//status reg
        data = servo_control[servo].status_reg2_shadow; //shadow reg used for performance to reduce the computation done inside the interrupt handler
    } else if (reg_addr == 0x03) {//control reg
        data = 0x00;
        data = data | servo_control[servo].active;
        data = data | (servo_control[servo].scanning_enabled << 1);
    } else if (reg_addr == 0x04) {
        data = (unsigned char) servo_control[servo].current_angle;
    } else if (reg_addr == 0x05) {
        data = (unsigned char) servo_control[servo].target_angle;
    } else if (reg_addr == 0x06) {
        data = ((servo_control[servo].current >> 8) & 0xFF);
    } else if (reg_addr == 0x07) {
        data = (servo_control[servo].current & 0xFF);
    } else if (reg_addr == 0x08) {
        data = ((servo_control[servo].current_adc_value >> 8) & 0xFF);
    } else if (reg_addr == 0x09) {
        data = (servo_control[servo].current_adc_value & 0xFF);
    } else if (reg_addr == 0x0A) {
        data = ((servo_control[servo].on_ticks >> 8) & 0xFF);
    } else if (reg_addr == 0x0B) {
        data = (servo_control[servo].on_ticks & 0xFF);
    } else if (reg_addr == 0x0C) {
        data = ((servo_control[servo].off_ticks >> 8) & 0xFF);
    } else if (reg_addr == 0x0D) {
        data = (servo_control[servo].off_ticks & 0xFF);
    } else if (reg_addr == 0x0E) {
        data = ((servo_control[servo].pulse_count >> 8) & 0xFF);
    } else if (reg_addr == 0x0F) {
        data = (servo_control[servo].pulse_count & 0xFF);
    } else if (reg_addr == 0x10) {
        data = ((servo_control[servo].total_pulse_count >> 8) & 0xFF);
    } else if (reg_addr == 0x11) {
        data = (servo_control[servo].total_pulse_count & 0xFF);
    } else if (reg_addr == 0x12) {
        data = (unsigned char) servo_control[servo].min_angle;
    } else if (reg_addr == 0x13) {
        data = (unsigned char) servo_control[servo].max_angle;
    } else if (reg_addr == 0x14) {
        data = ((servo_control[servo].min_pulse >> 8) & 0xFF);
    } else if (reg_addr == 0x15) {
        data = (servo_control[servo].min_pulse & 0xFF);
    } else if (reg_addr == 0x16) {
        data = ((servo_control[servo].max_pulse >> 8) & 0xFF);
    } else if (reg_addr == 0x17) {
        data = (servo_control[servo].max_pulse & 0xFF);
    } else if (reg_addr == 0x18) {
        data = ((servo_control[servo].max_current >> 8) & 0xFF);
    } else if (reg_addr == 0x19) {
        data = (servo_control[servo].max_current & 0xFF);
    } else if (reg_addr == 0x1A) {
        data = ((servo_control[servo].cache_value >> 8) & 0xFF);
    } else if (reg_addr == 0x1B) {
        data = (servo_control[servo].cache_value & 0xFF);
    }
    return data;
}

void SetControlRegister(unsigned char addr, unsigned char data) {
    volatile unsigned char servo = (addr >> 6);
    volatile unsigned char reg_addr = (addr & 0x3F);

    if (reg_addr == 0x03) {//control reg (bit 0: active, bit 1: scanning enabled)
        servo_control[servo].scanning_enabled = ((data >> 1) & 0x01);
        servo_control[servo].update_pending = 1;
        servo_control[servo].active_pending = (data & 0x01);
        servo_control[servo].active_change_pending = 1;
    } else if (reg_addr == 0x05) {//target angle
        if (servo_control[servo].active) {
            servo_control[servo].update_pending = 1;
            servo_control[servo].target_angle_change_pending = 1;
            servo_control[servo].target_angle_pending = data;
        } else {
            servo_control[servo].target_angle = data;
        }
    } else if (reg_addr == 0x12) {//min angle
        servo_control[servo].min_angle = data;
    } else if (reg_addr == 0x13) {//max angle
        servo_control[servo].max_angle = data;
    } else if (reg_addr == 0x14) {//min pulse high byte
        servo_control[servo].min_pulse = ((servo_control[servo].min_pulse & 0xFF00) | data);
    } else if (reg_addr == 0x15) {//min pulse low byte
        servo_control[servo].min_pulse = ((servo_control[servo].min_pulse & 0x00FF) | (((unsigned int) data) << 8));
    } else if (reg_addr == 0x16) {//max pulse high byte
        servo_control[servo].max_pulse = ((servo_control[servo].max_pulse & 0xFF00) | data);
    } else if (reg_addr == 0x17) {//max pulse low byte
        servo_control[servo].max_pulse = ((servo_control[servo].max_pulse & 0x00FF) | (((unsigned int) data) << 8));
    } else if (reg_addr == 0x18) {//max current high byte
        servo_control[servo].max_current = ((servo_control[servo].max_current & 0xFF00) | data);
    } else if (reg_addr == 0x19) {//max current low byte
        servo_control[servo].max_current = ((servo_control[servo].max_current & 0x00FF) | (((unsigned int) data) << 8));
    }
}

unsigned int ReadADC(unsigned char pin, unsigned int tad, unsigned int num_samples) {
    unsigned char data_hi, data_lo;
    unsigned int index, result;
    unsigned long long int result_sum = 0;

    ADCON2bits.ADFM = 1; //Right justify result
    ADCON2bits.ACQT = tad; //TAD (increase for better results and longer conversions)
    ADCON2bits.ADCS = 0x7; //Clock comes from FRC (600kHz))
    ADCON1bits.TRIGSEL = 0; //Don't Care
    ADCON1bits.PVCFG = 0; //Positive reference is VDD
    ADCON1bits.NVCFG = 0; //Positive reference is VSS
    ADCON0bits.ADON = 1;
    ADCON0bits.CHS = pin;

    for (index = 0; index < num_samples; ++index) {
        ADCON0bits.GO = 1;
        while (ADCON0bits.GO) {
            Nop();
        }
        data_hi = (ADRESH & 0x3);
        data_lo = ADRESL;

        result = data_hi;
        result = (result << 8) | (unsigned int) data_lo;
        result_sum += result;
    }

    return result_sum / num_samples;

}

void SerialPrint(char* buffer) {
    unsigned int index, length;

    length = strlen(buffer);
    for (index = 0; index < length; ++index) {
        while (Busy1USART()) {
            Nop();
        }
        Write1USART(buffer[index]);
    }
}

void delay_in_us(unsigned long us) {
    unsigned long index, loops = us / 100;

    if (loops == 0) loops = 1;
    for (index = 0; index < loops; ++index) {
        __delay_us(100);
    }
}

void delay_in_ms(unsigned long ms) {
    unsigned long index, loops = ms / 10;

    if (loops == 0) loops = 1;
    for (index = 0; index < loops; ++index) {
        __delay_ms(10);
    }
}

void Delay1Second(void) {
    int i;

    for (i = 0; i < 100; i++) {
        __delay_ms(10);
        ClrWdt();
    }
}