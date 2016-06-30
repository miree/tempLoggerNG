#ifndef ADS1248_H__
#define ADS1248_H__

#include <avr/io.h>

#define ADS_DDR_START DDRB
#define ADS_DDR_CS    DDRB
#define ADS_DDR_DRDY  DDRB
#define ADS_DDR_DIN   DDRB
#define ADS_DDR_DOUT  DDRB
#define ADS_DDR_SCLK  DDRB
#define ADS_DDR_RESET DDRD
#define ADS_DDR_MOS01 DDRD
#define ADS_DDR_MOS23 DDRD
#define ADS_DDR_MOS45 DDRD
#define ADS_DDR_MOS67 DDRD

#define ADS_PORT_START PORTB
#define ADS_PORT_CS    PORTB
#define ADS_PIN_DRDY   PINB
#define ADS_PORT_DIN   PORTB
#define ADS_PIN_DOUT   PINB
#define ADS_PORT_SCLK  PORTB
#define ADS_PORT_RESET PORTD
#define ADS_PORT_MOS01 PORTD
#define ADS_PORT_MOS23 PORTD
#define ADS_PORT_MOS45 PORTD
#define ADS_PORT_MOS67 PORTD

#define ADS_P_START PB0
#define ADS_P_CS    PB1
#define ADS_P_DRDY  PB2
#define ADS_P_DIN   PB3
#define ADS_P_DOUT  PB4
#define ADS_P_SCLK  PB5
#define ADS_P_RESET PD0
#define ADS_P_MOS01 PD4
#define ADS_P_MOS23 PD5
#define ADS_P_MOS45 PD6
#define ADS_P_MOS67 PD7


extern uint8_t ADS_ready;
extern uint8_t ADS_data_ready;


// initialize the AVR IO registers to control the ADS1248 device
void ADS_init();
void ADS_start();
void ADS_stop();
void ADS_start_pulse();


////////////////////////////////////////////////////////////////////////
// ADS SPI commands
////////////////////////////////////////////////////////////////////////

// all known ADS1248 SPI commands
enum Command
{
	ADS_COMMAND_WAKEUP   = 0x00,  // Exit sleep mode                  0000 000x (00h, 01h) 
	ADS_COMMAND_SLEEP    = 0x02,  // Enter sleep mode                 0000 001x (02h, 03h) 
	ADS_COMMAND_SYNC     = 0x04,  // Synchronize the A/D conversion   0000 010x (04h, 05h) 
	ADS_COMMAND_RESET    = 0x06,  // Reset to power-up values         0000 011x (06h, 07h) 
	ADS_COMMAND_NOP      = 0xff,  // No operation                     1111 1111 (FFh) 
	ADS_COMMAND_RDATA    = 0x12,  // Read data once                   0001 001x (12h, 13h) 
	ADS_COMMAND_RDATAC   = 0x14,  // Read data continuously           0001 010x (14h, 15h) 
	ADS_COMMAND_SDATAC   = 0x16,  // Stop reading data continuously   0001 011x (16h, 17h) 
	ADS_COMMAND_RREG     = 0x20,  // Read from register rrrr          0010 rrrr (2xh)         0000_nnnn (2nd byte)
	ADS_COMMAND_WREG     = 0x40,  // Write to register rrrr           0100 rrrr (4xh)         0000_nnnn (2nd byte)
	ADS_COMMAND_SYSOCAL  = 0x60,  // System offset calibration        0110 0000 (60h) 
	ADS_COMMAND_SYSGCAL  = 0x61,  // System gain calibration          0110 0001 (61h) 
	ADS_COMMAND_SELFOCAL = 0x62,  // Self offset calibration          0110 0010 (62h) 
	//Restricted command. Should never be sent to device. 1111 0001 (F1h)
};

void ADS_SPI_start();
uint8_t ADS_SPI_transmit(uint8_t cData);
void ADS_SPI_stop();
////////////////////////////////////////////////////////////////////////
// ADS data access 
////////////////////////////////////////////////////////////////////////

void ADS_read_data(uint8_t *result);
uint32_t ADS_resistance(uint8_t *ADC_values); // parameter is uint8_t[3] 
////////////////////////////////////////////////////////////////////////
// ADS register access (read/write)
////////////////////////////////////////////////////////////////////////

enum Reg
{
	ADS_REG_MUX0    = 0x00, // BCS1    BCS0     MUX_SP2  MUX_SP1  MUX_SP0   MUX_SN2  MUX_SN1 MUX_SN0
	ADS_REG_VBIAS   = 0x01, // VBIAS7  VBIAS6   VBIAS5   VBIAS4   VBIAS3    VBIAS2   VBIAS1  VBIAS0
	ADS_REG_MUX1    = 0x02, // CLKSTAT VREFCON1 VREFCON0 REFSELT1 REFSELT0  MUXCAL2  MUXCAL1 MUXCAL0
	ADS_REG_SYS0    = 0x03, // 0       PGA2     PGA1     PGA0     DR3       DR2      DR1     DR0
	ADS_REG_OFC0    = 0x04, // OFC7    OFC6     OFC5     OFC4     OFC3      OFC2     OFC1    OFC0
	ADS_REG_OFC1    = 0x05, // OFC15   OFC14    OFC13    OFC12    OFC11     OFC10    OFC9    OFC8
	ADS_REG_OFC2    = 0x06, // OFC23   OFC22    OFC21    OFC20    OFC19     OFC18    OFC17   OFC16
	ADS_REG_FSC0    = 0x07, // FSC7    FSC6     FSC5     FSC4     FSC3      FSC2     FSC1    FSC0
	ADS_REG_FSC1    = 0x08, // FSC15   FSC14    FSC13    FSC12    FSC11     FSC10    FSC9    FSC8
	ADS_REG_FSC2    = 0x09, // FSC23   FSC22    FSC21    FSC20    FSC19     FSC18    FSC17   FSC16
	ADS_REG_IDAC0   = 0x0A, // ID3     ID2      ID1      ID0      DRDY/MODE IMAG2    IMAG1   IMAG0
	ADS_REG_IDAC1   = 0x0B, // I1DIR3  I1DIR2   I1DIR1   I1DIR0   I2DIR3    I2DIR2   I2DIR1  I2DIR0
	ADS_REG_GPIOCFG = 0x0C, // IOCFG7  IOCFG6   IOCFG5   IOCFG4   IOCFG3    IOCFG2   IOCFG1  IOCFG0
	ADS_REG_GPIODIR = 0x0D, // IODIR7  IODIR6   IODIR5   IODIR4   IODIR3    IODIR2   IODIR1  IODIR0
	ADS_REG_GPIODAT = 0x0E, // IODAT7  IODAT6   IODAT5   IODAT4   IODAT3    IODAT2   IODAT1  IODAT0
};

void ADS_reg_write(uint8_t reg, uint8_t content);
uint8_t ADS_reg_read(uint8_t reg);




////////////////////////////////////////////////////////////////////////
// ADS high-level setup
////////////////////////////////////////////////////////////////////////

void ADS_setup_enable_voltage_ref();
void ADS_setup_enable_diode_temp();

void ADS_setup_select_ADC01();
void ADS_setup_select_ADC23();
void ADS_setup_select_ADC45();
void ADS_setup_select_ADC67();

void ADS_no_MOSFET();
void ADS_select_MOSFET01();
void ADS_select_MOSFET23();
void ADS_select_MOSFET45();
void ADS_select_MOSFET67();



// PGA = 0...7  ->  factor =   1...128
// DR  = 0...16 ->  rate   = 5Hz...2000Hz
void ADS_PGA_and_rate_setup(uint8_t PGA, uint8_t DR);

// IMAG = 0...7 ->  current = 0...1500uA
void ADS_IEXC_setup(uint8_t IMAG);


#endif
