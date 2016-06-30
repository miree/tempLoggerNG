#include "ADS1248.h"
#include <avr/interrupt.h>

uint8_t ADS_ready = 0;
uint8_t ADS_data_ready = 0;

// these variables are manipulated by the functions 
// ADS_PGA_and_rate_setup and ADS_IEXC_setup.
// they should not be written by the user
uint8_t IMAG_table[8] = {0,5,10,25,50,75,100,150}; // in units of 10 uA
uint16_t THREE_OVER_IMAG_table[8] = {0,60,30,12,6,4,3,2}; // (3 / IMAG) in units of 1/mA
uint8_t ADS_IMAG_setting = 0;
uint8_t ADS_DR_setting = 0;
uint8_t ADS_PGA_setting = 0;


void ADS_init()
{
	// set the AVR-DDR registers to correct input/output 
	ADS_DDR_START  |=  (1<<ADS_P_START);   // ADS_START pin is AVR-output
	ADS_DDR_CS     |=  (1<<ADS_P_CS);      // ADS_CS    pin is AVR-output
	ADS_DDR_DRDY   &= ~(1<<ADS_P_DRDY);    // ADS_DRDY  pin is AVR-input
	ADS_DDR_DIN    |=  (1<<ADS_P_DIN);     // ADS_DIN   pin is AVR-output
	ADS_DDR_DOUT   &= ~(1<<ADS_P_DOUT);    // ADS_DOUT  pin is AVR-input
	ADS_DDR_SCLK   |=  (1<<ADS_P_SCLK);    // ADS_SCLK  pin is AVR-output
	ADS_DDR_RESET  |=  (1<<ADS_P_RESET);   // ADS_RESET pin is AVR-output
	
	ADS_PORT_START &= ~(1<<ADS_P_START);   // pull ADS_START pin low  
	ADS_PORT_RESET &= ~(1<<ADS_P_RESET);   // pull ADS_RESET pin low  (active reset of ADS chip)
	ADS_PORT_CS    |=  (1<<ADS_P_CS);      // pull ADS_CS pin high     (disable serial SPI communication)
	ADS_PORT_SCLK  &= ~(1<<ADS_P_SCLK);    // pull ADS_SCLK pin low   (default state for clock pin)
	ADS_PORT_RESET |=  (1<<ADS_P_RESET);   // pull ADS_RESET pin high (release the reset condition)
	
	// set the AVR-DDR registers for the current selection MOSFETs 
	ADS_DDR_MOS01 |=  (1<<ADS_P_MOS01);
	ADS_DDR_MOS23 |=  (1<<ADS_P_MOS23);
	ADS_DDR_MOS45 |=  (1<<ADS_P_MOS45);
	ADS_DDR_MOS67 |=  (1<<ADS_P_MOS67);
	
	ADS_select_MOSFET01();
}

void ADS_start()
{
	ADS_PORT_START |=  (1<<ADS_P_START);   // pull ADS_START pin high
}

void ADS_stop()
{
	ADS_PORT_START &= ~(1<<ADS_P_START);   // pull ADS_START pin low
}
void ADS_start_pulse()
{
	ADS_start();
	ADS_stop();
	
	// enable the PCINT2 interrupt
	PCICR  |= 0x01;
	PCMSK0 |= 0x04;
}


void ADS_reg_write(uint8_t reg, uint8_t content)
{
	ADS_SPI_start();
	ADS_SPI_transmit(ADS_COMMAND_WREG | reg); // write register content; start from adress 0x0
	ADS_SPI_transmit(0x00);                   // number of regs to write minus 1 (write 1 single register)
	ADS_SPI_transmit(content);
	ADS_SPI_stop();	
}

uint8_t ADS_reg_read(uint8_t reg)
{
	uint8_t result;
	ADS_SPI_start();
	ADS_SPI_transmit(ADS_COMMAND_RREG | reg); // read register content; start from adress 0x0
	ADS_SPI_transmit(0x00); // number of registers to read minus 1 (read one single register)
	result = ADS_SPI_transmit(ADS_COMMAND_NOP);
	ADS_SPI_stop();	
	return result;
}

 
void ADS_SPI_start()
{
	ADS_start();
	// pull SCLK pin up and down. For some reason the 
	//   hardware SPI interface only after doing this.
	ADS_PORT_SCLK  |=  (1<<ADS_P_SCLK);    // pull ADS_SCLK pin high (default state for clock pin)
	ADS_PORT_SCLK  &= ~(1<<ADS_P_SCLK);    // pull ADS_SCLK pin low  (default state for clock pin)

	// activate the SPI interface of the ADS device
	ADS_PORT_CS    &= ~(1<<ADS_P_CS);      // pull ADS_CS pin low (activate SPI interface)
}

uint8_t ADS_SPI_transmit(uint8_t cData)
{
	// disable all interrupts
	//cli();
	
	// set ~SS pin of AVR as output (to allow master SPI mode)
	DDRB |=  (1<<3);
	
	// configure the active SPI pins on the AVR as outputs
	ADS_DDR_DIN  |= (1<<ADS_P_DIN);
	ADS_DDR_SCLK |= (1<<ADS_P_SCLK);
	
	// AVR SPI setup
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPHA);//|(1<<SPR1);
	// start transmission 
	SPDR = cData;
	// wait for end of transmission
	while(!(SPSR & (1<<SPIF)));
	SPCR = 0;
	
	// enable all interrupts
	//sei();
	
	// set ~SS pin of AVR to input (it is also the DRDY pin of the ADS device)
	//  this pin indicates that data is ready to be read from ADS device
	DDRB &= ~(1<<3);
	return SPDR;
}

void ADS_SPI_stop()
{
	ADS_PORT_CS    |=  (1<<ADS_P_CS);      // pull ADS_CS pin low (activate SPI interface)
	//ADS_PORT_SCLK  &= ~(1<<ADS_P_SCLK);    // pull ADS_SCLK pin low (default state for clock pin)
	ADS_stop();
}


void ADS_read_data(uint8_t *result)
{
	ADS_SPI_start();
	result[0] = ADS_SPI_transmit(ADS_COMMAND_NOP);
	result[1] = ADS_SPI_transmit(ADS_COMMAND_NOP);
	result[2] = ADS_SPI_transmit(ADS_COMMAND_NOP);
	ADS_data_ready = 0;
	ADS_SPI_stop();	
}


void ADS_setup_enable_voltage_ref()
{
	// register MUX1 setup
	// CLKSTAT VREFCON1 VREFCON0 REFSELT1 REFSELT0  MUXCAL2  MUXCAL1 MUXCAL0
	//    0                                                                       // read-only
	//            0        1                                                      // internal reference always on
	//                              1         0                                   // Onboard reference selected
	//                                                 0        0       0         // normal operation
	ADS_reg_write(ADS_REG_MUX1, 0x30);
}

void ADS_setup_enable_diode_temp()
{
	// register MUX1 setup
	// CLKSTAT VREFCON1 VREFCON0 REFSELT1 REFSELT0  MUXCAL2  MUXCAL1 MUXCAL0
	//    0                                                                       // read-only
	//            0        1                                                      // internal reference always on
	//                              1         0                                   // Onboard reference selected
	//                                                 0        1       1         // internal diode temperature sensor
	ADS_reg_write(ADS_REG_MUX1, 0x33);
}
void ADS_setup_enable_offset_measurement()
{
	// register MUX1 setup
	// CLKSTAT VREFCON1 VREFCON0 REFSELT1 REFSELT0  MUXCAL2  MUXCAL1 MUXCAL0
	//    0                                                                       // read-only
	//            0        1                                                      // internal reference always on
	//                              1         0                                   // Onboard reference selected
	//                                                 0        0       1         // offset measurment (shorted inputs)
	ADS_reg_write(ADS_REG_MUX1, 0x31);
}
void ADS_setup_enable_gain_measurement()
{
	// register MUX1 setup
	// CLKSTAT VREFCON1 VREFCON0 REFSELT1 REFSELT0  MUXCAL2  MUXCAL1 MUXCAL0
	//    0                                                                       // read-only
	//            0        1                                                      // internal reference always on
	//                              1         0                                   // Onboard reference selected
	//                                                 0        1       0         // gain measurment (full-scale input)
	ADS_reg_write(ADS_REG_MUX1, 0x32);
}

void ADS_setup_select_ADC01()
{
	// register MUX0 setup
	// BCS1    BCS0     MUX_SP2  MUX_SP1  MUX_SP0   MUX_SN2  MUX_SN1 MUX_SN0
	//   0       0                                                                 // burnout current source off
	//                     0        0        0                                     // positive input: AIN0
	//                                                  0        0      1          // negative input: AIN1
	ADS_reg_write(ADS_REG_MUX0, 0x01);
}
void ADS_setup_select_ADC23()
{
	// register MUX0 setup
	// BCS1    BCS0     MUX_SP2  MUX_SP1  MUX_SP0   MUX_SN2  MUX_SN1 MUX_SN0
	//   0       0                                                                 // burnout current source off
	//                     0        1        0                                     // positive input: AIN2
	//                                                  0        1      1          // negative input: AIN3
	ADS_reg_write(ADS_REG_MUX0, 0x13);
}
void ADS_setup_select_ADC45()
{
	// register MUX0 setup
	// BCS1    BCS0     MUX_SP2  MUX_SP1  MUX_SP0   MUX_SN2  MUX_SN1 MUX_SN0
	//   0       0                                                                 // burnout current source off
	//                     1        0        0                                     // positive input: AIN4
	//                                                  1        0      1          // negative input: AIN5
	ADS_reg_write(ADS_REG_MUX0, 0x25);
}
void ADS_setup_select_ADC67()
{
	// register MUX0 setup
	// BCS1    BCS0     MUX_SP2  MUX_SP1  MUX_SP0   MUX_SN2  MUX_SN1 MUX_SN0
	//   0       0                                                                 // burnout current source off
	//                     1        1        0                                     // positive input: AIN6
	//                                                  1        1      1          // negative input: AIN7
	ADS_reg_write(ADS_REG_MUX0, 0x37);
}

void ADS_no_MOSFET()
{
	ADS_PORT_MOS01 &=  ~(1<<ADS_P_MOS01);
	ADS_PORT_MOS23 &=  ~(1<<ADS_P_MOS23);
	ADS_PORT_MOS45 &=  ~(1<<ADS_P_MOS45);
	ADS_PORT_MOS67 &=  ~(1<<ADS_P_MOS67);	
}
void ADS_select_MOSFET01()
{
	// open MOSFET for channel 01
	ADS_PORT_MOS01 |=  (1<<ADS_P_MOS01);
	// close all other MOSFETs
	ADS_PORT_MOS23 &=  ~(1<<ADS_P_MOS23);
	ADS_PORT_MOS45 &=  ~(1<<ADS_P_MOS45);
	ADS_PORT_MOS67 &=  ~(1<<ADS_P_MOS67);	
}
void ADS_select_MOSFET23()
{
	// open MOSFET for channel 23
	ADS_PORT_MOS23 |=  (1<<ADS_P_MOS23);
	// close all other MOSFETs
	ADS_PORT_MOS01 &=  ~(1<<ADS_P_MOS01);
	ADS_PORT_MOS45 &=  ~(1<<ADS_P_MOS45);
	ADS_PORT_MOS67 &=  ~(1<<ADS_P_MOS67);	
}
void ADS_select_MOSFET45()
{
	// open MOSFET for channel 45
	ADS_PORT_MOS45 |=  (1<<ADS_P_MOS45);
	// close all other MOSFETs
	ADS_PORT_MOS01 &=  ~(1<<ADS_P_MOS01);
	ADS_PORT_MOS23 &=  ~(1<<ADS_P_MOS23);
	ADS_PORT_MOS67 &=  ~(1<<ADS_P_MOS67);	
}
void ADS_select_MOSFET67()
{
	// open MOSFET for channel 67
	ADS_PORT_MOS67 |=  (1<<ADS_P_MOS67);
	// close all other MOSFETs
	ADS_PORT_MOS01 &=  ~(1<<ADS_P_MOS01);	
	ADS_PORT_MOS23 &=  ~(1<<ADS_P_MOS23);
	ADS_PORT_MOS45 &=  ~(1<<ADS_P_MOS45);
}

void ADS_PGA_and_rate_setup(uint8_t PGA, uint8_t DR)
{
	ADS_PGA_setting = PGA;
	ADS_DR_setting = DR;
	// SYS0 =    0       PGA2     PGA1     PGA0     DR3       DR2      DR1     DR0
	ADS_reg_write(ADS_REG_SYS0, ((PGA & 0x07) << 4) | (DR & 0x0f));
}

void ADS_IEXC_setup(uint8_t IMAG)
{
	ADS_IMAG_setting = IMAG;
	// IDAC0   = ID3     ID2      ID1      ID0      DRDY/MODE IMAG2    IMAG1   IMAG0
	//            0       0        0        0           0       x        x       x 
	ADS_reg_write(ADS_REG_IDAC0, (IMAG & 0x07));
	
	// IDAC1   = I1DIR3  I1DIR2   I1DIR1   I1DIR0   I2DIR3    I2DIR2   I2DIR1  I2DIR0
	//             1       0        0        0         1        0         0       1
	ADS_reg_write(ADS_REG_IDAC1, 0x89);
}

uint32_t ADS_resistance(uint8_t *ADC_values)
{
	// IMPLEMENT THIS!!!
	// use global vairables ADS_IMAG_setting, ADS_PGA_setting and ADS_DR_setting to convert ADC value to resistcane.
	//	double PGA = 8;
	//	double U = 2.048*adc/(0x7fffff)/PGA;
	//	double I = 0.000250;  // 250uA
	//	double R = U/I;	
	//	return R;
	uint32_t ADC_value  = ADC_values[0]; ADC_value <<= 8;
			 ADC_value |= ADC_values[1]; ADC_value <<= 8;
			 ADC_value |= ADC_values[2];
	uint64_t R = ADC_value;
	R *= THREE_OVER_IMAG_table[ADS_IMAG_setting];
	R *= 16; // don't know why this has to be here... without this, the resistance is factor 16 too small :-(
	R /= (1<<ADS_PGA_setting);
	R /= 3;
	//R /= 4096;
	return R;
}
