#define F_CPU 12000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "usbdrv.h"
#include "oddebug.h"
#include <util/delay.h>

#include "ADS1248.h"

uint8_t adc_buffer[5][3]; // the data buffer for the 4 channels with 3 bytes each. 5th channel is for on-board diode
uint8_t R_buffer[5][3];

// buffer for calibration coefficents: delta_R = a - b*V_diode
uint8_t calib_buffer_a[4]= {0xff, 0xff, 0xff, 0xff}; // value is ( buffer[0]<<24 | buffer[1]<<16 | buffer[2]<<8 | buffer[3] ) / 2^32
uint8_t calib_buffer_b[4]= {0xff, 0xff, 0xff, 0xff}; // value is ( buffer[0]<<24 | buffer[1]<<16 | buffer[2]<<8 | buffer[3] ) / 2^32

uint8_t do_measurement = 0;

#define CAL_START_ADDR 0x100


void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	// Wait for completion of previous write 
	while(EECR & (1<<EEPE));
	// Set up address and Data Registers 
	EEAR = uiAddress;
	EEDR = ucData;
	// Write logical one to EEMPE 
	EECR |= (1<<EEMPE);
	// Start eeprom write by setting EEPE 
	EECR |= (1<<EEPE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	// Wait for completion of previous write
	while(EECR & (1<<EEPE));
	// Set up address register
	EEAR = uiAddress;
	// Start eeprom read by writing EERE
	EECR |= (1<<EERE);
	// Return data from Data Register
	return EEDR;
}

ISR(PCINT0_vect)
{
	PCMSK0 &= ~(1<<ADS_P_DRDY);
	ADS_data_ready = 1;
}

USB_PUBLIC uchar usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (void *)data;
	static uchar    replyBuf[8];

	usbMsgPtr = replyBuf;
	if(rq->bRequest == 1) // return results
	{  
		if (ADS_ready)
		{
			replyBuf[0] = R_buffer[0][0]; // most significant byte
			replyBuf[1] = R_buffer[0][1];
			replyBuf[2] = R_buffer[0][2]; // least significant byte
			replyBuf[3] = 0;
			replyBuf[4] = R_buffer[2][0];
			replyBuf[5] = R_buffer[2][1];
			replyBuf[6] = R_buffer[2][2];
			replyBuf[7] = 0;
		}
		
		return 8;
		}
		if(rq->bRequest == 2) // return results
		{  
		if (ADS_ready)
		{
			replyBuf[0] = R_buffer[1][0];
			replyBuf[1] = R_buffer[1][1];
			replyBuf[2] = R_buffer[1][2];
			replyBuf[3] = 0;
			replyBuf[4] = R_buffer[3][0];
			replyBuf[5] = R_buffer[3][1];
			replyBuf[6] = R_buffer[3][2];
			replyBuf[7] = 0;
		}
		
		return 8;
		}
		if(rq->bRequest == 3) // return results
		{  
		if (ADS_ready)
		{
			replyBuf[0] = R_buffer[4][0];
			replyBuf[1] = R_buffer[4][1];
			replyBuf[2] = R_buffer[4][2];
			replyBuf[3] = 0;
			replyBuf[4] = 0;
			replyBuf[5] = 0;
			replyBuf[6] = 0;
			replyBuf[7] = 0;
		}

		return 8;
		}
		if(rq->bRequest == 4) // initiate measurment
		{  
		if (ADS_ready)
		{
			do_measurement = 1;
			//replyBuf[0] = 0;
			ADS_setup_select_ADC01();
			ADS_select_MOSFET01();
			ADS_start_pulse();
		}
	}
	if(rq->bRequest == 5) // read registers 1
	{  

		if (ADS_ready)
		{
		  // first half of all registers
			replyBuf[0] = ADS_reg_read(ADS_REG_MUX0);    
			replyBuf[1] = ADS_reg_read(ADS_REG_VBIAS);   
			replyBuf[2] = ADS_reg_read(ADS_REG_MUX1);    
			replyBuf[3] = ADS_reg_read(ADS_REG_SYS0);    
			replyBuf[4] = ADS_reg_read(ADS_REG_OFC0);    
			replyBuf[5] = ADS_reg_read(ADS_REG_OFC1);    
			replyBuf[6] = ADS_reg_read(ADS_REG_OFC2);    
			replyBuf[7] = ADS_reg_read(ADS_REG_FSC0);    
		}  
		return 8;
	}   
	if(rq->bRequest == 6) 
	{  
		if (ADS_ready)
		{                   
		  // second half of all registers
			replyBuf[0] = ADS_reg_read(ADS_REG_FSC1);    
			replyBuf[1] = ADS_reg_read(ADS_REG_FSC2);    
			replyBuf[2] = ADS_reg_read(ADS_REG_IDAC0);   
			replyBuf[3] = ADS_reg_read(ADS_REG_IDAC1);   
			replyBuf[4] = ADS_reg_read(ADS_REG_GPIOCFG); 
			replyBuf[5] = ADS_reg_read(ADS_REG_GPIODIR); 
			replyBuf[6] = ADS_reg_read(ADS_REG_GPIODAT); 
			replyBuf[7] = 0; 
		}  
		return 8;
	}   
	if(rq->bRequest == 7)  
	{  
		// if calib_buffer was not yet restored from eeprom, do it now)
		if ((calib_buffer_a[0] & calib_buffer_a[1] & calib_buffer_a[2] & calib_buffer_a[3] & 
			calib_buffer_a[0] & calib_buffer_a[1] & calib_buffer_a[2] & calib_buffer_a[3]) == 0xff)
		{
			for (int i = 0; i < 4; ++i)
				calib_buffer_a[i] = EEPROM_read(CAL_START_ADDR + i);
			for (int i = 0; i < 4; ++i)
				calib_buffer_b[i] = EEPROM_read(CAL_START_ADDR + 4 + i);
		}
		// return calibration coefficients
		replyBuf[0] = calib_buffer_a[0];
		replyBuf[1] = calib_buffer_a[1];
		replyBuf[2] = calib_buffer_a[2];
		replyBuf[3] = calib_buffer_a[3];
		replyBuf[4] = calib_buffer_b[0];
		replyBuf[5] = calib_buffer_b[1];
		replyBuf[6] = calib_buffer_b[2];
		replyBuf[7] = calib_buffer_b[3];
		return 8;
	}   	
	if(rq->bRequest == 8)  
	{  
		// receive calibration coefficients
		calib_buffer_a[0] = rq->wValue.bytes[1];
		calib_buffer_a[1] = rq->wValue.bytes[0];
		replyBuf[0] = calib_buffer_a[0];
		replyBuf[1] = calib_buffer_a[1];
		EEPROM_write(CAL_START_ADDR + 0, calib_buffer_a[0]);
		EEPROM_write(CAL_START_ADDR + 1, calib_buffer_a[1]);
		return 2;
	}   
	if(rq->bRequest == 9)  
	{  
		// receive calibration coefficients
		calib_buffer_a[2] = rq->wValue.bytes[1];
		calib_buffer_a[3] = rq->wValue.bytes[0];
		replyBuf[0] = calib_buffer_a[2];
		replyBuf[1] = calib_buffer_a[3];
		EEPROM_write(CAL_START_ADDR + 2, calib_buffer_a[2]);
		EEPROM_write(CAL_START_ADDR + 3, calib_buffer_a[3]);
		return 2;
	}   
	if(rq->bRequest == 10)  
	{  
		// receive calibration coefficients
		calib_buffer_b[0] = rq->wValue.bytes[1];
		calib_buffer_b[1] = rq->wValue.bytes[0];
		replyBuf[0] = calib_buffer_b[0];
		replyBuf[1] = calib_buffer_b[1];
		EEPROM_write(CAL_START_ADDR + 4, calib_buffer_b[0]);
		EEPROM_write(CAL_START_ADDR + 5, calib_buffer_b[1]);
		return 2;
	}   
	if(rq->bRequest == 11)  
	{  
		// receive calibration coefficients
		calib_buffer_b[2] = rq->wValue.bytes[1];
		calib_buffer_b[3] = rq->wValue.bytes[0];
		replyBuf[0] = calib_buffer_b[2];
		replyBuf[1] = calib_buffer_b[3];
		EEPROM_write(CAL_START_ADDR + 6, calib_buffer_b[2]);
		EEPROM_write(CAL_START_ADDR + 7, calib_buffer_b[3]);
		return 2;
	}   
	return 0;
}


int main(void)
{
		uchar   i;

		wdt_enable(WDTO_1S);
		//odDebugInit();
		
		ADS_init();
		//ADS_start();
		
/* We fake an USB disconnect by pulling D+ and D- to 0 during reset. This is
 * necessary if we had a watchdog reset or brownout reset to notify the host
 * that it should re-enumerate the device. Otherwise the host's and device's
 * concept of the device-ID would be out of sync.
 */
		usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
		i = 0;
		while(--i){         /* fake USB disconnect for > 500 ms */
				wdt_reset();
				_delay_ms(2);
		}
		usbDeviceConnect();
		usbInit();
		sei();

		for(uint32_t n = 0;; ++n) /* main event loop */
		{    
		if (!ADS_ready && n > 200000) 
		{
			ADS_ready = 1;
			ADS_setup_enable_voltage_ref();
			ADS_PGA_and_rate_setup(7, 3); // PGA:7 = 128, rate:3 = 40 SPS
			ADS_IEXC_setup(3); // 1=50uA, 2=100uA, 3=250uA, 4=500uA, 5=750uA, 6=1mA, 7=1.5mA
			ADS_no_MOSFET();
			//ADS_setup_enable_offset_measurement();
			ADS_SPI_transmit(ADS_COMMAND_SELFOCAL);
			_delay_ms(500);
			//ADS_setup_enable_gain_measurement();
			//ADS_SPI_transmit(ADS_COMMAND_SYSGCAL);
			//_delay_ms(500);
			//ADS_setup_enable_voltage_ref();
			ADS_setup_select_ADC01();
			ADS_select_MOSFET01();
		}
	

		// each time a measurement is done, read the datat into a buffer and increment the buffer index.
		if (ADS_data_ready)
		{
			uint8_t buffer_nr = do_measurement - 1;
			//ADS_read_data(&adc_buffer[ADS_data_buffer]);
			
			ADS_read_data(adc_buffer[buffer_nr]);
			
			switch(do_measurement)
			{
				case 1:  // prepare for next measurement and send start pulse
					ADS_setup_select_ADC23(); 
					ADS_select_MOSFET23();
					//ADS_no_MOSFET();
					_delay_ms(1);
					ADS_start_pulse();
					do_measurement++;
				break;
				case 2:  // prepare for next measurement and send start pulse
					ADS_setup_select_ADC45();
					ADS_select_MOSFET45();
					//ADS_no_MOSFET();
					_delay_ms(1);
					ADS_start_pulse();
					do_measurement++;
				break;
				case 3:  // prepare for next measurement and send start pulse
					ADS_setup_select_ADC67();
					ADS_select_MOSFET67();
					//ADS_no_MOSFET();
					_delay_ms(1);
					ADS_start_pulse();
					do_measurement++;
				break;
				case 4: // prepare for next measurement and send start pulse
						// prepare on-board temperature measurment
					ADS_no_MOSFET();
					ADS_setup_enable_diode_temp();
					_delay_ms(1);
					ADS_start_pulse();
					do_measurement++;
				break;
				case 5: // prepare for first measurement but don't send start pulse (end of measurment)
					ADS_setup_enable_voltage_ref();
					ADS_setup_select_ADC01();
					ADS_select_MOSFET01();
					//ADS_no_MOSFET();
					_delay_ms(1);
					do_measurement = 0;
					ADS_data_ready = 0;
				break;
			}
			if (do_measurement == 0)
			{
				uint32_t R = ADS_resistance(adc_buffer[0]);
				R_buffer[0][2] = R; R >>= 8;
				R_buffer[0][1] = R; R >>= 8;
				R_buffer[0][0] = R;
				R = ADS_resistance(adc_buffer[1]);
				R_buffer[1][2] = R; R >>= 8;
				R_buffer[1][1] = R; R >>= 8;
				R_buffer[1][0] = R;
				R = ADS_resistance(adc_buffer[2]);
				R_buffer[2][2] = R; R >>= 8;
				R_buffer[2][1] = R; R >>= 8;
				R_buffer[2][0] = R;
				R = ADS_resistance(adc_buffer[3]);
				R_buffer[3][2] = R; R >>= 8;
				R_buffer[3][1] = R; R >>= 8;
				R_buffer[3][0] = R;
				
				R_buffer[4][2] = adc_buffer[4][2];
				R_buffer[4][1] = adc_buffer[4][1];
				R_buffer[4][0] = adc_buffer[4][0];
			}
		}
			
				wdt_reset();
				usbPoll();
		}
		return 0;
}

