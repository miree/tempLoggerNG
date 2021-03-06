/* Name: templogger.c
 * Project: adc measurement based on AVR USB driver
 * Author: Christian Starkjohann, modified by Michael Reese 2015
 * Creation Date: 2005-01-16
 * Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt)
 * This Revision: $Id$
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */
#include <math.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#define USBDEV_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   0x05DC  /* Obdev's free shared PID */
/* Use obdev's generic shared VID/PID pair and follow the rules outlined
 * in firmware/usbdrv/USBID-License.txt.
 */

#define PSCMD_ECHO                 0
#define PSCMD_READ_0_1             1
#define PSCMD_READ_2_3             2
#define PSCMD_READ_TEMP            3
#define PSCMD_MEASURE              4
#define PSCMD_READ_REGS_FIRST      5
#define PSCMD_READ_REGS_SECOND     6
#define PSCMD_READ_CALIB_COEFF     7
#define PSCMD_WRITE_CALIB_COEFFa1  8
#define PSCMD_WRITE_CALIB_COEFFa2  9
#define PSCMD_WRITE_CALIB_COEFFb1 10
#define PSCMD_WRITE_CALIB_COEFFb2 11
/* These are the vendor specific SETUP commands implemented by our USB device */

#define MAX_V (1250.*196./218.)


static int  usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen)
{
char    buffer[256];
int     rval, i;

	if((rval = usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, buffer, sizeof(buffer), 1000)) < 0)
		return rval;
	if(buffer[1] != USB_DT_STRING)
		return 0;
	if((unsigned char)buffer[0] < rval)
		rval = (unsigned char)buffer[0];
	rval /= 2;
	/* lossy conversion to ISO Latin1 */
	for(i=1;i<rval;i++){
		if(i > buflen)  /* destination buffer overflow */
			break;
		buf[i-1] = buffer[2 * i];
		if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
			buf[i-1] = '?';
	}
	buf[i-1] = 0;
	return i-1;
}


/* meacon uses the free shared default VID/PID. If you want to see an
 * example device lookup where an individually reserved PID is used, see our
 * RemoteSensor reference implementation.
 */

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

static int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName)
{
struct usb_bus      *bus;
struct usb_device   *dev;
usb_dev_handle      *handle = NULL;
int                 errorCode = USB_ERROR_NOTFOUND;
static int          didUsbInit = 0;

	if(!didUsbInit){
		didUsbInit = 1;
		usb_init();
	}
	usb_find_busses();
	usb_find_devices();
	for(bus=usb_get_busses(); bus; bus=bus->next){
		for(dev=bus->devices; dev; dev=dev->next){
			if(dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product){
				char    string[256];
				int     len;
				handle = usb_open(dev); /* we need to open the device in order to query strings */
				if(!handle){
					errorCode = USB_ERROR_ACCESS;
					fprintf(stderr, "Warning: cannot open USB device: %s\n", usb_strerror());
					continue;
				}
				if(vendorName == NULL && productName == NULL){  /* name does not matter */
					break;
				}
				/* now check whether the names match: */
				len = usbGetStringAscii(handle, dev->descriptor.iManufacturer, 0x0409, string, sizeof(string));
				if(len < 0){
					errorCode = USB_ERROR_IO;
					fprintf(stderr, "Warning: cannot query manufacturer for device: %s\n", usb_strerror());
				}else{
					errorCode = USB_ERROR_NOTFOUND;
					/* fprintf(stderr, "seen device from vendor ->%s<-\n", string); */
					if(strcmp(string, vendorName) == 0){
						len = usbGetStringAscii(handle, dev->descriptor.iProduct, 0x0409, string, sizeof(string));
						if(len < 0){
							errorCode = USB_ERROR_IO;
							fprintf(stderr, "Warning: cannot query product for device: %s\n", usb_strerror());
						}else{
							errorCode = USB_ERROR_NOTFOUND;
							/* fprintf(stderr, "seen product ->%s<-\n", string); */
							if(strcmp(string, productName) == 0)
								break;
						}
					}
				}
				usb_close(handle);
				handle = NULL;
			}
		}
		if(handle)
			break;
	}
	if(handle != NULL){
		errorCode = 0;
		*device = handle;
	}
	return errorCode;
}

double Resistance(double adc)
{
	return adc/4096.; // values from hardware are already resistances ( in  units of 4096 ohms )
}

double Rpt100(double T) // R(T) for T < 0 deg C
{
	double A =  3.9083e-3;
	double B = -5.775e-7;
	double C = -4.183e-12;
	return 100 * (1 + A*T + B*T*T + C* (T-100)*T*T*T);
}
double Temp(double R)
{
	double R0 = 100;
	if (R > R0) // T > 0 (Celsius)
	{
		double A =  3.9083e-3;
		double B = -5.775e-7;
		double T = (-A*R0 + sqrt((A*R0)*(A*R0) - 4*B*R0*(R0-R)) ) / ( 2*B*R0 );
		return T;
	}
	else
	{
		double Tmin = -200, Tmax = 0;
		while (Tmax-Tmin > 1e-8)
		{
			//double Rmin = Rpt100(Tmin);
			//double Rmax = Rpt100(Tmax);
			double Tmed = 0.5*(Tmin+Tmax);
			double Rmed = Rpt100(Tmed);
			if (Rmed > R) Tmax = Tmed;
			else          Tmin = Tmed;
		}
		return 0.5*(Tmin+Tmax);
	}
}

double R_temp_compensated(double R, int32_t V_diode, double a, double b, int temp_compensation)
{
	if (temp_compensation)
	{
		double deltaR = a - b*V_diode;
		return R - deltaR;
	}
	return R;
}

void read_calibration_coefficients(usb_dev_handle *handle, double *a, double *b)
{
	unsigned char       buffer[8];
	int                 nBytes;
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_READ_CALIB_COEFF, 0, 0, (char *)buffer, sizeof(buffer), 5000);
	if(nBytes < 8){
		if(nBytes < 0)
			fprintf(stderr, "USB error: %s\n", usb_strerror());
		fprintf(stderr, "only %d bytes status received\n", nBytes);
		exit(1);
	}	
	*a = 1.0*(uint32_t)(buffer[0]<<24 | buffer[1]<<16 | buffer[2]<<8 | buffer[3])/0x0000ffffu;
	*b = 1.0*(uint32_t)(buffer[4]<<24 | buffer[5]<<16 | buffer[6]<<8 | buffer[7])/0xffffffffu;
}

double single_readout(FILE *f, usb_dev_handle *handle, double T_start,  int ch1, int ch2, int ch3, int ch4, int verbose, int resistance, double a, double b, int temp_compensation)
{
	unsigned char       buffer[8];
	int                 nBytes;
	// read channels 0 and 1
	//FILE *f = fopen("data.dat","a+t");
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_READ_0_1, 0, 0, (char *)buffer, sizeof(buffer), 5000);
	if(nBytes < 2){
		if(nBytes < 0)
			fprintf(stderr, "USB error: %s\n", usb_strerror());
		fprintf(stderr, "only %d bytes status received\n", nBytes);
		exit(1);
	}
	int32_t adc0 = ((buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]))/256;
	int32_t adc1 = ((buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | (buffer[7]))/256;
	
	// read channels 2 and 3
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_READ_2_3, 0, 0, (char *)buffer, sizeof(buffer), 5000);
	if(nBytes < 2){
		if(nBytes < 0)
			fprintf(stderr, "USB error: %s\n", usb_strerror());
		fprintf(stderr, "only %d bytes status received\n", nBytes);
		exit(1);
	}

	int32_t adc2 = ((buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]))/256;
	int32_t adc3 = ((buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | (buffer[7]))/256;

	// read diode temperature
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_READ_TEMP, 0, 0, (char *)buffer, sizeof(buffer), 5000);
	if(nBytes < 2){
		if(nBytes < 0)
			fprintf(stderr, "USB error: %s\n", usb_strerror());
		fprintf(stderr, "only %d bytes status received\n", nBytes);
		exit(1);
	}

	int32_t V_diode = ((buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]))/256;
	
	double R0    = Resistance(adc0); 
	double R1    = Resistance(adc1); 
	double R2    = Resistance(adc2); 
	double R3    = Resistance(adc3); 
	double R0_tc = R_temp_compensated(R0,V_diode,a,b,temp_compensation);
	double R1_tc = R_temp_compensated(R1,V_diode,a,b,temp_compensation);
	double R2_tc = R_temp_compensated(R2,V_diode,a,b,temp_compensation);
	double R3_tc = R_temp_compensated(R3,V_diode,a,b,temp_compensation);
	double T0    = Temp(R0_tc);
	double T1    = Temp(R1_tc);
	double T2    = Temp(R2_tc);
	double T3    = Temp(R3_tc);


	struct timeval tv;
	gettimeofday(&tv, NULL);
	double T = tv.tv_sec + tv.tv_usec/1000000.;
	T -= T_start;
	//fprintf(f,"%f %f %f %f %f \n", T, T0,T1,T2,T3);
	fprintf(f,"%8.2f %8d ", T, V_diode);
	if (resistance)
	{
		if (ch1) fprintf(f, "%lf ", R0);
		if (ch2) fprintf(f, "%lf ", R1);
		if (ch3) fprintf(f, "%lf ", R2);
		if (ch4) fprintf(f, "%lf ", R3);
	}
	else
	{
		if (ch1) fprintf(f, "%f ", T0);
		if (ch2) fprintf(f, "%f ", T1);
		if (ch3) fprintf(f, "%f ", T2);
		if (ch4) fprintf(f, "%f ", T3);
	}
	fprintf(f,"\n");
	
	if (verbose)
	{
		printf("time: %3.2f    V_diode:%7d", T, V_diode);
		if (verbose == 2)
		{
			printf("\n");
			if (ch1) printf("adc1: %7d   ", adc0);
			if (ch2) printf("adc2: %7d   ", adc1);
			if (ch3) printf("adc3: %7d   ", adc2);
			if (ch4) printf("adc4: %7d   ", adc3);
			printf("\n");
			if (ch1) printf("  R1: %7.3f   ", R0_tc);
			if (ch2) printf("  R2: %7.3f   ", R1_tc);
			if (ch3) printf("  R3: %7.3f   ", R2_tc);
			if (ch4) printf("  R4: %7.3f   ", R3_tc);
			printf("\n");
		}
		if (ch1) printf("  T1: %7.3f   ", T0);
		if (ch2) printf("  T2: %7.3f   ", T1);
		if (ch3) printf("  T3: %7.3f   ", T2);
		if (ch4) printf("  T4: %7.3f   ", T3);
		printf("\n");	
	}
	fflush(f);
	return T; // returns time after start of measurement
}

void trigger_measurement(usb_dev_handle *handle)
{
	unsigned char       buffer[8];
	int                 nBytes;	
	//printf("trigger measurement\n");
	nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_MEASURE  , 0, 0, (char *)buffer, sizeof(buffer), 5000);
	nBytes = nBytes;
}

void print_usage(char *argv0)
{
	printf("usage: %s [ch1] [ch2] [ch3] [ch4] [all] [options]\n", argv0);
	printf("options are:\n");
	printf(" \n");
	printf(" -v          : verbose\n");
	printf(" -q          : quiet\n");
	printf(" -o filename : name of data file. default is \"temperature.dat\"\n");
	printf(" -N n        : stop measurement after n samples\n");
	printf(" -t t        : stop measurement after t seconds\n");
	printf(" -d delta    : take a sample every delta seconds. Default is 0.5 , min is 0.15\n");
	printf(" -r          : read registers (for debugging purposes)\n");
	printf(" -h          : output this help\n");
	printf(" \n");
	printf("calibration procedure:\n");
	printf(" -R          : write resistance instead of temperature to data file\n");
	printf(" -x          : no compensation for device temperature. Useful for calibration\n");
	printf(" -c a b      : apply calibration coefficients for temperature compensation to the device\n");
	printf("               R_comp = a - b * V_diode\n");
	printf("               R = R_measured - R_comp\n");
	printf("               a and b can be measured by switching temperature compensation\n");
	printf("               off (using -x switch) and correlating measured resitance of a fixed value resistor\n");
	printf("               e.g. with 100 ohms (using -R switch) with the on-chip diode junction voltage V_diode.\n");
	printf("               Then heat up the device (not the resistor) and take data while it cools down.\n");
	printf("               Plot \"V_diode\" vs. \"R_measured - 100 ohms\" and fit a line to determine a and b\n");
	printf("               Finally write coefficients a and b into the device (using switch -c a b)\n");
}

int main(int argc, char **argv)
{

	usb_dev_handle      *handle = NULL;
	unsigned char       buffer[8];
	int                 nBytes;
	
	int ch1 = 0;
	int ch2 = 0;
	int ch3 = 0;
	int ch4 = 0;
	
	int Tmax = 0;
	int Nmax = 0;
	float delta_t = 0.5;
	
	int resistance = 0; // if set to 1, resistance is written to file instead of temperature
	
	int verbose = 1;
	
	if (argc == 1)
	{
		print_usage(argv[0]);
		return 0;
	}

	usb_init();
	if(usbOpenDevice(&handle, USBDEV_SHARED_VENDOR, "www.obdev.at", USBDEV_SHARED_PRODUCT, "Temp-Logger") != 0)
	{
		fprintf(stderr, "Could not find USB device \"Temp-Logger\" with vid=0x%x pid=0x%x\n", USBDEV_SHARED_VENDOR, USBDEV_SHARED_PRODUCT);
		exit(1);
	}
	// We have searched all devices on all busses for our USB device above. Now
	// try to open it and perform the vendor specific control operations for the
	// function requested by the user.
	// 
	if(argc == 2 && strcmp(argv[1], "test") == 0)
	{
		int i, v, r;
		// The test consists of writing 1000 random numbers to the device and checking
		// the echo. This should discover systematic bit errors (e.g. in bit stuffing).
		// 
		for(i=0;i<1000;i++)
		{
			v = rand() & 0xffff;
			nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_ECHO, v, 0, (char *)buffer, sizeof(buffer), 5000);
			if(nBytes < 2)
			{
				if(nBytes < 0)
					fprintf(stderr, "USB error: %s\n", usb_strerror());
				fprintf(stderr, "only %d bytes received in iteration %d\n", nBytes, i);
				fprintf(stderr, "value sent = 0x%x\n", v);
				exit(1);
			}
			r = buffer[0] | (buffer[1] << 8);
			if(r != v)
			{
				fprintf(stderr, "data error: received 0x%x instead of 0x%x in iteration %d\n", r, v, i);
				exit(1);
			}
		}
		printf("test succeeded\n");
	}
	else if (argc == 4 && argv[1][0] == '-' && argv[1][1] == 'c') 
	{
		float af,bf;
		sscanf(argv[2],"%f",&af);
		sscanf(argv[3],"%f",&bf);
		uint32_t a = (uint32_t)(af*0x0000ffffu);
		uint32_t b = (uint32_t)(bf*0xffffffffu);
		printf("a=%d\n",a);
		printf("b=%d\n",b);
		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_WRITE_CALIB_COEFFa1, (a>>16)&0xffff, 0, (char *)buffer, sizeof(buffer), 5000);		
		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_WRITE_CALIB_COEFFa2,      a &0xffff, 0, (char *)buffer, sizeof(buffer), 5000);		
		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_WRITE_CALIB_COEFFb1, (b>>16)&0xffff, 0, (char *)buffer, sizeof(buffer), 5000);		
		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_WRITE_CALIB_COEFFb2,      b &0xffff, 0, (char *)buffer, sizeof(buffer), 5000);		
		if(nBytes < 2)
		{
			if(nBytes < 0)
				fprintf(stderr, "USB error: %s\n", usb_strerror());
			exit(1);
		}
		printf("buffer[0] = 0x%08x\n", buffer[0]);
		printf("buffer[1] = 0x%08x\n", buffer[1]);
	}
	else if (argc == 2 && argv[1][0] == '-' && argv[1][1] == 'r') 
	{

		printf("reading registers:\n");
		char *reg_name_first[]  = {"MUX0   ","VBIAS  ","MUX1   ","SYS0   ","OFC0   ","OFC1   ","OFC2   ","FSC0   "};
		char *reg_name_second[] = {"FSC1   ","FSC2   ","IDAC0  ","IDAC1  ","GPIOCFG","GPIODIR","GPIODAT","-      "};

		char *reg_bits[15][15]= {
									{	"BCS1   ","BCS0    ","MUX_SP2 ","MUX_SP1 ","MUX_SP0  ","MUX_SN2","MUX_SN1","MUX_SN0"},
									{	"VBIAS7 ","VBIAS6  ","VBIAS5  ","VBIAS4  ","VBIAS3   ","VBIAS2 ","VBIAS1 ","VBIAS0 "},
									{	"CLKSTAT","VREFCON1","VREFCON0","REFSELT1","REFSELT0 ","MUXCAL2","MUXCAL1","MUXCAL0"},
									{	"0      ","PGA2    ","PGA1    ","PGA0    ","DR3      ","DR2    ","DR1    ","DR0    "},
									{	"OFC7   ","OFC6    ","OFC5    ","OFC4    ","OFC3     ","OFC2   ","OFC1   ","OFC0   "},
									{	"OFC15  ","OFC14   ","OFC13   ","OFC12   ","OFC11    ","OFC10  ","OFC9   ","OFC8   "},
									{	"OFC23  ","OFC22   ","OFC21   ","OFC20   ","OFC19    ","OFC18  ","OFC17  ","OFC16  "},
									{	"FSC7   ","FSC6    ","FSC5    ","FSC4    ","FSC3     ","FSC2   ","FSC1   ","FSC0   "},
									{	"FSC15  ","FSC14   ","FSC13   ","FSC12   ","FSC11    ","FSC10  ","FSC9   ","FSC8   "},
									{	"FSC23  ","FSC22   ","FSC21   ","FSC20   ","FSC19    ","FSC18  ","FSC17  ","FSC16  "},
									{	"ID3    ","ID2     ","ID1     ","ID0     ","DRDY/MODE","IMAG2  ","IMAG1  ","IMAG0  "},
									{	"I1DIR3 ","I1DIR2  ","I1DIR1  ","I1DIR0  ","I2DIR3   ","I2DIR2 ","I2DIR1 ","I2DIR0 "},
									{	"IOCFG7 ","IOCFG6  ","IOCFG5  ","IOCFG4  ","IOCFG3   ","IOCFG2 ","IOCFG1 ","IOCFG0 "},
									{	"IODIR7 ","IODIR6  ","IODIR5  ","IODIR4  ","IODIR3   ","IODIR2 ","IODIR1 ","IODIR0 "},
									{	"IODAT7 ","IODAT6  ","IODAT5  ","IODAT4  ","IODAT3   ","IODAT2 ","IODAT1 ","IODAT0 "}
								};

		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_READ_REGS_FIRST, 0, 0, (char *)buffer, sizeof(buffer), 5000);
		//printf("%d bytes read\n", nBytes);
		for (int i = 0; i < 8; ++i)
		{
			printf("%s = %02x\n", reg_name_first[i], buffer[i]);
			for (int j = 0; j < 8; ++j)
				printf("  %10s", reg_bits[i][j]);
			printf("\n");
			for (int j = 0; j < 8; ++j)
				printf("     %d      ", (buffer[i] & (1<<(7-j)))>>(7-j));
			printf("\n");
			printf("--------------------------------------------------------------------------------------------------\n");
		}
		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_READ_REGS_SECOND, 0, 0, (char *)buffer, sizeof(buffer), 5000);
		//printf("%d bytes read\n", nBytes);
		for (int i = 0; i < 7; ++i)
		{
			printf("%s = %02x\n", reg_name_second[i], buffer[i]);
			for (int j = 0; j < 8; ++j)
				printf("  %10s", reg_bits[i+8][j]);
			printf("\n");
			for (int j = 0; j < 8; ++j)
				printf("     %d      ", (buffer[i] & (1<<(7-j)))>>(7-j));
			printf("\n");
			printf("--------------------------------------------------------------------------------------------------\n");
		}

	}else if (argc == 2 && argv[1][0] == '-' && argv[1][1] == 'm') {
		printf("trigger measurement\n");
		//nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, PSCMD_MEASURE  , 0, 0, (char *)buffer, sizeof(buffer), 5000);
		trigger_measurement(handle);
	}else {
		double a,b;
		int temp_compensation = 1; // true
		// do a measurement
		const char* filename = "temperature.dat";
		int i;
		for (i = 1; i < argc; ++i)
		{
			if (strcmp(argv[i], "-x") == 0)
			{
				temp_compensation = 0; // false
			}			
			if (strcmp(argv[i], "-o") == 0)
			{
				++i;
				if (i == argc)
				{
					printf("ERROR: filename expected after -o\n");
					return 1;
				}
				filename = argv[i];
				printf("filename is %s\n", filename);
			}
			if (strcmp(argv[i], "-N") == 0)
			{
				++i;
				if (i == argc)
				{
					printf("ERROR: number of samples expected after -N\n");
					return 1;
				}
				sscanf(argv[i],"%d", &Nmax);
				printf("taking %d smaples\n", Nmax);
			}
			if (strcmp(argv[i], "-t") == 0)
			{
				++i;
				if (i == argc)
				{
					printf("ERROR: time in seconds expected after -t\n");
					return 1;
				}
				sscanf(argv[i],"%d", &Tmax);
				printf("taking %d seconds of data\n", Tmax);
			}						
			if (strcmp(argv[i], "-d") == 0)
			{
				++i;
				if (i == argc)
				{
					printf("ERROR: delta time in seconds expected after -d\n");
					return 1;
				}
				sscanf(argv[i],"%f", &delta_t);
				if (delta_t < 0.15) 
				{
					printf("delta_t too small, changing it to 0.15 sec\n");
					delta_t = 0.15;
				}
				printf("taking one sample per %f seconds\n", delta_t);
			}			
			if (strcmp(argv[i], "ch1") == 0) ch1 = 1;
			if (strcmp(argv[i], "ch2") == 0) ch2 = 1;
			if (strcmp(argv[i], "ch3") == 0) ch3 = 1;
			if (strcmp(argv[i], "ch4") == 0) ch4 = 1;
			if (strcmp(argv[i], "all") == 0) ch1 = ch2 = ch3 = ch4 = 1;
			if (strcmp(argv[i], "-v") == 0) verbose = 2;
			if (strcmp(argv[i], "-q") == 0) verbose = 0;
			if (strcmp(argv[i], "-R") == 0) resistance = 1;
			if (strcmp(argv[i], "-h") == 0)
			{
				print_usage(argv[0]);
				return 0;
			}
		}

		if (temp_compensation)
		{
			read_calibration_coefficients(handle, &a, &b);
			if (verbose >= 1)
			{
				printf("calibration coefficients for temp. compensation: a=%lf  b=%lf   (deltaR = a - b*V_diode)\n", a, b);
			}
		}

		
		if ((ch1 || ch2 || ch3 || ch4)   == 0)
		{
			printf("ERROR: no channels given\n");
			printf("specify one or more of ch1,ch2,ch3,ch4,all as argument\n\n");
			return 1;
		}
		
		// read channels 0 and 1
		FILE *f = fopen(filename, "w+t");
		
		
		
		struct timeval tv;
		gettimeofday(&tv, NULL);
		double T_start = tv.tv_sec + tv.tv_usec/1000000.;
		
		fprintf(f, "# starting timestamp: %d    (convert a the timestamp to human readable date with: date --date=\"@%d\"\n", (int)tv.tv_sec, (int)tv.tv_sec);
		fprintf(f, "# time[s]  V(diode)  ");
		if (resistance)
		{
			if (ch1) fprintf(f, "R(ch1) ");
			if (ch2) fprintf(f, "R(ch2) ");
			if (ch3) fprintf(f, "R(ch3) ");
			if (ch4) fprintf(f, "R(ch4) ");		
		}
		else
		{
			if (ch1) fprintf(f, "T(ch1) ");
			if (ch2) fprintf(f, "T(ch2) ");
			if (ch3) fprintf(f, "T(ch3) ");
			if (ch4) fprintf(f, "T(ch4) ");		
		}
		fprintf(f,"\n");
		
		int n = 0;
		for (;;++n)
		{
			if (Nmax != 0 && n >= Nmax) break;
			trigger_measurement(handle);

			struct timespec req,rem;
			req.tv_sec  = (long)( (delta_t));
			req.tv_nsec = (long)(((delta_t)-req.tv_sec)*1000000000L);
			nanosleep(&req, &rem);

			double Tsec = single_readout(f, handle, T_start, ch1, ch2, ch3, ch4, verbose, resistance, a,b, temp_compensation);
			if (Tmax != 0 && Tsec > Tmax) break;
			
		}

	}
	usb_close(handle);
	return 0;
}
