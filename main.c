/*!
 * \file main.c
 *
 * \author Ckristian Duran (At On-Chip)
 * \date 20190206 (AAAAMMDD)
 *
 * Copyleft 2015-2019 On-Chip Investigation Group at UIS
 * 
 *
 * THIS SOFTWARE IS PROVIDED BY UIS 
 *
 * Rivision History:
 * 0.1  - 20190206 - Initial version
 */

// This file should work with a samd11, so, there is no compilation instructions

/******************************************************************************/
/* 							 Include files										   */
/******************************************************************************/
/* Standard C libraries */
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>

#include "asf.h"

/* Include GPIO mini library*/
#include "gpio.h"

/* Include reset library*/
#include "greset.h"

/******************************************************************************/
/*								Macro and type defines							   */
/******************************************************************************/
#define pabort(exp) {return 0;}
#define APP_CHECK_STATUS(exp) {if(exp!=FT_OK){return 0;}else{;}};
#define APP_CHECK_STATUS_INV(exp) {if(exp!=FT_OK){return 1;}else{;}};
#define CHECK_NULL(exp){if(exp==NULL){return 0;}else{;}};
#define CHECK_NULL_INV(exp){if(exp==NULL){return 1;}else{;}};
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/* Application specific macro definations */
#define CHANNEL_TO_OPEN				0	/*0 for first available channel, 1 for next... */
#define SPI_DEVICE_BUFFER_SIZE		256
#define SPI_CLOCK_RATE_HZ			1000000
#define MRISCV_MAX_SIZE				4096u
#define MRISCV_SPI_DUMMY_BITS		10
#define MRISCV_SPI_TASK_BITS		2
#define MRISCV_SPI_ADDR_BITS 		32
#define MRISCV_SPI_DATA_BITS 		32
#define MRISCV_SPI_COMM_WRITE_BITS	(MRISCV_SPI_TASK_BITS+MRISCV_SPI_ADDR_BITS+MRISCV_SPI_DATA_BITS)
#define MRISCV_SPI_COMM_READ_BITS	(MRISCV_SPI_TASK_BITS+MRISCV_SPI_ADDR_BITS+MRISCV_SPI_DATA_BITS)
#define MRISCV_SPI_COMM_RESET_BITS	(MRISCV_SPI_TASK_BITS+MRISCV_SPI_ADDR_BITS+MRISCV_SPI_DATA_BITS)
#define MRISCV_SPI_COMM_STATUS_BITS	MRISCV_SPI_TASK_BITS
#define MRISCV_SPI_COMM_SEND_BITS	MRISCV_SPI_TASK_BITS
#define MRISCV_SPI_READ_STATUS_BITS	MRISCV_SPI_DATA_BITS
#define MRISCV_SPI_READ_SEND_BITS	MRISCV_SPI_DATA_BITS
#define MRISCV_TASK_STATUS			0x0
#define MRISCV_TASK_READ			0x1
#define MRISCV_TASK_WRITE			0x2
#define MRISCV_TASK_SEND			0x3
#define MAX_COUNT_TIMEOUT			10

// CKDUR: Change me for CS pin (manual handled)
#define GPIO_CS 14 // PA14
#define GPIO_RST 2 // PA02
#define GPIO_STAT 31 // PA31
static uint8_t buffer[SPI_DEVICE_BUFFER_SIZE] = {0};
static uint8_t rbuffer[SPI_DEVICE_BUFFER_SIZE] = {0};

static uint8_t bits = 8;
static uint32_t speed = SPI_CLOCK_RATE_HZ;

extern const char _binary_test5_dat_start;
extern const char _binary_test5_dat_end;
int _binary_test5_dat_size;

struct spi_module spi_master_instance;
struct spi_slave_inst slave;
static int fd_gpio;
static int fd_gpio_stat;
struct rtc_module rtc_instance;

/******************************************************************************/
/*						Public function definitions						  		   */
/******************************************************************************/

// Declarations
void ResetSequence(void);
void configure_rtc_count(void);
unsigned sleep_ms(unsigned int msec);
//unsigned sleep(unsigned int sec);
static int send_dummy(void);
static int read_status(uint8_t *stat);
int spi_init_internal(void);
void spi_close(void);
void system_samd11_init(void);

// Definitions
void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);
	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
#ifdef FEATURE_RTC_CONTINUOUSLY_UPDATED
	config_rtc_count.continuously_update = true;
#endif
	config_rtc_count.compare_values[0]   = 1000;
	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	rtc_count_enable(&rtc_instance);
}

unsigned sleep_ms(unsigned int msec) {
	rtc_count_set_period(&rtc_instance, msec);
	rtc_count_clear_compare_match(&rtc_instance, RTC_COUNT_COMPARE_0);
	while (!rtc_count_is_compare_match(&rtc_instance, RTC_COUNT_COMPARE_0)) ;
	return 0;
}

unsigned sleep(unsigned int sec) {
  sleep_ms(sec*1000);
	return 0;
}

static int send_dummy(void)
{
	int ret;
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
		
	buffer[0] = 0;
	
	// Write and Read the data
	ret = spi_write_buffer_wait(&spi_master_instance, buffer, (MRISCV_SPI_DUMMY_BITS)/8 + 1); // byte adjust
	if (ret != STATUS_OK)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
	return 1;
}

static int read_status(uint8_t *stat)
{
	int ret;
	send_dummy();
	
	/*Write 2 bit SEND STATUS*/
	buffer[0] = MRISCV_TASK_STATUS << 6;	/* Write command (2bit, 6-bit displaced)*/
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = spi_transceive_buffer_wait(&spi_master_instance, buffer, rbuffer, (MRISCV_SPI_COMM_STATUS_BITS+MRISCV_SPI_READ_STATUS_BITS)/bits + 1); // byte adjust
	if (ret != STATUS_OK)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);

	/* Dummy Bits, for issue aditional clk cycles */
	send_dummy();
	
	/*Is done?*/
	(*stat) = rbuffer[4] >> 6;//buffer[3];
	
	send_dummy();
	return 1;
}

static int write_single_word(uint32_t address, uint32_t data)
{
	int ret;
	uint8_t stat;
	uint32_t count = 0;
RESTART_WRITE:
	/* Dummy Bits, for issue aditional clk cycles */
	send_dummy();

	/* CS_High + Write command + Address */
	//buffer[0] = ;	/* Write command (2bit, 6-bit displaced)*/
	buffer[0] = (MRISCV_TASK_WRITE << 6) | ( ( address >> 26 ) & 0x3F ); 	/*MSB  6-bit addr bits*/
	buffer[1] = ( ( address >> 18 ) & 0xFF ); 				/*Next 8-bit addr bits*/
	buffer[2] = ( ( address >> 10 ) & 0xFF ); 				/*Next 8-bit addr bits*/
	buffer[3] = ( ( address >> 2 ) & 0xFF ); 				/*Next 8-bit addr bits*/
	buffer[4] = ( ( address & 0x3 ) << 6 ); 				/*Last 2-bit addr bits*/
	buffer[4] = buffer[4] | ( ( data >> 26 ) & 0x3F ); 		/*MSB  6-bit data bits*/
	buffer[5] = ( ( data >> 18 ) & 0xFF ); 					/*Next 8-bit data bits*/
	buffer[6] = ( ( data >> 10 ) & 0xFF ); 					/*Next 8-bit data bits*/
	buffer[7] = ( ( data >> 2 ) & 0xFF ); 					/*Next 8-bit data bits*/
	buffer[8] = ( ( data & 0x3 ) << 6 ); 					/*Last 2-bit data bits*/
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = spi_write_buffer_wait(&spi_master_instance, buffer, (MRISCV_SPI_COMM_WRITE_BITS)/8 + 1); // byte adjust
	if (ret != STATUS_OK)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
	
	/* Dummy Bits, for issue aditional clk cycles */
	send_dummy();
	
	while(1) {
		read_status(&stat);
		if((stat & 0x7)) 
		{
			count ++;
			if(count > MAX_COUNT_TIMEOUT)
			{
				count = 0;
				//printf("TIMEOUT WRITE, RETRYING\n");
			  	ResetSequence();
				goto RESTART_WRITE;
			}
		}
		else break;
	}

	return 1;
}

static int read_single_word(uint32_t address, uint32_t *data)
{
	int ret;
	uint8_t stat;
	uint32_t count = 0;

RESTART_READ:
	/* Dummy Bits, for issue aditional clk cycles */
	send_dummy();

	/* CS_High + Write command + Address */
	//buffer[0] = ;	/* Write command (2bit, 6-bit displaced)*/
	buffer[0] = (MRISCV_TASK_READ << 6) | ( ( address >> 26 ) & 0x3F ); 	/*MSB  6-bit addr bits*/
	buffer[1] = ( ( address >> 18 ) & 0xFF ); 				/*Next 8-bit addr bits*/
	buffer[2] = ( ( address >> 10 ) & 0xFF ); 				/*Next 8-bit addr bits*/
	buffer[3] = ( ( address >> 2 ) & 0xFF ); 				/*Next 8-bit addr bits*/
	buffer[4] = ( ( address & 0x3 ) << 6 ); 				/*Last 2-bit addr bits*/
	//buffer[4] = buffer[4] | 0; 							/*MSB  6-bit data bits (IGNORED)*/
	buffer[5] = 0; 											/*Next 8-bit data bits (IGNORED)*/
	buffer[6] = 0; 											/*Next 8-bit data bits (IGNORED)*/
	buffer[7] = 0; 											/*Next 8-bit data bits (IGNORED)*/
	buffer[8] = 0; 										/*Last 2-bit data bits (IGNORED)*/
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = spi_write_buffer_wait(&spi_master_instance, buffer, (MRISCV_SPI_COMM_WRITE_BITS)/8 + 1); // byte adjust
	if (ret != STATUS_OK)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
	
	/* Dummy Bits, for issue aditional clk cycles */
	send_dummy();
	
	while(1) {
		read_status(&stat);
		if((stat & 0x7)) 
		{
			count ++;
			if(count > MAX_COUNT_TIMEOUT)
			{
				count = 0;
				//printf("TIMEOUT READ, RETRYING\n");
				goto RESTART_READ;
			}
		}
		else break;
	}
	
	/*Write 2 bit SEND RDATA*/ /*Read the data*/
	buffer[0] = MRISCV_TASK_SEND << 6;	/* Write command (2bit, 6-bit displaced)*/
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = spi_transceive_buffer_wait(&spi_master_instance, buffer, rbuffer, (MRISCV_SPI_COMM_SEND_BITS+MRISCV_SPI_READ_SEND_BITS)/8 + 1); // byte adjust
	if (ret != STATUS_OK)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
	
	(*data) =  (rbuffer[0] & 0x3F) << 26;
	(*data) |= rbuffer[1] << 18;
	(*data) |= rbuffer[2] << 10;
	(*data) |= rbuffer[3] << 2;
	(*data) |= (rbuffer[4] & 0xC0) >> 6;
	////printf("0x%X\n", (int)rbuffer[4]);
	
	/* Dummy Bits, for issue aditional clk cycles */
	send_dummy();

	return 1;
}

static int reset_status(uint8_t state)
{
	int ret;
	send_dummy();

	/* CS_High + Write command + Address */
	//buffer[0] = ;	/* Write command (2bit, 6-bit displaced)*/
	buffer[0] = (MRISCV_TASK_STATUS << 6) | (state & 0x3F); 								/*MSB  6-bit addr bits (IGNORED)*/
	buffer[1] = state; 							 				/*Next 8-bit addr bits (IGNORED)*/
	buffer[2] = state; 							 				/*Next 8-bit addr bits (IGNORED)*/
	buffer[3] = state; 							 				/*Next 8-bit addr bits (IGNORED)*/
	buffer[4] = state; 							 				/*Last 2-bit addr bits, MSB 6-bit data bits (IGNORED)*/
	buffer[5] = state; 											/*Next 8-bit data bits (IGNORED)*/
	buffer[6] = state; 											/*Next 8-bit data bits (IGNORED)*/
	buffer[7] = state; 											/*Next 8-bit data bits (IGNORED)*/
	buffer[8] = state;										/*Last 2-bit data bits (RESET STATE)*/
	buffer[9] = state;										/*Last 2-bit data bits (RESET STATE)*/
	buffer[10] = state;										/*Last 2-bit data bits (RESET STATE)*/
	
	// Set Intended CS to 0
	if (-1 == GPIOWrite(fd_gpio, LOW))
		return(0);

	// Write and Read the data
	ret = spi_write_buffer_wait(&spi_master_instance, buffer, (MRISCV_SPI_COMM_RESET_BITS+10)/8 + 1); // byte adjust
	if (ret != STATUS_OK)
		pabort("can't send spi message");
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
	
	/* Dummy Bits, for issue aditional clk cycles */
	send_dummy();

	return 1;
}

int spi_init_internal(void)
{
	// ** Configuring GPIO
    
  // Enable GPIO
	if (-1 == GPIOExport(GPIO_CS))
		return(0);
 
	// Set GPIO direction
	if (-1 == GPIODirection(GPIO_CS, OUT))
		return(0);

	// Open file feeder for GPIO
	fd_gpio = GPIOOpenValue(GPIO_CS, 0);
	if (-1 == fd_gpio)
		return(0);
	
	// Set Intended CS to 1
	if (-1 == GPIOWrite(fd_gpio, HIGH))
		return(0);
		
	// Enable GPIO
	if (-1 == GPIOExport(GPIO_STAT))
		return(0);
 
	// Set GPIO direction
	if (-1 == GPIODirection(GPIO_STAT, OUT))
		return(0);

	// Open file feeder for GPIO
	fd_gpio_stat = GPIOOpenValue(GPIO_STAT, 0);
	if (-1 == fd_gpio_stat)
		return(0);
	
	// Set Intended STAT to 0
	if (-1 == GPIOWrite(fd_gpio_stat, LOW))
		return(0);
		
	// ** Finish Configuring GPIO
	
	// ** Configuring SERCOM SPI
	struct spi_config config_spi_master;
	/* Configure, initialize and enable SERCOM SPI module */
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = CONF_MASTER_MUX_SETTING;
	config_spi_master.pinmux_pad0 = CONF_MASTER_PINMUX_PAD0;
	config_spi_master.pinmux_pad1 = CONF_MASTER_PINMUX_PAD1;
	config_spi_master.pinmux_pad2 = CONF_MASTER_PINMUX_PAD2;
	config_spi_master.pinmux_pad3 = CONF_MASTER_PINMUX_PAD3;
	spi_init(&spi_master_instance, CONF_MASTER_SPI_MODULE, &config_spi_master);
	spi_enable(&spi_master_instance);
	
	return 1;
}

void ResetSequence()
{
  GPIOWrite(fd_gpio_stat, HIGH);
	sendGlobalReset(1);
	send_dummy();
		reset_status(0xFF);
		sleep(2);
	
  GPIOWrite(fd_gpio_stat, LOW);
	sendGlobalReset(0);
	send_dummy();
		reset_status(0x0);
		sleep(2);
}

void spi_close(void)
{
	GPIOCloseValue(fd_gpio);
  GPIOUnexport(GPIO_CS);
}

void system_samd11_init() {
  /*samd11 classic initialization*/
  system_init();

  /*Put the clock in the AXICLK*/
  // This is at PA30, so maybe we need to set something to PINMUX_PA30H_GCLK_IO0
  struct system_pinmux_config config_pinmux;
  system_pinmux_get_config_defaults(&config_pinmux);
  config_pinmux.mux_position = MUX_PA30H_GCLK_IO0;
  config_pinmux.direction    = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
  system_pinmux_pin_set_config(PIN_PA30H_GCLK_IO0, &config_pinmux);
  
  configure_rtc_count();
}

int main(int argc, char **argv)
{
  /*samd11 classic initialization*/
  system_samd11_init();
  
	uint32_t i;
	uint32_t file_size;
	uint32_t data;
    
  /* Init Global Reset */
  initGlobalReset(GPIO_RST);
  
	//printf("Welcome to mRISC-V programmer!\n");
	
	/* Init SPI */
	if(!spi_init_internal()) 
	{
		//printf("Failed to init SPI (File: %s, Line %d)\n", __FILE__, __LINE__);
		return 1;
	}
	
	/* Deactivate the reset */
	sendGlobalReset(0);
	
	/* Issue CLK cycles */
	send_dummy();
	if(1)
	{
		_binary_test5_dat_size = (int)(&_binary_test5_dat_end) - (int)(&_binary_test5_dat_start);
		file_size = _binary_test5_dat_size;
		
		if(file_size > MRISCV_MAX_SIZE) {goto PANIC_EXIT;}
		file_size = min(MRISCV_MAX_SIZE, file_size);
		
		for(i = 0; i < (file_size/4); i+=1)
		{
			memcpy(&data, &_binary_test5_dat_start+(i*4), 4);
			while(1)
			{
				//printf("Send command WRITE 0x%x 0x%x.\n", i, data);
				if(!write_single_word(i, data)) {goto PANIC_EXIT;}
				if(1) // CHECK?
				{
				  uint32_t datar;
				  //printf("Send command READ 0x%x.\n", i);
				  if(!read_single_word(i, &datar)) {goto PANIC_EXIT;}
				
				  if(data != datar) 
				  {
						//printf("ERROR: read and written data is different, retrying... (0x%x != 0x%x)\n", data, datar);
						ResetSequence();
				  }
				  else break;
				}
				else break;
			}
			
		}
		// Put the reset
		int l;
		for(l = 0; l < 1; l++)
		{
	    if(!reset_status(0)) { goto PANIC_EXIT;}
	    sleep(1);
	
		  if(!reset_status(0xFF)) { goto PANIC_EXIT;}
		  sleep(1);
		}
	}
	
//NORMAL_EXIT:
	spi_close();
    
  /* Fini Global Reset */
  finiGlobalReset();

	//printf("Sucesfully programmed mRISC-V!\n");
	for(;;) {
	  GPIOWrite(fd_gpio_stat, LOW);
	  sleep_ms(200);
	  GPIOWrite(fd_gpio_stat, HIGH);
	  sleep_ms(200);
	}
	return 0;

PANIC_EXIT:
	spi_close();
    
  /* Fini Global Reset */
  finiGlobalReset();

	//printf("mRISCVprog terminated with errors!\n");
	for(;;) {
	  GPIOWrite(fd_gpio_stat, LOW);
	  sleep_ms(200);
	  GPIOWrite(fd_gpio_stat, HIGH);
	  sleep_ms(800);
	}
	return -1;
}
