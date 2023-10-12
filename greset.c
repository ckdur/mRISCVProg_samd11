/* Include GPIO handlers for samd11 */
#include "gpio.h"

int gpio_reset = -1;
static int fd_gpio_reset;

int initGlobalReset(int g)
{
  gpio_reset = g;
  // Enable GPIO
	if (-1 == GPIOExport(gpio_reset))
		return(0);
  
  // Set GPIO direction
	if (-1 == GPIODirection(gpio_reset, OUT))
		return(0);

	// Open file feeder for GPIO
	fd_gpio_reset = GPIOOpenValue(gpio_reset, 0);
	if (-1 == fd_gpio_reset)
		return(0);
  
  return 1;
}

int finiGlobalReset(void)
{
	GPIOCloseValue(fd_gpio_reset);
  GPIOUnexport(gpio_reset);
  return 1;
}

int sendGlobalReset(char val)
{
  // Set Intended RST to 1
	if (-1 == GPIOWrite(fd_gpio_reset, val?0:1))
		return(0);
  return 1;
}

