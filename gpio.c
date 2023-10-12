/* Include GPIO handlers for samd11 */
#include "asf.h"
#include "gpio.h"

// **********************************************
// ** GPIO AUXILIAR LIBRARY
// ** Extracted from rpi examples:

#define IN  0
#define OUT 1
 
#define LOW  0
#define HIGH 1

int GPIOExport(int pin)
{
	return 1;
}
 
int GPIOUnexport(int pin)
{
	return 1;
}
 
int GPIODirection(int pin, int dir)
{
  if(dir) REG_PORT_DIR0 |= (1<<pin);
  else REG_PORT_DIR0 &= ~(1<<pin);
	return 1;
}
 
int GPIOOpenValue(int pin, int mode)
{
	return pin;
}

int GPIOCloseValue(int fd)
{
	return 1;
}
 
int GPIORead(int fd)
{
	return (REG_PORT_IN0 >> fd) & 0x1;
}
 
int GPIOWrite(int fd, int value)
{
  if(value) REG_PORT_OUT0 &= ~(1<<fd);
  else REG_PORT_OUT0 |= 1<<fd;
	return 1;
}

