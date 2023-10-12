#ifndef GPIO_H
#define GPIO_H

#define IN  0
#define OUT 1
 
#define LOW  0
#define HIGH 1

int GPIOExport(int pin);
int GPIOUnexport(int pin);
int GPIODirection(int pin, int dir);
int GPIOOpenValue(int pin, int mode);
int GPIOCloseValue(int fd);
int GPIORead(int fd);
int GPIOWrite(int fd, int value);

#endif //GPIO_H
