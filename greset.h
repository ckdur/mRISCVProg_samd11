#ifndef GRESET_H
#define GRESET_H

#define GPIO_RESET 18

int initGlobalReset(int g);
int finiGlobalReset(void);
int sendGlobalReset(char val);

#endif //GRESET_H

