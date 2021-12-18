#ifndef TTY_H_
#define TTY_H_

void UART0_Init(void);
void PortF_Init(void);
void Delay(unsigned long counter);
char UART0_Receiver(void);
void UART0_Transmitter(char data);
void printstring(char *str);
void readstring(char *str);
void RemoveSpaces(char * p);
void ADC0_Initialization(void);
void ftoa(float f,char *buf);

#endif /* TTY_H_ */
