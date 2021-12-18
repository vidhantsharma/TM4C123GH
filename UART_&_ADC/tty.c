#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include <string.h>
#include <tty.h>


void UART0_Init(void)
{
      SYSCTL_RCGCUART_R |= 0x01;            /* activate UART0 */
      SYSCTL_RCGCGPIO_R |= 0x01;            /* activate port A */

      while((SYSCTL_PRGPIO_R&0x0001) == 0){};/* ready? */  /*Peripheral Ready GPIO? - 0x0001 for port A*/
      while((SYSCTL_PRUART_R&0x0001) == 0){};/* ready? */  /*Peripheral Ready GPIO? - 0x0001 for UART0*/
      UART0_CTL_R &= 0x0;      /* disable UART */
      UART0_IBRD_R = 8;        /* IBRD = int(16,000,000 / (16 * 115,200)) = int(8.680) */
      UART0_FBRD_R = 44;       /* FBRD = round(0.5104 * 64 ) = 44 */
      //UART0_LCRH_R = ((1<<6)|(1<<5)|(1<<4)); //8 bit FIFO enable
      UART0_LCRH_R = 0x60; //8 bit FIFO enable
      UART0_CC_R = 0x0;             // UART Click configuration. select system clock
      UART0_CTL_R |= 0x301;       /* enable UART,enable receive and transmit */

      GPIO_PORTA_AFSEL_R |= 0x03;           /* enable alt funct on PA1-0 */
      GPIO_PORTA_DEN_R |= 0x03;             /* enable digital I/O on PA1-0 */
      GPIO_PORTA_PCTL_R |= 0x11;            /* configure PA1-0 as UART */
      GPIO_PORTA_AMSEL_R &= ~0x03;          /* disable analog functionality on PA */
}

void PortF_Init(void){
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  Delay(10);                        // reading register adds a delay
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0
}

void ADC0_Initialization(void)
{

    SYSCTL_RCGC2_R |= (1<<4); //portE clock initialization
    SYSCTL_RCGCADC_R |= 1;       /* enable clock to ADC0 */
    GPIO_PORTE_AFSEL_R |= (1<<3);
    GPIO_PORTE_DEN_R &= ~(1<<3);
    GPIO_PORTE_AMSEL_R |= (1<<3);

    ADC0_CC_R |=0x0;
    ADC0_ACTSS_R &= ~(1<<3);             /* disable SS3 during configuration */
    ADC0_EMUX_R &= ~0xF000;
    //ADC0_EMUX_R &= 0x0;             // The trigger is initiated by setting the SSn bit in the ADCPSSI register.
    ADC0_SSMUX3_R = 0;           /* get input from channel 0 */
    ADC0_SSCTL3_R |= 0x0E;       /* take chip temperature, set flag at 1st sample */
    ADC0_ACTSS_R |= (1<<3);           /* enable ADC0 sequencer 3 */

}

void UART0_Transmitter(char data) //transmit from UART
{
    while((UART0_FR_R & 0x20) != 0); /* wait until Tx buffer not full */
    UART0_DR_R = data;                  /* before giving it another byte */
}

char UART0_Receiver(void) //receive by UART
{
    char data;    while((UART0_FR_R & 0x10) != 0); /* wait until Rx buffer is not full */
    data = UART0_DR_R;      /* before giving it another byte */
    return (char) data;
}

void printstring(char *str) //print on putty
{
  while(*str != '\0')
    {
        UART0_Transmitter(*(str++));
    }
}

void printnum(char *str) //print temperature on putty
{
    int i=0;
    while(i<8)
    {
        UART0_Transmitter(*(str++));
        i++;
    }
}

void readstring(char *str) //read from putty
{
        char ch;

        do
        {
            ch = UART0_Receiver();
            *str = ch;
            str++;
        }
        while((ch != '\n') && (ch != '\r'));
        *str = '\0';

}

void RemoveSpaces(char * p)
{
    if(p==NULL)
        return;

    int n=0;
    int i;
    for(i=0;i<strlen(p);++i)
    {
        if(p[i] !=' ')
            p[n++] = p[i];
    }
    p[n]='\0';
}

void Delay(unsigned long counter)
{
    int CLOCK_MHZ = 16; // 16 MHz Clock
    int cycles_per_ms = CLOCK_MHZ * 1000;
    NVIC_ST_RELOAD_R = (counter * cycles_per_ms) - 1;            // reload value for 'ms' milliseconds
    NVIC_ST_CTRL_R |= (1<<0) | (1<<2);    // set internal clock, enable the timer

    while ((NVIC_ST_CTRL_R & (1<<16)) == 0)  // loop until flag is set
    {
        ;   // do nothing
    }

    NVIC_ST_CTRL_R &= ~(1<<0);              // stop the timer

    return;
}

void ftoa(float f,char *buf)
{
    int pos=0,ix,dp,num;
    if (f<0)
    {
        buf[pos++]='-';
        f = -f;
    }
    dp=0;
    while (f>=10.0)
    {
        f=f/10.0;
        dp++;
    }
    for (ix=1;ix<8;ix++)
    {
            num = (int)f;
            f=f-num;
            if (num>9)
                buf[pos++]='#';
            else
                buf[pos++]='0'+num;
            if (dp==0) buf[pos++]='.';
            f=f*10.0;
            dp--;
    }
    printnum(buf);

}
