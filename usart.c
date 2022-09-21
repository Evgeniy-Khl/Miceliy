/*****************************************************
This program was produced by the
CodeWizardAVR V2.05.0 Professional
Automatic Program Generator
© Copyright 1998-2010 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 23.10.2020
Author  : 
Company : 
Comments: 


Chip type               : ATmega32
Program type            : Application
AVR Core Clock frequency: 16,000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*****************************************************/

#include <mega32.h>
#include <stdio.h>

#define SPOT1	        2	// датчики зоны № 1- психрометрический гигрометр 2 датчика с индикацией температуры и влажности;
#define SPOT2   	    4	// датчики зоны № 2- компост 4 датчика с индикацией средней температуры;
#define SPOT3	        3	// датчик  зоны № 3- наружный воздух,воздух подачи,смешаный воздух 3 датчикa;
#define RX_BUFFER_SIZE 12
#define TX_BUFFER_SIZE 32

#ifndef RXB8
#define RXB8 1
#endif

#ifndef TXB8
#define TXB8 0
#endif

#ifndef UPE
#define UPE 2
#endif

#ifndef DOR
#define DOR 3
#endif

#ifndef FE
#define FE 4
#endif

#ifndef UDRE
#define UDRE 5
#endif

#ifndef RXC
#define RXC 7
#endif

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)

//*********************************
#define SOH		        0xDD	// [221] Начало блока
#define CMD_LINK        0x31    // [81] Посылка программы
//#define CMD_D_READ      85      // Считать уставки digital
//#define CMD_A_READ      87      // Считать уставки analog
//*********************************
//------------------------------- CRC-16 -------------------------------------------
/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned int Crc16_update (unsigned int crc, unsigned char data)
{
  unsigned char i;
        crc ^= data << 8;
        for (i = 0; i < 8; i++) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;

    return crc;
}

/*
  Name  : CRC-16 CCITT
  Poly  : 0x1021    x^16 + x^12 + x^5 + 1
  Init  : 0xFFFF
  Revert: false
  XorOut: 0x0000
  Check : 0x29B1 ("123456789")
  MaxLen: 4095 байт (32767 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned int Crc16(unsigned char *pcBlock, unsigned char len)
{
    unsigned int crc = 0xFFFF;
    unsigned char i;

    while (len--)
    {
        crc ^= *pcBlock++ << 8;

        for (i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

//------------------------------- USART Receiver buffer -------------------------------------------
char rx_buffer[RX_BUFFER_SIZE];

#if RX_BUFFER_SIZE <= 256
unsigned char rx_wr_index,rx_rd_index,rx_counter;
#else
unsigned int rx_wr_index,rx_rd_index,rx_counter;
#endif

// This flag is set on USART Receiver buffer overflow
bit rx_buffer_overflow;

// USART Receiver interrupt service routine
interrupt [USART_RXC] void usart_rx_isr(void)
{
char status,data;
status=UCSRA;
data=UDR;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   rx_buffer[rx_wr_index++]=data;
#if RX_BUFFER_SIZE == 256
   // special case for receiver buffer size=256
   if (++rx_counter == 0)
      {
#else
   if (rx_wr_index == RX_BUFFER_SIZE) rx_wr_index=0;
   if (++rx_counter == RX_BUFFER_SIZE)
      {
      rx_counter=0;
#endif
      rx_buffer_overflow=1;
      }
   }
}

#ifndef _DEBUG_TERMINAL_IO_
// Get a character from the USART Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
char getchar(void)
{
char data;
while (rx_counter==0);
data=rx_buffer[rx_rd_index++];
#if RX_BUFFER_SIZE != 256
if (rx_rd_index == RX_BUFFER_SIZE) rx_rd_index=0;
#endif
#asm("cli")
--rx_counter;
#asm("sei")
return data;
}
#pragma used-
#endif

//------------------------------- USART Transmitter buffer ----------------------------------------
char tx_buffer[TX_BUFFER_SIZE];

#if TX_BUFFER_SIZE <= 256
unsigned char tx_wr_index,tx_rd_index,tx_counter;
#else
unsigned int tx_wr_index,tx_rd_index,tx_counter;
#endif

// USART Transmitter interrupt service routine
interrupt [USART_TXC] void usart_tx_isr(void)
{
if (tx_counter)
   {
   --tx_counter;
   UDR=tx_buffer[tx_rd_index++];
#if TX_BUFFER_SIZE != 256
   if (tx_rd_index == TX_BUFFER_SIZE) tx_rd_index=0;
#endif
   }
}

#ifndef _DEBUG_TERMINAL_IO_
// Write a character to the USART Transmitter buffer
#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar(char c)
{
while (tx_counter == TX_BUFFER_SIZE);
#asm("cli")
if (tx_counter || ((UCSRA & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer[tx_wr_index++]=c;
#if TX_BUFFER_SIZE != 256
   if (tx_wr_index == TX_BUFFER_SIZE) tx_wr_index=0;
#endif
   ++tx_counter;
   }
else
   UDR=c;
#asm("sei")
}
#pragma used-
#endif

// Declare your global variables here
unsigned int crcint;
eeprom unsigned char sys[5]={0,   10,   10,       5,       1};  // (6 байт)

void putint(unsigned int val){
 union {unsigned char buff[2]; unsigned int val;} crc;
  crc.val = val;
  putchar(crc.buff[0]); crcint = Crc16_update(crcint, crc.buff[0]);
  putchar(crc.buff[1]); crcint = Crc16_update(crcint, crc.buff[1]);
}

void check_command(void){
 unsigned int *p_int;
   rx_buffer_overflow=0;
   if (rx_buffer[0] == sys[0] || rx_buffer[0] == SOH)// Начало блока
    {
      crcint = Crc16(&rx_buffer[1], RX_BUFFER_SIZE-3);
      p_int = (unsigned int*)&rx_buffer[RX_BUFFER_SIZE-2];
      if (crcint == *p_int)
       {
        switch (rx_buffer[1])
        {
          case CMD_LINK:     // Запрос данных компьютером 
               putchar(SOH);
               crcint = 0xFFFF;
               putint(0x3231);//Tf[0] - Воздух
               putint(0x3433);//pvRH  - Влажность
               putint(0x3635);//Tf[2] - среднее значение датчиков компоста
               putint(0x3837);//z2[SPOT2] - датчики компоста               
               putchar(0x39); crcint = Crc16_update(crcint, 0x39);               
               putint(crcint);
               /*
               [221] Начало блока                             [1]
               Tf[0] - Воздух                                 [2]
               pvRH  - Влажность                              [2]
               Tf[2] - среднее значение датчиков компоста     [2]
               z2[SPOT2] - датчики компоста                   [8]
               z3[SPOT3] - датчики ПОДАЧИ, СМЕШАНЫЙ, НАРУЖНЫЙ [6]
               Tf[3] - CO2,ppm                                [2] (23 байт)
               ------------------ ВЫХОДЫ УПРАВЛЕНИЯ АНАЛОГОВЫЕ -----------
               outU[0] - Вент.Притока                         [1]
               outU[1] - Заслонка СВ                          [1]
               outU[2] - Кран Гор.воды                        [1]
               outU[3] - Кран Xол.воды или Рецеркуляция       [1] ( 4 байт)
               ------------------ ВЫХОДЫ УПРАВЛЕНИЯ АНАЛОГОВЫЕ -----------
               relay&0x0F - "Нагрев","Охлаждение","Увлажнение","Вентилятор" [1]
               sys[0] - ФАЗА                                                [1]
               
               CRC                                                          [2]  ( 4 байт)
                                                            ВСЕГО:               (31 байт)
               */
          break;
        };
       };
    };
//   i=UDR;                                               // сброс флага завершения приема
   UCSRB=0xD8;                                          // USART Receiver: On Transmitter: On Interrupt receiv: On
}


void main(void)
{
// Declare your local variables here
char i;
// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 9600
UCSRA=0x00;
UCSRB=0xD8;
UCSRC=0x86;
UBRRH=0x00;
UBRRL=0x67;

// Analog Comparator initialization
ACSR=0x80; // Analog Comparator: Off;  Analog Comparator Input Capture by Timer/Counter 1: Off
SFIOR=0x00;

// Global enable interrupts
#asm("sei")
rx_buffer[0] = SOH;
for (i=1;i<10;i++) rx_buffer[i]=i+48;    // !!!!!!!!!!!!!
rx_buffer[10] = 0xB1;
rx_buffer[11] = 0x29;

 while (1){
      check_command();
    
 }
}
