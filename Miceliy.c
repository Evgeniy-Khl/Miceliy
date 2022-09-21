/*****************************************************
Project : Мицелий
Version : 3.0.0
Chip type           : ATmega32
Program type        : Application
Clock frequency     : 16 MHz
Program size        :  16312 words (32624 bytes), 99,6% of FLASH [0x9BC7]
*****************************************************/

//#include <mega32a.h>
#include <stdio.h>      // Standard Input/Output functions
#include <spi.h>        // SPI functions
#include <1wire.h>
#include <delay.h>
#include <bcd.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "twi.h"
#include "fonts.h"
#include "tft.h"
#include "9341.h"

//#define USART

/***** used for TIMER0 count initialization *****/
//                              РазмерРегистра-Частота(МГц)/Делитель*Микросекунды
//                              РазмерРегистра-Частота(КГц)/Делитель*Милисекунды
//                              РазмерРегистра-Частота(Гц)/Делитель*Секунды
#define F_XTAL			16000L               //quartz crystal frequency [KHz]
#define INIT_TIMER1     TCNT1 = 0x10000L-F_XTAL/1024L*1000L // 1000 ms
#define INIT_TIMER0     TCNT0 =   0x100L-F_XTAL/1024L*5L    // 5 ms
//*********************************
//#define PRESET_1MC	        252		// (256 - n)*1024*0.2713=1111 us (1 ms)
//#define PRESET_3MC	        245		// (256 - n)*1024*0.2713=3056 us (3 ms)
//#define PRESET_5MC	        178		// (256 - n)*256*0.2713=5000 us (5 ms)
//#define PRESET_20MC	        184		// (256 - n)*1024*0.2713=20002 us (11979 us - 23 байт/Baud rate: 19200)
//#define PRESET_50MC	        75		// (256 - n)*1024*0.2713=50006 us (47917 us - 23 байт/Baud rate: 4800)

#define INIT_TIMER2_50MC    TCNT2 =   0x100L-F_XTAL/1024L*50L    // 50 ms
#define INIT_TIMER2_3MC     TCNT2 =   0x100L-F_XTAL/1024L*3L     // 3 ms
//*********************************
#define FRAMING_ERROR       (1<<FE)
#define PARITY_ERROR        (1<<UPE)
#define DATA_OVERRUN        (1<<DOR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE         (1<<RXC)
#define RX_BUFFER_SIZE	24	// USART Receiver buffer (ADR,COMMAND,ID,DAT1(4),DAT2(5),DAT3(6),DAT4(7),DAT5(8),
                                //DAT6(9),DAT7(10),DAT8(11),DAT9(12),DAT10(13),DAT11(14),DAT12(15),DAT13(16),
                                //DAT14(17),DAT15(18),DAT16(19),DAT17(20),DAT18(21),DAT19(22),CRC1,CRC2)
#define TX_BUFFER_SIZE	32	// USART Transmitter buffer   ??????????????????????????????????????
#define CLOCK_BUFFER	8
//#define SBL		        0x82// Начало блока Flash memory
//#define TOPTIME         250	// задержка на x мин. для заполнения Tf[i]
//#define WAITCOUNT 	    50	// максимальная пауза перед реакцией на кнопку

//#define ADC_VREF_TYPE	0xE3
#define SPOT1	        2	// датчики зоны № 1- психрометрический гигрометр 2 датчика с индикацией температуры и влажности;
#define SPOT2   	    4	// датчики зоны № 2- компост 4 датчика с индикацией средней температуры;
#define SPOT3	        3	// датчик  зоны № 3- наружный воздух,воздух подачи,смешаный воздух 3 датчикa;

#define MAX_DEVICES	(SPOT1+SPOT2)// максимальное количество микросхем DS18b20 подключенных 1 Wire шине
#define ON		1
#define OFF		0
#define ZERO	50

#define SPI_MOUD_FL	    0x51	// SPI Type: Master, Clock Rate: 1000,000 kHz, Clock Phase: Cycle Half, Clock Polarity: Low, Data Order: MSB First
//#define SPI_MOUD_DSP	0xD2
//#define SPIF		SPSR.7
//#define ADIF		ADCSRA.4// флаг завершения преобразования
#define SQWE_32768Hz    0x13
#define SQWE_1Hz        0x10
#define OUT_OFF         0x80

//*********************************
#define SOH		        0xDD	// Начало блока
#define CMD_LINK        81	// Посылка программы
//#define CMD_D_READ      85      // Считать уставки digital
//#define CMD_A_READ      87      // Считать уставки analog
//*********************************
#define KEY_UP		1
#define KEY_RIGHT       2
#define KEY_LEFT        4
#define KEY_L_R         6
#define KEY_DOWN        8
#define KEY_ENTER       16
#define KEY_ESC		    32
#define KEY_EDIT        64
#define KEY_REPORTS 	128
//*********************************
#define dhtport     PORTA.3
#define dhtddr      DDRA.3
#define dhtpin      PINA.3
//*********************************
#define CSDAC1		PORTD.4
#define CSDAC2		PORTD.5
#define CSDAC3		PORTD.6
#define CSDAC4	    PORTD.7

#define LOCK	    PORTB.1
#define BEEP		PORTB.2
#define DE485		PORTB.3
//*********************************
#define ID_CO2          0xF5    // идентификатор блока
#define DATAREAD        0xA1    // Read Scratchpad
#define SIZCOPY         165
#define ADC_VREF_TYPE   0x40

// Declare your global variables here
flash float A1=1.8, A2=0.81, A3=0.01;  // порядок a=0.9 (A1=2a; A2=a^2; A3=(1-a)^2)
flash float B1=1.6, B2=0.64, B3=0.04;  // порядок a=0.8 (A1=2a; A2=a^2; A3=(1-a)^2)
//flash float A1=1.2, A2=0.36, A3=0.16;  // порядок a=0.6 (A1=2a; A2=a^2; A3=(1-a)^2)

//--------- Registr region ----------------------------------------------------
#pragma regalloc-
register char relay @2;/* allocate the variable ‘relay’ to the register pair R2*/
#pragma regalloc+
unsigned char *ptr_to_ram;
unsigned char eeprom *ptr_to_eeprom;
unsigned char counter0, counter1, BeepT;
//****************
unsigned char displ_num, newButton, newSetButt, pointY, error;
signed char numMenu;
unsigned int max_X, max_Y, fillScreen = BLACK;
//******************************************************************************
#define MAX_FAZA        7
const char* setFaza[MAX_FAZA]={"Задания","Динамика","Разрешения","Управление А","Управление D","Просушка","Специальные"}; // ,"Ограничения"
#define MAX_KOFF        8
#ifndef USART
  const char* setKoff[MAX_KOFF]={"Вент.Притока","Заслонка СВ","Вентилятор","Кран Гор.воды","Кран Xол.воды","Коэф D12","Коэф D34","Датчики"};
#else
  const char* setKoff[MAX_KOFF]={"Вент.Притока","Заслонка СВ","Вентилятор","Кран Гор.воды","Рецеркуляция","Коэф D12","Коэф D34","Датчики"};
#endif
#define MAX_FAZA_SET    4
const char* nameFaza[MAX_FAZA_SET]={"воздух","компост","влажность","СО2"};
const char* nameD[4]={"Нагрев","Охлаждение","Увлажнение","Вентилятор"};
const char* nameE0[3]={"Просушка","Повышение Темп","Понижение Влж"};
const char* nameE1[5]={"Задание фазы","Стоп системы","Замораживание","Функция","ID прибора"};
const char* nameE2[4]={"MIN","MAX","kP","kI"};
const char* nameE3[8]={"D1t1","D1t2","D1k1","D1k2","D2t1","D2t2","D2k1","D2k2"};
const char* nameE4[8]={"D3t1","D3t2","D3k1","D3k2","D4t1","D4t2","D4k1","D4k2"};
//******************************************************************************
signed char timerPump[2], timerStop, timerDrying, displCO2;
unsigned char minuts, dayStage, error, devices, signchar, intval, frcval;
unsigned char rom_code[MAX_DEVICES][9], dacU[4]={ZERO,ZERO,ZERO,ZERO}, outU[4], clock_buffer[8], buff[65];
signed int Tf[4], z1[SPOT1]={999,999}, z2[SPOT2]={999,999,999,999}, z3[SPOT3]={999,999,999}, newval[8], pauseButt, pvT, pvRH, pvCO2; // значения датчиков
struct ram_err {int  dry, soil, co2; signed char hum;} errors;
struct ram_flg {char  outU0, outU1, outU3, ratDry, ratSoil, ratRH, ratCO2, ratOld, vent, flap, freez, cold, unevenT;} flags;
/*
-------- Приточный вентилятор. ----------------------
flags.outU0 = 1  - напряжение управления определяется регулятором равномерности температуры компоста в фазе "ПОДГОТОВКА" и "ПОСАДКА"
flags.outU0 = 2  - напряжение управления определяется регулятором содержания СО2
flags.outU0 = 3  - напряжение управления определяется регулятором для понижения температуры
flags.outU0 = 4  - напряжение управления определяется регулятором влажности
flags.outU0 = 5  - напряжение управления определяется положением клапана горячей воды.
flags.outU0 = 6  - напряжение управления определяется положением клапана жолодной воды.
-------- Заслонка свежего воздуха. ----------------------
flags.outU2 = 1  - напряжение управления определяется регулятором содержания СО2
flags.outU2 = 2  - напряжение управления определяется регулятором для понижения температуры
flags.outU2 = 3  - напряжение управления определяется регулятором влажности
-------- Клапана холодной воды. ----------------------
flags.outU3 = 1  - напряжение управления определяется температурой
flags.outU3 = 2  - напряжение управления определяется регулятором влажности
*/
union {unsigned char buffer[8]; unsigned int pvT;} ds;
union {unsigned char buffer[4]; unsigned int val[2];} in;
union {unsigned char buffer[4]; unsigned int val[2];} out;
//-------------------
float iPart[4], Told1[9],Told2[9];

eeprom unsigned char romcodeZ1[SPOT1][7]={{219, 95,196,0,0,0,239},
                                         {69,223, 36,0,0,0,207}}; // зонa № 1- 2 датчика  ПСИХРОМЕТР
eeprom unsigned char romcodeZ2[SPOT2][7]={{219, 95,196,0,0,0,239},
                                         {219, 95,196,0,0,0,239},
                                         {219, 95,196,0,0,0,239},
                                         {219, 95,196,0,0,0,239}};// зонa № 2- 4 датчика  КОМПОСТ
eeprom unsigned char romcodeZ3[SPOT3][7]={{69,223, 36,0,0,0,207}, // z3[0] ПОДАЧИ возд.
                                         {69,223, 36,0,0,0,207},  // z3[1] СМЕШАНЫЙ возд.
                                         {69,223, 36,0,0,0,207}}; // z3[2] НАРУЖНЫЙ возд.

  // Параметры системы -   ФАЗА;  upT   dnRh  antifreez   ID
eeprom unsigned char sys[5]={0,   10,   10,       5,       1};          // (5 байт)
eeprom signed char autoA[4]={ -1, -1, -1, -1};                          // (4 байт)
eeprom signed char autoD[4]={ -1, -1, -1, -1};                          // (4 байт)
eeprom unsigned char spot1=2, spot2=4, spot3=3;                         // (3 байт)
                                                                        //                                                  ВСЕГО (16 байт)
// Основные уставки для каждой ФАЗЫ
eeprom struct {
                signed int Dry,                // температура воздуха
               		       Soil,               // температура компоста
               		       CO2;                // измеритель CO2
             unsigned char RH,                 // отностительная влажность
                           DeltaDry,           // скорость изменения
                           DeltaSoil,          // скорость изменения
                           DeltaRH,            // скорость изменения
                           DeltaCO2,           // скорость изменения
                           Min0,               // ограничение
                           Max0,               // ограничение
                           Min1,               // ограничение
                           Max1,               // ограничение
                           EnableDry,          // разрешения
                           EnableSoil,         // разрешения
                           EnableRH,           // разрешения
                           EnableCO2;          // разрешения
              } faza[5]={ 		   
// dry   soil   CO2   RH  Ddry Dsoil Drh Dco2 Min0 Max0 Min1 Max1 Edry Esoil ERH Eco2
{  214,  214,  3000,  80,   2,   2,   1,  50,  10, 100,   5, 100,   3,   3,   7,  3}, // 0 - ФАЗА I      ПОДГОТОВКА  1-14 день
{  230,  230,  2000,  85,   2,   2,   1,  50,  10, 100,   5, 100,   3,   3,   7,  3}, // 1 - ФАЗА II     ПОСАДКА  15-22 день
{  220,  220,  1800,  90,   2,   2,   1,  50,   0, 100,   0, 100,   0,   3,   0,  0}, // 2 - ФАЗА III    СТОП 23-24 день
{  180,  180,  1600,  95,   2,   2,   1,  50,   0, 100,   0, 100,   3,   3,   7,  3}, // 3 - ФАЗА IV     ШОК 25-29 день
{  180,  180,  1000,  80,   2,   2,   1,  50,   5, 100,   5, 100,   3,   0,   7,  3}};// 4 - ФАЗА V      ПЛОДОНОШЕНИЕ 30-65 день
                                                                                      //                                               ВСЕГО (95 байт)
// Уставка относительной температуры ЦИФРОВЫХ выходов
eeprom struct {unsigned char t1, t2, k1, k2;} digital[4]={
//t1  t2  k1  k2 
{ 10, 5,  8,  4},  // 0 - Насос ГВ (нагрев) t1-> порог вкл.(%); t2-> порог откл.; k1-> задержка вкл./откл.(сек.) k2-> делитель (analog[0].limT/k2)
{ 10, 5,  8,  4},  // 1 - Насос ХВ (охлаж.) t1-> порог вкл.(%); t2-> порог откл.; k1-> задержка вкл./откл.(сек.) k2-> делитель (analog[1].limT/k2)
{  1, 0,100, 30},  // 2 - Клапан увлажнит.  t1-> порог вкл.(%); t2-> коррекция RH;k1-> коэф.интегральн.регулят.  k2-> коэф.каскадного регулят.
                    //                                                                       UpdatePI()                   controlHotBelimo()
{  5,100,  5, 10}}; // 3 - прочее           t1->НОРМАЛ.нерав."T"-компоста; t2-> ошибка по сухому; k1-> неисполь. k2-> неиспользуется;
                    //                                                           controlFreshAir()                                      ВСЕГО (16 байт)
// Уставка относительной температуры АНАЛОГОВЫХ выходов                          controlVent()
eeprom struct {
                        int limCO;        // пороговое значение CO2
               unsigned char limT,        // неравномерность температуры компоста
                            limCl,        // пороговое значение температуры внешнего воздуха используемое для охлаждения
                            limRh,        // пороговое значение RH
                           minVal,        // минимальное значение
                           maxVal,        // максимальное значение
                               kP,        // пропорциональный коэфициент
                               kI;        // интегральный коэфициент
              } analog[4]={
//limCO limT limCl limRh min max  kP   kI 
{ 100,   20,   2,    3,   0, 100, 20, 100}, // 0 вент.Приточ.возд. limT->неравн. Т компоста только для faza[0] и faza[1]
                                            //                     limCl->порог начала охлаждения; limRh->порог начала осушения
{ 100,   30,   2,    3,   0, 100, 20, 100}, // 1 "СВЕЖЕГО ВОЗДУХА" limT->разн.темп.наруж. и внутр.воздуха
                                            //                     limCl->порог начала охлаждения; limRh->порог начала осушения
{  70,   10,   0,    0,   0, 100, 20, 100}, // 2 кран "BELIMO" ГВ. limCO->ВКЛ.вент.Вытяжной(%); limT->ГВ-порог(%) вкл.вент.Приточ.возд.;
                                            //                     limCl->нет; limRh->нет
{  60,   10,  30,    6,   0, 100, 20, 100}};// 3 кран "BELIMO" ХВ. limCO->ОТКЛ.вент.Вытяжной(%);limT->ХВ-порог(%) вкл.вент.Приточ.возд.;
                                            //                     limCl->увел.при охл.+осуш.; limRh->порог начала осушения
                                            //                                                                                         ВСЕГО (36 байт)                                            
                                                           // dry   soil   CO   RH
eeprom struct eep_sp  {float dry, soil, co2, rh;} setpoint={  214,  214,  3000,  80}; // X - ФАЗА      //                                                                                   ИТОГО EEPROM(174 байт)

//----------------------------------Таблица влажности ------------------------------------------------------
eeprom unsigned char tabRH[420]={
95,90,86,81,77,72,68,64,60,56,52,48,44,40,36,32,29,25,22,18,
95,91,86,82,77,73,69,65,61,57,53,49,45,42,38,34,31,27,24,20,
95,91,87,82,78,74,70,66,62,58,54,50,47,43,40,36,33,29,26,23,
96,91,87,83,79,75,71,67,63,59,55,52,48,45,41,38,34,31,28,25,
96,91,87,83,79,75,71,67,64,60,56,53,49,46,43,39,36,33,30,26,
96,92,88,84,80,76,72,68,65,61,57,54,51,47,44,41,37,34,31,28,
96,92,88,84,80,76,73,69,65,62,58,55,52,48,45,42,39,36,33,30,
96,92,88,84,80,77,73,70,66,63,59,56,53,50,46,43,40,37,34,32,
96,92,88,85,81,77,74,70,67,64,60,57,54,51,48,45,42,39,36,33,
96,92,89,85,81,78,74,71,68,64,61,58,55,52,49,46,43,40,37,35,
96,92,89,85,82,78,75,71,68,65,62,59,56,53,50,47,44,41,39,36,
96,93,89,85,82,79,75,72,69,66,63,60,57,54,51,48,45,42,40,37,
96,93,89,86,82,79,76,73,69,66,63,60,57,55,52,49,46,44,41,38,
96,93,89,86,83,79,76,73,70,67,64,61,58,55,53,50,47,45,42,40,
96,93,90,86,83,80,77,74,71,68,65,62,59,56,54,51,48,46,43,41,
97,93,90,87,83,80,77,74,71,68,65,62,60,57,54,52,49,47,44,42,
97,93,90,87,84,80,77,74,72,69,66,63,60,58,55,53,50,48,45,43,
97,93,90,87,84,81,78,75,72,69,66,64,61,58,56,53,51,48,46,44,
97,93,90,87,84,81,78,75,72,70,67,64,62,59,57,54,52,49,47,45,
97,94,90,87,84,81,79,76,73,70,67,65,62,60,57,55,52,50,48,46,
97,94,91,88,85,82,79,76,73,71,68,65,63,60,58,56,53,51,49,46
};

eeprom unsigned char copyEEPROM[SIZCOPY]={255}; //  область сохранения заводских установок
//eeprom unsigned char idxClcPnt=0;
//#define INITROW	40
//eeprom struct {
//               unsigned int workHour;              // отработано часов
//               unsigned char workMin,              // отработано минут
//                             resFlag;              // вектор сброса
//              } initPoint[INITROW];
//----------------------------------------------------------------------------------------------------------
//--- R2 region ---relay -------------
bit CAN1;               // Управление насосом горячей воды (нагревом)
bit CAN2;               // Управление насосом холодной воды (охлаждением)
bit CAN3;               // Управление клапаном увлажнителя
bit CAN4;               // Управление вытяжным вентилятором.
//--- R3 region ----------------------
bit LcdDispl;		    // отобразить информацию
bit NewCalc;		    // новый расчет
bit NewMin;             // новая минута
bit Dht;                // подключен DHT-22
bit CO2module;          // подключен измеритель СО2
bit ChangeFaza;         // плавное изменение задания фазы

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)  // интервал 5 mS отсчитывает текущее время
{
  INIT_TIMER0;
  if (BeepT) {--BeepT; BEEP = ON;}		// включить серену  1
  else BEEP = OFF;			            // отключить серену 0      конец  1,9-2,44 us
}

// Timer 1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{
  INIT_TIMER1;                                 // 1 sek.
  NewCalc=1;
  ++counter0;
  if(++counter1>59){counter1=0; NewMin=1;}
}
#ifdef USART
// Timer 2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)// интервал 10 mS
{
  rx_wr_index = 0;	// обнулим счетчик буфера приема
  DE485 = OFF;             // прямое управление
  TIMSK &= 0xBF;	         // Timer2 - OFF;
}
#endif
/*
// ADC interrupt service routine
interrupt [ADC_INT] void adc_isr(void)
{
  unsigned char adc_data;
  adc_data=ADCH;	// Read the 8 most significant bits of the AD conversion result
}
// External Interrupt 0 service routine
interrupt [EXT_INT0] void ext_int0_isr(void)
{

}
// External Interrupt 1 service routine
interrupt [EXT_INT1] void ext_int1_isr(void)
{

}
*/
#include "twi.c"
#include "tft.c"
#include "9341.c"
#include "buttons.c"
#include "procedure.c"
#include "touchpad.c"
#include "control.c"
#include "displayTFT.c"

void main(void)
{
// Declare your local variables here
unsigned char i, x;
signed int crc;
#include "init.c"

while (1){
//  if (rx_buffer_overflow) check_command(); // если установлен флаг - "комманда компьютера"
  if(counter0>5) {counter0=0; LcdDispl=1;};   // основной счетчик интервалов (НОВАЯ СЕКУНДА !!)
  if(NewCalc){
//     #asm("wdr")                              // сброс сторожевого таймера
     NewCalc=0;
     if(displ_num==0 && error) BeepT = 100;     // Тревога
     error = 0; x = 0;
     for (i=0; i<4; i++) if (autoA[i]>=0) x++;// проверка - все ли находится в автоматическом режиме
     for (i=0; i<6; i++) if (autoD[i]>=0) x++;// проверка - все ли находится в автоматическом режиме
     for (i=1; i<4; i++) temperature_check(i);// если линия Ок то измеряем температуру SPOT1, SPOT2, SPOT3
//     if(waitset) waitset--; else {strnum=0; /*displ_num=0; newSetButt=1;*/ edit=0; reports=0; waitkey=WAITCOUNT;}// displnum=0;
//------ Цифровые фильтры ----------------------------------------------------------------------
     x = 1;
     if(z1[0]<850) Tf[0]= lowPF2a(z1[0],0); else {Tf[0]=999; x = 0;}
     if(z1[1]<850) Tf[1]= lowPF2a(z1[1],1); else {Tf[1]=999; x = 0;}
     //-- T компоста ---------------------------
     crc = mean(&z2[0]);                      // среднее значение датчиков компоста
     if(crc<999) Tf[2] =  lowPF2a(crc,2);     // текущая температура компоста
     else Tf[2] = 999;
     //-- RH -----------------------------------
     if(Dht) // есть электронный датчик
      {
        x = readDHT();
        if(x)
         {
           crc = abs(Tf[0]-pvT); if(crc>10) flags.unevenT=0x23; else flags.unevenT=0x20;// указатель отличия значений датчиков
           crc = lowPF2a(pvRH,4);
           pvRH = crc/10;
           pvRH +=digital[2].t2; // коррекция датчика влажности
         }
        else pvRH = 199;// Ошибка датчикa DHT22
      }
      else if(x) pvRH = tableRH(Tf[0],Tf[1]); else pvRH=255; // психрометрический датчик  если ошибка pvRH = 255
      pvRH = 85; // !!!!!!!
     //-- CO2 ----------------------------------
     if(CO2module) x = readCO2();     // если модуль подключен то обмениваемся командами и обновляем только pvCO2
     if(x) Tf[3] = lowPF2a(pvCO2,3);// если идет измерение СО2 то обновляем Tf[3]
// ------ УПРАВЛЕНИЕ ---------------------------------------------------------------------------
     if(timerStop){for (i=0;i<4;i++) outU[i]=0; relay=0;}  // ПОЛНАЯ ОСТАНОВКА
     else
      {
       calculationErrors();             // Расчет ошибок регулирования.
       flags.vent = controlVent();      // Определение производительности приточного вентилятора.
       flags.flap = controlFreshAir();  // Определения положения заслонки свежего воздуха.
       flags.freez = controlHotBelimo();// Угроза замораживания!
#ifndef USART
       flags.cold = controlColdBelimo();// Положение клапана холодной воды.
#else
       controlRecycling();              // Определения положения заслонки рециркуляции. 
#endif
       controlHumidifierValve();        // Клапан увлажнителя.
       limitationOut();                 // ограничение управляющего сигнала
       controlVentOutlet();             // Работа вытяжного вентилятора
      }
     for (i=0; i<4; i++) {if(autoA[i]>=0) outU[i] = autoA[i]; dacU[i] = adapt(outU[i]);}// конверсия для ЦАП
//------- Проверка на ручное управление --------------------------------------------------------
     for (i=0; i<4; i++)
      {
        if(autoD[i]==1)
         {
           switch (i)
            {
              case 0: CAN1=ON; break;                   // управление цифровой выход № 1
              case 1: CAN2=ON; break;                   // управление цифровой выход № 2
              case 2: CAN3=ON; break;                   // управления цифровой выход № 3 
              case 3: CAN4=ON; break;                   // управления цифровой выход № 4
            }; 
         };
        if(autoD[i]==0)
         {
           switch (i)
            {
              case 0: CAN1=OFF; break;                   // управление цифровой выход № 1
              case 1: CAN2=OFF; break;                   // управление цифровой выход № 2
              case 2: CAN3=OFF; break;                   // управления цифровой выход № 3 
              case 3: CAN4=OFF; break;                   // управления цифровой выход № 4
            }; 
         };
      };
//-----------------------------------------------------------------------------------------------
     LOCK = OFF;
     PORTC = relay<<2;                   // подать напряжение на цифровые выходы
     LOCK = ON;
     setDAC();                           // подать напряжение на аналоговые выходы
     if(ChangeFaza) setSP(); // плавное изменение задания фазы!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  };
  if(LcdDispl){LcdDispl=0; display();}
//****************************** НОВАЯ МИНУТА !! ************************************************
  if(NewMin){
     NewMin = 0;
     if(timerStop) timerStop--;     // режим ПОЛНЫЙ ОСТАНОВ
     if(timerDrying) timerDrying--; // режим ПРОСУШКА
     dayStage = (clock_buffer[4]-1)+31*(clock_buffer[5]-1); // вычисляем текущий день!
//------------------------------ НОВЫЙ ЧАС !! ---------------------------------------------------
     if(++minuts>59)
      {
       minuts=0; 
//       initPoint[idxClcPnt].workHour++; initPoint[idxClcPnt].workMin=minuts;// число часов работы
       if(ChangeFaza) setSP(); // плавное изменение задания фазы!
      }
//     else initPoint[idxClcPnt].workMin=minuts;
  };
//***********************************************************************************************
  if(newButton==100){        // нажатие не обнаружено
     if(checkTouch()){       // проверить снова
       x = (checkButton(buttonCount));
       if (x < buttonCount) touchpad(x);
       display();
     }
  }
  else {
    touchpad(newButton);
    newButton=100;            // нажатие отработано 
    display();
  }  
 };
}
