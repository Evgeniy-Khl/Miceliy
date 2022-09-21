
unsigned char adapt(unsigned char n)
{
  if (n>100) n=100;
  n <<= 1; n += ZERO;
  return n;
}

void setSP()
{
signed char rating, count=0;
float ftemp;
   //--  DeltaDry  -----------------------------------------------------------------------------------
   rating = sign(faza[sys[0]].Dry-(int)setpoint.dry);
   if(rating>0) flags.ratDry = 0xD9; else if(rating<0) flags.ratDry = 0xDA; else flags.ratDry = 0x20;
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaDry/10 * rating;
     setpoint.dry = setpoint.dry + ftemp; count++;
    }
   //--  DeltaSoil  ----------------------------------------------------------------------------------
   rating = sign(faza[sys[0]].Soil-(int)setpoint.soil);
   if(rating>0) flags.ratSoil = 0xD9; else if(rating<0) flags.ratSoil = 0xDA; else flags.ratSoil = 0x20;
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaSoil/10 * rating;
     setpoint.soil = setpoint.soil + ftemp; count++;
    }
   //--  DeltaRH  ------------------------------------------------------------------------------------
   rating = sign(faza[sys[0]].RH-(int)setpoint.rh);
   if(rating>0) flags.ratRH = 0xD9; else if(rating<0) flags.ratRH = 0xDA; else flags.ratRH = 0x20;
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaRH/10 * rating;
     setpoint.rh = setpoint.rh + ftemp; count++;
    }
   //--  DeltaCO2  -----------------------------------------------------------------------------------
   rating = sign(faza[sys[0]].CO2-(int)setpoint.co2);
   if(rating>0){flags.ratOld = flags.ratCO2 = 0xD9;} else if(rating<0){flags.ratOld = flags.ratCO2 = 0xDA;} else {flags.ratOld = flags.ratCO2 = 0x20;}
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaCO2 * rating;
     setpoint.co2 = setpoint.co2 + ftemp; count++;
    }
   if(count==0) ChangeFaza = 0;
}

void checksign(signed int t)
{
  if (t<0) {t = -t; signchar = '-';} else signchar = ' ';
  tDry = t%10; tDryInt = t/10;
}

#define COUNT    500
unsigned char module_check(unsigned char fc)
{
 unsigned char *p, try, byte;
  p = outbuffer;
  outbuffer[3]=w1_dow_crc8(p,3);// контрольная сумма
  for (try=0; try<3; try++)
   {
    w1_init();            // 1 Wire Bus initialization
    w1_write(fc);         // Family code
    p = outbuffer;
    for (byte=0; byte<4; byte++) w1_write(*p++);
    delay_us(COUNT);      // ожидаем пока модуль обработает информацию
    p = inbuffer;
    for (byte=0; byte<4; byte++) *p++ = w1_readnew();// Read 4 byte
    byte = w1_readnew();  // Read CRC byte #5
    p = inbuffer;
    if(byte==w1_dow_crc8(p,4)) break;
   };
  if(try>2) byte=0; else byte=1;
  return byte;
}

unsigned char readCO2(void) // чтение модуля СО2
{
 unsigned char byte;
 static unsigned char try;
 unsigned int *prt_int;
  outbuffer[0]=DATAREAD;       // Function Command
  outbuffer[1]=0x00;           // Data 1
  outbuffer[2]=displCO2;       // Data 2 1->компрессор отключен; 2->подготовка к замеру; 3->выполнить замер;
  byte = module_check(ID_CO2); // идентификатор блока
  if(byte)                     // если блок ответил ...
   {
     prt_int = &inbuffer[2]; pvCO2 = *prt_int;// CO2
     try = 0;
   }
  else if(++try>5){pvCO2=0; error|=0x08;}// Отказ модуля CO2
  return byte;
}

unsigned char startDHT(void)
{
   unsigned char flag=0;
   dhtport = 0;
   dhtddr = 1;   // MCU pull down
   delay_ms(10); // MCU Sends out Start Signal to DHT and pull down voltage for at least 18ms to let DHT21 detect the signal.
   dhtddr = 0;   // MCU pull up
   delay_us(60); // wait for DHT respond 20-40uS
   if(!dhtpin)
   {
      while(!dhtpin) {flag++;} // low-voltage-level response signal & keeps it for 80us
      if(flag<10) return 0;
      else {flag=0; while(dhtpin) {flag++;}}  // hi-voltage-level response signal & keeps it for 80us
      if(flag<10) return 0;
      else return 1;
   }
   else return 0;
}

unsigned char readDHT(void)
{
 unsigned char i, j, flag=0, tem[5];
 int val;
 static unsigned char err;
 if(startDHT())
  {
    for(i=0; i<5; i++)
     {
       tem[i]=0;
       for(j=0; j<8; j++)             
        {
          tem[i]<<= 1;
          delay_us(30);   // When DHT is sending data to MCU, every bit of data begins with the 50us low-voltage-level
                          // and the length of the following high-voltage-level signal determines whether data bit is "0" or "1"         
          while(!dhtpin) {flag++;}// ожидаем фронт сигнала
          delay_us(32);   //26-28u voltage-length means data "0" / 70u voltage-length means data "1"
          flag=0;
          while(dhtpin) {flag++;}// ожидаем спад сигнала
          if(flag>10) tem[i]|= 1;// data "1"
        }
     }
    flag=tem[0]+tem[1]+tem[2]+tem[3];
    if(flag==tem[4])
     {
      val =(int)(tem[0]*256+tem[1]);
      if(val>1000) val=1000; else if(val<0) val=0;
      pvRH = val;
      pvT =(int)tem[2]*256+tem[3];
      err = 0;
      return 1;
     }
    else if(++err>3) return 0;  // НЕ верная CRC датчика влажности
  }
 else return 0;                 // потерян датчик влажности
}

unsigned char humidity(int dryT, int humT)
{
   unsigned char humData;
   int addr;
   if(dryT>0 & humT>0) addr = dryT-humT;  else addr = 0xFFFF;
   if(addr<0) addr=0;
   if (dryT<639 & addr<159)// Тсух=<63.9 и Тдельта=<15.9
    {
      addr >>= 1;
      addr <<= 9;
      dryT >>= 1;
      addr |= dryT;
      humData = read_AT24C512(addr);
    }
   else humData=255;
   return humData;
}

unsigned char temperature_check(unsigned char zone)
{
unsigned char item, byte, devices, crc, try=0, count=0, errval, ds_buffer[8];
unsigned char *p;
unsigned int *prt_int1, *prt_int2;
  switch (zone)
      {
       case 1: devices = spot1; break;
       case 2: devices = spot2; break;
       case 3: devices = spot3; break;
      }
  if (devices)
   {
    for (item=0; item < devices;)
     {
     switch (zone)
       {
        case 1: {ptr_to_eeprom = &romcodeZ1[item,0]; prt_int2 = &z1[item]; errval=0x01;} break;
        case 2: {ptr_to_eeprom = &romcodeZ2[item,0]; prt_int2 = &z2[item]; errval=0x02;} break;
        case 3: {ptr_to_eeprom = &romcodeZ3[item,0]; prt_int2 = &z3[item]; errval=0x04;} break;
       };
     w1_init(); // 1 Wire Bus initialization
     w1_write(0x55); // Load MATCH ROM [55H] comand
     w1_write(0x28); // Load family code [28H]
     for (byte=0; byte < 7; byte++) w1_write(*ptr_to_eeprom++); // Load cont. byte
     p = ds_buffer;
     w1_write(0xBE); // Read Scratchpad command [BE]
     for (byte=0; byte < 8; byte++) *p++ = w1_readnew(); // Read cont. byte
     crc = w1_readnew(); // Read CRC byte
     p = ds_buffer;
     if (w1_dow_crc8(p, 8)==crc) {try = 0; prt_int1 = ds_buffer; *prt_int2 = *prt_int1*10/16; count++;}  // !!!!!!!!!!!!!!!!а если отрицательная ???????????
     else if (++try > 2) {*prt_int2 = 999; try = 0; error |= errval;}// (99.9) если ошибка более X раз то больше не опрашиваем
     if (try==0) item++;
     };
   }
  else
   {
     switch (zone)
       {
        case 1: for (item=0; item<SPOT1; item++) z1[item]=999; break;    // (99.9)
        case 2: z2[0]=999; break;                                        // (99.9)
       };
   };
  w1_init(); // 1 Wire Bus initialization
  w1_write(0xCC); // Load Skip ROM [CCH] command
  w1_write(0x44); // Load Convert T [44H] command
  return count;
}
/*
USART Receiver buffer
(ADR,COMMAND,ID,DAT1(4),DAT2(5),DAT3(6),DAT4(7),DAT5(8),DAT6(9),DAT7(10),DAT8(11),DAT9(12),DAT10(13),
DAT11(14),DAT12(15),DAT13(16),DAT14(17),DAT15(18),DAT16(19),DAT17(20),DAT18(21),DAT19(22),CRC1,CRC2)
*/
void check_command(void)
{
   unsigned char i;
   unsigned int crc=0;
   unsigned int *ptr_int;
   rx_buffer_overflow=0;
   if (rx_buffer[0] == sys[0] || rx_buffer[0] == SOH)// Начало блока
    {
      for (i=0; i<RX_BUFFER_SIZE-2; i++) crc+=rx_buffer[i];
      ptr_int = &rx_buffer[RX_BUFFER_SIZE-2];     // считываем CRC
      if (crc == *ptr_int)
       {
        switch (rx_buffer[1])
        {
          case CMD_LINK:     // Запрос данных компьютером 
               crc = 0;
               ptr_to_ram = Tf; // Tf[4], z1[SPOT1], z2[SPOT2], z3[SPOT3], pvT, pvCO2;
               for (i=0; i<30; i++) {crc += *ptr_to_ram; putchar(*ptr_to_ram++);};                 // (30 байт) 
               //--- Управляющие сигналы ----------------------------------------------------------------------
               ptr_to_ram = outU;
               for (i=0; i<6; i++) {crc += *ptr_to_ram; putchar(*ptr_to_ram++);};                  //  (6 байт)
               //--- Текущие SP -------------------------------------------------------------------------------
               ptr_to_ram = ptr_int;
               *ptr_int = setpoint.dry;  crc += *ptr_to_ram; putchar(*ptr_to_ram++); crc += *ptr_to_ram; putchar(*ptr_to_ram--);
               *ptr_int = setpoint.soil; crc += *ptr_to_ram; putchar(*ptr_to_ram++); crc += *ptr_to_ram; putchar(*ptr_to_ram--);
               *ptr_int = setpoint.co2;  crc += *ptr_to_ram; putchar(*ptr_to_ram++); crc += *ptr_to_ram; putchar(*ptr_to_ram--);
               *ptr_int = setpoint.rh;   crc += *ptr_to_ram; putchar(*ptr_to_ram++); crc += *ptr_to_ram; putchar(*ptr_to_ram--);
                                                                                                   //  (8 байт)
               //--- Параметры системы ------------------------------------------------------------------------
               ptr_to_eeprom = sys;
               for (i=0; i<20; i++) {crc += *ptr_to_eeprom; putchar(*ptr_to_eeprom++);};           // (20 байт)
               //--- Прочая информация ------------------------------------------------------------------------
               crc+= leddispl; putchar(leddispl);                  // состояния индикаторов на дисплее (1 байт)
               crc+= error;    putchar(error);                     // передача текущих ошибок          (1 байт)
               for (i=0; i<2; i++) putchar(0);                     // передача резервных байт          (2 байт)
               ptr_to_ram = &crc;
               putchar(*ptr_to_ram++);	                           // перадача контрольной суммы
               putchar(*ptr_to_ram);	                           // перадача контрольной суммы       (2 байт) ВСЕГО 70 байт
          break;
        };
       };
    };
   i=UDR;                                               // сброс флага завершения приема
   UCSRB=0xD8;                                          // USART Receiver: On Transmitter: On Interrupt receiv: On
}

void setDAC(char n) // V = Vref x (255/256)
{
unsigned char i;
  for (i=0; i<2; i++)
    {
     switch (n)// enable DAC 1/2
       {
        case 0: CSDAC1 = 0; break;
        case 2: CSDAC2 = 0; break;
        case 4: CSDAC3 = 0; break;
       }
     SPCR = SPI_MOUD_FL;// SPI port enabled
     SPDR = 0x21+i;     // Load Register to DAC A/B
     while (!SPIF);     // ожидаем конца передачи по SPI Opcode
     #pragma warn-
     SPDR = dacU[n+i];
     #pragma warn+
     while (!SPIF);     // ожидаем конца передачи по SPI Opcode
     PORTB|=0x0E;       // INSTRUCTION EXECUTED
    };
  SPDR = 0;             // УБИРАЕМ МУСОР ИЗ TPIC6B595
  while (!SPIF);        // ожидаем конца передачи по SPI 0
  SPCR = 0;             //disable SPI
}

signed int mean(signed int *array)
{
unsigned char i, x=0;
signed int t=0;
 for (i=0; i<spot1; i++)
  {
    if(*array < 850){t += *array; x++;} else error |= 0x02;// 0x01 - ошибка датчиков среднего значения внутр. воздуха
    array++;
  };
 if(x) t /= x; else t = 999;// записываем 99.9 C
 return t;
}

signed int LowPassF2(signed int t,unsigned char i)
{
float val;
  val = A1*Told1[i]-A2*Told2[i]+A3*t;
  Told2[i] = Told1[i];
  Told1[i] = val;
  return val;
}