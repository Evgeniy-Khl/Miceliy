#define DIVIDER 10

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
   if(rating) flags.ratDry = 0x21; else flags.ratDry = 0x20;
//   if(rating>0) flags.ratDry = 0x3C; else if(rating<0) flags.ratDry = 0x3E; else flags.ratDry = 0x20;
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaDry/DIVIDER * rating;
     setpoint.dry = setpoint.dry + ftemp; count++;
    }
   //--  DeltaSoil  ----------------------------------------------------------------------------------
   rating = sign(faza[sys[0]].Soil-(int)setpoint.soil);
   if(rating) flags.ratSoil = 0x21; else flags.ratSoil = 0x20;
//   if(rating>0) flags.ratSoil = 0x3C; else if(rating<0) flags.ratSoil = 0x3E; else flags.ratSoil = 0x20;
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaSoil/DIVIDER * rating;
     setpoint.soil = setpoint.soil + ftemp; count++;
    }
   //--  DeltaRH  ------------------------------------------------------------------------------------
   rating = sign(faza[sys[0]].RH-(int)setpoint.rh);
   if(rating) flags.ratRH = 0x21; else flags.ratRH = 0x20;
//   if(rating>0) flags.ratRH = 0x3C; else if(rating<0) flags.ratRH = 0x3E; else flags.ratRH = 0x20;
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaRH/DIVIDER * rating;
     setpoint.rh = setpoint.rh + ftemp; count++;
    }
   //--  DeltaCO2  -----------------------------------------------------------------------------------
   rating = sign(faza[sys[0]].CO2-(int)setpoint.co2);
   if(rating){flags.ratOld = flags.ratCO2 = 0x21;} else {flags.ratOld = flags.ratCO2 = 0x20;}
//   if(rating>0){flags.ratOld = flags.ratCO2 = 0x3C;} else if(rating<0){flags.ratOld = flags.ratCO2 = 0x3E;} else {flags.ratOld = flags.ratCO2 = 0x20;}
   if(rating)
    {
     ftemp = (float)faza[sys[0]].DeltaCO2 * rating;
     setpoint.co2 = setpoint.co2 + ftemp; count++;
    }
   if(count==0) ChangeFaza = 0;
}

#define COUNT    500
unsigned char module_check(unsigned char fc)
{
 unsigned char *p, try, byte;
  p = out.buffer;
  out.buffer[3]=w1_dow_crc8(p,3);// контрольная сумма
  for (try=0; try<3; try++)
   {
    w1_init();            // 1 Wire Bus initialization
    w1_write(fc);         // Family code
    p = out.buffer;
    for (byte=0; byte<4; byte++) w1_write(*p++);
    delay_us(COUNT);      // ожидаем пока модуль обработает информацию
    p = in.buffer;
    for (byte=0; byte<4; byte++) *p++ = w1_read();// Read 4 byte
    byte = w1_read();  // Read CRC byte #5
    p = in.buffer;
    if(byte==w1_dow_crc8(p,4)) break;
   };
  if(try>2) byte=0; else byte=1;
  return byte;
}

unsigned char readCO2(void) // чтение модуля СО2
{
 unsigned char byte;
 static unsigned char try;
  out.buffer[0]=DATAREAD;       // Function Command
  out.buffer[1]=0x00;           // Data 1
  out.buffer[2]=displCO2;       // Data 2 1->компрессор отключен; 2->подготовка к замеру; 3->выполнить замер;
  byte = module_check(ID_CO2); // идентификатор блока
  if(byte)                     // если блок ответил ...
   {
     pvCO2 = in.val[1];// CO2
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

unsigned char tableRH(signed int maxT, signed int minT)
 {
  signed int dT;
   if (maxT>199 && maxT<410) // maxT> 19.9 и maxT< 41.0
    {
     dT = (maxT-minT)*16/10;
     if (dT<0) dT = 240;        // задаем число при котором dT >>=3; выполняется -> dT>20
     maxT /=10;
     dT >>=3;
     if (dT>20) dT = 255;
     else if (dT==0) dT = 100;
     else {maxT -= 20; maxT *= 20; maxT += (dT-1); dT = tabRH[maxT];};
    }
   else dT = 255;
   return dT;
 }

#define TUNING   170
unsigned char temperature_check(unsigned char zone)
{
unsigned char item, byte, devices, crc, try=0, count=0, errval;
unsigned char *p;
int val;
signed int *prt_int;
  switch (zone){
       case 1: devices = spot1; break;
       case 2: devices = spot2; break;
       case 3: devices = spot3; break;
  }
  if (devices){
    for (item=0; item < devices;){
         switch (zone){
            case 1: {ptr_to_eeprom = &romcodeZ1[item][0]; prt_int = &z1[item]; errval=0x01;} break;
            case 2: {ptr_to_eeprom = &romcodeZ2[item][0]; prt_int = &z2[item]; errval=0x02;} break;
            case 3: {ptr_to_eeprom = &romcodeZ3[item][0]; prt_int = &z3[item]; errval=0x04;} break;
         };
         w1_init(); // 1 Wire Bus initialization
         w1_write(0x55); // Load MATCH ROM [55H] comand
         w1_write(0x28); // Load family code [28H]
         for (byte=0; byte < 7; byte++) w1_write(*ptr_to_eeprom++); // Load cont. byte
         p = ds.buffer;
         w1_write(0xBE); // Read Scratchpad command [BE]
         for (byte=0; byte < 8; byte++) *p++ = w1_read(); // Read cont. byte
         crc = w1_read(); // Read CRC byte
         p = ds.buffer;
         if(w1_dow_crc8(p, 8)==crc){
               try = 0; val = ds.pvT; count++;
               if(val<0) {val=-val; val = val*10/16; val=-val;} else val = val*10/16; // перевод в десятичное значение !!
             //----- Коректировка датчика DS18B20 ------------------------------------------------------------------------------------------
//               if(ds.buffer[2]==TUNING) val +=(signed char)ds.buffer[3]; // корекция показаний датчика
             //-----------------------------------------------------------------------------------------------------------------------------
         }
         else if (++try > 2) {val = 990; try = 0; error |= errval;}// (99.0) если ошибка более X раз то больше не опрашиваем     
         *prt_int = val;
         if (try==0) item++;      
    };
  }
  else {
     switch (zone){
        case 1: for (item=0; item<SPOT1; item++) z1[item]=999; break;    // (99.9)
        case 2: z2[0]=999; break;                                        // (99.9)
     };
  };
  w1_init(); // 1 Wire Bus initialization
  w1_write(0xCC); // Load Skip ROM [CCH] command
  w1_write(0x44); // Load Convert T [44H] command
  return count;
}

void setDAC(void) // V = Vref x (255/256)
{
unsigned char i, val;
  for (i=0; i<4; i++){
     switch (i){
        case 0: CSDAC1 = 0; break;
        case 1: CSDAC2 = 0; break;
        case 2: CSDAC3 = 0; break;
        case 3: CSDAC4 = 0; break;
     }
     SPCR = SPI_MOUD_FL;// SPI port enabled
     val = dacU[i]>>4;
     SPDR = val & 0x0F; // Load Register to DAC
     while (!(SPSR & (1<<SPIF)));     // ожидаем конца передачи по SPI Opcode
     val = dacU[i]<<4;
     SPDR = val & 0xF0;
     while (!(SPSR & (1<<SPIF)));     // ожидаем конца передачи по SPI Opcode
     PORTD|=0xF0;       // INSTRUCTION EXECUTED
  };
  SPCR = 0;             //disable SPI
}

signed int mean(signed int *array)
{
unsigned char i, x=0;
signed int t=0;
 for (i=0; i<spot2; i++)
  {
    if(*array < 850){t += *array; x++;} else error |= 0x02;// ошибка датчиков компоста
    array++;
  };
 if(x) t /= x; else t = 999;// записываем 99.9 C
 return t;
}

signed int lowPF2a(signed int t,unsigned char i)
{
float val;
  val = A1*Told1[i]-A2*Told2[i]+A3*t;
  Told2[i] = Told1[i];
  Told1[i] = val;
  return val;
}

signed int lowPF2b(unsigned char u,unsigned char i)
{
float val;
  val = B1*Told1[i]-B2*Told2[i]+B3*u;
  Told2[i] = Told1[i];
  Told1[i] = val;
  return val;
}

void resetSP(void)
{
 char i;
 char eeprom *p_from_eeprom;
 ptr_to_eeprom = sys;
 p_from_eeprom = copyEEPROM;
 for (i=0; i<SIZCOPY; i++) *ptr_to_eeprom++ = *p_from_eeprom++; // востановление заводских настроек
 BeepT = 250;
}

void copySP(void)
{
 char i;
 char eeprom *p_from_eeprom;
 ptr_to_eeprom = sys;
 p_from_eeprom = copyEEPROM;
 for (i=0; i<SIZCOPY; i++) *p_from_eeprom++ = *ptr_to_eeprom++; // копирование заводских настроек
 BeepT = 250;
}