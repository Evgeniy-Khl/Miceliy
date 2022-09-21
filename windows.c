void windowMain(void)
{
 unsigned char i,x;
 signed int temp;
//-------------------------------- Основные показатели микроклимата. ----------------------------------------------------------------------------
//-- СТРОКА 1 ---------------------------
  lcd_clear(); 
  if(timerStop){lcd_putsf(str23); sprintf(lcd_buffer," %3i",timerStop); lcd_puts(lcd_buffer);}                          // Полная остановка xx
  else if(timerDrying){lcd_putsf(menu42); sprintf(lcd_buffer," %3i",timerDrying); lcd_puts(lcd_buffer); lcd_putsf(st8);}// Просушка xx мин.
  else if(error && (countTIM1_OVF & 4))
   {
    switch (error)
     {
      case 0x01: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" Z1   "); break;// Ошибка датчикa SPOT1
      case 0x02: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" Z2   "); break;// Ошибка датчикa SPOT2
      case 0x03: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" Z12  "); break;// Ошибка датчикa SPOT1+SPOT2
      case 0x04: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" Z3   "); break;// Ошибка датчикa SPOT3
      case 0x05: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" Z13  "); break;// Ошибка датчикa SPOT1+SPOT3
      case 0x06: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" Z23  "); break;// Ошибка датчикa SPOT2+SPOT3
      case 0x07: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" Z123 "); break;// Ошибка датчикa SPOT1+SPOT2+SPOT3
      case 0x08: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" CO2  "); break;// Ошибка датчикa CO2
      case 0x10: lcd_putsf(st7);lcd_putsf(st6);lcd_putsf(" DHT22"); break;// Ошибка датчикa DHT22
      default:   lcd_putsf(st7);lcd_putsf(" Потеряны ВСЕ датчики");
     };
   }
  else
   {
    i = sys[0]; if(ChangeFaza & (countTIM1_OVF & 1)) x=0xDD; else x=0x20;
    read_clock(); lcd_putsf(str10);//Фаза
    sprintf(lcd_buffer,"-%i%c %2x\xE3.%02x\xC0.%02x\xBC.",i+1,x,dayStage,clock_buffer[2],clock_buffer[1]);//Фаза;дней;час;мин.
    lcd_puts(lcd_buffer);
   }

//-- СТРОКА 2 ---------------------------
  lcd_gotoxy(0,1); lcd_putsf(ms0);// возд.
  temp = Tf[0];                   // датчик воздуха  z1[0]
  checksign(temp);                // проверка знака температуры
  if(temp<850)
   {
    sprintf(lcd_buffer,"%c%2i.%i",signchar,tDryInt,tDry); lcd_puts(lcd_buffer);
    temp = setpoint.dry; checksign(temp); if(timerDrying) tDryInt += sys[4];// Задание возд.
    sprintf(lcd_buffer,"[%2i.%i]%c%с",tDryInt,tDry,flags.ratDry,flags.unevenT); lcd_puts(lcd_buffer);
   }
  else lcd_putsf(st7);            // ОШИБКА

//-- СТРОКА 3 ---------------------------
  lcd_gotoxy(0,2); lcd_putsf(ms1);// комп. 
  temp = Tf[2];                   // среднее значение датчиков компоста  z2[]
  checksign(temp);                // проверка знака температуры
  if(temp<850)
   {
    sprintf(lcd_buffer,"%c%2i.%i",signchar,tDryInt,tDry); lcd_puts(lcd_buffer);
    temp = setpoint.soil; checksign(temp);// Задание комп.
    sprintf(lcd_buffer,"[%2i.%i]%c",tDryInt,tDry,flags.ratSoil); lcd_puts(lcd_buffer);    
   }
  else lcd_putsf(st7);            // ОШИБКА

//-- СТРОКА 4 ---------------------------
  lcd_gotoxy(0,3); lcd_putsf(ms2);// влажн.
  x = setpoint.rh; if(timerDrying) x -= sys[5];
  if(pvRH<101){sprintf(lcd_buffer," %3i%%[%3i%%]%c",pvRH,x,flags.ratRH); lcd_puts(lcd_buffer);}
  else lcd_putsf(st7);  // ОШИБКА
}

//-------------------------------- Положение заслонок и кранов. ---------------------------------------------------------------------------------
void windowA1(void)
{
 signed int temp;
//-- СТРОКА 1 --------------------------
  lcd_clear();
  temp = outU[0];
  lcd_putsf(ms3); sprintf(lcd_buffer,"%3i%% ",temp); lcd_puts(lcd_buffer);// Вент.Пр
  switch (flags.vent)
   {
    case 1: lcd_putsf(ms1);lcd_putsf("\x23"); break;//"комп.#" определяется регулятором равномерности температуры компоста в фазе "ПОДГОТОВКА" и "ПОСАДКА"
    case 2: lcd_putsf("CO2 \xDA"); break;           //"CO2 |"  определяется регулятором содержания СО2
    case 3: lcd_putsf(ms0);lcd_putsf("\xDA"); break;//"возд.|" определяется регулятором для понижения температуры
    case 4: lcd_putsf(ms2);lcd_putsf("\xDA"); break;//"влаж.|" определяется регулятором влажности
    case 5: lcd_putsf(ms0);lcd_putsf("\xD9"); break;//"возд.|" определяется Положением клапана горячей воды.
    case 6: lcd_putsf(ms0);lcd_putsf("\xDA"); break;//"возд.|" определяется Положением клапана холодной воды.
    default:lcd_putsf(str11);lcd_putsf("MIN");// Зад. MIN
   };
   
//-- СТРОКА 2 --------------------------
  lcd_gotoxy(0,1);
  temp = outU[1];
  lcd_putsf(ms4); sprintf(lcd_buffer,"%3i%% ",temp); lcd_puts(lcd_buffer);// Засл.СВ
  switch (flags.flap)
   {
    case 1: lcd_putsf("CO2 \xDA"); break;           //"CO2 |" определяется регулятором содержания СО2
    case 2: lcd_putsf(ms0);lcd_putsf("\xDA"); break;//"возд.|" определяется регулятором для понижения температуры
    case 3: lcd_putsf(ms2);lcd_putsf("\xDA"); break;//"влаж.|" определяется регулятором влажности   
    default:lcd_putsf(str11);lcd_putsf("MIN");// Зад. MIN
   };

//-- СТРОКА 3 -------------------------
  lcd_gotoxy(0,2);
  temp = outU[4];
  lcd_putsf(ms5); sprintf(lcd_buffer,"%3i%% ",temp); lcd_puts(lcd_buffer);// Кран ГВ
  if(flags.freez){lcd_putsf(str16);lcd_putsf("\x2A");}   // Угроза замораживания !!!
  else if(temp>0){lcd_putsf(ms0);lcd_putsf("\xD9");}//"возд.|"
  else {lcd_putsf(str11);lcd_putsf("MIN");}// Зад. MIN
  
//-- СТРОКА 4 -------------------------
  lcd_gotoxy(0,3);
  temp = outU[5];
  lcd_putsf(ms6); sprintf(lcd_buffer,"%3i%% ",temp); lcd_puts(lcd_buffer);// Кран XВ
  switch (flags.cold)
   {
    case 1: lcd_putsf(ms0);lcd_putsf("\xDA"); break;//"возд.|" определяется регулятором для понижения температуры
    case 2: lcd_putsf(ms2);lcd_putsf("\xDA"); break;//"влаж.|" определяется регулятором влажности   
    default:lcd_putsf(str11);lcd_putsf("MIN");// Зад. MIN
   };
}

//-------------------------------- Датчики компоста. --------------------------------------------------------------------------------------------
void windowTz2(void)
{
 char i;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
    lcd_gotoxy(0,i);lcd_putsf(str17);lcd_putsf(ms1);lcd_gotoxy(11,i);sprintf(lcd_buffer,".%u",i); lcd_puts(lcd_buffer);  //Датчик комп.i 
    temp = z2[i];                          // датчик компоста  z2[]
    checksign(temp);                       // проверка знака температуры
    if(temp==990) lcd_putsf(st7);          // ОШИБКА
    else {sprintf(lcd_buffer,"  %c%2i.%i",signchar,tDryInt,tDry); lcd_puts(lcd_buffer);}
   };
}

//-------------------------------- Датчик СО2 ---------------------------------------------------------------------------------------------------
void windowTz3(void)
{
 char i;
 signed int temp;
  lcd_clear();     lcd_putsf(str20); lcd_putsf(ms0);// ПОДАЧИ возд.
  lcd_gotoxy(0,1); lcd_putsf(str19); lcd_putsf(ms0);// СМЕШАНЫЙ возд.
  lcd_gotoxy(0,2); lcd_putsf(str18); lcd_putsf(ms0);// НАРУЖНЫЙ возд.     
  lcd_gotoxy(0,3); lcd_putsf("CO2,ppm ");
  for (i=0;i<4;i++)
   {
    temp = z3[i];                          // датчик зоны 3  z3[]
    checksign(temp);                       // проверка знака температуры 
    if(i<3)                                // только для датчиков температуры
     {
      if(temp<850){sprintf(lcd_buffer,"%c%2i.%i",signchar,tDryInt,tDry); lcd_gotoxy(15,i); lcd_puts(lcd_buffer);}
      else {lcd_gotoxy(8,i); lcd_putsf(st7);}// ОШИБКА
     }
    else
     {
      temp = setpoint.co2;
      if(CO2module){sprintf(lcd_buffer,"[%4i]%c %4i",temp,flags.ratCO2,Tf[3]); lcd_gotoxy(8,i); lcd_puts(lcd_buffer);}
      else {lcd_gotoxy(8,i); lcd_putsf(st7);}// ОШИБКА
     };
   };
}