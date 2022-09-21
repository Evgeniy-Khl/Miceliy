void save_faza(unsigned char can)
{
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; else if(bufdispl>400) bufdispl=400; faza[can].Dry = bufdispl; break;// датчик сухой
     case 1: if(bufdispl<0) bufdispl=0; else if(bufdispl>400) bufdispl=400; faza[can].Soil = bufdispl; break;// температура компоста
     case 2: if(bufdispl<0) bufdispl=0; else if(bufdispl>400) bufdispl=400; faza[can].RH = bufdispl; break;// датчик влажный
     case 3: if(bufdispl<400) bufdispl=400; else if(bufdispl>5000) bufdispl=5000; faza[can].CO2 = bufdispl; break;// измеритель CO2
   };
  if(can==sys[0]){ChangeFaza = 1; setSP();}
}
void get_faza(unsigned char can)
{
  switch (strnum)
   {
     case 0: bufdispl = faza[can].Dry; break;     // датчик сухой
     case 1: bufdispl = faza[can].Soil; break;    // температура компоста
     case 2: bufdispl = faza[can].RH; break;      // отностительная влажность
     case 3: bufdispl = faza[can].CO2; break;     // измеритель CO2
   };
}
void save_faza_ext(unsigned char can)
{
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].DeltaDry  = bufdispl; break;// датчик сухой
     case 1: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].DeltaSoil = bufdispl; break;// температура компоста
     case 2: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].DeltaRH   = bufdispl; break;// датчик влажный
     case 3: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].DeltaCO2  = bufdispl; break;// измеритель CO2
   };
}
void get_faza_ext(unsigned char can)
{
  switch (strnum)
   {
     case 0: bufdispl = faza[can].DeltaDry; break;     // датчик сухой
     case 1: bufdispl = faza[can].DeltaSoil; break;    // температура компоста
     case 2: bufdispl = faza[can].DeltaRH; break;      // отностительная влажность
     case 3: bufdispl = faza[can].DeltaCO2; break;     // измеритель CO2
   };
}
void save_faza_lim(unsigned char can)
{
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; else if(bufdispl>100) bufdispl=100; faza[can].Min0 = bufdispl; break;// Вент.Пр MIN
     case 1: if(bufdispl<0) bufdispl=0; else if(bufdispl>100) bufdispl=100; faza[can].Max0 = bufdispl; break;// Вент.Пр MAX
     case 2: if(bufdispl<0) bufdispl=0; else if(bufdispl>100) bufdispl=100; faza[can].Min1 = bufdispl; break;// Засл.СВ MIN
     case 3: if(bufdispl<0) bufdispl=0; else if(bufdispl>100) bufdispl=100; faza[can].Max1 = bufdispl; break;// Засл.СВ MAX
   };
}
void get_faza_lim(unsigned char can)
{
  switch (strnum)
   {
     case 0: bufdispl = faza[can].Min0; break;// Вент.Пр MIN
     case 1: bufdispl = faza[can].Max0; break;// Вент.Пр MAX
     case 2: bufdispl = faza[can].Min1; break;// Засл.СВ MIN
     case 3: bufdispl = faza[can].Max1; break;// Засл.СВ MAX
   };
}
void save_faza_enabl(unsigned char can)
{
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].EnableDry  = bufdispl; break;
     case 1: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].EnableSoil = bufdispl; break;
     case 2: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].EnableRH   = bufdispl; break;
     case 3: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; faza[can].EnableCO2  = bufdispl; break;
   };
}

void get_faza_enabl(unsigned char can)
{
  switch (strnum)
   {
     case 0: bufdispl = faza[can].EnableDry; break; // 
     case 1: bufdispl = faza[can].EnableSoil; break;// 
     case 2: bufdispl = faza[can].EnableRH; break;  // 
     case 3: bufdispl = faza[can].EnableCO2; break; // 
   };
}

void save_digital(unsigned char can)
{
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; digital[can].t1 = bufdispl; break;
     case 1: if(bufdispl<0) bufdispl=0; else if(bufdispl>250) bufdispl=250; digital[can].t2 = bufdispl; break;
     case 2: if(bufdispl<1) bufdispl=1; else if(bufdispl>250) bufdispl=250; digital[can].k1 = bufdispl; break;
     case 3: if(bufdispl<1) bufdispl=1; else if(bufdispl>250) bufdispl=250; digital[can].k2 = bufdispl; break;
   };
}

void get_digital(unsigned char can)
{
  switch (strnum)
   {
     case 0: bufdispl = digital[can].t1; break;
     case 1: bufdispl = digital[can].t2; break;
     case 2: bufdispl = digital[can].k1; break;
     case 3: bufdispl = digital[can].k2; break;
   };
}

void save_analog(unsigned char can)
{
  if(can==2)
   {
    switch (strnum)
     {
       case 0: if(bufdispl<0) bufdispl=0; analog[2].limCO = bufdispl; break;//
       case 1: if(bufdispl<0) bufdispl=0; analog[3].limCO = bufdispl; break;//
       //case 2: if(bufdispl<0) bufdispl=0; analog[can].limCl = bufdispl; break;//
       //case 3: if(bufdispl<0) bufdispl=0; analog[can].limRh = bufdispl; break;//
     };
   }
  else if(can==3)
   {
    switch (strnum)
     {
       case 0: if(bufdispl<0) bufdispl=0; analog[2].limT  = bufdispl; break;//
       case 1: if(bufdispl<0) bufdispl=0; analog[3].limT  = bufdispl; break;//
       case 2: if(bufdispl<0) bufdispl=0; analog[3].limCl = bufdispl; break;//
       case 3: if(bufdispl<0) bufdispl=0; analog[3].limRh = bufdispl; break;//
     };
   }
  else
   {
    switch (strnum)
     {
       case 0: if(bufdispl<0) bufdispl=0; analog[can].limCO = bufdispl; break;//
       case 1: if(bufdispl<0) bufdispl=0; analog[can].limT  = bufdispl; break;//
       case 2: if(bufdispl<0) bufdispl=0; analog[can].limCl = bufdispl; break;//
       case 3: if(bufdispl<0) bufdispl=0; analog[can].limRh = bufdispl; break;//
     };
   }
}
void save_analog_ext(unsigned char can)
{
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; else if(bufdispl>100) bufdispl=100; analog[can].minVal = bufdispl; break;// min величина
     case 1: if(bufdispl<0) bufdispl=0; else if(bufdispl>100) bufdispl=100; analog[can].maxVal = bufdispl; break;// max величина
     case 2: if(bufdispl<1) bufdispl=1; else if(bufdispl>250) bufdispl=250; analog[can].kP = bufdispl; break;    //
     case 3: if(bufdispl<10) bufdispl=10; else if(bufdispl>250) bufdispl=250; analog[can].kI = bufdispl; break;  //
     
   };
}
void get_analog(unsigned char can)
{
  if(can==2)
   {
    switch (strnum)
     {
       case 0: bufdispl = analog[2].limCO;  break;//
       case 1: bufdispl = analog[3].limCO; break;//
      // case 2: bufdispl = analog[can].limCl; break;//
      // case 3: bufdispl = analog[can].limRh; break;//
     };
   }
  else if(can==3)
   {
    switch (strnum)
     {
       case 0: bufdispl = analog[2].limT;  break;//
       case 1: bufdispl = analog[3].limT; break;//
       case 2: bufdispl = analog[3].limCl; break;//
       case 3: bufdispl = analog[3].limRh; break;//
     };
   }
  else
   {
    switch (strnum)
     {
       case 0: bufdispl = analog[can].limCO;  break;//
       case 1: bufdispl = analog[can].limT; break;//
       case 2: bufdispl = analog[can].limCl; break;//
       case 3: bufdispl = analog[can].limRh; break;//
     };
   }
}
void get_analog_ext(unsigned char can)
{
  switch (strnum)
   {
     case 0: bufdispl = analog[can].minVal; break; // min величина
     case 1: bufdispl = analog[can].maxVal; break; // max величина
     case 2: bufdispl = analog[can].kP; break;     //
     case 3: bufdispl = analog[can].kI; break;     //
     
   };
}
void save_system(void)
{
 unsigned char i;
  switch (strnum)
   {
     case 0: if(bufdispl<1) bufdispl=1; else if(bufdispl>5) bufdispl=5;
             sys[0] = (bufdispl-1); ChangeFaza=1; setSP();// Текущая фаза
             for (i=0; i<3; i++) clock_buffer[i] = 0;// сек.[0],мин[1],час[2]
             for (i=4; i<7; i++) clock_buffer[i] = 1;// дата[4],мес[5],год[6]
             dayStage = 0; // обнуляем текущий день!
             clock_buffer[0]&= 0x7F; clock_buffer[7]=SQWE_1Hz; write_clock(clock_buffer);// запишем в микросхему
     break;
     case 1: if(bufdispl<0) bufdispl=0; sys[2] = timerStop = bufdispl; break;// Полная остановка
   };
}
void save_drying(void)
{
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; sys[3] = timerDrying = bufdispl; break;// Время просушки
     case 1: if(bufdispl<0) bufdispl=0; sys[4] = bufdispl; break;// Повышение темпер.
     case 2: if(bufdispl<0) bufdispl=0; sys[5] = bufdispl; break;// Понижение влажн.
     case 3: if(bufdispl<0) bufdispl=0; sys[6] = bufdispl; break;// Защита заморажив.
   };
}
void resetSP(void)
{
 char i;
 char eeprom *p_from_eeprom;
 ptr_to_eeprom = sys;
 p_from_eeprom = &copyEEPROM;
 for (i=0; i<SIZCOPY; i++) *ptr_to_eeprom++ = *p_from_eeprom++; // востановление заводских настроек
 BeepT = 250;
}
void copySP(void)
{
 char i;
 char eeprom *p_from_eeprom;
 ptr_to_eeprom = sys;
 p_from_eeprom = &copyEEPROM;
 for (i=0; i<SIZCOPY; i++) *p_from_eeprom++ = *ptr_to_eeprom++; // копирование заводских настроек
 BeepT = 250;
}

void save_net(void)
{
  char i;
  switch (strnum)
   {
     case 0: if(bufdispl<0) bufdispl=0; sys[1] = bufdispl; break;// Адрес прибора
     case 1: if(bufdispl==63) resetSP();      // Восстановление всех параметров и настроек к значениям, установленным на заводе изготовителе
             else if(bufdispl==-36) copySP(); // Резервное сохранение всех параметров и настроек пользователя.
             else if(bufdispl==5)             // Обнуление таблицы интервалов времени работы и векторов сброса.
              {
               idxClcPnt=0;
               for (i=0;i<INITROW;i++){initPoint[i].workHour=0; initPoint[i].workMin=0; initPoint[i].resFlag=0;};
              }
             else if(bufdispl==3)             // Немедленное применение параметров текущей фазы.
              {
                setpoint.dry = faza[sys[0]].Dry;
                setpoint.soil = faza[sys[0]].Soil;
                setpoint.rh = faza[sys[0]].RH;
                setpoint.co2 = faza[sys[0]].CO2;
                ChangeFaza = 0;
              }
     break;// Сброс параметров
   };
}
void get_system(void)
{
  switch (strnum)
   {
     #pragma warn-
     case 0: bufdispl = sys[0]+1;      break;// Текущая фаза
     #pragma warn+
     case 1: bufdispl = sys[2];        break;// Полная остановка
   };
}
void get_drying(void)
{
  switch (strnum)
   {
     case 0: bufdispl = sys[3]; break;// Время просушки
     case 1: bufdispl = sys[4]; break;// Повышение темпер.
     case 2: bufdispl = sys[5]; break;// Понижение влажн.
     case 3: bufdispl = sys[6]; break;// Защита заморажив.
   };
}
void get_net(void)
{
  switch (strnum)
   {
     case 0: bufdispl = sys[1]; break;// Адрес прибора
     case 1: bufdispl = 0;      break;// Сброс параметров
   };
}
void checkkey(unsigned char k)
{
static unsigned char key, count;
unsigned char i, j;
  if (k==0) count=0;
  else if (key==k) ++count;        // защита от дребезга контактов
  else {count=0; key=k;};          // нажата новая кнопка
  if (edit) //----------------------- если режим редактирования ВКЛЮЧЕН -----------------------------
   {
     if (count>1)
      {
       LcdDispl=1; waitset=60; BeepT = 1;
       switch (key)
        {
          case KEY_DOWN: if(menunum>0x20 && menunum<0x26 && strnum==3) bufdispl-=50; else bufdispl--; break;// для СО2 шаг 50 ppm
          case KEY_UP:   if(menunum>0x20 && menunum<0x26 && strnum==3) bufdispl+=50; else bufdispl++; break;// для СО2 шаг 50 ppm
          case KEY_ESC:  strnum=0; menunum=0; edit=0; waitkey=WAITCOUNT; break;
          case KEY_ENTER:
               {
                edit = 0;
                switch (menunum)
                 {
                   //-- Параметры фаз --------------------------------------
                   case 0x21: save_faza(0);     break;// Параметры фазы № 1
                   case 0x22: save_faza(1);     break;// Параметры фазы № 2
                   case 0x23: save_faza(2);     break;// Параметры фазы № 3
                   case 0x24: save_faza(3);     break;// Параметры фазы № 4
                   case 0x25: save_faza(4);     break;// Параметры фазы № 5
                   case 0x26: save_faza_ext(0); break;// Динамика фазы № 1
                   case 0x27: save_faza_ext(1); break;// Динамика фазы № 2
                   case 0x28: save_faza_ext(2); break;// Динамика фазы № 3
                   case 0x29: save_faza_ext(3); break;// Динамика фазы № 4
                   case 0x2A: save_faza_ext(4); break;// Динамика фазы № 5
                   case 0x2B: save_faza_lim(0); break;// Ограничения фазы № 1
                   case 0x2C: save_faza_lim(1); break;// Ограничения фазы № 2
                   case 0x2D: save_faza_lim(2); break;// Ограничения фазы № 3
                   case 0x2E: save_faza_lim(3); break;// Ограничения фазы № 4
                   case 0x2F: save_faza_lim(4); break;// Ограничения фазы № 5
                   case 0x30: save_faza_enabl(0); break;// Разрешения фазы № 1
                   case 0x31: save_faza_enabl(1); break;// Разрешения фазы № 2
                   case 0x32: save_faza_enabl(2); break;// Разрешения фазы № 3
                   case 0x33: save_faza_enabl(3); break;// Разрешения фазы № 4
                   case 0x34: save_faza_enabl(4); break;// Разрешения фазы № 5
                   //-- Парам.цифр.вых.-------------------------------------
                   case 0x41: if(bufdispl<0) bufdispl=-1; else if (bufdispl>1) bufdispl=1; autoD[strnum]=bufdispl; break;  // Ручное управление Цифровыми выходами 0-3
                   #pragma warn-
                   case 0x42: if(bufdispl<0) bufdispl=-1; else if (bufdispl>1) bufdispl=1; autoD[strnum+4]=bufdispl; break;// Ручное управление Цифровыми выходами 4-7
                   #pragma warn+
                   case 0x43: save_digital(0);  break;// Цифровой выход № 1
                   case 0x44: save_digital(1);  break;// Цифровой выход № 2
                   case 0x45: save_digital(2);  break;// Цифровой выход № 3
                   case 0x46: save_digital(3);  break;// Цифровой выход № 4
                   //-- Парам.анал.вых.--------------------------------------
                   case 0x61: if(bufdispl<0) bufdispl=-1; else if(bufdispl>100) bufdispl=100; autoA[strnum]=bufdispl; break;  // Ручное управление Аналоговыми выходами 0-3
                   #pragma warn-
                   case 0x62: 
                              if(bufdispl<0) bufdispl=-1; else if(bufdispl>100) bufdispl=100;
                              if((strnum+4)<6) autoA[strnum+4]=bufdispl; break;// Ручное управление Аналоговыми выходами 4,5
                   #pragma warn+
                   case 0x63: save_analog(0);  break;// Аналоговый выход № 1
                   case 0x64: save_analog_ext(0);  break;// Аналоговый выход № 1
                   case 0x65: save_analog(1);  break;// Аналоговый выход № 2
                   case 0x66: save_analog_ext(1);  break;// Аналоговый выход № 2
                   case 0x67: save_analog(2);  break;// Аналоговый выход № 3(кран BELIMO ГВ)
                   case 0x68: save_analog_ext(2);  break;// Аналоговый выход № 3(кран BELIMO ГВ)
                   case 0x69: save_analog(3);  break;// Аналоговый выход № 4(кран BELIMO ХВ)
                   case 0x6A: save_analog_ext(3);  break;// Аналоговый выход № 4(кран BELIMO ХВ)
                   /*
                   case 0x6B: save_analog(4);  break;// Аналоговый выход № 5
                   case 0x6C: save_analog_ext(4);  break;// Аналоговый выход № 5
                   case 0x6D: save_analog(5);  break;// Аналоговый выход № 6
                   case 0x6E: save_analog_ext(5);  break;// Аналоговый выход № 6
                   */
                   //--------------------------------------------------------
                   case 0x81: save_system(); break;//Переключение фазы
                   case 0x82: save_drying(); break;//Просушка
                   case 0x83: save_net(); break;//Локальная сеть
                   //--------------------------------------------------------
                   case -1:
                    {
                      switch (strnum)
                       {
                         case 0:
                          {
                            ptr_to_eeprom = &romcodeZ1[0][0];
                            if(bufdispl!=SPOT1) bufdispl=0;           // если датчиков больше чем положено генерируем ошибку
                            spot1 = bufdispl;
                          } break;
                         case 1:
                          {
                            ptr_to_eeprom = &romcodeZ2[0][0];
                            if(bufdispl>SPOT2) bufdispl=0;           // если датчиков больше чем положено генерируем ошибку
                            spot2 = bufdispl;
                          } break;
                         case 2:
                          {
                            ptr_to_eeprom = &romcodeZ3[0][0];
                            if(bufdispl>SPOT3) bufdispl=0;           // если датчиков больше чем положено генерируем ошибку
                            spot3 = bufdispl;
                          } break;
                       };
                      if (bufdispl)                   //bufdispl==end
                       {
                         ptr_to_ram = &rom_code[0][0];
                         for (i=0; i<bufdispl; i++)     //i=0; i<end; i++
                           {
                             ptr_to_ram++;// пропускаем 24h family code
                             for (j=0; j<7; j++) *ptr_to_eeprom++ = *ptr_to_ram++;
                             ptr_to_ram++;// пропускаем 0h
                           };
//                         if(LcdOk){lcd_clear(); lcd_gotoxy(5,1); lcd_putsf(st9);};  //записаны // !!!!!
                       }
                      else
                       {
//                         if(LcdOk){lcd_clear(); lcd_gotoxy(7,1); lcd_putsf(st7);};  //Ошибка  // !!!!!
                       };
                      delay_ms(1000); 
                    } break;
                 };
               } break;
        };
      };
   }
  else //---------------------------------- если режим редактирования ОТКЛЮЧЕН ------------------------------------------------
   {
     if (count==2)
      {
       LcdDispl=1; waitset=60; BeepT = 1;
       switch (key)
        {
          case KEY_DOWN:  if(menunum){if(++strnum>3) strnum=0;} break;
          case KEY_UP:    if(menunum){if(--strnum<0) strnum=3;} break;
          case KEY_L_R:   strnum=0; menunum=-1; pauseDispl=10;  break;
          case KEY_LEFT:  if(menunum){strnum=0; if(menunum>menubeg) menunum--;} else if(--displnum<0) displnum=3; break;
          case KEY_RIGHT: if(menunum){strnum=0; if(menunum<menuend) menunum++;} else if(++displnum>3) displnum=0; break;
          //---------------------------------------------------------------------------------------------------------------------
          case KEY_ENTER: 
                          #pragma warn-
                          if(menunum==1){menunum=(strnum+1)<<1; menunum<<=4; strnum=0;}
                          else if(menunum==0x20)
                            {
                              switch (strnum)
                               {
                                 case 0: menunum+=0x01; strnum=0; break;
                                 case 1: menunum+=0x06; strnum=0; break;
                                 case 2: menunum+=0x0B; strnum=0; break;
                                 case 3: menunum+=0x10; strnum=0; break;
                               }; 
                            }
                          else if(menunum==0x60)
                            {
                              switch (strnum)
                               {
                                 case 0: menunum+=0x01; strnum=0; break;
                                 case 1: menunum+=0x03; strnum=0; break;
                               }; 
                            }
                          else if(menunum==0x80)//#pragma warn-
                            {
                              switch (strnum)
                               {
                                 case 0: menunum+=0x01; strnum=0; break;
                                 case 1: menunum+=0x02; strnum=0; break;
                                 case 2: menunum+=0x03; strnum=0; break;
                               }; 
                            }
                          else if(menunum==0){menunum=1; strnum=0;}
                          #pragma warn+
          break;
          //---------------------------------------------------------------------------------------------------------------------
          case KEY_ESC: reports=0; strnum=0; menunum=0; displnum=0; edit=0; waitkey=WAITCOUNT; break;
          case KEY_REPORTS: if(++reports>24) reports=0; break;
          case KEY_EDIT:
               {
                if(menunum) edit = 1;
                switch (menunum)
                 {
                   //-- Параметры фаз --------------------------------------
                   case 0x21: get_faza(0);     break;// Параметры фазы № 1
                   case 0x22: get_faza(1);     break;// Параметры фазы № 2
                   case 0x23: get_faza(2);     break;// Параметры фазы № 3
                   case 0x24: get_faza(3);     break;// Параметры фазы № 4
                   case 0x25: get_faza(4);     break;// Параметры фазы № 5
                   case 0x26: get_faza_ext(0); break;// Динамика фазы № 1
                   case 0x27: get_faza_ext(1); break;// Динамика фазы № 2
                   case 0x28: get_faza_ext(2); break;// Динамика фазы № 3
                   case 0x29: get_faza_ext(3); break;// Динамика фазы № 4
                   case 0x2A: get_faza_ext(4); break;// Динамика фазы № 5
                   case 0x2B: get_faza_lim(0); break;// Ограничения фазы № 1
                   case 0x2C: get_faza_lim(1); break;// Ограничения фазы № 2
                   case 0x2D: get_faza_lim(2); break;// Ограничения фазы № 3
                   case 0x2E: get_faza_lim(3); break;// Ограничения фазы № 4
                   case 0x2F: get_faza_lim(4); break;// Ограничения фазы № 5
                   case 0x30: get_faza_enabl(0); break;// Разрешения фазы № 1
                   case 0x31: get_faza_enabl(1); break;// Разрешения фазы № 2
                   case 0x32: get_faza_enabl(2); break;// Разрешения фазы № 3
                   case 0x33: get_faza_enabl(3); break;// Разрешения фазы № 4
                   case 0x34: get_faza_enabl(4); break;// Разрешения фазы № 5
                   //-- Парам.цифр.вых.----------------------------------------
                   case 0x41: waitkey=WAITCOUNT*2; bufdispl = autoD[strnum]; break;  // Ручное управление Цифровыми выходами 0-3
                   #pragma warn-
                   case 0x42: waitkey=WAITCOUNT*2; bufdispl = autoD[strnum+4]; break;// Ручное управление Цифровыми выходами 4-7
                   #pragma warn+
                   case 0x43: get_digital(0); break;// Цифровой выход № 1
                   case 0x44: get_digital(1); break;// Цифровой выход № 2
                   case 0x45: get_digital(2); break;// Цифровой выход № 3
                   case 0x46: get_digital(3); break;// Цифровой выход № 4
                   //-- Парам.анал.вых.---------------------------------------
                   case 0x61: bufdispl = autoA[strnum]; break;  // Ручное управление Аналоговыми выходами 0-3
                   #pragma warn-
                   case 0x62: if((strnum+4)<6) bufdispl = autoA[strnum+4]; break;// Ручное управление Аналоговыми выходами 4,5
                   #pragma warn+
                   case 0x63: get_analog(0);  break;// Аналоговый выход № 1
                   case 0x64: get_analog_ext(0);  break;// Аналоговый выход № 1
                   case 0x65: get_analog(1);  break;// Аналоговый выход № 2
                   case 0x66: get_analog_ext(1);  break;// Аналоговый выход № 2
                   case 0x67: get_analog(2);  break;// Аналоговый выход № 3(кран BELIMO ГВ)
                   case 0x68: get_analog_ext(2);  break;// Аналоговый выход № 3 (кран BELIMO ГВ)
                   case 0x69: get_analog(3);  break;// Аналоговый выход № 4 (кран BELIMO ХВ)
                   case 0x6A: get_analog_ext(3);  break;// Аналоговый выход № 4 (кран BELIMO ХВ)
                   /*
                   case 0x6B: get_analog(4);  break;// Аналоговый выход № 5 
                   case 0x6C: get_analog_ext(4);  break;// Аналоговый выход № 5
                   case 0x6D: get_analog(5);  break;// Аналоговый выход № 6
                   case 0x6E: get_analog_ext(5);  break;// Аналоговый выход № 6
                   */
                   //--------------------------------------------------------
                   case 0x81: get_system(); break;//Переключение фазы
                   case 0x82: get_drying(); break;//Просушка
                   case 0x83: get_net(); break;//Локальная сеть
                 };
               } break;
        };
      };
   };
}
