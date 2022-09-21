void displ_faza(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     lcd_gotoxy(0,i); lcd_putsf(str10); sprintf(lcd_buffer,"-%u ",can+1); lcd_puts(lcd_buffer);// Фаза-x
     switch (i)
      {
        case 0: lcd_putsf(ms0);    temp = faza[can].Dry;  break; // возд.
        case 1: lcd_putsf(ms1);    temp = faza[can].Soil; break; // комп.
        case 2: lcd_putsf(ms2);    temp = faza[can].RH;   break; // влаж. 
        case 3: lcd_putsf("CO2  ");temp = faza[can].CO2;  break; // CO2
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     switch (i)
      {
        case 0: checksign(temp); sprintf(lcd_buffer,"%c%c%2i.%u",marker,signchar,tDryInt,tDry); break;
        case 1: checksign(temp); sprintf(lcd_buffer,"%c%c%2i.%u",marker,signchar,tDryInt,tDry); break;
        case 2: sprintf(lcd_buffer,"%c %3i%%",marker,temp); break;
        case 3: sprintf(lcd_buffer,"%c %4i",marker,temp); break;
      }; 
     lcd_gotoxy(14,i); lcd_puts(lcd_buffer);
   }; 
}
void displ_faza_ext(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     lcd_gotoxy(0,i); sprintf(lcd_buffer,"\xAA-%u ",can+1); lcd_puts(lcd_buffer); lcd_putsf(str13);// Ф-x Изм/ч. (скорость изменения)
     switch (i)
      {
        case 0: lcd_putsf(ms0);    temp = faza[can].DeltaDry;  break; // возд.
        case 1: lcd_putsf(ms1);    temp = faza[can].DeltaSoil; break; // комп.
        case 2: lcd_putsf(ms2);    temp = faza[can].DeltaRH;   break; // влаж. 
        case 3: lcd_putsf("CO2  ");temp = faza[can].DeltaCO2;  break; // CO2
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     switch (i)
      {
        case 0: tDry = temp%100; tDryInt = temp/100; sprintf(lcd_buffer,".%c%i.%02u",marker,tDryInt,tDry); break;
        case 1: tDry = temp%100; tDryInt = temp/100; sprintf(lcd_buffer,".%c%i.%02u",marker,tDryInt,tDry); break;
        case 2: checksign(temp); sprintf(lcd_buffer,".%c%i.%u%%",marker,tDryInt,tDry); break;
        case 3: sprintf(lcd_buffer," %c%3i",marker,temp); break;
      }; 
     lcd_gotoxy(14,i); lcd_puts(lcd_buffer);
   }; 
}
void displ_faza_lim(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     lcd_gotoxy(0,i); sprintf(lcd_buffer,"\xAA-%u ",can+1); lcd_puts(lcd_buffer);// Ф-x  (ограничение)
     switch (i)
      {
        case 0: lcd_putsf(ms3); lcd_putsf(" MIN"); temp = faza[can].Min0; break;// Вент.Пр MIN
        case 1: lcd_putsf(ms3); lcd_putsf(" MAX"); temp = faza[can].Max0; break;// Вент.Пр MAX
        case 2: lcd_putsf(ms4); lcd_putsf(" MIN"); temp = faza[can].Min1; break;// Засл.СВ MIN
        case 3: lcd_putsf(ms4); lcd_putsf(" MAX"); temp = faza[can].Max1; break;// Засл.СВ MAX
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     sprintf(lcd_buffer,"%c %3i",marker,temp); lcd_gotoxy(15,i); lcd_puts(lcd_buffer);
   };
}
void displ_faza_enbl(unsigned char can)
{
 unsigned char i,marker; //,x
 signed int temp;
  lcd_clear();
//  x = 1<<can; 
  for (i=0;i<4;i++)
   {
     lcd_gotoxy(0,i); sprintf(lcd_buffer,"\xAA-%u ",can+1); lcd_puts(lcd_buffer); lcd_putsf(str14);// Ф-x Разреш.
     switch (i)
      {
        case 0: lcd_putsf(ms0);    temp = faza[can].EnableDry;  break; // возд.
        case 1: lcd_putsf(ms1);    temp = faza[can].EnableSoil; break; // комп.
        case 2: lcd_putsf(ms2);    temp = faza[can].EnableRH;   break; // влаж. 
        case 3: lcd_putsf("CO2  ");temp = faza[can].EnableCO2;  break; // CO2
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     sprintf(lcd_buffer,"%c%i",marker,temp); lcd_gotoxy(18,i); lcd_puts(lcd_buffer);
   };
}
void displ_digital(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear(); 
  for (i=0;i<4;i++)
   {
     lcd_gotoxy(0,i); lcd_putsf(str9); sprintf(lcd_buffer,"D%u ",can+1); lcd_puts(lcd_buffer);// Коэфф.Di
     switch (i)
      {
        case 0: lcd_putsf("t1"); temp = digital[can].t1; break;
        case 1: lcd_putsf("t2"); temp = digital[can].t2; break;
        case 2: lcd_putsf("k1"); temp = digital[can].k1; break;
        case 3: lcd_putsf("k2"); temp = digital[can].k2; break;
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     sprintf(lcd_buffer,"%c %3i",marker,temp); lcd_gotoxy(15,i); lcd_puts(lcd_buffer);
   };
}

void displ_analog(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     switch (can)
      {
       case 0:// Вент.Пр
         {
          lcd_gotoxy(0,i); lcd_putsf(ms3); lcd_putsf(" ");// Вент.Пр
          switch (i)
           {
            case 0: lcd_putsf("CO2");  temp = analog[can].limCO; break; // CO2
            case 1: lcd_putsf(str14);    temp = analog[can].limT;  break; // Разреш.
            case 2: lcd_putsf(ms0);    temp = analog[can].limCl; break; // возд.
            case 3: lcd_putsf(ms2);    temp = analog[can].limRh; break; // влаж. 
           };
         if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
         switch (i)
          {
           case 0: sprintf(lcd_buffer,"%c%4i",marker,temp); break;
           case 1: checksign(temp); sprintf(lcd_buffer,"%c%2i.%u",marker,tDryInt,tDry); break;
           case 2: checksign(temp); sprintf(lcd_buffer,"%c%2i.%u",marker,tDryInt,tDry); break;
           case 3: sprintf(lcd_buffer,"%c%3i%%",marker,temp); break;
          }; 
         lcd_gotoxy(15,i); lcd_puts(lcd_buffer);
         } break;
       case 1:// Засл.СВ
         {
          lcd_gotoxy(0,i); lcd_putsf(ms4); lcd_putsf(" ");// Засл.СВ
          switch (i)
           {
            case 0: lcd_putsf("CO2");  temp = analog[can].limCO; break; // CO2
            case 1: lcd_putsf(str14);    temp = analog[can].limT;  break; // Разреш.
            case 2: lcd_putsf(ms0);    temp = analog[can].limCl; break; // возд.
            case 3: lcd_putsf(ms2);    temp = analog[can].limRh; break; // влаж. 
           };
         if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
         switch (i)
          {
           case 0: sprintf(lcd_buffer,"%c%4i",marker,temp); break;
           case 1: checksign(temp); sprintf(lcd_buffer,"%c%2i.%u",marker,tDryInt,tDry); break;
           case 2: checksign(temp); sprintf(lcd_buffer,"%c%2i.%u",marker,tDryInt,tDry); break;
           case 3: sprintf(lcd_buffer,"%c%3i%%",marker,temp); break;
          }; 
         lcd_gotoxy(15,i); lcd_puts(lcd_buffer);
         } break;
       case 2:// Кран ГВ
         {
          if(i<2){lcd_gotoxy(0,i); lcd_putsf(str22); lcd_gotoxy(10,i);}// Вент.Вытяжной
          switch (i)
           {
            case 0: lcd_putsf(".\x42\x4B\xA7");     temp = analog[2].limCO; break; // ВКЛ
            case 1: lcd_putsf(".\x4F\x54\x4B\xA7"); temp = analog[3].limCO; break; // ОТКЛ
//            case 2: lcd_putsf(); break; //
//            case 3: lcd_putsf(); break; // 
           };
         if(i<2)
          {
           if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
           sprintf(lcd_buffer,"%c%3i%%",marker,temp); lcd_gotoxy(15,i); lcd_puts(lcd_buffer);
          }
         } break;
       case 3:// Кран ХВ
         {
          lcd_gotoxy(0,i); if(i) lcd_putsf(ms6); else lcd_putsf(ms5); lcd_putsf(" ");// Кран ХВ или Кран ГВ
          switch (i)
           {
            case 0: lcd_putsf(ms3);   temp = analog[2].limT;  break; // Вент.Пр
            case 1: lcd_putsf(ms3);   temp = analog[3].limT;  break; // Вент.Пр
            case 2: lcd_putsf(str15); temp = analog[3].limCl; break; // Допол.
            case 3: lcd_putsf(ms2);   temp = analog[3].limRh; break; // влаж. 
           };
         if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
         sprintf(lcd_buffer,"%c%3i%%",marker,temp); lcd_gotoxy(15,i); lcd_puts(lcd_buffer);         
         } break;
       default:lcd_gotoxy(0,i); lcd_putsf(str9); sprintf(lcd_buffer,"A%u ",can+1); lcd_puts(lcd_buffer);// Коэфф.Ai
      };
   };
}
void displ_analog_ext(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     switch (can)
      {
       case 0: lcd_gotoxy(0,i); lcd_putsf(ms3); break;// Вент.Пр
       case 1: lcd_gotoxy(0,i); lcd_putsf(ms4); break;// Засл.СВ
       case 2: lcd_gotoxy(0,i); lcd_putsf(ms5); break;// Кран ГВ
       case 3: lcd_gotoxy(0,i); lcd_putsf(ms6); break;// Кран ХВ
       default:lcd_gotoxy(0,i); lcd_putsf(str15); sprintf(lcd_buffer,"A%u ",can+1); lcd_puts(lcd_buffer);// Допол.Ai
      };
     lcd_putsf(" "); 
     switch (i)
      {
        case 0: lcd_putsf("MIN"); temp = analog[can].minVal; break;
        case 1: lcd_putsf("MAX"); temp = analog[can].maxVal; break;
        case 2: lcd_putsf("kP "); temp = analog[can].kP; break;
        case 3: lcd_putsf("kI "); temp = analog[can].kI; break;
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     sprintf(lcd_buffer,"%c %3i",marker,temp); lcd_gotoxy(15,i); lcd_puts(lcd_buffer);
   };
}
void control_digital(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     lcd_gotoxy(0,i); lcd_putsf(menu00); sprintf(lcd_buffer," D%u",i+can+1); lcd_puts(lcd_buffer);// Управление
   #pragma warn-
     temp = autoD[i+can];
   #pragma warn+
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     lcd_gotoxy(15,i); sprintf(lcd_buffer,"%c",marker); lcd_puts(lcd_buffer);
     if (temp<0) lcd_putsf(st2); //АВТ.
     else if(temp==0) lcd_putsf(st4);//ОТКЛ
     else if(temp==1) lcd_putsf(st3);//ВКЛ.
   };
}
void control_analog(unsigned char can)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     if((i+can)>5) break;// отображаем только 6 каналов управления
     lcd_gotoxy(0,i); lcd_putsf(menu00); sprintf(lcd_buffer," A%u",i+can+1); lcd_puts(lcd_buffer);// Управление
   #pragma warn-
     temp = autoA[i+can];
   #pragma warn+
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     lcd_gotoxy(15,i); sprintf(lcd_buffer,"%c",marker); lcd_puts(lcd_buffer);
     if(temp<0) lcd_putsf(st2); //АВТ.
     else {sprintf(lcd_buffer,"%3i%%",temp); lcd_puts(lcd_buffer);};
   };
}
void displ_system(void)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<2;i++)
   {
     lcd_gotoxy(0,i);
     switch (i)
      {
        #pragma warn-
        case 0: temp = sys[0]+1; lcd_putsf(str10); lcd_putsf(fazaname[sys[0]]); break;// Фаза-fazaname
        #pragma warn+
        case 1: lcd_putsf(str23); temp = 0; break;// Полная остановка
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     sprintf(lcd_buffer,"%c%3i",marker,temp); lcd_gotoxy(16,i); lcd_puts(lcd_buffer);
   }; 
}

void displ_drying(void)
{
 unsigned char i,marker,x;
 signed int temp;
  lcd_clear();
  for (i=0;i<4;i++)
   {
     lcd_gotoxy(0,i);
     switch (i)
      {
        case 0: lcd_putsf(str24); temp = 0; x='m';     break;// Время просушки
        case 1: lcd_putsf(str25); temp = sys[4]; x=0xDF; break;// Изменение темпер.
        case 2: lcd_putsf(str26); temp = sys[5]; x=0x25; break;// Изменение влажн.
        case 3: lcd_putsf(str21); temp = sys[6]; x=0xDF; break;// Защита заморажив.
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     sprintf(lcd_buffer,"%c%2i%c",marker,temp,x); lcd_gotoxy(16,i); lcd_puts(lcd_buffer);
   }; 
}

void displ_net(void)
{
 unsigned char i,marker;
 signed int temp;
  lcd_clear();
  for (i=0;i<2;i++)
   {
     lcd_gotoxy(0,i);
     switch (i)
      {
        case 0: lcd_putsf(str27); temp = sys[1]; break;// Адрес прибора
        case 1: lcd_putsf(menu44); temp = 0;     break;// Сброс параметров
      };
     if(strnum==i){if(edit){marker = 0xDC; temp = bufdispl;} else marker = '>';} else marker = ' ';
     sprintf(lcd_buffer,"%c%2i",marker,temp); lcd_gotoxy(17,i); lcd_puts(lcd_buffer);
   }; 
}

void setup(void)
{
unsigned char i,val,marker;
 switch (menunum)
      {
       case 1:// Главное МЕНЮ
        {
          lcd_clear();
          for (i=0;i<4;i++)
           {
             if(strnum==i) marker = '>'; else marker = ' ';
             lcd_gotoxy(0,i); sprintf(lcd_buffer,"%c",marker); lcd_puts(lcd_buffer);
             lcd_gotoxy(1,i);
             switch (i)
              {
                case 0: lcd_putsf(menu01); break; //Параметры фазы
                case 1: lcd_putsf(menu00);lcd_putsf(menu02); break; //Управление "D"
                case 2: lcd_putsf(menu00);lcd_putsf(menu03); break; //Управление "A"
                case 3: lcd_putsf(menu00);lcd_putsf(menu04); break; //Управление системой
              };
           };
        }  break;
       //-- Параметры фазы ----------------------------------------
       case 0x20:
        {
          lcd_clear();
          menubeg=0x21; menuend=0x34;// количество пунктов
          for (i=0;i<4;i++)
           {
             if(strnum==i) marker = '>'; else marker = ' ';
             lcd_gotoxy(0,i); sprintf(lcd_buffer,"%c",marker); lcd_puts(lcd_buffer);
             lcd_gotoxy(1,i);
             switch (i)
              {
                case 0: lcd_putsf(menu21); break; //Задания фаз
                case 1: lcd_putsf(menu22); break; //Динамика фазы
                case 2: lcd_putsf(menu23); break; //Ограничения фазы
                case 3: lcd_putsf(menu24); break; //Разрешения фазы
              };
           };
        }  break;
       //------- Параметры фазы -----------------------------------------------
       case 0x21: displ_faza(0);     break;// Параметры фазы № 1
       case 0x22: displ_faza(1);     break;// Параметры фазы № 2
       case 0x23: displ_faza(2);     break;// Параметры фазы № 3
       case 0x24: displ_faza(3);     break;// Параметры фазы № 4
       case 0x25: displ_faza(4);     break;// Параметры фазы № 5
       case 0x26: displ_faza_ext(0); break;// Динамика фазы № 1
       case 0x27: displ_faza_ext(1); break;// Динамика фазы № 2
       case 0x28: displ_faza_ext(2); break;// Динамика фазы № 3
       case 0x29: displ_faza_ext(3); break;// Динамика фазы № 4
       case 0x2A: displ_faza_ext(4); break;// Динамика фазы № 5
       case 0x2B: displ_faza_lim(0); break;// Ограничения фазы № 1
       case 0x2C: displ_faza_lim(1); break;// Ограничения фазы № 2
       case 0x2D: displ_faza_lim(2); break;// Ограничения фазы № 3
       case 0x2E: displ_faza_lim(3); break;// Ограничения фазы № 4
       case 0x2F: displ_faza_lim(4); break;// Ограничения фазы № 5
       case 0x30: displ_faza_enbl(0); break;// Разрешения фазы № 1
       case 0x31: displ_faza_enbl(1); break;// Разрешения фазы № 2
       case 0x32: displ_faza_enbl(2); break;// Разрешения фазы № 3
       case 0x33: displ_faza_enbl(3); break;// Разрешения фазы № 4
       case 0x34: displ_faza_enbl(4); break;// Разрешения фазы № 5
       //------- Управление "D" -----------------------------------------------
       case 0x40: menubeg=0x41; menuend=0x46; menunum++; break;// количество пунктов
       case 0x41: control_digital(0); break;// Ручное управление Цифровыми выходами 0-3
       case 0x42: control_digital(4); break;// Ручное управление Цифровыми выходами 4-7
       case 0x43: displ_digital(0); break;// Цифровой выход № 1
       case 0x44: displ_digital(1); break;// Цифровой выход № 2
       case 0x45: displ_digital(2); break;// Цифровой выход № 3
       case 0x46: displ_digital(3); break;// Цифровой выход № 4
       //------ Управление "A" ------------------------------------------------
       case 0x60:
        {
          lcd_clear();
          menubeg=0x61; menuend=0x6A;// количество пунктов
          for (i=0;i<2;i++)
           {
             if(strnum==i) marker = '>'; else marker = ' ';
             lcd_gotoxy(0,i); sprintf(lcd_buffer,"%c",marker); lcd_puts(lcd_buffer);
             lcd_gotoxy(1,i);
             switch (i)
              {
                case 0: lcd_putsf(menu00);lcd_putsf(menu31); break; //Управление выходами
                case 1: lcd_putsf(menu32); break; //Основные параметры
              };
           };
        }  break;
       case 0x61: control_analog(0);  break;// Ручное управление Аналоговыми выходами 0-3
       case 0x62: control_analog(4);  break;// Ручное управление Аналоговыми выходами 0-3
       case 0x63: displ_analog(0);  break;// Аналоговый выход № 1
       case 0x64: displ_analog_ext(0);  break;// Аналоговый выход № 1
       case 0x65: displ_analog(1);  break;// Аналоговый выход № 2
       case 0x66: displ_analog_ext(1);  break;// Аналоговый выход № 2
       case 0x67: displ_analog(2);  break;// Аналоговый выход № 3(кран BELIMO ГВ)
       case 0x68: displ_analog_ext(2);  break;// Аналоговый выход № 3(кран BELIMO ГВ)
       case 0x69: displ_analog(3);  break;// Аналоговый выход № 4(кран BELIMO ХВ)
       case 0x6A: displ_analog_ext(3);  break;// Аналоговый выход № 4(кран BELIMO ХВ)
       /*
       case 0x6B: displ_analog(4);  break;// Аналоговый выход № 5
       case 0x6C: displ_analog_ext(4);  break;// Аналоговый выход № 5
       case 0x6D: displ_analog(5);  break;// Аналоговый выход № 6
       case 0x6E: displ_analog_ext(5);  break;// Аналоговый выход № 6
       */
       //--------------------------------------------------------
       case 0x80:
        {
          lcd_clear();
          menubeg=0x81; menuend=0x8E;// количество пунктов
          for (i=0;i<3;i++)
           {
             if(strnum==i) marker = '>'; else marker = ' ';
             lcd_gotoxy(0,i); sprintf(lcd_buffer,"%c",marker); lcd_puts(lcd_buffer);
             lcd_gotoxy(1,i);
             switch (i)
              {
                case 0: lcd_putsf(menu41); break; //Переключение фазы
                case 1: lcd_putsf(menu42); break; //Просушка
                case 2: lcd_putsf(menu43); break; //Локальная сеть
              };
           };
        }  break;
       case 0x81: displ_system(); break;// Переключение фазы
       case 0x82: displ_drying(); break;// Просушка
       case 0x83: displ_net(); break;// Локальная сеть
       case -1:
        {
          if (++pauseDispl>3)  // обновление экрана не каждую сек. а каждые х сек.
           {
             pauseDispl=0; lcd_clear(); TPIC_disable = 1;
             #asm("wdr")
             for (i=0;i<3;i++) // поиск для 3 зон.
              {
                lcd_gotoxy(0,i);
                lcd_putsf(st5); sprintf(lcd_buffer,"%2u",i+1); lcd_puts(lcd_buffer);
                if(!edit) tDryInt = temperature_check(i+1); else tDryInt = 0;
                if (strnum==i)
                 {
                   if (edit)
                    {
                      val = 0xDC;
                      #asm("cli")                     // Global disable interrupts
                      tDryInt = w1_search(0xf0,rom_code);
                      #asm("sei")                     // Global enable interrupts
                      bufdispl = tDryInt;
                    }
                   else {val = '>'; }
                 }
                else val = ' ';
                lcd_gotoxy(8,i); sprintf(lcd_buffer,"%c%2u",val,tDryInt); lcd_puts(lcd_buffer); lcd_putsf(st6);// х датчикa
              };
             TPIC_disable = 0;
           };
        } break;
      };
}

void display(void)
{
  switch (displnum)
   {
    case 1: windowA1(); break;
    case 2: windowTz2(); break;
    case 3: windowTz3(); break;
       
    default:windowMain();
   };
}

void report(void)
{
unsigned char i, row, page;
 lcd_clear();
 page = reports-1;
 for (i=0; i<4; i++)
  {
   row = page*4+i;
   sprintf(lcd_buffer,"%2u\xC9\xC0.%05u \xBC.%02u B=%2u",row,initPoint[row].workHour,initPoint[row].workMin,initPoint[row].resFlag);
   lcd_gotoxy(0,i);lcd_puts(lcd_buffer);
  }
}
