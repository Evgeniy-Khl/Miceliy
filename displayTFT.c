void fraction(signed int t)
{
  if (t<0) {t = -t; signchar = '-';} else signchar = ' ';
  frcval = t%10; intval = t/10;
}

char buttonCheck(void){
 char res=0;
   if(checkTouch()){
     res = (checkButton(buttonCount));
     if (res < buttonCount) {newButton = res; res=1;} // была нажата кнопка
     else {newButton = 100; res=0;}                   // нажатие не обнаружено
   }
   return res;
}

signed char plusCheck(char *str, char *Ystr){
 signed char bt,res=0;
   if(checkTouch()){
       bt = (checkPlus(plusCount));
//       sprintf(buff,"but=%2u; amt=%u", bt, plusCount);
//       TFT_DrawString(buff,150,TFTBUTTON-25,1,1,BLACK,GREEN1);
       if (bt < plusCount){
            *str = bt/2;    // присваиваем номер строки в которой была нажата кнопка
            if(plusCount<11) *Ystr = 37+*str*36; else *Ystr = 27+*str*20; // координаты строки для широких или узких кнопок
            if(bt%2) res=-1; else res=1;   // кнопка "+" или кнопка "-"
       }
   }
//   sprintf(buff,"res=%2i",res);
//   TFT_DrawString(buff,5,TFTBUTTON-25,1,1,BLACK,GREEN1);
   return res;
}

void drawButtSkip(unsigned int fW){
    newSetButt = 0;
    TFT_FillScreen(0, max_X, 0, max_Y, fW);
    initializeButtons(4,1,25);// 4 колонки; одна строка; высота 25
    //---------- фон ------- рамкa --- текст - номер -текст ---------
    drawButton(BLUE, WHITE, WHITE, 0, "Выход");
    drawButton(GREEN, BLACK, BLACK, 1, "<-");
    drawButton(GREEN, BLACK, BLACK, 2, "->");
    drawButton(MAGENTA, BLACK, BLACK, 3, "Кор.");
}

void drawButtEdit(unsigned int fW){
    newSetButt = 0;
    TFT_FillScreen(0, max_X, 0, max_Y, fW);
    initializeButtons(2,1,25);// 2 колонки; одна строка; высота 25
    //---------- фон ------- рамкa --- текст - номер -текст ---------
    drawButton(BLUE, WHITE, WHITE, 0, "Отмена");
    drawButton(MAGENTA, BLACK, BLACK, 1, "Сохран.");
}

void displ_0(void){
 unsigned char i, x, inttemp, frctemp;
 unsigned int fillWindow = BLUE1, bordWindow = BLACK, temp, X_left;
//-------------------------------- Основные показатели микроклимата. ----------------------------------------------------------------------------
  if (newSetButt){
    newSetButt = 0;
    TFT_FillScreen(0, max_X, 0, max_Y, fillWindow);
    initializeButtons(4,1,25);// 4 колонки; одна строка; высота 25
    //---------- фон ------- рамкa --- текст - номер -текст ---------
    drawButton(fillWindow, fillWindow, bordWindow, 0, "Общие");
    drawButton(WHITE, WHITE, BLACK, 1, "Упр.");
    drawButton(WHITE, WHITE, BLACK, 2, "Фаза");
    drawButton(WHITE, WHITE, BLACK, 3, "Настр");
  }
//-- СТРОКА 1 ---------------------------
  X_left=5; pointY=5;
  if(timerStop){
    TFT_DrawString("ОСТАНОВ",X_left,pointY,2,0,RED,fillWindow);
    sprintf(buff,"%3u мин.",timerStop);
    TFT_DrawString(buff,180,pointY+10,1,1,RED,fillWindow);
  }
  else if(timerDrying){
    TFT_DrawString("ПРОСУШКА",X_left,pointY,2,0,MAGENTA,fillWindow);
    sprintf(buff,"%3u мин.",timerDrying);
    TFT_DrawString(buff,180,pointY+10,1,1,MAGENTA,fillWindow);
  }
  else if(error && (counter1 & 4)){
    sprintf(buff,"ОШИБКА ");    
    switch (error)
     {
      case 0x01: strcat(buff,"Z1   "); break;// Ошибка датчикa SPOT1
      case 0x02: strcat(buff,"Z2   "); break;// Ошибка датчикa SPOT2
      case 0x03: strcat(buff,"Z12  "); break;// Ошибка датчикa SPOT1+SPOT2
      case 0x04: strcat(buff,"Z3   "); break;// Ошибка датчикa SPOT3
      case 0x05: strcat(buff,"Z13  "); break;// Ошибка датчикa SPOT1+SPOT3
      case 0x06: strcat(buff,"Z23  "); break;// Ошибка датчикa SPOT2+SPOT3
      case 0x07: strcat(buff,"Z123 "); break;// Ошибка датчикa SPOT1+SPOT2+SPOT3
      case 0x08: strcat(buff,"CO2  "); break;// Ошибка датчикa CO2
      case 0x10: strcat(buff,"DHT22"); break;// Ошибка датчикa DHT22
      default:   sprintf(buff,"ОШИБКА! Потеряны ВСЕ датчики");
     };
    TFT_DrawString(buff,5,pointY,2,0,MAGENTA,fillWindow);
  }
  else {
    i = sys[0]; x = 0x20; if(ChangeFaza) x=0x21; 
    sprintf(buff,"Фаза-%i %c",i+1,x);
    TFT_DrawString(buff,X_left,pointY,2,0,bordWindow,fillWindow);
    read_clock(CLOCK_BUFFER-1);
    sprintf(buff,"%2uд.%02xч.%02xм.%02x",dayStage,clock_buffer[2],clock_buffer[1],clock_buffer[0]);//Фаза;дней;час;мин;сек.
    TFT_DrawString(buff,170,pointY+10,1,1,bordWindow,fillWindow);
  }
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++  
  pointY = pointY+35;
//----------------------------------------------------------------------------------------------------------------------------------------- Датчики ВОЗДУХА ---
  temp = Tf[0];         // датчик воздуха  z1[0]
  if(temp<850)
   {
    fraction(temp);     // проверка знака температуры
    inttemp = intval; frctemp = frcval; x = signchar;  
    temp = setpoint.dry; fraction(temp); if(timerDrying) intval += sys[4];// Задание возд.
    sprintf(buff,"   Воздух: %c%2i.%i [%2i.%i]%c%с",x,inttemp,frctemp,intval,frcval,flags.ratDry,flags.unevenT);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
   }
  else TFT_DrawString("ОШИБКА датчик воздуха  ",5,pointY,1,1,MAGENTA,fillWindow);            // ОШИБКА
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//----
  x = setpoint.rh; if(timerDrying) x -= sys[2];
  if(pvRH<101){
    sprintf(buff,"Влажность:  %3i%% [%3i%%]%c",pvRH,x,flags.ratRH);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
  }
  else TFT_DrawString("ОШИБКА датчик влажности",5,pointY,1,1,MAGENTA,fillWindow);  // ОШИБКА
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//----------------------------------------------------------------------------------------------------------------------------------------- Компост -----------
  temp = Tf[2];         // среднее значение датчиков компоста  z2[]
  if(temp<850){
    fraction(temp);     // проверка знака температуры
    inttemp = intval; frctemp = frcval; x = signchar;
    temp = setpoint.soil; fraction(temp);// Задание комп.
    sprintf(buff,"  Компост: %c%2i.%i [%2i.%i]%c",x,inttemp,frctemp,intval,frcval,flags.ratSoil);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);    
  }
  else TFT_DrawString("ОШИБКА датчик компоста ",5,pointY,1,1,MAGENTA,fillWindow);            // ОШИБКА
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//----------------------------------------------------------------------------------------------------------------------------------------- Датчики компоста --
  X_left=10;
  for (i=0;i<4;i++){
    temp = z2[i];       // датчик компоста  z2[]
    fraction(temp);     // проверка знака температуры
    if(temp==999){
      TFT_DrawString(" **.*;",X_left,pointY,1,1,MAGENTA,fillWindow);
    }
    else {
      sprintf(buff,"%c%2i.%i;",signchar,intval,frcval);
      TFT_DrawString(buff,X_left,pointY,1,1,GRAY1,fillWindow);
    }
    X_left = X_left + 58;
  };
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
  X_left = 26;
//----------------------------------------------------------------------------------------------------------------------------------------- Воздух ПОДАЧИ -----
  temp = z3[0];                          // датчик зоны 3  z3[]
  fraction(temp);                       // проверка знака температуры
  if(temp<850){sprintf(buff,"Воздух ПОДАЧИ  %c%2i.%i",signchar,intval,frcval);}
  else {sprintf(buff,"Воздух ПОДАЧИ ОШИБКА");}// ОШИБКА
  TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//----------------------------------------------------------------------------------------------------------------------------------------- СМЕШАНЫЙ возд. ----  
  temp = z3[1];                          // датчик зоны 3  z3[]
  fraction(temp);                       // проверка знака температуры
  if(temp<850){sprintf(buff,"СМЕШАНЫЙ возд. %c%2i.%i",signchar,intval,frcval);}
  else {sprintf(buff,"СМЕШАНЫЙ возд. ОШИБКА");}// ОШИБКА
  TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//----------------------------------------------------------------------------------------------------------------------------------------- НАРУЖНЫЙ возд. ----  
  temp = z3[2];                          // датчик зоны 3  z3[]
  fraction(temp);                       // проверка знака температуры
  if(temp<850){sprintf(buff,"НАРУЖНЫЙ возд. %c%2i.%i",signchar,intval,frcval);}
  else {sprintf(buff,"НАРУЖНЫЙ возд. ОШИБКА");}// ОШИБКА
  TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//----------------------------------------------------------------------------------------------------------------------------------------- CO2 ---------------  
  temp = setpoint.co2;
  if(CO2module){sprintf(buff,"CO2,ppm [%4i]%c %4i",temp,flags.ratCO2,Tf[3]);}
  else {sprintf(buff,"CO2,ppm [%4i] ОШИБКА",temp);}// ОШИБКА    
  TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
}
  
//--------------------------------------------------------- ВЫХОДЫ УПРАВЛЕНИЯ ---------------------------------------------------------------displ_1--
void displ_1(void)
{
 char i, x;
 unsigned int fillWindow = GRAY1, bordWindow = BLACK, color_box;
 signed int temp;
  //-- Инициализация кнопок ---------------
  pointY=7;
  if (newSetButt){
    newSetButt = 0;
    TFT_FillScreen(0, max_X, 0, max_Y, fillWindow);
    initializeButtons(4,1,25);// 4 колонки; одна строка; высота 25
    //---------- фон ------- рамкa --- текст - номер -текст ---------
    drawButton(WHITE, WHITE, BLACK, 0, "Общие");
    drawButton(fillWindow, fillWindow, bordWindow, 1, "Упр.");
    drawButton(WHITE, WHITE, BLACK, 2, "Фаза");
    drawButton(WHITE, WHITE, BLACK, 3, "Настр");
    TFT_DrawString("ВЫХОДЫ УПРАВЛЕНИЯ",50,pointY,1,1,bordWindow,fillWindow);
  }
  pointY = pointY+25;
//---------------------------- АНАЛОГОВЫЕ ВЫХОДЫ ---------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------- Вент.Пр ----------
  temp = outU[0];
  sprintf(buff,"%14s %3i%% ",setKoff[0],temp);// Вент.Пр
  if(autoA[0]==-1){ 
      bordWindow = BLACK;
      switch (flags.vent)
       {
        case 1: strcat(buff," комп.#"); break;    // определяется регулятором равномерности температуры компоста в фазе "ПОДГОТОВКА" и "ПОСАДКА"
        case 2: strcat(buff,"CO2 пониж."); break; // определяется регулятором содержания СО2
        case 3: strcat(buff,"охлаждение"); break;// определяется регулятором для понижения температуры
        case 4: strcat(buff,"осушение  "); break;// определяется регулятором влажности
        case 5: strcat(buff,"нагрев    "); break;// определяется Положением клапана горячей воды.
        case 6: strcat(buff,"охлаждение"); break;// определяется Положением клапана холодной воды.
        default: strcat(buff,"задано MIN");// Зад. MIN
       };
  }
  else bordWindow = RED;
  TFT_DrawString(buff,5,pointY,1,1,bordWindow,fillWindow);
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20; 
//----------------------------------------------------------------------------------------------------------------------------------------- Засл.СВ ----------
  temp = outU[1];
  sprintf(buff,"%14s %3i%% ",setKoff[1],temp);// Засл.СВ
  if(autoA[1]==-1){ 
      bordWindow = BLACK;
      switch (flags.flap)
       {
        case 1: strcat(buff,"CO2 пониж."); break;  // определяется регулятором содержания СО2
        case 2: strcat(buff,"охлаждение"); break;// определяется регулятором для понижения температуры
        case 3: strcat(buff,"осушение  "); break;// определяется регулятором влажности   
        default: strcat(buff,"задано MIN");// Зад. MIN
       };
  }
  else bordWindow = RED;
  TFT_DrawString(buff,5,pointY,1,1,bordWindow,fillWindow);
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//----------------------------------------------------------------------------------------------------------------------------------------- Кран ГВ ----------
  temp = outU[2];
  sprintf(buff,"%14s %3i%% ",setKoff[3],temp);// Кран Гор.воды
  if(autoA[2]==-1){ 
      bordWindow = BLACK;
      if(flags.freez){strcat(buff,"ЗАЩИТА ** ");}   // Угроза замораживания !!!
      else if(temp>0){strcat(buff,"нагрев    ");}// определяется регулятором для повышения температуры
      else {strcat(buff,"задано MIN");}// Зад. MIN
  }
  else bordWindow = RED;
  TFT_DrawString(buff,5,pointY,1,1,bordWindow,fillWindow);
  pointY = pointY+20;
//----------------------------------------------------------------------------------------------------------------------------------------- Кран XВ ----------
  temp = outU[3]; 
  sprintf(buff,"%14s %3i%% ",setKoff[4],temp);// Кран Xол.воды
  if(autoA[3]==-1){ 
      bordWindow = BLACK;
      switch (flags.cold)
       {
        case 1: strcat(buff,"охлаждение"); break;// определяется регулятором для понижения температуры
        case 2: strcat(buff,"осушение  "); break;// определяется регулятором влажности   
        default: strcat(buff,"задано MIN");// Зад. MIN
       };
  }
  else bordWindow = RED;
  TFT_DrawString(buff,5,pointY,1,1,bordWindow,fillWindow);
  if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
  pointY = pointY+20;
//---------------------------- РЕЛЕЙНЫЕ ВЫХОДЫ ---------------------------------------------------
  for (i=0;i<4;i++){
    sprintf(buff,"%10s: ", nameD[i]); 
    switch (autoD[i]){
      case -1: strcat(buff,"АВТ"); bordWindow = BLACK; break;
      case 1: strcat(buff,"ON "); bordWindow = RED; break;
      case 0: strcat(buff,"OFF"); bordWindow = RED; break;
    };
    TFT_DrawString(buff,5,pointY,1,1,bordWindow,fillWindow);
    x = 1<<i; 
    if (relay&x) color_box=YELLOW;
    else color_box=BLACK;
    TFT_FillRectangle(170,pointY,25,18,color_box);
    if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
    pointY = pointY+20;
  };
}

//--------------------------------------------------- ПАРАМЕТРЫ ФАЗЫ <Общий список> -------------------------------------------------------- displ_2 -
void displ_2(void){
 char item, X_left;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, color_txt = BLACK, color_fon = GREEN1; 
  pointY=7;
  if (newSetButt){
    drawButtSkip(fillWindow);
    TFT_DrawString("ПАРАМЕТРЫ ФАЗ",50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- ПАРАМЕТРЫ ФАЗ -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+25;
  X_left = 20;
  for (item = 0; item < MAX_FAZA; item++){
    sprintf(buff,"%s", setFaza[item]);   //  %10s
    if (item == numMenu){color_txt = WHITE; color_fon = BLACK;} else {color_txt = BLACK; color_fon = GREEN1;}
    TFT_DrawString(buff,X_left,pointY,1,1,color_txt,color_fon);
    if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
    pointY = pointY+20;
  }
}

//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Общий список> ------------------------------------------------------ displ_3 -
void displ_3(void){
 char item, X_left;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, color_txt = BLACK, color_fon = GREEN1; 
  pointY=7;
  if (newSetButt){
    drawButtSkip(fillWindow);
    TFT_DrawString("НАСТРОЙКИ СИСТЕМЫ",50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- НАСТРОКИ СИСТЕМЫ -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+25;
  X_left = 20;
  for (item = 0; item < MAX_KOFF; item++){
    sprintf(buff,"%s", setKoff[item]);   //  %10s
    if (item == numMenu){color_txt = WHITE; color_fon = BLACK;} else {color_txt = BLACK; color_fon = YELLOW1;}
    TFT_DrawString(buff,X_left,pointY,1,1,color_txt,color_fon);
    if (buttonCheck()) return; //++++ проверим нажатие кнопки +++++++++++++++++++++++++
    pointY = pointY+20;
  }
}

//--------------------------------------------------- ПАРАМЕТРЫ ФАЗЫ <Задания Фазы> -------------------------------------------------------- displ_4 -
void displ_4(void){
 char item, fz=sys[0], X_left;
 signed char x;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 4, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    sprintf(buff,"ЗАДАНИЯ ФАЗЫ %u", fz+1);
    TFT_DrawString(buff,50,pointY,1,1,bordWindow,fillWindow);  //---------------------------------------- ЗАДАНИЯ ФАЗЫ -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 30;  
  for (item = 0; item < MAX_FAZA_SET; item++){
    x = plusCheck(&item, &pointY);
    if(x){
        if (item==3) x*=50;
        newval[item] = newval[item] + x;
    }
    temp = newval[item];
    switch (item){
        case 0: sprintf(buff,"%9s= %2u.%u", nameFaza[item],temp/10,temp%10); break;
        case 1: sprintf(buff,"%9s= %2u.%u", nameFaza[item],temp/10,temp%10); break;
        case 2: sprintf(buff,"%9s= %3u%%", nameFaza[item], temp); break;
        case 3: sprintf(buff,"%9s= %4u", nameFaza[item], temp); break;   
    }; 
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- ПАРАМЕТРЫ ФАЗЫ <Динамика Фазы> ------------------------------------------------------- displ_5 -
void displ_5(void){
 char item, fz=sys[0], X_left;
 signed char x;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 4, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    sprintf(buff,"ДИНАМИКА ФАЗЫ %u изм. за час", fz+1);
    TFT_DrawString(buff,20,pointY,1,1,bordWindow,fillWindow);   //---------------------------------------- ДИНАМИКА ФАЗЫ -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 30;  
  for (item = 0; item < MAX_FAZA_SET; item++){
    x = plusCheck(&item, &pointY);
    if(x){
        if (item==3) x*=10;
        newval[item] = newval[item] + x;
    }
    temp = newval[item];
    switch (item){
        case 0: sprintf(buff,"%9s= %i.%02u", nameFaza[item],temp/100,temp%100); break;
        case 1: sprintf(buff,"%9s= %i.%02u", nameFaza[item],temp/100,temp%100); break;
        case 2: sprintf(buff,"%9s= %i.%u%%", nameFaza[item],temp/10,temp%10); break;
        case 3: sprintf(buff,"%9s= %u", nameFaza[item], temp); break;   
    }; 
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- ПАРАМЕТРЫ ФАЗЫ <Разрешения Фазы> ------------------------------------------------------- displ_6 -
void displ_6(void){
 char item, fz=sys[0], X_left;
 signed char x;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 4, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    sprintf(buff,"РАЗРЕШЕНИЯ ФАЗЫ %u", fz+1);
    TFT_DrawString(buff,50,pointY,1,1,bordWindow,fillWindow);   //---------------------------------------- РАЗРЕШЕНИЯ ФАЗЫ -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 30;  
  for (item = 0; item < MAX_FAZA_SET; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item]; 
    sprintf(buff,"%9s= %u", nameFaza[item], temp);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- Управление А ---------------------------------------------------------------------------- displ_7 -
void displ_7(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 4, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString("УПРАВЛЕНИЕ А",50,pointY,1,1,bordWindow,fillWindow);   //---------------------------------------- УПРАВЛЕНИЕ А -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 25;  
  for (item = 0; item < 4; item++){
    x = plusCheck(&item, &pointY);
    if(x){
        newval[item] = newval[item] + x;
        if (newval[item]>100) newval[item]=-1;
        else if (newval[item]<-1) newval[item]=100;
    }
    temp = newval[item]; 
    sprintf(buff,"%14s ", setKoff[item]);
    if(temp==-1) sprintf(buff,"%14s АВТ ", setKoff[item]);
    else sprintf(buff,"%14s %2u%%", setKoff[item], temp);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- Управление D ---------------------------------------------------------------------------- displ_8 -
void displ_8(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 4, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString("УПРАВЛЕНИЕ D",50,pointY,1,1,bordWindow,fillWindow);   //---------------------------------------- УПРАВЛЕНИЕ D -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 25;  
  for (item = 0; item < 4; item++){
    x = plusCheck(&item, &pointY);
    if(x){
        newval[item] = newval[item] + x;
        if(newval[item]>1) newval[item]=-1;
        else if (newval[item]<-1) newval[item]=1;
    }
    temp = newval[item]; 
    sprintf(buff,"%14s ", nameD[item]);
    if(temp==-1) strcat(buff,"АВТ.");
    else if(temp==0) strcat(buff,"ОТКЛ");
    else strcat(buff,"ВКЛ.");
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- СПЕЦИВЛЬНЫЕ УСТАНОВКИ 0 ------------------------------------------------------- displ_9 -
void displ_9(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 3, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString("СПЕЦИВЛЬНЫЕ УСТАНОВКИ",50,pointY,1,1,bordWindow,fillWindow);   //---------------------------------------- СПЕЦИВЛЬНЫЕ УСТАНОВКИ -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 20;  
  for (item = 0; item < 3; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item]; 
    sprintf(buff,"%13s= %u", nameE0[item], temp);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- СПЕЦИВЛЬНЫЕ УСТАНОВКИ 1 ------------------------------------------------------- displ_10 -
void displ_10(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = GREEN1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 5, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString("СПЕЦИВЛЬНЫЕ УСТАНОВКИ",50,pointY,1,1,bordWindow,fillWindow);   //---------------------------------------- СПЕЦИВЛЬНЫЕ УСТАНОВКИ -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 20;  
  for (item = 0; item < 5; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item]; 
    sprintf(buff,"%13s= %u", nameE1[item], temp);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Вентилятор Притока> ------------------------------------------------------- displ_11 -
void displ_11(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 8, 14);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString(setKoff[0],55,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Вентилятор Притока -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+20;
  X_left = 30;  
  for (item = 0; item < MAX_FAZA_SET*2; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item];
    switch (item){
        case 0: sprintf(buff,"%9s= %2u.%u", nameFaza[item],temp/10,temp%10); break;
        case 1: sprintf(buff,"%9s= %2u.%u", nameFaza[item],temp/10,temp%10); break;
        case 2: sprintf(buff,"%9s= %3u%%", nameFaza[item], temp); break;
        case 3: sprintf(buff,"%9s= %4u", nameFaza[item], temp); break;
        default: sprintf(buff,"%9s= %4u", nameE2[item-4], temp);  
    }; 
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+20;}
  }
}

//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Заслонка Сежего Воздуха> -------------------------------------------------- displ_12 -
void displ_12(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 8, 14);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString(setKoff[1],55,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Заслонка Сежего Воздуха -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+20;
  X_left = 30;  
  for (item = 0; item < MAX_FAZA_SET*2; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item]; 
    switch (item){
        case 0: sprintf(buff,"%9s= %2u.%u", nameFaza[item],temp/10,temp%10); break;
        case 1: sprintf(buff,"  разреш.= %2u.%u",temp/10,temp%10); break;
        case 2: sprintf(buff,"%9s= %3u%%", nameFaza[item], temp); break;
        case 3: sprintf(buff,"%9s= %4u", nameFaza[item], temp); break;
        default: sprintf(buff,"%9s= %4u", nameE2[item-4], temp);  
    };
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+20;}
  }
}

//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Вытяжной вентилятор> ------------------------------------------------- displ_13 -
void displ_13(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 2, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString(setKoff[2],50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Вытяжной вентилятор -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 30;  
  for (item = 0; item < 2; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item];
    switch (item){
        case 0: sprintf(buff," ВКЛЮЧИТЬ= %3u%%", temp); break;
        case 1: sprintf(buff,"ОТКЛЮЧИТЬ= %3u%%", temp); break;   
    }; 
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}

//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Кран горячей воды> -------------------------------------------------- displ_14 -
void displ_14(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 5, 30);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString(setKoff[3],50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Кран горячей воды -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 30;  
  for (item = 0; item < 5; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item]; 
    if (item<4) sprintf(buff,"%12s= %3u", nameE2[item], temp);
    else sprintf(buff,"%12s= %2u%%", setKoff[0], temp);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+36;}
  }
}
//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Кран холодной воды> -------------------------------------------------- displ_15 -
void displ_15(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 7, 14);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString(setKoff[4],50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Кран холодной воды -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+20;
  X_left = 30;  
  for (item = 0; item < 7; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item];
    
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+20;}
  }
}
//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Коэф D12> -------------------------------------------------- displ_16 -
void displ_16(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 8, 14);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString(setKoff[5],50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Коэф D12 -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+20;
  X_left = 30;  
  for (item = 0; item < 8; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item];
    sprintf(buff,"%12s= %3u", nameE3[item], temp);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+20;}
  }
}
//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Коэф D34> -------------------------------------------------- displ_17 -
void displ_17(void){
 char item, X_left;
 signed char x;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, temp;
  pointY=7; pauseButt = 600;
  if (newSetButt){
    drawButtEdit(fillWindow);
    initializePlus(pointY+25, 8, 14);
    for (item=0;item<plusCount;item++) drawPlus(item, fillWindow);
    TFT_DrawString(setKoff[6],50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Коэф D34 -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+20;
  X_left = 30;  
  for (item = 0; item < 8; item++){
    x = plusCheck(&item, &pointY);
    if(x) newval[item] = newval[item] + x;
    temp = newval[item];
    sprintf(buff,"%12s= %3u", nameE4[item], temp);
    TFT_DrawString(buff,X_left,pointY,1,1,bordWindow,fillWindow);
    if(x) {item--; delay_ms(pauseButt); pauseButt -= 100; if(pauseButt<0) pauseButt=1;}
    else {if (buttonCheck()) return; pointY = pointY+20;}
  }
}
//--------------------------------------------------- НАСТРОЙКИ СИСТЕМЫ <Датчики> -------------------------------------------------- displ_18 -
void displ_18(void){
 char item, X_left;
 unsigned int fillWindow = YELLOW1, bordWindow = BLACK, color_txt = BLACK, color_fon = GREEN1; 
  pointY=7;
  if (newSetButt){
    drawButtSkip(fillWindow);
    TFT_DrawString(setKoff[6],50,pointY,1,1,bordWindow,fillWindow); //---------------------------------------- Датчики -
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pointY = pointY+30;
  X_left = 20;
  devices = w1_search(0xf0,rom_code);
  sprintf(buff,"Датчиков температуры %u",devices);
  TFT_DrawString(buff,X_left,pointY,1,1,color_txt,color_fon);
  pointY = pointY+30;
  X_left = 40;
  for (item = 0; item < 3; item++){
    if (item<2) sprintf(buff,"%s", nameFaza[item]);
    else sprintf(buff,"%s", setFaza[6]);    
    if (item == numMenu){color_txt = WHITE; color_fon = BLACK;} else {color_txt = BLACK; color_fon = YELLOW1;}
    TFT_DrawString(buff,X_left,pointY,1,1,color_txt,color_fon);
    pointY = pointY+20;
  }
}

void display(void)
{
  switch (displ_num)
   {
    case 0: displ_0(); break;
    case 1: displ_1(); break;
    case 2: displ_2(); break;
    case 3: displ_3(); break;
    case 4: displ_4(); break;
    case 5: displ_5(); break;
    case 6: displ_6(); break;
    case 7: displ_7(); break;
    case 8: displ_8(); break;
    case 9: displ_9(); break;
    case 10: displ_10(); break;
    case 11: displ_11(); break;
    case 12: displ_12(); break;
    case 13: displ_13(); break;
    case 14: displ_14(); break;
    case 15: displ_15(); break;
    case 16: displ_16(); break;
    case 17: displ_17(); break;
    case 18: displ_18(); break;
   };
}
