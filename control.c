unsigned char UpdatePI(signed int error, char i)// i-> индекс iPart[i]; time-> период между замерами ошибки регулирования
{
float pPart, Kp, Ki, Ud;//
  Kp = (float) analog[i].kP/4;   // Пропорциональный    analog[i].kP=20/4=5
  Ki = (float) analog[i].kI*100; // Интегральный        analog[i].kI=100*100=10000
  pPart = (float) Kp * error;    // расчет пропорциональной части
  //---- функция ограничения pPart -----------------------------
  if(pPart <0){pPart = 0; iPart[i] = 0;} else if(pPart > 100) pPart = 100; // функция ограничения ????? if(pPart <=0)
//--------------------------------------------------------------
  iPart[i] += (float) Kp / Ki  * error; // приращение интегральной части  
  Ud = pPart + iPart[i];                // выход регулятора до ограничения
//------ функция ограничения -----------------------------------
  if(Ud < 0) Ud = 0; else if(Ud > 100) Ud = 100;
  iPart[i] = Ud - pPart;                // "антинасыщяющая" поправка
  if(iPart[i]>digital[2].k1) iPart[i]=digital[2].k1;// "ограничивающая" поправка
  error = Ud;                           // преобразование формата из float к signed int
  return error;
};

void limitationOut(void)// ограничение управляющего сигнала
{
int itemp;
//---- НОРМАЛИЗАЦИЯ outU[0] ------------------------------------
 itemp = (int)(faza[sys[0]].Max0-faza[sys[0]].Min0)*outU[0]/100;
 itemp += faza[sys[0]].Min0; outU[0] = itemp;
 itemp = (int)(analog[0].maxVal-analog[0].minVal)*outU[0]/100;
 itemp += analog[0].minVal; outU[0] = lowPF2b(itemp,5);
//---- НОРМАЛИЗАЦИЯ outU[1] ------------------------------------ 
 itemp = (int)(faza[sys[0]].Max1-faza[sys[0]].Min1)*outU[1]/100;
 itemp += faza[sys[0]].Min1; outU[1] = itemp;
 itemp = (int)(analog[1].maxVal-analog[1].minVal)*outU[1]/100;
 itemp += analog[1].minVal; outU[1] = lowPF2b(itemp,6);
//---- НОРМАЛИЗАЦИЯ outU[2] ------------------------------------ 
 itemp = (int)(analog[2].maxVal-analog[2].minVal)*outU[2]/100;
 itemp += analog[2].minVal; outU[2] = lowPF2b(itemp,7);
//---- НОРМАЛИЗАЦИЯ outU[3] ------------------------------------
 itemp = (int)(analog[3].maxVal-analog[3].minVal)*outU[3]/100;
 itemp += analog[3].minVal; outU[3] = lowPF2b(itemp,8); 
};

//-------------------------------- Расчет ошибок регулирования. ---------------------------------------------------------------------------------
void calculationErrors(void)
{
 //char i;
 int sp;
  if(Tf[0]<400) sp = setpoint.dry;  else sp=0; if(sp) errors.dry = sp - Tf[0]; else errors.dry = 0;
  if(pvRH<101)  sp = setpoint.rh;   else sp=0; if(sp) errors.hum = sp - pvRH; else errors.hum = 0;
  if(Tf[2]<400) sp = setpoint.soil; else sp=0; if(sp) errors.soil= sp - Tf[2]; else errors.soil= 0;
  //----------------------------- ВЛИЯНИЕ температуры компоста на нагрев ------------------------------------------------------------------
  if(faza[sys[0]].EnableSoil&0x02)// если использование разрешено!!!
   {
    errors.dry += (errors.soil*10/digital[2].k2); // ВЛИЯНИЕ температуры компоста на нагрев (digital[2].k2=10)
   }
  //-----------------------------------------------------------------------------------------------------------------------------------
  if(timerDrying){errors.dry +=(int)sys[1]*10; errors.hum -= sys[2];}// изменение ошибки для выполнения ПРОСУШКИ
  if(CO2module){sp = setpoint.co2; if(sp) errors.co2 = Tf[3] - sp; else errors.co2 = 0;}// если подключен модуль СО2
  else errors.co2 = 0;// если модуль СО2 НЕ подключен
  if(flags.outU0!=2) outU[0] = 0;
  if(flags.outU1!=1) outU[1] = 0;
};
//--************************* Определение производительности приточного вентилятора. *****************************------------------
char controlVent(void)// ВНИМАНИЕ в расчете ПИД регулятора используется iPart[0]
{
 char i;
 int maxtemp=0, mintemp=500;
 //--------------- определяется регулятором равномерности температуры компоста в фазе "ПОДГОТОВКА" и "ПОСАДКА"
 if(sys[0]<2)// 0-ФАЗА I "ПОДГОТОВКА"; 1-ФАЗА II "ПОСАДКА"
  {
   if(faza[sys[0]].EnableSoil&0x01)// если использование разрешено!!!
    {
      for(i=0;i<4;i++){if(z2[i]<850){maxtemp = max(maxtemp, z2[i]); mintemp = min(mintemp, z2[i]);};};
      maxtemp -= mintemp;// величина неравномерности НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 0,1 грд.С) порог = (analog[0].limT = 20/k2; outU = 10%)
      if(flags.outU0==0 && maxtemp>=analog[0].limT){flags.outU0 = 1; iPart[0]=0; outU[0] = UpdatePI(maxtemp/digital[3].t1,0); return flags.outU0;}
      else if(flags.outU0==1 && maxtemp>(analog[0].limT/digital[0].k2)){outU[0] = UpdatePI(maxtemp/digital[2].k2,0); return flags.outU0;}
      else if(flags.outU0==1) flags.outU0 = 0;
    }
  }
 //--------------- определяется регулятором содержания СО2 --------------------------------------------
 if(faza[sys[0]].EnableCO2&0x01)// если использование разрешено!!!
  {
     maxtemp = errors.co2/50;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 50 ppm СО2) порог = (analog[0].limCO = 100; outU = 10%)
     if(flags.outU0==0 && errors.co2>=analog[0].limCO){flags.outU0 = 2; iPart[0]=0; outU[0] = UpdatePI(maxtemp,0); return flags.outU0;}
     else if(flags.outU0==2 && errors.co2>(analog[0].limCO/digital[0].k2)){outU[0] = UpdatePI(maxtemp,0); return flags.outU0;}
     else if(flags.outU0==2) flags.outU0 = 0;
  }
 //--------------- определяется регулятором для понижения температуры ---------------------------------
 if(faza[sys[0]].EnableDry&0x01)// если использование разрешено!!!
  {
   mintemp = Tf[0] - z3[2];   // если температура свежего воздуха это позволяет
   maxtemp = -errors.dry;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 0,1 грд.С) порог = (analog[0].limCl = 0.2; outU = 10%)
   if(mintemp>=analog[1].limT)// если температура свежего воздуха это позволяет
    {
     if(flags.outU0==0 && maxtemp>=analog[0].limCl){flags.outU0 = 3; iPart[0]=0; outU[0] = UpdatePI(maxtemp,0); return flags.outU0;}
     else if(flags.outU0==3 && maxtemp>(analog[0].limCl/digital[0].k2)){outU[0] = UpdatePI(maxtemp,0); return flags.outU0;}
     else if(flags.outU0==3) flags.outU0 = 0;
    }
   else if(flags.outU0==3) flags.outU0 = 0;
  }
 //--------------- определяется регулятором влажности -------------------------------------------------
 if(faza[sys[0]].EnableRH&0x01)// если использование разрешено!
  {
   if(errors.hum<0 && errors.dry<digital[3].t2)// если осушение необходимо и отклонение воздуха меньще 10,0 грд.
    {
     mintemp = -errors.hum;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 1% RH) порог = (analog[0].limRh = 3; outU = 15%)
     if(flags.outU0==0 && mintemp>=analog[0].limRh){flags.outU0 = 4; iPart[0]=0; outU[0] = UpdatePI(mintemp,0); return flags.outU0;}
     else if(flags.outU0==4 && mintemp>(analog[0].limRh/digital[0].k2)){outU[0] = UpdatePI(mintemp,0); return flags.outU0;}
     else if(flags.outU0==4) flags.outU0 = 0;
    }
   else if(flags.outU0==4) flags.outU0 = 0;
  }
 return flags.outU0;
};
//--****************************** Определения положения заслонки свежего воздуха. **************************************------------------
char controlFreshAir(void)// ВНИМАНИЕ в расчете ПИД регулятора используется iPart[1]
{
 int maxtemp, mintemp;
 //--------------- положение заслонки определяется регулятором содержания СО2 -------------------------
 if(faza[sys[0]].EnableCO2&0x02)// если использование разрешено!!!
  {
     maxtemp = errors.co2/50;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 50 ppm СО2) порог = (analog[1].limCO = 100; outU = 10%)
     if(flags.outU1==0 && errors.co2>=analog[1].limCO){flags.outU1 = 1; iPart[1]=0; outU[1] = UpdatePI(maxtemp,1); return flags.outU1;}
     else if(flags.outU1==1 && errors.co2>(analog[1].limCO/digital[1].k2)){outU[1] = UpdatePI(maxtemp,1); return flags.outU1;}
     else if(flags.outU1==1) {flags.outU1 = 0; outU[1]=0;}// закрыть заслонку
  }
 //--------------- положение заслонки определяется регулятором для понижения температуры --------------
 if(faza[sys[0]].EnableDry&0x02)// если использование разрешено!!!
  {
   mintemp = Tf[0] - z3[2];   // разница между сухим датчиком и датчиком наружной температуры   
   if(mintemp>=analog[1].limT)// если температура наружного воздуха это позволяет
    {
      maxtemp = - errors.dry;
      if(flags.outU1==0 && maxtemp>=analog[1].limCl){flags.outU1 = 2; iPart[1]=0; outU[1] = UpdatePI(maxtemp,1); return flags.outU1;}
      else if(flags.outU1==2 && maxtemp>(analog[1].limCl/digital[1].k2)){outU[1] = UpdatePI(maxtemp,1); return flags.outU1;}
      else if(flags.outU1==2) {flags.outU1 = 0; outU[1]=0;}// закрыть заслонку
    }
   else if(flags.outU1==2) {flags.outU1=0; outU[1]=0;}
  }
 //--------------- положение заслонки определяется регулятором влажности ------------------------------
 if(faza[sys[0]].EnableRH&0x02)// если использование разрешено!!!
  {
   if(errors.hum<0 && errors.dry<digital[3].t2)// если осушение необходимо и ошибка по сухому меньще 10,0 грд.
    {
     mintemp = -errors.hum;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 1% RH) порог = (analog[1].limRh = 3; outU = 15%)
     if(flags.outU1==0 && mintemp>=analog[1].limRh){flags.outU1 = 3; iPart[1]=0; outU[1] = UpdatePI(mintemp,1); return flags.outU1;}
     else if(flags.outU1==3 && mintemp>(analog[1].limRh/digital[1].k2)){outU[1] = UpdatePI(mintemp,1); return flags.outU1;}
     else if(flags.outU1==3) {flags.outU1 = 0; outU[1]=0;}// закрыть заслонку
    }
   else if(flags.outU1==3) {flags.outU1 = 0; outU[1]=0;}// закрыть заслонку
  }
 return flags.outU1;
};
//-------------------------------- Работа вытяжного вентилятора. --------------------------------------------------------------------------------
void controlVentOutlet(void)
{
 if(outU[0]>analog[2].limCO) CAN4 = ON; else if(outU[0]<(analog[3].limCO)) CAN4 = OFF;
}
//--****************************** Положение клапана горячей воды. *******************************************-----------------------------------
char controlHotBelimo(void)// ВНИМАНИЕ в расчете ПИД регулятора используется iPart[2]
{
 unsigned char val, antifreeze=0;
 int itemp;
   itemp = errors.dry;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 0.1 грд.С )
   val = UpdatePI(itemp,2);
   //----------------- КОРЕКЦИЯ производительности приточного вентилятора!! ---------------------------
   if(val>=analog[2].limT){outU[0]=val; flags.outU0=5;}
   else if(sys[3]>0)  //----------------- Защита от замораживания -------------------------------------
    {
     itemp = (int) sys[3]*10 - z3[0];// вступает ограничение 7 грд.С. для датчикa воздуха подачи зоны № 3
     if(itemp>0){antifreeze=1; itemp >>=1; if(itemp>25) itemp=25; else if(itemp<5) itemp=5; val=(char)itemp;};
    }
   if(flags.outU0 == 5 && val>(analog[2].limT/3)){outU[0]=val; flags.outU0=5;} else if(flags.outU0 == 5) flags.outU0=0;
   outU[2] = val;
   //--- определение момента включения Циркуляционных насосов -----------------------------------------
   if(val > digital[0].t1)// порог ВКЛ. в % открытия заслонки
    {
      if(++timerPump[0]>digital[0].k1){CAN1 = ON; timerPump[0]=digital[0].k1;}// задержка в сек. перед ВКЛ.
    }
   else if(val < digital[0].t2)// порог ОТКЛ. в % открытия заслонки
    {
      if(--timerPump[0]<0){CAN1 = OFF; timerPump[0]=0;}// задержка в сек. перед ОТКЛ.
    }
 return antifreeze;
};
#ifndef USART
//--****************************** Положение клапана холодной воды. ***************************************--------------------------------------
char controlColdBelimo(void)// ВНИМАНИЕ в расчете ПИД регулятора используется iPart[3]
{
 unsigned char val;
 int itemp;
   itemp = -errors.dry;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 0.1 грд.С )
   val = UpdatePI(itemp,3); if(val>0) flags.outU3 = 1; else flags.outU3 = 0;
   //----------------- КОРЕКЦИЯ производительности приточного вентилятора!! ---------------------------
   if(val>=analog[3].limT){outU[0]=val; flags.outU0=6;}
   if(flags.outU0 == 6 && val>(analog[3].limT/3)){outU[0]=val; flags.outU0=6;} else if(flags.outU0 == 6) flags.outU0=0;
   outU[3] = val;
   //--------------- положение клапана определяется регулятором влажности  ----------------------------
   if(faza[sys[0]].EnableRH&0x20) // если использование разрешено!!!
    {
       if(errors.hum<0 && errors.dry<5)// если осушение необходимо и отклонение воздуха меньще 0,5 грд.
        {
          itemp = -errors.hum;// НОРМАЛИЗАЦИЯ переменной процесса (1 единица = 1% RH) порог = (analog[3].limRh = 6; outU = 30%)
          if(flags.outU3==0 && itemp>=analog[3].limRh){flags.outU3 = 2; iPart[3]=0; outU[3] = UpdatePI(itemp,3);} // если НЕТ охлаждения
          else if(flags.outU3==1 && itemp>=analog[3].limRh){outU[3] += analog[3].limCl; if(outU[3]>100) outU[3]=100;} // если идет охлаждение то открываем на analog[3].limCl (30%)
          else if(flags.outU3==2 && itemp>=(analog[3].limRh>>digital[1].k2)){outU[3] = UpdatePI(itemp,3);}
          else if(flags.outU3==2){flags.outU3=0; iPart[3]=0; outU[3]=0;}
        }
       else if(flags.outU3==2){flags.outU3=0; iPart[3]=0; outU[3]=0;} 
    }
   //--- определение момента включения Циркуляционных насосов -----------------------------------------
   if(outU[3] > digital[1].t1)// порог ВКЛ. в % открытия заслонки
    {
      if(++timerPump[1]>digital[1].k1){CAN2 = ON; timerPump[1]=digital[1].k1;}// задержка в сек. перед ВКЛ.
    }
   else if(outU[3] < digital[1].t2)// порог ОТКЛ. в % открытия заслонки
    {
      if(--timerPump[1]<0){CAN2 = OFF; timerPump[1]=0;}// задержка в сек. перед ОТКЛ.
    }
 return flags.outU3;
};
#else
//-------------------------------- Определения положения заслонки рециркуляции. -----------------------------------------------------------------
void controlRecycling(void)
{
//Заслонки работают в противофазе, сумма процентов открытия заслонок свежего и рециркуляционного воздуха всегда равна 100%. 
 outU[3]=100-outU[1];
}
#endif
//-------------------------------- Клапан увлажнителя. ------------------------------------------------------------------------------------------
void controlHumidifierValve(void) 
{
 if(faza[sys[0]].RH && (error&0x01)==0 && timerDrying==0)// если уставка увлажнения больше 0 и ошибок датчиков нет то включаем клапан
  {
    if(errors.hum > digital[2].t1) CAN3 = ON; else if(errors.hum <0) CAN3 = OFF;
  }
 else CAN3 = OFF;
};

