unsigned char calcDj(signed int actualT)      // Расчетное задание ЦИФРОВЫХ выходов № 1,2,3,4,5,6
{
 signed int err;
 unsigned char i,sw;
 static unsigned char outDj;
  for (i=0;i<8;i++)
   {
     sw = 2;
 
     err = faza[sys[0]].Dry;
     if(err) // если уставка не равна 0 то расчитываем...
      {
        err = (err + digital[i].biasT) - actualT;
        if(digital[i].trend) err = -err;
        if(err > digital[i].bandT){sw = 0; outDj &= ~(1<<i);}; // отключить (+x.xx грд.C.)
        if(err < 0) {sw = 1; outDj |= (1<<i);}                 // включить    timerDj[i]
      }
     else sw = 0;// иначе отключить
     if(timerDj[i]>(digital[i].onTime-1+digital[i].offTime)) timerDj[i]=0;
     if(sw == 0)
      {
        if (timerDj[i]<digital[i].onTime) outDj |= (1<<i);  // включить
        else  outDj &= ~(1<<i);                             // отключить
      };
   };
 return outDj;
}

void calcAn(signed int actualT)      // Расчетное задание АНАЛОГОВЫХ выходов № 1,2,3,4
{
 signed int err;
 unsigned char i;
  for (i=0;i<6;i++)
   {
     /*
     switch (analog[i].sensor) // выбираем уставку в соответствии с опорным сенсором
      {
        case 1:  err = faza[stage].RH;  break;      // отностительная влажность
        case 2:  err = faza[stage].Soil; break;     // температура компоста
        case 3:  err = faza[stage].CO2;  break;     // датчик CO2
        case 4:  err = faza[stage].RHumidity; break;// датчик отностительной влажности
        default: err = faza[stage].Dry;
      };*/
     err = faza[sys[0]].Dry;
     if(err) // если уставка не равна 0 то расчитываем...
      {
        err = (err + analog[i].biasT) - actualT;
        if(digital[i].trend) err = -err;
        if(err<=0) err = (int)analog[i].minVal;//*corrA/100;
        else if(err>=analog[i].bandT) err = (int)analog[i].maxVal;//*corrA/100;
        else {err *= 100; err /= analog[i].bandT;};// err *= corrA; err /= 100;
// если sys[1]=   0 то редукция отсутствует! если sys[1]=100 то клапаны всегда закрыты!
// если corrA = 100 то редукция отсутствует! если corrA =  0 то клапаны всегда закрыты!
        if(err < analog[i].minVal) err = analog[i].minVal;
      }
     else err = analog[i].minVal;// иначе минимальное значение
     outU[i]=(char)err;
   };
}

void control(void)
{
 signed int actualT;
 actualT = Tf[0];                           // Среднее значение датчиков воздуха (3 шт.)
 if (actualT>maxT) maxT=actualT;
 if (actualT<minT) minT=actualT;
 calcAn(actualT);
 relay&=0xC0;
 relay|=calcDj(actualT);
 if (spT[1] - actualT < 0) error = 0x08;   // Температура ВЫСОКАЯ
 if (actualT - spT[2] < 0) error = 0x04;   // Температура НИЗКАЯ
}