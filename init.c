//x = MCUCSR&0x0F; MCUCSR = 0;
//initPoint[idxClcPnt].resFlag = x;
//if(++idxClcPnt>INITROW-1) idxClcPnt = 0;

// Port B initialization
PORTB=0x00;// State7=0 State6=T State5=0 State4=T State3=T State2=0 State1=0 State0=T 
DDRB=0xA6; // Func7=Out Func6=In Func5=Out Func4=In Func3=In Func2=Out Func1=Out Func0=In 

// Port D initialization
PORTD=0xF0;// State7=1 State6=1 State5=1 State4=1 State3=T State2=T State1=T State0=T
DDRD=0xF0; // Func7=Out Func6=Out Func5=Out Func4=Out Func3=In Func2=In Func1=In Func0=In

// Timer/Counter 0 initialization
TCCR0=0x05;// (F_XTAL/1024)
INIT_TIMER0;// 5 ms

// Timer/Counter 1 initialization
TCCR1B=0x05;// (F_XTAL/1024)
INIT_TIMER1;// 1000 ms

// Timer/Counter 2 initialization
TCCR2=0x07;// (F_XTAL/1024)

// External Interrupt(s) initialization

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=0x05;

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 9600
//UCSRA=0x00;
//UCSRB=0xD8;
//UCSRC=0x86;
//UBRRH=0x00;
//UBRRL=0x67;

// Analog Comparator initialization
ACSR=0x80;// Analog Comparator: Off
SFIOR=0x00;

// TWI initialization
//TWBR=0x0C;// Bit Rate: 400,000 kHz
TWBR=0x48;// Bit Rate: 100,000 kHz
//TWAR=0x00;// Two Wire Bus Slave Address: 0x0; General Call Recognition: Off
TWCR=0x44;// Generate Acknowledge Pulse: On; TWI Interrupt: Off
//TWSR=0x00;

// ADC initialization
// ADC Clock frequency: 125,000 kHz
// ADC Voltage Reference: AREF pin
ADMUX=ADC_VREF_TYPE & 0xff;
ADCSRA=0x84;

setDAC();                           // подать напряжение на аналоговые выходы

w1_init(); // 1 Wire Bus initialization
w1_write(0xCC); // Load Skip ROM [CCH] command
w1_write(0x44); // Load Convert T [44H] command
read_clock(CLOCK_BUFFER);// проверка состояния CONTROL REGISTER и Bit7-'CH'
if(clock_buffer[7]!=SQWE_1Hz){
    clock_buffer[0] &= 0x7F;  // Bit7-'CH'=0
    for (i=1; i<7; i++) clock_buffer[i] = 1;
    clock_buffer[7] = SQWE_1Hz;
    write_clock(clock_buffer);
}
InitInterface();
TFT_Init();
//TFT_SetOrientation(3);  //0
TFT_FillScreen(0, max_X, 0, max_Y, fillScreen);
pointY = 2;
sprintf(buff,"www.graviton.com.ua"); TFT_DrawString(buff,70,pointY,1,1,WHITE,fillScreen);
pointY = pointY+30;
sprintf(buff,"МИЦЕЛИЙ"); TFT_DrawString(buff,40,pointY,2,0,WHITE,fillScreen);  TFT_DrawString("v3.0.0",200,pointY+10,1,1,WHITE,fillScreen);
pointY = pointY+60;
//------ detect модуль CO2 ---------------------------------------------------------    
//for (i=0; i<3; i++)
//{
// CO2module = module_check(ID_CO2); delay_ms(1000);
// if(CO2module){readCO2(); Told1[3] = Told2[3] = pvCO2; break;} // датчик углекислого газа
//}
x = 0;
for (i=1; i<4; i++) x += temperature_check(i);
Tf[0] = Told1[0] = Told2[0] = z1[0];// текущая температура воздуха
Tf[1] = Told1[1] = Told2[1] = z1[1];// психрометрический датчик
crc = mean(&z2[0]);                 // среднее значение датчиков компоста - SPOT1
Tf[2] = Told1[2] = Told2[2] = crc;  // текущая температура компоста
//------------------------------------------------------------------------------   
//Dht = readDHT();                       // detect DHT-21
if(Dht){Told1[4] = Told2[4] = pvRH;}   // электронный датчик
else pvRH = tableRH(z1[0], z1[1]);     // психрометрический датчик
//------------------------------------------------------------------------------
//sprintf(buff,"Вектор сброса: %u",x); 
//TFT_DrawString(buff,40,pointY,1,1,WHITE,fillScreen);
//pointY = pointY+20;
sprintf(buff,"%02x.%02x.%02xг.%02x:%02x:%02x  R=%2x",clock_buffer[4],clock_buffer[5],clock_buffer[6],clock_buffer[2],clock_buffer[1],clock_buffer[0],clock_buffer[7]);
TFT_DrawString(buff,40,pointY,1,1,WHITE,fillScreen);
pointY = pointY+20;
sprintf(buff,"Датчиков температуры %u",x); TFT_DrawString(buff,40,pointY,1,1,WHITE,fillScreen); // количество датчиков
pointY = pointY+20;
sprintf(buff,"Датчик влажности %u",Dht); TFT_DrawString(buff,40,pointY,1,1,WHITE,fillScreen); // датчик AM2301
pointY = pointY+20;
sprintf(buff,"Датчик CO2 %u",CO2module); TFT_DrawString(buff,40,pointY,1,1,WHITE,fillScreen); // датчик углекислого газа
pointY = pointY+20;
BeepT = 50;
if(copyEEPROM[0]==255) copySP();    // дубликат EEPROM  
delay_ms(3000);

flags.unevenT = flags.ratOld = flags.ratCO2 = flags.ratRH = flags.ratSoil = flags.ratDry = 0x20;

if(faza[sys[0]].Dry != setpoint.dry)  ChangeFaza=1;
if(faza[sys[0]].Soil!= setpoint.soil) ChangeFaza=1;
if(faza[sys[0]].RH  != setpoint.rh)   ChangeFaza=1;
if(faza[sys[0]].CO2 != setpoint.co2)  ChangeFaza=1;
if(ChangeFaza) setSP();
 
// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/2048k
//#asm("wdr")                   // сброс сторожевого таймера
//WDTCR=(0<<WDTOE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0); 


#asm("sei")                   // Global enable interrupts
newSetButt = 1;
NewCalc = 1;
NewMin = 1;