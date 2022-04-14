///////////////////////////////////// ТЕРМОФЕН / HOT AIR GUN ////////////////////////////////////
///////////////////// ТЕРМОВОЗДУШНАЯ ПАЯЛЬНАЯ СТАНЦИЯ / SMD REWORK STATION //////////////////////
//////////////////////////////////// ВЕРСИЯ 1.5 / VERSION 1.5 ///////////////////////////////////
///////////////////// СКЕТЧ АНАТОЛИЯ НЕВЗОРОВА / CODE BY ANATOLY NEVZOROFF //////////////////////
////////////////////////////// https://github.com/AnatolyNevzoroff //////////////////////////////

//БОЛЬШИЕ ЦИФРЫ ДЛЯ millis()
#define p60000 60000L //ИНТЕРВАЛ - МИНУТА
#define p1000 1000L //ИНТЕРВАЛ 1000 mS (1 СЕКУНДА), ТИП ДАННЫХ "L" (long)
#define p500 500L
#define p400 400L
#define p100 100L


#define MENU_key 5        //D5 ВХОД КНОПКА МЕНЮ (КНОПКА ЭНКОДЕРА SW)
#define HOT_PWR_LED_out 6 //D6 ВЫХОД УПРАВЛЕНИЯ СИМИСТОРОМ НА ТЭН И ИНДИКАТОРОМ (Оранжевый)
#define POWER_LED_out 9   //D9 ВЫХОД ИНДИКАТОР ВКЛЮЧЕНИЯ POWER ON (Красный)
#define FAN_out 10        //D10 ВЫХОД ДЛЯ УПРАВЛЕНИЯ НАПРЯЖЕНИЕМ НА ВЕНТИЛЯТОРЕ ТУРБИНЫ
#define BUZZER_out 11     //D11 ВЫХОД ДЛЯ ПЬЕЗОПИЩАЛКИ 
#define RELAY_out 13      //D13 ВЫХОД УПРАВЛЕНИЯ СИМИСТОРОМ (РЭЛЕ ПИТАНИЯ) ТЕРМОФЕНА 
#define GERKON_key 15     //D15 ВХОД А1 МАГНИТНЫЙ ДАТЧИК "ГЕРКОН" ОТСЛЕЖИВАЕМ ПОЛОЖЕНИЕ РУЧКИ
#define POWER_key 16      //D16 ВХОД А2 КНОПКА ВКЛЮЧЕНИЯ/ОТКЛЮЧЕНИЯ "POWER ON/OFF"


#define hot 45 //БЕЗОПАСНАЯ ТЕМПЕРАТУРА СТАЛЬНОЙ ПОВЕРХНОСТИ ТЕРМОФЕНА (ОТКЛЮЧЕНИЯ ОХЛАЖДЕНИЯ)
#define mintemp 20    //ГРАНИЦА МИНИМАЛЬНО-ВОЗМОЖНОЙ ТЕМПЕРАТУРЫ ФЕНА
#define maxtemp 380   //ГРАНИЦА МАКСИМАЛЬНО-ВОЗМОЖНОЙ ТЕМПЕРАТУРЫ ФЕНА
#define minimtemp 130 //ГРАНИЦА МИНИМАЛЬНО-РЕГУЛИРУЕМОЙ ТЕМПЕРАТУРЫ ФЕНА

//ДЛЯ ЗАЩИТЫ ФЕНА ОТ ПРЕГРЕВА, В ИНЖЕНЕРНОМ МЕНЮ, ПРИ ОТКЛЮЧЁННОЙ РУЧКЕ ФЕНА (ИММИТИРУЕМ 
//ОБРЫВ ТЕРМОПАРЫ ИЛИ ПЕРЕГРЕВ) ОПРЕДЕЛЯЕМ МАКСИМАЛЬНО ВОЗМОЖНОЕ ЗНАЧЕНИЕ ВЫДАВАЕМОЕ АЦП, 
//ОНО ЗАВИСИТ ОТ РАБОТЫ УСИЛИТЕЛЯ И ОПОРНОГО НАПРЯЖЕНИЯ (В НАШЕМ, ПРОСТЕЙШЕМ СЛУЧАЕ ЭТО 
//НАПРЯЖЕНИЕ ПИТАНИЯ АРДУИНО) НАПРИМЕР, У МЕНЯ НАПРЯЖЕНИЕ МЕНЬШЕ 5 ВОЛЬТ, ПОЛУЧАЕМ ЗНАЧЕНИЕ: 1023
//ПОЭТОМУ ДЛЯ АВАРИЙНОГО ОТКЛЮЧЕНИЯ, КОНСТАНТУ "maxin" ВЫБИРАЕМ НЕСКОЛЬКО МЕНЬШЕ, НАПРИМЕР: 1021
#define maxin 1021 //В ЛЮБОМ СЛУЧАЕ, НЕ РЕКОМЕНДУЮ СТАВИТЬ ЗДЕСЬ 1024


//////////////////////////////////// ПОДКЛЮЧАЕМЫЕ БИБЛИОТЕКИ ////////////////////////////////////
#include <EEPROM.h>//ЧТЕНИЕ И ЗАПИСЬ ПЕРЕМЕННЫХ В ЭНЕРГОНЕЗАВИСИМУЮ ПАМЯТЬ EEPROM
#include <Encoder.h>//ОБРАБОТКА УГЛА ПОВОРОТА ЭНКОДЕРА
#include <PID_v1.h>//https://github.com/br3ttb/Arduino-PID-Library  \
БИБЛИОТЕКА ПРОПОРЦИОНАЛЬНО-ИНТЕГРАЛЬНОГО ДИФФЕРЕНЦИРОВАНИЯ ДЛЯ ИНЕРТНЫХ СИСТЕМ
#include <LedControl.h>//https://github.com/wayoda/LedControl  \
БИБЛИОТЕКА ДЛЯ ДРАЙВЕРОВ MAX7219 И MAX7221 СВЕТОДИОДНЫХ ИНДИКАТОРОВ И ПАНЕЛЕЙ

////////////////////////////////// ИНИЦИАЛИЗАЦИЯ ОБОРУДОВАНИЯ ///////////////////////////////////
Encoder myEnc(3,4);//DT и CLK ИЛИ S2 и S1 ВЫВОДЫ ЭНКОДЕРА
LedControl LC=LedControl(12,8,7,1);//12-DIN, 8-CLC, 7-CS, 1-ОДИН ИНДИКАТОР

/////////////////////////////////// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ///////////////////////////////////////
//unsigned long от 0 до 4294967295
//ТАЙМЕРЫ ДЛЯ ФУНКЦИИ millis()
uint32_t mill,timer_autoret,timer_BTN,timer_BUZZER,timer_MENU,timer_GERKON,timer_LED;

//int от -32768 до 32767
int16_t triac,inputdata,temp,In_TEMP,KpSet,KiSet,KdSet,trig0,trig1;
int16_t freq[5]={0,2077,1577,1077,577};//МАССИВ ЧАСТОТ ДЛЯ ФУНКЦИИ tone() 

//byte от 0 до 255
uint8_t temp00,temp01,KpSet10,KpSet11,KiSet10,KiSet11,KdSet10,KdSet11;
uint8_t cold,fan,fanspeed,sharp,bright,ar1,ar2,ar3,ar4,ar5,ar6,ar7,ar8;
uint8_t FL0,FL1,FL2,FL3,FL4,FL5,FL6,FL8,FL9;//ФЛАГИ
uint8_t zen[2]={5,10};//МАССИВ ДЛЯ УСТАНОВКИ ШАГА ТЕМПЕРАТУРЫ
//МАССИВ ЗНАЧЕНИЙ ШИМ ДЛЯ СКОРОСТИ ВЕНТИЛЯТОРА (подбираем индивидуально)
uint8_t pwm[9]={0,100,109,118,125,137,150,165,255};

//char от -128 до 127
int8_t fr,m0,m1,m2,menu,down,wait,oldPos,newPos;

//boolean от false до true
bool led,GERKON_flag,power,btn,POWER_btn,GERKON_btn,MENU_btn;

//СТРУКТУРА ДЛЯ УПРАВЛЕНИЯ ПОДСВЕТКОЙ ТОЧЕК
struct DPLed{bool L0:1,L1:1,L2:1,L3:1,L4:1,L5:1,L6:1,L7:1;};DPLed D;


//ЗАДАЁМ ПЕРЕМЕННЫЕ ДЛЯ БИБЛИОТЕКИ ПИД (PID)
double Setpoint;//ТРЕБУЕМАЯ ТЕМПЕРАТУРА НА ВЫХОДЕ ИЗ СОПЛА ТЕРМОФЕНА
double Input;   //ДАННЫЕ О ТЕКУЩЕЙ ТЕМПЕРАТУРЕ
double Output;  //ЗНАЧЕНИЕ ОТ 0 ДО 255 ВЫДАВАЕМОЕ БИБЛИОТЕКОЙ В ЗАВИСИМОСТИ ОТ Setpoint И Input
double Kp;      //АГРЕССИВНОСТЬ НАБОРА ТЕМПЕРАТУРЫ Setpoint, ЧЕМ БОЛЬШЕ ТЕМ БЫСТРЕЕ
double Ki;      //"ПРЕДСКАЗЫВАЕТ" ЗНАЧЕНИЕ Input, ЧЕМ БОЛЬШЕ ТЕМ ИНЕРЦИОННЕЕ (ОСЛАБЛЯЕТ Kd)
double Kd;      //УСРЕДНЯЕТ, СГЛАЖИВАЕТ Output ПРЕДОТВРАЩАЕТ КОЛЕБАНИЯ
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //ИНИЦИАЛИЗИРУЕМ ПЕРЕМЕННЫЕ ДЛЯ PID




/////////////////////////////////////////// SETUP //////////////////////////////////////////////
void setup(){


//ПЕРЕВОДИМ ПИНЫ D9 и D10 ТАЙМЕРА №1 НА ЧАСТОТУ - 31.4 кГц
TCCR1A=0b00000001;//8bit
TCCR1B=0b00000001;//x1 phase correct


//pinMode(2,INPUT);   //ВХОД ДЛЯ ОПРЕДЕЛЕНИЯ ПЕРЕХОДА СИНУСОИДЫ ЧЕРЕЗ "0"
//pinMode(3,INPUT);   //ПОВОРОТ ЭНКОДЕРА белый провод DT (S2) (ПИН ИНИЦИАЛИЗИРУЕТСЯ БИБЛИОТЕКОЙ)
//pinMode(4,INPUT);   //ПОВОРОТ ЭНКОДЕРА жёлтый провод CLK (S1) (ПИН ИНИЦИАЛИЗИРУЕТСЯ БИБЛИОТЕКОЙ)
pinMode(MENU_key,INPUT);//ВХОД D5 КНОПКА МЕНЮ (Кнопка SW энкодера)
pinMode(HOT_PWR_LED_out,OUTPUT);//ВЫХОД D6 УПРАВЛЕНИЕ СИМИСТОРОМ ТЭН И ИНДИКАТОРОМ (Оранжевый)
//pinMode(7,OUTPUT);  //ВЫХОД ДЛЯ ВЫВОДА "LOAD/CS" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
//pinMode(8,OUTPUT);  //ВЫХОД ДЛЯ ВЫВОДА "CLK" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
pinMode(POWER_LED_out,OUTPUT);//ВЫХОД D9 ИНДИКАТОР ВКЛЮЧЕНИЯ POWER ON/OFF (Красный светодиод)
pinMode(FAN_out,OUTPUT);//ВЫХОД D10 УПРАВЛЯЕМ НАПРЯЖЕНИЕМ НА ВЕНТИЛЯТОРЕ
pinMode(BUZZER_out,OUTPUT);//ВЫХОД D11 ПЬЕЗОПИЩАЛКА (tone() РАБОТАЕТ НА ПИНАХ 3 И 11 ТАЙМЕРА №2)
//pinMode(12,OUTPUT); //ВЫХОД ДЛЯ ВЫВОДА "DIN" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
pinMode(RELAY_out,OUTPUT);//ВЫХОД D13 СИМИСТОР ТВЕРДОТЕЛЬНОГО РЕЛЕ ПО ПИТАНИЮ ТЭН
//pinMode(A0,INPUT);  //ВХОД А0 ДЛЯ ВЫЧИСЛЕНИЯ ТЕМПЕРАТУРЫ ЧЕРЕЗ НАПРЯЖЕНИЕ С ТЕРМОПАРЫ
pinMode(GERKON_key,INPUT_PULLUP);//D15 ВХОД А1 ДЛЯ ОТСЛЕЖИВАНИЯ СОСТОЯНИЯ ГЕРКОНА В РУЧКЕ
pinMode(POWER_key,INPUT_PULLUP);//D16 ВХОД А2 ДЛЯ КНОПКИ ВКЛЮЧЕНИЯ POWER ON


//ОТКЛЮЧАЕМ ИНДИКАТОРЫ И ВЫХОДЫ УПРАВЛЯЮЩИЕ СИЛОВЫМИ ЭЛЕМЕНТАМИ ФЕНА ПРИ СТАРТЕ
digitalWrite(HOT_PWR_LED_out,LOW);//СИМИСТОР УПРАВЛЯЮЩИЙ НАГРУЗКОЙ
digitalWrite(POWER_LED_out,LOW);//ИНДИКАТОР ВКЛЮЧЕНИЯ
digitalWrite(FAN_out,LOW);//ВЕНТИЛЯТОР
digitalWrite(RELAY_out,LOW);//РЭЛЕ ВКЛЮЧЕНИЯ ФЕНА


myPID.SetMode(AUTOMATIC);myPID.SetOutputLimits(0,255);//ЗАДАЁМ ПАРАМЕТРЫ ПИД
analogReference(DEFAULT);//ВКЛЮЧАЕМ ВНУТРЕННЕЕ ОПОРНОЕ НАПРЯЖЕНИЕ 5,0 ВОЛЬТ (1,1В INTERNAL)


///////////////////////// ЧИТАЕМ ЗНАЧЕНИЯ ПЕРЕМЕННЫХ ИЗ EEPROM ///////////////////////////////
//ПОСЛЕДНЯЯ ЗАДАННАЯ ТЕМПЕРАТУРА (130-390 ГРАДУСОВ)
temp00=EEPROM.read(0);temp01=EEPROM.read(1);temp=word(temp00,temp01);
//КОЭФФИЦИЕНТ ИНТЕГРИРОВАНИЯ (0.00-9.99)
KiSet10=EEPROM.read(2);KiSet11=EEPROM.read(3);KiSet=word(KiSet10,KiSet11);
//КОЭФФИЦИЕНТ ПРОПОРЦИОНАЛЬНОСТИ (0.00-9.99)
KpSet10=EEPROM.read(4);KpSet11=EEPROM.read(5);KpSet=word(KpSet10,KpSet11);
//КОЭФФИЦИЕНТ ДИФФЕРЕНЦИРОВАНЯ (0.00-9.99)
KdSet10=EEPROM.read(6);KdSet11=EEPROM.read(7);KdSet=word(KdSet10,KdSet11);
fan=EEPROM.read(8);//ПОТОК ВОЗДУХА ОТ ТУРБИНЫ (8 ЗНАЧЕНИЙ)
led=EEPROM.read(9);//ПОКАЗЫВАТЬ ТЕМПЕРАТУРУ В РЕЖИМЕ "POWER OFF" (ДА / НЕТ)
sharp=EEPROM.read(10);//ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ (5 ИЛИ 10 ГРАДУСОВ)
wait=EEPROM.read(11);//ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ (3 - 30 СЕКУНД)
down=EEPROM.read(12);//ВРЕМЯ ЗАДЕРЖКИ ДО УХОДА В "POWER OFF" ЕСЛИ ФЕН НА БАЗЕ (1 - 99 МИНУТ)
bright=EEPROM.read(13);//ЯРКОСТЬ ИНДИКАТОРА (0 - 15 ПОПУГАЕВ)


LC.shutdown(0,false);//ОТКЛЮЧАЕМ РЕЖИМ ЭНЕРГОСБЕРЕЖЕНИЯ МАХ7219
LC.setIntensity(0,bright);//УСТАНАВЛИВАЕМ ИНТЕНСИВНОСТЬ СВЕЧЕНИЯ ОТ 0 ДО 15 МАХ7219
LC.clearDisplay(0);//ОЧИЩАЕМ ОТ МУСОРА ПАМЯТЬ МАХ7219


//ЗАДАЁМ КОЭФФИЦИЕНТЫ ПИД ПОСЛЕ ЧТЕНИЯ ИЗ ПАМЯТИ ИЗ int16_t В ТИП float (double) 
Kp=(double)KpSet/100.0;Ki=(double)KiSet/100.0;Kd=(double)KdSet/100.0;


}//END SETUP 




/////////////////////////////////////////////////////////////////////////////////////////////
void loop(){


newPos=myEnc.read()/4;//ЧИТАЕМ ПОЛОЖЕНИЕ ЭНКОДЕРА 
mill=millis();//ПЕРЕЗАПИСЫВАЕМ ПЕРЕМЕННУЮ "mill" ОПЕРИРУЯ ЕЮ ВМЕСТО ФУНКЦИИ


//КНОПКА ВКЛЮЧЕНИЯ POWER ON/OFF ЛОГИЧЕСКИЙ "0" НАЖАТА / "1" НЕ НАЖАТА
btn=digitalRead(POWER_key);
if(btn!=POWER_btn){delay(30);btn=digitalRead(POWER_key);
if(!btn&&POWER_btn){power=!power;FL0=true;}
POWER_btn=btn;}




//////////////////////////////////////////// POWER ON ////////////////////////////////////////////
if(power==true){
//ЗАДАЁМ НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ РЕЖИМА "POWER ON"
if(FL0==true){
FL2=true;FL5=true;fr=5;menu=0;clful();
LC.clearDisplay(0);LC.shutdown(0,false);//ЧИСТИМ ПАМЯТЬ У ИНДИКАТОРА И ВКЛЮЧАЕМ ЕГО
digitalWrite(RELAY_out,HIGH);//ПОДАЁМ НАПРЯЖЕНИЕ НА ВТОРОЙ КОНТАКТ ТЭН И БП ВЕНТИЛЯТОРА
FL0=false;}


//ПОДАЁМ МНОГОТОНАЛЬНЫЙ ЗВУКОВОЙ СИГНАЛ ВКЛЮЧЕНИЯ ТЕРМОФЕНА
if(fr>0&&mill-timer_BUZZER>p100){timer_BUZZER=mill;tone(BUZZER_out,freq[fr],p100);fr--;}


temperatura();//ПОСТОЯННО ВЫЧИСЛЯЕМ ТЕКУЩУЮ ТЕМПЕРАТУРУ 
In_TEMP=Input;//ПЕРЕВОДИМ ЗНАЧЕНИЕ ПЕРЕМЕННОЙ ФОРМАТА "double" В ФОРМАТ "int" 


//МИНИМАЛЬНАЯ ЗАЩИТА ОТ ПЕРЕГРЕВА ТЭН, ТЕРМОПАРА ПОВРЕЖДЕНА, ОБРЫВ КАБЕЛЯ, ПЛОХОЙ КОНТАКТ И Т.Д.\
ПРОВЕРЯЕМ ЗНАЧЕНИЯ С ПОРТА "А0" НА ЗАВЕДОМО ЗАВЫШЕННОЕ, ЕСЛИ ПРЕВЫШАЕТ ТО ПЕРЕХОДИМ В "POWER OFF"
if(inputdata>maxin){power=false;FL0=true;}


//КОНТРОЛЬ ДАТЧИКА ПОЛОЖЕНИЯ (ГЕРКОНА) В РУЧКЕ ТЕРМОФЕНА (HIGH - ФЕН В РУКЕ, LOW - ФЕН НА БАЗЕ)
btn=digitalRead(GERKON_key);
if(btn!=GERKON_btn){FL9=true;GERKON_btn=btn;
timer_GERKON=mill+p100;}//ЕСЛИ СОСТОЯНИЕ ГЕРКОНА ИЗМЕНИЛОСЬ, УСТАНАВЛИВАЕМ ЗАДЕРЖКУ 100мс 
if(FL9==true&&mill>timer_GERKON){//ЕСЛИ СПУСТЯ 100мс РУЧКА ПРОДОЛЖАЕТ НАХОДИТЬСЯ ВНЕ БАЗЫ 
if(GERKON_btn){GERKON_flag=true;}//ФЕН В РАБОТЕ
else{GERKON_flag=false;}//ФЕН НА БАЗЕ
FL2=true;clmil();
FL9=false;}


//////////////////////////// ЕСЛИ ФЕН НА БАЗЕ, СТАВИМ ЕГО НА ПАУЗУ //////////////////////////////
if(GERKON_flag==false){

if(FL2==true){FL5=true;FL3=true;
tone(BUZZER_out,1400,300);//ПОДАЁМ ЗВУКОВОЙ СИГНАЛ РЕЖИМА "ПАУЗА" 
detachInterrupt(0);//ОТКЛЮЧАЕМ ОТСЛЕЖИВАНИЕ ПРЕРЫВАНИЯ 0 НА pin D2
digitalWrite(HOT_PWR_LED_out,LOW);//БЕЗОГОВОРОЧНО ОТКЛЮЧАЕМ НАГРЕВ ТЭН
FL2=false;}


//ЕСЛИ ТЭН ГОРЯЧИЙ, ВКЛЮЧАЕМ ВЕНТИЛЯТОР ДЛЯ ОХЛАЖДЕНИЯ СО СКОРОСТЬЮ ЗАВИСЯЩЕЙ ОТ ТЕМПЕРАТУРЫ
if(FL3==true&&In_TEMP>hot){
cold=map(In_TEMP,hot,temp,1,8);//ЧЕМ НИЖЕ ТЕМПЕРАТУРА ТЕМ МЕНЬШЕ СКОРОСТЬ 
cold=constrain(cold,1,8);analogWrite(FAN_out,pwm[cold]);fanspeed=cold;}
//ЕСЛИ ТЕМПЕРАТУРА ПАДАЕТ НИЖЕ БЕЗОПАСНОЙ ОТКЛЮЧАЕМ ВЕНТИЛЯТОР
else{digitalWrite(FAN_out,LOW);fanspeed=fan;
FL3=false;}


//МИГАЕМ ИНДИКАТОРОМ "POWER" СООБЩАЯ, ЧТО ФЕН НАХОДИТСЯ НА БАЗЕ В РЕЖИМЕ "ПАУЗА"
if(mill-timer_LED>(FL8?p1000:p400)){timer_LED=mill;FL8=!FL8;digitalWrite(POWER_LED_out,FL8);}


}//КОНЕЦ РЕЖИМА "ПАУЗА"



///////////////////////////// ЕСЛИ ФЕН В РУКЕ, ВКЛЮЧАЕМ ЯДРО ФЕНА /////////////////////////////
if(GERKON_flag==true){
if(FL2==true){FL5=true;
tone(BUZZER_out,2400,300);//ПОДАЁМ ТОНАЛЬНЫЙ СИГНАЛ ВКЛЮЧЕНИЯ РЕЖИМА "РАБОТА"
attachInterrupt(0,tracking,RISING);//ПЕРЕДАЁМ УПРАВЛЕНИЕ ФУНКЦИЕЙ tracking ПРЕРЫВАНИЮ 0
digitalWrite(POWER_LED_out,HIGH);//ВКЛЮЧАЕМ ИНДИКАТОР "POWER" СООБЩАЯ, ЧТО ФЕН В РАБОТЕ
myPID.SetTunings(Kp,Ki,Kd);//УСТАНАВЛИВАЕМ КОЭФФИЦИЕНТЫ ПИД
FL2=false;}


/////////////////////////////////// УПРАВЛЯЕМ ТЭН И ВЕНТИЛЯТОРОМ /////////////////////////////
Setpoint=temp;//ОТСЫЛАЕМ ЗАДАННУЮ ТЕМПЕРАТУРУ В ПИД
myPID.Compute();//ПРОПОРЦИОНАЛЬНО-ИНТЕГРАЛЬНОЕ ДИФФЕРЕНЦИРОВАНИЕ ТЕМПЕРАТУРЫ (ПИД)
triac=Output;//РЕЗУЛЬТАТ РАБОТЫ ПИД ИСПОЛЬЗУЕМ В ФУНКЦИИ tracking
//ПОДАЁМ НАПРЯЖЕНИЕ НА ВЕНТИЛЯТОР В СООТВЕТСТВИИ С ЗАДАННЫМ НОМЕРОМ ИЗ МАССИВА ЗНАЧЕНИЙ pwm[]
analogWrite(FAN_out,pwm[fan]);


}//КОНЕЦ РЕЖИМА "РАБОТА"



/////////////////////////////////////////// ГЛАВНОЕ МЕНЮ /////////////////////////////////////////
if(menu==0){
//ЗАПОЛНЯЕМ ПУСТЫМИ СИМВОЛАМИ НЕИСПОЛЬЗУЕМЫЕ В ДАННОМ МЕНЮ СЕГМЕНТЫ ИНДИКАТОРА
if(FL5==true){
clful();dot0();ar4=ar5=' ';m0=0;fanspeed=fan;
FL5=false;}


//НАЖАТИЕМ НА ЭНКОДЕР ВЫБИРАЕМ ТЕМПЕРАТУРУ ИЛИ ПОТОК ВОЗДУХА
btn=digitalRead(MENU_key);
if(btn!=MENU_btn){delay(10);btn=digitalRead(MENU_key);
if(btn==LOW&&MENU_btn==HIGH){++m0;if(m0>1){m0=0;}clmil();timer_BTN=mill;FL4=true;}
MENU_btn=btn;}

if(FL1==true){
switch(m0){
  case 0: trig0=temp;break;
  case 1: trig0=fan;break;}
FL1=false;}


//МЕНЯЕМ ЗНАЧЕНИЯ ТЕМПЕРАТУРЫ ИЛИ ПОТОКА ВОЗДУХА 
if(newPos!=oldPos){
if(m0==0){trig0=trig0+(newPos*sharp);}else{trig0=trig0+newPos;}clful();
switch (m0){
  case 0: if(trig0>=minimtemp){temp=constrain(trig0,minimtemp,maxtemp);}
else{if(trig0<temp){trig0=0;}else{trig0=minimtemp;}temp=trig0;};break;
  case 1: fan=constrain(trig0,1,8);break;}
fanspeed=fan;timer_MENU=mill-p500;}


//ВЫВОДИМ ЗНАЧЕНИЯ НА ИНДИКАТОР КАЖДЫЕ 0,5 СЕК (чаще нет необходимости)
if(mill-timer_MENU>p500){timer_MENU=mill;
//РАЗБИВАЕМ ПО РАЗРЯДАМ ТЕКУЩУЮ ВЫЧИСЛЕННУЮ ТЕМПЕРАТУРУ
ar1=In_TEMP/100%10;if(ar1==0){ar1=' ';}ar2=In_TEMP/10%10;ar3=In_TEMP%10;
if(m0==0){
ar6=temp/100%10;ar7=temp/10%10;ar8=temp%10;}//РАЗБИВАЕМ ПО РАЗРЯДАМ ЗАДАННУЮ ТЕМПЕРАТУРУ, ЛИБО
else{ar6='F';ar7=' ';ar8=fan;}//СКОРОСТЬ ВЕНТИЛЯТОРА
dotfan();menu0();}//ВЫЗЫВАЕМ ФУНКЦИИ ОТОБРАЖЕНИЯ СКОРОСТИ В ВИДЕ ТОЧЕК, ЦИФРЫ И СИМВОЛЫ


//ЕСЛИ ФЕН НИКАК НЕ ИСПОЛЬЗУЕТСЯ ЗАДАННЫЙ ИНТЕРВАЛ УХОДИМ В РЕЖИМ "STANDBY"
if(GERKON_flag==false&&mill-timer_autoret>(down*p60000)){power=false;FL0=true;}


}//END MENU 0




////////////////////////////////// МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ /////////////////////////////////////
// 1 ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ (5 ИЛИ 10 ГРАДУСОВ)
// 2 ВРЕМЯ ДО АВТООТКЛЮЧЕНИЯ ФЕНА ПРИ ОТСУТСТВИИ АКТИВНОСТИ (ОТ 1 ДО 99 МИНУТ)
// 3 ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ (ОТ 3 ДО 30 СЕКУНД)
// 4 КОЭФФИЦИЕНТ ПРОПОРЦИОНАЛЬНОСТИ (0.00-9.99)
// 5 КОЭФФИЦИЕНТ ИНТЕГРИРОВАНИЯ (0.00-9.99)
// 6 КОЭФФИЦИЕНТ ДИФФЕРЕНЦИРОВАНИЯ (0.00-9.99)
// 7 ЯРКОСТЬ СВЕТОДИОДНОГО ИНДИКАТОРА (0 - 15)
if(menu==1){
if(FL5==true){ar2=ar3=ar4=ar5=ar6=ar7=' ';//ЧИСТИМ ПОЗИЦИИ ДО ОТОБРАЖЕНИЯ МЕНЮ НАСТРОЙКИ
clmil();//СБРАСЫВАЕМ ТАЙМЕР
FL6=true;dot8();tone(BUZZER_out,3000,100);
FL5=false;}


//ПРИ НАЖАТИИ НА ЭНКОДЕР В МЕНЮ НАСТРОЙКИ, ВЫБИРАЕМ РЕЖИМ РЕДАКТИРОВАНИЯ ЛИБО ПРОКРУТКИ ПАРАМЕТРОВ
btn=digitalRead(MENU_key);
if(btn!=MENU_btn){delay(10);btn=digitalRead(MENU_key);
if(btn==LOW&&MENU_btn==HIGH){timer_MENU=mill;timer_BTN=mill;clmil();FL4=true;FL6=!FL6;
if(FL6==true){dot8();}//ДЛЯ ПРОКРУТКИ ЗАЖИГАЕМ ТОЧКИ
if(FL6==false){dot0();}}//ДЛЯ РЕДАКТИРОВАНИЯ ГАСИМ
MENU_btn=btn;}


//ОБРАБАТЫВАЕМ ПОВОРОТЫ ЭНКОДЕРА
if(FL6==true&&newPos!=oldPos){//МЕНЯЕМ ПУНКТЫ МЕНЮ
m1=m1+newPos;if(m1>6){m1=0;}if(m1<0){m1=6;}clful();ar2=ar3=ar4=ar5=ar6=ar7=' ';}


if(FL6==false&&newPos!=oldPos){//МЕНЯЕМ ПАРАМЕТРЫ ВНУТРИ ПУНКТА МЕНЮ
trig1=trig1+newPos;clful();
switch (m1){
//ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ 5 ИЛИ 10 ГРАДУСОВ
case 0: if(trig1>1){trig1=0;}if(trig1<0){trig1=1;}sharp=zen[trig1];break;
case 1: down=constrain(trig1,1,99);break;//ВРЕМЯ ОТКЛЮЧЕНИЯ ФЕНА ПРИ ОТСУТСТВИИ АКТИВНОСТИ
case 2: wait=constrain(trig1,3,30);break;//ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ
case 3: KpSet=constrain(trig1,0,999);Kp=(double)KpSet/100.0;break;
case 4: KiSet=constrain(trig1,0,999);Ki=(double)KiSet/100.0;break;
case 5: KdSet=constrain(trig1,0,999);Kd=(double)KdSet/100.0;break;
case 6: bright=constrain(trig1,0,15);LC.setIntensity(0,bright);break;}}//ЯРКОСТЬ ИНДИКАТОРА


if(FL1==true){menu1();menu0();FL1=false;}//ОТОБРАЖАЕМ НА ИНДИКАТОРЕ ВНЕСЁННЫЕ ИЗМЕНЕНИЯ


//АВТОВОЗВРАТ В ГЛАВНОЕ МЕНЮ ИЗ МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ ЧЕРЕЗ ЗАДАННЫЙ ИНТЕРВАЛ
if(mill-timer_autoret>wait*p1000){menu=0;FL5=true;}


}//END MENU 1


//ДЛЯ ВХОДА ИЛИ ВЫХОДА В/ИЗ МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ УДЕРЖИВАЕМ ЭНКОДЕР НЕ МЕНЕЕ 0,5 СЕК.
if(FL4==true&&MENU_btn==LOW&&mill-timer_BTN>p500){++menu;if(menu>1){menu=0;}FL5=true;
FL4=false;}


}//END POWER ON






///////////////////////////////////// POWER OFF (STANDBY) ////////////////////////////////////////
if(power==false){


//ЗАДАЁМ ПАРАМЕТРЫ ДЛЯ РЕЖИМА "STANDBY", СОХРАНЯЕМ ПЕРЕМЕННЫЕ В ПАМЯТЬ
if(FL0==true){fr=0;menu=0;FL5=true;LC.clearDisplay(0);EEPROM_UPDATE();
FL0=false;}


//ПОДАЁМ ЗВУКОВОЙ СИГНАЛ ОТКЛЮЧЕНИЯ ФЕНА
if(fr<5&&mill>timer_BUZZER+p100){timer_BUZZER=mill;tone(BUZZER_out,freq[fr],p100);fr++;}


/////////////////////////////////////// ОСНОВНОЕ МЕНЮ /////////////////////////////////////////
if(menu==0){
if(FL5==true){dot0();ar1=ar7=ar8=' ';triac=0;FL3=true;
digitalWrite(HOT_PWR_LED_out,LOW);//ОТКЛЮЧАЕМ СИМИСТОР УПРАВЛЕНИЯ ТЭН
digitalWrite(RELAY_out,LOW);//ОТКЛЮЧАЕМ СИМИСТОР УПРАВЛЯЮЩИЙ ОБЩИМ ПИТАНИЕМ ФЕНА
digitalWrite(POWER_LED_out,LOW);//ОТКЛЮЧАЕМ ИНДИКАТОР "POWER ON"
detachInterrupt(0);//ОТКЛЮЧАЕМ ОТСЛЕЖИВАНИЕ ПРЕРЫВАНИЯ 0 НА pin D2
(led?LC.shutdown(0,false):LC.shutdown(0,true));//ВКЛЮЧАЕМ ИЛИ ВЫКЛЮЧАЕМ ПОКАЗ ТЕМПЕРАТУРЫ
FL5=false;}


//ОБРАБАТЫВАЕМ НАЖАТИЕ НА ЭНКОДЕР
btn=digitalRead(MENU_key);if(btn!=MENU_btn){delay(10);btn=digitalRead(MENU_key);
if(btn==LOW&&MENU_btn==HIGH){led=!led;(led?LC.shutdown(0,false):LC.shutdown(0,true));
timer_BTN=mill+p1000;FL4=true;}
MENU_btn=btn;}


//ВЫВОДИМ ТЕКУЩУЮ ТЕМПЕРАТУРУ НА ИНДИКАТОР КАЖДУЮ СЕКУНДУ
if(mill-timer_MENU>p1000){timer_MENU=mill;temperatura();In_TEMP=Input;
ar2=In_TEMP/100%10;if(ar2==0){ar2=' ';}ar3=In_TEMP/10%10;ar4=In_TEMP%10;
menuOFF();}


//ЕСЛИ ТЭН ГОРЯЧИЙ, ВКЛЮЧАЕМ ВЕНТИЛЯТОР
if(FL3==true&&In_TEMP>hot){temperatura();In_TEMP=Input;
digitalWrite(RELAY_out,HIGH);cold=map(In_TEMP,hot,temp,1,8);
cold=constrain(cold,1,8);analogWrite(FAN_out,pwm[cold]);}
else{digitalWrite(FAN_out,LOW);digitalWrite(RELAY_out,LOW);
FL3=false;}


}//END MENU 0




//ИНЖЕНЕРНОЕ МЕНЮ НАСТРОЙКИ УСИЛИТЕЛЯ ПОД ПАРАМЕТРЫ ТЕРМОПАРЫ 
if(menu==1){


if(FL5==true){digitalWrite(RELAY_out,HIGH);//ВКЛЮЧАЕМ ВЫХОД УПРАВЛЯЮЩИЙ ОБЩИМ ПИТАНИЕМ ФЕНА
clenc();dotfan();ar4=ar5=' ';LC.shutdown(0,false);tone(BUZZER_out,3000,100);
attachInterrupt(0,tracking,RISING);triac=0;analogWrite(FAN_out,pwm[fan]);
FL5=false;}


//ОБРАБАТЫВАЕМ НАЖАТИЕ НА ЭНКОДЕР
btn=digitalRead(MENU_key);
if(btn!=MENU_btn){delay(10);btn=digitalRead(MENU_key);
if(btn==LOW&&MENU_btn==HIGH){++m2;if(m2>1){m2=0;}FL4=true;timer_BTN=mill+p1000;}
MENU_btn=btn;}

temperatura();In_TEMP=Input;

//ДЛЯ НАСТРОЙКИ ТЕМПЕРАТУРЫ ПОЛУЧАЕМОЙ С ТЕРМОПАРЫ, УПРАВЛЯЕМ СИМИСТОРОМ НАПРЯМУЮ
if(newPos!=oldPos){
triac=triac+(newPos*sharp);clenc();
triac=constrain(triac,0,255);timer_MENU=mill+p500;}

if(mill-timer_MENU>p500){timer_MENU=mill;
if(m2==0){
//ВЫВОДИМ РЕЗУЛЬТАТ АЦП 0...1024 ОЦИФРОВКИ НАПРЯЖЕНИЯ С УСИЛИТЕЛЯ, ЛИБО... 
ar1=inputdata/1000%10;ar2=inputdata/100%10;ar3=inputdata/10%10;ar4=inputdata%10;}
//ЭТОТ ЖЕ РЕЗУЛЬТАТ ПРЕСЧИТАННЫЙ В ТЕМПЕРАТУРУ 20...390
else{ar1=' ';ar2=In_TEMP/100%10;if(In_TEMP<100){ar2=' ';}ar3=In_TEMP/10%10;ar4=In_TEMP%10;}
//ВЫВОДИМ ПРОЦЕНТ ЗАПОЛНЕНИЯ ШИМ НАПРЯМУЮ (БЕЗ ПИД) ДЛЯ НАГРЕВА ТЭН
ar6=(triac)/100%10;if(ar6==0){ar6=' ';}ar7=(triac)/10%10;ar8=(triac)%10;menu0();}


}//END MENU 1


//УДЕРЖИВАЯ КНОПКУ ЭНКОДЕРА 1 СЕК. И БОЛЕЕ ПЕРЕКЛЮЧАЕМСЯ МЕЖДУ ОСНОВНЫМ И ИНЖЕНЕРНЫМ МЕНЮ
if(FL4==true&&MENU_btn==LOW&&mill>timer_BTN){++menu;if(menu>1){menu=0;}
led=true;FL5=true;LC.clearDisplay(0);
FL4=false;}


}//////////////////////////////////////// END POWER OFF ///////////////////////////////////////////


}/////////////////////////////////////////// END LOOP /////////////////////////////////////////////
