////////////////////////////////////////////////////////////////////////////////////////////////
//                                      ФУНКЦИИ / FUNCTIONS                                   //
////////////////////////////////////////////////////////////////////////////////////////////////


//ФУНКЦИЯ ОБНУЛЕНИЯ ПОЛОЖЕНИЯ ЭНКОДЕРА
void EncodCL(){myEnc.write(0);oldPos=0;newPos=0;}



//ФУНКЦИЯ СЧИТЫВАЕТ АНАЛОГОВЫЙ ВХОД samples КОЛИЧЕСТВО РАЗ И ВОЗВРАЩАЕТ СРЕДНЕ-МЕДИАННОЕ ЗНАЧЕНИЕ\
ОТСЕИВАЯ ВСЕ ЗНАЧЕНИЯ ОТЛИЧНЫЕ ОТ НАИБОЛЕЕ ЧАСТО ВСТРЕЧАЮЩИХСЯ В ОДНОЙ ВЫБОРКЕ ИЗ samples ЧИСЕЛ
int16_t median(uint8_t samples){
int16_t raw[samples];//МАССИВ ДЛЯ ХРАНЕНИЯ ДАННЫХ
int16_t tem=0;uint8_t i,j;//ВРЕМЕННЫЕ ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
for(i=0;i<samples;i++){raw[i]=analogRead(A0);}//ЧИТАЕМ ДАННЫЕ С А0 И ПИШЕМ ИХ В ЯЧЕЙКИ МАССИВА
//СОРТИРУЕМ МАССИВ ПО ВОЗРАСТАНИЮ ЗНАЧЕНИЙ В ЯЧЕЙКАХ
for(i=0;i<samples;i++){
for(j=0;j<samples-1;j++){
//АНОМАЛЬНО БОЛЬШИЕ И МАЛЫЕ ОТПРАВЛЯЕМ В НАЧАЛО И КОНЕЦ МАССИВА
if(raw[j]>raw[j+1]){tem=raw[j];raw[j]=raw[j+1];raw[j+1]=tem;}}}
return raw[samples>>1];}//РЕГИСТРОВЫЙ СДВИГ >>1 ВОЗВРАЩАЕТ РЕЗУЛЬТАТ ДЕЛЕНИЯ НА 2 \
И В СЛЕДУЮЩЕМ ПРОХОДЕ ПОВТОРЯЕМ ЦИКЛ ВЫБОРКИ ДЛЯ ПЕРЕМЕННОЙ median



//ВЫЧИСЛЯЕМ ТЕКУЩУЮ ТЕМПЕРАТУРУ
void TEMPERATURA(){
inputdata=median(15);//ЧИТАЕМ ОТФИЛЬТРОВАННЫЕ СРЕДНЕ-МЕДИАННЫЕ ЗНАЧЕНИЯ С ПОРТА "А0"
//ПЕРЕВОДИМ ДАННЫЕ С АЦП В ТЕМПЕРАТУРУ
Input_TEMP=Input=map(inputdata,0,MAXIMUM_input_ADC,MIN_temp,MAX_temp);}
//ЗНАЧЕНИЕ ТЕМПЕРАТУРЫ ПЕРЕНОСИМ В 2 ПЕРЕМЕННЫЕ РАЗНЫХ ФОРМАТОВ "int" И "float" (double)



//ФУНКЦИЯ ОБРАБОТКИ ПРЕРЫВАНИЯ 0 ЧЕРЕЗ attachInterrupt, \
ОТСЛЕЖИВАЕТ ПЕРЕХОД СИНУСОИДЫ ЧЕРЕЗ "0" И ВКЛЮЧАЕТ ТЭН ВЫПОЛНЯЯ digitalWrite НА D6 ВЫХОДЕ \
ПРОПУСКАЯ НЕКОТОРОЕ КОЛЛИЧЕСТВО ПЕРЕХОДОВ СИНУСОИДЫ В ЗАВИСИМОСТИ ОТ ЗНАЧЕНИЯ triac \
255 - НЕ ПРОПУСКАЕМ НИ ОДНОГО, 0 - ПРОПУСКАЕМ ВСЕ
void tracking(){
static byte count,last,lastVal;//ВРЕМЕННЫЕ ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
int val=(++count*triac)>>8;//РЕГИСТРОВЫЙ СДВИГ x>>8 АНАЛОГИЧНО x/(2^8) АНАЛОГИЧНО x/256
if(lastVal!=(val!=last)){digitalWrite(HOT_PWR_LED_out,val!=last);}lastVal=(val!=last);last=val;}


  
//ПЕЧАТАЕМ ЗНАЧЕНИЯ В ОСНОВНОМ МЕНЮ setDigit setChar setRow
void ShowMENU(){
  LC.setChar(0,7,Digit1,LD1);
  LC.setChar(0,6,Digit2,LD2);
  LC.setChar(0,5,Digit3,LD3);
  LC.setChar(0,4,Digit4,LD4);
  LC.setChar(0,3,Digit5,LD5);
  LC.setChar(0,2,Digit6,LD6);
  LC.setChar(0,1,Digit7,LD7);
  LC.setChar(0,0,Digit8,LD8);}



//ПОДГОТАВЛИВАЕМ ВЫБРАННЫЕ ЗНАЧЕНЕИЯ ДЛЯ ВЫВОДА НА ИНДИКАТОР
void menu1(){ 
switch (m1){
case 0: for(uint8_t i=0;i<2;i++){if(StepValue==zen[i]){trig1=i;}}
Digit1=1;
Digit7=StepValue/10%10;if(Digit7==0){Digit7=' ';}
Digit8=StepValue%10;break;
case 1: trig1=DownTime;
Digit1=2;
Digit7=DownTime/10%10;if(Digit7==0){Digit7=' ';}
Digit8=DownTime%10;break;
case 2: trig1=WaitAutoRet;
Digit1=3;
Digit7=WaitAutoRet/10%10;if(Digit7==0){Digit7=' ';}
Digit8=WaitAutoRet%10;break;
case 3: trig1=KpSet;
Digit1=4;
Digit6=KpSet/100%10;LD6=true;
Digit7=KpSet/10%10;
Digit8=KpSet%10;break;
case 4: trig1=KiSet;
Digit1=5;
Digit6=KiSet/100%10;LD6=true;
Digit7=KiSet/10%10;
Digit8=KiSet%10;break;
case 5: trig1=KdSet;
Digit1=6;
Digit6=KdSet/100%10;LD6=true;
Digit7=KdSet/10%10;
Digit8=KdSet%10;break;
case 6: trig1=LED_Bright;
Digit1=7;
Digit7=LED_Bright/10%10;if(Digit7==0){Digit7=' ';}
Digit8=LED_Bright%10;break;}}



//ПЕЧАТАЕМ ЗНАЧЕНИЯ ТЕМПЕРАТУРЫ В РЕЖИМЕ "POWER OFF" 
void menuOFF(){ 
 LC.setChar(0,6,Digit2,LD2);
 LC.setChar(0,5,Digit3,LD3);
 LC.setChar(0,4,Digit4,LD4);
  LC.setRow(0,3,B01100011);//СПЕЦСИМВОЛ ГРАДУСА
  LC.setRow(0,2,B01001110);}//СПЕЦСИМВОЛ "С"


//ВЫБИРАЕМ ФУНКЦИЮ ОТОБРАЖЕНИЯ ТОЧЕК ИНДИЦИРУЮЩИХ СКОРОСТЬ ВЕНТИЛЯТОРА
void DotFan(){
  switch (FanSpeed){
    case 1: dot1();break;
    case 2: dot2();break;
    case 3: dot3();break;
    case 4: dot4();break;
    case 5: dot5();break;
    case 6: dot6();break;
    case 7: dot7();break;
    case 8: dot8();break;}}


//ПАКЕТ ФУНКЦИЙ ДЛЯ ИНДИКАЦИИ ТОЧЕК (ничего красивее не придумал...)
void dot8(){LD1=LD2=LD3=LD4=LD5=LD6=LD7=LD8=true;}
void dot7(){LD1=LD2=LD3=LD4=LD5=LD6=LD7=true;LD8=false;}
void dot6(){LD1=LD2=LD3=LD4=LD5=LD6=true;LD7=LD8=false;}
void dot5(){LD1=LD2=LD3=LD4=LD5=true;LD6=LD7=LD8=false;}
void dot4(){LD1=LD2=LD3=LD4=true;LD5=LD6=LD7=LD8=false;}
void dot3(){LD1=LD2=LD3=true;LD4=LD5=LD6=LD7=LD8=false;}
void dot2(){LD1=LD2=true;LD3=LD4=LD5=LD6=LD7=LD8=false;}
void dot1(){LD1=true;LD2=LD3=LD4=LD5=LD6=LD7=LD8=false;}
void dot0(){LD1=LD2=LD3=LD4=LD5=LD6=LD7=LD8=false;}




//ПЕРЕЗАПИСЫВАЕМ ЗНАЧЕНИЯ В EEPROM, НО ТОЛЬКО ЕСЛИ ОНИ ИЗМЕНИЛИСЬ В РЕЖИМЕ "POWER ON"
//(EEPROM.update / EEPROM.write) 
void EEPROM_UPDATE(){
if(SET_Temp!=SET_TempMOD){
temp00=highByte(SET_Temp);EEPROM.write(0,temp00);//ПИШЕМ СТАРШИЙ БАЙТ ПЕРЕМЕННОЙ SET_Temp
temp01=lowByte(SET_Temp);EEPROM.write(1,temp01);//ПИШЕМ МЛАДШИЙ БАЙТ ПЕРЕМЕННОЙ SET_Temp
SET_TempMOD=SET_Temp;}//ОБНОВЛЯЕМ ЗНАЧЕНИЕ ВРЕМЕННОЙ ПЕРЕМЕННОЙ
if(KiSet!=KiSetMOD){
KiSet10=highByte(KiSet);KiSet11=lowByte(KiSet);EEPROM.write(2,KiSet10);EEPROM.write(3,KiSet11);
KiSetMOD=KiSet;}
if(KpSet!=KpSetMOD){
KpSet10=highByte(KpSet);KpSet11=lowByte(KpSet);EEPROM.write(4,KpSet10);EEPROM.write(5,KpSet11);
KpSetMOD=KpSet;}
if(KdSet!=KdSetMOD){
KdSet10=highByte(KdSet);KdSet11=lowByte(KdSet);EEPROM.write(6,KdSet10);EEPROM.write(7,KdSet11);
KdSetMOD=KdSet;}
if(AirFlow!=AirFlowMOD){EEPROM.write(8,AirFlow);AirFlowMOD=AirFlow;}
if(LEDshow!=LEDshowMOD){EEPROM.write(9,LEDshow);LEDshowMOD=LEDshow;}
if(StepValue!=StepValueMOD){EEPROM.write(10,StepValue);StepValueMOD=StepValue;}
if(WaitAutoRet!=WaitAutoRetMOD){EEPROM.write(11,WaitAutoRet);WaitAutoRetMOD=WaitAutoRet;}
if(DownTime!=DownTimeMOD){EEPROM.write(12,DownTime);DownTimeMOD=DownTime;}
if(LED_Bright!=LED_BrightMOD){EEPROM.write(13,LED_Bright);LED_BrightMOD=LED_Bright;}}

//////////////////////////////////////////////////////////////////////////////////////////////
