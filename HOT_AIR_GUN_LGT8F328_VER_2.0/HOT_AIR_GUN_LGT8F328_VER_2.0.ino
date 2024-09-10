/////////////////////////////////////////////////////////////////////////////////////////////////
//                                    ТЕРМОФЕН / HOT AIR GUN                                   //
//                    ТЕРМОВОЗДУШНАЯ ПАЯЛЬНАЯ СТАНЦИЯ / SMD REWORK STATION                     //
//                                     ВЕРСИЯ (VERSION) 2.0                                    //
//                    СКЕТЧ АНАТОЛИЯ НЕВЗОРОВА / CODE BY ANATOLY NEVZOROFF                     //
//                             https://github.com/AnatolyNevzoroff                             //
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//Для поддержки Arduino IDE МК семейства LGT8F's необходимо:
//ВАРИАНТ 1. 
//В настройках Arduino IDE в окошко “Дополнительные ссылки для менеджера плат” Вставить:
// https://raw.githubusercontent.com/dbuezas/lgt8fx/master/package_lgt8fx_index.json 
//В Инструменты/Плата/Менеджер плат… выбрать и установить LGT8fx Boards
//ВАРИАНТ 2. Качаем архив:
//https://github.com/LGTMCU/Larduino_HSP/archive/master.zip
//Распаковываем каталог hardware в каталог установки Arduino IDE (там такой уже есть, 
//фактически добавляем туда каталог LGT), и перезапускаем Arduino IDE 
//More info https://github.com/LGTMCU/Larduino_HSP
//В качестве AVR BOARD в платах указываем: LGT8F328P-LQFP32
//Драйвер для USB to UART Bridge IC (HT42B534-1 EOL) Support Windows 7/8/8.1
//https://www.holtek.com/documents/10179/116677/USBBridgeSetup_CA.zip
//More info https://www.holtek.com/productdetail/-/vg/42b534-x
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//                                   ПОДКЛЮЧАЕМЫЕ БИБЛИОТЕКИ                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////
#include <EEPROM.h>//ЧТЕНИЕ И ЗАПИСЬ ПЕРЕМЕННЫХ В ЭНЕРГОНЕЗАВИСИМУЮ ПАМЯТЬ EEPROM
#include <Encoder.h>//ОБРАБОТКА УГЛА ПОВОРОТА ЭНКОДЕРА
#include <PID_v1.h>//https://github.com/br3ttb/Arduino-PID-Library  \
БИБЛИОТЕКА ПРОПОРЦИОНАЛЬНО-ИНТЕГРАЛЬНОГО ДИФФЕРЕНЦИРОВАНИЯ ДЛЯ ИНЕРТНЫХ СИСТЕМ
#include <LedControl.h>//https://github.com/wayoda/LedControl  \
БИБЛИОТЕКА ДЛЯ ДРАЙВЕРОВ MAX7219 И MAX7221 СВЕТОДИОДНЫХ ИНДИКАТОРОВ И ПАНЕЛЕЙ



/////////////////////////////////////////////////////////////////////////////////////////////////
//         ОБЪЯВИВ LOG_ENABLE ОРГАНИЗУЕМ ВЫВОД ДАННЫХ В СЕРИЙНЫЙ ПОРТ ДЛЯ ОТЛАДКИ КОДА         //
//                ПЕРЕД ОКОНЧАТЕЛЬНОЙ КОМПИЛЯЦИЕЙ ЗАКОМЕНТИРОВАТЬ ИЛИ УДАЛИТЬ                  //
/////////////////////////////////////////////////////////////////////////////////////////////////
// #define LOG_ENABLE 


//БОЛЬШИЕ ЦИФРЫ ДЛЯ millis()
#define p60000 60000L //ИНТЕРВАЛ - МИНУТА
#define p1000 1000L //ИНТЕРВАЛ 1000 mS (1 СЕКУНДА), ТИП ДАННЫХ "L" (long)
#define p500 500L
#define p200 200L
#define p100 100L
#define p50 50L

#define MENU_key 5        //D5 ВХОД КНОПКА МЕНЮ (КНОПКА ЭНКОДЕРА SW)
#define HOT_PWR_LED_out 6 //D6 ВЫХОД УПРАВЛЕНИЯ СИМИСТОРОМ НА ТЭН И ИНДИКАТОР (Оранжевый)
#define POWER_LED_out 9   //D9 ВЫХОД ИНДИКАТОР ВКЛЮЧЕНИЯ POWER ON (Красный)
#define FAN_out 10        //D10 ВЫХОД ДЛЯ УПРАВЛЕНИЯ НАПРЯЖЕНИЕМ НА ВЕНТИЛЯТОРЕ ТУРБИНЫ
#define BUZZER_out 11     //D11 ВЫХОД ДЛЯ ПЬЕЗОПИЩАЛКИ 
#define RELAY_out 13      //D13 ВЫХОД НА СИМИСТОР ЗАЩИТЫ ТЭНа И БЛОКА ПИТАНИЯ ВЕНТИЛЯТОРА 24В
#define GERKON_key 15     //D15 ВХОД А1 МАГНИТНЫЙ ДАТЧИК "ГЕРКОН" ОТСЛЕЖИВАЕМ ПОЛОЖЕНИЕ РУЧКИ
#define POWER_key 16      //D16 ВХОД А2 КНОПКА ВКЛЮЧЕНИЯ/ОТКЛЮЧЕНИЯ "POWER ON/OFF"


#define Safe_HOT 44 //РАЗУМНО-ДОСТАТОЧНАЯ ТЕМПЕРАТУРА ОХЛАЖДЕНИЯ ТЭНа НА БАЗЕ
//ПО ДОСТИЖЕНИЮ Safe_HOT ОТКЛЮЧАЕМ ВЕНТИЛЯТОР, НО ФАКТИЧЕСКАЯ ТЕМПЕРАТУРА СТАЛЬНОЙ 
//ПОВЕРХНОСТИ ТЕРМОФЕНА БУДЕТ ВЫШЕ, ПРИМЕРНО 60 ГРАДУСОВ, ЧТО ВПОЛНЕ БЕЗОПАСНО
#define MIN_temp 20    //ГРАНИЦА МИНИМАЛЬНО-ВОЗМОЖНОЙ ТЕМПЕРАТУРЫ ФЕНА
#define MAX_temp 380   //ГРАНИЦА МАКСИМАЛЬНО-ВОЗМОЖНОЙ ТЕМПЕРАТУРЫ ФЕНА
#define MINIMUM_temp 130 //ГРАНИЦА МИНИМАЛЬНО-РЕГУЛИРУЕМОЙ ТЕМПЕРАТУРЫ ФЕНА 

//ДЛЯ ЗАЩИТЫ ФЕНА ОТ ПРЕГРЕВА, В ИНЖЕНЕРНОМ МЕНЮ, ПРИ ОТКЛЮЧЁННОЙ РУЧКЕ ФЕНА (ИММИТИРУЕМ 
//ОБРЫВ ТЕРМОПАРЫ ИЛИ ПЕРЕГРЕВ) ОПРЕДЕЛЯЕМ МАКСИМАЛЬНО ВОЗМОЖНОЕ ЗНАЧЕНИЕ ВЫДАВАЕМОЕ АЦП, 
//ОНО ЗАВИСИТ ОТ РАБОТЫ УСИЛИТЕЛЯ И ОПОРНОГО НАПРЯЖЕНИЯ (В НАШЕМ, ПРОСТЕЙШЕМ СЛУЧАЕ ЭТО 
//НАПРЯЖЕНИЕ ПИТАНИЯ MK) НАПРИМЕР, У МЕНЯ АЦП LGT8F328 БОЛЬШЕ 4064 НЕ ВЫДАЁТ, ПОЭТОМУ ДЛЯ 
//АВАРИЙНОГО ОТКЛЮЧЕНИЯ, "MAXIMUM_input_ADC" ВЫБРАНО НЕСКОЛЬКО МЕНЬШЕ, НАПРИМЕР: 4062 ИЛИ 1060
#define MAXIMUM_input_ADC 1014 //В ЛЮБОМ СЛУЧАЕ, ЗНАЧЕНИЕ ДОЛЖНО БЫТЬ МЕНЬШЕ: 2^12(bit)=4096



/////////////////////////////////////////////////////////////////////////////////////////////////
//                                  ИНИЦИАЛИЗАЦИЯ ОБОРУДОВАНИЯ                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////
Encoder myEnc(3,4);//DT и CLK ИЛИ S2 и S1 ВЫВОДЫ ЭНКОДЕРА
LedControl LC=LedControl(12,8,7,1);//12-DIN, 8-CLC, 7-CS, 1-ОДИН (ЕДИНСТВЕННЫЙ) ИНДИКАТОР


/////////////////////////////////////////////////////////////////////////////////////////////////
//                                     ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////
//unsigned long от 0 до 4294967295
//ТАЙМЕРЫ ДЛЯ ФУНКЦИИ millis()
uint32_t timer_BTN,timer_BUZZER,timer_MENU,timer_GERKON,timer_LED;
uint32_t mill,timer_autoret_MENU,timer_autoret_DOWN,timer_POWER_key;


//int от -32768 до 32767
int16_t triac,inputdata,SET_Temp,SET_TempMOD,Input_TEMP,temp_for_FAN;
int16_t KpSet,KpSetMOD,KiSet,KiSetMOD,KdSet,KdSetMOD,trig0,trig1,oldPos,newPos;
int16_t Frequency[5]={0,2077,1577,1077,577};//МАССИВ ЧАСТОТ ДЛЯ ФУНКЦИИ tone() 

//byte от 0 до 255
uint8_t temp00,temp01,KpSet10,KpSet11,KiSet10,KiSet11,KdSet10,KdSet11;
uint8_t SpeedCooling,AirFlow,AirFlowMOD,FanSpeed,StepValue,StepValueMOD,LED_Bright,LED_BrightMOD;
uint8_t Digit1,Digit2,Digit3,Digit4,Digit5,Digit6,Digit7,Digit8;//НОМЕРА РАЗРЯДОВ ИНДИКАТОРА
uint8_t zen[2]={5,10};//МАССИВ ДЛЯ УСТАНОВКИ ШАГА ТЕМПЕРАТУРЫ
//МАССИВ ЗНАЧЕНИЙ ШИМ ДЛЯ СКОРОСТИ ВЕНТИЛЯТОРА (подбираем индивидуально)
uint8_t PWM[9]={0,99,109,119,127,138,150,165,255};

//char от -128 до 127
int8_t INDEX_Freq,m0,m1,m2,DownTime,DownTimeMOD,WaitAutoRet,WaitAutoRetMOD;

//boolean от false до true
bool PowerStatus,LEDshow,LEDshowMOD,GERKON_flag,POWER_btn,POWER_btn_status;
bool GERKON_btn,GERKON_btn_status,MENU_btn,MENU_btn_status;
bool LD1,LD2,LD3,LD4,LD5,LD6,LD7,LD8;//ДЛЯ УПРАВЛЕНИЯ ПОДСВЕТКОЙ ТОЧЕК
bool FL_start_POWER=true,FL_start_MENU,FL_autoret_MENU,FL_start_COOLING,FL_ShowMenu;
bool FL_delay_MENU,FL_pause_MODE,MenuSwitch,FL_delay_GERKON,Wink,FL_delay_POWER_key;
bool MAIN_menu,SETUP_menu,STANDBY_menu,ENGINEERING_menu;



//ЗАДАЁМ ПЕРЕМЕННЫЕ ДЛЯ БИБЛИОТЕКИ ПИД (PID)
double Setpoint;//ТРЕБУЕМАЯ ТЕМПЕРАТУРА НА ВЫХОДЕ ИЗ СОПЛА ТЕРМОФЕНА
double Input;   //ДАННЫЕ О ТЕКУЩЕЙ ТЕМПЕРАТУРЕ
double Output;  //ЗНАЧЕНИЕ ОТ 0 ДО 255 ВЫДАВАЕМОЕ БИБЛИОТЕКОЙ В ЗАВИСИМОСТИ ОТ Setpoint И Input
double Kp;      //АГРЕССИВНОСТЬ НАБОРА ТЕМПЕРАТУРЫ Setpoint, ЧЕМ БОЛЬШЕ ТЕМ БЫСТРЕЕ
double Ki;      //"ПРЕДСКАЗЫВАЕТ" ЗНАЧЕНИЕ Input, ЧЕМ БОЛЬШЕ ТЕМ ИНЕРЦИОННЕЕ (ОСЛАБЛЯЕТ Kd)
double Kd;      //УСРЕДНЯЕТ, СГЛАЖИВАЕТ Output ПРЕДОТВРАЩАЕТ КОЛЕБАНИЯ
PID myPID(&Input,&Output,&Setpoint,Kp,Ki,Kd, DIRECT);//ИНИЦИАЛИЗИРУЕМ ПЕРЕМЕННЫЕ ДЛЯ PID





/////////////////////////////////////////////////////////////////////////////////////////////////
//                                          S E T U P                                          //
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

#ifdef LOG_ENABLE
Serial.begin(9600);
#endif

//pinMode(2,INPUT); //ВХОД ДЛЯ ОПРЕДЕЛЕНИЯ ПЕРЕХОДА СИНУСОИДЫ ЧЕРЕЗ "0"
pinMode(3,INPUT); //ПОВОРОТ ЭНКОДЕРА белый провод DT (S2) (ПИН ИНИЦИАЛИЗИРУЕТСЯ БИБЛИОТЕКОЙ)
pinMode(4,INPUT); //ПОВОРОТ ЭНКОДЕРА жёлтый провод CLK (S1) (ПИН ИНИЦИАЛИЗИРУЕТСЯ БИБЛИОТЕКОЙ)
pinMode(MENU_key,INPUT);//ВХОД D5 КНОПКА МЕНЮ (Кнопка SW энкодера)
pinMode(HOT_PWR_LED_out,OUTPUT);//ВЫХОД D6 УПРАВЛЕНИЕ СИМИСТОРОМ ТЭН И ИНДИКАТОРОМ (Оранжевый)
//pinMode(7,OUTPUT);//ВЫХОД ДЛЯ ВЫВОДА "LOAD/CS" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
//pinMode(8,OUTPUT);//ВЫХОД ДЛЯ ВЫВОДА "CLK" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
pinMode(POWER_LED_out,OUTPUT);//ВЫХОД D9 ИНДИКАТОР ВКЛЮЧЕНИЯ POWER ON/OFF (Красный светодиод)
pinMode(FAN_out,OUTPUT);//ВЫХОД D10 УПРАВЛЯЕМ НАПРЯЖЕНИЕМ НА ВЕНТИЛЯТОРЕ
pinMode(BUZZER_out,OUTPUT);//ВЫХОД D11 ПЬЕЗОПИЩАЛКА (tone() РАБОТАЕТ НА ПИНАХ 3 И 11 ТАЙМЕРА №2)
//pinMode(12,OUTPUT);//ВЫХОД ДЛЯ ВЫВОДА "DIN" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
pinMode(RELAY_out,OUTPUT);//ВЫХОД D13 СИМИСТОР ТВЕРДОТЕЛЬНОГО РЕЛЕ ПО ПИТАНИЮ ТЭН
//pinMode(A0,INPUT);//ВХОД А0 ДЛЯ ВЫЧИСЛЕНИЯ ТЕМПЕРАТУРЫ ЧЕРЕЗ НАПРЯЖЕНИЕ С ТЕРМОПАРЫ
pinMode(GERKON_key,INPUT);//D15 ВХОД А1 ДЛЯ ОТСЛЕЖИВАНИЯ СОСТОЯНИЯ ГЕРКОНА В РУЧКЕ
//РЕКОМЕНДУЮ ПИН D15 ПОДТЯНУТЬ РЕЗИСТОРОМ 10К К +5В И ЗАМЕНИТЬ НА pinMode(GERKON_key,INPUT);
pinMode(POWER_key,INPUT_PULLUP);//D16 ВХОД А2 ДЛЯ КНОПКИ ВКЛЮЧЕНИЯ POWER ON


//ОТКЛЮЧАЕМ ИНДИКАТОРЫ И ВЫХОДЫ УПРАВЛЯЮЩИЕ СИЛОВЫМИ ЭЛЕМЕНТАМИ ФЕНА ПРИ СТАРТЕ MK
digitalWrite(HOT_PWR_LED_out,LOW);//СИМИСТОР УПРАВЛЯЮЩИЙ НАГРУЗКОЙ
digitalWrite(POWER_LED_out,LOW);//ИНДИКАТОР ВКЛЮЧЕНИЯ
digitalWrite(FAN_out,LOW);//ВЕНТИЛЯТОР
digitalWrite(BUZZER_out,LOW);//ПЬЕЗОПИЩАЛКА
digitalWrite(RELAY_out,LOW);//РЭЛЕ ВКЛЮЧЕНИЯ ФЕНА


myPID.SetMode(AUTOMATIC);myPID.SetOutputLimits(0,255);//ЗАДАЁМ ПАРАМЕТРЫ ПИД
analogReference(DEFAULT);//ВКЛЮЧАЕМ ВНУТРЕННЕЕ ОПОРНОЕ НАПРЯЖЕНИЕ 5,0 ВОЛЬТ (1,1В INTERNAL)


/////////////////////////////////////////////////////////////////////////////////////////////////
//                                          EEPROM READ                                        //
//                           СЧИТЫВАЕМ ЗНАЧЕНИЯ ПЕРЕМЕННЫХ ИЗ EEPROM                           //
/////////////////////////////////////////////////////////////////////////////////////////////////
//ПОСЛЕДНЯЯ ЗАДАННАЯ ТЕМПЕРАТУРА (130-380 ГРАДУСОВ)
temp00=EEPROM.read(0);temp01=EEPROM.read(1);SET_Temp=word(temp00,temp01);
//КОЭФФИЦИЕНТ ИНТЕГРИРОВАНИЯ (0.00-9.99)
KiSet10=EEPROM.read(2);KiSet11=EEPROM.read(3);KiSet=word(KiSet10,KiSet11);
//КОЭФФИЦИЕНТ ПРОПОРЦИОНАЛЬНОСТИ (0.00-9.99)
KpSet10=EEPROM.read(4);KpSet11=EEPROM.read(5);KpSet=word(KpSet10,KpSet11);
//КОЭФФИЦИЕНТ ДИФФЕРЕНЦИРОВАНИЯ (0.00-9.99)
KdSet10=EEPROM.read(6);KdSet11=EEPROM.read(7);KdSet=word(KdSet10,KdSet11);
AirFlow=EEPROM.read(8);//ПОТОК ВОЗДУХА ОТ ТУРБИНЫ (8 ЗНАЧЕНИЙ)
LEDshow=EEPROM.read(9);//ПОКАЗЫВАТЬ ТЕМПЕРАТУРУ В РЕЖИМЕ "POWER OFF" (ДА / НЕТ)
StepValue=EEPROM.read(10);//ШАГ ПЕРЕКЛЮЧЕНИЯ ТЕМПЕРАТУРЫ (5 ИЛИ 10 ГРАДУСОВ)
WaitAutoRet=EEPROM.read(11);//ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ (3 - 30 СЕКУНД)
DownTime=EEPROM.read(12);//ВРЕМЯ ЗАДЕРЖКИ ДО УХОДА В "POWER OFF" ЕСЛИ ФЕН НА БАЗЕ (1 - 99 МИНУТ)
LED_Bright=EEPROM.read(13);//ЯРКОСТЬ ИНДИКАТОРА (0 - 15 ПОПУГАЕВ)
//ПРИРАВНИВАЕМ ЗНАЧЕНИЯ ВРЕМЕННЫХ ПЕРЕМЕННЫХ ОСНОВНЫМ ДЛЯ СРАВНЕНИЯ ПЕРЕД ЗАПИСЬЮ
SET_TempMOD=SET_Temp,KiSetMOD=KiSet,KpSetMOD=KpSet,KdSetMOD=KdSet,AirFlowMOD=AirFlow,LEDshowMOD=LEDshow,
StepValueMOD=StepValue,WaitAutoRetMOD=WaitAutoRet,DownTimeMOD=DownTime,LED_BrightMOD=LED_Bright;

//ПОДГОТАВЛИВАЕМ ДЛЯ РАБОТЫ МАХ7219
LC.shutdown(0,false);//ОТКЛЮЧАЕМ РЕЖИМ ЭНЕРГОСБЕРЕЖЕНИЯ 
LC.setIntensity(0,LED_Bright);//УСТАНАВЛИВАЕМ ИНТЕНСИВНОСТЬ СВЕЧЕНИЯ (0 ... 15)
//LC.clearDisplay(0);//ОЧИЩАЕМ ОТ МУСОРА ПАМЯТЬ 


//ЗАДАЁМ КОЭФФИЦИЕНТЫ ПИД 
//ПОСЛЕ ЧТЕНИЯ ИЗ ПАМЯТИ ЗНАЧЕНИЙ ПЕРЕМЕННЫХ ТИПА "int" ПЕРЕВОДИМ ВО "float" (double) 
Kp=(double)KpSet/100.0;Ki=(double)KiSet/100.0;Kd=(double)KdSet/100.0;


}//END SETUP 






/////////////////////////////////////////////////////////////////////////////////////////////////
//                                           L O O P                                           //
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){


newPos=myEnc.read()/4;//ЧИТАЕМ ПОЛОЖЕНИЕ ЭНКОДЕРА 
mill=millis();//ПЕРЕЗАПИСЫВАЕМ ПЕРЕМЕННУЮ "mill" ОПЕРИРУЯ ПЕРЕМЕННОЙ ВМЕСТО ФУНКЦИИ


/////////////////////////////////////////////////////////////////////////////////////////////////
//                                КНОПКА ВКЛЮЧЕНИЯ POWER ON/OFF                                //
/////////////////////////////////////////////////////////////////////////////////////////////////
/*
//АЛГОРИТМ ДЛЯ ОБРАБОТКИ КОМАНДЫ С МЕМБРАННОЙ КНОПКИ (КАК НА ЭНКОДЕРЕ)
//ЛОГИЧЕСКИЙ "0" НАЖАТА / "1" НЕ НАЖАТА
POWER_btn=digitalRead(POWER_key);
if(POWER_btn!=POWER_btn_status){delay(10);POWER_btn=digitalRead(POWER_key);
if(POWER_btn==LOW&&POWER_btn_status==HIGH){PowerStatus=!PowerStatus;FL_start_POWER=true;}
POWER_btn_status=POWER_btn;}
*/

//ЕСЛИ КНОПКА СИЛЬНО ДРЕБЕЗЖИТ (НЕ ПРЕДНАЗНАЧЕНА ДЛЯ СЛАБОТОЧНЫХ ПЕРЕКЛЮЧЕНИЙ)
POWER_btn=digitalRead(POWER_key);
if(POWER_btn!=POWER_btn_status){//ЕСЛИ СОСТОЯНИЕ КНОПКИ POWER ИЗМЕНИЛОСЬ
POWER_btn_status=POWER_btn;//ЗАПИСЫВАЕМ ВРЕМЕННОЕ СОСТОЯНИЕ КНОПКИ КАК ТЕКУЩЕЕ
FL_delay_POWER_key=true;//ПОДНИМАЕМ ФЛАГ ДЛЯ ДОПОЛНИТЕЛЬНОЙ ПРОВЕРКИ СОСТОЯНИЯ
timer_BTN=mill+p100;}//УСТАНАВЛИВАЕМ АНТИДРЕБЕЗГОВУЮ ЗАДЕРЖКУ 100мс 

if(FL_delay_POWER_key==true&&mill>timer_BTN){//ЕСЛИ СПУСТЯ 100мс 
if(POWER_btn==LOW){//СОСТОЯНИЕ КНОПКИ POWER НЕ ИЗМЕНИЛОСЬ
PowerStatus=!PowerStatus;FL_start_POWER=true;}
POWER_btn_status=POWER_btn;
   FL_delay_POWER_key=false;}





/////////////////////////////////////////////////////////////////////////////////////////////////
//                                       РЕЖИМ POWER ON                                        //
/////////////////////////////////////////////////////////////////////////////////////////////////
if(PowerStatus==true){

  
//ЗАДАЁМ НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ РЕЖИМА "POWER ON"
if(FL_start_POWER==true){
FL_pause_MODE=true;FL_start_MENU=true;MAIN_menu=true;SETUP_menu=false;
EncodCL();timer_autoret_DOWN=mill;//FL_ShowMenu=true;
LC.clearDisplay(0);//ЧИСТИМ ПАМЯТЬ У ИНДИКАТОРА
LC.shutdown(0,false);//БЕЗУСЛОВНО ВКЛЮЧАЕМ ИНДИКАТОР
digitalWrite(RELAY_out,HIGH);//ПОДАЁМ НАПРЯЖЕНИЕ НА ВТОРОЙ КОНТАКТ ТЭН И БП ВЕНТИЛЯТОРА
digitalWrite(POWER_LED_out,HIGH);//ВКЛЮЧАЕМ ИНДИКАТОР "POWER" СООБЩАЯ, ЧТО ФЕН ВКЛЮЧЕН
//ПОДАЁМ МНОГОТОНАЛЬНЫЙ ЗВУКОВОЙ СИГНАЛ ВКЛЮЧЕНИЯ ФЕНА "РЕЖИМ POWER ON"
for(INDEX_Freq=5;INDEX_Freq>0;INDEX_Freq--){tone(BUZZER_out,Frequency[INDEX_Freq],p100);delay(p100);}
delay(p200);
   FL_start_POWER=false;}


TEMPERATURA();//ВЫЧИСЛЯЕМ ТЕКУЩУЮ ТЕМПЕРАТУРУ ТЕРМОПАРЫ


//МИНИМАЛЬНАЯ ЗАЩИТА ОТ ПЕРЕГРЕВА ТЭН, ЕСЛИ ТЕРМОПАРА ПОВРЕЖДЕНА, ОБОРВАН КАБЕЛЬ, И Т.Д.
//ПОСТОЯННО ПРОВЕРЯЕМ ЗНАЧЕНИЯ С ПОРТА "А0" ЕСЛИ ОНИ ЗАВЫШЕНЫ, СРАЗУ УХОДИМ В "POWER OFF"
if(inputdata>MAXIMUM_input_ADC){PowerStatus=false;FL_start_POWER=true;}


//ПОДАЁМ МНОГОТОНАЛЬНЫЙ ЗВУКОВОЙ СИГНАЛ ВКЛЮЧЕНИЯ ТЕРМОФЕНА
//if(INDEX_Freq>0&&mill>timer_BUZZER+p100){
//timer_BUZZER=mill;tone(BUZZER_out,Frequency[INDEX_Freq],p100);INDEX_Freq--;}




//////////////////////////////////////////////////////////////////////////////////////////////////
//         КОНТРОЛЬ ДАТЧИКА ПОЛОЖЕНИЯ (ГЕРКОНА) В РУЧКЕ ТЕРМОФЕНА ОТНОСИТЕЛЬНО БАЗЫ             //
//////////////////////////////////////////////////////////////////////////////////////////////////

//КОНТАКТЫ РАЗОМКНУТЫ (СОСТОЯНИЕ HIGH, 1, true) - ФЕН В РУКЕ
//КОНТАКТЫ ЗАМКНУТЫ (СОСОТОЯНИЕ LOW, 0, false) - ФЕН НА БАЗЕ
GERKON_btn=digitalRead(GERKON_key);//ОПРАШИВАЕМ СОСТОЯНИЕ ВХОДА С ПОДКЛЮЧЁННЫМ ГЕРКОНОМ
if(GERKON_btn!=GERKON_btn_status){//ЕСЛИ СОСТОЯНИЕ ГЕРКОНА ИЗМЕНИЛОСЬ
GERKON_btn_status=GERKON_btn;//ЗАПИСЫВАЕМ ВРЕМЕННОЕ СОСТОЯНИЕ ГЕРКОНА КАК ТЕКУЩЕЕ
FL_delay_GERKON=true;//ПОДНИМАЕМ ФЛАГ ДЛЯ ДОПОЛНИТЕЛЬНОЙ ПРОВЕРКИ СОСТОЯНИЯ
timer_GERKON=mill+p200;}//УСТАНАВЛИВАЕМ АНТИДРЕБЕЗГОВУЮ ЗАДЕРЖКУ 200мс 


if(FL_delay_GERKON==true&&mill>timer_GERKON){//ЕСЛИ СПУСТЯ 200мс 
if(GERKON_flag!=GERKON_btn_status){//СОСТОЯНИЕ ГЕРКОНА НЕ СООТВЕТСТВУЕТ СОСТОЯНИЮ ФЛАГА ДЛЯ РУЧКИ
if(GERKON_btn_status){GERKON_flag=true;}//РУЧКА ВНЕ БАЗЫ И ФЕН В РАБОТЕ
else{GERKON_flag=false;}//РУЧКА НА БАЗЕ И ФЕН НА ПАУЗЕ
FL_pause_MODE=true;FL_start_MENU=true;FL_ShowMenu=true;}
   FL_delay_GERKON=false;}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                           ЕСЛИ ФЕН НА БАЗЕ, ВКЛЮЧАЕМ РЕЖИМ "ПАУЗА"                           //
//////////////////////////////////////////////////////////////////////////////////////////////////
if(GERKON_flag==false){

if(FL_pause_MODE==true){
detachInterrupt(0);//ОТКЛЮЧАЕМ ОТСЛЕЖИВАНИЕ ПРЕРЫВАНИЯ 0 НА pin D2
digitalWrite(HOT_PWR_LED_out,LOW);//БЕЗУСЛОВНО ОТКЛЮЧАЕМ НАГРЕВ ТЭНа
temp_for_FAN=SET_Temp;//if(temp_for_FAN<MINIMUM_temp){temp_for_FAN=MINIMUM_temp;}
FL_start_COOLING=true;//ПОДНИМАЕМ ФЛАГ ДЛЯ ВКЛЮЧЕНИЯ ВЕНТИЛЯТОРА ОХЛАЖДЕНИЯ
timer_autoret_DOWN=mill;//ОБНУЛЯЕМ ТАЙМЕР УХОДА В СПЯЩИЙ РЕЖИМ
tone(BUZZER_out,977,350);//ПОДАЁМ ЗВУКОВОЙ СИГНАЛ РЕЖИМА "ПАУЗА" 
   FL_pause_MODE=false;}


//ЕСЛИ ТЭН ГОРЯЧИЙ, ВКЛЮЧАЕМ ВЕНТИЛЯТОР ДЛЯ ОХЛАЖДЕНИЯ СО СКОРОСТЬЮ ЗАВИСЯЩЕЙ ОТ ТЕМПЕРАТУРЫ
if(FL_start_COOLING==true&&Input_TEMP>Safe_HOT){
SpeedCooling=map(Input_TEMP,Safe_HOT,temp_for_FAN,1,8);//ЧЕМ НИЖЕ ТЕМПЕРАТУРА, ТЕМ МЕНЬШЕ СКОРОСТЬ 
SpeedCooling=constrain(SpeedCooling,1,8);//УСТАНАВЛИВАЕМ СКОРОСТЬ В ДИАПАЗОНЕ 1 - 8
analogWrite(FAN_out,PWM[SpeedCooling]);//ИЗ МАССИВА ЗНАЧЕНИЙ PWM ВЫБИРАЕМ СООТВЕТСТВУЮЩЕЕ СКОРОСТИ
FanSpeed=SpeedCooling;}//ОТОБРАЖАЕМАЯ ТОЧКАМИ СКОРОСТЬ ВЕНТИЛЯТОРА ПРИ ОХЛАЖДЕНИИ
else{digitalWrite(FAN_out,LOW);//ЕСЛИ ТЕМПЕРАТУРА ПАДАЕТ НИЖЕ БЕЗОПАСНОЙ ОТКЛЮЧАЕМ ВЕНТИЛЯТОР
FanSpeed=AirFlow;//ОТОБРАЖАЕМУЮ ТОЧКАМИ СКОРОСТЬ ВЕНТИЛЯТОРА ПРИРАВНИВАЕМ К ЗАДАННОЙ
FL_start_COOLING=false;}//НЕ ВКЛЮЧАЕМ ВЕНТИЛЯТОР ДАЖЕ ЕСЛИ ТЕМПЕРАТУРА СЛЕГКА ПРЕВЫСИТ БЕЗОПАСНУЮ


}//КОНЕЦ РЕЖИМА "ПАУЗА"




//////////////////////////////////////////////////////////////////////////////////////////////////
//                      ЕСЛИ ФЕН В РУКЕ, ВКЛЮЧАЕМ ЯДРО ФЕНА И РЕЖИМА "РАБОТА"                   //
//////////////////////////////////////////////////////////////////////////////////////////////////
if(GERKON_flag==true){

if(FL_pause_MODE==true){
attachInterrupt(0,tracking,RISING);//ПЕРЕДАЁМ УПРАВЛЕНИЕ ФУНКЦИЕЙ tracking ПРЕРЫВАНИЮ 0
myPID.SetTunings(Kp,Ki,Kd);//УСТАНАВЛИВАЕМ КОЭФФИЦИЕНТЫ ПИД
timer_autoret_DOWN=mill;//ОБНУЛЯЕМ ТАЙМЕР УХОДА В СПЯЩИЙ РЕЖИМ
tone(BUZZER_out,2050,350);//ПОДАЁМ ТОНАЛЬНЫЙ СИГНАЛ ВКЛЮЧЕНИЯ РЕЖИМА "РАБОТА"
   FL_pause_MODE=false;}


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                 УПРАВЛЯЕМ ТЭН И ВЕНТИЛЯТОРОМ                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////
Setpoint=SET_Temp;//ОТСЫЛАЕМ ЗАДАННУЮ ТЕМПЕРАТУРУ В ПИД
myPID.Compute();//ПРОПОРЦИОНАЛЬНО-ИНТЕГРАЛЬНОЕ ДИФФЕРЕНЦИРОВАНИЕ ТЕМПЕРАТУРЫ (ПИД)
triac=Output;//РЕЗУЛЬТАТ РАБОТЫ ПИД ИСПОЛЬЗУЕМ В ФУНКЦИИ tracking() ДЛЯ УПРАВЛЕНИЯ ТЭН
//ПОДАЁМ НАПРЯЖЕНИЕ НА ВЕНТИЛЯТОР В СООТВЕТСТВИИ С ЗАДАННЫМ НОМЕРОМ ИЗ МАССИВА ЗНАЧЕНИЙ PWM[]
analogWrite(FAN_out,PWM[AirFlow]);


}//КОНЕЦ РЕЖИМА "РАБОТА"




////////////////////////////////////////////////////////////////////////////////////////////////
//                                       ГЛАВНОЕ МЕНЮ                                         //
////////////////////////////////////////////////////////////////////////////////////////////////
//РАБОТУ ФЕНА СОПРОВОЖДАЕМ ИНФОРМАЦИЕЙ НА ИНДИКАТОРЕ
if(MAIN_menu==true){

//ЗАПОЛНЯЕМ ПУСТЫМИ СИМВОЛАМИ НЕИСПОЛЬЗУЕМЫЕ В ДАННОМ МЕНЮ СЕГМЕНТЫ ИНДИКАТОРА
if(FL_start_MENU==true){
m0=0;Digit4=Digit5=' ';FanSpeed=AirFlow;timer_MENU=mill-p1000;FL_ShowMenu=true;
   FL_start_MENU=false;}


//НАЖАТИЕМ НА ЭНКОДЕР ВЫБИРАЕМ ЧТО МЕНЯТЬ: ТЕМПЕРАТУРУ ИЛИ ПОТОК ВОЗДУХА
MENU_btn=digitalRead(MENU_key);
if(MENU_btn!=MENU_btn_status){delay(10);MENU_btn=digitalRead(MENU_key);
if(MENU_btn==LOW&&MENU_btn_status==HIGH){++m0;if(m0>1){m0=0;}
timer_BTN=mill;timer_MENU=mill-p1000;FL_delay_MENU=true;
timer_autoret_DOWN=mill;FL_ShowMenu=true;}
MENU_btn_status=MENU_btn;}


if(FL_ShowMenu==true){
switch(m0){
  case 0: trig0=SET_Temp;break;
  case 1: trig0=AirFlow;break;}
   FL_ShowMenu=false;}


//МЕНЯЕМ ЭНКОДЕРОМ ЗНАЧЕНИЕ ТРЕБУЕМОЙ ТЕМПЕРАТУРЫ ИЛИ ПОТОК ВОЗДУХА 
if(newPos!=oldPos){

switch (m0){
case 0:trig0=trig0+(newPos*StepValue);
    if(trig0>=MINIMUM_temp){SET_Temp=constrain(trig0,MINIMUM_temp,MAX_temp);}
    else{if(trig0<SET_Temp){trig0=0;}else{trig0=MINIMUM_temp;}SET_Temp=trig0;};break;
case 1:trig0=trig0+newPos;AirFlow=constrain(trig0,1,8);FanSpeed=AirFlow;break;}

timer_autoret_DOWN=mill;timer_MENU=mill-p1000;FL_ShowMenu=true;EncodCL();}


//ВЫВОДИМ ЗНАЧЕНИЯ НА ИНДИКАТОР КАЖДУЮ СЕКУНДУ (чаще нет необходимости)
if(mill-timer_MENU>p1000){timer_MENU=mill;
//РАЗБИВАЕМ ПО РАЗРЯДАМ ТЕКУЩУЮ ВЫЧИСЛЕННУЮ ТЕМПЕРАТУРУ
Digit1=Input_TEMP/100%10;if(Digit1==0){Digit1=' ';}
Digit2=Input_TEMP/10%10;
Digit3=Input_TEMP%10;
//В ЗАВИСИМОСТИ ОТ ВЫБРАННОГО ПОДМЕНЮ (ЗАДАННАЯ ТЕМПЕРАТУРА ИЛИ СКОРОСТЬ ВЕНТИЛЯТОРА)
if(m0==0){//РАЗБИВАЕМ ПО РАЗРЯДАМ ЗАДАННУЮ ТЕМПЕРАТУРУ
Digit6=SET_Temp/100%10;
Digit7=SET_Temp/10%10;
Digit8=SET_Temp%10;}
else{//ЛИБО СКОРОСТЬ ВЕНТИЛЯТОРА ОТ 1 ДО 8
Digit6='F';
Digit7=' ';
Digit8=AirFlow;}

DotFan();//ОТОБРАЖАЕМ СКОРОСТЬ ВЕНТИЛЯТОРА В ВИДЕ ТОЧЕК
ShowMENU();}//ОТОБРАЖАЕМ ФАКТИЧЕСКУЮ И ЗАДАННУЮ ТЕМПЕРАТУРЫ


//ЕСЛИ ФЕН НИКАК НЕ ИСПОЛЬЗУЕТСЯ ЗАДАННЫЙ ИНТЕРВАЛ УХОДИМ В РЕЖИМ "STANDBY"
if(mill-timer_autoret_DOWN>DownTime*p60000){PowerStatus=false;FL_start_POWER=true;}



}//END MAIN MENU





////////////////////////////////////////////////////////////////////////////////////////////////
//                                  МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ                                 //
////////////////////////////////////////////////////////////////////////////////////////////////

// 1 ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ (5 ИЛИ 10 ГРАДУСОВ)
// 2 ВРЕМЯ ДО АВТООТКЛЮЧЕНИЯ ФЕНА ПРИ ОТСУТСТВИИ АКТИВНОСТИ (ОТ 1 ДО 99 МИНУТ)
// 3 ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ (ОТ 3 ДО 30 СЕКУНД)
// 4 КОЭФФИЦИЕНТ ПРОПОРЦИОНАЛЬНОСТИ (0.00-9.99)
// 5 КОЭФФИЦИЕНТ ИНТЕГРИРОВАНИЯ (0.00-9.99)
// 6 КОЭФФИЦИЕНТ ДИФФЕРЕНЦИРОВАНИЯ (0.00-9.99)
// 7 ЯРКОСТЬ СВЕТОДИОДНОГО ИНДИКАТОРА (0 - 15 ПОПУГАЕВ)

if(SETUP_menu==true){

if(FL_start_MENU==true){
Digit2=Digit3=Digit4=Digit5=Digit6=Digit7=' ';//ЧИСТИМ ПОЗИЦИИ ДО ОТОБРАЖЕНИЯ МЕНЮ НАСТРОЙКИ
timer_autoret_MENU=mill;timer_autoret_DOWN=mill;//СБРАСЫВАЕМ ТАЙМЕРЫ
FL_ShowMenu=true;//ПОДНИМАЕМ ФЛАГ ПЕЧАТИ МЕНЮ
MenuSwitch=true;dot8();//ПО УМОЛЧАНИЮ ВКЛЮЧАЕМ РЕЖИМ ПРОКРУТКИ ПУНКТОВ МЕНЮ (ЗАЖИГАЕМ ВСЕ ТОЧКИ)
tone(BUZZER_out,3000,100);//КОРОТКИЙ СИГНАЛ АКТИВАЦИИ МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ
   FL_start_MENU=false;}


//ПРИ НАЖАТИИ НА ЭНКОДЕР, ВЫБИРАЕМ ЛИБО РЕЖИМ ИЗМЕНЕНИЯ ПАРАМЕТРОВ, ЛИБО ПРОКРУТКИ ПУНКТОВ МЕНЮ
MENU_btn=digitalRead(MENU_key);
if(MENU_btn!=MENU_btn_status){delay(10);MENU_btn=digitalRead(MENU_key);
if(MENU_btn==LOW&&MENU_btn_status==HIGH){timer_MENU=mill;timer_BTN=mill;
timer_autoret_MENU=mill;timer_autoret_DOWN=mill;FL_ShowMenu=true;
FL_delay_MENU=true;MenuSwitch=!MenuSwitch;
if(MenuSwitch==true){dot8();}//ДЛЯ ПРОКРУТКИ ПУНКТОВ МЕНЮ ЗАЖИГАЕМ ВСЕ ТОЧКИ
if(MenuSwitch==false){dot0();}}//ДЛЯ РЕДАКТИРОВАНИЯ (ИЗМЕНЕНИЯ ПАРАМЕТРОВ) ГАСИМ ВСЕ ТОЧКИ
MENU_btn_status=MENU_btn;}


//ОБРАБАТЫВАЕМ ПОВОРОТЫ ЭНКОДЕРА

//МЕНЯЕМ ПУНКТЫ МЕНЮ
if(MenuSwitch==true&&newPos!=oldPos){
m1=m1+newPos;if(m1>6){m1=0;}if(m1<0){m1=6;}Digit2=Digit3=Digit4=Digit5=Digit6=Digit7=' ';
EncodCL();timer_autoret_MENU=mill;timer_autoret_DOWN=mill;FL_ShowMenu=true;}

//МЕНЯЕМ ПАРАМЕТРЫ ВНУТРИ ПУНКТА МЕНЮ
if(MenuSwitch==false&&newPos!=oldPos){
trig1=trig1+newPos;
EncodCL();timer_autoret_MENU=mill;timer_autoret_DOWN=mill;FL_ShowMenu=true;
switch (m1){
case 0: if(trig1>1){trig1=0;}if(trig1<0){trig1=1;}StepValue=zen[trig1];break;
//ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ 5 ИЛИ 10 ГРАДУСОВ
case 1: DownTime=constrain(trig1,1,99);break;//ВРЕМЯ ОТКЛЮЧЕНИЯ ФЕНА ПРИ ОТСУТСТВИИ АКТИВНОСТИ
case 2: WaitAutoRet=constrain(trig1,3,30);break;//ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ
case 3: KpSet=constrain(trig1,0,999);Kp=(double)KpSet/100.0;break;
case 4: KiSet=constrain(trig1,0,999);Ki=(double)KiSet/100.0;break;
case 5: KdSet=constrain(trig1,0,999);Kd=(double)KdSet/100.0;break;
case 6: LED_Bright=constrain(trig1,0,15);LC.setIntensity(0,LED_Bright);break;}}//ЯРКОСТЬ ИНДИКАТОРА


//ОТОБРАЖАЕМ НА ИНДИКАТОРЕ ВНЕСЁННЫЕ ИЗМЕНЕНИЯ
if(FL_ShowMenu==true){menu1();ShowMENU();FL_ShowMenu=false;}



//АВТОВОЗВРАТ В ГЛАВНОЕ МЕНЮ ИЗ МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ ЧЕРЕЗ ЗАДАННЫЙ ИНТЕРВАЛ
if(mill-timer_autoret_MENU>WaitAutoRet*p1000){
tone(BUZZER_out,3000,100);//КОРОТКИЙ СИГНАЛ АВТОВОЗВРАТА В ГЛАВНОЕ МЕНЮ
MAIN_menu=true;SETUP_menu=false;FL_start_MENU=true;}


}//END SETUP MENU 





//ДЛЯ ВХОДА ИЛИ ВЫХОДА В/ИЗ МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ УДЕРЖИВАЕМ ЭНКОДЕР НЕ МЕНЕЕ 0,5 СЕК.
if(FL_delay_MENU==true&&MENU_btn==LOW&&mill-timer_BTN>p500){
if(MAIN_menu==true){SETUP_menu=true;MAIN_menu=false;}else{MAIN_menu=true;SETUP_menu=false;}
FL_start_MENU=true;
   FL_delay_MENU=false;}




}//END POWER ON





/////////////////////////////////////////////////////////////////////////////////////////////////
//                                  РЕЖИМ POWER OFF (STANDBY)                                  //
/////////////////////////////////////////////////////////////////////////////////////////////////
if(PowerStatus==false){

//ЗАДАЁМ ПАРАМЕТРЫ ДЛЯ РЕЖИМА "STANDBY"
if(FL_start_POWER==true){
EEPROM_UPDATE();TEMPERATURA();LC.clearDisplay(0);//СОХРАНЯЕМ ПЕРЕМЕННЫЕ В ПАМЯТЬ
STANDBY_menu=true;ENGINEERING_menu=false;FL_start_MENU=true;
//tone(BUZZER_out,470,500);//ПОДАЁМ ЗВУКОВОЙ СИГНАЛ ОТКЛЮЧЕНИЯ ФЕНА "РЕЖИМ STANDBY"
//ПОДАЁМ МНОГОТОНАЛЬНЫЙ ЗВУКОВОЙ СИГНАЛ ОТКЛЮЧЕНИЯ ФЕНА "РЕЖИМ STANDBY"
for(INDEX_Freq=0;INDEX_Freq<5;INDEX_Freq++){tone(BUZZER_out,Frequency[INDEX_Freq],p100);delay(100);}
   FL_start_POWER=false;}


//ПОДАЁМ МНОГОТОНАЛЬНЫЙ ЗВУКОВОЙ СИГНАЛ ОТКЛЮЧЕНИЯ ФЕНА
//if(INDEX_Freq<5&&mill>timer_BUZZER+p100){
//timer_BUZZER=mill;tone(BUZZER_out,Frequency[INDEX_Freq],p100);INDEX_Freq++;}





///////////////////////////////////////////////////////////////////////////////////////////////
//                               ОСНОВНОЕ МЕНЮ РЕЖИМА "STANDBY"                              //
///////////////////////////////////////////////////////////////////////////////////////////////
if(STANDBY_menu==true){
if(FL_start_MENU==true){
dot0();Digit1=Digit7=Digit8=' ';triac=0;TEMPERATURA();
FL_start_COOLING=true;//ПОДНИМАЕМ ФЛАГ ДЛЯ ОХЛАЖДЕНИЯ ФЕНА
digitalWrite(HOT_PWR_LED_out,LOW);//ОТКЛЮЧАЕМ СИМИСТОР УПРАВЛЕНИЯ ТЭН
digitalWrite(RELAY_out,LOW);//ОТКЛЮЧАЕМ СИМИСТОР УПРАВЛЯЮЩИЙ ОБЩИМ ПИТАНИЕМ ФЕНА
digitalWrite(POWER_LED_out,LOW);//ОТКЛЮЧАЕМ ИНДИКАТОР "POWER ON"
detachInterrupt(0);//ОТКЛЮЧАЕМ ОТСЛЕЖИВАНИЕ ПРЕРЫВАНИЯ 0 НА pin D2
LEDshow?LC.shutdown(0,false):LC.shutdown(0,true);//ВКЛЮЧАЕМ ИЛИ ВЫКЛЮЧАЕМ ПОКАЗ ТЕМПЕРАТУРЫ
timer_MENU=mill-p1000;//ОТНИМАЕМ СЕКУНДУ ДЛЯ МГНОВЕННОГО ОТОБРАЖЕНИЯ ОСНОВНОГО МЕНЮ
   FL_start_MENU=false;}


////////////////////////////////////////////////////////////////////////////////////////////////
//                          ОБРАБАТЫВАЕМ НАЖАТИЕ НА КНОПКУ ЭНКОДЕРА                           //
////////////////////////////////////////////////////////////////////////////////////////////////
MENU_btn=digitalRead(MENU_key);
if(MENU_btn!=MENU_btn_status){delay(10);MENU_btn=digitalRead(MENU_key);
if(MENU_btn==LOW&&MENU_btn_status==HIGH){
LEDshow=!LEDshow;LEDshow?LC.shutdown(0,false):LC.shutdown(0,true);
timer_BTN=mill+p1000;FL_delay_MENU=true;timer_MENU=mill-p1000;}
MENU_btn_status=MENU_btn;}



//ВЫВОДИМ ТЕКУЩУЮ ТЕМПЕРАТУРУ НА ИНДИКАТОР КАЖДУЮ СЕКУНДУ
if(mill-timer_MENU>p1000){timer_MENU=mill;TEMPERATURA();
Digit2=Input_TEMP/100%10;if(Digit2==0){Digit2=' ';}
Digit3=Input_TEMP/10%10;
Digit4=Input_TEMP%10;
#ifdef LOG_ENABLE
Serial.print("FL_start_COOLING ");Serial.println(FL_start_COOLING);
Serial.print("Input_TEMP ");Serial.println(Input_TEMP);
Serial.print("SpeedCooling ");Serial.println(SpeedCooling);
Serial.print("PWM[SpeedCooling] ");Serial.println(PWM[SpeedCooling]);
#endif
menuOFF();}


//ЕСЛИ ТЭН ГОРЯЧИЙ, ВКЛЮЧАЕМ ВЕНТИЛЯТОР
if(FL_start_COOLING==true&&Input_TEMP>Safe_HOT){
digitalWrite(RELAY_out,HIGH);//ДЛЯ ВЕРСИИ 12В ВЕНТИЛЯТОРА ВКЛЮЧАТЬ БЛОК ПИТАНИЯ 24В НЕНУЖНО
SpeedCooling=map(Input_TEMP,Safe_HOT,SET_Temp,1,8);
SpeedCooling=constrain(SpeedCooling,1,8);
analogWrite(FAN_out,PWM[SpeedCooling]);}
else{digitalWrite(FAN_out,LOW);
digitalWrite(RELAY_out,LOW);//ДЛЯ ВЕРСИИ 12В ВЕНТИЛЯТОРА ОТКЛЮЧАТЬ БЛОК ПИТАНИЯ 24В НЕНУЖНО
   FL_start_COOLING=false;}



}//END STANDBY MENU







//////////////////////////////////////////////////////////////////////////////////////////////
//               ИНЖЕНЕРНОЕ МЕНЮ НАСТРОЙКИ УСИЛИТЕЛЯ ПОД ПАРАМЕТРЫ ТЕРМОПАРЫ                //
//////////////////////////////////////////////////////////////////////////////////////////////
if(ENGINEERING_menu==true){
if(FL_start_MENU==true){
digitalWrite(RELAY_out,HIGH);//ВКЛЮЧАЕМ ВЫХОД УПРАВЛЯЮЩИЙ ОБЩИМ ПИТАНИЕМ ФЕНА
FanSpeed=AirFlow;DotFan();Digit4=Digit5=' ';LC.shutdown(0,false);tone(BUZZER_out,3000,100);
attachInterrupt(0,tracking,RISING);triac=0;analogWrite(FAN_out,PWM[AirFlow]);EncodCL();
   FL_start_MENU=false;}


//ПОДМИГИВАЕМ ИНДИКАТОРОМ "POWER" СООБЩАЯ, ЧТО ФЕН ВКЛЮЧЕН И НАХОДИТСЯ В РЕЖИМЕ "ИНЖЕНЕРНОЕ МЕНЮ"
if(mill-timer_LED>(Wink?p1000:p100)){timer_LED=mill;Wink=!Wink;digitalWrite(POWER_LED_out,Wink);}


////////////////////////////////////////////////////////////////////////////////////////////////
//                          ОБРАБАТЫВАЕМ НАЖАТИЕ НА КНОПКУ ЭНКОДЕРА                           //
////////////////////////////////////////////////////////////////////////////////////////////////
MENU_btn=digitalRead(MENU_key);
if(MENU_btn!=MENU_btn_status){delay(10);MENU_btn=digitalRead(MENU_key);
if(MENU_btn==LOW&&MENU_btn_status==HIGH){
m2++;if(m2>1){m2=0;}
FL_delay_MENU=true;timer_BTN=mill+p500;}
MENU_btn_status=MENU_btn;}

TEMPERATURA();//ВЫЧИСЛЯЕМ ТЕМПЕРАТУРУ

//ДЛЯ НАСТРОЙКИ ТЕМПЕРАТУРЫ ПОЛУЧАЕМОЙ С ТЕРМОПАРЫ, УПРАВЛЯЕМ СИМИСТОРОМ НАПРЯМУЮ
if(newPos!=oldPos){
triac=triac+(newPos*StepValue);EncodCL();
triac=constrain(triac,0,255);timer_MENU=mill+p500;}

//КАЖДЫЕ 0.5 СЕК...
if(mill-timer_MENU>p500){timer_MENU=mill;
if(m2==0){
//ВЫВОДИМ РЕЗУЛЬТАТ АЦП 0...1024 ОЦИФРОВКИ НАПРЯЖЕНИЯ С УСИЛИТЕЛЯ, ЛИБО... 
Digit1=inputdata/1000%10;
Digit2=inputdata/100%10;
Digit3=inputdata/10%10;
Digit4=inputdata%10;}

//ЭТОТ ЖЕ РЕЗУЛЬТАТ ПРЕСЧИТАННЫЙ В ТЕМПЕРАТУРУ 20...380
else{
Digit1=' ';
Digit2=Input_TEMP/100%10;if(Input_TEMP<100){Digit2=' ';}
Digit3=Input_TEMP/10%10;
Digit4=Input_TEMP%10;}

//ВЫВОДИМ ПРОЦЕНТ ЗАПОЛНЕНИЯ ШИМ НАПРЯМУЮ (БЕЗ ПИД) ДЛЯ НАГРЕВА ТЭН
Digit6=(triac)/100%10;if(Digit6==0){Digit6=' ';}
Digit7=(triac)/10%10;
Digit8=(triac)%10;
ShowMENU();}
}//END MENU 1



//УДЕРЖИВАЯ КНОПКУ ЭНКОДЕРА 1 СЕК. И БОЛЕЕ ПЕРЕКЛЮЧАЕМСЯ МЕЖДУ ОСНОВНЫМ И ИНЖЕНЕРНЫМ МЕНЮ
if(FL_delay_MENU==true&&MENU_btn==LOW&&mill>timer_BTN){
if(STANDBY_menu){STANDBY_menu=false;ENGINEERING_menu=true;}
else{STANDBY_menu=true;ENGINEERING_menu=false;}
LEDshow=true;FL_start_MENU=true;LC.clearDisplay(0);
   FL_delay_MENU=false;}



}//////////////////////////////////////// END POWER OFF ///////////////////////////////////////////


}/////////////////////////////////////////// END LOOP /////////////////////////////////////////////
