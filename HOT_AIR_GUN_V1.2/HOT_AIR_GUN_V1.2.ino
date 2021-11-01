///////////////////////////////////// ТЕРМОФЕН / HOT AIR GUN /////////////////////////////////////
///////////////////// ТЕРМОВОЗДУШНАЯ ПАЯЛЬНАЯ СТАНЦИЯ / SMD REWORK STATION ///////////////////////
////////////////////////////////////// ВЕРСИЯ / VERSION 1.2 //////////////////////////////////////
///////////////////// СКЕТЧ АНАТОЛИЯ НЕВЗОРОВА / CODE BY ANATOLY NEVZOROFF ///////////////////////

#define p60000 60000L
#define p1000 1000L//ИНТЕРВАЛ 1000 mS (1 СЕКУНДА), ТИП ДАННЫХ "L" (long)
#define p500 500L
#define p400 400L
#define p100 100L

#define pin5 5
#define pin6 6
#define pin9 9
#define pin10 10
#define pin11 11
#define pin13 13
#define pin15 15
#define pin16 16
#define hot 45//БЕЗОПАСНАЯ ТЕМПЕРАТУРА СТАЛЬНОЙ ПОВЕРХНОСТИ ТЕРМОФЕНА
//ЕСЛИ МАКСИМАЛЬНЫЙ УРОВЕНЬ НА ВХОДЕ АЦП ДЛЯ АВАРИЙНОГО ОТКЛЮЧЕНИЯ = 1023 (питание 4,95 вольта)
//ТО maxin ВЫБИРАЕМ НЕСКОЛЬКО МЕНЬШЕ НАПРИМЕР = 1021
#define maxin 1021
#define mintemp 20//ГРАНИЦА МИНИМАЛЬНОЙ ТЕМПЕРАТУРЫ ФЕНА
#define minimtemp 130//ГРАНИЦА МИНИМАЛЬНО-РЕГУЛИРУЕМОЙ ТЕМПЕРАТУРЫ ФЕНА
#define maxtemp 390//ГРАНИЦА МАКСИМАЛЬНОЙ ТЕМПЕРАТУРЫ ФЕНА

//////////////////////////////////// ПОДКЛЮЧАЕМЫЕ БИБЛИОТЕКИ /////////////////////////////////////
#include <EEPROM.h>//ЧТЕНИЕ И ЗАПИСЬ ПЕРЕМЕННЫХ В ЭНЕРГОНЕЗАВИСИМУЮ ПАМЯТЬ EEPROM
#include <Encoder.h>//ОБРАБОТКА УГЛА ПОВОРОТА ЭНКОДЕРА
#include <PID_v1.h>//https://github.com/br3ttb/Arduino-PID-Library  \
БИБЛИОТЕКА ПРОПОРЦИОНАЛЬНО -ИНТЕГРАЛЬНОГО ДИФФЕРЕНЦИРОВАНИЯ ДЛЯ ИНЕРТНЫХ СИСТЕМ
#include <LedControl.h>//https://github.com/wayoda/LedControl  \
БИБЛИОТЕКА ДЛЯ ДРАЙВЕРОВ MAX7219 И MAX7221 СВЕТОДИОДНЫХ ИНДИКАТОРОВ И ПАНЕЛЕЙ

////////////////////////////////// ИНИЦИАЛИЗАЦИЯ ОБОРУДОВАНИЯ ////////////////////////////////////
Encoder myEnc(3, 4); // DT и CLK ИЛИ S2 и S1 ВЫВОДЫ ЭНКОДЕРА
LedControl LC = LedControl(12, 8, 7, 1); //12-DIN, 8-CLC, 7-CS, 1-ОДИН ИНДИКАТОР

/////////////////////////////////// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ////////////////////////////////////////
//unsigned long от 0 до 4294967295
uint32_t mill, timer1, timer2, timer3, timer4, timer5; //ТАЙМЕРЫ ДЛЯ ФУНКЦИИ millis()

//int от -32768 до 32767
int16_t triac, inputdata, temp, tempcold, KpSet, KiSet, KdSet, trig0, trig1; //
int16_t fakt, freq[4] = {2077, 1577, 1077, 577}; //МАССИВ ЧАСТОТ ДЛЯ ФУНКЦИИ tone()

//byte от 0 до 255
uint8_t temp00, temp01, KpSet10, KpSet11, KiSet10, KiSet11, KdSet10, KdSet11;
uint8_t t, cold, fan, fanspeed, sharp, ar1, ar2, ar3, ar4, ar5, ar6, ar7, ar8;
uint8_t zen[2] = {5, 10}; //МАССИВЧИК ДЛЯ УСТАНОВКИ ШАГА ТЕМПЕРАТУРЫ
//МАССИВ ЗНАЧЕНИЙ ШИМ ДЛЯ СКОРОСТИ ВЕНТИЛЯТОРА (подбираем индивидуально)
uint8_t pwm[9] = {0, 100, 109, 118, 125, 137, 150, 165, 255};

//char от -128 до 127
int8_t fr, m0, m1, m2, menu, down, autoret, oldPos, newPos; //

//boolean от false до true
bool led, flag, btn5, oldbtn5, btn9, oldbtn9, btn15, oldbtn15, save, power; //

//СТРУКТУРА ДЛЯ УПРАВЛЕНИЯ ПОДСВЕТКОЙ ТОЧЕК
struct DPLed {
  bool L0: 1, L1: 1, L2: 1, L3: 1, L4: 1, L5: 1, L6: 1, L7: 1;
}; DPLed D;

//СТРУКТУРА ДЛЯ ЭКОНОМИИ ПАМЯТИ НА ФЛАГАХ
struct Flags {
  bool w0: 1, w1: 1, w2: 1, w3: 1, w4: 1, w5: 1, w6: 1, w7: 1, w8: 1, w9: 1, w10: 1;
}; Flags f;
//ЗАДАЁМ ПЕРЕМЕННЫЕ ДЛЯ БИБЛИОТЕКИ ПИД (PID)
double Setpoint;//ТРЕБУЕМАЯ ТЕМПЕРАТУРА
double Input;   //ДАННЫЕ О ТЕКУЩЕЙ ТЕМПЕРАТУРЕ
double Output;  //ЗНАЧЕНИЕ ОТ 0 ДО 255 ВЫДАВАЕМОЕ БИБЛИОТЕКОЙ В ЗАВИСИМОСТИ ОТ Setpoint И Input
double Kp;      //АГРЕССИВНОСТЬ НАБОРА ТЕМПЕРАТУРЫ Setpoint, ЧЕМ БОЛЬШЕ ТЕМ БЫСТРЕЕ
double Ki;      //"ПРЕДСКАЗЫВАЕТ" ЗНАЧЕНИЕ Input, ЧЕМ БОЛЬШЕ ТЕМ ИНЕРЦИОННЕЕ (ОСЛАБЛЯЕТ Kd)
double Kd;      //УСРЕДНЯЕТ, СГЛАЖИВАЕТ Output ПРЕДОТВРАЩАЕТ КОЛЕБАНИЯ
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //ИНИЦИАЛИЗИРУЕМ ПЕРЕМЕННЫЕ ДЛЯ PID

/////////////////////////////////////// SETUP //////////////////////////////////////////
void setup() {

  //ПЕРЕВОДИМ ПИНЫ D9 и D10 ТАЙМЕРА №1 НА ЧАСТОТУ - 31.4 кГц
  TCCR1A = 0b00000001;//8bit
  TCCR1B = 0b00000001;//x1 phase correct

  LC.shutdown(0, false); //ОТКЛЮЧАЕМ РЕЖИМ ЭНЕРГОСБЕРЕЖЕНИЯ МАХ7219
  LC.setIntensity(0, 1); //УСТАНАВЛИВАЕМ ИНТЕНСИВНОСТЬ СВЕЧЕНИЯ ОТ 0 ДО 15 МАХ7219
  LC.clearDisplay(0);//ОЧИСТКА ОТ МУСОРА ПАМЯТИ МАХ7219

  myPID.SetMode(AUTOMATIC); myPID.SetOutputLimits(0, 255); //ЗАДАЁМ ПАРАМЕТРЫ ПИД

  pinMode(2, INPUT);     //ВХОД ДЛЯ ОПРЕДЕЛЕНИЯ ПЕРЕХОДА СИНУСОИДЫ ЧЕРЕЗ "0"
  //pinMode(3,INPUT);    //ПОВОРОТ ЭНКОДЕРА белый провод DT (S2) (ПИН ИНИЦИАЛИЗИРУЕТСЯ БИБЛИОТЕКОЙ)
  //pinMode(4,INPUT);    //ПОВОРОТ ЭНКОДЕРА жёлтый провод CLK (S1) (ПИН ИНИЦИАЛИЗИРУЕТСЯ БИБЛИОТЕКОЙ)
  pinMode(pin5, INPUT);     //КНОПКА МЕНЮ (Кнопка SW энкодера)
  pinMode(pin6, OUTPUT);    //ВЫХОД УПРАВЛЕНИЯ СИМИСТОРОМ НА ТЭН И ИНДИКАТОРОМ (Оранжевый светодиод)
  //pinMode(7,OUTPUT);   //ВЫХОД ДЛЯ ВЫВОДА "LOAD/CS" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
  //pinMode(8,OUTPUT);   //ВЫХОД ДЛЯ ВЫВОДА "CLK" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
  pinMode(pin9, OUTPUT);    //ИНДИКАТОР ВКЛЮЧЕНИЯ POWER ON (Красный светодиод)
  pinMode(pin10, OUTPUT);   //ВЫХОД ДЛЯ УПРАВЛЕНИЯ МОСФЕТ ВЕНТИЛЯТОРА
  pinMode(pin11, OUTPUT);   //ВЫХОД ДЛЯ ПЬЕЗОПИЩАЛКИ (tone() РАБОТАЕТ НА ПИНАХ 3 И 11 ВТОРОГО ТАЙМЕРА)
  //pinMode(12,OUTPUT);  //ВЫХОД ДЛЯ ВЫВОДА "DIN" НА ИНДИКАТОРЕ С ДРАЙВЕРОМ MAX7219
  pinMode(pin13, OUTPUT);   //ВЫХОД УПРАВЛЕНИЯ СИМИСТОРОМ ПИТАНИЯ ТЕРМОФЕНА ПО ВТОРОЙ ШИНЕ ПИТАНИЯ
  //pinMode(A0,INPUT);   //ВХОД А0 ДЛЯ ВЫЧИСЛЕНИЯ ТЕМПЕРАТУРЫ ЧЕРЕЗ НАПРЯЖЕНИЕ С ТЕРМОПАРЫ
  pinMode(pin15, INPUT_PULLUP); //ВХОД А1 ДЛЯ ОТСЛЕЖИВАНИЯ СОСТОЯНИЯ ГЕРКОНА В РУЧКЕ
  pinMode(pin16, INPUT_PULLUP); //ВХОД А2 ДЛЯ КНОПКИ ВКЛЮЧЕНИЯ POWER ON
  //ОТКЛЮЧАЕМ ИНДИКАТОРЫ И ВЫХОДЫ УПРАВЛЯЮЩИЕ СИЛОВЫМИ ЭЛЕМЕНТАМИ ФЕНА
  digitalWrite(pin6, LOW); digitalWrite(pin9, LOW); digitalWrite(pin10, LOW); digitalWrite(pin13, LOW);
  analogReference(DEFAULT);//ВКЛЮЧАЕМ ВНУТРЕННЕЕ ОПОРНОЕ НАПРЯЖЕНИЕ 5,0 ВОЛЬТ (1,1В INTERNAL)

  ///////////////////////// ЧИТАЕМ ЗНАЧЕНИЯ ПЕРЕМЕННЫХ ИЗ EEPROM ///////////////////////////////
  //for(uint8_t Rd=0;Rd<20;Rd++){EEPROM.update(Rd,0);}//ОЧИСТКА ПАМЯТИ ДЛЯ САМОЙ ПЕРВОЙ ЗАГРУЗКИ
  //ПОСЛЕДНЯЯ ЗАДАННАЯ ТЕМПЕРАТУРА (130-390 ГРАДУСОВ)
  temp00 = EEPROM.read(0); temp01 = EEPROM.read(1); temp = word(temp00, temp01);
  //КОЭФФИЦИЕНТ ИНТЕГРИРОВАНИЯ (0.000-9.999)
  KiSet10 = EEPROM.read(2); KiSet11 = EEPROM.read(3); KiSet = word(KiSet10, KiSet11);
  //КОЭФФИЦИЕНТ ПРОПОРЦИОНАЛЬНОСТИ (0.00-9.99)
  KpSet10 = EEPROM.read(4); KpSet11 = EEPROM.read(5); KpSet = word(KpSet10, KpSet11);
  //КОЭФФИЦИЕНТ ДИФФЕРЕНЦИРОВАНЯ (0.00-9.99)
  KdSet10 = EEPROM.read(6); KdSet11 = EEPROM.read(7); KdSet = word(KdSet10, KdSet11);
  fan = EEPROM.read(8); //ПОТОК ВОЗДУХА ОТ ТУРБИНЫ (8 ЗНАЧЕНИЙ)
  led = EEPROM.read(9); //ПОКАЗЫВАТЬ ТЕМПЕРАТУРУ В РЕЖИМЕ "POWER OFF" (ДА / НЕТ)
  sharp = EEPROM.read(10); //ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ (5 ИЛИ 10 ГРАДУСОВ)
  autoret = EEPROM.read(11); //ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ (3 - 30 СЕКУНД)
  down = EEPROM.read(12); //ВРЕМЯ ЗАДЕРЖКИ ДО УХОДА В "POWER OFF" ЕСЛИ ФЕН НА БАЗЕ (1-99 МИНУТ)

  //ЗАДАЁМ КОЭФФИЦИЕНТЫ ПИД ПОСЛЕ ЧТЕНИЯ ИЗ ПАМЯТИ
  Kp = (double)KpSet / 100.0; Ki = (double)KiSet / 1000.0; Kd = (double)KdSet / 100.0;


}////////////////////////////////// END SETUP ////////////////////////////////////////


void loop() {
  //ЧТЕНИЕ ПОЛОЖЕНИЯ ЭНКОДЕРА И ПЕРЕЗАПИСЬ ПЕРЕМЕННОЙ mill
  newPos = myEnc.read() / 4; mill = millis();

  //КНОПКА ВКЛЮЧЕНИЯ POWER ON/OFF ЛОГИЧЕСКИЙ "0" НАЖАТА / "1" НЕ НАЖАТА
  btn9 = digitalRead(16); if (btn9 != oldbtn9) {
    delay(30); btn9 = digitalRead(16); \
    if (!btn9 && oldbtn9) {
      power = !power;
      f.w0 = true;
      clmil();
    } oldbtn9 = btn9;
  }

  //////////////////////////////////////////// POWER ON ////////////////////////////////////////////
  if (power == true) {
    //ЗАДАЁМ НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ РЕЖИМА "POWER ON"
    if (f.w0 == true) {
      f.w0 = false; f.w2 = true; f.w5 = true; f.w9 = true; fr = 3; menu = 0; clmil();
      LC.clearDisplay(0); LC.shutdown(0, false); //ЧИСТИМ ПАМЯТЬ У ИНДИКАТОРА И ВКЛЮЧАЕМ ЕГО
      //ОБЩЕЕ ВКЛЮЧЕНИЕ (ПОДАЁМ 230В НА ОДИН КОНЕЦ ТЭН И ВКЛЮЧАЕМ 24В БЛОК ПИТАНИЯ)
      digitalWrite(pin13, HIGH);
    }

    //ПОДАЁМ МНОГОТОНАЛЬНЫЙ ЗВУКОВОЙ СИГНАЛ ВКЛЮЧЕНИЯ ТЕРМОФЕНА
    if (fr > -1 && mill > timer3 + p100) {
      timer3 = mill;
      tone(11, freq[fr], p100);
      fr--;
    }
    temperatura();
    fakt = Input;

    //МИНИМАЛЬНАЯ ЗАЩИТА ОТ ПЕРЕГРЕВА ТЭН, ТЕРМОПАРА ПОВРЕЖДЕНА, ОБРЫВ КАБЕЛЯ, ПЛОХОЙ КОНТАКТ И Т.Д.\
    ПРОВЕРЯЕМ ЗНАЧЕНИЯ С ПОРТА "А0" НА ЗАВЕДОМО ЗАВЫШЕННОЕ, ЕСЛИ ПРЕВЫШАЕТ ТО ПЕРЕХОДИМ В "POWER OFF"
    if (inputdata > maxin) {
      power = false;
      f.w0 = true;
      clmil();
    }

    //КОНТРОЛЬ ДАТЧИКА ПОЛОЖЕНИЯ (ГЕРКОНА) В РУЧКЕ ТЕРМОФЕНА HIGH - ФЕН В РУКЕ, LOW - ФЕН НА БАЗЕ
    btn15 = digitalRead(pin15); if (btn15 != oldbtn15) {
      timer5 = mill;
      flag = true;
      oldbtn15 = btn15;
    }
    if (flag == true && mill - timer5 > p100 && btn15 == HIGH) {
      flag = false; f.w9 = true; menu = 0; clmil(); f.w2 = true;
    }//ФЕН В РУКЕ
    if (f.w9 == true && btn15 == LOW) {
      f.w9 = false;  //ФЕН НА БАЗЕ
      clmil();
      f.w2 = true;
    }

    //////////////////////////// ЕСЛИ ФЕН НА БАЗЕ, СТАВИМ ЕГО НА ПАУЗУ //////////////////////////////
    if (f.w9 == false) {
      if (f.w2 == true) {
        f.w2 = false; f.w5 = true; clful(); tone(11, 800, 350); f.w10 = true;
        detachInterrupt(0);//ОТКЛЮЧАЕМ ОТСЛЕЖИВАНИЕ ПРЕРЫВАНИЯ 0 НА pin D2
        digitalWrite(pin6, LOW); //ОТКЛЮЧАЕМ НАГРЕВ ТЭН
        if (temp == 0) {
          tempcold = minimtemp;
        } else {
          tempcold = temp;
        }
      }

      //ЕСЛИ ТЭН ГОРЯЧИЙ, ВКЛЮЧАЕМ ВЕНТИЛЯТОР ДЛЯ ОХЛАЖДЕНИЯ
      if (fakt > hot && f.w10 == true) {
        cold = map(fakt, hot, tempcold, 1, 8); analogWrite(pin10, pwm[cold]); fanspeed = cold;
      }
      else {
        digitalWrite(pin10, LOW);
        f.w10 = false;
        fanspeed = fan;
      }

      //МИГАЕМ ИНДИКАТОРОМ "POWER" СООБЩАЯ, ЧТО ФЕН ВКЛЮЧЕН, НО НАХОДИТСЯ НА ПАУЗЕ
      if (mill - timer4 > (f.w8 ? p1000 : p400)) {
        timer4 = mill;
        f.w8 = !f.w8;
        digitalWrite(pin9, f.w8);
      }
    }

    ///////////////////////////// ЕСЛИ ФЕН В РУКЕ, ВКЛЮЧАЕМ ЯДРО ФЕНА ///////////////////////////////
    if (f.w9 == true) {
      if (f.w2 == true) {
        f.w2 = false; f.w5 = true; clful(); tone(11, 2000, 350);
        attachInterrupt(0, tracking, RISING); //ПЕРЕДАЁМ УПРАВЛЕНИЕ ФУНКЦИЕЙ tracking ПРЕРЫВАНИЮ 0
        digitalWrite(pin9, HIGH); //ВКЛЮЧАЕМ ИНДИКАТОР "POWER" СООБЩАЯ, ЧТО ФЕН В РАБОТЕ
      }
      Setpoint = temp; //ОТСЫЛАЕМ ЗАДАННУЮ ТЕМПЕРАТУРУ В ПИД
      myPID.SetTunings(Kp, Ki, Kd); //УСТАНАВЛИВАЕМ КОЭФФИЦИЕНТЫ ПИД
      myPID.Compute();//ПРОПОРЦИОНАЛЬНО-ИНТЕГРАЛЬНОЕ ДИФФЕРЕНЦИРОВАНИЕ ТЕМПЕРАТУРЫ (ПИД)
      triac = Output; //РЕЗУЛЬТАТ РАБОТЫ ПИД ИСПОЛЬЗУЕМ В ФУНКЦИИ tracking
      analogWrite(pin10, pwm[fan]); //ПОДАЁМ НАПРЯЖЕНИЕ НА ВЕНТИЛЯТОР В СООТВЕТСТВИИ С ЗАДАННЫМ ЗНАЧЕНИЕМ
    }

    /////////////////////////////////////////// ГЛАВНОЕ МЕНЮ /////////////////////////////////////////
    if (menu == 0) {
      //ЗАПОЛНЯЕМ ПУСТЫМИ СИМВОЛАМИ НЕИСПОЛЬЗУЕМЫЕ В ДАННОМ МЕНЮ СЕГМЕНТЫ ИНДИКАТОРА
      if (f.w5 == true) {
        f.w5 = false;
        f.w7 = true;
        clful();
        dot0();
        ar4 = ar5 = ' ';
        m0 = 0;
        fanspeed = fan;
      }

      //НАЖАТИЕМ НА ЭНКОДЕР ВЫБИРАЕМ ТЕМПЕРАТУРУ ИЛИ ПОТОК ВОЗДУХА
      btn5 = digitalRead(pin5); if (btn5 != oldbtn5) {
        delay(10); btn5 = digitalRead(pin5);
        if (btn5 == LOW && oldbtn5 == HIGH) {
          ++m0;
          if (m0 > 1) {
            m0 = 0;
          } clmil();
          f.w4 = true;
        } oldbtn5 = btn5;
      }

      if (f.w1 == true) {
        switch (m0) {
          case 0: trig0 = temp; break;
          case 1: trig0 = fan; break;
        }
        f.w1 = false;
      }

      //МЕНЯЕМ ЗНАЧЕНИЯ ТЕМПЕРАТУРЫ ИЛИ ПОТОКА ВОЗДУХА
      if (newPos != oldPos) {
        if (m0 == 0) {
          trig0 = trig0 + (newPos * sharp);
        }
        else {
          trig0 = trig0 + newPos;
        } clful(); timer2 = mill + p500;
        switch (m0) {
          case 0: if (trig0 >= minimtemp) {
              temp = constrain(trig0, minimtemp, maxtemp);
            } else {
              if (trig0 < temp) {
                trig0 = 0;
              }
              else {
                trig0 = minimtemp;
              } temp = trig0;
            }; break;
          case 1: fan = constrain(trig0, 1, 8); break;
        }
        fanspeed = fan;
      }

      //ВЫВОДИМ ЗНАЧЕНИЯ НА ИНДИКАТОР КАЖДЫЕ 0,5 СЕК (чаще нет необходимости)
      if (mill - timer2 > p500) {
        timer2 = mill;
        ar1 = fakt / 100 % 10; if (ar1 == 0) {
          ar1 = ' ';
        } ar2 = fakt / 10 % 10; ar3 = fakt % 10;
        if (m0 == 0) {
          ar6 = temp / 100 % 10;
          ar7 = temp / 10 % 10;
          ar8 = temp % 10;
        } else {
          ar6 = 'F';
          ar7 = ' ';
          ar8 = fan;
        }
        dotfan(); menu0();
      }

      //ЕСЛИ ФЕН ОСТАЁТСЯ НА БАЗЕ ЗАДАННЫЙ ИНТЕРВАЛ, ПЕРЕХОДИМ В РЕЖИМ "POWER OFF"
      if (f.w9 == false && mill - timer1 > (down * p60000)) {
        power = false;
        f.w0 = true;
      }
    }//END MENU 0

    ////////////////////////////////// МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ /////////////////////////////////////
    // 1 ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ (5 ИЛИ 10 ГРАДУСОВ)
    // 2 ВРЕМЯ ДО АВТООТКЛЮЧЕНИЯ ФЕНА ПРИ ОТСУТСТВИИ АКТИВНОСТИ (ОТ 1 ДО 99 МИНУТ)
    // 3 ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ (ОТ 3 ДО 30 СЕКУНД)
    // 4 КОЭФФИЦИЕНТ ПРОПОРЦИОНАЛЬНОСТИ (0.00-9.99)
    // 5 КОЭФФИЦИЕНТ ИНТЕГРИРОВАНИЯ (0.000-9.999)
    // 6 КОЭФФИЦИЕНТ ДИФФЕРЕНЦИРОВАНИЯ (0.00-9.99)
    if (menu == 1) {
      if (f.w5 == true) {
        f.w5 = false; clful(); f.w6 = true; dot8(); tone(11, 3200, 250);
        ar2 = ar3 = ar4 = ar5 = ar6 = ar7 = ' ';
      }

      btn5 = digitalRead(pin5); if (btn5 != oldbtn5) {
        delay(10); btn5 = digitalRead(pin5);
        if (btn5 == LOW && oldbtn5 == HIGH) {
          LC.clearDisplay(0); timer1 = mill; f.w1 = true; f.w4 = true; f.w6 = !f.w6;
          if (f.w6 == true) {
            dot8();
          } if (f.w6 == false) {
            dot0();
          }
        } oldbtn5 = btn5;
      }

      if (f.w6 == true && newPos != oldPos) {
        m1 = m1 + newPos; if (m1 > 5) {
          m1 = 0;
        } if (m1 < 0) {
          m1 = 5;
        } clful();
        ar2 = ar3 = ar4 = ar5 = ar6 = ar7 = ' ';
      }
      if (f.w6 == false && newPos != oldPos) {
        trig1 = trig1 + newPos; clful();
        switch (m1) {
          //ТОЧНОСТЬ УСТАНОВКИ ТЕМПЕРАТУРЫ 5 ИЛИ 10 ГРАДУСОВ
          case 0: if (trig1 > 1) {
              trig1 = 0;
            } if (trig1 < 0) {
              trig1 = 1;
            }; sharp = zen[trig1]; break;
          case 1: down = constrain(trig1, 1, 99); break; //ВРЕМЯ ОТКЛЮЧЕНИЯ ФЕНА ПРИ ОТСУТСТВИИ АКТИВНОСТИ
          case 2: autoret = constrain(trig1, 3, 30); break; //ВРЕМЯ АВТОВОЗВРАТА В ОСНОВНОЕ МЕНЮ
          case 3: KpSet = constrain(trig1, 0, 999); Kp = ((double)KpSet) / 100.0; break;
          case 4: KiSet = constrain(trig1, 0, 9999); Ki = (double)KiSet / 1000.0; break;
          case 5: KdSet = constrain(trig1, 0, 999); Kd = ((double)KdSet) / 100.0; break;
        }
      }
      if (f.w1 == true) {
        menu1();
        f.w1 = false;
      }
    }//END MENU 1

    //ПЕРЕКЛЮЧЕНИЕ В МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ
    if (f.w4 == true && oldbtn5 == LOW && mill - timer1 > p500) {
      timer1 = mill; ++menu;
      if (menu > 1) {
        menu = 0; //
      } f.w5 = true; f.w3 = true; f.w4 = false;
    }

    //АВТОВОЗВРАТ В ГЛАВНОЕ МЕНЮ ИЗ МЕНЮ НАСТРОЙКИ ПАРАМЕТРОВ
    if (menu == 1 && mill - timer1 > (autoret * p1000)) {
      menu = 0;
      f.w5 = true;
    }

  }//END POWER ON

  ///////////////////////////////////// POWER OFF (STANDBY) ////////////////////////////////////////
  if (power == false) {
    //ЗАДАЁМ НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ РЕЖИМА "POWER OFF"
    if (f.w0 == true) {
      f.w0 = false;
      fr = 0;
      save = true;
      menu = 0;
      f.w5 = true;
      f.w10 = true;
      LC.clearDisplay(0);
    }

    //ПОДАЁМ ЗВУКОВОЙ СИГНАЛ ОТКЛЮЧЕНИЯ ФЕНА
    if (fr < 5 && mill > timer3 + p100) {
      timer3 = mill;
      tone(11, freq[fr], p100);
      fr++;
    }

    /////////////////////////////////////// ОСНОВНОЕ МЕНЮ ////////////////////////////////////////////
    if (menu == 0) {
      if (f.w5 == true) {
        f.w5 = false; clful(); dot0(); ar1 = ar7 = ar8 = ' '; triac = 0; f.w10 = true;
        digitalWrite(pin6, LOW); //ОТКЛЮЧАЕМ СИМИСТОР УПРАВЛЕНИЯ ТЭН
        digitalWrite(pin13, LOW); //ОТКЛЮЧАЕМ СИМИСТОР УПРАВЛЯЮЩИЙ ОБЩИМ ПИТАНИЕМ ФЕНА
        digitalWrite(pin9, LOW); //ОТКЛЮЧАЕМ ИНДИКАТОР "POWER ON"
        detachInterrupt(0);//ОТКЛЮЧАЕМ ОТСЛЕЖИВАНИЕ ПРЕРЫВАНИЯ 0 НА pin D2
        (led ? LC.shutdown(0, false) : LC.shutdown(0, true)); //ВКЛЮЧАЕМ ИЛИ ВЫКЛЮЧАЕМ ПОКАЗ ТЕМПЕРАТУРЫ
      }

      //ОБРАБАТЫВАЕМ НАЖАТИЕ НА ЭНКОДЕР
      btn5 = digitalRead(pin5); if (btn5 != oldbtn5) {
        delay(10); btn5 = digitalRead(pin5);
        if (btn5 == LOW && oldbtn5 == HIGH) {
          led = !led; (led ? LC.shutdown(0, false) : LC.shutdown(0, true));
          clmil(); f.w4 = true;
        } oldbtn5 = btn5;
      }

      if (mill - timer2 > p500) {
        timer2 = mill; temperatura(); //ВЫВОДИМ НА ИНДИКАТОР КАЖДЫЕ 0,5 СЕК
        fakt = Input; ar2 = fakt / 100 % 10; if (ar2 == 0) {
          ar2 = ' ';
        } ar3 = fakt / 10 % 10; ar4 = fakt % 10;
        menuOFF();
      }

      //ЕСЛИ ТЭН ГОРЯЧИЙ, ВКЛЮЧАЕМ ВЕНТИЛЯТОР
      if (fakt > hot && f.w10 == true) {
        digitalWrite(pin13, HIGH);
        cold = map(fakt, hot, tempcold, 1, 8); analogWrite(pin10, pwm[cold]);
      }
      else {
        f.w10 = false;
        digitalWrite(pin10, LOW);
        digitalWrite(pin13, LOW);
      }
    }//END MENU 0

    //ИНЖЕНЕРНОЕ МЕНЮ НАСТРОЙКИ УСИЛИТЕЛЯ ПОД ПАРАМЕТРЫ ТЕРМОПАРЫ \
    ВХОД В МЕНЮ ЧЕРЕЗ 2 СЕК. УДЕРЖАНИЯ НАЖАТОЙ КНОПКИ ЭНКОДЕРА
    if (menu == 1) {
      digitalWrite(pin13, HIGH); //ВКЛЮЧАЕМ ВЫХОД УПРАВЛЯЮЩИЙ ОБЩИМ ПИТАНИЕМ ФЕНА
      if (f.w5 == true) {
        f.w5 = false; clful(); dotfan(); ar4 = ar5 = ' '; LC.shutdown(0, false); f.w7 = true;
        attachInterrupt(0, tracking, RISING); tone(11, 3200, 250); triac = 0; analogWrite(pin10, pwm[fan]);
      }

      //ОБРАБАТЫВАЕМ НАЖАТИЕ НА ЭНКОДЕР
      btn5 = digitalRead(pin5); if (btn5 != oldbtn5) {
        delay(10); btn5 = digitalRead(pin5);
        if (btn5 == LOW && oldbtn5 == HIGH) {
          ++m2;
          if (m2 > 1) {
            m2 = 0;
          } clmil();
          f.w4 = true;
        } oldbtn5 = btn5;
      }
      temperatura();

      //ДЛЯ НАСТРОЙКИ ТЕМПЕРАТУРЫ ПОЛУЧАЕМОЙ С ТЕРМОПАРЫ, УПРАВЛЯЕМ СИМИСТОРОМ НАПРЯМУЮ
      if (newPos != oldPos) {
        triac = triac + (newPos * sharp); clful();
        triac = constrain(triac, 0, 255); timer2 = mill + p500;
      }
      if (mill - timer2 > p500) {
        timer2 = mill; fakt = Input;
        if (m2 == 0) {
          ar1 = inputdata / 1000 % 10;
          ar2 = inputdata / 100 % 10;
          ar3 = inputdata / 10 % 10;
          ar4 = inputdata % 10;
        }
        else {
          ar1 = ' ';
          ar2 = fakt / 100 % 10;
          if (fakt < 100) {
            ar2 = ' ';
          } ar3 = fakt / 10 % 10;
          ar4 = fakt % 10;
        }
        ar6 = (triac) / 100 % 10; if (ar6 == 0) {
          ar6 = ' ';
        } ar7 = (triac) / 10 % 10; ar8 = (triac) % 10;
        menu0();
      }
    }//END MENU 1

    //ПЕРЕХОД В ИНЖЕНЕРНОЕ МЕНЮ НАСТРОЙКИ УСИЛИТЕЛЯ (ПОДСТРОЙКА ПОД ПАРАМЕТРЫ ТЕРМОПАРЫ)
    if (f.w4 == true && oldbtn5 == LOW && mill - timer1 > p1000 + p1000) {
      timer1 = mill; ++menu;
      if (menu > 1) {
        menu = 0; //
      } led = true; f.w4 = false; f.w5 = true; LC.clearDisplay(0);
    }

  }//////////////////////////////////////// END POWER OFF ///////////////////////////////////////////

  ///////////////////////////////////////// EEPROM UPDATE ///////////////////////////////////////////
  if (save == true) {
    //ПЕРЕЗАПИСЫВАЕМ ЗНАЧЕНИЯ В EEPROM ПРИ КАЖДОМ ПЕРЕХОДЕ В РЕЖИМ "POWER OFF" ЕСЛИ ОНИ ИЗМЕНИЛИСЬ
    temp00 = highByte(temp); EEPROM.update(0, temp00); //ПИШЕМ СТАРШИЙ БАЙТ ПЕРЕМЕННОЙ temp
    temp01 = lowByte(temp); EEPROM.update(1, temp01); //ПИШЕМ МЛАДШИЙ БАЙТ ПЕРЕМЕННОЙ temp
    KiSet10 = highByte(KiSet); KiSet11 = lowByte(KiSet); EEPROM.update(2, KiSet10); EEPROM.update(3, KiSet11);
    KpSet10 = highByte(KpSet); KpSet11 = lowByte(KpSet); EEPROM.update(4, KpSet10); EEPROM.update(5, KpSet11);
    KdSet10 = highByte(KdSet); KdSet11 = lowByte(KdSet); EEPROM.update(6, KdSet10); EEPROM.update(7, KdSet11);
    EEPROM.update(8, fan); EEPROM.update(9, led); EEPROM.update(10, sharp); EEPROM.update(11, autoret);
    EEPROM.update(12, down);
    save = false;
  }

}/////////////////////////////////////////// END LOOP /////////////////////////////////////////////
