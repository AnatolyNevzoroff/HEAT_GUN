/////////////////////////////////////// ФУНКЦИИ / FUNCTIONS ////////////////////////////////////

//ФУНКЦИЯ СЧИТЫВАЕТ АНАЛОГОВЫЙ ВХОД samples КОЛИЧЕСТВО РАЗ И ВОЗВРАЩАЕТ СРЕДНЕ-МЕДИАННОЕ ЗНАЧЕНИЕ\
ОТСЕИВАЯ ВСЕ ЗНАЧЕНИЯ ОТЛИЧНЫЕ ОТ НАИБОЛЕЕ ЧАСТО ВСТРЕЧАЮЩИХСЯ В ОДНОЙ ВЫБОРКЕ ИЗ samples ЧИСЕЛ
int16_t median(uint8_t samples) {
  int16_t raw[samples];//МАССИВ ДЛЯ ХРАНЕНИЯ ДАННЫХ
  int16_t tem = 0; uint8_t i, j; //ВРЕМЕННЫЕ ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
  for (i = 0; i < samples; i++) {
    raw[i] = analogRead(A0); //ЧИТАЕМ ДАННЫЕ С А0 И ПИШЕМ ИХ В ЯЧЕЙКИ МАССИВА
  }
  for (i = 0; i < samples; i++) { //СОРТИРУЕМ МАССИВ ПО ВОЗРАСТАНИЮ ЗНАЧЕНИЙ В ЯЧЕЙКАХ
    for (j = 0; j < samples - 1; j++) { //АНОМАЛЬНО БОЛЬШИЕ И МАЛЫЕ В НАЧАЛО И КОНЕЦ МАССИВА
      if (raw[j] > raw[j + 1]) {
        tem = raw[j];
        raw[j] = raw[j + 1];
        raw[j + 1] = tem;
      }
    }
  }
  return raw[samples >> 1];
}//РЕГИСТРОВЫЙ СДВИГ >>1 ВОЗВРАЩАЕТ РЕЗУЛЬТАТ ДЕЛЕНИЯ НА 2 \
И В СЛЕДУЮЩЕМ ПРОХОДЕ ПОВТОРЯЕМ ЦИКЛ ВЫБОРКИ ДЛЯ ПЕРЕМЕННОЙ median

//ФУНКЦИЯ ОБРАБОТКИ ПРЕРЫВАНИЯ 0 ЧЕРЕЗ attachInterrupt, \
ОТСЛЕЖИВАЕТ ПЕРЕХОД СИНУСОИДЫ ЧЕРЕЗ "0" И ВКЛЮЧАЕТ ТЭН ВЫПОЛНЯЯ digitalWrite НА D6 ВЫХОДЕ \
ПРОПУСКАЯ НЕКОТОРОЕ КОЛЛИЧЕСТВО ПЕРЕХОДОВ СИНУСОИДЫ В ЗАВИСИМОСТИ ОТ ЗНАЧЕНИЯ triac \
255 - НЕ ПРОПУСКАЕМ НИ ОДНОГО, 0 - ПРОПУСКАЕМ ВСЕ
void tracking() {
  static byte count, last, lastVal; //ВРЕМЕННЫЕ ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
  int val = (++count * triac) >> 8; //РЕГИСТРОВЫЙ СДВИГ x>>8 АНАЛОГИЧНО x/(2^8) АНАЛОГИЧНО x/256
  if (lastVal != (val != last)) {
    digitalWrite(pin6, val != last);
  }
  lastVal = (val != last); last = val;
}

void temperatura() {
  inputdata = median(15); //ЧИТАЕМ ОТФИЛЬТРОВАННЫЕ СРЕДНЕ-МЕДИАННЫЕ ЗНАЧЕНИЯ С ПОРТА "А0"
  Input = map(inputdata, 0, maxin, mintemp, maxtemp); //ПЕРЕВОДИМ ДАННЫЕ С АЦП В ТЕМПЕРАТУРУ
}

void menu0() { //ПЕЧАТАЕМ ЗНАЧЕНИЯ В ОСНОВНОМ МЕНЮ  setDigit  setRow
  LC.setChar(0, 7, ar1, D.L0);
  LC.setChar(0, 6, ar2, D.L1);
  LC.setChar(0, 5, ar3, D.L2);
  LC.setChar(0, 4, ar4, D.L3);
  LC.setChar(0, 3, ar5, D.L4);
  LC.setChar(0, 2, ar6, D.L5);
  LC.setChar(0, 1, ar7, D.L6);
  LC.setChar(0, 0, ar8, D.L7);
}

void menuOFF() { //ПЕЧАТАЕМ ЗНАЧЕНИЯ ТЕМПЕРАТУРЫ В РЕЖИМЕ "POWER OFF" setDigit  setRow
  //LC.setChar(0,7, ar1, D.L0);
  LC.setChar(0, 6, ar2, D.L1);
  LC.setChar(0, 5, ar3, D.L2);
  LC.setChar(0, 4, ar4, D.L3);
  LC.setRow(0, 3, B01100011);
  LC.setRow(0, 2, B01001110);
}

//ОБНУЛЯЕМ ТАЙМЕР, ПОДНИМАЕМ ФЛАГ
void clmil() {
  timer1 = mill;
  f.w1 = true;
}

//ОБНУЛЯЕМ ЭНКОДЕР И ОСНОВНОЙ ТАЙМЕР 1
void clful() {
  myEnc.write(0);
  oldPos = 0;
  newPos = 0;
  timer1 = mill;
  f.w1 = true;
}

void dotfan() { //ВЫБИРАЕМ ФУНКЦИЮ ОТОБРАЖЕНИЯ ТОЧЕК ИНДИЦИРУЮЩИХ СКОРОСТЬ ВЕНТИЛЯТОРА
  switch (fanspeed) {
    case 1: dot1(); break;
    case 2: dot2(); break;
    case 3: dot3(); break;
    case 4: dot4(); break;
    case 5: dot5(); break;
    case 6: dot6(); break;
    case 7: dot7(); break;
    case 8: dot8(); break;
  }
}

//ПАКЕТ ФУНКЦИЙ ДЛЯ ИНДИКАЦИИ ТОЧЕК
void dot8() {
  D.L0 = true;
  D.L1 = true;
  D.L2 = true;
  D.L3 = true;
  D.L4 = true;
  D.L5 = true;
  D.L6 = true;
  D.L7 = true;
}
void dot7() {
  D.L0 = true;
  D.L1 = true;
  D.L2 = true;
  D.L3 = true;
  D.L4 = true;
  D.L5 = true;
  D.L6 = true;
  D.L7 = false;
}
void dot6() {
  D.L0 = true;
  D.L1 = true;
  D.L2 = true;
  D.L3 = true;
  D.L4 = true;
  D.L5 = true;
  D.L6 = false;
  D.L7 = false;
}
void dot5() {
  D.L0 = true;
  D.L1 = true;
  D.L2 = true;
  D.L3 = true;
  D.L4 = true;
  D.L5 = false;
  D.L6 = false;
  D.L7 = false;
}
void dot4() {
  D.L0 = true;
  D.L1 = true;
  D.L2 = true;
  D.L3 = true;
  D.L4 = false;
  D.L5 = false;
  D.L6 = false;
  D.L7 = false;
}
void dot3() {
  D.L0 = true;
  D.L1 = true;
  D.L2 = true;
  D.L3 = false;
  D.L4 = false;
  D.L5 = false;
  D.L6 = false;
  D.L7 = false;
}
void dot2() {
  D.L0 = true;
  D.L1 = true;
  D.L2 = false;
  D.L3 = false;
  D.L4 = false;
  D.L5 = false;
  D.L6 = false;
  D.L7 = false;
}
void dot1() {
  D.L0 = true;
  D.L1 = false;
  D.L2 = false;
  D.L3 = false;
  D.L4 = false;
  D.L5 = false;
  D.L6 = false;
  D.L7 = false;
}
void dot0() {
  D.L0 = false;
  D.L1 = false;
  D.L2 = false;
  D.L3 = false;
  D.L4 = false;
  D.L5 = false;
  D.L6 = false;
  D.L7 = false;
}

void menu1() { //ПЕЧАТАЕМ НА ЭКРАНЕ ЗНАЧЕНИЯ ДЛЯ НАСТРОЙКИ В МЕНЮ № 1
  uint8_t i;
  switch (m1) {
    case 0: for (i = 0; i < 2; i++) {
        if (sharp == zen[i]) {
          trig1 = i;
        }
      }; ar1 = 1; ar7 = sharp / 10 % 10;
      if (ar7 == 0) {
        ar7 = ' ';
      } ar8 = sharp % 10; break;
    case 1: trig1 = down; ar1 = 2; ar7 = down / 10 % 10; if (ar7 == 0) {
        ar7 = ' ';
      } ar8 = down % 10; break;
    case 2: trig1 = autoret; ar1 = 3; ar7 = autoret / 10 % 10; if (ar7 == 0) {
        ar7 = ' ';
      } ar8 = autoret % 10; break;
    case 3: trig1 = KpSet; ar1 = 4; ar6 = KpSet / 100 % 10; D.L5 = true; ar7 = KpSet / 10 % 10; ar8 = KpSet % 10; break;
    case 4: trig1 = KiSet; ar1 = 5; ar5 = KiSet / 1000 % 10;; D.L4 = true; ar6 = KiSet / 100 % 10; ar7 = KiSet / 10 % 10;
      ar8 = KiSet % 10; break;
    case 5: trig1 = KdSet; ar1 = 6; ar6 = KdSet / 100 % 10; D.L5 = true; ar7 = KdSet / 10 % 10; ar8 = KdSet % 10; break;
  }
  menu0();
}
