#include <Wire.h>
#include <avr/wdt.h>
#include <LiquidCrystal_I2C.h>

//служебный код для работы i2c дисплея 
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

//разметка выходов контроллера для контрольных ламп
const int P1_Alert = 2; //контрольная лампа датчика давления P1
const int P2_Alert = 3; //контрольная лампа датчика давления P2
const int P3_Alert = 4; //контрольная лампа датчика давления P3
const int T1_Alert = 5; //контрольная лампа датчика температуры T1
const int T2_Alert = 6; //контрольная лампа датчика температуры T2

#define speaker (7) //пин пищалки D7
#define alarm {digitalWrite(AlarmOut, HIGH);}

//функции включения контрольных ламп
#define AlertP1 {digitalWrite(P1_Alert, HIGH); delay(500); digitalWrite(P1_Alert, LOW); beep(200, 550);} //мигаем контрольной лампой давления P1
#define AlertP2 {digitalWrite(P2_Alert, HIGH); delay(500); digitalWrite(P2_Alert, LOW); beep(200, 550);} //мигаем контрольной лампой давления P2
#define AlertP3 {digitalWrite(P3_Alert, HIGH); delay(500); digitalWrite(P3_Alert, LOW); beep(200, 550);} //мигаем контрольной лампой давления P3
#define AlertT1 {digitalWrite(T1_Alert, HIGH); delay(500); digitalWrite(T1_Alert, LOW); beep(200, 550);} //мигаем контрольной лампой температуры T1
#define AlertT2 {digitalWrite(T2_Alert, HIGH); delay(500); digitalWrite(T2_Alert, LOW); beep(200, 550);} //мигаем контрольной лампой температуры T2

//перемычки включения мониторинга: наличие джампера означает включение мониторинга
const int P1_Control = 8; //перемычка датчика давления P1, пин D8
const int P2_Control = 9; //перемычка датчика давления P2, пин D9
const int P3_Control = 10; //перемычка датчика давления P3, пин D10
const int T1_Control = 11; //перемычка датчика температуры T1, пин D11
const int T2_Control = 12; //перемычка датчика температуры T2, пин D12

const int P1 = 1; //датчик давления 1, пин A1
const int P2 = 2; //датчик давления 2, пин A2
const int P3 = 3; //датчик давления 3, пин A3

const int T1 = 6; //датчик температуры 1, пин A6
const int T2 = 7; //датчик температуры 2, пин A7

//рабочие диапазоны датчиков
const int Pmax = 150;
const int Pmin = 0;
const int Tmax = 150;
const int Tmin = -50;

const int AlarmOut = 13; //выход реле тревоги

const double ADCStep = 5.0/1024; //шаг АЦП: 5 вольт, рарядность АЦП 10 бит (1024). 
const float SensorGis = 0.5; // значение "нулевого давления".
const int SensorPsi = 150; //диапазон датчика давления
const double Psi2Atm = 0.068046; //коэффициент преобразования

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display - инициализация дисплея.

#define _beep {for(byte i=0; i<50; i++) {digitalWrite(speaker, HIGH); delayMicroseconds(220); digitalWrite(speaker, LOW); delayMicroseconds(220);} digitalWrite(speaker, LOW);}

void beep(byte dur, int frq) // функция пищалки
{ 
  dur=(1000/frq)*dur; 
  for(byte i=0; i<dur; i++) 
  { 
   digitalWrite(speaker, HIGH);  
   //_delay_us(frq);
   delayMicroseconds(frq);  
   digitalWrite(speaker, LOW); 
   delayMicroseconds(frq); 
  }  
} 

void setup()
{
  //конфигурируем порты
  pinMode(P1_Alert, OUTPUT);
  pinMode(P2_Alert, OUTPUT);
  pinMode(P3_Alert, OUTPUT);
  pinMode(T1_Alert, OUTPUT);
  pinMode(T2_Alert, OUTPUT);
  pinMode(P1_Control, INPUT);
  pinMode(P2_Control, INPUT);
  pinMode(P3_Control, INPUT);
  pinMode(T1_Control, INPUT);
  pinMode(T2_Control, INPUT);
  pinMode(AlarmOut, OUTPUT);
  pinMode(speaker, OUTPUT);
  digitalWrite(P1_Control, HIGH);
  digitalWrite(P2_Control, HIGH);
  digitalWrite(P3_Control, HIGH);
  digitalWrite(T1_Control, HIGH);
  digitalWrite(T2_Control, HIGH);
  DDRC |= B00110000;
  
  //инициализируем дисплей
  lcd.init();                      // initialize the lcd 
  lcd.backlight();   
  lcd.home();
  
  Serial.begin(115200); //запускаем консольный вывод в ком-порт
  wdt_enable(WDTO_4S); //запускаем сторожевой таймер
}

double P(int P_number) //снимаем показания датчика давления; параметр функции - опрашиваемый датчик давления (P1, P2, P3)
{
  double VoltValue = ADCStep*analogRead(P_number)-SensorGis;
  double PsiValue = VoltValue*SensorPsi/4.5;
  double Atm=PsiValue*Psi2Atm;
  lcd.setCursor(0,0);
  Serial.print("P");
  lcd.print("P");
  Serial.print(P_number);
  lcd.print(P_number);
  Serial.print(": V=");
  Serial.print(VoltValue);
  Serial.print(" Psi=");
  lcd.print(" P=");
  Serial.print(PsiValue);
  lcd.print(PsiValue);
  Serial.print(" Atm=");
  lcd.print(" A=");
  Serial.println(Atm);
  lcd.print(Atm);
  delay(1000);
  wdt_reset();
  return Atm;
}

boolean CheckPRange(int P_number) //проверка значения датчика давления на вхождение в рабочий диапазон Pmin<=P<=Pmax
{
  double Pval=P(P_number);
  if ((Pval>=Pmin) && (Pval<=Pmax)) 
  {
    return true; //все в порядке, датчик в рабочем диапазоне
  }
  else //если нет
  {
    delay(300); //небольшая пауза
    Pval=P(P_number); //проверим еще раз
    if (!((Pval>=Pmin) && (Pval<=Pmax))) return false; //значение с датчика находится вне рабочего диапазона
  }
}

double T(int T_number)
{
  double val = analogRead(T_number);
  double temp = (val*5.0/1024)*100 - 273.15; //преобразование вольт-кельвин-цельсий
  lcd.setCursor(0,1);
  Serial.print("T");
  lcd.print("T");
  Serial.print(T_number);
  lcd.print(T_number);
  Serial.print(": ");
  lcd.print(": ");
  Serial.println(temp/2);
  lcd.print(temp/2);
  delay(1000);
  wdt_reset();
  return temp/2;
}

boolean CheckTRange(int T_number) //проверка значения датчика темепературы на вхождение в рабочий диапазон Tmin<=T<=Tmax
{
  double Tval=T(T_number);
  if ((Tval>=Tmin) && (Tval<=Tmax)) 
  {
    return true; //все в порядке, датчик в рабочем диапазоне
  }
  else //если нет
  {
    delay(300); //небольшая пауза
    Tval=T(T_number); //проверим еще раз
    if (!((Tval>=Tmin) && (Tval<=Tmax))) return false; //значение с датчика находится вне рабочего диапазона
  }
}

boolean SensorControl(int Sensor) //проверка датчика на предмет включенности. если перемычка установлена, то датчик задействован
{
  if (digitalRead(Sensor)==LOW) return true; else return false;  //джампер подтягивает пин к земле. Если состояние LOW, джампер установлен, датчик на мониторинге
}

void loop()
{
  wdt_reset(); //сброс сторожевого таймера
  if (SensorControl(P1_Control)) //проверяем, включен ли мониторинг датчика P1
  {
    if (!CheckPRange(P1)) //проверяем значение датчика
    {
      //проверку не прошли, пищим, включаем реле тревоги
      AlertP1;//контрольная лампа 1
      beep(500, 550);
      alarm; //включаем реле тревоги
    }
  }
  if (SensorControl(P2_Control)) //проверяем, включен ли мониторинг датчика P2
  {
    if (!CheckPRange(P2)) //проверяем значение датчика
    {
      //проверку не прошли, пищим, включаем реле тревоги
      AlertP2;//контрольная лампа 2
      beep(500, 550);
      alarm; //включаем реле тревоги
    }
  }
  if (SensorControl(P3_Control)) //проверяем, включен ли мониторинг датчика P3
  {
    if (!CheckPRange(P3)) //проверяем значение датчика
    {
      //проверку не прошли, пищим, включаем реле тревоги
      AlertP3; //контрольная лампа 3
      beep(500, 550);
      alarm; //включаем реле тревоги
    }
  }
  if (SensorControl(T1_Control)) //проверяем, включен ли мониторинг датчика T1
  {
    if (!CheckTRange(T1)) //проверяем значение датчика
    {
      //проверку не прошли, пищим, включаем реле тревоги
      AlertT1; //контрольная лампа 4
      beep(500, 550);
      alarm; //включаем реле тревоги
    }
  }
  if (SensorControl(T2_Control)) //проверяем, включен ли мониторинг датчика T2
  {
    //Serial.println("checking T7");
    if (!CheckTRange(T2)) //проверяем значение датчика
    {
      //проверку не прошли, пищим, включаем реле тревоги
      AlertT2; //контрольная лампа 5
      beep(500, 550);
      alarm; //включаем реле тревоги
    }
  }
}
  
