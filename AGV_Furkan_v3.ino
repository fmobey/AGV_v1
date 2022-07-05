#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define Sensor_Forward 24
#define Sensor_Back 26
#define Sensor_Right 28
#define Sensor_Left 30

#define A_Palse 8
#define A_Alarm 4
#define A_Enabled 12
#define A_Dir 10

#define B_Palse 44
#define B_Alarm 40
#define B_Enabled 48
#define B_Dir 46

#define RedPin 43
#define GreenPin 45
#define BluePin 47
#define Siren 49

#define C_Palse 13
#define C_Alarm 23
#define C_Enabled 29
#define C_Dir 27
/*
Red43 Green45 Blue47 49Siren
*/
#define D_Palse 9
#define D_Alarm 5
#define D_Enabled 22
#define D_Dir 11
int RGBCount = 0;
int AnalogCount = 0;

int hiz = 680;
int speedPlusButton = 0;
int speedNegativeButton = 0;
String MotorStateMessagesText = "";

int pulsestate = 0;

unsigned long time_now = 0;
unsigned long time_now_AZ = 0;
unsigned long time_now2 = 0;
unsigned long time_now3 = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define ILERI_PIN A6
#define GERI_PIN A4
#define SAGA_PIN A0
#define SOLA_PIN A2
#define SPEEDPLUSPIN A8
#define SPEEDNEGATIVEPIN A10

long lastMsg = 0;
long last_RPM_Msg = 0;
long TIME_NOW = 0;
long TIME_RPM_NOW = 0;

int A_Alarm_val = 0;
int B_Alarm_val = 0;
int C_Alarm_val = 0;
int D_Alarm_val = 0;

#define ACCELARATION_VALUE 20
static float acceleration_value = ACCELARATION_VALUE;
int Ramp_function(int maxspeed, float acceleration);
bool ramp_decrease = false;

int yuzde = 0;
int VbattRaw;
int Vbatt;
int VbattAver;
int x,y;
int acc_x = 0;
int acc_x1 = 0;

int avarage_x,avarage_y,avarage_z = 0;

unsigned long adc_millis = 0;

void setup()
{

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("      OMNIMAN       ");
  Wire.begin();
  Serial.begin(115200);

  delay(1000);

  pinMode(RedPin, OUTPUT);
  pinMode(GreenPin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  pinMode(Siren, OUTPUT);

  pinMode(A_Palse, OUTPUT);
  pinMode(B_Palse, OUTPUT);
  pinMode(C_Palse, OUTPUT);
  pinMode(D_Palse, OUTPUT);

  pinMode(Sensor_Forward, INPUT);

  pinMode(A_Dir, OUTPUT);
  pinMode(A_Alarm, INPUT);
  pinMode(A_Enabled, OUTPUT);

  pinMode(B_Dir, OUTPUT);
  pinMode(B_Alarm, INPUT);
  pinMode(B_Enabled, OUTPUT);

  pinMode(C_Dir, OUTPUT);
  pinMode(C_Alarm, INPUT);
  pinMode(C_Enabled, OUTPUT);

  pinMode(D_Dir, OUTPUT);
  pinMode(D_Alarm, INPUT);
  pinMode(D_Enabled, OUTPUT);

  digitalWrite(A_Enabled, LOW);
  digitalWrite(A_Dir, HIGH);

  digitalWrite(B_Enabled, LOW);
  digitalWrite(B_Dir, HIGH);

  digitalWrite(C_Enabled, LOW);
  digitalWrite(C_Dir, LOW);

  digitalWrite(D_Enabled, LOW);
  digitalWrite(D_Dir, HIGH);
  digitalWrite(GreenPin, HIGH);
  delay(500);
  digitalWrite(GreenPin, LOW);
  digitalWrite(RedPin, HIGH);
  delay(500);
  digitalWrite(RedPin, LOW);
  digitalWrite(BluePin, HIGH);
  delay(500);
  digitalWrite(BluePin, LOW);
  lcd.clear();

  LCD_MESSAGE(hiz);
}

void loop()
{






 
  VbattRaw = analogRead(15); 
 
    // 25=1v min 10v max 14v 
    
  Vbatt = VbattRaw/25;
 //Serial.println(Vbatt);

if (Vbatt < 21)
    {
      
        digitalWrite(Siren, HIGH);
       // digitalWrite(RedPin, HIGH);  
    }



  TIME_NOW = millis();
  if (TIME_NOW - adc_millis > 1)
  {
    adc_millis = TIME_NOW;
    speedControlButton();
        AnalogKumandaReadAdc();

    speedPlusButton = digitalRead(SPEEDPLUSPIN);
    speedNegativeButton = digitalRead(SPEEDNEGATIVEPIN);
  }


  // int Forward_Sensor_Val = digitalRead(Sensor_Forward);

  int ILERI_TRY = digitalRead(ILERI_PIN);
  int GERI_TRY = digitalRead(GERI_PIN);
  int SAGA_TRY = digitalRead(SAGA_PIN);
  int SOLA_TRY = digitalRead(SOLA_PIN);

  // A_Alarm_val = digitalRead(A_Alarm);
  // B_Alarm_val = digitalRead(B_Alarm);
  // C_Alarm_val = digitalRead(C_Alarm);
  // D_Alarm_val = digitalRead(D_Alarm);

  if (SAGA_TRY == LOW || (avarage_x < 550 && avarage_x >= 490))
  {

    RotateWheels(false, false, false, true, false, false);
    Ramp_function((speedControlButton() / 2), 0.1);
    RGBController(false, true, false);
    ramp_decrease = true;
    //Serial.println("SAGA");
  }
  else if (GERI_TRY == LOW || (avarage_x < 350 && avarage_x >= 290))
  {

    RotateWheels(true, false, false, false, false, false);
    Ramp_function(speedControlButton(), 0.1);
    RGBController(true, false, false);
    ramp_decrease = true;
   // Serial.println("GERI");
  }
  else if (ILERI_TRY == LOW || (avarage_x < 750 && avarage_x >= 685))
  {

    RotateWheels(false, true, false, false, false, false);
    Ramp_function(speedControlButton(), 0.1);
    RGBController(true, false, false);

   //Serial.println("ILERI");
  }
  else if (SOLA_TRY == LOW || (avarage_x <= 80 && avarage_x >= 0))
  {

    RotateWheels(false, false, true, false, false, false);
    RGBController(false, true, false);
    Ramp_function((speedControlButton() / 2), 0.1);
   // Serial.println("SOLA");
  }
  else if ((avarage_x <= 860 && avarage_x >= 800)) // BEYAZ BUTON
  {

    RotateWheels(false, false, false, false, true, false);
    RGBController(false, true, false);
    Ramp_function((speedControlButton() / 2), 0.1);
  //  Serial.println("SOLADON");
  }
  else if ((avarage_x <= 980 && avarage_x >= 870)) // SIYAH BUTON
  {

    RotateWheels(false, false, false, false, false, true);
    RGBController(false, true, false);
    Ramp_function((speedControlButton() / 2), 0.1);
   
  //  Serial.println("SAGADON");
  }
  else if (ILERI_TRY == HIGH && SAGA_TRY == HIGH && SOLA_TRY == HIGH && GERI_TRY == HIGH || (avarage_x <= 1020 && avarage_x >= 980))
  {

    Ramp_reset(speedControlButton(), 2);
    RGBController(false, false, true);
 
  }
  else
  {
    Ramp_reset(speedControlButton(), 2);

    RGBController(false, false, true);
  }
}

int speedControlButton()
{
  if (speedPlusButton == LOW)
  {
    if (hiz < 700)
    {
      hiz = hiz + 10;
      LCD_MESSAGE(hiz);
    }
  }
  if (speedNegativeButton == LOW)
  {
    if (hiz > 140)
    {
      hiz = hiz - 10;
      LCD_MESSAGE(hiz);
    }
  }
  return hiz;
}

void RotateWheels(bool ileri, bool geri, bool sag, bool sol, bool soldon, bool sagdon)
{
  
  if (ileri == true)
  {
    digitalWrite(A_Dir, HIGH);
    digitalWrite(B_Dir, LOW);
    digitalWrite(C_Dir, LOW);
    digitalWrite(D_Dir, HIGH);
  }
  else if (geri == true)
  {
    digitalWrite(A_Dir, LOW);
    digitalWrite(B_Dir, HIGH);
    digitalWrite(C_Dir, HIGH);
    digitalWrite(D_Dir, LOW);
  }
  else if (sag == true)
  {
    digitalWrite(A_Dir, LOW);
    digitalWrite(B_Dir, LOW);
    digitalWrite(C_Dir, HIGH);
    digitalWrite(D_Dir, HIGH);
  }
  else if (sol == true)
  {
    digitalWrite(A_Dir, HIGH);
    digitalWrite(B_Dir, HIGH);
    digitalWrite(C_Dir, LOW);
    digitalWrite(D_Dir, LOW);
  }
  else if (soldon == true)
  {
    digitalWrite(A_Dir, HIGH);
    digitalWrite(B_Dir, LOW);
    digitalWrite(C_Dir, HIGH);
    digitalWrite(D_Dir, LOW);
  }
  else if (sagdon == true)
  {
    digitalWrite(A_Dir, LOW);
    digitalWrite(B_Dir, HIGH);
    digitalWrite(C_Dir, LOW);
    digitalWrite(D_Dir, HIGH);
  }
  else
  {
  }
}

void MotorStateMessages()
{
  if (A_Alarm_val == HIGH)
  {
    MotorStateMessagesText = "A MOT. ERR";
  }
  else if (B_Alarm_val == HIGH)
  {
    MotorStateMessagesText = "B MOT. ERR";
  }
  else if (C_Alarm_val == HIGH)
  {
    MotorStateMessagesText = "C MOT. ERR";
  }
  else if (D_Alarm_val == HIGH)
  {
    MotorStateMessagesText = "D MOT. ERR";
  }
  else if (A_Alarm_val == LOW && B_Alarm_val == LOW && C_Alarm_val == LOW && D_Alarm_val == LOW)
  {
    MotorStateMessagesText = " UYARI YOK";
  }
  else
  {
  }
}

void LCD_MESSAGE(int hiz)
{
  lcd.setCursor(0, 0);
  lcd.print("      OMNIMAN       ");
  lcd.setCursor(0, 1);
  lcd.print("RGB MOD : ON");
  lcd.setCursor(0, 2);
  lcd.print("AKU:");
  lcd.setCursor(5, 2);
  lcd.print(Vbatt);
   lcd.setCursor(7, 2);
  lcd.print("V");
  lcd.setCursor(11, 2);
  lcd.print("%");
   lcd.setCursor(13, 2);   
   // 28v = 100% 20v = %5 
   yuzde = (Vbatt - 16)*10; 
   
  lcd.print(yuzde);
    lcd.setCursor(15, 2);
  lcd.print("   ");
  
  lcd.setCursor(0, 3);
  lcd.print("HIZ: ");
  lcd.setCursor(6, 3);
  int mBoluDK = hiz/60;
  
  lcd.print(mBoluDK);
  lcd.setCursor(9, 3);
  lcd.print("metre/dk");
}
/*
void lcdReflesh(unsigned long RPMValue)
{
  if (RPMValue < 10)
  {

    lcd.print("    ");
  }
  else if (RPMValue < 100)
  {

    lcd.print("   ");
  }
  else if (RPMValue < 1000)
  {

    lcd.print("  ");
  }
  else if (RPMValue < 10000)
  {

    lcd.print(' ');
  }
  else
  {
  }
}
*/
int Ramp_function(int maxspeed, float acceleration)
{

  static int ramp_time_ms = 1000; // 2000=2sn
  static unsigned long period_micros = 0;

   ramp_decrease = true;

  if (millis() > time_now2 + 10)
  {

    digitalWrite(A_Enabled, LOW);
    digitalWrite(B_Enabled, LOW);
    digitalWrite(C_Enabled, LOW);
    digitalWrite(D_Enabled, LOW);

    time_now2 = millis();
    if (acceleration_value > 1 + acceleration)
    {
      acceleration_value = acceleration_value - acceleration;
    }
  }

  period_micros = (100000 / maxspeed) * acceleration_value;

  if (micros() > time_now + period_micros)
  {
    time_now = micros();

    pulsestate = !pulsestate;

    digitalWrite(A_Palse, pulsestate);
    digitalWrite(B_Palse, pulsestate);
    digitalWrite(C_Palse, pulsestate);
    digitalWrite(D_Palse, pulsestate);
  }
}

void Ramp_reset(int maxspeed, float acceleration)
{
  static long time_ramp = 0;
  static long time_ramp_millis = 0;
  static unsigned long period_micros = 0;

  if (ramp_decrease == true)
  {

    if (millis() > time_ramp_millis + 100)
    {
      time_ramp_millis = millis();
      acceleration_value = acceleration_value + acceleration;
      if (acceleration_value > ACCELARATION_VALUE)
      {
        ramp_decrease = false;
      }
    }

    period_micros = (100000 / maxspeed) * acceleration_value;

    if (micros() > time_ramp + period_micros)
    {
      time_ramp = micros();

      pulsestate = !pulsestate;

      digitalWrite(A_Palse, pulsestate);
      digitalWrite(B_Palse, pulsestate);
      digitalWrite(C_Palse, pulsestate);
      digitalWrite(D_Palse, pulsestate);
    }
  }else{
    digitalWrite(A_Enabled, HIGH);
    digitalWrite(B_Enabled, HIGH);
    digitalWrite(C_Enabled, HIGH);
    digitalWrite(D_Enabled, HIGH);
  acceleration_value = ACCELARATION_VALUE;
  }
     
}

void AnalogKumandaReadAdc()
{
  static int indexsamples = 0;
    static int indexsamples1 = 0;

  x = analogRead(9);
      acc_x1 = acc_x1 + x;
    indexsamples1++;

  if (indexsamples1 >= 4)
  {
   indexsamples1 = 0;
    avarage_y = acc_x1 / 4;
    y=avarage_y;
    acc_x1 = 0;
  }

  if(!(985<=y && y <=1023)){
  acc_x = acc_x + y;
  indexsamples++;
  if (indexsamples >= 16)
  {

    indexsamples = 0;
    avarage_x = acc_x / 16;
    acc_x = 0;      
   Serial.println(avarage_x);

  }}else{
    avarage_x=1023;
    //Serial.println("muzaffer in tatli ruyasi");
    }
}

/*
void AnalogKumandaReadAdc()
{
  static int indexsamples = 0;
    x = analogRead(9);

  acc_x = acc_x + (x);
  indexsamples++;
  if (indexsamples >= 8)
  {

    indexsamples = 0;
    if (AnalogCount == 1)
      {
    avarage_y = acc_x / 8;

      }
      else if (AnalogCount == 2)
      {
    avarage_z = acc_x / 8;

      }
      else
      {
        AnalogCount = 0;
      }
      AnalogCount++;
if(avarage_z-avarage_y<10 || avarage_y-avarage_z<10){
  avarage_x=(avarage_y+avarage_z)/2;
}else{
            Serial.println("HATAAAA");

  }
}
    acc_x = 0;
            Serial.println("X");
            Serial.println(avarage_x);
            Serial.println("y");
            Serial.println(avarage_y);
            Serial.println("Z");
                        Serial.println(avarage_z);



  
}
*/
void RGBController(bool straight, bool lateral, bool constant)
{
  
  TIME_NOW = millis();

  if (TIME_NOW - lastMsg > 600)
  {
    lastMsg = TIME_NOW;
    if (straight)
    {
      if (RGBCount == 1)
      {
        digitalWrite(Siren, LOW);
        digitalWrite(RedPin, LOW);
        digitalWrite(GreenPin, LOW);
        digitalWrite(BluePin, LOW);
      }
      else if (RGBCount == 2)
      {
        digitalWrite(Siren, HIGH);
        digitalWrite(RedPin, HIGH);
        digitalWrite(GreenPin, LOW);
        digitalWrite(BluePin, LOW);
      }
      else
      {
        RGBCount = 0;
      }
      RGBCount++;
    }
    else if (lateral)
    {

      if (RGBCount == 1)
      {
        digitalWrite(Siren, LOW);
        digitalWrite(RedPin, LOW);
        digitalWrite(GreenPin, LOW);
        digitalWrite(BluePin, LOW);
      }
      else if (RGBCount == 2)
      {
        digitalWrite(Siren, HIGH);
        digitalWrite(RedPin, LOW);
        digitalWrite(GreenPin, LOW);
        digitalWrite(BluePin, HIGH);
      }
      else
      {
        RGBCount = 0;
      }
      RGBCount++;
    }
    else if (constant)
    {
      if (RGBCount == 1)
      {
        digitalWrite(Siren, LOW);
        digitalWrite(RedPin, LOW);
        digitalWrite(GreenPin, LOW);
        digitalWrite(BluePin, LOW);
      }
      else if (RGBCount == 2)
      {
        digitalWrite(Siren, LOW);
        digitalWrite(RedPin, LOW);
        digitalWrite(GreenPin, HIGH);
        digitalWrite(BluePin, LOW);
        LCD_MESSAGE(hiz);
       } 
          else if (Vbatt < 21)
      {
        digitalWrite(Siren, LOW);
        digitalWrite(RedPin, LOW);
        digitalWrite(GreenPin, LOW);
        digitalWrite(BluePin, HIGH);
        LCD_MESSAGE(hiz);    
        digitalWrite(Siren, HIGH);
      }
      else
      {
        RGBCount = 0;
      }
      RGBCount++;
    }
    else
    {
    }
  }
}
