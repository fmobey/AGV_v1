#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// A MOTOR
#define APWM 12
#define ADIR 10
#define ADIRFW 8
// B MOTOR
#define BPWM 46
#define BDIR 48
#define BDIRFW 30
// C MOTOR
#define CPWM 45
#define CDIR 25
#define CDIRFW 23
// D MOTOR
#define DPWM 13
#define DDIR 11
#define DDIRFW 9
// LINE FOLLOWING SENSOR
#define SENSOR_PIN_RIGHT_1 A6
#define SENSOR_PIN_RIGHT_2 A7
#define SENSOR_PIN_RIGHT_3 A1
#define SENSOR_PIN_RIGHT_4 A3
#define SENSOR_PIN_RIGHT_5 A5
#define SENSOR_PIN_RIGHT_6 A2
#define SENSOR_PIN_RIGHT_7 A4

#define SENSOR_PIN_LEFT_1 A12
#define SENSOR_PIN_LEFT_2 32
#define SENSOR_PIN_LEFT_3 A8
#define SENSOR_PIN_LEFT_4 A9
#define SENSOR_PIN_LEFT_5 A11
#define SENSOR_PIN_LEFT_6 A10
#define SENSOR_PIN_LEFT_7 A13

#define SENSOR_PIN_FRONT_1 16
#define SENSOR_PIN_FRONT_2 15
#define SENSOR_PIN_FRONT_3 50
#define SENSOR_PIN_FRONT_4 52
#define SENSOR_PIN_FRONT_5 51
#define SENSOR_PIN_FRONT_6 53
#define SENSOR_PIN_FRONT_7 14

#define SENSOR_PIN_BACK_1 49
#define SENSOR_PIN_BACK_2 47
#define SENSOR_PIN_BACK_3 34
#define SENSOR_PIN_BACK_4 36
#define SENSOR_PIN_BACK_5 38
#define SENSOR_PIN_BACK_6 40
#define SENSOR_PIN_BACK_7 42
// LINE FOLLOWING SENSOR ENABLE PIN

// STANBY
#define STANBY_PIN 6
// CONTROL PANEL BUTTONS
#define ILERI_PIN 43
#define GERI_PIN 41
#define SAGA_PIN 7
#define SOLA_PIN 44
#define SOLA_DON_PIN 29
#define SAGA_DON_PIN 31
// STATİONS BUTTONS
#define ANALOG_PIN_BUTTON A0
// BATTERY
#define VBATTADC 14
#define BATTERY_AVARAGE_READ_COUNT 16
// AMPERAGE PIN
#define AMPERAGE_PIN A15
// RGB
#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4
// BUZZER
#define BUZZER_PIN 5
// PWM VALUE
#define PWM_START 255
#define PWM_STOP 0
// LİDAR WARNING PIOUT
#define WARN_LIDAR11 39
#define WARN_LIDAR12 37
#define WARN_LIDAR21 35
#define WARN_LIDAR22 33
// MILLIS
unsigned long oldTime = 0;
unsigned long newTime;
unsigned long oldTime1 = 0;
unsigned long newTime1;
unsigned long oldTime2 = 0;
unsigned long newTime2;
unsigned long oldTime3 = 0;
unsigned long newTime3;
unsigned long oldTime4 = 0;
unsigned long newTime4;
unsigned long oldTime5 = 0;
unsigned long newTime5;
unsigned long oldTime6 = 0;
unsigned long newTime6;
int buzzerCount = 0;
#define FRONT_SENSOR 0
#define BACK_SENSOR 1
#define RIGHT_SENSOR 2
#define LEFT_SENSOR 3
int sensorStraightStatus = 0;
int sensorLateralStatus = 0;
int chargeStatus = -1;
long TIME_NOW = 0;
long TIME_NOW1 = 0;
long TIME_NOW2 = 0;
long TIME_NOW3 = 0;
long TIME_NOW4 = 0;
unsigned long adc_millis = 0;
unsigned long adc_millis1 = 0;
unsigned long adc_millis2 = 0;
unsigned long adc_millis3 = 0;
unsigned long adc_millis4 = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

int ileriAnalogValue = 686;
int geriAnalogValue = 333;

// panel buttonarının durumlarını okuyup gösteriyoruz
int ILERI_TRY = -1;
int GERI_TRY = -1;
int SAGA_TRY = -1;
int SOLA_TRY = -1;
int SAGA_DON = -1;
int SOLA_DON = -1;
// istasyon butonunun durumlarını okuyup gösteriyoruz
int ANALOG_GIT_BUTTON = -1;
// Sensor Pinlerini durumlarını okuyup gösteriyoruz
int SensorFront1 = -1;
int SensorFront2 = -1;
int SensorFront3 = -1;
int SensorFront4 = -1;
int SensorFront5 = -1;
int SensorFront6 = -1;
int SensorFront7 = -1;
int SensorBack1 = -1;
int SensorBack2 = -1;
int SensorBack3 = -1;
int SensorBack4 = -1;
int SensorBack5 = -1;
int SensorBack6 = -1;
int SensorBack7 = -1;
int SensorRight1 = -1;
int SensorRight2 = -1;
int SensorRight3 = -1;
int SensorRight4 = -1;
int SensorRight5 = -1;
int SensorRight6 = -1;
int SensorRight7 = -1;
int SensorLeft1 = -1;
int SensorLeft2 = -1;
int SensorLeft3 = -1;
int SensorLeft4 = -1;
int SensorLeft5 = -1;
int SensorLeft6 = -1;
int SensorLeft7 = -1;
int rgbState = -1;
int followStationState = -1;
int followLateralState = -1;
int followLateralState2 = -1;
int followStopState = 0;
int buzzerStateLoop = -1;
int Warn_Lidar11 = -1;
int Warn_Lidar12 = -1;
int Warn_Lidar21 = -1;
int Warn_Lidar22 = -1;

int sayacStop = 0;
int sayacStop2 = 0;
int agvDirection = 0;

unsigned long sayacmsduraklama = 0;
void setup()
{

  Wire.begin();
  Serial.begin(115200);

  // MOTOR PİNMODE
  pinMode(APWM, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(CPWM, OUTPUT);
  pinMode(DPWM, OUTPUT);

  analogWrite(APWM, 0);
  analogWrite(BPWM, 0);
  analogWrite(CPWM, 0);
  analogWrite(DPWM, 0);

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("      OMNIMAN       ");

  delay(1000);
  // RGB PİNMODE
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // VOLTAGE SENSOR PİNMODE
  pinMode(VBATTADC, INPUT);
  // AMPERAGE PİNMODE
  pinMode(AMPERAGE_PIN, INPUT);
  // CONTROL BUTTON PİNMODE
  pinMode(ILERI_PIN, INPUT_PULLUP);
  pinMode(GERI_PIN, INPUT_PULLUP);
  pinMode(SAGA_PIN, INPUT_PULLUP);
  pinMode(SOLA_PIN, INPUT_PULLUP);
  pinMode(SOLA_DON_PIN, INPUT_PULLUP);
  pinMode(SAGA_DON_PIN, INPUT_PULLUP);
  // ANALOG PİNMODE
  pinMode(ANALOG_PIN_BUTTON, INPUT);
  // MOTOR DİRECTİON PİNMODE
  pinMode(ADIR, OUTPUT);
  pinMode(BDIR, OUTPUT);
  pinMode(CDIR, OUTPUT);
  pinMode(DDIR, OUTPUT);
  pinMode(ADIRFW, OUTPUT);
  pinMode(BDIRFW, OUTPUT);
  pinMode(CDIRFW, OUTPUT);
  pinMode(DDIRFW, OUTPUT);
  // LINE FOLLOWING SENSOR PİNMODE
  pinMode(SENSOR_PIN_FRONT_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_7, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_7, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_7, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_7, INPUT_PULLUP);

  // LIDAR WARNING PİNMODE
  pinMode(WARN_LIDAR11, INPUT);
  pinMode(WARN_LIDAR12, INPUT);
  pinMode(WARN_LIDAR21, INPUT);
  pinMode(WARN_LIDAR22, INPUT);

  // MOTOR DİRECTİON SETUP
  digitalWrite(ADIR, HIGH);
  digitalWrite(ADIRFW, LOW);
  digitalWrite(BDIR, HIGH);
  digitalWrite(BDIRFW, LOW);
  digitalWrite(CDIR, HIGH);
  digitalWrite(CDIRFW, LOW);
  digitalWrite(DDIR, HIGH);
  digitalWrite(DDIRFW, LOW);

  // START ROBOT
  delay(1000);
  digitalWrite(STANBY_PIN, HIGH);

  lcd.clear();
}

void loop()
{

  rgbStatus(rgbState);

  buzzerFlipFlop(buzzerStateLoop);
  TIME_NOW4 = millis();
  if (TIME_NOW4 - adc_millis4 > 10)
  {

    Serial.print(agvDirection);

    Serial.print("  FRONT ");
    for (int j = 6; j >= 0; j--)
    {
      if (((lineFrontSensorValue() >> j) & 1) == 1)
      {
        Serial.print("1");
      }
      else
      {
        Serial.print("0");
      }
    }
    Serial.print("  BACK ");
    for (int j = 6; j >= 0; j--)
    {
      if (((lineBackSensorValue() >> j) & 1) == 1)
      {
        Serial.print("1");
      }
      else
      {
        Serial.print("0");
      }
    }
    Serial.print(" RIGHT ");
    for (int j = 6; j >= 0; j--)
    {
      if (((lineRightSensorValue() >> j) & 1) == 1)
      {
        Serial.print("1");
      }
      else
      {
        Serial.print("0");
      }
    }
    Serial.print(" LEFT ");
    for (int j = 6; j >= 0; j--)
    {
      if (((lineLeftSensorValue() >> j) & 1) == 1)
      {
        Serial.print("1");
      }
      else
      {
        Serial.print("0");
      }
    }

    Serial.println();

    // panel buttonarının durumlarını okuyup gösteriyoruz
    ILERI_TRY = digitalRead(ILERI_PIN);
    GERI_TRY = digitalRead(GERI_PIN);
    SAGA_TRY = digitalRead(SAGA_PIN);
    SOLA_TRY = digitalRead(SOLA_PIN);
    SAGA_DON = digitalRead(SAGA_DON_PIN);
    SOLA_DON = digitalRead(SOLA_DON_PIN);
    // istasyon butonunun durumlarını okuyup gösteriyoruz
    ANALOG_GIT_BUTTON = analogRead(ANALOG_PIN_BUTTON);
    // Sensor Pinlerini durumlarını okuyup gösteriyoruz
    /*     SensorFront1 = digitalRead(SENSOR_PIN_FRONT_1);
        SensorFront2 = digitalRead(SENSOR_PIN_FRONT_2);
        SensorFront3 = digitalRead(SENSOR_PIN_FRONT_3);
        SensorFront4 = digitalRead(SENSOR_PIN_FRONT_4);
        SensorFront5 = digitalRead(SENSOR_PIN_FRONT_5);
        SensorFront6 = digitalRead(SENSOR_PIN_FRONT_6);
        SensorFront7 = digitalRead(SENSOR_PIN_FRONT_7);
        SensorBack1 = digitalRead(SENSOR_PIN_BACK_1);
        SensorBack2 = digitalRead(SENSOR_PIN_BACK_2);
        SensorBack3 = digitalRead(SENSOR_PIN_BACK_3);
        SensorBack4 = digitalRead(SENSOR_PIN_BACK_4);
        SensorBack5 = digitalRead(SENSOR_PIN_BACK_5);
        SensorBack6 = digitalRead(SENSOR_PIN_BACK_6);
        SensorBack7 = digitalRead(SENSOR_PIN_BACK_7);
        SensorRight1 = digitalRead(SENSOR_PIN_RIGHT_1);
        SensorRight2 = digitalRead(SENSOR_PIN_RIGHT_2);
        SensorRight3 = digitalRead(SENSOR_PIN_RIGHT_3);
        SensorRight4 = digitalRead(SENSOR_PIN_RIGHT_4);
        SensorRight5 = digitalRead(SENSOR_PIN_RIGHT_5);
        SensorRight6 = digitalRead(SENSOR_PIN_RIGHT_6);
        SensorRight7 = digitalRead(SENSOR_PIN_RIGHT_7);
        SensorLeft1 = digitalRead(SENSOR_PIN_LEFT_1);
        SensorLeft2 = digitalRead(SENSOR_PIN_LEFT_2);
        SensorLeft3 = digitalRead(SENSOR_PIN_LEFT_3);
        SensorLeft4 = digitalRead(SENSOR_PIN_LEFT_4);
        SensorLeft5 = digitalRead(SENSOR_PIN_LEFT_5);
        SensorLeft6 = digitalRead(SENSOR_PIN_LEFT_6);
        SensorLeft7 = digitalRead(SENSOR_PIN_LEFT_7);
     */
    // Lidar Warning Pinlerini durumlarını okuyup gösteriyoruz
    Warn_Lidar11 = digitalRead(WARN_LIDAR11);
    Warn_Lidar12 = digitalRead(WARN_LIDAR12);
    Warn_Lidar21 = digitalRead(WARN_LIDAR21);
    Warn_Lidar22 = digitalRead(WARN_LIDAR22);
    // Serial.println(lineFrontSensorValue(),BIN);
    // Serial.println(ANALOG_GIT_BUTTON);
    adc_millis4 = TIME_NOW4;
    // lcdMessages();
  }
  /*
  if (WARN_LIDAR11 == HIGH || WARN_LIDAR12 == HIGH || WARN_LIDAR21 == HIGH || WARN_LIDAR22 == HIGH)
  {
    followStationState = 0;
    rgbState = 1;
  }
*/
  if (amparageRead() > 3)
  {
    Serial.print("AMPARAGE: ");
    Serial.println(amparageRead());
    chargeStatus = 1;
    rgbState = 9;
    buzzerStateLoop = 1;
  }
  else if (amparageRead() < 3)
  {
    chargeStatus = 0;
    rgbState = 8;
  }
  if (chargeStatus == 0)
  {




    if (ANALOG_GIT_BUTTON >= 660 && ANALOG_GIT_BUTTON <= 680) // ileri
    {

    }
    else if (ANALOG_GIT_BUTTON >= 310 && ANALOG_GIT_BUTTON <= 330) // geri
    {

    }
    else
    {

      if (agvDirection==5) {  // bekleme durumu stop
           agvDirection = 6;  // bekleme durumu ready
      }else if {
        
      }

    }





    if (sayacmsduraklama > 0)
    {
      sayacmsduraklama -= 1;
    }









    switch (agvDirection)
    {
    case 0: // stop
      Pwm(PWM_STOP);
      rgbController(false, false, false, false, false, false, false, true);
      buzzerStateLoop = 1;
      break;
    case 1: // ileri
      rgbController(false, true, false, false, false, false, false, false);
      buzzerStateLoop = 3;
      RotateWheels(true, false, false, false, false, false);

      if (lineFrontSensorValue() == 127 || lineFrontSensorValue() == 0 || lineBackSensorValue() == 127 || lineBackSensorValue() == 0) // ön ve arka sensor full veya hiçbir deger görmezse stop
      {
        agvDirection = 0;
      }
      else if ((lineFrontSensorValue() & 0b1110000) >= 16 && (lineFrontSensorValue() & 0b0000111) >= 1 && lineRightSensorValue() == 0 && lineLeftSensorValue() > 0)
      {
        // sola git
      }
      else if ((lineFrontSensorValue() & 0b1110000) >= 16 && (lineFrontSensorValue() & 0b0000111) >= 1 && lineRightSensorValue() > 0 && lineLeftSensorValue() == 0)
      {
        // saga git
      }
      else if (lineRightSensorValue() > 0 && lineLeftSensorValue() > 0 && sayacmsduraklama == 0) // hem sag hem sol sensorde deger okunursa
      {
        // ara durak detect
      }
      else if ((lineFrontSensorValue() & 0b1111000) >= 16 && (lineFrontSensorValue() & 0b1111000) <= 120) // ileri sollu git
      {
        PwmStraigtLeft(PWM_START);
      }
      else if ((lineFrontSensorValue() & 0b001111) <= 15 && (lineFrontSensorValue() & 0b001111) > 0) // ileri sagli git
      {
        PwmStraigtRight(PWM_START);
      }
      else
      {
        Pwm(PWM_START);
      }

      break;
    case 2: // geri
      RotateWheels(false, true, false, false, false, false);
      rgbController(false, false, false, true, false, false, false, false);
      buzzerStateLoop = 3;

      if (lineFrontSensorValue() == 127 || lineFrontSensorValue() == 0 || lineBackSensorValue() == 127 || lineBackSensorValue() == 0) // ön ve arka sensor full veya hiçbir deger görmezse stop
      {
        agvDirection = 0;
      }
      else if ((lineBackSensorValue() & 0b1110000) >= 16 && (lineBackSensorValue() & 0b0000111) >= 1 && lineRightSensorValue() == 0 && lineLeftSensorValue() > 0)
      {
        // sola git
      }
      else if ((lineBackSensorValue() & 0b1110000) >= 16 && (lineBackSensorValue() & 0b0000111) >= 1 && lineRightSensorValue() > 0 && lineLeftSensorValue() == 0)
      {
        // saga git
      }
      else if (lineRightSensorValue() > 0 && lineLeftSensorValue() > 0 && sayacmsduraklama == 0) // hem sag hem sol sensorde deger okunursa
      {
        // ara durak
      }
      else if ((lineBackSensorValue() & 0b1111000) >= 16 && (lineBackSensorValue() & 0b1111000) <= 120) // geri sagli git
      {
        PwmStraigtRight(PWM_START);
      }
      else if ((lineBackSensorValue() & 0b001111) <= 15 && (lineBackSensorValue() & 0b001111) > 0) // geri sollu git
      {
        PwmStraigtLeft(PWM_START);
      }
      else
      {
        Pwm(PWM_START);
      }
      break;
    case 3: // saga git
      RotateWheels(false, false, false, true, false, false);
      rgbController(false, false, false, true, false, false, false, false);
      buzzerStateLoop = 3;

      if (lineRightSensorValue() == 127 || lineRightSensorValue() == 0)
      {
        // sag  sensor full veya hiçbir deger görmezse stop
        agvDirection = 0;
      }
      else if ((lineRightSensorValue() & 0b1110000) >= 16 && (lineRightSensorValue() & 0b0000111) >= 1 && lineFrontSensorValue() > 0 && lineBackSensorValue() == 0)
      {
        // ileri hareket
         agvDirection = 1;
      }
      else if ((lineRightSensorValue() & 0b1110000) >= 16 && (lineRightSensorValue() & 0b0000111) >= 1 && lineFrontSensorValue() == 0 && lineBackSensorValue() > 0)
      {
        agvDirection = 2; // geri
      }
      else if ((lineRightSensorValue() & 0b1111000) >= 16 && (lineRightSensorValue() & 0b1111000) <= 120) // ileri sagli git
      {
        PwmLateralRight(PWM_START);
      }
      else if ((lineRightSensorValue() & 0b001111) <= 15 && (lineRightSensorValue() & 0b001111) > 0) // ileri sollu git
      {
        PwmLateralLeft(PWM_START);
      }
      else
      {
        Pwm(PWM_START);
      }
      break;
    case 4: // sola git
      RotateWheels(false, false, true, false, false, false);
      rgbController(false, false, false, false, true, false, false, false);
      buzzerStateLoop = 3;

      if (lineLeftSensorValue() == 127 || lineLeftSensorValue() == 0 || lineRightSensorValue() == 127 || lineRightSensorValue() == 0) // sag ve sol sensor full veya hiçbir deger görmezse stop
      {
        agvDirection = 0;
      }

      if ((lineLeftSensorValue() & 0b1110000) >= 16 && (lineLeftSensorValue() & 0b0000111) >= 1 && lineFrontSensorValue() == 0 && lineBackSensorValue() > 0)
      {
        agvDirection = 2; // geri
      }
      else if ((lineLeftSensorValue() & 0b1110000) >= 16 && (lineLeftSensorValue() & 0b0000111) >= 1 && lineFrontSensorValue() > 0 && lineBackSensorValue() == 0)
      {
        agvDirection = 1; // ileri
      }
      else if (lineFrontSensorValue() > 0 && lineBackSensorValue() > 0 && sayacmsduraklama == 0) // hem sag hem sol sensorde deger okunursa
      {
        agvDirection = 15; // ara durak detect
      }
      else if ((lineLeftSensorValue() & 0b1111000) >= 16 && (lineLeftSensorValue() & 0b1111000) <= 120) // ileri sagli git
      {
        PwmLateralRight(PWM_START);
      }
      else if ((lineLeftSensorValue() & 0b001111) <= 15 && (lineLeftSensorValue() & 0b001111) > 0) // ileri sollu git
      {
        PwmLateralLeft(PWM_START);
      }
      else
      {
        Pwm(PWM_START);
      }
      break;
    case 5: //  Bekleme stop
      rgbController(false, false, true, false, false, false, false, false);
      Pwm(PWM_STOP);
      buzzerStateLoop = 1;
      break;
    case 6: // Bekleme Ready
      rgbController(false, false, true, false, false, false, false, false);
      buzzerStateLoop = 1;
      Pwm(PWM_STOP);
      break;
    default:
      rgbController(false, false, false, false, false, false, false, true);
      buzzerStateLoop = 1;
      Pwm(PWM_STOP);
      break;
    }

    if (SAGA_DON == LOW)
    {
      RotateWheels(false, false, false, true, false, false);
      rgbState = 3;
      Pwm(PWM_START);
      buzzerStateLoop = 3;
      //  Serial.println("SAGA");
    }
    else if (GERI_TRY == LOW)
    {
      RotateWheels(true, false, false, false, false, false);
      rgbState = 4;
      Pwm(PWM_START);
      buzzerStateLoop = 3;

      // Serial.println("GERI");
    }
    else if (ILERI_TRY == LOW)
    {
      RotateWheels(false, true, false, false, false, false);
      rgbState = 5;
      Pwm(PWM_START);
      buzzerStateLoop = 3;

      // Serial.println("ILERI");
    }
    else if (SOLA_DON == LOW)
    {
      RotateWheels(false, false, true, false, false, false);
      rgbState = 1;
      Pwm(PWM_START);
      buzzerStateLoop = 3;

      // Serial.println("SOLA");
    }
    else if (SAGA_TRY == LOW)

    {
      RotateWheels(false, false, false, false, true, false);
      rgbState = 3;
      Pwm(PWM_START);
      buzzerStateLoop = 3;

      // Serial.println("SOLA_DON");
    }
    else if (SOLA_TRY == LOW)

    {
      RotateWheels(false, false, false, false, false, true);
      rgbState = 2;
      Pwm(PWM_START);
      buzzerStateLoop = 3;

      // Serial.println("SAGA_DON");
    }
    else
    {
      // RotateWheels(false, false, false, false, false, false);
      // rgbController(true, false, false, false,false,false,false,false);
      // PwmStop(PWM_STOP);
    }
  }
}
void PwmStraigtRight(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value / 2);
  analogWrite(DPWM, pwm_value / 2);
}
void PwmStraigtLeft(int pwm_value)
{
  analogWrite(APWM, pwm_value / 2);
  analogWrite(BPWM, pwm_value / 2);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value);
}
void PwmLateralRight(int pwm_value)
{
  analogWrite(APWM, pwm_value / 2);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value / 2);
  analogWrite(DPWM, pwm_value);
}
void PwmLateralLeft(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value / 2);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value / 2);
}

void Pwm(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value);
}

// 10 munite timer stanby mode
void timer_standby()
{
  if (TIME_NOW2 - oldTime2 > 600000)
  {
    digitalWrite(STANBY_PIN, HIGH);
  }
}

void rgbStatus(int rgbState)
{
  if (rgbState == 1)
  {
    rgbController(true, false, false, false, false, false, false, false);
  }
  else if (rgbState == 2)
  {
    rgbController(false, true, false, false, false, false, false, false);
  }
  else if (rgbState == 3)
  {
    rgbController(false, false, true, false, false, false, false, false);
  }
  else if (rgbState == 4)
  {
    rgbController(false, false, false, true, false, false, false, false);
  }
  else if (rgbState == 5)
  {
    rgbController(false, false, false, false, true, false, false, false);
  }
  else if (rgbState == 6)
  {
    rgbController(false, false, false, false, false, true, false, false);
  }
  else if (rgbState == 7)
  {
    rgbController(false, false, false, false, false, false, true, false);
  }
  else if (rgbState == 8)
  {
    rgbController(false, false, false, false, false, false, false, true);
  }
  else if (rgbState == 9)
  {
    chargerRgbStatus();
  }
}

void RotateWheels(bool ileri, bool geri, bool sag, bool sol, bool soldon, bool sagdon)
{
  if (ileri)
  {
    digitalWrite(ADIR, HIGH);
    digitalWrite(ADIRFW, LOW);
    digitalWrite(BDIR, LOW);
    digitalWrite(BDIRFW, HIGH);
    digitalWrite(CDIR, LOW);
    digitalWrite(CDIRFW, HIGH);
    digitalWrite(DDIR, HIGH);
    digitalWrite(DDIRFW, LOW);
  }
  else if (geri)
  {
    digitalWrite(ADIR, LOW);
    digitalWrite(ADIRFW, HIGH);
    digitalWrite(BDIR, HIGH);
    digitalWrite(BDIRFW, LOW);
    digitalWrite(CDIR, HIGH);
    digitalWrite(CDIRFW, LOW);
    digitalWrite(DDIR, LOW);
    digitalWrite(DDIRFW, HIGH);
  }
  else if (sag)
  {
    digitalWrite(ADIR, LOW);
    digitalWrite(ADIRFW, HIGH);
    digitalWrite(BDIR, LOW);
    digitalWrite(BDIRFW, HIGH);
    digitalWrite(CDIR, HIGH);
    digitalWrite(CDIRFW, LOW);
    digitalWrite(DDIR, HIGH);
    digitalWrite(DDIRFW, LOW);
  }
  else if (sol)
  {
    digitalWrite(ADIR, HIGH);
    digitalWrite(ADIRFW, LOW);
    digitalWrite(BDIR, HIGH);
    digitalWrite(BDIRFW, LOW);
    digitalWrite(CDIR, LOW);
    digitalWrite(CDIRFW, HIGH);
    digitalWrite(DDIR, LOW);
    digitalWrite(DDIRFW, HIGH);
  }
  else if (soldon)
  {
    digitalWrite(ADIR, HIGH);
    digitalWrite(ADIRFW, LOW);
    digitalWrite(BDIR, LOW);
    digitalWrite(BDIRFW, HIGH);
    digitalWrite(CDIR, HIGH);
    digitalWrite(CDIRFW, LOW);
    digitalWrite(DDIR, LOW);
    digitalWrite(DDIRFW, HIGH);
  }
  else if (sagdon)
  {
    digitalWrite(ADIR, LOW);
    digitalWrite(ADIRFW, HIGH);
    digitalWrite(BDIR, HIGH);
    digitalWrite(BDIRFW, LOW);
    digitalWrite(CDIR, LOW);
    digitalWrite(CDIRFW, HIGH);
    digitalWrite(DDIR, HIGH);
    digitalWrite(DDIRFW, LOW);
  }
  else
  {
  }
}

void lcdMessages()
{
  // en son yapılacak
  lcd.setCursor(0, 0);
  lcd.print("      OMNIMAN       ");
  lcd.setCursor(0, 1);
  lcd.print("RGB MOD : ON");
  lcd.setCursor(0, 2);
  lcd.print("AKU:");
  lcd.setCursor(5, 2);
  lcd.setCursor(7, 2);
  lcd.print("V");
  lcd.setCursor(11, 2);
  lcd.print("%");
  lcd.setCursor(13, 2);
  lcd.setCursor(15, 2);
  lcd.print(vBattRead());
  lcd.setCursor(0, 3);
  lcd.print(amparageRead());
  lcd.setCursor(6, 3);
  lcd.print("HIZ: ");
  lcd.setCursor(9, 3);
  lcd.print("metre/dk");
}

float vBattRead()
{
  int adc_value = 0;
  for (int i = 0; i < BATTERY_AVARAGE_READ_COUNT; i++)
  {
    adc_value += analogRead(VBATTADC);
  }
  adc_value = adc_value / BATTERY_AVARAGE_READ_COUNT;
  float Vbatt = (adc_value * 0.11809);
  return Vbatt;
}

float amparageRead()
{
  int adc_value = 0;
  for (int i = 0; i < BATTERY_AVARAGE_READ_COUNT; i++)
  {
    adc_value += analogRead(AMPERAGE_PIN);
  }
  adc_value = adc_value / BATTERY_AVARAGE_READ_COUNT;
  float Vbatt = (adc_value * 0.01782);
  return Vbatt;
}
void chargerRgbStatus()
{
  newTime6 = millis();
  static int i = 0;
  static int artirFlag = 0;
  int amp = 30;
  if (amparageRead() >= 20)
  {
    amp = 4;
  }
  else if (amparageRead() >= 15 && amparageRead() < 20)
  {
    amp = 10;
  }
  else if (amparageRead() >= 10 && amparageRead() < 15)
  {
    amp = 25;
  }
  else if (amparageRead() >= 0 && amparageRead() < 10)
  {
    amp = 50;
  }

  if (newTime6 - oldTime6 > amp)
  {

    if (i == 0)
    {
      artirFlag = 1;
    }

    if (i == 20)
    {
      artirFlag = 0;
    }

    if (artirFlag == 1)
    {
      i++;
    }
    else
    {
      i--;
    }
    analogWrite(RED_PIN, i);
    analogWrite(GREEN_PIN, i);
    analogWrite(BLUE_PIN, i);
    oldTime6 = newTime6;
  }
}
void rgbController(bool white, bool red, bool green, bool blue, bool purple, bool cyan, bool yellow, bool stop)
{
  newTime1 = millis();
  static int i = 0;
  static int artirFlag = 0;
  if (newTime1 - oldTime1 > 15)
  {

    if (i == 0)
    {
      artirFlag = 1;
    }

    if (i == 30)
    {
      artirFlag = 0;
    }

    if (artirFlag == 1)
    {
      i++;
    }
    else
    {
      i--;
    }
    if (white)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, i);
    }
    else if (red)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, 0);
    }
    else if (green)
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, 0);
    }
    else if (blue)
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, i);
    }
    else if (purple)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, i);
    }
    else if (cyan)
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, i);
    }
    else if (yellow)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, 0);
    }
    else
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, 0);
    }
    oldTime1 = newTime1;
  }
}

void buzzerFlipFlop(int buzzerStatus) // 0 = off 2 = on 3 = flipflop
{
  newTime3 = millis();
  static int i = 0;
  static int artirFlag = 0;

  if (newTime3 - oldTime3 > 20)
  {
    if (buzzerStatus == 1)
    {
      analogWrite(BUZZER_PIN, 0);
    }
    else if (buzzerStatus == 2)
    {
      analogWrite(BUZZER_PIN, 10);
    }
    else if (buzzerStatus == 3)
    {
      if (i == 0)
      {
        artirFlag = 1;
      }

      if (i == 15)
      {
        artirFlag = 0;
      }

      if (artirFlag == 1)
      {
        i++;
      }
      else
      {
        i--;
      }

      analogWrite(BUZZER_PIN, i);
    }

    oldTime3 = newTime3;
  }
}

uint16_t lineFrontSensorValue()
{

  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < 100)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < 100)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= 100)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= 100)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < 100)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < 100)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= 100)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= 100)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < 100)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < 100)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= 100)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= 100)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < 100)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < 100)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= 100)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= 100)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < 100)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < 100)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= 100)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= 100)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < 100)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < 100)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= 100)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= 100)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < 100)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < 100)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= 100)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= 100)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    lineSensor.sensor1 = digitalRead(SENSOR_PIN1);
    lineSensor.sensor2 = digitalRead(SENSOR_PIN2);
    lineSensor.sensor3 = digitalRead(SENSOR_PIN3);
    lineSensor.sensor4 = digitalRead(SENSOR_PIN4);
    lineSensor.sensor5 = digitalRead(SENSOR_PIN5);
    lineSensor.sensor6 = digitalRead(SENSOR_PIN6);
    lineSensor.sensor7 = digitalRead(SENSOR_PIN7);
  */

  return lineSensor.sensorAll;
}

uint16_t lineBackSensorValue()
{

  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < 100)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < 100)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= 100)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= 100)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < 100)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < 100)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= 100)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= 100)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < 100)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < 100)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= 100)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= 100)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < 100)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < 100)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= 100)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= 100)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < 100)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < 100)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= 100)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= 100)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < 100)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < 100)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= 100)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= 100)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < 100)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < 100)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= 100)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= 100)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    lineSensor.sensor1 = digitalRead(SENSOR_PIN1);
    lineSensor.sensor2 = digitalRead(SENSOR_PIN2);
    lineSensor.sensor3 = digitalRead(SENSOR_PIN3);
    lineSensor.sensor4 = digitalRead(SENSOR_PIN4);
    lineSensor.sensor5 = digitalRead(SENSOR_PIN5);
    lineSensor.sensor6 = digitalRead(SENSOR_PIN6);
    lineSensor.sensor7 = digitalRead(SENSOR_PIN7);
  */

  return lineSensor.sensorAll;
}

uint16_t lineRightSensorValue()
{

  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < 100)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < 100)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= 100)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= 100)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < 100)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < 100)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= 100)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= 100)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < 100)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < 100)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= 100)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= 100)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < 100)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < 100)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= 100)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= 100)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < 100)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < 100)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= 100)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= 100)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < 100)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < 100)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= 100)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= 100)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < 100)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < 100)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= 100)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= 100)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    lineSensor.sensor1 = digitalRead(SENSOR_PIN1);
    lineSensor.sensor2 = digitalRead(SENSOR_PIN2);
    lineSensor.sensor3 = digitalRead(SENSOR_PIN3);
    lineSensor.sensor4 = digitalRead(SENSOR_PIN4);
    lineSensor.sensor5 = digitalRead(SENSOR_PIN5);
    lineSensor.sensor6 = digitalRead(SENSOR_PIN6);
    lineSensor.sensor7 = digitalRead(SENSOR_PIN7);
  */

  return lineSensor.sensorAll;
}

uint16_t lineLeftSensorValue()
{

  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < 100)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < 100)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= 100)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= 100)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < 100)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < 100)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= 100)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= 100)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < 100)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < 100)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= 100)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= 100)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < 100)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < 100)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= 100)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= 100)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < 100)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < 100)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= 100)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= 100)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < 100)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < 100)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= 100)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= 100)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < 100)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < 100)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= 100)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= 100)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    lineSensor.sensor1 = digitalRead(SENSOR_PIN1);
    lineSensor.sensor2 = digitalRead(SENSOR_PIN2);
    lineSensor.sensor3 = digitalRead(SENSOR_PIN3);
    lineSensor.sensor4 = digitalRead(SENSOR_PIN4);
    lineSensor.sensor5 = digitalRead(SENSOR_PIN5);
    lineSensor.sensor6 = digitalRead(SENSOR_PIN6);
    lineSensor.sensor7 = digitalRead(SENSOR_PIN7);
  */

  return lineSensor.sensorAll;
}