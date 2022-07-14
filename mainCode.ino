#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//A MOTOR
#define APWM 12
#define ADIR 10
#define ADIRFW 8
//B MOTOR
#define BPWM 46
#define BDIR 48
#define BDIRFW 30
//C MOTOR
#define CPWM 45
#define CDIR 25
#define CDIRFW 23
//D MOTOR
#define DPWM 13
#define DDIR 11
#define DDIRFW 9
//LINE FOLLOWING SENSOR
#define SENSOR_PIN1 A1
#define SENSOR_PIN2 A2
#define SENSOR_PIN3 A3
#define SENSOR_PIN4 A4
#define SENSOR_PIN5 A5
#define SENSOR_PIN6 A6
#define SENSOR_PIN7 A7
#define SENSOR_PIN8 A8
#define SENSOR_PIN9 A9
#define SENSOR_PIN10 A10
#define SENSOR_PIN11 A11
#define SENSOR_PIN12 A12
#define SENSOR_PIN13 A13
#define SENSOR_PIN14 32
#define SENSOR_PIN15 34
//LINE FOLLOWING SENSOR ENABLE PIN
#define FRONT_SENSOR_EN 36
#define BACK_SENSOR_EN 40
#define RIGT_SENSOR_EN 38
#define LEFT_SENSOR_EN 42
//STANBY 
#define STANBY_PIN 6
//CONTROL PANEL BUTTONS
#define ILERI_PIN 43
#define GERI_PIN 41
#define SAGA_PIN 7
#define SOLA_PIN 44
#define SOLA_DON_PIN 29
#define SAGA_DON_PIN 31
//STATİONS BUTTONS
#define ANALOG_PIN_BUTTON A0
//BATTERY
#define VBATTADC 14
#define BATTERY_AVARAGE_READ_COUNT 10
//AMPERAGE PIN
#define AMPERAGE_PIN A15
//RGB 
#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4
//BUZZER
#define BUZZER_PIN 5
//PWM VALUE
#define PWM_START 255
#define PWM_STOP 0
//LİDAR WARNING PIOUT
#define WARN_LIDAR11 39
#define WARN_LIDAR12 37
#define WARN_LIDAR21 35
#define WARN_LIDAR22 33
//MILLIS
unsigned long oldTime=0;
unsigned long newTime;
unsigned long oldTime1=0;
unsigned long newTime1;
unsigned long oldTime2=0;
unsigned long newTime2;
unsigned long oldTime3=0;
unsigned long newTime3;
unsigned long oldTime4=0;
unsigned long newTime4;
int buzzerCount = 0;
int sensorCount = 0;


long TIME_NOW = 0;long TIME_NOW1 = 0;long TIME_NOW2 = 0;long TIME_NOW3 = 0; long TIME_NOW4 = 0;
unsigned long adc_millis = 0;unsigned long adc_millis1 = 0; unsigned long adc_millis2 = 0; unsigned long adc_millis3 = 0; unsigned long adc_millis4 = 0;



LiquidCrystal_I2C lcd(0x27, 20, 4);


int ileriAnalogValue = 686;
int geriAnalogValue = 333;



  //panel buttonarının durumlarını okuyup gösteriyoruz
  int ILERI_TRY = -1;
  int GERI_TRY = -1;
  int SAGA_TRY = -1;
  int SOLA_TRY = -1;
  int SAGA_DON = -1;
  int SOLA_DON = -1;
  //istasyon butonunun durumlarını okuyup gösteriyoruz
  int ANALOG_GIT_BUTTON = -1;
  //Sensor Pinlerini durumlarını okuyup gösteriyoruz
  int Sensor1 = -1;
  int Sensor2 = -1;
  int Sensor3 = -1;
  int Sensor4 = -1;
  int Sensor5 = -1;
  int Sensor6 = -1;
  int Sensor7 = -1;
  int Sensor8 = -1;
  int Sensor9 = -1;
  int Sensor10 = -1;
  int Sensor11 = -1;
  int Sensor12 = -1;
  int Sensor13 = -1;
  int Sensor14 = -1;
  int Sensor15 = -1;


void setup()
{

  Wire.begin();
  Serial.begin(115200);

//MOTOR PİNMODE
  pinMode(APWM, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(CPWM, OUTPUT);
  pinMode(DPWM, OUTPUT);
  
  analogWrite(APWM,0);
   analogWrite(BPWM,0);
    analogWrite(CPWM,0);
     analogWrite(DPWM,0);

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("      OMNIMAN       ");

  delay(1000);
//RGB PİNMODE
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

//VOLTAGE SENSOR PİNMODE
  pinMode(VBATTADC, INPUT);
//AMPERAGE PİNMODE
  pinMode(AMPERAGE_PIN, INPUT);
//CONTROL BUTTON PİNMODE
  pinMode(ILERI_PIN, INPUT);
  pinMode(GERI_PIN, INPUT);
  pinMode(SAGA_PIN, INPUT);
  pinMode(SOLA_PIN, INPUT);
  pinMode(SOLA_DON_PIN, INPUT);
  pinMode(SAGA_DON_PIN, INPUT);
//ANALOG PİNMODE
  pinMode(ANALOG_PIN_BUTTON, INPUT);
//MOTOR DİRECTİON PİNMODE
  pinMode(ADIR, OUTPUT);
  pinMode(BDIR, OUTPUT);
  pinMode(CDIR, OUTPUT);
  pinMode(DDIR, OUTPUT);
  pinMode(ADIRFW, OUTPUT);
  pinMode(BDIRFW, OUTPUT);
  pinMode(CDIRFW, OUTPUT);
  pinMode(DDIRFW, OUTPUT);
//LINE FOLLOWING SENSOR PİNMODE
  pinMode(SENSOR_PIN1, INPUT);
  pinMode(SENSOR_PIN2, INPUT);
  pinMode(SENSOR_PIN3, INPUT);
  pinMode(SENSOR_PIN4, INPUT);
  pinMode(SENSOR_PIN5, INPUT);
  pinMode(SENSOR_PIN6, INPUT);
  pinMode(SENSOR_PIN7, INPUT);
  pinMode(SENSOR_PIN8, INPUT);
  pinMode(SENSOR_PIN9, INPUT);
  pinMode(SENSOR_PIN10, INPUT);
  pinMode(SENSOR_PIN11, INPUT);
  pinMode(SENSOR_PIN12, INPUT);
  pinMode(SENSOR_PIN13, INPUT);
  pinMode(SENSOR_PIN14, INPUT);
  pinMode(SENSOR_PIN15, INPUT);
//LINE FOLLOWING SENSOR ENABLE PİNMODE
  pinMode(FRONT_SENSOR_EN, OUTPUT);
  pinMode(BACK_SENSOR_EN, OUTPUT);
  pinMode(RIGT_SENSOR_EN, OUTPUT);
  pinMode(LEFT_SENSOR_EN, OUTPUT);
  //LIDAR WARNING PİNMODE
  pinMode(WARN_LIDAR11, INPUT);
  pinMode(WARN_LIDAR12, INPUT);
  pinMode(WARN_LIDAR21, INPUT);
  pinMode(WARN_LIDAR22, INPUT);
  //LINE FOLLOWING SENSOR ENABLE
  digitalWrite(FRONT_SENSOR_EN, HIGH);
  digitalWrite(BACK_SENSOR_EN, HIGH);
  digitalWrite(RIGT_SENSOR_EN, HIGH);
  digitalWrite(LEFT_SENSOR_EN, HIGH);

//MOTOR DİRECTİON SETUP
  digitalWrite(ADIR, HIGH);
  digitalWrite(ADIRFW, LOW);
  digitalWrite(BDIR, HIGH);
  digitalWrite(BDIRFW, LOW);
  digitalWrite(CDIR, HIGH);
  digitalWrite(CDIRFW, LOW);
  digitalWrite(DDIR, HIGH);
  digitalWrite(DDIRFW, LOW);

//START ROBOT
  delay(1000);
  digitalWrite(STANBY_PIN, HIGH);

  lcd.clear();

}



void loop()
{
  /*
  RotateWheels(bool ileri, bool geri, bool sag, bool sol, bool soldon, bool sagdon)
  rgbController(bool white, bool red, bool green, bool blue, bool purple,bool cyan,bool yellow,bool stop){
  LineFlowingEnable(bool front,bool back,bool right,bool left)
  PwmStart(int pwm_value)
  PwmStop(int pwm_value)

  */




  TIME_NOW4 = millis();
  if (TIME_NOW4 - adc_millis4 > 10)
  {
//panel buttonarının durumlarını okuyup gösteriyoruz
   ILERI_TRY = digitalRead(ILERI_PIN);
   GERI_TRY = digitalRead(GERI_PIN);
   SAGA_TRY = digitalRead(SAGA_PIN);
   SOLA_TRY = digitalRead(SOLA_PIN);
   SAGA_DON = digitalRead(SAGA_DON_PIN);
   SOLA_DON = digitalRead(SOLA_DON_PIN);
//istasyon butonunun durumlarını okuyup gösteriyoruz
   ANALOG_GIT_BUTTON = analogRead(ANALOG_PIN_BUTTON);
//Sensor Pinlerini durumlarını okuyup gösteriyoruz
   Sensor1 = digitalRead(SENSOR_PIN1);
   Sensor2 = digitalRead(SENSOR_PIN2);
   Sensor3 = digitalRead(SENSOR_PIN3);
   Sensor4 = digitalRead(SENSOR_PIN4);
   Sensor5 = digitalRead(SENSOR_PIN5);
   Sensor6 = digitalRead(SENSOR_PIN6);
   Sensor7 = digitalRead(SENSOR_PIN7);
   Sensor8 = digitalRead(SENSOR_PIN8);
   Sensor9 = digitalRead(SENSOR_PIN9);
   Sensor10 = digitalRead(SENSOR_PIN10);
   Sensor11 = digitalRead(SENSOR_PIN11);
   Sensor12 = digitalRead(SENSOR_PIN12);
   Sensor13 = digitalRead(SENSOR_PIN13);
   Sensor14 = digitalRead(SENSOR_PIN14);
   Sensor15 = digitalRead(SENSOR_PIN15);
//Lidar Warning Pinlerini durumlarını okuyup gösteriyoruz
    Warn_Lidar11 = digitalRead(WARN_LIDAR11);
    Warn_Lidar12 = digitalRead(WARN_LIDAR12);
    Warn_Lidar21 = digitalRead(WARN_LIDAR21);
    Warn_Lidar22 = digitalRead(WARN_LIDAR22);
// Serial.println(lineSensorValue(),BIN);
//Serial.println(ANALOG_GIT_BUTTON);
    adc_millis4 = TIME_NOW4;
      sensorEnabled();
// lcdMessages();
  }

    newTime = millis(); 

if(newTime-oldTime > 1) {
    if(ANALOG_GIT_BUTTON  >= 675 && ANALOG_GIT_BUTTON  <= 690)
    {
    //Serial.println(ANALOG_GIT_BUTTON);
    //RotateWheels(false, true, false, false, false, false);
    //PwmStart(PWM_START);
    //1000000010000000  orta          
    //Serial.println(lineSensorValue(),BIN);
 
        if (lineSensorValue()==65535 || lineSensorValue()==32768) {
          PwmStop(PWM_STOP);

        }else {
          
      if (lineSensorValue()==0b1000000010000000) {
        if(WARN_LIDAR11 == HIGH || WARN_LIDAR12 == HIGH || WARN_LIDAR21 == HIGH || WARN_LIDAR22 == HIGH) {
          Pwm(PWM_STOP);
          rgbController(false, true, false, false,false,false,false,false);

       }else {
          RotateWheels(false, true, false, false, false, false);
          Pwm(PWM_START);
       }


        }

          if ((lineSensorValue() & 0b0111111110000000)>=256 && (lineSensorValue() & 0b0111111110000000)<=32640) {  // 2
       //     Serial.println("sol");
       if(WARN_LIDAR11 == HIGH || WARN_LIDAR12 == HIGH || WARN_LIDAR21 == HIGH || WARN_LIDAR22 == HIGH) {
          Pwm(PWM_STOP);
          rgbController(false, true, false, false,false,false,false,false);

       }else {
          RotateWheels(false, true, false, false, false, false);
          PwmStraigtRight(PWM_START);
       }

       
        }else if ((lineSensorValue() & 0b0000000001111111)<=127 && (lineSensorValue() & 0b0000000001111111)>0 ) {
         //     Serial.println("sag");
        if(WARN_LIDAR11 == HIGH || WARN_LIDAR12 == HIGH || WARN_LIDAR21 == HIGH || WARN_LIDAR22 == HIGH) {
          Pwm(PWM_STOP);
          rgbController(false, true, false, false,false,false,false,false);
       }else {
          RotateWheels(false, true, false, false, false, false);
          PwmStraigtLeft(PWM_START);
       }

        }
        }

        

        
    }
    else if(ANALOG_GIT_BUTTON == geriAnalogValue)
    {

    }
    else
    {
      Pwm(PWM_STOP);
    }
        oldTime = newTime;

}


 if (SAGA_DON == LOW)

  {
    RotateWheels(false, false, false, true, false, false);
    rgbController(true, false, false, false,false,false,false,false);
    Pwm(PWM_START);
    buzzerFlipFlop();
    // Serial.println("SAGA");
  }
  else if (GERI_TRY == LOW )
  {
    RotateWheels(true, false, false, false, false, false);
    rgbController(false, true, false, false,false,false,false,false);
    Pwm(PWM_START);
    buzzerFlipFlop();

    // Serial.println("GERI");
  }
  else if (ILERI_TRY == LOW )
  {
    RotateWheels(false, true, false, false, false, false);
    rgbController(false, false, true, false,false,false,false,false);
    Pwm(PWM_START);
    buzzerFlipFlop();

    //Serial.println("ILERI");
  }
  else if (SOLA_DON == LOW)
  {
    RotateWheels(false, false, true, false, false, false);
    rgbController(false, false, true, false,false,false,false,false);
    Pwm(PWM_START);
    buzzerFlipFlop();

    // Serial.println("SOLA");
  }
   else if (SAGA_TRY == LOW )

  {
   RotateWheels(false, false, false, false, true, false);
   rgbController(false, false, false, true,false,false,false,false);
   Pwm(PWM_START);
   buzzerFlipFlop();

    // Serial.println("SOLA_DON");
  }
       else if (SOLA_TRY == LOW)


  {
   rgbController(false, true, false, false,false,false,false,false);
   RotateWheels(false, false, false, false, false, true);
   Pwm(PWM_START);
   buzzerFlipFlop();

    // Serial.println("SAGA_DON");
   }
   else
   {
  //RotateWheels(false, false, false, false, false, false);
  //rgbController(true, false, false, false,false,false,false,false);
  //PwmStop(PWM_STOP);
   }

}





























void PwmStraigtRight(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value/2);
  analogWrite(DPWM, pwm_value/2);
}
void PwmStraigtLeft(int pwm_value)
{
  analogWrite(APWM, pwm_value/2);
  analogWrite(BPWM, pwm_value/2);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value);
}

void Pwm(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value);
}


//10 munite timer stanby mode 
void timer_standby()
{
  if (TIME_NOW2 - oldTime2 > 600000)
  {
    /*   */
  }
}
void LineFlowingEnable(bool front,bool back,bool right,bool left)
{
if(front){
  digitalWrite(FRONT_SENSOR_EN, HIGH);
  digitalWrite(BACK_SENSOR_EN, LOW);
  digitalWrite(RIGT_SENSOR_EN, LOW);
  digitalWrite(LEFT_SENSOR_EN, LOW);
}else if(back){
  digitalWrite(FRONT_SENSOR_EN, LOW);
  digitalWrite(BACK_SENSOR_EN, HIGH);
  digitalWrite(RIGT_SENSOR_EN, LOW);
  digitalWrite(LEFT_SENSOR_EN, LOW);
}else if(right){
  digitalWrite(FRONT_SENSOR_EN, LOW);
  digitalWrite(BACK_SENSOR_EN, LOW);
  digitalWrite(RIGT_SENSOR_EN, HIGH);
  digitalWrite(LEFT_SENSOR_EN, LOW);
}else if(left){
  digitalWrite(FRONT_SENSOR_EN, LOW);
  digitalWrite(BACK_SENSOR_EN, LOW);
  digitalWrite(RIGT_SENSOR_EN, LOW);
  digitalWrite(LEFT_SENSOR_EN, HIGH);
}else{
  digitalWrite(FRONT_SENSOR_EN, LOW);
  digitalWrite(BACK_SENSOR_EN, LOW);
  digitalWrite(RIGT_SENSOR_EN, LOW);
  digitalWrite(LEFT_SENSOR_EN, LOW);
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
  //en son yapılacak 
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
    float Vbatt = (adc_value * 5) / 1024;
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
    float Vbatt = (adc_value * 5) / 1024;
    return Vbatt;
}


void rgbController(bool white, bool red, bool green, bool blue, bool purple,bool cyan,bool yellow,bool stop){
    newTime1 = millis(); 
    static int i=0;
    static int artirFlag = 0;
if(newTime1-oldTime1 > 15) {

  if (i==0) {
    artirFlag=1;
  }

  if (i==30) {
    artirFlag=0;
  }

  if (artirFlag==1) {
    i++;
  }
  else {
    i--;
  }
  if(white){
    analogWrite(RED_PIN, i);
    analogWrite(GREEN_PIN, i);
    analogWrite(BLUE_PIN, i);
  }else if(red){
    analogWrite(RED_PIN, i);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
  }else if(green){
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, i);
    analogWrite(BLUE_PIN, 0);
  }else if(blue){
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, i);
  }else if(purple){
    analogWrite(RED_PIN, i);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, i);
  }else if(cyan){
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, i);
    analogWrite(BLUE_PIN, i);
  }else if(yellow){
    analogWrite(RED_PIN, i);
    analogWrite(GREEN_PIN, i);
    analogWrite(BLUE_PIN, 0);
  }else{
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
  }
    oldTime1 = newTime1;
}}


void buzzerFlipFlop(){
    newTime3 = millis();
if(newTime3-oldTime3 > 1000) {
if(buzzerCount == 0){
digitalWrite(BUZZER_PIN, HIGH);
buzzerCount = 1;
}
else if(buzzerCount == 1){
digitalWrite(BUZZER_PIN, LOW);
buzzerCount = 0;
}
oldTime3 = newTime3;
}
}

uint16_t lineSensorValue(){

struct{
  union {
    struct {
      uint16_t sensorAll:16;
    };
    struct {
      uint16_t sensor1:1;
      uint16_t sensor2:1;
      uint16_t sensor3:1;
      uint16_t sensor4:1;
      uint16_t sensor5:1;
      uint16_t sensor6:1;
      uint16_t sensor7:1;
      uint16_t sensor8:1;
      uint16_t sensor9:1;
      uint16_t sensor10:1;
      uint16_t sensor11:1;
      uint16_t sensor12:1;
      uint16_t sensor13:1;
      uint16_t sensor14:1;
      uint16_t sensor15:1;
      uint16_t sensor16:1;
    };
  };
} lineSensor;

lineSensor.sensorAll=0;

static int sayac1_on=0;
static int sayac1_off=0;

if (digitalRead(SENSOR_PIN1)==0) {
  sayac1_off=0;
  if (sayac1_on<10) {
    sayac1_on++;
  }
}else {
  sayac1_on=0;
    if (sayac1_off<10) {
    sayac1_off++;
  }
}

if (sayac1_on>=10) {
  lineSensor.sensor1=0;
}

if (sayac1_off>=10) {
  lineSensor.sensor1=1;
}

static int sayac2_on=0;
static int sayac2_off=0;

if (digitalRead(SENSOR_PIN2)==0) {
  sayac2_off=0;
  if (sayac2_on<10) {
    sayac2_on++;
  }
}else {
  sayac2_on=0;
    if (sayac2_off<10) {
    sayac2_off++;
  }
}

if (sayac2_on>=10) {
  lineSensor.sensor2=0;
}

if (sayac2_off>=10) {
  lineSensor.sensor2=1;
}

static int sayac3_on=0;
static int sayac3_off=0;

if (digitalRead(SENSOR_PIN3)==0) {
  sayac3_off=0;
  if (sayac3_on<10) {
    sayac3_on++;
  }
}else {
  sayac3_on=0;
    if (sayac3_off<10) {
    sayac3_off++;
  }
}

if (sayac3_on>=10) {
  lineSensor.sensor3=0;
}

if (sayac3_off>=10) {
  lineSensor.sensor3=1;
}

static int sayac4_on=0;
static int sayac4_off=0;

if (digitalRead(SENSOR_PIN4)==0) {
  sayac4_off=0;
  if (sayac4_on<10) {
    sayac4_on++;
  }
}else {
  sayac4_on=0;
    if (sayac4_off<10) {
    sayac4_off++;
  }
}

if (sayac4_on>=10) {
  lineSensor.sensor4=0;
}

if (sayac4_off>=10) {
  lineSensor.sensor4=1;
}

static int sayac5_on=0;
static int sayac5_off=0;

if (digitalRead(SENSOR_PIN5)==0) {
  sayac5_off=0;
  if (sayac5_on<10) {
    sayac5_on++;
  }
}else {
  sayac5_on=0;
    if (sayac5_off<10) {
    sayac5_off++;
  }
}

if (sayac5_on>=10) {
  lineSensor.sensor5=0;
}

if (sayac5_off>=10) {
  lineSensor.sensor5=1;
}

static int sayac6_on=0;
static int sayac6_off=0;

if (digitalRead(SENSOR_PIN6)==0) {
  sayac6_off=0;
  if (sayac6_on<10) {
    sayac6_on++;
  }
}else {
  sayac6_on=0;
    if (sayac6_off<10) {
    sayac6_off++;
  }
}

if (sayac6_on>=10) {
  lineSensor.sensor6=0;
}

if (sayac6_off>=10) {
  lineSensor.sensor6=1;
}

static int sayac7_on=0;
static int sayac7_off=0;

if (digitalRead(SENSOR_PIN7)==0) {
  sayac7_off=0;
  if (sayac7_on<10) {
    sayac7_on++;
  }
}else {
  sayac7_on=0;
    if (sayac7_off<10) {
    sayac7_off++;
  }
}

if (sayac7_on>=10) {
  lineSensor.sensor7=0;
}

if (sayac7_off>=10) {
  lineSensor.sensor7=1;
}

static int sayac8_on=0;
static int sayac8_off=0;

if (digitalRead(SENSOR_PIN8)==0) {
  sayac8_off=0;
  if (sayac8_on<10) {
    sayac8_on++;
  }
}else {
  sayac8_on=0;
    if (sayac8_off<10) {
    sayac8_off++;
  }
}

if (sayac8_on>=10) {
  lineSensor.sensor8=0;
}

if (sayac8_off>=10) {
  lineSensor.sensor8=1;
}

static int sayac9_on=0;
static int sayac9_off=0;

if (digitalRead(SENSOR_PIN9)==0) {
  sayac9_off=0;
  if (sayac9_on<10) {
    sayac9_on++;
  }
}else {
  sayac9_on=0;
    if (sayac9_off<10) {
    sayac9_off++;
  }
}

if (sayac9_on>=10) {
  lineSensor.sensor9=0;
}

if (sayac9_off>=10) {
  lineSensor.sensor9=1;
}

static int sayac10_on=0;
static int sayac10_off=0;

if (digitalRead(SENSOR_PIN10)==0) {
  sayac10_off=0;
  if (sayac10_on<10) {
    sayac10_on++;
  }
}else {
  sayac10_on=0;
    if (sayac10_off<10) {
    sayac10_off++;
  }
}

if (sayac10_on>=10) {
  lineSensor.sensor10=0;
}

if (sayac10_off>=10) {
  lineSensor.sensor10=1;
}

static int sayac11_on=0;
static int sayac11_off=0;

if (digitalRead(SENSOR_PIN11)==0) {
  sayac11_off=0;
  if (sayac11_on<10) {
    sayac11_on++;
  }
}else {
  sayac11_on=0;
    if (sayac11_off<10) {
    sayac11_off++;
  }
}

if (sayac11_on>=10) {
  lineSensor.sensor11=0;
}

if (sayac11_off>=10) {
  lineSensor.sensor11=1;
}

static int sayac12_on=0;
static int sayac12_off=0;

if (digitalRead(SENSOR_PIN12)==0) {
  sayac12_off=0;
  if (sayac12_on<10) {
    sayac12_on++;
  }
}else {
  sayac12_on=0;
    if (sayac12_off<10) {
    sayac12_off++;
  }
}

if (sayac12_on>=10) {
  lineSensor.sensor12=0;
}

if (sayac12_off>=10) {
  lineSensor.sensor12=1;
}

static int sayac13_on=0;
static int sayac13_off=0;

if (digitalRead(SENSOR_PIN13)==0) {
  sayac13_off=0;
  if (sayac13_on<10) {
    sayac13_on++;
  }
}else {
  sayac13_on=0;
    if (sayac13_off<10) {
    sayac13_off++;
  }
}

if (sayac13_on>=10) {
  lineSensor.sensor13=0;
}

if (sayac13_off>=10) {
  lineSensor.sensor13=1;
}

static int sayac14_on=0;
static int sayac14_off=0;

if (digitalRead(SENSOR_PIN14)==0) {
  sayac14_off=0;
  if (sayac14_on<10) {
    sayac14_on++;
  }
}else {
  sayac14_on=0;
    if (sayac14_off<10) {
    sayac14_off++;
  }
}

if (sayac14_on>=10) {
  lineSensor.sensor14=0;
}

if (sayac14_off>=10) {
  lineSensor.sensor14=1;
}

static int sayac15_on=0;
static int sayac15_off=0;

if (digitalRead(SENSOR_PIN15)==0) {
  sayac15_off=0;
  if (sayac15_on<10) {
    sayac15_on++;
  }
}else {
  sayac15_on=0;
    if (sayac15_off<10) {
    sayac15_off++;
  }
}

if (sayac15_on>=10) {
  lineSensor.sensor15=0;
}

if (sayac15_off>=10) {
  lineSensor.sensor15=1;
}


/*
lineSensor.sensor1 = digitalRead(SENSOR_PIN1);
lineSensor.sensor2 = digitalRead(SENSOR_PIN2);  
lineSensor.sensor3 = digitalRead(SENSOR_PIN3);
lineSensor.sensor4 = digitalRead(SENSOR_PIN4);
lineSensor.sensor5 = digitalRead(SENSOR_PIN5);
lineSensor.sensor6 = digitalRead(SENSOR_PIN6);
lineSensor.sensor7 = digitalRead(SENSOR_PIN7);
lineSensor.sensor8 = digitalRead(SENSOR_PIN8);
lineSensor.sensor9 = digitalRead(SENSOR_PIN9);
lineSensor.sensor10 = digitalRead(SENSOR_PIN10);
lineSensor.sensor11 = digitalRead(SENSOR_PIN11);
lineSensor.sensor12 = digitalRead(SENSOR_PIN12);
lineSensor.sensor13 = digitalRead(SENSOR_PIN13);
lineSensor.sensor14 = digitalRead(SENSOR_PIN14);
lineSensor.sensor15 = digitalRead(SENSOR_PIN15);
*/
lineSensor.sensor16 = 1;


return lineSensor.sensorAll;
}

void sensorEnabled(){
  /*
  if(sensorCount==0){
digitalWrite(FRONT_SENSOR_EN, HIGH);
digitalWrite(BACK_SENSOR_EN, LOW);
digitalWrite(LEFT_SENSOR_EN, LOW);
digitalWrite(RIGT_SENSOR_EN, LOW);
    sensorCount=1;
  }
  else if(sensorCount==1){
digitalWrite(FRONT_SENSOR_EN, LOW);
digitalWrite(BACK_SENSOR_EN, HIGH);
digitalWrite(LEFT_SENSOR_EN, LOW);
digitalWrite(RIGT_SENSOR_EN, LOW);
    sensorCount=0;
  }
  */

  digitalWrite(FRONT_SENSOR_EN, HIGH);
}
