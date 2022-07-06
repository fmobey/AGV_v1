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
//RGB 
#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4
//BUZZER
#define BUZZER_PIN 5
//PWM VALUE
#define PWM_START 255
#define PWM_STOP 0
//MILLIS
unsigned long oldTime=0;
unsigned long newTime;
int buzzerCount = 0;

long TIME_NOW = 0;
unsigned long adc_millis = 0;


LiquidCrystal_I2C lcd(0x27, 20, 4);


int ileriAnalogValue = 256;
int geriAnalogValue = 512;










void setup()
{

  Wire.begin();
  Serial.begin(115200);

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
//MOTOR PİNMODE
  pinMode(APWM, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(CPWM, OUTPUT);
  pinMode(DPWM, OUTPUT);
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
  rgbController(bool WakeUpLight, bool straightLight, bool lateralLight,bool lidarLight,bool stationLight)
  LineFlowingEnable(bool front,bool back,bool right,bool left)
  PwmStart(int pwm_value)
  PwmStop(int pwm_value)

  */


  //panel buttonarının durumlarını okuyup gösteriyoruz
  int ILERI_TRY = digitalRead(ILERI_PIN);
  int GERI_TRY = digitalRead(GERI_PIN);
  int SAGA_TRY = digitalRead(SAGA_PIN);
  int SOLA_TRY = digitalRead(SOLA_PIN);
  int SAGA_DON = digitalRead(SAGA_DON_PIN);
  int SOLA_DON = digitalRead(SOLA_DON_PIN);
  //istasyon butonunun durumlarını okuyup gösteriyoruz
  int ANALOG_GIT_BUTTON = analogRead(ANALOG_PIN_BUTTON);
  //Sensor Pinlerini durumlarını okuyup gösteriyoruz
  int Sensor1 = digitalRead(SENSOR_PIN1);
  int Sensor2 = digitalRead(SENSOR_PIN2);
  int Sensor3 = digitalRead(SENSOR_PIN3);
  int Sensor4 = digitalRead(SENSOR_PIN4);
  int Sensor5 = digitalRead(SENSOR_PIN5);
  int Sensor6 = digitalRead(SENSOR_PIN6);
  int Sensor7 = digitalRead(SENSOR_PIN7);
  int Sensor8 = digitalRead(SENSOR_PIN8);
  int Sensor9 = digitalRead(SENSOR_PIN9);
  int Sensor10 = digitalRead(SENSOR_PIN10);
  int Sensor11 = digitalRead(SENSOR_PIN11);
  int Sensor12 = digitalRead(SENSOR_PIN12);
  int Sensor13 = digitalRead(SENSOR_PIN13);
  int Sensor14 = digitalRead(SENSOR_PIN14);
  int Sensor15 = digitalRead(SENSOR_PIN15);


  TIME_NOW = millis();
  if (TIME_NOW - adc_millis > 1)
  {
    adc_millis = TIME_NOW;
    int battery_value = vBattRead();
  }

    if(ANALOG_GIT_BUTTON == geriAnalogValue)
    {


    }
    else if(ANALOG_GIT_BUTTON == ileriAnalogValue)
    {


    }
    else
    {
      
    }
    


  if (SAGA_TRY == LOW )
  {
    RotateWheels(false, false, false, true, false, false);
    rgbController(true, false, false, false,false);
    PwmStart(PWM_START);
    // Serial.println("SAGA");
  }
  else if (GERI_TRY == LOW )
  {
    RotateWheels(true, false, false, false, false, false);
    rgbController(true, false, false, false,false);
    PwmStart(PWM_START);

   // Serial.println("GERI");
  }
  else if (ILERI_TRY == LOW )
  {
    RotateWheels(false, true, false, false, false, false);
    rgbController(true, false, false, false,false);
    PwmStart(PWM_START);

   //Serial.println("ILERI");
  }
  else if (SOLA_TRY == LOW)
  {
    RotateWheels(false, false, true, false, false, false);
    rgbController(false, true, false, false,false);
    PwmStart(PWM_START);

   // Serial.println("SOLA");
  }
  else if (SOLA_DON == LOW)
  {
   
   RotateWheels(false, false, false, false, true, false);
   rgbController(false, true, false, false,false);
   PwmStart(PWM_START);

    // Serial.println("SOLA_DON");
  }
  else if (SAGA_DON == LOW)
  {
   rgbController(false, true, false, false,false);
   RotateWheels(false, false, false, false, false, true);
   PwmStart(PWM_START);

    // Serial.println("SAGA_DON");
   }
   else
   {
    RotateWheels(false, false, false, false, false, false);
    rgbController(false, true, false, false,false);
    PwmStop(PWM_STOP);
   }

}































void PwmStart(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value);
}
void PwmStop(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value);
}

//10 munite timer stanby mode 
void timer_standby()
{
  if (TIME_NOW - oldTime > 600000)
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

void LCD_MESSAGE(int hiz)
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
lcd.print("   ");
lcd.setCursor(0, 3);
lcd.print("HIZ: ");
lcd.setCursor(6, 3);
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

void rgbController(bool WakeUpLight, bool straightLight, bool lateralLight,bool lidarLight,bool stationLight)
{
if(WakeUpLight == true)
{
WakeUpLightFade();
}
else if(straightLight == true)
{
straightLightFade();
}
else if(lateralLight == true)
{
lateralLightFade();
}
else if(lidarLight == true)
{
lidarLightFade();
}
else if(stationLight == true)
{
stationLightFade();
}
else
{
  analogWrite(RED_PIN, 0);
  analogWrite(GREEN_PIN, 0);
  analogWrite(BLUE_PIN, 0);
}
}

void WakeUpLightFade(){
    newTime = millis(); 
if(newTime-oldTime > 1000) {
for(int i = 0; i < 127; i++){
analogWrite(RED_PIN, i);
analogWrite(GREEN_PIN, i);
analogWrite(BLUE_PIN, i);
    }
    for(int i = 127; i < 0; i--){
analogWrite(RED_PIN, i);
analogWrite(GREEN_PIN, i);
analogWrite(BLUE_PIN, i);
    }
    oldTime = newTime;
}}

void straightLightFade(){
    newTime = millis(); 
    int redCount = 0;
if(newTime-oldTime > 1000) {
for(int i = 0; i < 102; i++){
analogWrite(BLUE_PIN, i);
analogWrite(GREEN_PIN, 0);
if(redCount != 76){
analogWrite(RED_PIN, redCount);
redCount++;
}
    }
for(int i = 102; i < 0; i--){
analogWrite(BLUE_PIN, i);
analogWrite(GREEN_PIN, 0);
if(redCount != 0){
analogWrite(RED_PIN, redCount);
redCount--;
}
    }
    oldTime = newTime;
}}

void lateralLightFade(){
    newTime = millis(); 
    int greenCount = 0;
if(newTime-oldTime > 1000) {
for(int i = 0; i < 77; i++){
analogWrite(BLUE_PIN, i);
analogWrite(RED_PIN, 0);
if(greenCount != 26){
analogWrite(GREEN_PIN, greenCount);
greenCount++;
}
    }
for(int i = 77; i < 0; i--){
analogWrite(BLUE_PIN, i);
analogWrite(RED_PIN, 0);
if(greenCount != 0){
analogWrite(GREEN_PIN, greenCount);
greenCount--;
}
    }
    oldTime = newTime;
}}

void lidarLightFade(){
    newTime = millis(); 
if(newTime-oldTime > 1000) {
for(int i = 0; i < 127; i++){
analogWrite(RED_PIN, i);
analogWrite(GREEN_PIN, 0);
analogWrite(BLUE_PIN, 0);
}
    }
for(int i = 127; i < 0; i--){
analogWrite(RED_PIN, i);
analogWrite(GREEN_PIN, 0);
analogWrite(BLUE_PIN, 0);
    }
    oldTime = newTime;
}

void stationLightFade(){
    newTime = millis(); 
if(newTime-oldTime > 1000) {
for(int i = 0; i < 127; i++){
analogWrite(GREEN_PIN, i);
analogWrite(RED_PIN, 0);
analogWrite(BLUE_PIN, 0);
}
    }
for(int i = 127; i < 0; i--){
analogWrite(RED_PIN, i);
analogWrite(RED_PIN, 0);
analogWrite(BLUE_PIN, 0);
    }
    oldTime = newTime;
}

void buzzerFlipFlop(){
    newTime = millis();
if(newTime-oldTime > 1000) {
if(buzzerCount == 0){
digitalWrite(BUZZER_PIN, HIGH);
buzzerCount = 1;
}
else if(buzzerCount == 1){
digitalWrite(BUZZER_PIN, LOW);
buzzerCount = 0;
}
oldTime = newTime;
}
}