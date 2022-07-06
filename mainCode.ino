#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define BATTERY_AVARAGE_READ_COUNT 10

#define APWM 12
#define ADIR 10
#define ADIRFW 8

#define BPWM 46
#define BDIR 48
#define BDIRFW 30

#define CPWM 45
#define CDIR 25
#define CDIRFW 23

#define DPWM 13
#define DDIR 11
#define DDIRFW 9


//rgb pin
#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4
//buzzer pin
#define BUZZER_PIN 5
//millis function
unsigned long oldTime=0;
unsigned long newTime;
int buzzerCount = 0;

long TIME_NOW = 0;
unsigned long adc_millis = 0;


LiquidCrystal_I2C lcd(0x27, 20, 4);

#define ILERI_PIN 43
#define GERI_PIN 41
#define SAGA_PIN 7
#define SOLA_PIN 44
#define SOLA_DON_PIN 29
#define SAGA_DON_PIN 31

#define ANALOG_PIN_BUTTON A0

#define VBATTADC 14

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

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(APWM, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(CPWM, OUTPUT);
  pinMode(DPWM, OUTPUT);

  pinMode(ILERI_PIN, INPUT);
  pinMode(GERI_PIN, INPUT);
  pinMode(SAGA_PIN, INPUT);
  pinMode(SOLA_PIN, INPUT);
  pinMode(SOLA_DON_PIN, INPUT);
  pinMode(SAGA_DON_PIN, INPUT);

  pinMode(ILERI_GIT, INPUT);
  pinMode(GERI_GIT, INPUT);

  pinMode(ADIR, OUTPUT);
  pinMode(BDIR, OUTPUT);
  pinMode(CDIR, OUTPUT);
  pinMode(DDIR, OUTPUT);

  digitalWrite(ADIR, HIGH);
  digitalWrite(BDIR, HIGH);
  digitalWrite(CDIR, HIGH);
  digitalWrite(DDIR, HIGH);

  lcd.clear();

}

void loop()
{
  //panel buttonarının durumlarını okuyup gösteriyoruz
  int ILERI_TRY = digitalRead(ILERI_PIN);
  int GERI_TRY = digitalRead(GERI_PIN);
  int SAGA_TRY = digitalRead(SAGA_PIN);
  int SOLA_TRY = digitalRead(SOLA_PIN);
  int SAGA_DON = digitalRead(SAGA_DON_PIN);
  int SOLA_DON = digitalRead(SOLA_DON_PIN);
  //istasyon butonlarının durumlarını okuyup gösteriyoruz
  int ANALOG_GIT_BUTTON = analogRead(ANALOG_PIN_BUTTON);


  TIME_NOW = millis();
  if (TIME_NOW - adc_millis > 1)
  {
    adc_millis = TIME_NOW;
    int battery_value = vBattRead();
  }




  if (SAGA_TRY == LOW )
  {

    RotateWheels(false, false, false, true, false, false);
    rgbController(true, false, false, false);
    //Serial.println("SAGA");
  }
  else if (GERI_TRY == LOW || ILERI_GIT_BUTTON == ileriAnalogValue)
  {
    RotateWheels(true, false, false, false, false, false);
    rgbController(true, false, false, false);
   // Serial.println("GERI");
  }
  else if (ILERI_TRY == LOW )
  {

    RotateWheels(false, true, false, false, false, false);
    rgbController(true, false, false, false);
   //Serial.println("ILERI");
  }
  else if (SOLA_TRY == LOW)
  {

    RotateWheels(false, false, true, false, false, false);
    rgbController(false, true, false, false);
   // Serial.println("SOLA");
  }
   else if (SOLA_DON == LOW)
    {
    
    RotateWheels(false, false, false, false, true, false);
    rgbController(false, true, false, false);
     // Serial.println("SOLA_DON");
    }
    else if (SAGA_DON == LOW)
    {
    rgbController(false, true, false, false);
    RotateWheels(false, false, false, false, false, true);
     // Serial.println("SAGA_DON");
    }else if(ANALOG_GIT_BUTTON == geriAnalogValue){


    }else if(ANALOG_GIT_BUTTON == ileriAnalogValue){


    }
    else
    {
        RotateWheels(false, false, false, false, false, false);
    }

}
//10 munite timer stanby mode 
void timer_standby()
{
  if (TIME_NOW - oldTime > 10000)
  {

  }
}




void RotateWheels(bool ileri, bool geri, bool sag, bool sol, bool soldon, bool sagdon)
{
  
  if (ileri == true)
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
  else if (geri == true)
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
  else if (sag == true)
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
  else if (sol == true)
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
  else if (soldon == true)
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
  else if (sagdon == true)
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

void rgbController(bool WakeUpLight, bool straightLight, bool lateralLight,bool lidarLight)
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
else
{

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
if(redCount != 76){
analogWrite(RED_PIN, redCount);
redCount++;
}
    }
for(int i = 102; i < 0; i--){
analogWrite(BLUE_PIN, i);
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
if(greenCount != 26){
analogWrite(GREEN_PIN, greenCount);
greenCount++;
}
    }
for(int i = 77; i < 0; i--){
analogWrite(BLUE_PIN, i);
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
}
    }
for(int i = 127; i < 0; i--){
analogWrite(RED_PIN, i);

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