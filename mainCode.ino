#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define BATTERY_AVARAGE_READ_COUNT 10

#define A_Palse 8
#define A_Dir 10
#define B_Palse 44
#define B_Dir 46
#define C_Palse 13
#define C_Dir 27
#define D_Palse 9
#define D_Dir 11

//rgb pin
#define RED_PIN 5
#define GREEN_PIN 6
#define BLUE_PIN 7
//buzzer pin
#define BUZZER_PIN 8
//millis function
unsigned long oldTime=0;
unsigned long newTime;
int buzzerCount = 0;

long TIME_NOW = 0;
unsigned long adc_millis = 0;


LiquidCrystal_I2C lcd(0x27, 20, 4);

#define ILERI_PIN A6
#define GERI_PIN A4
#define SAGA_PIN A0
#define SOLA_PIN A2
#define SOLA_DON_PIN A1
#define SAGA_DON_PIN A3

#define ILERI_GIT A6
#define GERI_GIT A4
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

  pinMode(A_Palse, OUTPUT);
  pinMode(B_Palse, OUTPUT);
  pinMode(C_Palse, OUTPUT);
  pinMode(D_Palse, OUTPUT);

  pinMode(ILERI_PIN, INPUT);
  pinMode(GERI_PIN, INPUT);
  pinMode(SAGA_PIN, INPUT);
  pinMode(SOLA_PIN, INPUT);
  pinMode(SOLA_DON_PIN, INPUT);
  pinMode(SAGA_DON_PIN, INPUT);

  pinMode(ILERI_GIT, INPUT);
  pinMode(GERI_GIT, INPUT);

  pinMode(A_Dir, OUTPUT);
  pinMode(B_Dir, OUTPUT);
  pinMode(C_Dir, OUTPUT);
  pinMode(D_Dir, OUTPUT);

  digitalWrite(A_Dir, HIGH);
  digitalWrite(B_Dir, HIGH);
  digitalWrite(C_Dir, HIGH);
  digitalWrite(D_Dir, HIGH);

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
  int ILERI_GIT_BUTTON = analogRead(ILERI_GIT);
  int GERI_GIT_BUTTON = analogRead(GERI_GIT);

  TIME_NOW = millis();
  if (TIME_NOW - adc_millis > 1)
  {
    adc_millis = TIME_NOW;
    int battery_value = vBattRead();
  }




  if (SAGA_TRY == LOW )
  {

    RotateWheels(false, false, false, true, false, false);
    rgbController(true, false, false);
    //Serial.println("SAGA");
  }
  else if (GERI_TRY == LOW || ILERI_GIT_BUTTON == ileriAnalogValue)
  {
    RotateWheels(true, false, false, false, false, false);
    rgbController(true, false, false);
   // Serial.println("GERI");
  }
  else if (ILERI_TRY == LOW )
  {

    RotateWheels(false, true, false, false, false, false);
    rgbController(true, false, false);
   //Serial.println("ILERI");
  }
  else if (SOLA_TRY == LOW)
  {

    RotateWheels(false, false, true, false, false, false);
    rgbController(false, true, false);
   // Serial.println("SOLA");
  }
   else if (SOLA_DON == LOW)
    {
    
    RotateWheels(false, false, false, false, true, false);
    rgbController(false, true, false);
     // Serial.println("SOLA_DON");
    }
    else if (SAGA_DON == LOW)
    {
    rgbController(false, true, false);
    RotateWheels(false, false, false, false, false, true);
     // Serial.println("SAGA_DON");
    }else if(GERI_GIT_BUTTON == geriAnalogValue){


    }else if(ILERI_GIT_BUTTON == ileriAnalogValue){


    }
    else
    {
        RotateWheels(false, false, false, false, false, false);
    }

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

void LCD_MESSAGE(int hiz)
{
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
        adc_value += analogRead(Vbatt_ADC);
    }
    adc_value = adc_value / BATTERY_AVARAGE_READ_COUNT;
    float Vbatt = (adc_value * 5) / 1024;
    return Vbatt;
}

void rgbController(bool WakeUpLight, bool straightLight, bool lateralLight)
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