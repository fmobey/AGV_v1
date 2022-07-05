
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
void setup() {
    Serial.begin(9600);
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
}

void loop()
{ 

}
/*
void wakeUpLight(){
    newTime = millis(); 
if(newTime-oldTime > 1000) {

if(lightCount == 0){
analogWrite(RED_PIN, 254/2);
analogWrite(GREEN_PIN, 254/2);
analogWrite(BLUE_PIN, 254/2);
    lightCount = 1;
}
else if(lightCount == 1){
analogWrite(RED_PIN, 0);
analogWrite(GREEN_PIN, 0);
analogWrite(BLUE_PIN, 0);
    lightCount = 0;
}
    }
    oldTime = newTime;
}

void straightLight(){
    newTime = millis(); 
if(newTime-oldTime > 1000) {

if(lightCount == 0){
analogWrite(RED_PIN, 154/2);
analogWrite(GREEN_PIN, 0 );
analogWrite(BLUE_PIN, 204/2);
    lightCount = 1;
}
else if(lightCount == 1){
analogWrite(RED_PIN, 0);
analogWrite(GREEN_PIN, 0);
analogWrite(BLUE_PIN, 0);
    lightCount = 0;
}
    }
    oldTime = newTime;
}

void lateralLight(){
    newTime = millis(); 
if(newTime-oldTime > 1000) {
if(lightCount == 0){
analogWrite(RED_PIN, 0);
analogWrite(GREEN_PIN, 52/2);
analogWrite(BLUE_PIN, 154/2);
    lightCount = 1;
}

else if(lightCount == 1){
analogWrite(RED_PIN, 0);
analogWrite(GREEN_PIN, 0);
analogWrite(BLUE_PIN, 0);
    lightCount = 0;
}
    }
    oldTime = newTime;
}
*/
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