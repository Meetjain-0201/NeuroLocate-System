/*
  Brain Biopsy Pen

  Takes 2 target values and traces the distance travelled.


  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 31 Jan 2023
  by Meet Jain

*/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "AS5600.h"

#define len 3
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //0x3C for 128x32
#define AS5600_ADDRESS 0x36
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


const int D1 = 2;    // Pin [ok/increment] button is connected
const int D2 = 3;    // Pin [cancel/decrement] button is connected
int green_led = 7;
int red_led = 6;
int buzzer = 5;      //get low voltage buzzer for the code to work

unsigned long buttonPressStart1 = 0;  // Variable to store button press start time
unsigned long buttonPressStart2 = 0;
unsigned long releasedtime1 = 0;
unsigned long releasedtime2 = 0;
unsigned long pressDuration = 600;  // Press duration threshold (in milliseconds)

bool shouldRestart = false;
float voltage;
float threshold_voltage = 2;

int reading1;
int reading2;

int store1[len] = {0, 0, 0}; 
int store2[len] = {0, 0, 0}; 
int store3[len] = {0, 0, 0};
int store4[len] = {0, 0, 0};
int j;
int count = 0;

//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value


float distance;
float distance1;
float distance2;
float startAngle = 0; //starting angle
float currentangle;
float totalAngle = 0; //total absolute angular displacement
float firstAngle = 0;
float previoustotalAngle = 0; //for the display printing
//char buffer[20] = {0};
//wheel
int diameter = 15; //mm

void setup() {
  
  Serial.begin(115200); 
  Wire.begin(); //start i2C  
  Wire.setClock(800000L); //fast clock
  checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  startAngle = degAngle; //update startAngle with degAngle - for taring

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();
  display.display();
  delay(1000);
  pinMode(D1, INPUT);  // Set button pin as input
  pinMode(D2, INPUT);
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
}


void loop() {
  display.clearDisplay();
  teststart(); 
  testy(); 
  testx(); 
  encodery();
  delay(5000);     //replace this with trigger of 90 degree turn
  encoderx();
 //distancecalc();
}

void teststart() {
  display.clearDisplay();
  status();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 15);
  display.println(F("HOLA AMIGOS"));
  display.display();      // Show initial text
  delay(1000);
  store1[0]=0;
  store1[1]=0;
  store1[2]=0;
  store2[0]=0;
  store2[1]=0;
  store2[2]=0;
  store3[0]=0;
  store3[1]=0;
  store3[2]=0;
  store4[0]=0;
  store4[1]=0;
  store4[2]=0;
  
}


void encoderx() {  // take continuous encoder reading find difference in angle calculate distance value
  
  
  reading1 = digitalRead(D1);
  reading2 = digitalRead(D2);
  status();
  
  while((reading1 == HIGH) && (reading2 == HIGH)){
    status();
    correctAngle();
    firstAngle = totalAngle;
    Serial.print(" firstAngle ");
    Serial.println(firstAngle);
    reading1 = digitalRead(D1);
  }
  digitalWrite(buzzer, HIGH);
  digitalWrite(buzzer, LOW);
  
  for(int i=0;i<3;i++){
    while(store2[i] != store4[i]){
      status();
      Serial.print("stuck");
      displayscreen();
      distancecalcX();
      display.clearDisplay();
    }
  }
  
  digitalWrite(buzzer, HIGH);
  digitalWrite(green_led, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(green_led, HIGH);
}
void encodery() {  // take continuous encoder reading find difference in angle calculate distance value
  
  reading1 = digitalRead(D1);
  reading2 = digitalRead(D2);
  status();
  
  while((reading1 == HIGH) && (reading2 == HIGH)){
    status();
    correctAngle();
    firstAngle = totalAngle;
    Serial.print(" firstAngle ");
    Serial.println(firstAngle);
    reading1 = digitalRead(D1);
  }
  digitalWrite(buzzer, HIGH);
  digitalWrite(buzzer, LOW);
  
  for(int i=0;i<3;i++){
    while(store1[i] != store3[i]){
      status();
      Serial.print("stuck");
      displayscreen();
      distancecalcY();
      display.clearDisplay();
    }
  }
  
  digitalWrite(buzzer, HIGH);
  digitalWrite(green_led, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(green_led, HIGH);
}

void distancecalcY(){
  correctAngle();
  currentangle = totalAngle - firstAngle;
  
  Serial.print(" currentangle ");
  Serial.print(currentangle);
  distance = (currentangle/360)*3.141592*1.5;
  Serial.print(" distance ");
  Serial.print(distance, 1);
  Serial.print(" ");
  store3[0] = (int)(distance / 10);
  store3[1] = (int)(distance) % 10;
  store3[2] = (int)(distance * 10) % 10;
  
  if((store3[0] >= 0) && (store3[1] >= 0) && (store3[2] >= 0)) {
    distance1 = 1;
  }
  else{
    distance1 =-1;
  }
  for(int i=0;i<3;i++){
    Serial.print(store3[i]);
  }
  Serial.println(" ");
}

void distancecalcX(){
  correctAngle();
  currentangle = totalAngle - firstAngle;
  
  Serial.print(" currentangle ");
  Serial.print(currentangle);
  distance = (currentangle/360)*3.141592*1.5;
  Serial.print(" distance ");
  Serial.print(distance, 1);
  Serial.print(" ");
  store4[0] = (int)(distance / 10);
  store4[1] = (int)(distance) % 10;
  store4[2] = (int)(distance * 10) % 10;
  
  if((store4[0] >= 0) && (store4[1] >= 0) && (store4[2] >= 0)) {
    distance2 = 1;
  }
  else{
    distance2 =-1;
  }
   
  for(int i=0;i<3;i++){
    Serial.print(store4[i]);
  }
  Serial.println(" ");
}

void correctAngle()
{
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 
  
//  Serial.print("Deg angle: ");
//  Serial.print(degAngle, 1); //absolute position of the encoder within the 0-360 circle

  ///////////////////////////////
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
//  Serial.print(" Corrected angle: ");
//  Serial.print(correctedAngle, 1); //print the corrected/tared angle 

  /////////////////
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  Serial.print(" Total angle: ");
  Serial.print(totalAngle, 1); //absolute position of the motor expressed in degree angles, 2 digits
}

void testy() {
  display.clearDisplay();
  status();
  display.setTextSize(1); // Draw 1X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 10);
  display.println(F("Insert Targets"));
  display.display();      // Show initial text
  delay(2000);
  display.clearDisplay();

  
  for(int j = 0; j < len;){  
    Serial.print("0 ");
    count = 0;
    int previous_j = j;
    while(j == previous_j){ 
      status();  
      Serial.print("1");
      displayscreen();    
      reading1 = digitalRead(D1);
      reading2 = digitalRead(D2);    
      
      while(reading1 == reading2){ // wait for button press
       Serial.println("2");
       status();
       reading1 = digitalRead(D1);
       reading2 = digitalRead(D2);
       if(reading1 == LOW){     //  B1 is pressed
       buttonPressStart1 = millis();
       Serial.print("b11111111111111/////////////////////////////////////////////////////");
       }
       else if(reading2 == LOW){ //  B2 is pressed
       buttonPressStart2 = millis();
       Serial.print("b2222222222222//////////////////////////////////////////////");
       }
      }
      while(reading1 != reading2){ // wait for releasing button
        status();
        digitalWrite(buzzer, HIGH);  
        if(reading1 == LOW){    
          releasedtime1 = millis();    //  B1 is released
          Serial.println("/////////////////////////////////////////////////////b11111111111111");
        }
        else if(reading2 == LOW){     //  B2 is released
          releasedtime2 = millis();
          Serial.println("/////////////////////////////////////////////////////b22222222222222");
        }
        digitalWrite(buzzer, LOW); 
        reading1 = digitalRead(D1);
        reading2 = digitalRead(D2);
      }
      Serial.print("b1  ");
      Serial.println(releasedtime1-buttonPressStart1);
      Serial.print("b2  ");
      Serial.println(releasedtime2-buttonPressStart2);
      
      //comparing duration of button pressed
      
      if((releasedtime1-buttonPressStart1) >= pressDuration) {
        store1[j]=count;
        j++;  
        Serial.print("[stored1:");
        Serial.print(count);
        Serial.println("]");              
      }
      else if((0 < (releasedtime1-buttonPressStart1)) && ((releasedtime1-buttonPressStart1) < pressDuration)) {
        count++;
        if(count > 9){
          count = 0;
        }
        store1[j]=count;
        Serial.print("[count1:");
        Serial.print(count);
        Serial.println("]");
        delay(100);
      }

      else if((releasedtime2-buttonPressStart2) >= pressDuration) {
        store1[j]=0;
        j--;
        if(j<0){
          j=0;
          count=0;
        }
        count=0;
        store1[j]=count;
        Serial.print("[stored1:");
        Serial.print(count);
        Serial.println("]");
      }
      else if((0 < (releasedtime2-buttonPressStart2)) && ((releasedtime2-buttonPressStart2) < pressDuration)){
        
        count--;
        
        if(count<0){
          count=9;
        }
        store1[j]=count;
        Serial.print("[count1");
        Serial.print(count);
        Serial.println("]");
        delay(100);
      }
      Serial.print("end");
      display.clearDisplay(); 
      releasedtime1=buttonPressStart1;
      releasedtime2=buttonPressStart2;
      delay(500);
    } 
  }     
}

void testx() {
  status();
  display.clearDisplay();
  store2[0]=0;
  store2[1]=0;
  store2[2]=0;
  
  for(int j = 0; j < len;){ 

    Serial.print("0 ");
    count = 0;
    int previous_j = j;
    while(j == previous_j){   
      status();
      Serial.print("1");
      displayscreen();
      reading1 = digitalRead(D1);
      reading2 = digitalRead(D2);    
      
      while(reading1 == reading2){ // wait for button press
       Serial.println("2");
       status();
       reading1 = digitalRead(D1);
       reading2 = digitalRead(D2);
       if(reading1 == LOW){
       buttonPressStart1 = millis();
       Serial.print("b11111111111111/////////////////////////////////////////////////////");
       }
       else if(reading2 == LOW){ //reading2 is low
       buttonPressStart2 = millis();
       Serial.print("b2222222222222//////////////////////////////////////////////");
       }
      }
      while(reading1 != reading2){
        status();
        if(reading1 == LOW){
          releasedtime1 = millis();
          digitalWrite(buzzer, HIGH); 
          Serial.println("/////////////////////////////////////////////////////b11111111111111");
        }
        else if(reading2 == LOW){
          releasedtime2 = millis();
          digitalWrite(buzzer, HIGH); 
          Serial.println("/////////////////////////////////////////////////////b22222222222222");
        }
        reading1 = digitalRead(D1);
        reading2 = digitalRead(D2);
      }
      digitalWrite(buzzer, LOW); 
      Serial.print("b1  ");
      Serial.println(releasedtime1-buttonPressStart1);
      Serial.print("b2  ");
      Serial.println(releasedtime2-buttonPressStart2);
      
      if((releasedtime1-buttonPressStart1) >= pressDuration) {
        store2[j]=count;
        j++;  
        Serial.print("[stored2:");
        Serial.print(count);
        Serial.println("]");              
      }
      else if((0 < (releasedtime1-buttonPressStart1)) && ((releasedtime1-buttonPressStart1) < pressDuration)) {
        count++;
        if(count > 9){
          count = 0;
        }
        store2[j]=count;
        Serial.print("[count2:");
        Serial.print(count);
        Serial.println("]");
        delay(100);
      }

      else if((releasedtime2-buttonPressStart2) >= pressDuration) {
        store2[j]=0;
        j--;
        if(j<0){
          j=0;
          count=0;
        }
        count=0;
        store2[j]=count;
        Serial.print("[stored2:");
        Serial.print(count);
        Serial.println("]");
      }
      else if((0 < (releasedtime2-buttonPressStart2)) && ((releasedtime2-buttonPressStart2) < pressDuration)){
        
        count--;
        
        if(count<0){
          count=9;
        }
        store2[j]=count;
        Serial.print("[count2");
        Serial.print(count);
        Serial.println("]");
        delay(100);
      }
      Serial.print("end");
      display.clearDisplay(); 
      releasedtime1=buttonPressStart1;
      releasedtime2=buttonPressStart2;
      delay(500);
    } 
  }     
}

void displayscreen(){
  
  display.setTextSize(1); // Draw 1X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 5);
  display.print(F("T1="));
  display.print(store1[0]);
  display.print(store1[1]);
  display.print(F("."));
  display.print(store1[2]);
  display.println(F("cm"));
  display.setCursor(64, 5);
  display.print(F("T2="));
  display.print(store2[0]);
  display.print(store2[1]);
  display.print(F("."));
  display.print(store2[2]);
  display.println(F("cm"));
  display.setCursor(0, 20);
  display.print(F("D1=")); 
  if (distance1 < 0){
    display.print(F("-"));
    display.print(-store3[0]);
    display.print(-store3[1]);
    display.print(F("."));
    display.print(-store3[2]);
    display.println(F("cm")); 
  }
  else{
    display.print(F(""));
    display.print(store3[0]);
    display.print(store3[1]);
    display.print(F("."));
    display.print(store3[2]);
    display.println(F("cm"));
  }
  display.setCursor(64, 20);
  display.print(F("D2="));
  if (distance2 < 0){
    display.print(F("-"));
    display.print(-store4[0]);
    display.print(-store4[1]);
    display.print(F("."));
    display.print(-store4[2]);
    display.println(F("cm"));
    
  }
  else{
    display.print(store4[0]);
    display.print(store4[1]);
    display.print(F("."));
    display.print(store4[2]);
    display.println(F("cm"));
  }
  display.display();      // Show initial text  
  
}

void status() {
//  Serial.print(" checking battery status:");

  checkMagnetPresence();
  voltage = analogRead(A1);
  voltage = map(voltage, 0, 1023, 0, 5);
  while (voltage < threshold_voltage){
    digitalWrite(green_led, LOW);   
    digitalWrite(red_led, HIGH);
    digitalWrite(buzzer, HIGH); 
    delay(100);
    digitalWrite(red_led, LOW);
    digitalWrite(buzzer, LOW); 
    delay(100);
    display.clearDisplay();
    display.setTextSize(1); // Draw 1X-scale text
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(30, 15);
    display.println(F("Low Battery"));
    display.display();      // Show initial text
    display.clearDisplay();
    delay(100); 
    voltage = analogRead(A0);
    voltage = map(voltage, 0, 1023, 0, 5);
    Serial.println(voltage);
    loop();
  }
  Serial.println("battery charged");
  digitalWrite(green_led, HIGH);
  
  
}

void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  if((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print(" Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)
        
  }      
  else{
    //Status register output: 0 0 MD ML MH 0 0 0  
    //MH: Too strong magnet - 100111 - DEC: 39 
    //ML: Too weak magnet - 10111 - DEC: 23     
    //MD: OK magnet - 110111 - DEC: 55
  
    Serial.print(" Magnet found!");
    delay(10);  
  }
}
