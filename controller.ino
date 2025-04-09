/*
  Brain Biopsy Pen - Precision Measurement Tool

  Hardware:
  - AS5600 magnetic rotary encoder (I2C)
  - SSD1306 OLED display (128x32)
  - Two-button interface (D1/D2)
  - Status LEDs and buzzer

  Functionality:
  - Measures linear distance using rotary encoder
  - Stores target values for X/Y axes
  - Compares actual vs target distances
  - Provides visual/audible feedback
*/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "AS5600.h"

// Configuration Constants
#define len 3
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define AS5600_ADDRESS 0x36
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Pin Definitions
const int D1 = 2;    // Primary button (OK/Increment)
const int D2 = 3;    // Secondary button (Cancel/Decrement)
const int green_led = 7;  // System status LED
const int red_led = 6;    // Warning LED
const int buzzer = 5;     // Audible feedback device

// Timing Variables
unsigned long buttonPressStart1 = 0;
unsigned long buttonPressStart2 = 0;
unsigned long releasedtime1 = 0;
unsigned long releasedtime2 = 0;
const unsigned long pressDuration = 600;  // Long-press threshold (ms)

// System State
bool shouldRestart = false;
float voltage;
const float threshold_voltage = 2.0;

// Button States
int reading1;
int reading2;

// Measurement Storage
int store1[len] = {0};  // Target Y values [units, tenths, hundredths]
int store2[len] = {0};  // Target X values
int store3[len] = {0};  // Measured Y values
int store4[len] = {0};  // Measured X values

// Encoder Variables
int magnetStatus = 0;
int lowbyte;
word highbyte;
int rawAngle;
float degAngle;
int quadrantNumber, previousquadrantNumber;
float numberofTurns = 0;
float correctedAngle = 0;
float startAngle = 0;
float currentangle;
float totalAngle = 0;
float previoustotalAngle = 0;
float firstAngle = 0;

// Physical Constants
const int diameter = 15;  // Measurement wheel diameter (mm)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(800000L);
  
  // Initialize magnetic encoder
  checkMagnetPresence();
  startAngle = degAngle;

  // Initialize display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("[ERROR] Display initialization failed");
    while(1);
  }
  display.clearDisplay();
  display.display();
  delay(1000);

  // Configure I/O
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  display.clearDisplay();
  initializeSystem(); 
  setYTarget(); 
  setXTarget(); 
  measureYAxis();
  delay(5000);     // Simulated 90° turn period
  measureXAxis();
}

/* ======================
   CORE FUNCTIONS
   ====================== */

void initializeSystem() {
  display.clearDisplay();
  checkSystemStatus();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 15);
  display.println(F("SYSTEM READY"));
  display.display();
  delay(1000);
  
  // Reset all measurement buffers
  for(int i=0; i<len; i++) {
    store1[i]=0;
    store2[i]=0;
    store3[i]=0;
    store4[i]=0;
  }
}

void measureXAxis() {
  reading1 = digitalRead(D1);
  reading2 = digitalRead(D2);
  checkSystemStatus();
  
  // Wait for measurement start
  while((reading1 == HIGH) && (reading2 == HIGH)) {
    checkSystemStatus();
    updateAngle();
    firstAngle = totalAngle;
    Serial.print("[X] Initial angle: ");
    Serial.println(firstAngle);
    reading1 = digitalRead(D1);
  }
  provideFeedback();
  
  // Perform measurement until target reached
  for(int i=0; i<len; i++) {
    while(store2[i] != store4[i]) {
      checkSystemStatus();
      updateDisplay();
      calculateXDistance();
      display.clearDisplay();
    }
  }
  
  measurementComplete();
}

void calculateXDistance() {
  updateAngle();
  currentangle = totalAngle - firstAngle;
  
  Serial.print("[X] Angle change: ");
  Serial.print(currentangle);
  
  distance = (currentangle/360)*3.141592*1.5;
  Serial.print(" | Distance: ");
  Serial.print(distance, 1);
  
  // Store measurement components
  store4[0] = (int)(distance / 10);
  store4[1] = (int)(distance) % 10;
  store4[2] = (int)(distance * 10) % 10;
  
  distance2 = (distance >= 0) ? 1 : -1;
  
  Serial.print(" | Value: ");
  for(int i=0; i<len; i++) {
    Serial.print(store4[i]);
  }
  Serial.println();
}

/* ======================
   USER INTERFACE
   ====================== */

void setYTarget() {
  display.clearDisplay();
  checkSystemStatus();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 10);
  display.println(F("SET Y TARGET"));
  display.display();
  delay(2000);
  display.clearDisplay();

  for(int digit = 0; digit < len; digit++) {  
    int currentValue = 0;
    int previousDigit = digit;
    
    while(digit == previousDigit) { 
      checkSystemStatus();  
      updateDisplay();    
      reading1 = digitalRead(D1);
      reading2 = digitalRead(D2);    
      
      // Wait for button press
      while(reading1 == reading2) {
        checkSystemStatus();
        reading1 = digitalRead(D1);
        reading2 = digitalRead(D2);
        if(reading1 == LOW) buttonPressStart1 = millis();
        else if(reading2 == LOW) buttonPressStart2 = millis();
      }
      
      // Wait for button release
      while(reading1 != reading2) {
        checkSystemStatus();
        digitalWrite(buzzer, HIGH);  
        if(reading1 == LOW) releasedtime1 = millis();
        else if(reading2 == LOW) releasedtime2 = millis();
        digitalWrite(buzzer, LOW); 
        reading1 = digitalRead(D1);
        reading2 = digitalRead(D2);
      }
      
      // Process button actions
      if((releasedtime1-buttonPressStart1) >= pressDuration) {
        store1[digit]=currentValue;
        digit++;  
        Serial.println("[Y] Target confirmed");              
      }
      else if((0 < (releasedtime1-buttonPressStart1)) && 
             ((releasedtime1-buttonPressStart1) < pressDuration)) {
        currentValue = (currentValue + 1) % 10;
        store1[digit]=currentValue;
        Serial.print("[Y] Incremented to: ");
        Serial.println(currentValue);
        delay(100);
      }
      else if((releasedtime2-buttonPressStart2) >= pressDuration) {
        store1[digit]=0;
        digit = max(0, digit-1);
        currentValue=0;
        store1[digit]=currentValue;
        Serial.println("[Y] Value cancelled");
      }
      else if((0 < (releasedtime2-buttonPressStart2)) && 
             ((releasedtime2-buttonPressStart2) < pressDuration)) {
        currentValue = (currentValue - 1 + 10) % 10;
        store1[digit]=currentValue;
        Serial.print("[Y] Decremented to: ");
        Serial.println(currentValue);
        delay(100);
      }
      
      // Reset timing variables
      releasedtime1 = buttonPressStart1;
      releasedtime2 = buttonPressStart2;
      delay(500);
    } 
  }     
}

/* ======================
   DISPLAY FUNCTIONS
   ====================== */

void updateDisplay() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Target values
  display.setCursor(0, 5);
  display.print(F("Y="));
  display.print(store1[0]);
  display.print(store1[1]);
  display.print(F("."));
  display.print(store1[2]);
  display.println(F("cm"));
  
  display.setCursor(64, 5);
  display.print(F("X="));
  display.print(store2[0]);
  display.print(store2[1]);
  display.print(F("."));
  display.print(store2[2]);
  display.println(F("cm"));
  
  // Measured values
  display.setCursor(0, 20);
  display.print(F("MY="));
  if(distance1 < 0) display.print(F("-"));
  display.print(abs(store3[0]));
  display.print(abs(store3[1]));
  display.print(F("."));
  display.print(abs(store3[2]));
  display.println(F("cm"));
  
  display.setCursor(64, 20);
  display.print(F("MX="));
  if(distance2 < 0) display.print(F("-"));
  display.print(abs(store4[0]));
  display.print(abs(store4[1]));
  display.print(F("."));
  display.print(abs(store4[2]));
  display.println(F("cm"));
  
  display.display();
}

/* ======================
   SYSTEM FUNCTIONS
   ====================== */

void checkSystemStatus() {
  checkMagnetPresence();
  
  voltage = analogRead(A1) * (5.0 / 1023.0);
  while(voltage < threshold_voltage) {
    handleLowVoltage();
    voltage = analogRead(A0) * (5.0 / 1023.0);
  }
  digitalWrite(green_led, HIGH);
}

void provideFeedback() {
  digitalWrite(buzzer, HIGH);
  delay(50);
  digitalWrite(buzzer, LOW);
}

void measurementComplete() {
  digitalWrite(buzzer, HIGH);
  digitalWrite(green_led, LOW);
  delay(100);
  digitalWrite(buzzer, LOW);
  digitalWrite(green_led, HIGH);
}

void handleLowVoltage() {
  digitalWrite(green_led, LOW);   
  digitalWrite(red_led, HIGH);
  digitalWrite(buzzer, HIGH); 
  delay(100);
  digitalWrite(red_led, LOW);
  digitalWrite(buzzer, LOW); 
  delay(100);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 15);
  display.println(F("LOW VOLTAGE"));
  display.display();
}
