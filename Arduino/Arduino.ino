#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SoftwareSerial.h>
//defining the analog pin for gassensor
// #define MQ2pin (0)
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial nodemcu(2, 3);
//defining digital pins
int pirled = 9;
int mq2led = 11;
int greenled = 10;
int fan = 4;
int pirsensor = 8;
//int MQ2pin = 7;

const byte MQ2pin = A0;


//values
int pir = 0;
float gasSensor = 0;
int buzzer = 5;

//creating servo object
Servo myservo;
int pos = 0;


void setup() {
  // put your setup code here, to run once:
  //servo pos
  myservo.attach(6);

  SPI.begin();  // Init SPI bus
  //pinmodes
  pinMode(pirsensor, INPUT);
  pinMode(MQ2pin, INPUT);
  pinMode(pirled, OUTPUT);
  pinMode(mq2led, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // Serial Monitor Setup
  Serial.begin(115200);
  nodemcu.begin(115200);
  //LCD setup
  // lcd.begin(16,2);
  // lcd.init();
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Working...");

  //initial values for LED AND FAN set to LOW
  digitalWrite(pirled, LOW);
  digitalWrite(mq2led, LOW);
  digitalWrite(greenled, HIGH);
  digitalWrite(fan, LOW);
}

float getMethanePPM(){
    const int R_0 = 945;

   float a0 = analogRead(A0); // get raw reading from sensor
   float v_o = a0 * 5 / 1023; // convert reading to volts
   float R_S = (5-v_o) * 1000 / v_o; // apply formula for getting RS
   float PPM = pow(R_S/R_0,-2.95) * 1000; //apply formula for getting PPM
   return PPM; // return PPM value to calling function
}

void loop() {



  //LCD
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);

  //pir sensor
  pir = digitalRead(pirsensor);
  //gas sensor
  // gasSensor = analogRead(MQ2pin);
  gasSensor = getMethanePPM();
  digitalWrite(greenled, HIGH);
  digitalWrite(pirled, LOW);
  digitalWrite(mq2led, LOW);

  Serial.print("gasSensor = ");
  Serial.println(gasSensor);
  Serial.print("pirSensor = ");
  Serial.println(pir);
  //condition for Gaensor alert
  if (gasSensor <= 200 && pir == LOW) {
    digitalWrite(mq2led, LOW);
    digitalWrite(pirled, LOW);
    digitalWrite(greenled, HIGH);
    lcd.print("ALL OK");
    Serial.println("ALL OK");
    digitalWrite(fan, LOW);
    noTone(buzzer);
     nodemcu.println("0 all okay");
    // delay(1000);
    //transmitting data to webserver
  }
  
  //transmitting data to webserver



  // condition for PIR Sensor alert
  if (pir == HIGH) {
    digitalWrite(pirled, HIGH);
    digitalWrite(greenled, LOW);
    lcd.print("Intruder");
    tone(buzzer, 200);
    Serial.println("Inside PIR");
    Serial.print("gasSensor = ");
    Serial.println(gasSensor);
    Serial.print("pirSensor = ");
    Serial.println(pir);
    // delay(3000);

    //transmitting data to webserver
    nodemcu.println("1 intruder");
  }


  if (gasSensor > 200) {
    lcd.print("Gas Detected");
    digitalWrite(fan, HIGH);
    digitalWrite(mq2led, HIGH);
    digitalWrite(greenled, LOW);
    Serial.println("Inside GAS");
    Serial.print("gasSensor = ");
    Serial.println(gasSensor);
    Serial.print("pirSensor = ");
    Serial.println(pir);
    tone(buzzer, 500);
    nodemcu.println("2 Gas detected");
    // servo opening windows
    for (pos = 0; pos <= 90; pos += 1) {  // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);  // tell servo to go to position in variable 'pos'
      delay(10);           // waits 15ms for the servo to reach the position
    }
    // delay(1000);
  }

  delay(1000);
  //transmitting data to webserver
}