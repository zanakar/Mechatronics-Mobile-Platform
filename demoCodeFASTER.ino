#include <QTRSensors.h>
#include <TimerOne.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
//#include <MotorControl.h>
#define SETPOINT    3500  // The goal for readLine (center)
#define KP          0.052   // The P value in PID .02, 0.05, 0.052
#define KD          2.5       // The D value in PID  0.5, 1.7, 2.201
#define KI           0;
#define L_MOTOR     9     // Left motor pin
#define R_MOTOR     10    // Right motor pin
#define MAX_SPEED   100   // The max speed to set motors to
#define SET_SPEED   50   // The goal speed to set motors to
#define MIN_SPEED   0     // The min speed to set motors to
#define NUM_SENSORS 8     // The number of QTR sensors
#define TIMEOUT     2500  // Timeout for the QTR sensors to go low
#define EMITTER_PIN 2     // Emitter pin for QTR sensor emitters
#define distWall    10     //distance from wall for dist sensor
#define echoPin 11 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 38 //attach pin D3 Arduino to pin Trig of HC-SR04
//motor control
QTRSensors qtr;
int ENA = 6; //  RIGHT MOTOR
int ENB = 7; // LEFT MOTOR
int IN1 = 23;
int IN2 = 24;
int IN3 = 25;
int IN4 = 26;
int firatEn = 0;
//int DW4 = 46;
//int DW3 = 48;
//int DW2 = 50;
//int DW1 = 52;
int enable = 0;
int Speed = 80;
unsigned int sumOfErrors = 0;
int lastError = 0;  // For storing PID error
double dist = 0;
double voltage = 0;
unsigned long previousMillis = 0;
int firstStop = 0;
int secondStop = 0;
int cal = 250;
int state = 0;
int cmdistance = 10;
double distcm = 0;
float conversionVar = .0105425;
int num = 0;
int BLineCount = 0;
int flagB = 0;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
int redgreen = 0;
int blackCount = 0;
//library
//distance count global variables
volatile long pulse = 0;
int long currNumPulse = 0;
char menu;
int count = 0;
int error = 0;
int adjust = 0;
int Scolor = 0;
char inp1 = 0;
char inp2 = 0;
int Start = 0;
int thresholdW = 270;
int thresholdB = 650;
uint16_t whiteSens[8];
uint16_t blackSens[8];
int16_t positionB;
int redC = 0;
int greenC = 0;



// defines variables
double duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement

void setup() {
  Serial.begin(250000); //230400
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    28, 29, 30, 31, 32, 33, 34, 35
  }, 8); //insert pins here
  pinMode(46, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  // motor control
  // put your setup code here, to run once:
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  analogWrite(ENA, LOW);
  analogWrite(ENB, LOW);
  // enables for forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  // pin for distance sensor
  pinMode(A0, INPUT);
  Serial.println("Black-level calibration Starting soon, please make sure sensors are on a black line.");
  delay(1000);
  Serial.println("Starting in 3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Black-level calibration has begun");
  for (uint8_t i = 0; i < cal; i++)
  {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Black-level calibration is done.");
  Start = 1;
  previousMillis = micros();
  Timer1.initialize(500000); // set a timer of length 500000 microseconds
  //attach and define service routines on timer1 and pin2
  attachInterrupt(digitalPinToInterrupt(2), isrforward, CHANGE);
}
void forward();
void reverse();
void reverse(int);
void PivotLeft();
void PivotLeft(int x);
void PivotRight();
void cmReverse(int);
void cmReverse(int, int);
void cmForward(int);
void brakeLOW();
void coast();
void TurnLeft();
void TurnRight();
void measure();
void enBackwards();
//void lineFollowReverse();
double distSens();
void moveLine(int x);
void enForward();
int saber();
int SColor = 0;
void lineFollow();
void lineFollow(int x, int y);


void loop() {
  //forward();
  //lineFollow();
  //   SColor = saber();
  //   Serial.println(SColor);
  //   delay(3000);

  //
  //
  //TEST CODE IN HERE

  //    while(1){
  //      dist=distSens();
  //      Serial.print("Distance cm: \t");
  //      Serial.println(dist);
  //    }

  //TEST CODE ABOVE
  //
  //
  //
  if (Start == 1) {

    unsigned long currentMillis = micros();
    //line follow forward
    enForward();
    while (micros() - currentMillis < 14000000) {
      lineFollow();
      //      if((micros() - currentMillis > 7000000)){
      //        while(micros()-currentMillis < 10000000){
      //          lineFollow(20,40);
      //        }
      //}

    }
    Serial.println("before dist");
    dist = distSens();
    while (dist > 10.5) {
      lineFollow(30, 60);
      dist = distSens();
    }
    Serial.println("after dist");
    if (dist <= distWall + 10 && firstStop == 0) // first stop
    {
      Start == 0;
      coast();
      brakeLOW();
      //stepper motor stuff
      delay(200); //delete later for stepper motor stuff

      //Serial.print("Stop 2 Dist: ");
      //Serial.println(dist);
      firstStop = 1;
      secondStop = 1;
      //reverse for second stop
      //reverse();
      //delay(750);
      enBackwards();
      dist = distSens();
      while (dist < 20) {
        lineFollow(30, 60);
        dist = distSens();
      }
      coast();
      brakeLOW();
      //stepper motor
      delay(100);
      PivotLeft(90);                                                      //Pivots Left after reversing from the wall (measured by distance sensor)
      delay(100);                                                       //Should be facing outwards
      Start = 0;
      cmForward(50);                                                    //moving forward into lightsaber territory
      //line follow for a second
      //currentMillis = micros();
      //    while (micros() - currentMillis < 1500000) {
      //      lineFollow();
      //    }
      delay(100);
      PivotLeft();                                                      //Pivots Left after reaching the center of the 5 light sabers
      delay(100); //300 before
      TurnRight();
      delay(60);
      //TurnLeft();                                                       //Turns left to try and reach the first light saber
      //delay(50);
      brakeLOW();
      cmForward(15);
      // count number of black lines till end
      moveLine(2); //move three black lines over
      delay(100);
      cmReverse(1);                                                     //Should be on last black line here. Reverses 1 to align itself. Not oriented yet
      PivotRight();                                                     //Should be oriented to the line now.
      delay(100);
      while (blackCount < 5) {
        dist = distSens();
        //Serial.print("dist to saber 1: ");                              //test prints for distance
        //Serial.print(dist);
        // line follow forward
        enForward();
        while (dist > 40) {
          lineFollow(40, 60);
          dist = distSens();
        }
        while (dist > 30) {
          lineFollow(30, 50);
          dist = distSens();
        }
        while (dist > 20) {
          lineFollow(20, 40);                                             //testing slower speed
          dist = distSens();
        }
        brakeLOW();
        delay(100);
        for (int i = 0; i < 4; i++) {
          SColor = saber();
          dist = distSens();
        }
        if (dist > 35) {
          cmForward(1);
          delay(100);
          dist = distSens();
        }
        if (SColor == 0) { //red
          cmForward(dist / 2);
          dist = distSens();
          while (dist > 20) {
            cmForward(1);
            dist = distSens();
          }
          cmForward(dist - 1);
          //forward();
          coast();
          delay(100);
          enBackwards();
          currentMillis = micros();
          while (micros() - currentMillis < 1000000) {
            lineFollow(70, 100, 0.052, 10);
          }
          while (dist < 25) {
            lineFollow(65, 100);
            dist = distSens();
          }
          while (dist > 35) {
            cmReverse(2);
            enBackwards();
            dist = distSens();
          }
          while (dist < 40)
          {
            lineFollow(60, 80, 0.052, 10);
            dist = distSens();
          }
          redC++;
        }
        else if (SColor == 1) { //green
          dist = 5; //random
          enBackwards();
          currentMillis = micros();
          while (micros() - currentMillis < 1000000) {
            lineFollow(70, 100, 0.052, 10);
          }

          while (dist < 25) {
            lineFollow(65, 100);
            dist = distSens();
          }
          while (dist > 40) {
            cmReverse(1, 30);
            enBackwards();
            dist = distSens();
          }
          while (dist < 40) {
            lineFollow(60, 80, 0.052, 10);
            dist = distSens();
          }
          greenC++;
        }
        if(redC == 3){
          blackCount = 5;
        }
        if (blackCount <= 3) {
          brakeLOW();
          delay(100);
          while (dist < 40) {
            lineFollow(60, 100, 0.052, 10);
            dist = distSens();
          }
          delay(100);
          PivotRight();
          cmForward(10);
          moveLine(1);
          delay(100);
          cmReverse(2);
          BLineCount = 0;
          PivotLeft(70);
          enBackwards();
          dist = distSens();
          while (dist < 25) {
            lineFollow();
            dist = distSens();
          }
          cmReverse(1, 30);
          enBackwards();
          dist = distSens();
          while (dist < 40) {                 //was if previously
            lineFollow(50, 80);
            dist = distSens();
          }
        }


        blackCount++;
      }
      coast();
      brakeLOW;
      delay(500);
      dist = distSens();
      enBackwards();
      while (dist < 25) {
        lineFollow();
        dist = distSens();
      }
      while (dist < 40) {
        lineFollow(50, 100);
        dist = distSens();
      }
      if (redC == 3 && greenC == 0) {
        PivotLeft();
        PivotLeft();
      } else if (redC == 3 && greenC == 1) {
        PivotLeft(60);;
        cmForward(15);
        moveLine(1);
        cmReverse(2);
        PivotLeft(70);
      } else {

        PivotLeft(60);                                                      //Pivots Left after reaching the center of the 5 light sabers
        TurnRight();
        cmForward(5);
        moveLine(2);
        cmReverse(2);
        PivotLeft(70);
      }
      ////////////////////////////////////////ESCAPE ROUTE
      dist = distSens();
      enForward();
      currentMillis = micros();
      while (micros() - currentMillis < 2000000) {
        // lineFollow(20, 40);
        lineFollow();
      }
      //cmForward(20);

      //coast();
      //brakeLOW();
      //moveLine(1);
      moveLineEnd();
      cmReverse(2);
      PivotRight();
      brakeLOW();

      //      enBackwards();
      //      if(dist <50){
      //        lineFollow();
      //        dist=distSens();
      //      }
    }
  }
}
void enForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void enBackwards() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void lineFollow() {
  // unsigned int linePos=0;
  unsigned int linePos = qtr.readLineBlack(blackSens);
  for (int j = 0; j < 8; j++) {
    unsigned int positionB = qtr.readLineBlack(blackSens);

    if (blackSens[j] < thresholdW) {
      digitalWrite(j + 46, HIGH);
    } else {
      digitalWrite(j + 46, LOW);
    }
    //linePos+=blackSens[j];
  }
  // Compute the error
  error = SETPOINT - linePos;
  // Compute the motor adjustment
  adjust = error * KP + KD * (error - lastError);

  // Record the current error for the next iteration
  lastError = error;

  // Adjust motors, one negatively and one positively
  analogWrite(ENB, constrain(SET_SPEED - adjust, MIN_SPEED, MAX_SPEED));
  analogWrite(ENA, constrain(SET_SPEED + adjust, MIN_SPEED, MAX_SPEED));
  //Start = 1;
}

void lineFollow(int x, int y) {
  // unsigned int linePos=0;
  unsigned int linePos = qtr.readLineBlack(blackSens);
  for (int j = 0; j < 8; j++) {
    unsigned int positionB = qtr.readLineBlack(blackSens);

    if (blackSens[j] < thresholdW) {
      digitalWrite(j + 46, HIGH);
    } else {
      digitalWrite(j + 46, LOW);
    }
    //linePos+=blackSens[j];
  }
  // Compute the error
  error = SETPOINT - linePos;
  // Compute the motor adjustment
  adjust = error * KP + KD * (error - lastError);

  // Record the current error for the next iteration
  lastError = error;

  // Adjust motors, one negatively and one positively
  analogWrite(ENB, constrain(x - adjust, MIN_SPEED, y));
  analogWrite(ENA, constrain(x + adjust, MIN_SPEED, y));
  //Start = 1;
}

void lineFollow(int x, int y, float p, float d) {
  // unsigned int linePos=0;
  unsigned int linePos = qtr.readLineBlack(blackSens);
  for (int j = 0; j < 8; j++) {
    unsigned int positionB = qtr.readLineBlack(blackSens);

    if (blackSens[j] < thresholdW) {
      digitalWrite(j + 46, HIGH);
    } else {
      digitalWrite(j + 46, LOW);
    }
    //linePos+=blackSens[j];
  }
  // Compute the error
  error = SETPOINT - linePos;
  // Compute the motor adjustment
  adjust = error * p + d * (error - lastError);

  // Record the current error for the next iteration
  lastError = error;

  // Adjust motors, one negatively and one positively
  analogWrite(ENB, constrain(x - adjust, MIN_SPEED, y));
  analogWrite(ENA, constrain(x + adjust, MIN_SPEED, y));
  //Start = 1;
}

void moveLine(int x) {
  Speed = 80;
  while (BLineCount != x) {
    forward();
    lineFollow();
    if ((digitalRead(48) == LOW ) && (digitalRead(49) == LOW) && (digitalRead(50) == LOW) && (digitalRead(51) == LOW )) {
      flagB = 1;

    }
    if (flagB == 1 && digitalRead(49) == HIGH && digitalRead(50) == HIGH && flagB == 1) {
      BLineCount ++;
      flagB = 0;
    }
  }
  BLineCount = 0;
  coast();
  brakeLOW();
  Speed = 80;
}

void moveLineEnd() {
  Speed = 80;
  while (BLineCount != 1) {
    forward();
    for (int j = 0; j < 8; j++) {
      unsigned int positionB = qtr.readLineBlack(blackSens);

      if (blackSens[j] < thresholdW) {
        digitalWrite(j + 46, HIGH);
      } else {
        digitalWrite(j + 46, LOW);
      }
    }
    while (flagB == 0) {            //delete if doesnt work
      lineFollow();
      if ((digitalRead(48) == HIGH ) && (digitalRead(49) == HIGH) && (digitalRead(50) == HIGH) && (digitalRead(51) == HIGH )) {
        flagB = 1;
        forward();
        unsigned int linePos = qtr.readLineBlack(blackSens);
        for (int j = 0; j < 8; j++) {
          unsigned int positionB = qtr.readLineBlack(blackSens);

          if (blackSens[j] < thresholdW) {
            digitalWrite(j + 46, HIGH);
          } else {
            digitalWrite(j + 46, LOW);
          }
        }
      }
    }
    if (flagB == 1 && digitalRead(49) == LOW && digitalRead(50) == LOW) {
      BLineCount ++;
      flagB = 0;
    }
  }
  BLineCount = 0;
  coast();
  brakeLOW();
  Speed = 80;
}
int saber() {
  uint16_t r, g, b, c, colorTemp, lux, red, green;
  delay(100);
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  redgreen = 0;

  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r); Serial.print(" ");
  Serial.print("G: "); Serial.print(g); Serial.print(" ");
  //Serial.print("B: "); Serial.print(b); Serial.print(" ");
  Serial.print("C: "); Serial.print(c); Serial.print(" ");
  //Serial.println(" ");

  redgreen = (r - g);

  Serial.print("redgreen value: \t");
  Serial.println(redgreen);
  if (redgreen >= 13) {
    Serial.println("Current Saber: Red  ");
    return 0;
  } else if (redgreen < 12) {
    Serial.println("Current Saber: Green  ");
    return 1;
  }
  else
  {
    Serial.println("Current Saber: Green, Right Saber: Red   "); //this is unsure territory. Might have it linefollow until it has an accurate reading
  }
}

void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}
void coast() {
  analogWrite(ENA, LOW);
  analogWrite(ENB, LOW);
}

void brakeLOW() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, HIGH);
  analogWrite(ENB, HIGH);
}

void reverse() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}
void reverse(int y) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, y);
  analogWrite(ENB, y);
}

void TurnLeft() {
  analogWrite(ENA, LOW);
  analogWrite(ENB, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

void TurnRight() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, LOW);

  digitalWrite(IN1, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
}

void PivotLeft() {
  coast();
  brakeLOW();
  delay(200);

  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(675);
  brakeLOW();

}
void PivotLeft(int x) {
  coast();
  brakeLOW();
  delay(200);

  analogWrite(ENA, x);
  analogWrite(ENB, x);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(675);
  brakeLOW();

}


void PivotRight() {
  coast();
  brakeLOW();
  delay(200);

  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(660);
  coast();
  brakeLOW();
}
void isrforward() {
  pulse++;
}

void cmForward(int x) {
  pulse = 0;
  forward();
  distcm = cmdistance / conversionVar;
  while (pulse < 1788 / 18.85 * x) {

    //18.85cm per rotation
    if (pulse > 1788 / 18.85 * x) {
      coast();
      brakeLOW();
      distcm = pulse * conversionVar;
    }
  }

}

void cmReverse(int x) {
  pulse = 0;
  reverse();
  distcm = cmdistance / conversionVar;
  while (pulse < 1788 / 18.85 * x) {

    Serial.println(pulse);
    if (pulse > 1788 / 18.85 * x) {
      coast();
      brakeLOW();
      distcm = pulse * conversionVar;
    }
  }

}
void cmReverse(int x, int y) {
  pulse = 0;
  reverse(y);
  distcm = cmdistance / conversionVar;
  while (pulse < 1788 / 18.85 * x) {

    Serial.println(pulse);
    if (pulse > 1788 / 18.85 * x) {
      coast();
      brakeLOW();
      distcm = pulse * conversionVar;
    }
  }

}
void measure() {
  pulse = 0;
  Serial.println("measure");
  delay(2000);
}

double distSens() {

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // Displays the distance on the Serial Monitor
  //  Serial.print("Distance: ");
  //  Serial.print(distance);
  //  Serial.println(" cm");
  return distance;
}
/*
  double distSens() {
  voltage = (double(analogRead(A0)) * 5 / 1024);
  //else-if statements to iterate through the levels of voltage with differing
  //slope equations. Outputs distance according to corresponding equation.
  if (voltage >= 2.94) {
    dist = (voltage - 3.28) / (-0.043);
  }
  else if (voltage >= 2.91) {
    dist = (voltage - 3.18) / (-0.03);
  }
  else if (voltage >= 2.38) {
    dist = (voltage - 7.68) / (-0.53);
  }
  else if (voltage >= 1.76) {
    dist = (voltage - 3.62) / (-0.124);
  }
  else if (voltage >= 1.31) {
    dist = (voltage - 3.11) / (-0.09);
  }
  else if (voltage >= 1.09) {
    dist = (voltage - 2.19) / (-0.044);
  }
  else if (voltage >= 0.92) {
    dist = (voltage - 1.94) / (-0.034);
  }
  else if (voltage >= 0.8) {
    dist = (voltage - 1.64) / (-0.024);
  }
  else if (voltage >= 0.73) {
    dist = (voltage - 1.62) / (-0.023);
  }
  else if (voltage >= 0.71) {
    dist = (voltage - 1.11) / (-0.01);
  }
  else {
    dist = (voltage - 1.51) / (-0.02);
  }

  return dist;
  }
*/
