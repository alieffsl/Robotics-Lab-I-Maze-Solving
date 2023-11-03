#include <SparkFun_TB6612.h> //TB6612 Motor Driver
#include <HCSR04.h> //Sensor Ultrasonic
#include "Adafruit_TCS34725.h"


uint16_t r, g, b, c;
const int buttonPin = 38;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//HCSR04(trig, echo)
HCSR04 depan(22, 24);
HCSR04 kiri(48, 50);
HCSR04 kanan(5, 6);

//Driver motor
#define AIN1 11
#define BIN1 9
#define AIN2 12
#define BIN2 8
#define PWMA 13
#define PWMB 7
#define STBY 10
#define port_buzzer 32

const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
Motor motor2 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

int motor_drive(int speedM1, int speedM2) {
  motor1.drive(speedM1);
  motor2.drive(speedM2);
}

int motorDelay(int speedl, int speedr, int dtime) {
  motor1.drive(speedl);
  motor2.drive(speedr);
  delay(dtime);
}

bool isRobotStarted = false;
int disDe, disKi, disKa;
float err = 0, integral = 0, derivative = 0, lastErr = 0;
int speedL = 0, speedR = 0, steering = 0;

//REF
const int speedRef = 100, distRef = 8;

// PID Constants
const float Kp = 4;
const float Ki = 0;
const float Kd = 2;

enum WallFollowMode {
  LEFT_WALL_FOLLOW,
  RIGHT_WALL_FOLLOW
};

WallFollowMode currentMode = LEFT_WALL_FOLLOW; // Start with left wall following

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(port_buzzer, OUTPUT);
  Serial.begin(9600);
  buzzer(3, 50, 50);
}

void loop()
{
  //Baca Jarak Sensor
//  disDe = depan.dist();
//  disKi = kiri.dist();
//  disKa = kanan.dist();

  if (!isRobotStarted && digitalRead(buttonPin) == LOW) {
    isRobotStarted = true;
    buzzer(1, 50, 50);
    delay(100);
    while (digitalRead(buttonPin) == LOW) {}
  }

  if (!isRobotStarted) {
    return;
  }
    disDe = depan.dist();
  disKi = kiri.dist();
  disKa = kanan.dist();

  if (currentMode == LEFT_WALL_FOLLOW) {
    leftWallFollowing();
  } else if (currentMode == RIGHT_WALL_FOLLOW) {
    rightWallFollowing();
  }

  //  Serial.print("Speed Kiri: "); Serial.print(speedL);
  //  Serial.print("  Speed kanan: "); Serial.print(speedR);
  //
  //  Serial.print("  Dist Kanan: "); Serial.print(disKa);
  //  Serial.print("  Dist Kiri: "); Serial.println(disKi);
}

void leftWallFollowing()
{
  err = disKi - distRef;
  integral += err;
  derivative = err - lastErr;
  lastErr = err;

  steering = Kp * err + Ki * integral + Kd * derivative;

  speedL = speedRef - steering;
  speedR = speedRef + steering;

  speedL = constrain(speedL, -speedRef, speedRef);
  speedR = constrain(speedR, -speedRef, speedRef);

  if (disDe < 8 && disDe > 0) {
    buzzer(2, 100, 50);
    motorDelay(0, 0, 100);
    motorDelay(-90, -90, 300);
    motorDelay(100, 0, 850);
    motorDelay(50,100,50);

  }   
  else if (disKi > 50 && disDe < 9) {
    buzzer(3, 50, 10);
    motorDelay(0, 70, 170);
//    motorDelay(0,0,1000);
  }
  else if (disKi > 50) {
    motorDelay(30, 150, 250); //30 150 250
    buzzer(1, 10, 10);
  }
  else {
    motor_drive(speedL, speedR);
  }

  // Check if it's time to switch to the right wall following mode
  if (isTimeToSwitchMode(16250)) {
    motorDelay(0, 0, 1500);
    buzzer(1, 1000, 50);
    motorDelay(100, 60, 650);
    currentMode = RIGHT_WALL_FOLLOW;
    resetStartTime();
  }
}

void rightWallFollowing()
{
  err = disKa - distRef;
  integral += err;
  derivative = err - lastErr;
  lastErr = err;

  steering = Kp * err + Ki * integral + Kd * derivative;

  speedL = speedRef + steering;
  speedR = speedRef - steering;

  speedL = constrain(speedL, -speedRef, speedRef);
  speedR = constrain(speedR, -speedRef, speedRef);

  if (disDe < 8 && disDe > 0) {
    buzzer(2, 100, 50);
    motorDelay(0, 0, 100);
    motorDelay(-90, -90, 300);
    motorDelay(0, 100, 850);

  } else if (disKa > 50) {
    motorDelay(150, 30, 250);
    buzzer(1, 10, 10);

  } else {
    motor_drive(speedL, speedR);
  }
  // Check if it's time to switch to the left wall following mode
  if (isTimeToSwitchMode(100000)) {
    motorDelay(0, 0, 1500);
    buzzer(1, 1000, 50);
    motorDelay(60, 100, 400);
    currentMode = LEFT_WALL_FOLLOW;
    resetStartTime();
  }

  tcs.getRawData(&r, &g, &b, &c);
  if (r > 200 && g > 80 && b > 80 && g <= r && b <= r)
  {
    motorDelay(0, 0, 1000);
    buzzer(10, 50, 50);
    motorDelay(0, 0, 9000);
  }
}

unsigned long startTime = 0;
const unsigned long leftWallFollowingTime = 5000;
const unsigned long rightWallFollowingTime = 5000;

bool isTimeToSwitchMode(unsigned long modeTime)
{
  unsigned long elapsedTime = millis() - startTime;
  return elapsedTime >= modeTime;
}

void resetStartTime()
{
  startTime = millis();
}

void buzzer(int num, int ton, int toff)
{
  for (int i = 0; i < num; i++)
  {
    digitalWrite(port_buzzer, HIGH);
    delay(ton);
    digitalWrite(port_buzzer, LOW);
    delay(toff);
  }
}
