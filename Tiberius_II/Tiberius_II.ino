// Authors: Kathryn DiPippo, Mark Blanco and Liam McEneaney
// 15 February 2015
// Tiberius II, the line-following robot
// IED Section 3 Line Follower Mini-Project
//  Based on sample code provided by Pololu.com and Hazim Bitar/Techbitar

#include <Adafruit_MotorShield.h>
#include <QTRSensors.h>
#include <Wire.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield mShield = Adafruit_MotorShield(); 
// Motors
Adafruit_DCMotor *motor1 = mShield.getMotor(4);    // Left motor in our physical implementation
Adafruit_DCMotor *motor2 = mShield.getMotor(3);    // Right motor (this one was faster)


// Change the values below to suit your robot's motors, weight, wheel type, etc.
#define KP .075
#define KD 0.1
#define M1_DEFAULT_SPEED 120
#define M2_DEFAULT_SPEED 118
#define M1_MAX_SPEED 228
#define M2_MAX_SPEED 230
#define NUM_SENSORS  8     // Number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   9     // emitter is controlled by digital pin 8
#define DEBUG 1             // set to 1 if serial debug output needed

QTRSensorsRC qtrrc((unsigned char[]) {3,4,5,6,7,8,9,10} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

// Prototypes:
int calculateError();  // Returns calculated error


void setup()
{
  if (DEBUG){
    Serial.begin(9600);
  }
  mShield.begin();
  delay(1000);
  manual_calibration(); 
  set_motors(0,0);
}

int lastError = 0;
int last_position = 0;

void loop()
{
  //int position = qtrrc.readLine(sensors);  // <-- MAYBE USE readCalibrated() instead...  
  //int error = position - 3500;    // Adjusted the value from -2000 to -2500 to change sensor middle
  int error = calculateError();
  /*if (abs(last_position - position) > 3000){    // If we just ran off the line, go straight.
    error = 0;
  }*/
  if (DEBUG) {
    //Serial.print(position);
    //Serial.print(' ');
    Serial.println(error);
  }
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = M1_DEFAULT_SPEED + motorSpeed;
  int rightMotorSpeed = M2_DEFAULT_SPEED - motorSpeed;

  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);
  //last_position = position;
}

int calculateError(){
  static bool left = 0;
  static bool right = 0;
  unsigned int sensors[5];
  unsigned long avg = 0;
  unsigned int sum = 0;
  qtrrc.readCalibrated(sensors);
  int position = 0;
  int error = 0;
  
  for(unsigned char i = 0; i < NUM_SENSORS; i++) {
      int value = sensors[i];
      // only average in values that are above a noise threshold
      if(value > 50) {
          avg += (long)(value) * (i * 1000);
          sum += value;
      }
  }

  if (sum == 0){// If the line has been lost, go straight
    if (right && left){
      error = 0;
    }
    else if (right){
      error = 3500;
    } else {
      error = -3500;
    }
  } else {
    position = avg/sum;
    error =  position - 3500;
  }
  
  if (sensors[0] > 50){  // Save old left/right values for next scan
    left = 1;
  } else {
    left = 0;
  }
  if (sensors[7] > 50){
    right = 1;
  } else {
    right = 0;
  }
  return error;
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  /*if (motor1speed < 0 && motor2speed < 0){  // If both motors are set to go backwards, set them to default
    motor1speed = M1_DEFAULT_SPEED;
    motor2speed = M2_DEFAULT_SPEED;
  }
  if (motor1speed < 0) {  //  Go backwards
    motor1speed = abs(motor1speed)*0.25;
    motor1->setSpeed(motor1speed);    // set motor speed
    motor1->run(BACKWARD);  
  } else {                // Else go forwards
    motor1->setSpeed(motor1speed);    // set motor speed
    motor1->run(FORWARD);  
  }
  if (motor2speed < 0) {  //  Go backwards
    motor2speed = abs(motor1speed)*0.25;
    motor2->setSpeed(motor2speed);     // set motor speed
    motor2->run(BACKWARD);  
  } else {                // Else go forwards
    motor2->setSpeed(motor2speed);     // set motor speed
    motor2->run(FORWARD);  
  }*/
  if (motor1speed < 0) {  
    motor1speed = 0;
  }              
  if (motor2speed < 0) {
    motor2speed = 0;  
  }
  motor1->setSpeed(motor1speed);    
  motor2->setSpeed(motor2speed);     // set motor speed
  motor1->run(FORWARD); 
  motor2->run(FORWARD);
  delay(10);
  if (DEBUG) {
    Serial.print("Motor 1 Speed: ");
    Serial.println(motor1speed);
    Serial.print("Motor 2 Speed: ");
    Serial.println(motor2speed);
  }
}


void manual_calibration() {

  int i;
  for (i = 0; i < 130; i++)  // the calibration will take a few seconds
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }

  if (DEBUG) { // if true, generate sensor data via serial output
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
}
