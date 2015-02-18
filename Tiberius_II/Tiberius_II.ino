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
#define M1_DEFAULT_SPEED 110
#define M2_DEFAULT_SPEED 108
#define M1_MAX_SPEED 228
#define M2_MAX_SPEED 230
#define NUM_SENSORS  8     // Number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   9     // emitter is controlled by digital pin 8
#define DEBUG 0             // set to 1 if serial debug output needed

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
  //int error = position - 3500;    // 3500 represents sensor middle
  int error = calculateError();
  if (DEBUG) {
    Serial.println(error);
  }
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = M1_DEFAULT_SPEED + motorSpeed;
  int rightMotorSpeed = M2_DEFAULT_SPEED - motorSpeed;

  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);
}

int calculateError(){
  static bool last_left = 0;
  static bool last_right = 0;
  bool right = 0;
  bool left = 0;
  unsigned int sensors[5];
  unsigned long avg = 0;
  unsigned int sum = 0;
  int position = 0;
  int error = 0;
  
  qtrrc.readCalibrated(sensors);
  for(unsigned char i = 0; i < NUM_SENSORS; i++) {
      int value = sensors[i];
      // only average in values that are above a noise threshold
      if(value > 50) {
          avg += (long)(value) * (i * 1000);
          sum += value;
      }
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

  if (sum == 0){// If the line has been lost, go straight
    if (last_left != left || last_right != right){  // We lost the path
      if (last_left != left){
        error = -1000;
      } else if (last_right != right){
        error = 1000;
      }
    } else {
      error = 0;
    }
  } else {
    position = avg/sum;
    error =  position - 3500;
  }
  last_left = left;
  last_right = right;
  return error;
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
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
