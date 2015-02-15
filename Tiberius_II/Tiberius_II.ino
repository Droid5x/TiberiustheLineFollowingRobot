//  Remixed by Hazim Bitar/Techbitar
//  Date:Feb 12, 2012
//  Based on sample code provided by Pololu.com
//  Contact: techbitar at gmail dot com


#include <Adafruit_MotorShield.h>
#include <QTRSensors.h>
<<<<<<< HEAD
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield mShield = Adafruit_MotorShield(); 
// Motors
Adafruit_DCMotor *RightMotor = mShield.getMotor(1);
Adafruit_DCMotor *LeftMotor = mSheild.getMotor(2);
=======

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// AF_DCMotor motor1(1, MOTOR12_8KHZ ); // PIN 11 - create motor #1 pwm
// AF_DCMotor motor2(2, MOTOR12_8KHZ ); // PIN 3 - create motor #2 pwm
>>>>>>> FETCH_HEAD

// Change the values below to suit your robot's motors, weight, wheel type, etc.
#define KP .2
#define KD 5
#define M1_DEFAULT_SPEED 50
#define M2_DEFAULT_SPEED 50
#define M1_MAX_SPEED 70
#define M2_MAX_SPEED 70
#define MIDDLE_SENSOR 3
#define NUM_SENSORS  6      // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   8     // emitter is controlled by digital pin 8
#define DEBUG 1             // set to 1 if serial debug output needed

QTRSensorsRC qtrrc((unsigned char[]) {  2,3,4,5,6,7} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  delay(1000);
  manual_calibration(); 
  set_motors(0,0);
}

int lastError = 0;
int  last_proportional = 0;
int integral = 0;


void loop()
{
  unsigned int sensors[5];
  int position = qtrrc.readLine(sensors);
  int error = position - 2000;

  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = M1_DEFAULT_SPEED + motorSpeed;
  int rightMotorSpeed = M2_DEFAULT_SPEED - motorSpeed;

  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  if (motor1speed < 0) motor1speed = 0; // keep motor above 0
  if (motor2speed < 0) motor2speed = 0; // keep motor speed above 0
  motor1.setSpeed(motor1speed);     // set motor speed
  motor2.setSpeed(motor2speed);     // set motor speed
  motor1.run(FORWARD);  
  motor2.run(FORWARD);
}


void manual_calibration() {

  int i;
  for (i = 0; i < 250; i++)  // the calibration will take a few seconds
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }

  if (DEBUG) { // if true, generate sensor dats via serial output
    Serial.begin(9600);
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
