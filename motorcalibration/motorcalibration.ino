#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create Motor Shield Object:
Adafruit_MotorShield motor_shield = Adafruit_MotorShield(); 
// Create an object for each motor:
Adafruit_DCMotor *RightMotor = motor_shield.getMotor(1);	// On port 1
Adafruit_DCMotor *LeftMotor = motor_shield.getMotor(2);		// On port 2

// Motor Speed Variable:
int motor_speed = 0;

void setup() {
  motor_shield.begin();		// At default frequency of 1.6 KHz
  Serial.begin(9600); 		// Start serial connection at 9600 baud
}

void loop(){
  if (Serial.read() == 's'){
    motor_speed -= 10;
  }
  delay(1);
  if (Serial.read() == 'w'){
    motor_speed += 10;
  }
  Serial.println(motor_speed);
  RightMotor->setSpeed(motor_speed);
  LeftMotor->setSpeed(motor_speed);
  RightMotor->run(BACKWARD);
  delay(10);
}
