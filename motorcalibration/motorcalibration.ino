#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create Motor Shield Object:
Adafruit_MotorShield motor_shield = Adafruit_MotorShield(); 
// Create an object for each motor:
Adafruit_DCMotor *Motor1 = motor_shield.getMotor(4);	// Left
Adafruit_DCMotor *Motor2 = motor_shield.getMotor(3); // Right

// Motor Speed Variable:
int motor1_speed = 0;
int motor2_speed = 0;

void setup() {
  Serial.begin(9600); 		// Start serial connection at 9600 baud
  motor_shield.begin();		// At default frequency of 1.6 KHz

}

void loop(){
  if (Serial.read() == 's'){
    motor1_speed -= 10;
  }
  delay(1);
  if (Serial.read() == 'w'){
    motor1_speed +=10;
  }
  if (Serial.read() == 'e'){
    motor2_speed += 10;
  }
  if (Serial.read() == 'd'){
    motor2_speed -= 10;
  }
  Serial.print(motor1_speed);
  Serial.print(' ');
  Serial.println(motor2_speed);
  Motor1->setSpeed(motor1_speed);
  Motor2->setSpeed(motor2_speed);
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  delay(10);
}
