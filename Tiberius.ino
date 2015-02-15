#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Sensor pins from left to right. indices 0 to 4:
int SensorPins[] = {0, 1, 2, 3, 4};

// Sensor values from left to right, indices 0 to 4:
int SensorValues[5];

// Sensor threshold table:
int SensorTable[5];

// Sensor flags
bool Sensors[] = {0,0,0,0,0};
bool FLSV = false;
bool LSV  = false;
bool MSV = false;
bool RSV = false;
bool FRSV = false;

// LED light
int ledPin = 13;

// Motors
Adafruit_DCMotor *RightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(2);

// Function prototypes  
void updateSensors();     // Updates sensor boolean values
void refreshSensors();    // gets analog values from sensors
void calibrateSensors();  // Calibrates sensors on runtime

// write a function that refreshes them every tick
// since those values are global, it would just reread the data 

//------------------------------------------------------
// FORWARD
// RightMotor->run(FORWARD);
// LeftMotor->run(FORWARD);

// STOP
// initiate a for loop that slows down
// right stops
// left stops
// if this is just a gap in the line, then at some point it'll cross back over black.
// So then at this point stop the stopping algorithm and switch to forward.

// SLIGHT TURN RIGHT
// change speed of right motor in half??
// left motor stays forward

// HARD TURN RIGHT
// right motor stops
// left motor stays forward

// SLIGHT TURN LEFT
// change speed of left motor in half??
// right motor stays forward

// HARD TURN LEFT
// left motor stops
// right motor stays forward
//------------------------------------------------------

// Setup is what starts up when the code is refreshed. So this will be the calibration
void setup() {
  Serial.begin(9600);  // Open a serial communications channel for debuf info
  Serial.println("Tiberius coming online!");
  calibrateSensors();
}

void loop() {
  updateSensors();
  // heyo this is where the main code goes isn't this cool

  
  // Main comparisons
  if (((LSV != MSV) && (RSV != MSV)) || ((FLSV != MSV) && (FRSV != MSV))) {
    // ON LINE. MOVE FORWARD FAST
    // move left motor forward
    // move right motor forward
    RightMotor->run(FORWARD);
    LeftMotor->run(FORWARD);
    Serial.println("Onward!");
  }
  else {
    Serial.println("Where are we going again?");
  }
}

void updateSensors(){
  refreshSensors();   // get the new analog values from each sensor
  for (int i = 0; i < 5; i++){
    if (SensorValues[i] < SensorTable[i]){
      Sensors[i] = false;
    } else {
      Sensors[i] = true;
    }
  }
  // Translate sensors array to independent bools:
  bool FLSV = Sensors[0];
  bool LSV  = Sensors[1];
  bool MSV = Sensors[2];
  bool RSV = Sensors[3];
  bool FRSV = Sensors[4];
}

void refreshSensors(){
  for (int i = 0; i < 5; i++){
    SensorValues[i] = analogRead(SensorPins[i]);
  }
}

void calibrateSensors(){
  //------------------------------------------------------
  // Calibration
  // --give first signal that bot is on
  // --give signal to calibrate for white
  //    read in voltage for white and store as variable
  // --give signal to calibrate for black
  //    read in voltage for black and store as variable
  //------------------------------------------------------
  // convert the sensor values to boolean flags
  refreshSensors();
  int low[5];
  for (int i = 0; i < 5; i++){
    low[i] = SensorValues[i];  
  }
  delay(5000);           // Wait five seconds for car to be moved from black line
  refreshSensors();
  int high[5];
  for (int i = 0; i < 5; i++){
    high[i] = SensorValues[i];  
  }  
  for (int i = 0; i < 5; i++){ // Take the averages of all the values and store in thresholds table
    SensorTable[i] = ( low[i] + high[i] ) / 2;
    Serial.print("The threshold of sensor ");
    Serial.print(i);
    Serial.print(" is ");
    Serial.println(SensorTable[i]);
  }
  // Maybe we should add a buzzer that beeps once the values have been taken?
}

/* CODE TO BUILD OFF OF...
main
   
    ' Default Process                                                                         
    
    if (((lsensor <> msensor) and (rsensor <> msensor)) or ((lsensor <> Fsensor) and (rsensor <> Fsensor))) then     
        'On line, move forward fast
        lmf = 1
        lmb = 0
        rmf = 1
        rmb = 0
        online = 1 
        mustturn = 0
    else
        'Save sensors state at the begining
        if (online = 1) then
            lls = lsensor
            lrs = rsensor
            lms = msensor
            online = 0
        endif
        if (mustturn = 1) then
            if (mustturnleft = 1) then
                lmf = 0
                lmb = FastRotate
                rmf = 1
                rmb = 0
            else
                lmf = 1
                lmb = 0
                rmf = 0
                rmb = FastRotate
            endif               
        else                           
            if ((lsensor <> lls) or (rsensor <> lrs)) then     
                'It's not a damage in path, I really lost the path
                mustturn = 1
                if (Rsensor <> lrs) then mustturnleft = 0 
                if (Lsensor <> lls) then mustturnleft = 1
                'FastRotate = 0
                'if (MSensor = lms) then FastRotate = 1
            endif  
        endif        
    endif
}*/
