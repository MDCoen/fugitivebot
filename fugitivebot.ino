/*
   Created By Cameron Rogers, Temi Balogun, and Mauricio Coen
   for Dr. Kim
   MEEN 667

*/
/* Include needed libraries */
#include <Wire.h> //I2C Arduino library
#include <NewPing.h>  //new ping library
#include "TimerThree.h" //interupt library

/* Define sampling time and control gains */
#define T 0.1   //sampling time in seconds
#define Kp 90  //perportional control
#define Ki .00  //integral control

/* Define motor pins */
#define enablePin1 3   //enable pin for motor 1
#define enablePin2 6   //enable pin for motor 2 
#define motor11Pin 4    //control pin 1 for motor 1
#define motor12Pin 5    //control pin 2 for motor 1
#define motor21Pin 7   //control pin 1 for motor 2
#define motor22Pin 8   //control pin 2 for motor 2

/* Define address for reading the heading */
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

/* Define ultrasound paramaters*/
#define SONAR_NUM     8 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

/* Create global variables */
int curH = 0;
int desH = 0;
float e = 0;
float eP = 0;
float u = 0;
float uP = 0;
int pwml = 165;
int pwmr = 165;
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(30, 31, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(32, 33, MAX_DISTANCE),
  NewPing(34, 35, MAX_DISTANCE),
  NewPing(36, 37, MAX_DISTANCE),
  NewPing(38, 39, MAX_DISTANCE),
  NewPing(40, 41, MAX_DISTANCE),
  NewPing(42, 43, MAX_DISTANCE),
  NewPing(44, 45, MAX_DISTANCE)
};

void setup() {
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();

  /* Put the HMC5883 IC into the correct operating mode */
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  /* Set motor pins as outputs */
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(motor11Pin, OUTPUT);
  pinMode(motor12Pin, OUTPUT);
  pinMode(motor21Pin, OUTPUT);
  pinMode(motor22Pin, OUTPUT);

  /* Set forward direction */
  digitalWrite(motor11Pin, LOW);
  digitalWrite(motor12Pin, HIGH);
  digitalWrite(motor21Pin, HIGH);
  digitalWrite(motor22Pin, LOW);

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) {   // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  /* Initialize timer3 */
  //  Timer3.initialize(250000);
  //  Timer3.attachInterrupt(interruptExecution);
}

void loop() {
  getHeading();
  calculateControl();
  //  Serial.print("Sine of heading error ");
  //  Serial.println(e);
  //  Serial.print("Control ");
  //  Serial.println(round(u));
  executeControl();
  updateParams();
  delay(100);
}

void interruptExecution() {
  deshUpdate();
  getHeading();
  calculateControl();
  executeControl();
  updateParams();
}

void calculateControl() {
  e = sin((desH - curH) * M_PI / 180);
  //  u = uP + (Kp+0.5*Ki*T)*e + (0.5*Kp*T+Ki)*eP;
  u  = Kp * e;
}

void updateParams() {
  uP = u;
  eP = e;
}

void executeControl() {
  changeSpeed(1, pwml + u);
  changeSpeed(2, pwmr - u);
}

void getHeading() {
  int x, y, z; //triple axis data
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read() << 8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read() << 8; //Y msb
    y |= Wire.read(); //Y lsb
  }

  //Print out values of each axis
  //  Serial.print("x: ");
  //  Serial.print(x);
  //  Serial.print("  y: ");
  //  Serial.print(y);
  //  Serial.print("  z: ");
  //  Serial.println(z);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(y, x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.05468;
  heading += declinationAngle;

  //  // Correct for when signs are reversed.
  //  if(heading < 0)
  //    heading += 2*PI;
  //
  //  // Check for wrap due to addition of declination.
  //  if(heading > 2*PI)
  //    heading -= 2*PI;

  // Convert radians to degrees for readability.
  curH = heading * 180 / M_PI;
  //  Serial.println(headingDegrees);
  //  return headingDegrees;
}

void changeSpinDirection(int wheel, bool pin1High) {
  // left wheel (1), right wheel (2)
  if (wheel == 1) {
    if (pin1High) {
      digitalWrite(motor11Pin, LOW);
      digitalWrite(motor12Pin, HIGH);
    }
    else {
      digitalWrite(motor11Pin, HIGH);
      digitalWrite(motor12Pin, LOW);
    }
  }
  else {
    if (pin1High) {
      digitalWrite(motor21Pin, LOW);
      digitalWrite(motor22Pin, HIGH);
    }
    else {
      digitalWrite(motor21Pin, HIGH);
      digitalWrite(motor22Pin, LOW);
    }
  }
}

void changeSpeed(int wheel, int pwm) {
  if (wheel == 1) {
    analogWrite(enablePin1, pwm);
  }
  else {
    analogWrite(enablePin2, pwm);
  }
}

void getUltrasound() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}

void deshUpdate() {
  getUltrasound();
  /* This is where the heading logic is added */
}

