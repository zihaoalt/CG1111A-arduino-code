#include <MeMCore.h>

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeBuzzer buzzer;
MeLineFollower lineFinder(PORT_2);

#define DECODER_PIN_0 A0 
#define DECODER_PIN_1 A1
#define IRREAD A2 //IR input
#define LDR A3 //LDR input 
#define ULTRASONIC 12 //used to control the input and output

#define TURNING_TIME_MS 700 // The time duration (ms) for turning
#define NUDGE_TURNING_TIME_MS 100 // The time duration for turning slightly
#define motorSpeed 120


#define TIMEOUT 2000 // max microseconds for ultrasonic to wait
#define SPEED_OF_SOUND 340


// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 200 //in milliseconds
// Define time delay before taking another LDR reading
#define LDRWait 10 //in milliseconds

int side =-1; //used to decide the uturn direction 
int ambIR; // used to store the ambientIR 

// Define colour sensor LED pins
int ledArray[] = {0,1,2};
//placeholders for colour detected
int red = 0;
int green = 0;
int blue = 0;
//floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {712.00, 894.00, 819.00};
float blackArray[] = {580.00, 710.00, 540.00};
float greyDiff[] = {137.00, 178.00, 283.00};
char colourStr[3][5] = {"R = ", "G = ", "B = "};

int calibratedColorArray[6][3] = {
  {234.0, 97.0, 93.0}, // red = 0
  {104.0, 239.0, 251.0}, // light blue = 1
  {100.0, 247.0, 177}, // green = 2
  {268, 183, 122}, // orange = 3
  {260, 236, 229}, // pink = 4
  {249, 262, 253}, // white = 5
};


void celebrate() {
  int melody[] = {
  // E4, E4, E4, E4, E4, E4, E4, G4, C4, D4, E4
  660, 660, 660, 660, 660, 660, 660, 784, 523, 587, 660,
  // F4, F4, F4, F4, F4, E4, E4, E4, E4, D4, D4, E4, D4, G4
  698, 698, 698, 698, 698, 660, 660, 660, 660, 587, 587, 660, 587, 784
  };
  int duration[] = {
  250, 250, 500, 250, 250, 500, 250, 250, 250, 250, 500,
  250, 250, 250, 250, 250, 250, 250, 125, 125, 250, 250, 500, 500
  };

  int songlength = sizeof(melody) / sizeof(int);
  for (int i = 0; i < songlength; i++) {
    buzzer.tone(melody[i], duration[i]/2);
    int pause = 1.5 * duration[i];
    delay(pause);
  }
  buzzer.noTone();
}
void stopMotor(int duration) { // stop for fixed time period
  leftMotor.stop();
  rightMotor.stop();
  delay(duration);
}
void moveForwardFixDuration(int duration, int speed) { // moving forward with custom speed and duration
  leftMotor.run(-speed);
  rightMotor.run(speed);
  delay(duration);
  stopMotor(5);
}
void moveForward() { // keeps moving forward
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed);
}
void turnRight() {
  leftMotor.run(-motorSpeed); // Positive: wheel turns clockwise
  rightMotor.run(-motorSpeed); // Positive: wheel turns clockwise
  delay(TURNING_TIME_MS); // Keep turning left for this time duration
  stopMotor(100);
}
void turnLeft() {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(TURNING_TIME_MS);
  stopMotor(100);
}
void uTurn(int side) { //1 if left, -1 if right 
  leftMotor.run(-255 *side);
  rightMotor.run(-255*side);
  delay(600);
  stopMotor(5);
}
void doubleLeftTurn() {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(TURNING_TIME_MS);

  moveForwardFixDuration(660, 255);

  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(TURNING_TIME_MS);

  stopMotor(5);
}
void doubleRightTurn() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(TURNING_TIME_MS);

  moveForwardFixDuration(700, 255);

  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(TURNING_TIME_MS - 20);

  stopMotor(5);
}
void nudgeLeft() {
  int slowerSpeed = 10; // motor speed constant
  leftMotor.run(-slowerSpeed);
  rightMotor.run(motorSpeed);
  delay(NUDGE_TURNING_TIME_MS);
}
void nudgeRight() {
  int slowerSpeed = 20;
  leftMotor.run(-motorSpeed);
  rightMotor.run(slowerSpeed);
  delay(NUDGE_TURNING_TIME_MS);
}

int readIR() {
  digitalWrite(DECODER_PIN_0, LOW);
  digitalWrite(DECODER_PIN_1, LOW);
  delay(5);

  int VOUT = analogRead(IRREAD);
  digitalWrite(DECODER_PIN_0, HIGH);
  return VOUT;
}

int readUltrasonic() {
  int distance = -1;
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(20);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC, INPUT);
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  if (duration > 0) {
  // Serial.print("distance(cm) = ");
  distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;
  // Serial.println(distance);
  }
  else {
  // Serial.println("out of range");
  distance = 50; //out of range distance is 50
  }
  delay(50);
  return distance;
}


void shineLED(int color) {
  if (color == 0) { // red
    digitalWrite(DECODER_PIN_0, HIGH);
    digitalWrite(DECODER_PIN_1, HIGH);
    //Serial.println("Red LED on");
    return;
  } else if (color == 1) { // green
    digitalWrite(DECODER_PIN_0, HIGH);
    digitalWrite(DECODER_PIN_1, LOW);
    //Serial.println("Green LED on");
    return;
  } else if (color == 2) { // blue
    digitalWrite(DECODER_PIN_0, LOW);
    digitalWrite(DECODER_PIN_1, HIGH);
    //Serial.println("Blue LED on");
    return;
  } else {
    // Serial.println("Wrong color passed in to shine, abort and nothing will light up.");
  }
}

void turnOffLED() {
  digitalWrite(DECODER_PIN_0, LOW);
  digitalWrite(DECODER_PIN_1, LOW);
  return;
}

int getAvgReading(int times){ //find the average reading for the requested number of times of scanning LDR
  int total = 0;
  //take the reading as many times as requested and add them up
  for(int i = 0;i < times;i++){
    total += analogRead(LDR);
    delay(LDRWait);
  }
  return total/times; //calculate the average and return it
}

void setupColorSensor(){ // same function from studio
  // Serial.println("Put White Sample For Calibration ...");
  delay(5000);

  for(int i = 0;i<=2;i++){
    shineLED(i);
    delay(RGBWait);
    whiteArray[i] = getAvgReading(5);
    turnOffLED();
    delay(RGBWait);
  }
  Serial.println("Now white array.");
  Serial.println(whiteArray[0]);
  Serial.println(whiteArray[1]);
  Serial.println(whiteArray[2]);

  Serial.println("Put Black Sample For Calibration ...");
  delay(5000); 
  for(int i = 0;i<=2;i++){
    shineLED(i);
    delay(RGBWait);
    blackArray[i] = getAvgReading(5);
    turnOffLED();
    delay(RGBWait);
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }
  // following printing are for calibration reading array value
  Serial.println("Now black array.");
  Serial.println(blackArray[0]);
  Serial.println(blackArray[1]);
  Serial.println(blackArray[2]);
  // Serial.println("Now grey array.");
  // Serial.println(greyDiff[0]);
  // Serial.println(greyDiff[1]);
  // Serial.println(greyDiff[2]);
  // //delay another 5 seconds for getting ready colour objects
  // Serial.println("Colour Sensor Is Ready.");
  delay(5000);
}

int detectColor(){
  //turn on one colour at a time and LDR reads 5 times
  for(int c = 0;c<=2;c++){
    // Serial.print(colourStr[c]);
    shineLED(ledArray[c]);
    delay(RGBWait);
    //get the average of 5 consecutive readings for the current colour and return an
    // average
    colourArray[c] = getAvgReading(5);
    // Serial.println(colourArray[c]);
    colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
    turnOffLED();
    delay(RGBWait);

    // Serial.println(int(colourArray[c])); //show the value for the current colour
    // LED, which corresponds to either the R, G or B of the RGB code
  }

  double minDiff = 32767; // smallest int value
  int color = -1;

  double diff[6];

  for (int i = 0; i < 6; i++) {
    diff[i] = abs(colourArray[0] - calibratedColorArray[i][0]) + abs(colourArray[1] - calibratedColorArray[i][1]) + abs(colourArray[2] - calibratedColorArray[i][2]);
    // diff[i] = sqrt(pow(2, (colourArray[0] - calibratedColorArray[i][0])) + pow(2, (colourArray[1] - calibratedColorArray[i][1])) + pow(2, (colourArray[2] - calibratedColorArray[i][2])));
  }

  for (int j = 0; j < 6; j++) {
    if (minDiff == diff[j]) {
      // Serial.println("Unfortunately, we cannot differentiate this color between two colors ");
      // Serial.println(j);
    }
    if (minDiff > diff[j]) {
      minDiff = diff[j];
      color = j;
    }

  }
  return color;
}





void setup()
{
  pinMode(DECODER_PIN_0, OUTPUT); // LED1
  pinMode(DECODER_PIN_1, OUTPUT); // LED2
  pinMode(LDR, INPUT); // LDR
  pinMode(IRREAD, INPUT);
  // Serial.begin(9600);
  ambIR =0;
  for (int i =0; i<5 ;i++){
    ambIR += readIR();
  }
  ambIR = ambIR/ 5 + 60;
  
  // setupColorSensor();
  // detectColor();
}

void loop()
{
  moveForward();
  // closer to wall, IR increase
  int IRreading = readIR();
  int Ultrasonicreading = readUltrasonic();
  

  bool rightWallClose = (IRreading > ambIR + 25);
  bool leftWallClose = (Ultrasonicreading > 0 && Ultrasonicreading < 8);
  bool leftWallGone = (Ultrasonicreading >= 45 || Ultrasonicreading <= 0);
  bool rightWallGone = (IRreading < ambIR);

  if (leftWallGone && rightWallGone) moveForward();
  else if (rightWallClose) nudgeLeft();
  else if (leftWallClose) nudgeRight();
  else moveForward();


  int sensorState = lineFinder.readSensors();
  if (sensorState == S1_IN_S2_OUT){
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
    delay(100);
  }
  if (sensorState == S1_OUT_S2_IN){
    leftMotor.stop();
    rightMotor.stop();
    delay(5);
    leftMotor.run(-motorSpeed);
    rightMotor.run(-motorSpeed);
    delay(100);
  }
  if (sensorState == S1_IN_S2_IN) {
    leftMotor.stop();
    rightMotor.stop();
    delay(1000);
    int color=detectColor();
    // Serial.println(color);
    if (color==0) turnLeft();
    if (color==2) turnRight();
    if (color==3){
      if(leftWallGone|| rightWallClose) side = -1;
      else if(leftWallClose) side = 1;
      uTurn(side);  //1 if left, -1 if right 
    }
    if (color==4) doubleLeftTurn();
    if (color==1) doubleRightTurn();
    if (color==5) 
    {
    leftMotor.stop(); // Stop left motor
    rightMotor.stop(); // Stop right motor
    celebrate();
    delay(999999);
    }
    
 }

}
