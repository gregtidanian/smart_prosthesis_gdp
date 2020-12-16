//libraries needed for running the code
#include <Filter.h> //library for exponential smoothing algorithm
#include <Servo.h> //library for servo
#include <SoftwareSerial.h> //library for serial communication between Arduino and Pi
#include <Rhino.h> //library for rhino servo
#include<Wire.h> // library used for the IMU
#include <QuickStats.h> //library used for calculating median of IMU angles

//#define LOWTHRESHOLD 1.5 //ankle threshold value at 2 kph
//#define HIGHTHRESHOLD 3.1 // ankle threshold value at 5kph
#define LOWTHRESHOLD 1.5 //ankle threshold value at 1 kph
#define HIGHTHRESHOLD 2.3 // ankle threshold value at 6kph
//#define ankle_threshold = LOWTHRESHOLD // EMG Threshold to activate the motor movement when exceeded
#define GRADIENTTHRESHOLD 3.75 // EMG threshold to estimate flat foot - Activates IMU to measure current slope

#define MEDIANVALUES 5 //number of IMU angle values to be included in the median calculation

#define FLATINCLINELIMIT 10  //limit of averaged angle to distinguish between flat and incline
#define GRADIENTITERATIONS 10 // number of iterations before starting to calculate gradient

#define FILTERING_FACTOR 20

#define LED_IMU 12 // Pin number for GradientThreshold LED

#define MPU_addr 0x68 //I2C bus address
#define PWR_MGMT 0x6B // PWR_MGMT_1 register
#define ACCEL_XOUT_H 0x3B
#define SPEED_GENERIC 50
#define SPEED_LOW 20
#define SPEED_HIGH 255
#define PLANTARSPEED 50 //set speed of servo movement during plantarflexion - previous 255
#define DORSISPEED 50 //set speed of servo movement during dorsiflexion

#define INCREMENTANGLE 30 //angle that motor moves by when commanded to by the mobile app
#define MAXPLANTARFLEXANGLE 30 //maximum angle of servo to move ankle to maximum plantarflexion when commanded to by the mobile app
#define MAXDORSIFLEXANGLE 20 //maximum angle of servo to move ankle to maximum dorsiflexion when commanded to by the mobile app
#define ANGLE_LIMIT 40

//IMU variable declarations
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //16-bit integer defining IMU read-outs
int minVal=265; 
int maxVal=402;

// Variable Declarations for angle calculations
double x;
double y;
double z;

// Set Digital Pin 9 and 10 to be RX and TX
Rhino motor1(9,10);  // RX( -> TX of motor), TX( -> RX of motor)

// Number of analog input pin on the arduino. Gastro refers to signals 
// coming from the Gastrocnemius muscle, and Tibi the Tibialis Medialis
int analogInputGastro = 0;

// variable of value of emg response
int valueRMSGastro;

// variable to initialise the timer
float initialtimer;

//threshold for indicating the gastrocnemius has been contracted in the desired manner for toe off
float ankle_threshold;
float gradient_threshold = GRADIENTTHRESHOLD; // threshold for determining the gradient when the foot is flat on the ground


//previous and current EMG signals are set to false, showing they are below the threshold EMG. 
bool prEMG = false;
bool cEMG = false;

// previous and current EMG for gradient calculation using IMU
bool imuprEMG = false;
bool imucEMG = false;

// variables needed to calculate the IMU angle and average them over a number of n values
float currentangle;

//string of data recieved from the raspberry pi, sent to it from the app
String stringcommand;
String commandoutput;

//status of whether or not the prosthetic is locked EMG has no influence on ankle movement
bool lockStatus = false;

// variable of exponential filter
float FilteringFactor = FILTERING_FACTOR;
ExponentialFilter<float> FilteredEMG(FilteringFactor, 0);

//variable for the state of the slope - decline 1=-1, flat=0, incline 1=1
int slopestate = 0;

//global variable for the angle - not 0 when there is flat foot
double slopedeg;

const int n = MEDIANVALUES; //number of values in running median calculation
int count = 0;
float patternList[n]={0};
float medangle = 0;
QuickStats stats; //Initialise stats for median calculation
float currentgradient = 0; // Initialise and declare the gradient. 0-->flat and 1-->incline
int gradientcounter = 0; //counter to start giving gradients after a number of iterations
const int niterations = GRADIENTITERATIONS;
float motorspeed = SPEED_GENERIC;

// variable declarations for step counter
int stepcounter = 0;


void setup()
{
  //Wire functions are for setting up IMU
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(PWR_MGMT);
  Wire.write(0);
  Wire.endTransmission(true);

  motor1.printOutput(1); // enables Serial printing of function outputs
  motor1.init();  // initializes the motor and sets current position as origin
  motor1.setSpeed(motorspeed);
  motor1.setConstraintDeg(-ANGLE_LIMIT,ANGLE_LIMIT); // can only move at -40 and 40 from initial position
  
  ankle_threshold = HIGHTHRESHOLD; // EMG Threshold to activate the motor movement when exceeded
  Serial.println(ankle_threshold);
  // declaration of LED for gait, emg
  pinMode(LED_BUILTIN, OUTPUT);
  
  // initialise timer
  initialtimer = millis();

  prEMG = false;
  cEMG = false;
  
  // begin sending over serial port
  Serial.begin(9600);
  Serial.setTimeout(10); //set serial communication to 10 ms
  
  
}


void loop()
{
  stepcounter = 0;
  slopedeg=0;// set walking angle to 0
  
  currentangle=IMUangle(); //calculate the current angle IMU angle 

  picommand(); // check in serial connection for a command to move the motor

  // read the value on analog input
  valueRMSGastro = analogRead(analogInputGastro);
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):  
  float RMSGastro = valueRMSGastro * (5.0 / 1023.0);

  //filtering the EMG function using the exponential filer command from the Filter library
  FilteredEMG.Filter(RMSGastro);
  float SmoothEMG = FilteredEMG.Current();
  float timer = (millis() - initialtimer)*0.001;

  //if EMG signal surpasses the gradient threshold, the current imu EMG status is set to true
  if(SmoothEMG > gradient_threshold){
    imucEMG = true;
  }
  else{
    imucEMG = false;
  }

  //if EMG signal surpasses the threshold, the current EMG status is set to true, if not, it is set to false
  if(SmoothEMG > ankle_threshold){
    cEMG = true;
  }
  else{
    cEMG = false;
  }

  //if the previous imu EMG signal is below the threshold, and current is above, it calculates the gradient of the ground to optimise power output
  if(imuprEMG == false and imucEMG == true){
    slopedeg = currentangle;
    medangle = medianangle(slopedeg);
  }
  
  //if the previous EMG signal is below the threshold, and current is above, it signifies the instance the EMG signal supasses the threshold and calls the motor function
  if(prEMG == false and cEMG == true and lockStatus == false){
    stepcounter = 1;
    //Serial.println("SHOULD HACE MOVED");
    //ledtestEMG();
    //motor(20, 50, 400);
  }


  //previous EMG signal is now equal to the current EMG signal, ready for the next cycle
  prEMG = cEMG;
  imuprEMG = imucEMG;

  //determint the gradient if a number niterations is passed
  if(gradientcounter >= niterations){
    currentgradient = checkgradient(medangle);
  }
  gradientcounter = gradientcounter+1;


  Serial.println(String(SmoothEMG) + "," + String(timer) + "," + String(currentangle) + "," + String(medangle) + "," + String(slopedeg) + "," + String(currentgradient) + "," + String(stepcounter) + "," + String(0));
  
  //Serial.println(SmoothEMG);

}


// turn the LED on to indicate threshold was exceeded
void ledtestEMG(){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
}


// function to move the motor (plantarflexion and dorsiflexion) by a specific number of degrees
void motor(float deg, float vel, float del)
{
Serial.println("gait cycle initiated");
float counts_angle = deg*5;
Serial.println(counts_angle);
float counts_angle2 = -(2*counts_angle);
Serial.println(counts_angle2);
motor1.setSpeed(vel);
float pos0 = 0;
//Serial.println("POS0: "+String(pos0));
float pos1 = deg;
//Serial.println("POS1: "+String(pos1));
float pos2 = -deg;
//Serial.println("POS2: "+String(pos2));
motor1.gotoAngleDeg(pos1);
//Serial.println("pos1");
delay(del);
motor1.gotoAngleDeg(pos2);
//Serial.println("pos2");
delay(del);
motor1.gotoAngleDeg(pos0);
//Serial.println("pos0");
delay(del);
//motor1.gotoRel(counts_angle); // move the motor by a specific angle in degrees
//delay(1000); 
//motor1.gotoRel(counts_angle2); // go back -deg from initial position 0
//delay(1000);
//motor1.gotoRel(counts_angle); // go back to position 0
//delay(500);
motor1.setSpeed(motorspeed);
}


// function to plantarflex
void plantarflex(float angledeg)
{
digitalWrite(LED_BUILTIN, HIGH);
motor1.setSpeed(PLANTARSPEED);
motor1.gotoRel(angledeg*5);
motor1.setSpeed(motorspeed);
digitalWrite(LED_BUILTIN, LOW);
}


//function to dorsiflex
void dorsiflex(float angledeg)
{
digitalWrite(LED_BUILTIN, HIGH);
motor1.setSpeed(DORSISPEED);
motor1.gotoRel(-angledeg*5);
motor1.setSpeed(motorspeed);
digitalWrite(LED_BUILTIN, LOW);
}


void maxplantarflex()
{
digitalWrite(LED_BUILTIN, HIGH);
motor1.setSpeed(PLANTARSPEED);
motor1.gotoAngleDeg(ANGLE_LIMIT);
motor1.setSpeed(motorspeed);
digitalWrite(LED_BUILTIN, LOW);
}


//function to dorsiflex
void maxdorsiflex()
{
digitalWrite(LED_BUILTIN, HIGH);
motor1.setSpeed(DORSISPEED);
motor1.gotoAngleDeg(-ANGLE_LIMIT);
motor1.setSpeed(motorspeed);
digitalWrite(LED_BUILTIN, LOW);
}

//function to dorsiflex
void recalibrate()
{
digitalWrite(LED_BUILTIN, HIGH);
motor1.setSpeed(motorspeed);
motor1.setPos(0);
motor1.setConstraintDeg(-ANGLE_LIMIT,ANGLE_LIMIT);
digitalWrite(LED_BUILTIN, LOW);
}



// function to set this position
void setPosDeg(float deg){
  float counts = deg*5;
  motor1.setPos(counts);
}

// function to get the current position
float getPosDeg(){
  float countspos = motor1.getPos();
  float degpos = countspos/5.0;
  return degpos;
}

// function to calculate the angle from the ground using the IMU
float IMUangle(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);

  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  if(x>180){
    x = x-360;
  }
  return x; // return the x angle
  }

  // function to check for an input from the serial connection
  void picommand(){
    stringcommand = Serial.readString();

    //while there is a serial connection, read data as a string and move the servo to plantarflex or dosiflex depening on the incoming data
    
    if (stringcommand.length() > 0) {
    //while there is a serial connection
    Serial.println("longting");
    Serial.println(stringcommand);
    
    if(stringcommand == "plantarflex"){
      lockStatus = true;
      Serial.println("plantarflex");
      plantarflex(INCREMENTANGLE); //moves motor by a predefined incremental angle
      delay(1000);
      //digitalWrite(LED_BUILTIN, LOW);
    }
    
    if(stringcommand == "dorsiflex"){
      //dorsiflex();
      lockStatus = true;
      Serial.println("dorsiflex");
      dorsiflex(INCREMENTANGLE); //moves motor by a predefined incremental angle
    }
    if(stringcommand == "maxplantarflex"){
      lockStatus = true; //EMG signals do not influence the movement of the ankle in this instance
      maxplantarflex();
    }
    if(stringcommand == "maxdorsiflex"){
      lockStatus = true; //EMG signals do not influence the movement of the ankle in this instance
      maxdorsiflex();
    }
    if(stringcommand == "lock"){
      lockStatus = true; //EMG signals do not influence the movement of the ankle in this instance
    }
    if(stringcommand == "unlock"){
      lockStatus = false; //EMG signals influence the movement of the ankle in this instance
    }
    if(stringcommand == "lowvel"){
      ankle_threshold = LOWTHRESHOLD; //Adjusts ankle threshold for low walking velocity
      motorspeed = SPEED_LOW;
      
    }
    if(stringcommand == "highvel"){
      ankle_threshold = HIGHTHRESHOLD; //Adjusts ankle threshold for high walking velocity
      motorspeed = SPEED_HIGH;
    }

    if(stringcommand == "elaa"){
      
      for (int i = 0; i < 3; i++) {
        motor(20, 50, 400);
        delay(200);
        }
       //Adjusts ankle threshold for high walking velocity
    }

    if(stringcommand == "resetposition"){
      motor1.gotoAngleDeg(0); //Adjusts ankle threshold for high walking velocity
    }
    if(stringcommand == "recalibrate"){
      recalibrate(); //Adjusts ankle threshold for high walking velocity
    }
    
    }
  
    }

    
  float medianangle(float angleForMedian){
    
  float medianangle;
  patternList[count] = angleForMedian;
  count = count + 1;

  if(count > n-1){
    medianangle = stats.median(patternList, n);
    count = count - 1;
    memmove(&patternList[0], &patternList[1], (n)*sizeof(patternList[0])); //shift the array one value to the left
    return medianangle;
  } else{
    return 0;
  }
  }

  // function that takes the current averaged angle (theta) and calculates the gradient. returns 0 if flat and 1 if incline
  int checkgradient(float theta){
    int gradient = 0;
    float slopethreshold = FLATINCLINELIMIT;
    if(theta<slopethreshold){
      gradient = 0;
    } else {
      gradient = 1;
    }
    return gradient;
  }
