#include <Adafruit_BMP085_U.h>
#include <SPI.h>
#include <SD.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>
#include <ArduinoBLE.h>

//sd card
File myFile;

//important variables
double flightTimer = 0;
double flightTimerBegin = 0;
double expectedThrustTime = 3000;//IMPORTANT MOTOR BURN TIME IN MILLISECONDS
double delpoyTime = 0;
double deployDuration = 1000; //duration of servo deploy time in millisseconds

//Bluetooth stuff
long previousMillis = 0;
int interval = 0;
int ledState = LOW;

BLEService launchService("180A"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("2A57", BLERead | BLEWrite);

//random External variables
int buzzer = 6;
int redLED = 15;

//Servo Variables
Servo xAxisServo;
int xServoPos = 0;
int xServoPin = 2;
double xServoHome = 0;
//360 degree servo
Servo ejectServo;
int ejectServoPin = 4;


//altimeter tracking varibles
double previousAltitude = 0;
double timer = millis();

//Barometric Variables
SFE_BMP180 pressure; //define pressure sensor
double baseline; // baseline pressur
double rocketAltitude = 0; //height in meters



//IMU variables 
float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

int rocketState = 0; //what r we doing?

long lastTime;
long lastInterval;

//DEFINE PID
double pThrust = 1;
double dThrust = .05;
double iThrust = .0000001;


//define error variables
double xError = 0;
double dxError = 0;
double ixError = 0;


double yError = 0; 
double dyError = 0;
double iyError = 0;

double xOutput = 0;
double yOutput = 0;

double desiredXAngle = 0;
double desiredYAngle = 0;


void setup() {
  
  // put your setup code here, to run once:
//  Serial.begin(9600);
//  Serial.println("REBOOT");
  //PIN INITIALIZING
  pinMode(buzzer, OUTPUT);
  pinMode(redLED, OUTPUT);
  
  xAxisServo.attach(xServoPin); //PWM pin 2
  ejectServo.attach(ejectServoPin);

  ejectServo.write(90);
  //Serial.println(rocketAltitude);
  
  rocketState = 0;
  
  initializeAltimeter();
  //initializeSd();
   //digitalWrite(redLED, HIGH);
  
  initializeIMU();
  //bluetoothInit();
 }

void loop() {
  // put your main code here, to run repeatedly:
  readAltimeter();
  excecuteIMU();
  determineState();
  //sdCardWrite();
  
  excecuteState();
  //excecuteBluetooth();
  calculateErrorsAndOutputs();
   xAxisServo.write(xServoPos);
  Serial.println(rocketAltitude);
}
void deployParachute() {
  ejectServo.write(180);
  delay(deployDuration);
  ejectServo.write(90);
    
}

void determineState (){
  if (rocketState == 0 && accelX >=3){ //If we're waiting to launch then get sudden g load
    
    rocketState = 1;
    digitalWrite(redLED, HIGH);
    flightTimerBegin = millis();
    
  } else if (rocketState == 1 && flightTimer >= expectedThrustTime && accelX <=2){ //if we have been flying for the expected time under thrust and we're still not under acceleration
    rocketState = 2; //unpowered ascent 
    tone(buzzer, 500);
    Serial.println("YAY");
    
  } else if (rocketState == 2 &&checkApogee()
  ){ //hit apogee
   digitalWrite(redLED, LOW);
  
   rocketState = 3; //past apogee, descent
   deployParachute();
   
   noTone(buzzer);
   
 }
}


void excecuteState () { //performs operations based on the state of the rocket
  if (rocketState == 0){
    //excecuteBluetooth();
    calculateErrorsAndOutputs();
    xAxisServo.write(xServoPos);
    //Serial.print(xServoPos);
  } else if (rocketState == 1) {
    calculateErrorsAndOutputs();
   
    xAxisServo.write(xServoPos);
    
  } else if (rocketState == 2) {
    
  }
}

void calculateErrorsAndOutputs() { //PID controller
  xError = complementaryYaw - desiredXAngle;
  dxError = gyroZ; //dx of error is the same as angular rate
  ixError += xError * lastInterval; //integrate with dT
  
  yError = complementaryPitch - desiredYAngle;
  dyError = gyroY; //same thing as above
  iyError += yError * lastInterval;
  
  xOutput = (pThrust * xError) + (dThrust * dxError) + (iThrust * ixError); //calculate output using PID with the PID constants
  
  yOutput = (pThrust * yError) + (dThrust * dyError) + (iThrust * iyError);
  
  xServoPos = xOutput+90;

  flightTimer = millis() - flightTimerBegin;
  
  
}


//Have we hit apogee yet?? (works pretty reliably)
boolean checkApogee() {
  if (millis()-timer>1000){ //waits to check every second
    //Serial.println("1 second");
    
    timer = millis();
    
    if (previousAltitude>rocketAltitude+.75){ //have we descended in the last second by more than a meter? eliminates false triggers due to barometric drift
      //Serial.println("APOGEE!!");
      return true;
    
    } 
    
    previousAltitude = rocketAltitude; //reassign previous altitude
    return false;
  } else {
    return false;
  }
}

void initializeSd() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("This is a test file :)");
    myFile.println("testing 1, 2, 3.");
//    for (int i = 0; i < 20; i++) {
//      myFile.println(i);
//    }
    // close the file:
//    myFile.close();
    
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void sdCardWrite () {
  myFile.println(rocketAltitude);
  Serial.println(rocketAltitude);
  myFile.close();
}

void initializeAltimeter (){ //initializes altimeter
 // Initialize the sensor (it is important to get calibration values stored on the device).
  
  if (pressure.begin()){
    Serial.println("BMP180 init success");
    
   //digitalWrite(redLED, HIGH);
   } else {
    
   //digitalWrite(redLED, HIGH);
  
   Serial.println("BMP180 init fail (disconnected?)\n\n");
   //while(1); // Pause forever.
  }
  
//digitalWrite(redLED, HIGH);
  // Get the baseline pressure:
  
  baseline = getPressure(); //ground altitude
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb"); 

   
}

void readAltimeter () {
  double a,P; //previous is to store altitude change
  
  // Get a new pressure reading:

  P = getPressure();

  // Show the relative altitude difference between
  // the new reading and the baseline reading:

  a = pressure.altitude(P,baseline);
  

  rocketAltitude = a;
  
//  Serial.print("relative altitude: ");
//  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  //Serial.println(a,1);
//  Serial.print(" meters, ");
//  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
//  Serial.print(a*3.28084,0);
//  Serial.println(" feet");
  
}

double getPressure() {
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
 }


void initializeIMU() {
  pinMode(LED_BUILTIN, OUTPUT);

  // this sketch will wait until something connects to serial!
  // this could be 'serial monitor', 'serial plotter' or 'processing.org P3D client' (see ./processing/RollPitchYaw3d.pde file)
//  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  calibrateIMU(250, 250);

  lastTime = micros();}
  
 


void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

}

bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}


void excecuteIMU() {
  if (readIMU()) {
    long currentTime = micros();
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    doCalculations();
    //printCalculations();

  }
}


void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / M_PI;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;


  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = (gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency));
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch; //ADDED 90 SO 0 DEGREES IS UPWARDS

}

void bluetoothInit() {
  // set built in LED pin to output mode
  pinMode(LED_BUILTIN, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Nano 33 IoT");
  BLE.setAdvertisedService(launchService);

  // add the characteristic to the service
  launchService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(launchService);

  // set the initial value for the characteristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");

}

void excecuteBluetooth () {
    // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
   
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        switch (switchCharacteristic.value()) {   // any value other than 0
            case 01:
            //initializeIMU();           //resetIMU
            break;
          case 02:
              
            break;
          case 03:
            
                     // will turn the LED off
            break;
          default:
                  // will turn the LED off
            break;
        }
      }
    

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, LOW);         // will turn the LED off
  }
}
