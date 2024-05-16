#include <TinyGPS++.h>
#include <Wire.h>
#include <ShiftRegister74HC595.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>


#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif

#define LED_PIN 13

#define SDI 7
#define SCLK 6
#define LOAD 5
#define DIGITS 2
// create shift register object (number of shift registers, data pin, clock pin, latch pin)
ShiftRegister74HC595 sr (DIGITS, SDI, SCLK, LOAD);

Servo servo1;
Servo servo2;

MPU6050 mpu;

// The TinyGPS++ object
TinyGPSPlus gps;
long latitude;      //Latitude (North - South)
long longitude;     //Longitude (East - West)
long scale = 10000000UL;     //10 milion. Why ? This technique is called - integer scaling. Please see below

const int encoderPin1  = 18;
const int encoderPin2  = 19;
const int buzzer = 52;

/* sensor used:
   Ublox M8N
   MPU6050
   Rotary Encoder
*/

// data variable
// gear
int digit1 = 1, digit2 = 1, backproGear = 0;

uint8_t  digits[] = {B11000000, //0
                     B11111001, //1
                     B10100100, //2
                     B10110000, //3
                     B10011001, //4
                     B10010010, //5
                     B10000010, //6
                     B11111000, //7
                     B10000000, //8
                     B10010000 //9
                    };

int gearBelakangNaik[] = {18,
                          34,
                          40,
                          47,
                          53,
                          61
                         };

int gearBelakangTurun[] = {15,
                           24,
                           30,
                           37,
                           44,
                           60
                          };

int gearDepan[] = {
  10,
  30,
};


// supply your own gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = {  -2215,  -574,   1020,    106,    4,     6};


// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte StartUP = 100; // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

// Timers
// unsigned long timer = 0;
// float timeStep = 0.01;
// Encoder
float readRPM = 0, readRPS = 0, bikeSpeed = 0;
unsigned long lastUpdate = 0;
int updateTime = 500;

int lastEncoded = 0;
long encoderValue = 0, ecoderLastTime = 0;
int lastMSB = 0, lastLSB = 0;

void coordinate() {
  while (Serial3.available() > 0) {
    gps.encode(Serial3.read());
    if (gps.location.isUpdated()) {
      latitude = gps.location.rawLat().deg * scale + gps.location.rawLat().billionths / 100UL;
      longitude = gps.location.rawLng().deg * scale + gps.location.rawLng().billionths / 100UL;
    }
  }
}

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    // if(millis()-lastDelay > MPUDelay){
    // bismillah tidka usah
    // lastDelay = millis();
    // MPU6050Connect(); // Lets try again
    // return;
    // }
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  //  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  //  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  //  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() { // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}


//// ================================================================
//// ===                        MPU Math                          ===
//// ================================================================
float Yaw, Pitch, Roll;
void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = -((ypr[1] *  180.0 / M_PI) + 18);
  Roll = (ypr[2] *  180.0 / M_PI);
}

float calculateSpeed() {
  //  float CPR = 1400; // Counts Per Revolution
  // diameter ban sepeda = 57cm. Satu putaran = pi*d = jarak yang ditempuh satu putaran = 179.09cm
  float kelilingRoda = 179.0; //cm
  float kmH = 0;
  //  jumlah revolusi (nilai encoder terhadap pprnya) / selisih waktu sampling
  readRPS = (encoderValue / 24000.0) / ((millis() - ecoderLastTime) / 1000.00);
  kmH = readRPS * kelilingRoda * 3600 / 100000;

  ecoderLastTime = millis();
  encoderValue = 0;
  
  return kmH;
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  //LMSD - LMSDB -MSB - LSB
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; //store this value for next time
}

void setup() {
  Serial.begin(9600); // usb serial
  Serial3.begin(9600);
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP); //gear belakang
  pinMode(4, INPUT_PULLUP); //gear depan
  pinMode(buzzer, OUTPUT);

  servo1.attach(9);
  servo2.attach(10);

  servo1.write(gearDepan[digit1]);
  servo2.write(gearBelakangNaik[digit2 - 1]);

  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);

  //  push button = default 1, klik 0, lepas 1
  //   pin 3
  attachInterrupt(digitalPinToInterrupt(3), updateGear, RISING);

    i2cSetup();
    MPU6050Connect();
    updateDisplay();
}

void loop() {
//  coordinate(); //update latitude longitude
  if (mpuInterrupt ) { // wait for MPU interrupt or extra packet(s) available
    GetDMP();
  }
	coordinate();

  //  training
  //   kecepatan,kemiringan,geardepan,gearbelakang
  if (millis() - lastUpdate > updateTime) {
    if(Pitch > 40)digitalWrite(buzzer, HIGH);
    else digitalWrite(buzzer, LOW);
    bikeSpeed = calculateSpeed();
    // baca gear depan
    // switch on = 2, off = 1
    if (digit1 != ((!digitalRead(4)) + 1)) {
      digit1 = ((!digitalRead(4)) + 1);
      updateDisplay();
      servo1.write(gearDepan[digit1-1]);
    }
    Serial.print(bikeSpeed);
    Serial.print(",");
    Serial.print(Pitch);
    Serial.print(",");
    Serial.print(digit1+1);
    Serial.print(",");
    Serial.println(digit2);
    Serial.print(",");
	Serial.print(latitude);
	Serial.print(",");
	Serial.println(longitude);
	lastUpdate = millis();
  }
  coordinate();
}

void updateGear() {
  if (digit2 >= 6) {
    digit2 = 1;
  }
  else {
    digit2++;
  }
  servo2.write(gearBelakangNaik[digit2 - 1]);
  updateDisplay();
}

void updateDisplay() {
  uint8_t numberToPrint[] = {digits[digit2], digits[digit1+1]};
  sr.setAll(numberToPrint);
}

void serialEvent(){
	if(Serial.available() > 0){
		backproGear = ((char)Serial.read()).toInt();
		if(backproGear > digit2){
			servo2.write(gearBelakangNaik[digit2 - 1]);
		}else servo2.write(gearBelakangTurun[digit2 - 1]);
		
		digit2 = backproGear;
	}
}
