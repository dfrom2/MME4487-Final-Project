//MME4487 Robot Design Project
//Daniel From
//Cole Russell
//Sherif Elmaghraby

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// Control data packet structure
struct ControlDataPacket {
  int leftDir;                                        // left drive direction: -1 = forward, 1 = reverse, 0 = stop
  int rightDir;                                       // right drive direction: 1 = forward, -1 = reverse, 0 = stop
  unsigned long time;                                 // time packet sent
  int pickup;
  int dump;
};

// Drive data packet structure
struct DriveDataPacket {
  unsigned long time;                                 // time packet sent                                     
};

// Encoder structure
struct Encoder {
  const int chanA;                                    // GPIO pin for encoder channel A
  const int chanB;                                    // GPIO pin for encoder channel B
  long pos;                                           // current encoder position
};

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
const int cStatusLED = 13;                            // GPIO pin of communication status LED
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cNumMotors = 2;                             // Number of DC motors
const int cIN1Pin[] = {17, 19};                       // GPIO pin(s) for INT1
const int cIN1Chan[] = {0, 1};                        // PWM channel(s) for INT1
const int c2IN2Pin[] = {16, 18};                      // GPIO pin(s) for INT2
const int cIN2Chan[] = {2, 3};                        // PWM channel(s) for INT2
const int cPWMRes = 8;                                // bit resolution for PWM
const int cMinPWM = 0;                                // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;              // PWM value for maximum speed
const int cPWMFreq = 20000;                           // frequency of PWM signal
const int cCountsRev = 1096;                          // encoder pulses per motor revolution
const int cMaxSpeedInCounts = 1600;                   // maximum encoder counts/sec
const int cMaxChange = 14;                            // maximum increment in counts/cycle
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop
const float kp = 1.5;                                 // proportional gain for PID
const float ki = 0.2;                                 // integral gain for PID
const float kd = 0.8;                                 // derivative gain for PID

//constants for sorting components
const int cboomServo = 14;                            // GPIO pin for boom Servo. CHANGE TO CORRECT VALUE!!!
const int cbucketServo = 27;                          // GPIO pin for bucket Servo. CHANGE TO CORRECT VALUE!!!
const int cgateServo = 5;                             // GPIO pin for gate Servo. CHANGE TO CORRECT VALUE!!!
const int cscoopservo = 12;                           // GPIO pin for scoop servo
const int cBoServoChannel = 5;                        // PWM channel used for Boom Servo
const int cBuServoChannel = 6;                        // PWM channel used for Bucket Servo
const int cGServoChannel = 7;                         // PWM channel used for Gate Servo   
const int cSServoChannel = 4;                         // PWM channel used for scoop servo   

const int cTCSLED = 23;                               // GPIO pin for LED on TCS34725

// Variables
unsigned long lastHeartbeat = 0;                      // time of last heartbeat state change
unsigned long lastTime = 0;                           // last time of motor control was updated
unsigned int commsLossCount = 0;                      // number of sequential sent packets have dropped
Encoder encoder[] = {{25, 26, 0},                     // encoder 0 on GPIO 25 and 26, 0 position
                     {32, 33, 0}};                    // encoder 1 on GPIO 32 and 33, 0 position
long target[] = {0, 0};                               // target encoder count for motor
long lastEncoder[] = {0, 0};                          // encoder count at last control cycle
float targetF[] = {0.0, 0.0};                         // target for motor as float
ControlDataPacket inData;                             // control data packet from controller
DriveDataPacket driveData;                            // data packet to send controller

int i_boomVal = 0;                                    // desired servo angle for boom
int i_bucketVal = 0;                                  // desired servo angle for bucket
int i_gateVal;                                        // desired servo angle for gate
int i_scoopVal;                                       // desired servo angle for scoop 

int pickupState = 0;                                  // current state of the sorting loop
int wait;                                             // delay used for sorting loop
bool good;                                            // flag for checking for desired item

// REPLACE WITH MAC ADDRESS OF YOUR CONTROLLER ESP32
uint8_t receiverMacAddress[] = {0xA8,0x42,0xE3,0xCA,0xF1,0xBC};  // MAC address of controller A8:42:E3:CA:F1:BC
esp_now_peer_info_t peerInfo = {};                    // ESP-NOW peer information

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor
  WiFi.mode(WIFI_STA);                                // Use WiFi in station mode
  Serial.print("MAC address ");
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  WiFi.disconnect();                                  // disconnect from network
  
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat
  pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output
  ledcAttachPin(cboomServo, cBoServoChannel);         // assign boom servo pin to servo channel
  ledcAttachPin(cbucketServo, cBuServoChannel);       // assign bucket servo pin to servo channel
  ledcAttachPin(cgateServo, cGServoChannel);          // assign gate servo pin to servo channel
  ledcAttachPin(cscoopservo, cSServoChannel);         // assign scoop servo pin to servo channel
  ledcSetup(cBoServoChannel, 50, 16);                 // setup channel for 50Hz and 16-bit resolution
  ledcSetup(cBuServoChannel, 50, 16);                 // setup channel for 50Hz and 16-bit resolution
  ledcSetup(cGServoChannel, 50, 16);                  // setup channel for 50Hz and 16-bit resolution
  ledcSetup(cSServoChannel, 50, 16);                  // setup channel for 50hz and 16-bit resolution
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO for control of LED on TCS34725


  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);           // attach INT1 GPIO to PWM channel
    ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);        // configure PWM channel frequency and resolution
    ledcAttachPin(c2IN2Pin[k], cIN2Chan[k]);          // attach INT2 GPIO to PWM channel
    ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);        // configure PWM channel frequency and resolution
    pinMode(encoder[k].chanA, INPUT);                 // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);                 // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.printf("Error initializing ESP-NOW\n");
    return;
  }
  else
  {
    Serial.printf("Successfully initialized ESP-NOW\n");
  }
  esp_now_register_recv_cb(onDataRecv);               // register callback function for received data
  esp_now_register_send_cb(onDataSent);               // register callback function for data transmission
  
  // Set controller info
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);  // set address of peer
  peerInfo.channel = 0;                               // set peer channel
  peerInfo.encrypt = false;                           // no encryption of data
  
  // Add controller as ESP-NOW peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.printf("Failed to add peer\n");
    return;
  }
  else
  {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n to list", receiverMacAddress[0], receiverMacAddress[1], 
                                                                         receiverMacAddress[2], receiverMacAddress[3], 
                                                                         receiverMacAddress[4], receiverMacAddress[5]);
  }
  // COnnect to TCS34725
  if(tcs.begin()){
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);                         // turn on onboard LED
  }
  else{
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}

void loop() {
  float deltaT = 0;                                   // time interval
  long pos[] = {0, 0};                                // current motor positions
  float velEncoder[] = {0, 0};                        // motor velocity in counts/sec
  float velMotor[] = {0, 0};                          // motor shaft velocity in rpm
  float posChange[] = {0, 0};                         // change in position for set speed
  long e[] = {0, 0};                                  // position error
  float ePrev[] = {0, 0};                             // previous position error
  float dedt[] = {0, 0};                              // rate of change of position error (de/dt)
  float eIntegral[] = {0, 0};                         // integral of error 
  float u[] = {0, 0};                                 // PID control signal
  int pwm[] = {0, 0};                                 // motor speed(s), represented in bit resolution
  int dir[] = {1, 1};                                 // direction that motor should turn
  int gateWait = 0;

  // if too many sequential packets have dropped, assume loss of controller, restart as safety measure
  if (commsLossCount > cMaxDroppedPackets) {
    delay(1000);                                      // okay to block here as nothing else should be happening
    ESP.restart();                                    // restart ESP32
  }

  uint16_t r, g, b, c;                                // RGBC values from TCS34725

  if(tcsFlag){                                        // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values

    Serial.printf("R: %d, G: %d, B: %d, C: %d good?: %d\n", r, g, b, c, good);
    //Serial.printf(" Boom: %d, Bucket: %d, Scoop: %d, Gate: %d, good?: %d Pickup state: %d\n", i_boomVal, i_bucketVal, i_scoopVal, i_gateVal, good, pickupState);

  }

   if((r>=0&&r<=8) && (g>=0&&g<=4) && (b>=0&&b<=4)){                  // colour is set to the small green rocks
     good = HIGH;                          // boolean value is true
   }
   else{
     good = LOW;                           // else boolean value is false
   }

  
    

  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();                                     // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;                          // read and store current motor position
  }
  interrupts();                                       // turn interrupts back on
  
  unsigned long curTime = micros();                   // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                               // update start time for next control cycle
    driveData.time = curTime;                         // update transmission time
  
  if(inData.dump){
    i_gateVal = 0;
    //good = !good;
  } else {
    i_gateVal=45;
  }


  if(inData.pickup){                                  // if the sorting button is pressed on the controller
    pickupState = 1;                                  // start the pickup sequence
  }  
        if(pickupState==0){                           // if the pickup sequence is not active, hold the servos in place
          i_bucketVal = 98;
          i_boomVal = 180;
          i_scoopVal = 90;
          wait = 0;
        }
          
       if(pickupState==1){                            // if sorting sequence is at stage 1, raise boom slightly to move object towards colour sensor
           if(i_scoopVal>0){
            i_scoopVal=i_scoopVal-2;   
          }
          if(wait > 100){
            if(i_boomVal>120){                          // if the boom position is greater than the desired position, move the boom until it reaches the desired position
              i_boomVal--;
            } else if(i_boomVal<=120&&i_scoopVal<=0) {
              wait = 0;                                 // reset the delay variable
             pickupState = 2;                          // change to next pickup state
            }
          } else {
            wait ++;
          }
       }

        if(pickupState==2){                           // wait 300 cycles to allow the colour sensor to read the value of the object
          wait++;
          if(wait>150){
            if(good){                                 // if desired object, move to the next stage in the sorting sequence
              pickupState = 3;  
            } else {                                  // else, drop the object and reset the sorting sequence                      
              pickupState = 5;
            } 
          }
        }
            
        if(pickupState==3){                          // move bucket and boom until its above the storage box
          if(i_boomVal>4){
            i_boomVal=i_boomVal-2;
          }
          if(i_bucketVal< 180){
            i_bucketVal=i_bucketVal+2;
          }
          if(i_boomVal <= 4 && i_bucketVal >= 180){
            pickupState = 4;                        // switch to the next stage in the sorting sequence
          }
        }
          

        if(pickupState==4){                         // drop the object into the storage box
          if(i_bucketVal > 88){
            i_bucketVal--;
          } else {
            pickupState = 0;                       // reset the sorting sequence
          }
        }

        if(pickupState==5){
          if(i_bucketVal < 190) {
                i_bucketVal++;
          } if(i_scoopVal < 90){
                i_scoopVal++;
          }else if(i_bucketVal>=190&&i_scoopVal>=90) {
                pickupState = 0;
          }
        }
    ledcWrite(cBoServoChannel, degreesToDutyCycle(i_boomVal));    // setting the position of the boom
    ledcWrite(cBuServoChannel, degreesToDutyCycle(i_bucketVal));  // setting the position of the bucket
    ledcWrite(cSServoChannel, degreesToDutyCycle(i_scoopVal));    // setting the position of the scoop
    ledcWrite(cGServoChannel, degreesToDutyCycle(i_gateVal));     // setting the position of the gate

    for (int k = 0; k < cNumMotors; k++) {
      velEncoder[k] = ((float) pos[k] - (float) lastEncoder[k]) / deltaT; // calculate velocity in counts/sec
      lastEncoder[k] = pos[k];                        // store encoder count for next control cycle
      velMotor[k] = velEncoder[k] / cCountsRev * 60;  // calculate motor shaft velocity in rpm

      // update target for set direction
      posChange[0] = (float) (inData.leftDir * cMaxChange); // update left side with maximum speed
      posChange[1] = (float) (inData.rightDir * cMaxChange); // update right side with maximum speed
      targetF[k] = targetF[k] + posChange[k];         // set new target position
      if (k == 0) {                                   // assume differential drive
        target[k] = (long) targetF[k];                // motor 1 spins one way
      }
      else {
        target[k] = (long) -targetF[k];               // motor 2 spins in opposite direction
      }

      // use PID to calculate control signal to motor
      e[k] = target[k] - pos[k];                      // position error
      dedt[k] = ((float) e[k]- ePrev[k]) / deltaT;    // derivative of error
      eIntegral[k] = eIntegral[k] + e[k] * deltaT;    // integral of error (finite difference)
      u[k] = kp * e[k] + kd * dedt[k] + ki * eIntegral[k]; // compute PID-based control signal
      ePrev[k] = e[k];                                // store error for next control cycle
  
      // set direction based on computed control signal
      dir[k] = 1;                                     // default to forward directon
      if (u[k] < 0) {                                 // if control signal is negative
        dir[k] = -1;                                  // set direction to reverse
      }

      // set speed based on computed control signal
      u[k] = fabs(u[k]);                              // get magnitude of control signal
      if (u[k] > cMaxSpeedInCounts) {                 // if control signal will saturate motor
        u[k] = cMaxSpeedInCounts;                     // impose upper limit
      }
      pwm[k] = map(u[k], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM); // convert control signal to pwm
      if (commsLossCount < cMaxDroppedPackets / 4) {
        setMotor(dir[k], pwm[k], cIN1Chan[k], cIN2Chan[k]); // update motor speed and direction
      }
      else {
        setMotor(0, 0, cIN1Chan[k], cIN2Chan[k]);     // stop motor
      }
    
      //Serial.printf("Direction 1: %d, Direction 2: %d, PWM 1: %d, PWM 2: %d\n",dir[0],dir[1],pwm[0],pwm[1]);
    } 

    // send data from drive to controller
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &driveData, sizeof(driveData)); 
    if (result == ESP_OK) {                           // if sent successfully
      digitalWrite(cStatusLED, 0);                    // turn off communucation status LED
    }
    else {                                            // otherwise
      digitalWrite(cStatusLED, 1);                    // turn on communication status LED
    }
  }

  doHeartbeat();                                      // update heartbeat LED
}

// blink heartbeat LED
void doHeartbeat() {
  unsigned long curMillis = millis();                 // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat toggle time for the next cycle
    digitalWrite(cHeartbeatLED, !digitalRead(cHeartbeatLED)); // toggle state of LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                     // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                               // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                              // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);            // cast pointer to static structure
  
  int b = digitalRead(s->chanB);                      // read state of channel B
  if (b > 0) {                                        // high, leading channel A
    s->pos++;                                         // increase position
  }
  else {                                              // low, lagging channel A
    s->pos--;                                         // decrease position
  }
}

// callback function for when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == 0)                                       // if empty packet
  {
    return;                                           // return
  }
  memcpy(&inData, incomingData, sizeof(inData));      // store drive data from controller

  //Serial.printf("%d, %d, %d\n", inData.leftDir, inData.rightDir, inData.time);
}

// callback function for when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef PRINT_SEND_STATUS
  Serial.printf("Last packet send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
#endif
  if (status != ESP_NOW_SEND_SUCCESS) {
    digitalWrite(cStatusLED, 1);                      // turn on communication status LED
    commsLossCount++;                                 // increase lost packet count
  }
  else {
    commsLossCount = 0;                               // reset communication loss counter
  }
}

//changing a degree input into a duty cycle usable by the servos
long degreesToDutyCycle(int deg) {
  const long cl_MinDutyCycle = 1650;                 // duty cycle for 0 degrees
  const long cl_MaxDutyCycle = 8175;                 // duty cycle for 180 degrees

  long l_DutyCycle = map(deg, 0, 180, cl_MinDutyCycle, cl_MaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float f_Percent = l_DutyCycle * 0.0015259;         // dutyCycle / 65535 * 100
  
#endif

  return l_DutyCycle;
}
