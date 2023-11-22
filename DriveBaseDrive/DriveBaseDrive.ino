
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
  int sortButton;                                     // the button we activate the sorting with
  int gateButton;                                     // the button we use to open the gate door
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
const int cIN1Chan[] = {0, 1};                        // PWM channe(s) for INT1
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
const int cTCSLED = 23;                               // GPIO pin for LED on TCS34725
const int servoGate = 0;                              // servo that rotates for the gate
const int servoBucket = 0;                            // servo that rotates for the bucket
const int servoBoom = 0;                              // servo that rotates for the boom

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

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

// REPLACE WITH MAC ADDRESS OF YOUR CONTROLLER ESP32
uint8_t receiverMacAddress[] = {0x08,0xD1,0xF9,0x98,0x99,0xB8};  // MAC address of controller 00:01:02:03:04:05
esp_now_peer_info_t peerInfo = {};                    // ESP-NOW peer information

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor
  WiFi.mode(WIFI_STA);                                // Use WiFi in station mode
  Serial.print("MAC address ");
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  WiFi.disconnect();                                  // disconnect from network
  
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat
  pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output

// setup for the colour sensor
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO for control of LED on TCS34725

    // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);                         // turn on onboard LED 
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
  
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

 // if (sortButton){
      // if (rgb values match){
          //rotate servo bucket
     // }
      // if (rgb values do not match){
          //rotate servo forward (dump it)  
      //}
  // } else{keep in default orientation}

  // if (gateButton){
    // rotate gate servo to open door
  //}
  
  // if too many sequential packets have dropped, assume loss of controller, restart as safety measure
  if (commsLossCount > cMaxDroppedPackets) {
    delay(1000);                                      // okay to block here as nothing else should be happening
    ESP.restart();                                    // restart ESP32
  }
 //if (tcsFlag) {                                      // if colour sensor initialized
    //tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
//#ifdef PRINT_COLOUR            
     // Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
//#endif
  }

 // if((r > 10&&r<20)&&(g>35&&g<45)&&(b>40&&b<50)){   // checking to see if the colour of the desired object is detected
   // driveData.good = true;
  //} else {
    //driveData.good = false;
  //}
  
#ifdef PRINT_OUTGOING
  Serial.printf("Good: %d\n", driveData.good);
#endif

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

  Serial.printf("%d, %d, %d\n", inData.leftDir, inData.rightDir, inData.time);
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
