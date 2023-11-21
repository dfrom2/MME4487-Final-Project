

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

void doHeartbeat();
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

struct Button{                                        // button structure
  const int pin;
  unsigned int numberPresses;
  unsigned int lastPressTime;
  bool pressed;
  bool state;
  bool lastState;
};

struct ControlDataPacket {
  int leftDir;                                        // drive direction: -1 = forward, 1 = reverse, 0 = stop
  int rightDir;                                       // right drive direction 1 = forward, -1 = reverse, 0 = stop
  unsigned long time;                                 // time packet sent
  int sortButton;
  int gateButton;
};

struct DriveDataPacket {
  unsigned long time;                                 // time packet sent
};

const int cStatusLED = 26;
const int cHeartbeatLED = 2;
const int cHeartbeatInterval = 500;
const long cDebounceDelay = 50;
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop

unsigned long lastHeartbeat = 0;                      // time since last heartbeat
unsigned long lastTime = 0;                           // time since last cycle
unsigned int commsLossCount = 0;                      // number of sequential sent packets have dropped

//defining buttons
Button buttonFwd = {14, 0, 0, false, true, true};
Button buttonRev = {13, 0, 0, false, true, true};
Button buttonLeft = {27, 0, 0, false, true, true};
Button buttonRight = {12, 0, 0, false, true, true};
Button buttonPickup = {25, 0, 0, false, true, true};
Button buttonDrop = {33, 0, 0, false, true, true};

// REPLACE WITH MAC ADDRESS OF YOUR DRIVE ESP32
uint8_t receiverMacAddress[] = {0x08,0xD1,0xF9,0x98,0x99,0xB8};  // MAC address of drive 00:01:02:03:04:05 
esp_now_peer_info_t peerInfo = {};                    // ESP-NOW peer information
ControlDataPacket controlData;                        // data packet to send to drive system
DriveDataPacket inData;                               // data packet from drive system

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);                                // use WiFi in station mode
  Serial.print("MAC address ");
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  WiFi.disconnect(); 
  
  pinMode(cHeartbeatLED, OUTPUT);                     // configuring GPIO for the heartbeat LED
  pinMode(cStatusLED, OUTPUT);                        // configuring GPIO for the status LED

  // configuring GPIO for all buttons
  pinMode(buttonFwd.pin, INPUT_PULLUP);
  attachInterruptArg(buttonFwd.pin, buttonISR, &buttonFwd, CHANGE);
  pinMode(buttonRev.pin, INPUT_PULLUP);
  attachInterruptArg(buttonRev.pin, buttonISR, &buttonRev, CHANGE);
  pinMode(buttonLeft.pin, INPUT_PULLUP);
  attachInterruptArg(buttonLeft.pin, buttonISR, &buttonLeft, CHANGE);
  pinMode(buttonRight.pin, INPUT_PULLUP);
  attachInterruptArg(buttonRight.pin, buttonISR, &buttonRight, CHANGE);
  pinMode(buttonPickup.pin, INPUT_PULLUP);
  attachInterruptArg(buttonPickup.pin, buttonISR, &buttonPickup, CHANGE);
  pinMode(buttonDrop.pin, INPUT_PULLUP);
  attachInterruptArg(buttonDrop.pin, buttonISR, &buttonDrop, CHANGE);

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
  
  // Set drive info
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);  // set address of peer
  peerInfo.channel = 0;                               // set peer channel 
  peerInfo.encrypt = false;                           // no encryption of data
  
  // Add drive as ESP-NOW peer        
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
  esp_err_t result;
  unsigned long curTime = millis();
  if(curTime - lastTime > 10){
    lastTime = curTime;
    if (!buttonFwd.state) {                           // forward pushbutton pressed
      if (!buttonLeft.state){                         // left pushbutton pressed turn robot left
        controlData.leftDir = 0;  
      } else if (!buttonRight.state){                 // right pushbutton pressed turn robot right
        controlData.rightDir=0;                    
      } else{                                         // if neither button is pressed move robot forwards
        controlData.leftDir = -1;
        controlData.rightDir = 1;
      }
    }
    else if (!buttonRev.state) {                       // reverse pushbutton pressed
      if (!buttonLeft.state){                          // left pushbutton pressed turn robot left
        controlData.leftDir = 0;
      } else if (!buttonRight.state){                  // right pushbutton pressed turn robot right
        controlData.rightDir=0;
      } else{                                          // if neither button is pressed move robot backwards
        controlData.leftDir = 1;
        controlData.rightDir = -1;
      }
    }
    else {                                            // no input, stop
      controlData.leftDir = 0;
      controlData.rightDir = 0;
    }
      // if drive appears disconnected, update control signal to stop before sending
    if (commsLossCount > cMaxDroppedPackets) {
      controlData.leftDir = 0;
      controlData.rightDir = 0;
    }
    // send control signal to drive
    result = esp_now_send(receiverMacAddress, (uint8_t *) &controlData, sizeof(controlData));
    if (result == ESP_OK) {                           // if sent successfully
      digitalWrite(cStatusLED, 0);                    // turn off communucation status LED
    }
    else {                                            // otherwise
      digitalWrite(cStatusLED, 1);                    // turn on communication status LED
    }
  }
  doHeartbeat();
}

void doHeartbeat() {
  unsigned long curMillis = millis();                 // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat toggle time for the next cycle
    digitalWrite(cHeartbeatLED, !digitalRead(cHeartbeatLED)); // toggle state of LED
  }
}


void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);              // cast pointer to static structure

  unsigned int pressTime = millis();                  // capture current time
  s->state = digitalRead(s->pin);                     // capture state of button
  // if button has been pressed and sufficient time has elapsed
  if ((!s->state && s->lastState == 1) && (pressTime - s->lastPressTime > cDebounceDelay)) {
    s->numberPresses += 1;                            // increment button press counter
    s->pressed = true;                                // set flag for "valid" button press
  }
  s->lastPressTime = pressTime;                       // update time of last state change
  s->lastState = s->state;                            // save last state
}

// callback function for when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == 0)                                       // if empty packet
  {
    return;                                           // return
  }
  memcpy(&inData, incomingData, sizeof(inData));      // store data from drive
#ifdef PRINT_INCOMING
  Serial.printf("%d\n", inData.time);
#endif
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
