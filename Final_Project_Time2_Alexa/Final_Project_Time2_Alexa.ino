#include <Arduino.h>
#include <ESP32Encoder.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Universial Variables
  int state = 1; //state used for state transition
  int AIN1 = 21; //Initialize AIN1 Pin
  int AIN2 = 32; ////Initialize AIN2 Pin

// Encorder Variables
  ESP32Encoder encoder;
  int currentCount = 0;

//Timer Interrupt variable setup
  hw_timer_t * timer = NULL;
  volatile SemaphoreHandle_t timerSemaphore;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t isrCounter = 0; //counter for # of interrupts
  //Timer Interrupt functions
  void IRAM_ATTR onTimer(){
    // Increment the counter
    portENTER_CRITICAL_ISR(&timerMux);
    isrCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
    // Give a semaphore that we can check in the loop
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
  }

// Alexa Variables & Functions
  int alexaPin = 27;    
  int alexaVal = 0; 
  int oldAlexaVal = 0;
  
// Transition 1 Change Motor Direction Variables & Functions
  // Debounce Variables (Max one press per 2 seconds )
  unsigned long lastDebounceTime = 0; 
  unsigned long debounceDelay = 2000000;
  //setup Buttons
  struct Button {
      const uint8_t PIN;
      uint32_t numberKeyPresses;
      bool pressed;
  };
  Button MotorButton = {14, 0, false};//Initialize Button
  
  //GPIO Interrupt functions for the button with debouncing
  void IRAM_ATTR isr(void* arg) {
      Button* s = static_cast<Button*>(arg);
      s->numberKeyPresses += 1;
      s->pressed = true;
  }
  void IRAM_ATTR isr() {
      //debouncing
      if ((micros() - lastDebounceTime) > debounceDelay){
         MotorButton.numberKeyPresses += 1;
         MotorButton.pressed = true;
      }
      lastDebounceTime = micros(); //reset timer
  }

// Clock Variables
  // Replace with your network credentials
  const char* ssid     = "PorQueFi";
  const char* password = "CanIUseTheRoomFor20Minutes?";
  // Define NTP Client to get time
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP);
  
  // Variables to save date and time
  String formattedDate;
  String timeStamp;
  String dawn = "9:00:00";
  String dusk = "17:00:00"; 

// Error Conditions:
  int startTime =0;
  int currentTime =0;

//Setup loop 
void setup() {
// Universal Setup
  Serial.begin(115200);
  
// Wifi and clock setup
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

// Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(-28800);
  
//Counting Setup
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;
  // Attache pins for use as encoder pins
  encoder.attachFullQuad(33, 15);
  // set starting count value after attaching
  encoder.setCount(0);
  // clear the encoder's raw count and set the tracked count to zero
  encoder.clearCount();


// Create semaphore to inform us when the timer has fired 
  timerSemaphore = xSemaphoreCreateBinary(); 
  timer = timerBegin(0, 80, true); //Create a timer
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true); 
  // Define Interrupt Condition for 2Hz
  // for 1Hz blink, turn on LED 2 of every 4 interrupts
  timerAlarmWrite(timer, 1000000, true); 
  timerAlarmEnable(timer);  // Start an alarm
  
//forwards-motor-turning Setup
  ledcAttachPin(AIN1, 0);
  ledcSetup(0, 12000, 8); 
  pinMode(AIN1, OUTPUT);
  
//Button Setup
  pinMode(MotorButton.PIN, INPUT_PULLUP);
  attachInterrupt(MotorButton.PIN, isr, FALLING);
 
  
//Backwards-motor-turning Setup
  ledcAttachPin(AIN2, 1);
  ledcSetup(1, 12000, 8); 
  pinMode(AIN2, OUTPUT);

//Alexa Control setup
  pinMode(alexaPin, INPUT_PULLUP);
  alexaVal = digitalRead(alexaPin);
  oldAlexaVal = alexaVal;

// Error out setup 
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
   // Clock
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    while(!timeClient.update()) {
      timeClient.forceUpdate();
    }
    
    // Extract date
    formattedDate = timeClient.getFormattedDate();
    int splitT = formattedDate.indexOf("T");
    // Extract time
    timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
    //Serial.print("HOUR: ");
    //Serial.println(timeStamp);
  }
  switch (state) {
    case 1:{
      // Idle state when the window blinds is open
      // When the button is pressed switch to state2
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      encoder.clearCount();
      alexaVal = digitalRead(alexaPin);
      
      if (MotorButton.pressed) {
        MotorButton.pressed = false;
        state = 2;
        startTime =millis();
      }
      else if (timeStamp == dusk){
        state = 2;
        startTime =millis();
      }
      else if (alexaVal!=oldAlexaVal){
        if (alexaVal==0){
          state = 2;
          startTime =millis();
        }
      }
      oldAlexaVal =alexaVal;
      break; 
     }
    case 2 :{
      // Actuation state closing window blinds
      ledcWrite(1, 0);
      ledcWrite(0, 255);
      
      // After reaching closed position, switch to state3
      currentCount = (int32_t)encoder.getCount();
      Serial.println(currentCount);
      if (abs(currentCount)>4000) {
        state = 3;
      }
      currentTime =millis();
      if ((currentTime-startTime)>10000){
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        state =5;
      }
      break;
     }
    case 3:{
      // Idle state when the window blinds is closed
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      encoder.clearCount();
      alexaVal = digitalRead(alexaPin);
      
      // When the button is pressed switch to state4
      if (MotorButton.pressed) {
        MotorButton.pressed = false;
        state = 4;
        startTime =millis();
      }
      // When it's 9AM open the blinds
      else if (timeStamp == dawn){
        state = 4;
        startTime =millis();
      }
      else if (alexaVal!=oldAlexaVal){
        if (alexaVal==1){
          state = 4;
          startTime =millis();
        }
      }
      oldAlexaVal =alexaVal;
      break; 
     }
    case 4 :{
      //Actuation state opening window blinds
      ledcWrite(0, 0);
      ledcWrite(1, 255);
   
      // Switch back to state 1 when button is pressed
      currentCount = (int32_t)encoder.getCount();
      Serial.println(currentCount);
      if (abs(currentCount)>4000) {
        state = 1;
      }
      currentTime =millis();
      if ((currentTime-startTime)>10000){
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        state =5;
      }
      break;
    }
    case 5 :{
      digitalWrite(LED_BUILTIN, HIGH); 
    }
  }
}
