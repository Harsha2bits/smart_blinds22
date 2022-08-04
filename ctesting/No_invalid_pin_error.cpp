//Add necessary libraries
#include <Arduino.h>
#include <esp_now.h>  //To access the esp now functions
#include <WiFi.h>     //To Add Wifi Capabilities on ESP32
#include <iostream>
#include <string>
#include <AccelStepper.h>
#include <Encoder.h>
#include "AiEsp32RotaryEncoder.h"


#include "esp_sleep.h"
#include "esp_attr.h"
#include "rom/rtc.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/uart_reg.h"
#include "soc/timer_group_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"



//#include <stdio.h>
using namespace std;

const int stepsPerRevolution = 2038;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

#define FULLSTEP 4
#define HALFSTEP 8

//Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

AccelStepper myStepper(FULLSTEP, IN1, IN3, IN2, IN4);


//sleep
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 5
// #define wakeup_level 0

// Encoder

#define ROTARY_ENCODER_A_PIN 33
#define ROTARY_ENCODER_B_PIN 32
#define ROTARY_ENCODER_BUTTON_PIN 25


#define ROTARY_ENCODER_STEPS 4
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);




//Register Memory Variables
RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR int counterprev = 0;
RTC_DATA_ATTR int position = 0; // defining position of stepper as 1,2,3
RTC_DATA_ATTR int motorPosition = 0;
RTC_DATA_ATTR int wakeup_level = 0;
int interrupt = 0;
static unsigned long lastEncoderChanged = 0;

// Other variables
String command;
int received = 0;


//Communication Related

//save the MAC Address in an array named broadcastAddress;
uint8_t broadcastAddress[] = {0x24, 0xD7, 0xEB, 0x10, 0xE0, 0xC0}; //MAC address of my receiver

// Peer info
esp_now_peer_info_t peerInfo;

/*define the data types of  the multiple variables structured and
renamed all of it as struct_message*/
typedef struct struct_message {
    int id;
    int position;
    int steps;
    String status; // Open, Close , Manual
} struct_message;

// Create a struct_message called myData
struct_message sendData, receiveData;




//Communication functions


void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
// Formats MAC Address
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}


void send_data(){
  Serial.println("Sending");
  
  //Send data less than or equal 250 bytes via ESP-NOW and returns its status
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
  if (result == ESP_OK) { Serial.println("Sent with success");}
  else { Serial.println("Error sending the data"); }  
}



// function called when data is sent to print its status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  //if(status!=ESP_NOW_SEND_SUCCESS ){send_data();}
}


//will change later
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receiveData, incomingData, sizeof(receiveData));
   char macStr[18];
  formatMacAddress(mac, macStr, 18);

  // Send Debug log message to the serial port
  Serial.printf("Received message from: %s\n",macStr);
  
  Serial.printf(" Board Id : %i \n ", receiveData.id);
  Serial.printf(" Blind Position : %i \n", receiveData.position);
  Serial.printf(" Blind Steps : %i \n", receiveData.steps);
  Serial.printf(" Blind Status : %s \n", receiveData.status);
  received = 1;
  return;
  }

 //Encoder function

 int getCounter()
{
    return (int)rotaryEncoder.readEncoder() ;
}

void setCounter(int i)
{
  rotaryEncoder.setEncoderValue(i);
}

void rotary_onButtonClick()
{
    static unsigned long lastTimePressed = 0;
    if (millis() - lastTimePressed < 200)
        return;
    lastTimePressed = millis();

    Serial.print("Resetting Counter Value ");
    rotaryEncoder.setEncoderValue(51);
}



void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}




//wakeup reason function
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}





// Setup function


void setup() {
  
  //led
  pinMode(LED_BUILTIN, OUTPUT);
  //rtc_gpio_deinit(GPIO_NUM_33);

  //Encoder
    // Initialize encoder pins
  //pinMode(ENCODER_CLK, INPUT);
  //pinMode(ENCODER_DT, INPUT);
  //pinMode(ENCODER_SW, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), readEncoder, FALLING);
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    rotaryEncoder.setBoundaries(0, 102, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setAcceleration(0);
    //rotaryEncoder.setEncoderValue(counterprev);

    
    if(esp_sleep_get_wakeup_cause()== ESP_SLEEP_WAKEUP_EXT0)
    {
      Serial.println("wakeup because of encoder");
      rotaryEncoder.setEncoderValue(counterprev + 1);
    } // set default to 92.1 MHz
      else rotaryEncoder.setEncoderValue(counterprev);
  
    myStepper.setCurrentPosition(motorPosition);
    
    myStepper.setMaxSpeed(1000.0);
	  myStepper.setAcceleration(50.0);
	  myStepper.setSpeed(200);
	  //myStepper.moveTo(2038);
    //myStepper.run();

    // Set the baud rate for serial communication with ESP
    Serial.begin(921600);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA); // Starts the wifi

    // Init ESP-NOW and returns its status
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("Error initializing ESP-NOW");
      return;
  }

 
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  esp_now_register_recv_cb(OnDataRecv);
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }




//code for sleep interrupts

esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, wakeup_level);
print_wakeup_reason();

//delay(3000);
}






void loop() {

int change = 0;
int countertemp=0;

    countertemp = getCounter();
    if (countertemp!= counterprev && countertemp>=0 && countertemp <=102)
    {

      interrupt = 1;

      Serial.println("Entered encoder loop");
      
      /*
  Each counter step is equal to 4 steps of stepper.
  So to make 90 degree turn we need 512 steps which is 128 steps
   counter= 0 is position 0
   counter = 64 is pisition 1
   counter =128 is pisition 2
  */
      // change = countertemp - counterprev;
      lastEncoderChanged = millis();

      motorPosition = countertemp * 10;

      if (countertemp <= 0)
        position = 0;
      else if (countertemp >= 50 || countertemp <= 52)
        position = 1;
      else if (countertemp >= 102)
        position = 2;

      counterprev = countertemp;

      /* send data to Hub about new position */
      sendData.id = 2;
      sendData.position = position;
      sendData.steps = motorPosition;
      sendData.status = "Manual";

      Serial.println(motorPosition);
      Serial.println(myStepper.currentPosition());
    }

    /* Manual Input Code */
    int serialavailable = Serial.available();
    if (serialavailable!=0 || received == 1)
    {                                                  // if there is data comming
        
        
        if (serialavailable!=0){
        command = Serial.readStringUntil('\n'); // read string until newline character
        Serial.println("THE INPUT IS " + command);
        }
        
        
        if (received==1) command = receiveData.status;

        if (command == "open")
        {
            digitalWrite(LED_BUILTIN, HIGH); // turn on LED
            
            setCounter(51);
            motorPosition = 51 * 10;
            position = 1;
            sendData.position = 1;
            sendData.steps = motorPosition;
        }
        else if (command == "close")
        {
            digitalWrite(LED_BUILTIN, LOW); // turn off LED
            if (countertemp > 51)
             {

                setCounter(102);
                motorPosition = 102 * 10;                
                position = 2;
                sendData.position = 2;
                sendData.steps = motorPosition;
             }
             else if (countertemp <= 51)
              {
                setCounter(0);
                motorPosition = 0;
                countertemp = 0;
                counterprev = 0;
                position = 0;
                sendData.position = 0;
                sendData.steps = motorPosition;
              }
        }
        else if (command == "manual")
        {

            while (Serial.available())
            {
                Serial.println("Enter the position you want to set:");
                String psts = Serial.readStringUntil('\n');
                sendData.position = stoi("45");
            }

            for (int i = 0; i <= 20; i++)
            {
                delay(200);
                digitalWrite(LED_BUILTIN, LOW); // turn off LED
                delay(200);
                digitalWrite(LED_BUILTIN, HIGH); // turn off LED
            }
            Serial.println("Blinds are manual");
            sendData.status = command;

        }



        //myStepper.step(change * 8);
        


          // resetting the flags
          if (serialavailable != 0)
          {
            Serial.printf("Blinds %s", command);
            sendData.id = 2;
            sendData.status = command;
            send_data();
            serialavailable = 0;
        }

        if(received==1)
        {
          Serial.println("Executed received data");
          received = 0;
        }
    }



          myStepper.moveTo(motorPosition);
          //myStepper.run();
          
          
          
           if(myStepper.distanceToGo()!=0)
          {
            myStepper.run();
          }
          else
          {
              digitalWrite(IN1, LOW);
              digitalWrite(IN2, LOW);
              digitalWrite(IN3, LOW);
              digitalWrite(IN4, LOW);
          }
// esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
// esp_deep_sleep_start();


if(interrupt==1 || myStepper.distanceToGo()!=0 )
    {
    if (millis()-lastEncoderChanged > 7000 )
    {
      Serial.print(millis() - lastEncoderChanged > 7000);
      Serial.println("Going to Sleep as no inpur received");
      //wakeup_level=!gpio_get_level(GPIO_NUM_25);
      Serial.printf("Wakeup_level is:%i \n", wakeup_level);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
      Serial.println("This should not be printed");

    }
    else
      return;
    }
    else 
    { 
      Serial.println("Regular Sleep as no inpur received");
      //wakeup_level=!gpio_get_level(GPIO_NUM_33);
      //Serial.printf("Wakeup_level is:%i \n", wakeup_level);

      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
      
    }




}
