//Add necessary libraries
#include <Arduino.h>
#include <esp_now.h>  //To access the esp now functions
#include <WiFi.h>     //To Add Wifi Capabilities on ESP32
#include <iostream>
#include <string>

#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);


//sleep
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 3

// Encoder

#define ENCODER_CLK 32
#define ENCODER_DT  33
#define ENCODER_SW  25

RTC_DATA_ATTR int counter = 0;
RTC_DATA_ATTR int counterprev = 0;
RTC_DATA_ATTR int position = 0; // defining position of stepper as 1,2,3
String command;

int received = 0;

//#include <stdio.h>
using namespace std;

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

  int xounter = 0;
  static uint8_t prevNextCode = 0;
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  void readEncoder()
  {
    int dtValue = digitalRead(ENCODER_DT);
    // int clValue = digitalRead(ENCODER_CLK);
    int clValue = 0;
    Serial.println("Encoder Position");
    Serial.println(dtValue, clValue);
    prevNextCode <<= 2;
    if (dtValue) prevNextCode |= 0x02;
    prevNextCode &= 0x0f;
    int out= rot_enc_table[( prevNextCode & 0x0f )];
    Serial.println(out);
}

// Get the counter value, disabling interrupts.
// This make sure readEncoder() doesn't change the value
// while we're reading it.
int getCounter() {
  int result;
  noInterrupts();
  result = counter;
  interrupts();
  return result;
}

void resetCounter() {
  noInterrupts();
  counter = 0;
  interrupts();
}






void setup() {
  
  //led
  pinMode(LED_BUILTIN, OUTPUT);

  //Encoder
    // Initialize encoder pins
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), readEncoder, FALLING);




  
  //Set the baud rate for serial communication with ESP
  Serial.begin(921600);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);//Starts the wifi

  // Init ESP-NOW and returns its status
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  // Set up Serial Monitor
  Serial.begin(921600);
 
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
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


myStepper.setSpeed(5);
}


void loop() {

int change = 0;
int countertemp = 0;

    countertemp = getCounter();
    if (countertemp!= counterprev && countertemp>=0 && countertemp <=128)
    {
        /*
    Each counter step is equal to 4 steps of stepper.
    So to make 90 degree turn we need 512 steps which is 128 steps
     counter= 0 is position 0
     counter = 64 is pisition 1
     counter =128 is pisition 2
    */
        change = countertemp - counterprev;
        if (change>0) change = 1;
        if (change<0)change = -1;
        myStepper.step(change * 8);

        if (countertemp <=0)
            position = 0;
        else if (countertemp ==64)
            position = 1;
        else if (countertemp >=128)
            position = 2;
        

        counter = counterprev +change;
        counterprev = counter;
        countertemp = counter;

        /* send data to Hub about new position */
        sendData.id = 2;
        sendData.position = position;
        sendData.steps = countertemp * 8;
        sendData.status = "Manual";
        send_data();
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
            change = 64 - countertemp;
            
            
            // resetting the counter to right position
            counter = 64;
            countertemp = 64;
            counterprev = 64;
            position = 1;

            
            sendData.position = 1;
            sendData.steps = 64 * 8;
        }
        else if (command == "close")
        {
            digitalWrite(LED_BUILTIN, LOW); // turn off LED
            if (countertemp > 64)
             {
               change = 128 - countertemp;
                counter = 128;
                countertemp = 128;
                counterprev = 128;
                position = 2;
                sendData.position = 2;
                sendData.steps = 128 * 8;
             }
             else if (countertemp <= 64)
              {
                change = 0 - countertemp;
                counter = 0;
                countertemp = 0;
                counterprev = 0;
                position = 0;
                sendData.position = 0;
                sendData.steps = 0;
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



        myStepper.step(change * 8);
        Serial.printf("Blinds %s",command);
        sendData.id = 1;
        sendData.status = command;
        
        if (serialavailable!=0)
        {
        send_data();
        serialavailable = 0;
        }

        if(received==1)
        {
          Serial.println("Executed received data");
          received = 0;
        }
    }


digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);

//esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
//esp_deep_sleep_start();

}
