//Add necessary libraries
#include <Arduino.h>
#include <esp_now.h>  //To access the esp now functions
#include <WiFi.h>     //To Add Wifi Capabilities on ESP32
#include <iostream>
#include <string>
//#include <stdio.h>
using namespace std;

//save the MAC Address in an array named broadcastAddress;
uint8_t broadcastAddress[] = {0x24, 0xD7, 0xEB, 0x0E, 0xC5, 0x9C}; //MAC address of my receiver

// Peer info
esp_now_peer_info_t peerInfo;



/*define the data types of  the multiple variables structured and
renamed all of it as struct_message*/
typedef struct struct_message {
    int id;
    int position;
    String status;  // Open, Close , Manual
} struct_message;

// Create a struct_message called myData
struct_message sendData, receiveData;


void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
// Formats MAC Address
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
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
  
  Serial.printf(" Board Id : %i ", receiveData.id);
  Serial.printf(" Board Id : %i ", receiveData.position);
  Serial.printf(" Board Id : %s ", receiveData.status);
  if(receiveData.status=="open"){
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  else if (receiveData.status=="close"){
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }
}


void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  
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






}


void send_data(){
  Serial.println("Sending");
  
  //Send data less than or equal 250 bytes via ESP-NOW and returns its status
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
  if (result == ESP_OK) { Serial.println("Sent with success");}
  else { Serial.println("Error sending the data"); }  
}



void loop() {

  if (Serial.available()) { // if there is data comming
    String command = Serial.readStringUntil('\n'); // read string until newline character
    Serial.println("THE INPUT IS " + command );

    if (command == "open")
    {
      digitalWrite(LED_BUILTIN, HIGH); // turn on LED
      Serial.println("Blinds open");
      sendData.id = 1;
      sendData.position = 0;
      sendData.status = command;
      send_data();
    }
    else if (command == "close")
    {
      digitalWrite(LED_BUILTIN, LOW);  // turn off LED
      Serial.println("Blinds are closed");
      sendData.id = 1;
      sendData.position = 0;
      sendData.status = command;
      send_data();
    }
    else if (command == "manual")
    {
      
      while(Serial.available())
      {
        Serial.println("Enter the position you want to set:");
        String psts =Serial.readStringUntil('\n');
        sendData.position = stoi("45");
        
      }
      
      for (int i = 0; i <= 20; i++ )
      {
        delay(200);
        digitalWrite(LED_BUILTIN, LOW); // turn off LED
        delay(200);
        digitalWrite(LED_BUILTIN, HIGH); // turn off LED
       } 
      Serial.println("Blinds are closed");
      sendData.id = 1;
      sendData.status = command;
      
      send_data();
      }
  }





}



