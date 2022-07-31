//Add necessary libraries
#include <esp_now.h>  //To access the esp now functions
#include <WiFi.h>     //To Add Wifi Capabilities on ESP32

//save the MAC Address in an array named broadcastAddress;
uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0xC7, 0x9C, 0x77}; //MAC address of my receiver

/*define the data types of  the multiple variables structured and
renamed all of it as struct_message*/
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  String d;
  bool e;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// function called when data is sent to print its status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if(status!=ESP_NOW_SEND_SUCCESS ){send_data();}
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);
  Serial.print("String: ");
  Serial.println(myData.d);
  Serial.print("Bool: ");
  Serial.println(myData.e);
  Serial.println();
}

void setup() {
  //Set the baud rate for serial communication with ESP
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);//Starts the wifi

  // Init ESP-NOW and returns its status
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //call the function OnDataSent after sending ESPNOW data
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo; //initialize and assign the peer information as a pointer to an addres
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); //copy the value of  broadcastAddress with 6 bytes to peerInfo.peer_addr
  peerInfo.channel = 0;  //channel at which the esp talk. 0 means undefined and data will be sent on the current channel. 1-14 are valid channels which is the same with the local device 
  peerInfo.encrypt = false; //not encrypted
  
  
  //Add the device to the paired device list 
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ 
    Serial.println("Failed to add peer");
    return;
  }

esp_now_register_recv_cb(OnDataRecv); //call the function OnDataRecv after receiving ESPNOW data
send_data(); 
}

void loop() {}

void send_data(){
  Serial.println("Sending");
  // Set values to send
  strcpy(myData.a, "THIS IS A CHAR"); //save "THIS IS A CHAR" to variable a of my "data" defined earlier
  myData.b = random(1,20); //save a random value
  myData.c = 1.2; //save a float
  myData.d = "ESP32"; //save a string
  myData.e = false; //save a bool

  //Send data less than or equal 250 bytes via ESP-NOW and returns its status
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) { Serial.println("Sent with success");}
  else { Serial.println("Error sending the data"); }  
}


