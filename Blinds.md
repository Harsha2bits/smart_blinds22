# Automating Blinds Tilt

- [Automating Blinds Tilt](#automating-blinds-tilt)
  - [Code So Far documentation](#code-so-far-documentation)
  - [Features to add](#features-to-add)
    - [1. Calibration Step](#1-calibration-step)
    - [2. MQTT Integration with ESP32.](#2-mqtt-integration-with-esp32)
    - [3. Google Home Integration](#3-google-home-integration)
  - [Known Issues](#known-issues)
    - [1. ESP32 gets stuck after running for sometime.](#1-esp32-gets-stuck-after-running-for-sometime)
    - [2. ESP32](#2-esp32)


## Code So Far documentation


## Features to add
### 1. Calibration Step
**Description:** We need to setup a calibration step to ensure the blinds start and close at right position.  
**Cost of Failure(High):** If we do not get this right there is a risk of blinds spoiling.  
**Requirements:**  
* Should blink inbuilt lights till the calibration step is setup.
* Enter calibration step once the encoder button is long pressed.
* Go to start position (Tilt down) and double press to confirm it.
* Go to end position ( Tilt up) and double press to confirm.
* Blink led continiously for 5 seconds to confirm that the calibration is complet.
* Integrate with existing code and make sure it works in deep sleep mode.

### 2. MQTT Integration with ESP32.
**Description:** Current code communicates through espnow. The protocall allows us to communicate between 2 esp32 devices but we need to setup a method to communicate it with web applications.
**Cost of Failure(Medium):** Will impact communication
**Requirements:**
* Need to create hub with 2 esp32 to communicate in espnow and MQTT receiver.
* Integration with Home assistant


### 3. Google Home Integration
**Description:** We want the blinds to work on voice commands.  
**Cost of Failure(Low):** Will loose voice funcitons.  
**Requirements**
* Take the blinds control to cloud. 
* Integrate the cloud with google assistant.
* See if scenes can be created on google.



## Known Issues
### 1. ESP32 gets stuck after running for sometime.
 **Cause:** The issue is because of inadequate power supply. I have noticed that when I supply power from USB the issue never occurs.
 **Fix:** Submit external power supply form battery.

### 2. ESP32  