# Blinds Calibration
## Description:
This is the first step the blind should. The blind will not move forward if the calibration is not done.

## High level

### Commuinicate to user:
1. Communicate weather blinds calibration is done or not.
2. Give confirmation that input for low has been accepted
3. Give confirmation that input for high has been accepted.
4. Give confirmation that the calibration is complete.




### Output:
Will set the low point and high point. 

Instead of taking a fixed appraoch we will take setting min and max and divide the range to arrive at mid point or we can ask the user to set midepoint too.

1. Retrun min point.
2. Return max point.
3. Return mid point.


### Input:
On long pressing the roatary encoder switch it enters into calibration mode.

Rotate the encoder to reach the lowest point.

tripple press to confirm the position.

1. Long press to enter calibration.
2. Rotary encoder to get to position.
3. Tripple press to confirm.



### New Developments
Long press: Need to develop function for long press that works with deep sleep.

Tripple Press: Need to develop feature that works with deep sleep.




