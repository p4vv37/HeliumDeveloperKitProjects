Simple project: Low power solution for sending data to LoRaWan based on  https://github.com/RAKWireless/WisBlock/blob/master/examples/RAK4630/communications/LoRa/LoRaWAN/Low_Power_Example.md

Makefile is the complicated part, I could not make PlatformIO work with the board, ArduinoIDE is too primitive, so I had to create a complicated makefile that download all demendencies, compiles and uploads program to the board.

Run:
```
make
```
To compile and upload. 
Run:
```
make reset
```
To clean temporary files.
Run:
```
make clean
```
To delete downloaded 3rd party files and temporary files.

Based on: https://github.com/RAKWireless/WisBlock/blob/master/examples/RAK4630/solutions/Environment_Monitoring/Environment_Monitoring.ino