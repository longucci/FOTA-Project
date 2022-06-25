# Project name: Firmware-Over-The-Air
# Description: 
This project is used to upload a new file to the STM32F411 microcontroller through UART communication protocol using module bluetooth HC-05 based on the bootloader principle 
# Preparation:
- Hardware: 1x STM32F411, 2x module bluetooth HC-05, 1x USB-to-TTL UART
- Software: STM32CUBEIDE, Hercules Terminal
# How to use:
Firstly, you choose a file .BIN that you want to upload to the microcontroller from the computer and check its size. Then send the command "firmware size: x bytes" in the Hercules terminal, where x is the size of your .BIN file. Then right click on the terminal and choose "send file" command to starting sending your .BIN file. When the terminal confirm that the BIN file is successfully uploaded, it will send a signal on the terminal screen - which is "Uploaded Firmware". Done !!!!  
