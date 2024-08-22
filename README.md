# Hip_exoskeleton_v1.4_control_software

## Python version

## Matlab version

# How to setup the software

## Prerequisites

### 1. Teensy 4.1

1. Install the proper version of Arduino IDE from https://www.arduino.cc/en/software
2. Follow the instructions from https://www.pjrc.com/teensy/td_download.html
3. In the "LIBRARY MANAGER" section, search for "MovingAverager" by Ian Carey and install it.

### 2. Remote PC

1. Install the proper distribution of anaconda from https://www.anaconda.com/download/success
2. In anaconda, create a virtual environment and install the following libraries:
    1. pyqtgraph
    2. pyqt5
    3. pyserial
    4. pyqt-tools
    5. qt5-applications
    6. qt5-tools

3. Install the proper distribution of Visual Studio Code from https://code.visualstudio.com/download
4. Download all the required files (from this GitHub repository):
    1. High-level controller (Reinforcement Learning-based Controller)
    2. Arduino code (for Teensy 4.1)
    3. Raspberry Pi Zero OS image
    4. Exoskeleton Bluetooth module (Itsy-Bitsy)
    5. Remote PC Bluetooth module (Itsy-Bitsy)
    6. Control Software & Graphical User Interface

# About

This repository contains all the files related to the development of the GUI for the Hip Exoskeleton GUI.

Each of the files correspond to a different version of the GUI and track the development proccess of it.

For convinience, at the beggining of each of the files, a general description of the code and the changes made is given.

### The GUI operates along other pices of code stored and running in different elements of hardware. Without these other elements (hardware and software) the GUI cannot work properly.

## Brief overview of the whole system (software & hardware interactions)

There are different software elements running into different devices:

- Software
    1. High-level Controller [Python]
    2. Data handling (Receive & Transmit) and drive the motors [C++/Arduino]
    3. IMU data adquisition [Raspberry Pi OS]
    4. Data transmit & recieve (Exoskeleton) [C++/Arduino]
    5. Data transmit & recieve (Remote PC) [C++/Arduino]
    6. GUI [Python]

- Hardware
    1. PC
    2. Teensy 4.1
    3. Raspberry Pi Zero W
    4. Itsy-Bitsy nRF52840 (Exoskeleton)
    5. Itsy-Bitsy nRF52840 (Remote PC)
    6. PC

![Software Setup 1](Figures/Software_setup_01.png)

![Software Setup 2](Figures/Software_setup_02.png)

# Requiremets for running the GUI

1. The code running in Teensy 4.1 must contain a proper function to gather, encode and send that data through serial protocol to the Bluetooth module on the exoskeleton.
2. The Bluetooth module on the exoskeleton must have uploaded the corresponding code as well as the Bluetooth device on the Remote PC.

# Running the GUI

###  To run the code you need to install the following libraries:

 - PyQtGraph library: https://www.pyqtgraph.org/
 - Serial

### Running the GUI is very easy, just need to execute the code (Python interpreter needed).

1. Connect the Itsy-Bitsy nRF52840 (Remote PC) Bluetooth device to the Remote PC using the proper cable.
2. Execute the code (using the Python interpreter).
3. Select the COM port where the Bluetooth module is connected.
4. Click the "Connect Bluetooth" button.

The most recent version of the GUI is "HipExo_GUI_v11.py".

### The following window shows how the most recent version of the GUI looks like.

![GUI v.6](Figures/Graphical_Control_Software.png)

Youtube video: https://youtu.be/w6m0pwIsVQM

### The GUI has the following features:

- Display in real-time up to 6 different signals
    - Left IMU pitch angle
    - Right IMU pitch angle
    - Left motor's desired and actual torque
    - Right motor's desired and actual torque

- Log data (csv file)
- Detect the available COM ports
- Select the COM port for the communication
