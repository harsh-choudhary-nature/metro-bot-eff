# Autonomous Arduino Bot with Q-Learning

## Project Overview
This project explores the application of reinforcement learning, specifically Q-learning, in developing an autonomous Arduino-based bot. The bot simulates a train that stops at designated stations using an ultrasonic sensor to detect stations and IR sensors to follow a pre-defined path. The bot autonomously pauses at each station for a set interval before resuming movement. 

## Objective
The goal of this project is to demonstrate the integration of Q-learning with low-cost hardware (Arduino and basic sensors) to create an educational platform that helps students learn about autonomous navigation, decision-making, and machine learning.

## Key Features
- **Autonomous Navigation**: The bot uses an ultrasonic sensor to detect designated stations and IR sensors for path following.
- **Q-Learning Algorithm**: The bot is trained using the Q-learning algorithm to autonomously stop at stations and resume movement.
- **Educational Tool**: Designed to introduce students to AI and robotics concepts in a hands-on environment.
  
## Hardware Used
- **Arduino Uno**: Main microcontroller for controlling the bot.
- **Ultrasonic Sensor (HC-SR04)**: Detects the station.
- **IR Sensors**: For path following.
- **DC Motors**: For movement.

## Software Requirements
- **Arduino IDE**: To upload the code to the Arduino board.
- **Python**: For the Q-learning algorithm and to control the bot via a serial connection.
- **Libraries**: `pySerial`, `numpy`, `matplotlib` (if visualizing data), etc.
