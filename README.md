#  Gesture Controlled Omnidirectional Rover

![Repo Size](https://img.shields.io/github/repo-size/hiteshbhoyar03/Gesture-controlled-rover)
![Last Commit](https://img.shields.io/github/last-commit/hiteshbhoyar03/Gesture-controlled-rover)
![License](https://img.shields.io/github/license/hiteshbhoyar03/Gesture-controlled-rover)
![GitHub stars](https://img.shields.io/github/stars/hiteshbhoyar03/Gesture-controlled-rover?style=social)

---
<p align="center">
<img width="30%" src="https://github.com/hiteshbhoyar03/Gesture-controlled-rover/blob/main/rover%20receiver%20circuit.png">
<img width="30%" src="https://github.com/hiteshbhoyar03/Gesture-controlled-rover/blob/main/rover%20receiver%20circuit.png">
</p>

An open-source robotics project featuring a fully in-house designed **gesture-controlled omnidirectional rover**.  
Built from scratch — from the rover chassis to the handheld transmitter — the project demonstrates advanced control using **mecanum wheels**, **gesture sensors**, and **wireless communication**.

Subsystems include:

- Rover Drive (mecanum wheels, motor drivers)  
- Transmitter with gesture + joystick inputs  
- Four-way ultrasonic obstacle detection  
- GPS for location tracking  
- NRF24L01 wireless communication  
- OLED display for telemetry  

---

##  Rover

<a href="rover/">
<img align="right" width="30%" src="https://github.com/hiteshbhoyar03/Gesture-controlled-rover/blob/main/rover%20receiver%20circuit.png">
</a>

The rover uses **mecanum wheels** for full omnidirectional mobility.  
It receives control packets wirelessly, processes ultrasonic + GPS data, and sends telemetry back to the transmitter.

- **Drive System**: 4 × mecanum wheels with motor drivers  
- **Sensors**: 4 × ultrasonic (front, back, left, right), GPS module  
- **Wireless**: NRF24L01 receiver  
- **MCU**: Arduino Mega  

[View Rover Code](https://github.com/hiteshbhoyar03/Gesture-controlled-rover/tree/main/rover)

---

##  Transmitter

<a href="transmitter/">
<img align="right" width="30%" src="https://github.com/hiteshbhoyar03/Gesture-controlled-rover/blob/main/transmitter%20circuit%20with%20oled%20updated.png">
</a>

The handheld controller combines **gesture recognition** with **dual joysticks** for hybrid control.  
An **OLED screen** provides live status, GPS data, and obstacle warnings.

- **Gesture Input**: MPU6050  
- **Manual Input**: Dual joysticks, mode switches  
- **Display**: SSH1106 OLED  
- **Wireless**: NRF24L01 transmitter  
- **MCU**: Arduino Nano / ESP32  

[View Transmitter Code](https://github.com/hiteshbhoyar03/Gesture-controlled-rover/tree/main/transmitter)

---

##  Communication

- **Link**: NRF24L01 2.4GHz  
- **TX → Rover**: Gesture/joystick data, Toggle & button switches  
- **Rover → TX**: GPS coordinates, obstacle data  

---

##  Project Status

- Chassis design & fabrication complete  
- Transmitter hardware assembled  
- Basic gesture + joystick control working  
- Obstacle avoidance integration  
- GPS + telemetry feedback finalized  

---

##  Tools Used

- **Arduino IDE** – Firmware development  
- **RF24 library** – Wireless communication  
- **Adafruit libraries** – MPU6050 + SSH1106 support  
- **TinyGPS++** – GPS parsing  

---

##  Author

**Hitesh Bhoyar**  
Robotics | Embedded Systems | Wireless Communication  
[GitHub →](https://github.com/hiteshbhoyar03)

> For educational use. Fork, reuse, or reach out if you’re exploring robotics & embedded systems!

---

##  License

Licensed under the [MIT License](LICENSE)

---

##  Learning Goals

- Apply **gesture recognition** to robotics  
- Explore **mecanum wheel kinematics**  
- Practice **wireless data transfer with NRF24L01**  
- Develop **bidirectional telemetry**  
- Showcase **end-to-end embedded system design**  

---

##  Project Purpose

An academic and portfolio project to:

- Demonstrate **gesture-based rover control**  
- Design and build a **custom chassis + transmitter**  
- Integrate **GPS, ultrasonic sensors, and telemetry**  
- Practice **robust embedded firmware** (Arduino)  
- Explore **real-world robotics system design**  

This project highlights skills in **embedded systems, wireless communication, and robotic mobility**, built entirely in-house for **learning and demonstration purposes**.
