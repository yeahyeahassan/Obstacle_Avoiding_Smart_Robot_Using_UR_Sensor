# Obstacle-Detecting-Car-using-UR-Sensor
"Navigate safely with our obstacle-detecting car using an Ultrasonic Range Sensor. Arduino-powered, it autonomously avoids obstacles. Join us!"

# 🚗 ESP32 Obstacle Avoiding Robot Car 🚗

## 🔍 Purpose

This project builds upon two previous tutorials: 
- [ESP32 Bluetooth Remote Control Car - DIY](#)
- [ESP32 with HC-SR04 Ultrasonic Sensor](#)

By combining these concepts, you can create your own ESP32-based obstacle avoiding robot car! The car utilizes a servo-mounted ultrasonic sensor to detect objects in its path, both in front and on either side, and an L298N DC motor driver shield to control four geared motors—one for each wheel.

---

## 📦 Components Required

- ESP32 Microcontroller
- HC-SR04 Ultrasonic Sensor
- L298N Motor Driver Shield
- Servo Motor
- 4 Geared Motors
- Wheels
- Chassis
- Power Supply
- Jumper Wires
- Breadboard (optional)

---

## 🛠️ Assembly Instructions

1. **Mount the Components:** 
   - Attach the motors to the chassis and connect the wheels.
   - Secure the ultrasonic sensor on a servo motor to enable it to rotate and scan the area.
   - Place the ESP32 and the L298N motor driver shield on the chassis.

2. **Wiring:**
   - Connect the motors to the L298N motor driver shield.
   - Connect the ultrasonic sensor to the ESP32.
   - Wire the servo motor to the ESP32 for controlling the rotation.
   - Connect the power supply to the motor driver and ESP32.

3. **Programming:**
   - Upload the code to the ESP32 using the Arduino IDE or your preferred platform.
   - Ensure that the appropriate libraries are installed (see below).

---

## 📥 Download Required Library

Make sure to download and install the required library for controlling the motors:

- **RoboJax L298N DC Motor Library:**
  - 📦 [Download Here](https://robojax.com/learn/arduino/robojax_L298N-DC-Motor_library.zip)

---

## 🚀 Getting Started

1. **Install the Library:** 
   - Download the library from the link provided above.
   - Import it into your Arduino IDE.

2. **Upload the Code:**
   - Write or download the obstacle avoidance code for ESP32.
   - Upload the code to your ESP32 board.

3. **Test Your Robot Car:**
   - Power up the robot car.
   - Watch as it navigates around obstacles autonomously!

---

## 🤖 Features

- **Obstacle Detection:** The car detects obstacles in its path using the ultrasonic sensor and avoids collisions by changing direction.
- **Autonomous Navigation:** Moves without manual control, scanning the environment in real-time.
- **Modular Design:** Easily customizable and extendable with additional sensors or functionalities.

---

## 📸 Project Images

![ESP32 Robot Car](https://via.placeholder.com/500x300.png?text=ESP32+Robot+Car)  
*Figure: ESP32 Obstacle Avoiding Robot Car*

---

## 🤝 Contributing

Feel free to contribute to this project! Fork the repository, make your changes, and submit a pull request. Any enhancements, bug fixes, or new ideas are welcome!

---

### 📞 Contact

If you have any questions , please reach me out!

- Email: hsiddique667@gmail.com
- GitHub: [Your GitHub Profile](https://github.com/yeahyeahassan    )


![Thank You](https://img.shields.io/badge/Thank%20You!-blue?style=flat-square&logo=smile)

<!-- Graphical GIF Animation -->
<div class="gif-container" style="text-align: center; margin-bottom: 20px;">
  <img src="https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExYzdob2I4cHdsdWhnbmtmYTBxbnk4cnl2YjZ1bGw5ZGZvMXBwdWc4bSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/00n6TSoGffGTLXSMPO/giphy.gif" alt="Working on it GIF" />
</div>
