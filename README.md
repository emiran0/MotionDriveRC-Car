# MotionDriveRC-Car

**MotionDriveRC-Car** is the receiver-side module of a wearable gesture-controlled RC system. It receives real-time tilt data from the glove module and translates it into steering and throttle commands for a dual-motor RC car using an ESP32, motor driver, and servo setup.

The system supports low-latency wireless communication and includes enhancements such as Dynamic Speed Adjustment (DSA) and a Smart Steering System (SSS) for smoother handling and turn compensation.

---

## Features

- Real-time control of motor speed and direction based on glove tilt
- Servo-based steering controlled by mapped X-axis gesture data
- Dynamic Speed Adjustment (DSA) for smooth throttle transitions
- Smart Steering System (SSS) that adjusts motor power during sharp turns
- Low-latency wireless control using ESP-NOW
- Modular and extensible design for any two-motor vehicle

---

## Hardware Components

- ESP32 microcontroller (receiver)
- TB6612FNG or L298N motor driver
- 2 × 6V 1360 RPM DC motors (rear-wheel drive)
- SG90 servo motor (for front-wheel steering)
- 2 × 3.7V 2500mAh Li-ion batteries connected in series (7.4V total)
- 3 × 2A mini adjustable voltage regulators
- Custom 3D-printed chassis and steering system

---

## Communication Protocol: ESP-NOW

- Peer-to-peer wireless communication with the glove module
- MAC address of the glove is hardcoded into the car firmware
- Operates on 2.4 GHz with no external Wi-Fi router required
- Data packet includes two integer values: steering angle and throttle power
- Stable connection up to 70–80 meters outdoors and 40–50 meters indoors
- Sampling rate: approximately 50–100 Hz
- Latency: typically under 15 ms

---

## System Behavior

- Steering is controlled using a servo motor that interprets tilt as angle
- Throttle is applied via PWM output to both motors equally or asymmetrically
- Smart Steering adjusts the balance between left/right motors on tight turns
- Dynamic Speed Adjustment smooths acceleration to prevent skidding

---

## Getting Started

### Prerequisites

- Arduino IDE or PlatformIO
- ESP32 development board
- Required libraries:
  - ESP32Servo
  - esp_now
  - WiFi
  - Motor driver control libraries (if applicable)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/emiran0/MotionDriveRC-Car.git
   ```
2. Open the firmware in your IDE.

3. Flash the code to the ESP32 board on the car.

4. Connect all hardware components as described in the wiring section.

5. Set the MAC address of the glove in the receiver code to match your transmitter.

---

## Usage

1. Power on the car and the glove module.
2. The car will begin listening for gesture data via ESP-NOW.
3. As the glove tilts:

- Steering angle is updated using the servo motor
- Throttle level is applied to the DC motors
- Smart steering compensates for high-angle turns

4. Adjust motor voltage or PWM ranges to fine-tune response.

---

## Demo

Add a demo GIF or link here once available. Example:

![MotionDriveRC-Car Demo](media/demo_car.gif)

---

## Performance

| Feature                   | Value                      |
| ------------------------- | -------------------------- |
| Communication latency     | < 15 ms                    |
| Steering angle range      | 35° – 145° via servo       |
| Max speed (throttle PWM)  | Configurable (default 150) |
| Range (wireless)          | ~70–80 meters outdoor      |
| Supported input frequency | ~50–100 Hz                 |

---

## Contributing

Contributions are welcome!  
If you'd like to add new features (e.g. obstacle detection, multi-car race mode), open an issue or submit a pull request.

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Related Projects

- [MotionDriveRC-Glove](https://github.com/emiran0/MotionDriveRC-Glove)
