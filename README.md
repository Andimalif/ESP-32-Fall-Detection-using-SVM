# Fall Detection System Using ESP32 and MPU9250

This repository contains the code for a fall detection system that uses an ESP32 microcontroller and MPU9250 sensors. The system processes sensor data to detect falls in real time and sends alerts via Blynk.

## Features

- **Real-time Fall Detection**: Utilizes dual MPU9250 sensors to accurately detect falls using sensor fusion.
- **Machine Learning Integration**: Implements a Support Vector Machine (SVM) classifier to predict fall events.
- **WiFi Connectivity**: Uses the ESP32 to connect to a Wi-Fi network for remote monitoring.
- **Blynk Integration**: Sends alerts and notifications through the Blynk platform for real-time assistance.

## Setup and Installation

1. **Hardware Requirements**:
   - ESP32 microcontroller
   - Two MPU9250 sensors
   - Buzzer for fall alerts
   - WiFi network for connectivity

2. **Software Requirements**:
   - Arduino IDE installed
   - Relevant Arduino libraries:
     - `Wire`
     - `MPU9250_asukiaaa`
     - `eloquent_tinyml` for machine learning
     - `BlynkSimpleEsp32` for Blynk integration

3. **Installation**:
   - Clone this repository:  
     ```bash
     git clone https://github.com/yourusername/fall-detection-system.git
     ```
   - Open the project in Arduino IDE.
   - Update the Wi-Fi credentials and Blynk authentication token in the code.
   - Upload the code to the ESP32.

## Calibration

The system includes a calibration routine for the accelerometer, gyroscope, and magnetometer. The calibration runs automatically during startup and helps in improving the accuracy of fall detection.

## Usage

- Once powered on and connected to Wi-Fi, the system starts monitoring for falls.
- If a fall is detected (using the trained SVM model), the buzzer will sound, and a notification will be sent via Blynk.
- Users can monitor and receive alerts on the Blynk app.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please fork the repository and use a feature branch. Pull requests are warmly welcome.

## Author

- [Andi Muhammad Alif](https://github.com/Andimalif)

## Acknowledgments

- Special thanks to the creators and maintainers of the libraries used in this project.
