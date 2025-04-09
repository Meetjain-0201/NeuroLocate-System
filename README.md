# Brain Biopsy Pen - Precision Measurement Tool

## 📖 Overview
A precision measurement device that:
- Measures linear distances using a magnetic rotary encoder
- Stores target values for X/Y axes
- Compares actual vs target measurements
- Provides visual/audible feedback

## 🛠 Hardware Components
| Component               | Specification           |
|-------------------------|-------------------------|
| Microcontroller         | Arduino-compatible      |
| Rotary Encoder          | AS5600 Magnetic (I2C)  |
| Display                 | SSD1306 OLED (128x32)   |
| Buttons                 | 2x Tactile (D1/D2)      |
| Feedback                | Piezo Buzzer + LEDs     |

## 🔌 Pin Configuration
| Signal          | Arduino Pin |
|----------------|------------|
| Button 1       | 2          |
| Button 2       | 3          |
| Green LED      | 7          |
| Red LED        | 6          |
| Buzzer         | 5          |
| I2C SDA        | A4         |
| I2C SCL        | A5         |

## ⚙️ Core Functionality
1. **Measurement Modes**
   - Y-axis measurement
   - X-axis measurement (after 5s delay)
   
2. **User Interface**
   - OLED display shows:
     - Target values (T1/T2)
     - Measured distances (D1/D2)
   - Button-controlled value input

3. **Feedback Systems**
   - Visual indicators (LEDs)
   - Audible signals (buzzer)
   - Low voltage warning

## 📊 Technical Specifications
| Parameter          | Value       |
|--------------------|-------------|
| Measurement Range  | 0-99.9cm    |
| Resolution         | 0.1cm       |
| Wheel Diameter     | 15mm        |
| Update Rate        | 100ms       |
| Operating Voltage  | 5V DC       |

## 🚀 Getting Started
1. **Upload Code**
```bash
arduino-cli compile --fqbn arduino:avr:uno BrainBiopsyPen.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno
```

2. **Calibration**
- Ensure proper magnet alignment with AS5600
- Verify wheel diameter setting in code (`diameter` variable)

3. **Operation**
- Set targets using buttons
- Move device to take measurements
- System provides completion feedback

## 🛠️ Maintenance
- **Troubleshooting**
  - No display: Check I2C connections
  - No rotation detection: Verify magnet placement
  - Erratic values: Ensure stable power supply

- **Calibration**
  - Update `diameter` for different wheel sizes
  - Adjust `pressDuration` for button sensitivity

## 📝 Notes
- For accurate measurements:
  - Maintain consistent surface contact
  - Avoid strong magnetic interference
  - Keep battery voltage above 2.0V

## 📜 License
MIT License - Free for educational and personal use
