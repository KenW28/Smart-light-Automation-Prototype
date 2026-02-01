# Smart-light-Automation-Prototype

A Raspberry Pi–based smart lighting automation prototype that turns a lamp **ON automatically** when someone enters the area and allows the user to turn it **OFF manually** via a button. The system integrates:

- **Meross smart plug** (cloud-controlled)
- **Photoresistor** (state detection: is the light actually on/off?)
- **Ultrasonic sensor (HC-SR04)** (presence/motion detection)
- **ADS1115 ADC over I²C** (photoresistor analog measurement)
- Optional **TCP socket status updates** to a local web dashboard/server

This project demonstrates end-to-end **sensor → decision logic → cloud actuation → feedback validation** using Python on Linux.

---

## High-Level Behavior

### Light OFF → Auto ON
When the lamp is OFF (detected via photoresistor voltage), the Pi continuously measures distance using the ultrasonic sensor. If something approaches closer than a calibrated baseline by a threshold amount, the Pi turns the smart plug ON.

### Light ON → Manual OFF
When the lamp is ON, the system waits for a physical button press. On press, the Pi turns the smart plug OFF.

### Feedback Loop
The photoresistor provides a feedback loop so the software can confirm the lamp state and broadcast state changes to an external server (optional).

---

## Architecture
### Required
- Raspberry Pi (any model with GPIO + I²C)
- Meross Smart Plug (supported by `meross-iot`)
- HC-SR04 ultrasonic sensor
- Photoresistor (LDR)
- ADS1115 ADC module (I²C)
- Momentary pushbutton
- Resistors:
  - **Voltage divider** for HC-SR04 ECHO → Pi GPIO (5V → 3.3V safe)
  - Photoresistor divider network (if you’re not using a module)

### Important Electrical Note
- The HC-SR04 **ECHO pin outputs 5V**.
- Raspberry Pi GPIO pins are **3.3V max**.
- Use a resistor divider on ECHO (example: 1k/2k or 10k/20k) to step 5V down.

## Pin Mapping (BCM)

### Ultrasonic (HC-SR04)
- TRIG: `GPIO23`
- ECHO: `GPIO24` (through resistor divider)

### Button
- Button: `GPIO17` (configured with pull-up)
  - Not pressed: HIGH
  - Pressed: LOW (to GND)

### ADS1115 / I²C
- SDA: Pi SDA
- SCL: Pi SCL
- Photoresistor input to ADS1115 channel:
  - `A0` (as used in the code)

