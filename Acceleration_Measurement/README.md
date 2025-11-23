## Setup Instructions

### Arduino Library Setup
1. Install the Adafruit_ADXL345 library from https://github.com/adafruit/Adafruit_ADXL345/tree/master
2. Replace the installed library files with the custom version in this repo (`Adafruit_ADXL345-1.3.1/`)
3. The custom library uses `Wire` for I2C communication (configured for ESP32-S3)

### Hardware Configuration
- Connect the data wire to pin 9 on your Arduino/ESP32-S3
- For dual sensor setup:
  - Sensor 1: I2C address 0x53 (SDO pin grounded)
  - Sensor 2: I2C address 0x1D (SDO pin high/VCC)

### Arduino Code Configuration
- Update `ADXL345_I2C_ADDRESS` in `accelerometer_arduino_slave.ino`:
  - `0x53` for default address (SDO grounded)
  - `0x1D` for alternate address (SDO high)

### Python Setup and Running
1. Create and activate virtual environment (if not already done):
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

2. Install required packages:
   ```bash
   pip install numpy matplotlib pyserial
   ```

3. Update COM ports in `accelerometer_display_realtime.py`:
   - Set `PORT_SENSOR_1` and `PORT_SENSOR_2` to your serial ports
   - On macOS/Linux: `/dev/cu.usbmodemXXXX` or `/dev/ttyUSB0`
   - On Windows: `COM3`, `COM4`, etc.

4. Run the monitoring script:
   ```bash
   source venv/bin/activate
   python accelerometer_display_realtime.py
   ```

### Notes
- The script supports both single and dual sensor configurations
- Data is saved to `acc_data.txt` when you stop the script (Ctrl+C)
- Make sure to change the COM port numbers in the Python code to match your setup