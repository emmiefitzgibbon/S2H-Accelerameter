#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <string.h>  // For strlen
// #include "arduinoFFT.h"

// ADXL345 I2C Address Configuration:
// - Address 0x53 (default): SDO/ALT ADDRESS pin (Pin 12) is GROUNDED
// - Address 0x1D (alternate): SDO/ALT ADDRESS pin (Pin 12) is HIGH/VCC
// Change this value to match your hardware configuration:
#define ADXL345_I2C_ADDRESS 0x1D // Change to 0x1D for alternate address

    // arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
    // const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
    // const double samplingFrequency = 538;
    // double vReal[samples];
    // double vImag[samples];
    // int fftIndex = 0;

    // float data_array_x[100];
    // float data_array_y[100];
    // float data_array_z[100];
    unsigned long lastTimestamp;
unsigned long counter;

/* Assign a unique ID to this sensor at the same time */
// Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//  SPI mode (uncomment if using SPI)
//  Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(10, 11, 12, 13,
//  0xE5); I2C mode - address is set by ADXL345_I2C_ADDRESS define above
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(ADXL345_I2C_ADDRESS);
char outputChar[50];

void displaySensorDetails(void) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void) {
  Serial.print("Data Rate:    ");

  switch (accel.getDataRate()) {
  case ADXL345_DATARATE_3200_HZ:
    Serial.print("3200 ");
    break;
  case ADXL345_DATARATE_1600_HZ:
    Serial.print("1600 ");
    break;
  case ADXL345_DATARATE_800_HZ:
    Serial.print("800 ");
    break;
  case ADXL345_DATARATE_400_HZ:
    Serial.print("400 ");
    break;
  case ADXL345_DATARATE_200_HZ:
    Serial.print("200 ");
    break;
  case ADXL345_DATARATE_100_HZ:
    Serial.print("100 ");
    break;
  case ADXL345_DATARATE_50_HZ:
    Serial.print("50 ");
    break;
  case ADXL345_DATARATE_25_HZ:
    Serial.print("25 ");
    break;
  case ADXL345_DATARATE_12_5_HZ:
    Serial.print("12.5 ");
    break;
  case ADXL345_DATARATE_6_25HZ:
    Serial.print("6.25 ");
    break;
  case ADXL345_DATARATE_3_13_HZ:
    Serial.print("3.13 ");
    break;
  case ADXL345_DATARATE_1_56_HZ:
    Serial.print("1.56 ");
    break;
  case ADXL345_DATARATE_0_78_HZ:
    Serial.print("0.78 ");
    break;
  case ADXL345_DATARATE_0_39_HZ:
    Serial.print("0.39 ");
    break;
  case ADXL345_DATARATE_0_20_HZ:
    Serial.print("0.20 ");
    break;
  case ADXL345_DATARATE_0_10_HZ:
    Serial.print("0.10 ");
    break;
  default:
    Serial.print("???? ");
    break;
  }
  Serial.println(" Hz");
}

void displayRange(void) {
  Serial.print("Range:         +/- ");

  switch (accel.getRange()) {
  case ADXL345_RANGE_16_G:
    Serial.print("16 ");
    break;
  case ADXL345_RANGE_8_G:
    Serial.print("8 ");
    break;
  case ADXL345_RANGE_4_G:
    Serial.print("4 ");
    break;
  case ADXL345_RANGE_2_G:
    Serial.print("2 ");
    break;
  default:
    Serial.print("?? ");
    break;
  }
  Serial.println(" g");
}

void setup(void) {
#ifndef ESP8266
  while (!Serial)
    ; // for Leonardo/Micro/Zero
#endif
  Serial.begin(2000000);  // Increased to 2 Mbps for higher data rate
  //  Serial.println("Accelerometer Test"); Serial.println("");

  // ESP32-S3 I2C pin configuration (if needed)
  // Default I2C pins on ESP32-S3: SDA=GPIO8, SCL=GPIO9 (may vary by board)
  // Uncomment and adjust if your board uses different pins:
  // Wire.begin(SDA_PIN, SCL_PIN);

  /* Initialise the sensor */
  Serial.print("Looking for ADXL345 at I2C address 0x");
  Serial.println(ADXL345_I2C_ADDRESS, HEX);

  if (!accel.begin(ADXL345_I2C_ADDRESS)) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no ADXL345 detected at address 0x");
    Serial.print(ADXL345_I2C_ADDRESS, HEX);
    Serial.println(" ... Check your wiring!");
    Serial.println("Make sure SDO/ALT ADDRESS pin is configured correctly:");
    Serial.println("  - For address 0x53: SDO pin should be GROUNDED");
    Serial.println("  - For address 0x1D: SDO pin should be HIGH/VCC");
    while (1)
      ;
  }

  Serial.print("ADXL345 found at address 0x");
  Serial.println(ADXL345_I2C_ADDRESS, HEX);

  /* Set the range to whatever is appropriate for your project */

  accel.setRange(ADXL345_RANGE_16_G);
  //   accel.setRange(ADXL345_RANGE_8_G);
  //   accel.setRange(ADXL345_RANGE_4_G);
  //   accel.setRange(ADXL345_RANGE_2_G);

  accel.setDataRate(ADXL345_DATARATE_3200_HZ);
  // default = ADXL345_DATARATE_0_10_HZ

  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0x80);

  pinMode(9, INPUT);

  /* Display some basic information on this sensor */
  //  displaySensorDetails();

  /* Display additional settings (outside the scope of sensor_t) */
  //  displayDataRate();
  //  displayRange();
  //  Serial.println("");
  counter = 0;
  lastTimestamp = millis();
  //  Serial.println(lastTimestamp);

  //  for(int i=0; i<samples; ++i){
  //    vReal[i] = 0;
  //    vImag[i] = 0;
  //  }
  
  //  Serial.print("register 0x38 = ");
  //  Serial.println(accel.readRegister(0x38));
}

void loop(void) {
  /* Get a new sensor event - optimized for high speed with binary format */
  while (true) {
    // Removed digitalRead(9) check for maximum speed
    sensors_event_t event;
    unsigned long eventTime = micros();  // Use microseconds for better time resolution
    accel.getEventXYZ(&event);

    // Binary format: 16 bytes per packet
    // [4 bytes: timestamp (uint32_t, microseconds)] [4 bytes: X (float)] [4 bytes: Y (float)] [4 bytes: Z (float)]
    uint8_t packet[16];
    
    // Pack timestamp (uint32_t, little-endian) - microseconds since boot
    packet[0] = (uint8_t)(eventTime & 0xFF);
    packet[1] = (uint8_t)((eventTime >> 8) & 0xFF);
    packet[2] = (uint8_t)((eventTime >> 16) & 0xFF);
    packet[3] = (uint8_t)((eventTime >> 24) & 0xFF);
    
    // Pack X, Y, Z as floats (little-endian)
    float x = event.acceleration.x;
    float y = event.acceleration.y;
    float z = event.acceleration.z;
    
    memcpy(&packet[4], &x, 4);
    memcpy(&packet[8], &y, 4);
    memcpy(&packet[12], &z, 4);
    
    // Send binary packet
    Serial.write(packet, 16);

      //      counter++;
      //      unsigned long curTime = millis();
      //      if(curTime-lastTimestamp >= 1000){
      //      //  if(counter == 1000){
      //        Serial.print("counter = ");
      //        Serial.println(counter);
      //        counter = 0;
      //        lastTimestamp = curTime;
      //        Serial.println(curTime);
      //      }
      //      break;
    }
  }

  //  data_array_x[counter] = event.acceleration.x;
  //  data_array_y[counter] = event.acceleration.y;
  //  data_array_z[counter] = event.acceleration.z;

  //  if(counter == 100){
  //    for(int i=0; i<100; ++i){
  //        Serial.print("{\"X\": "); Serial.print(data_array_x[i]);
  //        Serial.print(",  "); Serial.print("\"Y\": ");
  //        Serial.print(data_array_y[i]); Serial.print(",  ");
  //        Serial.print("\"Z\": "); Serial.print(data_array_z[i]);
  //        Serial.print("}"); Serial.println("");
  //    }
  //    counter = 0;
  //  }

  //  if(fftIndex < samples){
  //    vReal[fftIndex] = event.acceleration.z;
  //    fftIndex++;
  //  }
  //  else{//reach sample size, calculate fft and clear array
  //    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //    double peakFreq = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //    Serial.print("major freq = ");
  //    Serial.println(peakFreq, 4);
  //    fftIndex = 0;
  //  }

  // b'X: 9.77  Y: 0.08  Z: -2.43  m/s^2 \r\n'

  /* Display the results (acceleration is measured in m/s^2) */
  //  Serial.print("{\"X\": "); Serial.print(event.acceleration.x);
  //  Serial.print(",  "); Serial.print("\"Y\": ");
  //  Serial.print(event.acceleration.y); Serial.print(",  ");
  //  Serial.print("\"Z\": "); Serial.print(event.acceleration.z);
  //  Serial.print("}"); Serial.println(""); String outputString =
  //  "{\"X\":"+String(event.acceleration.x,
  //  2)+",\"Y\":"+String(event.acceleration.y,
  //  2)+",\"Z\":"+String(event.acceleration.z, 2)+"}\n"; char outputChar[35];
  //  outputString.toCharArray(outputChar, 35);
  //  Serial.write(outputChar, 35);

