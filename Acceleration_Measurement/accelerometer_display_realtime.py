#!/usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import json
import time
import threading
import sys
import csv
import os
import struct
from multiprocessing import Process
from serial_control import serial_control

class AccelerationAnalyzer:

    def __init__(self, port1, port2=None, sensor1_name="Sensor_0x53", sensor2_name="Sensor_0x1D", save_only_sensor1=False):
        self.control1 = serial_control(port1)
        self.sensor1_name = sensor1_name
        self.port1 = port1

        # Sensor 1 data
        self.all_data1 = []
        self.all_data_timestamps1 = []
        self.buffer1 = []
        self.buffer_timestamps1 = []
        self.baseline_acc1 = None
        self.is_baseline_valid1 = False

        # Flag to only save sensor 1 data (for performance testing)
        self.save_only_sensor1 = save_only_sensor1

        # Sensor 2 data (if provided)
        if port2:
            self.control2 = serial_control(port2)
            self.sensor2_name = sensor2_name
            self.port2 = port2
            self.all_data2 = []
            self.all_data_timestamps2 = []
            self.buffer2 = []
            self.buffer_timestamps2 = []
            self.baseline_acc2 = None
            self.is_baseline_valid2 = False
        else:
            self.control2 = None
        
        if self.save_only_sensor1:
            print("[Performance] Only saving data from Sensor 1 (Sensor 2 will be received but not saved)")

        # Common settings
        self.buffer_size = 500
        self.frame_count = 0

        # Trial status tracking: list of (timestamp, status) tuples
        # Timestamps are stored in sensor time scale (not system time)
        self.trial_status_history = []
        self.current_trial_status = None
        self.time_offset = None  # Offset between sensor time and system time (sensor_time = system_time - offset)

        # Print instructions
        print("\n" + "="*60)
        print("Trial Status Control:")
        print("  Enter a message to save trial status")
        print("  Press Ctrl+C to stop and save data")
        print("="*60 + "\n")

        # Start terminal input handler in a separate thread
        self.input_thread = threading.Thread(target=self.handle_terminal_input, daemon=True)
        self.input_thread.start()

        self.loop()

    def loop(self):
        try:
            self.last_time = time.time()
            while True :
                # Receive data from both sensors - optimized tight loop
                self.receive_data()
                self.frame_count = self.frame_count + 1
                # Reduced plot frequency to every 200 frames for less overhead
                if self.frame_count == 200:
                    if not self.is_baseline_valid1:
                        self.baseline_calculate(1)
                    if self.control2 and not self.is_baseline_valid2:
                        self.baseline_calculate(2)
                    self.frame_count = 0
                    self.graph_plot()

        except KeyboardInterrupt:
            self.control1.finish()
            if self.control2:
                self.control2.finish()
            print(self.frame_count)
            self.save_data()
            print("data saved!")
        print("End...")

    def receive_data(self):
        # Receive from sensor 1
        try:
            input_bytes = self.control1.receive()
            if input_bytes:
                self.processing_data(input_bytes, 1)
        except Exception as e:
            print(f'{self.sensor1_name} data error = ', e)

        # Receive from sensor 2 (if available)
        if self.control2:
            try:
                input_bytes = self.control2.receive()
                if input_bytes:
                    self.processing_data(input_bytes, 2)
            except Exception as e:
                print(f'{self.sensor2_name} data error = ', e)

    def processing_data(self, input_bytes, sensor_num):
        try:
            if input_bytes is None or len(input_bytes) != 16:
                return
            
            # Parse binary packet: [4 bytes: timestamp (uint32_t, microseconds)] [4 bytes: X (float)] [4 bytes: Y (float)] [4 bytes: Z (float)]
            timestamp, x, y, z = struct.unpack('<Ifff', input_bytes)  # Little-endian: uint32_t, float, float, float
            
            accel_data = [x, y, z]
            
            # Timestamp is in microseconds from Arduino micros()
            timestamp_us = timestamp
            
            # Calculate time offset on first data point to sync with system time
            # This allows trial_status timestamps to match sensor timestamps
            if self.time_offset is None:
                system_time = time.time()
                # Store offset: sensor_time_us = (system_time - offset) * 1000000
                # So when we have a sensor timestamp in us, we can convert: system_time = (sensor_time_us / 1000000) + offset
                self.time_offset = system_time - (timestamp_us / 1000000.0)
                print(f"[Time Sync] Offset calculated: sensor_time = system_time - {self.time_offset:.3f}")
                print(f"[Time Sync] First sensor timestamp: {timestamp_us} us (Arduino micros)")

            if sensor_num == 1:
                self.all_data1.append(accel_data)
                self.all_data_timestamps1.append(timestamp_us)  # Store in microseconds
                self.buffer1.append(accel_data)
                self.buffer_timestamps1.append(timestamp_us)
                if len(self.buffer1) > self.buffer_size:
                    self.buffer1.pop(0)
                    self.buffer_timestamps1.pop(0)
            elif sensor_num == 2:
                # Only save sensor 2 data if save_only_sensor1 flag is False
                if not self.save_only_sensor1:
                    self.all_data2.append(accel_data)
                    self.all_data_timestamps2.append(timestamp_us)  # Store in microseconds
                # Still update buffer for plotting even if not saving
                self.buffer2.append(accel_data)
                self.buffer_timestamps2.append(timestamp_us)
                if len(self.buffer2) > self.buffer_size:
                    self.buffer2.pop(0)
                    self.buffer_timestamps2.pop(0)
        except Exception as e:
            print(f'error processing data from sensor {sensor_num}: {input_bytes}')

    def baseline_calculate(self, sensor_num):
        if sensor_num == 1:
            acc_array = np.array(self.buffer1)
            self.baseline_acc1 = np.mean(acc_array, axis=0)
            print(f'{self.sensor1_name} baseline:', self.baseline_acc1)
            self.is_baseline_valid1 = True
        elif sensor_num == 2:
            acc_array = np.array(self.buffer2)
            self.baseline_acc2 = np.mean(acc_array, axis=0)
            print(f'{self.sensor2_name} baseline:', self.baseline_acc2)
            self.is_baseline_valid2 = True

    def freq_plot(self):
        plt.clf()

        acc_array = np.array(self.buffer_fft)
        acc_len = acc_array.shape[0]
        # print(acc_array.shape)
        acc_abs = np.linalg.norm(acc_array, axis=1)
        # acc_abs = [np.sqrt(acc_array[i][0]*acc_array[i][0] + acc_array[i][1]*acc_array[i][1] + acc_array[i][2]*acc_array[i][2]) for i in range(acc_array.shape[0])]
        # print(acc_abs.shape)

        acc_fft = np.fft.fft(acc_abs)[1:acc_len>>1]
        acc_freq = np.fft.fftfreq(acc_len, 1/acc_len)[1:acc_len>>1]

        plt.plot(acc_freq, 2.0 / acc_len * np.abs(acc_fft))
        plt.xlim(0, 400)
        plt.ylim(0, 60)
        plt.xticks(np.arange(0, 400, 20))
        plt.pause(0.001)

    def graph_plot(self):
        plt.clf() # clear previous figure

        if self.control2:
            # Dual sensor display
            # Sensor 1 plots
            acc_array1 = np.array(self.buffer1)
            if len(acc_array1) > 0:
                if type(self.baseline_acc1) == type(acc_array1):
                    for i in range(acc_array1.shape[0]):
                        acc_array1[i] = np.round((acc_array1[i] - self.baseline_acc1), 2)
                acc_abs1 = np.linalg.norm(acc_array1, axis=1)
                acc_rms1 = np.round(np.sqrt(np.mean(acc_abs1**2)), 2)
                acc_rms_axis1 = np.sqrt(np.mean(acc_array1**2, axis=0))

                # Sensor 2 plots
                acc_array2 = np.array(self.buffer2)
                if len(acc_array2) > 0:
                    if type(self.baseline_acc2) == type(acc_array2):
                        for i in range(acc_array2.shape[0]):
                            acc_array2[i] = np.round((acc_array2[i] - self.baseline_acc2), 2)
                    acc_abs2 = np.linalg.norm(acc_array2, axis=1)
                    acc_rms2 = np.round(np.sqrt(np.mean(acc_abs2**2)), 2)
                    acc_rms_axis2 = np.sqrt(np.mean(acc_array2**2, axis=0))

                # Plot Sensor 1 (top)
                plt.subplot(221)
                if len(acc_array1) > 0:
                    x1 = np.arange(len(acc_array1))
                    plt.plot(x1, acc_array1[:, 0], x1, acc_array1[:, 1], x1, acc_array1[:, 2])
                    plt.legend(['x', 'y', 'z'])
                    plt.axis([0, self.buffer_size, -50, 50])
                    plt.ylabel("acceleration")
                    plt.title(f'{self.sensor1_name} - MAX X={np.max(acc_array1[:, 0]):.2f} Y={np.max(acc_array1[:, 1]):.2f} Z={np.max(acc_array1[:, 2]):.2f}')

                # Plot Sensor 1 magnitude
                plt.subplot(223)
                if len(acc_abs1) > 0:
                    x1 = np.arange(len(acc_abs1))
                    plt.plot(x1, acc_abs1)
                    plt.axis([0, self.buffer_size, 0, 100])
                    plt.xlabel("frame")
                    plt.ylabel("acceleration")
                    plt.title(f'{self.sensor1_name} - RMS: {acc_rms1}')

                # Plot Sensor 2 (top right)
                plt.subplot(222)
                if len(acc_array2) > 0:
                    x2 = np.arange(len(acc_array2))
                    plt.plot(x2, acc_array2[:, 0], x2, acc_array2[:, 1], x2, acc_array2[:, 2])
                    plt.legend(['x', 'y', 'z'])
                    plt.axis([0, self.buffer_size, -50, 50])
                    plt.ylabel("acceleration")
                    plt.title(f'{self.sensor2_name} - MAX X={np.max(acc_array2[:, 0]):.2f} Y={np.max(acc_array2[:, 1]):.2f} Z={np.max(acc_array2[:, 2]):.2f}')

                # Plot Sensor 2 magnitude
                plt.subplot(224)
                if len(acc_abs2) > 0:
                    x2 = np.arange(len(acc_abs2))
                    plt.plot(x2, acc_abs2)
                    plt.axis([0, self.buffer_size, 0, 100])
                    plt.xlabel("frame")
                    plt.ylabel("acceleration")
                    plt.title(f'{self.sensor2_name} - RMS: {acc_rms2}')

                # Removed constant print to keep terminal clear for user input
        else:
            # Single sensor display (original)
            acc_array = np.array(self.buffer1)
            if len(acc_array) > 0:
                if type(self.baseline_acc1) == type(acc_array):
                    for i in range(acc_array.shape[0]):
                        acc_array[i] = np.round((acc_array[i] - self.baseline_acc1), 2)
                acc_abs = np.linalg.norm(acc_array, axis=1)
                acc_rms = np.round(np.sqrt(np.mean(acc_abs**2)), 2)
                acc_rms_axis = np.sqrt(np.mean(acc_array**2, axis=0))
                # Removed constant print to keep terminal clear for user input

                # wave
                plt.subplot(211)
                x = np.arange(acc_array.shape[0])
                plt.plot(x, acc_array[:, 0], x, acc_array[:, 1], x, acc_array[:, 2])
                plt.legend(['x', 'y', 'z'])
                plt.axis([0, self.buffer_size, -50, 50])
                plt.ylabel("acceleration")
                display_text = 'MAX X='+str(np.max(acc_array[:, 0]))+' Y='+str(np.max(acc_array[:, 1]))+' Z='+str(np.max(acc_array[:, 2]))+'\n'+\
                               'MIN X='+str(np.min(acc_array[:, 0]))+' Y='+str(np.min(acc_array[:, 1]))+' Z='+str(np.min(acc_array[:, 2]))
                plt.title(display_text)
                #Spectrum
                plt.subplot(212)
                plt.plot(x, acc_abs)
                plt.axis([0, self.buffer_size, 0, 100])
                plt.xlabel("frame")
                plt.ylabel("acceleration")
                display_text = 'Max ACC = '+str(acc_rms)
                plt.title(display_text)

        plt.pause(.001)

    def handle_terminal_input(self):
        """Handle terminal input commands in a separate thread"""
        while True:
            try:
                # Prompt for input
                user_input = input("Enter message: ").strip()
                
                # Only process non-empty input
                if user_input:
                    message = user_input
                    # Convert system time to sensor time scale (in microseconds)
                    if self.time_offset is not None:
                        system_time = time.time()
                        sensor_timestamp_us = (system_time - self.time_offset) * 1000000  # Convert to microseconds
                        self.trial_status_history.append((sensor_timestamp_us, message))
                        self.current_trial_status = message
                        print(f"[Trial Status] Set to: '{message}' at sensor time {sensor_timestamp_us:.1f} us")
                    else:
                        # Store with system time, will be converted later when offset is known
                        system_time = time.time()
                        self.trial_status_history.append((system_time, message))
                        self.current_trial_status = message
                        print(f"[Trial Status] Set to: '{message}' (will sync when data arrives)")
                # Ignore empty input
                    
            except EOFError:
                # Handle case where stdin is closed
                break
            except Exception as e:
                print(f"[Trial Status] Error reading input: {e}")
                break

    def get_trial_status_for_timestamp(self, timestamp):
        """Get the most recent trial_status that occurred before or at the given timestamp"""
        if not self.trial_status_history:
            return None
        
        # Find the most recent status change that occurred before or at this timestamp
        status = None
        for status_timestamp, status_value in self.trial_status_history:
            if status_timestamp <= timestamp:
                status = status_value
            else:
                break
        
        return status

    def get_next_filename(self):
        """Find the next available filename to avoid overwriting existing data"""
        base_name = 'acc_data'
        extension = '.csv'
        
        # Check if base file exists
        if not os.path.exists(f'{base_name}{extension}'):
            return f'{base_name}{extension}'
        
        # Find the next available number
        counter = 1
        while os.path.exists(f'{base_name}({counter}){extension}'):
            counter += 1
        
        return f'{base_name}({counter}){extension}'

    def save_data(self):
        # Convert any trial_status entries that were stored before time_offset was known
        if self.time_offset is not None:
            converted_history = []
            for ts, status in self.trial_status_history:
                # If timestamp looks like system time (very large), convert it to sensor time (microseconds)
                if ts > 1000000000:  # System time is Unix timestamp (seconds since 1970)
                    # Convert system time to sensor time: sensor_time_us = (system_time - offset) * 1000000
                    sensor_ts_us = (ts - self.time_offset) * 1000000
                    converted_history.append((sensor_ts_us, status))
                else:
                    # Already in sensor time scale (microseconds)
                    converted_history.append((ts, status))
            self.trial_status_history = converted_history
        
        # Collect all data rows with timestamps for interleaving
        all_rows = []
        
        # Collect sensor 1 data
        if len(self.all_data1) > 0:
            for i in range(len(self.all_data1)):
                baseline = self.baseline_acc1 if self.baseline_acc1 is not None else np.array([0, 0, 0])
                timestamp = self.all_data_timestamps1[i]
                trial_status = self.get_trial_status_for_timestamp(timestamp)
                acc = np.round((self.all_data1[i] - baseline), 3)
                
                row = [
                    self.sensor1_name,
                    timestamp,
                    acc[0],
                    acc[1],
                    acc[2],
                    trial_status if trial_status is not None else ''
                ]
                all_rows.append((timestamp, row))
        
        # Collect sensor 2 data
        if self.control2 and len(self.all_data2) > 0:
            for i in range(len(self.all_data2)):
                baseline = self.baseline_acc2 if self.baseline_acc2 is not None else np.array([0, 0, 0])
                timestamp = self.all_data_timestamps2[i]
                trial_status = self.get_trial_status_for_timestamp(timestamp)
                acc = np.round((self.all_data2[i] - baseline), 3)
                
                row = [
                    self.sensor2_name,
                    timestamp,
                    acc[0],
                    acc[1],
                    acc[2],
                    trial_status if trial_status is not None else ''
                ]
                all_rows.append((timestamp, row))
        
        # Sort all rows by timestamp to interleave sensors
        all_rows.sort(key=lambda x: x[0])
        
        # Get next available filename
        filename = self.get_next_filename()
        
        # Write to CSV file
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Write header
            writer.writerow(['sensor', 'time', 'acc_x', 'acc_y', 'acc_z', 'trial_status'])
            # Write data rows (extract just the row data, not the timestamp tuple)
            writer.writerows([row for _, row in all_rows])
        
        print(f"Data saved to {filename}")


if __name__ == "__main__":
    # Configuration for dual sensor setup:
    # Change these to your COM ports:
    # On Windows: 'COM3', 'COM4', etc.
    # On macOS/Linux: '/dev/cu.usbmodem2101', '/dev/ttyUSB0', etc.

    PORT_SENSOR_1 = '/dev/cu.usbmodem101'  # Sensor at address 0x1D
    PORT_SENSOR_2 = None   # Not used - single sensor setup

    # Single sensor setup - using Sensor 0x1D
    spec = AccelerationAnalyzer(
        port1=PORT_SENSOR_1,
        port2=PORT_SENSOR_2,
        sensor1_name="Sensor_0x1D",
        sensor2_name="Sensor_0x53",
        save_only_sensor1=True  # Only saving Sensor 1 (0x1D) data
    )
