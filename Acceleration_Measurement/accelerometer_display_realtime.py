#!/usr/bin/env python
# encoding: utf-8

import numpy as np
import time
import threading
import sys
import csv
import os
import struct
from serial_control import serial_control
try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

class AccelerationAnalyzer:

    def __init__(self, port1, port2, sensor1_name="Sensor_0x53", sensor2_name="Sensor_0x1D", save_only_sensor1=False):
        # Print available serial ports
        print("\n" + "="*60)
        print("Serial Port Detection:")
        if list_ports:
            available_ports = list_ports.comports()
            if available_ports:
                print("Available serial ports:")
                for port in available_ports:
                    print(f"  - {port.device}: {port.description}")
            else:
                print("No serial ports found")
        else:
            print("serial.tools.list_ports not available - install pyserial to see available ports")
        print("="*60)
        
        # Print which ports are being used
        print(f"\n[Sensor 1] Using port: {port1} ({sensor1_name})")
        if port2:
            print(f"[Sensor 2] Using port: {port2} ({sensor2_name})")
        else:
            print(f"[Sensor 2] No port specified (single sensor mode)")
        print()
        
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
        if port2 and str(port2).strip():
            try:
                self.control2 = serial_control(port2)
                self.sensor2_name = sensor2_name
                self.port2 = port2
                self.all_data2 = []
                self.all_data_timestamps2 = []
                self.buffer2 = []
                self.buffer_timestamps2 = []
                self.baseline_acc2 = None
                self.is_baseline_valid2 = False
                print(f"[Sensor 2] Successfully connected to {port2}")
            except Exception as e:
                error_msg = str(e)
                if "Resource busy" in error_msg or "could not open port" in error_msg:
                    print(f"[ERROR] Failed to connect to Sensor 2 port {port2}")
                    print(f"        Port is busy - another program may be using it.")
                    print(f"        Try closing other serial monitor programs or check with: lsof | grep {port2}")
                    print(f"        Sensor 2 data will NOT be collected.")
                else:
                    print(f"[ERROR] Failed to connect to Sensor 2 port {port2}: {e}")
                self.control2 = None
                self.all_data2 = []
                self.all_data_timestamps2 = []
                self.buffer2 = []
                self.buffer_timestamps2 = []
                self.baseline_acc2 = None
                self.is_baseline_valid2 = False
        else:
            self.control2 = None
            # Initialize empty lists even if port2 is not provided (for safety)
            self.all_data2 = []
            self.all_data_timestamps2 = []
            self.buffer2 = []
            self.buffer_timestamps2 = []
            self.baseline_acc2 = None
            self.is_baseline_valid2 = False
        
        if self.save_only_sensor1:
            print("[Performance] Only saving data from Sensor 1 (Sensor 2 will be received but not saved)")

        # Common settings
        self.buffer_size = 500
        self.frame_count = 0
        self.packets_received = 0
        self.last_status_time = time.time()
        self.packets_dropped = 0  # Track potential dropped packets
        self.last_timestamp = None

        # Trial status tracking: list of (timestamp, status) tuples
        # Timestamps are stored in sensor time scale (not system time)
        self.trial_status_history = []
        self.current_trial_status = None
        self.time_offset = None  # Offset between sensor time and system time (sensor_time = system_time - offset)
        
        # Timestamp synchronization between sensors
        self.sensor1_first_timestamp = None
        self.sensor2_first_timestamp = None
        self.timestamp_offset_s2_to_s1 = None  # Offset to convert sensor 2 timestamps to sensor 1's time scale
        
        # Lock to prevent status messages from interfering with input
        self.print_lock = threading.Lock()
        
        # Lock for thread-safe data access (though list.append is mostly atomic)
        self.data_lock = threading.Lock()
        
        # Flag to stop data collection threads
        self.running = True

        # Print instructions
        print("\n" + "="*60)
        print("Trial Status Control:")
        print("  Enter a message to save trial status")
        print("  Press Ctrl+C to stop and save data")
        print("="*60 + "\n")

        # Start terminal input handler in a separate thread
        self.input_thread = threading.Thread(target=self.handle_terminal_input, daemon=True)
        self.input_thread.start()
        
        # Start data reception threads for each sensor
        self.sensor1_thread = threading.Thread(target=self.receive_sensor_data, args=(1,), daemon=True)
        self.sensor1_thread.start()
        
        if self.control2:
            self.sensor2_thread = threading.Thread(target=self.receive_sensor_data, args=(2,), daemon=True)
            self.sensor2_thread.start()

        self.loop()

    def loop(self):
        try:
            self.last_time = time.time()
            baseline_calculated = False
            
            while True:
                # Data reception is now handled by separate threads
                # Main loop just handles baseline calculation and status updates
                
                # Calculate baseline once after collecting some data
                if not baseline_calculated and len(self.buffer1) >= 100:
                    self.baseline_calculate(1)
                    if self.control2 and len(self.buffer2) >= 100:
                        self.baseline_calculate(2)
                    baseline_calculated = True
                
                # Print status every 5 seconds (use newline to avoid interfering with input)
                current_time = time.time()
                if current_time - self.last_status_time >= 5.0:
                    elapsed = current_time - (self.last_status_time - 5.0)
                    rate = self.packets_received / elapsed if elapsed > 0 else 0
                    with self.print_lock:
                        sensor2_info = f", Sensor 2: {len(self.all_data2)}" if self.control2 else ""
                        print(f"[Status] Packets: {self.packets_received}, Sensor 1: {len(self.all_data1)}{sensor2_info}, Rate: {rate:.0f} Hz, Dropped: {self.packets_dropped}")
                    self.last_status_time = current_time
                
                # Small sleep to prevent busy-waiting
                time.sleep(0.001)

        except KeyboardInterrupt:
            self.running = False  # Signal threads to stop
            time.sleep(0.1)  # Give threads a moment to finish
            
            self.control1.finish()
            if self.control2:
                self.control2.finish()
            print(f"\n[Final] Total packets received: {self.packets_received}")
            print(f"[Final] Sensor 1 data points: {len(self.all_data1)}")
            if self.control2:
                print(f"[Final] Sensor 2 data points: {len(self.all_data2)}")
            print(f"[Final] Estimated packets dropped: {self.packets_dropped}")
            if len(self.all_data_timestamps1) > 1:
                time_span = (self.all_data_timestamps1[-1] - self.all_data_timestamps1[0]) / 1000000.0
                print(f"[Final] Time span: {time_span:.2f} seconds")
            self.save_data()
            print("data saved!")
        print("End...")

    def receive_sensor_data(self, sensor_num):
        """Thread function to continuously receive data from a single sensor"""
        if sensor_num == 1:
            control = self.control1
            sensor_name = self.sensor1_name
        elif sensor_num == 2:
            control = self.control2
            sensor_name = self.sensor2_name
        else:
            return
        
        packets_received_count = 0
        
        try:
            while self.running:
                try:
                    # Process ALL available packets as fast as possible
                    while True:
                        input_bytes = control.receive()
                        if input_bytes:
                            packets_received_count += 1
                            # Debug first few packets for sensor 2
                            if sensor_num == 2 and packets_received_count <= 3:
                                print(f"[Sensor 2] Received packet {packets_received_count}, length: {len(input_bytes)} bytes")
                            self.processing_data(input_bytes, sensor_num)
                        else:
                            break  # No more packets available
                    
                    # Small sleep to prevent 100% CPU usage when no data available
                    time.sleep(0.0001)
                    
                except Exception as e:
                    print(f'{sensor_name} data error = ', e)
                    import traceback
                    traceback.print_exc()
                    time.sleep(0.01)  # Brief pause before retrying
                    
        except Exception as e:
            print(f'{sensor_name} thread error = ', e)
            import traceback
            traceback.print_exc()

    def processing_data(self, input_bytes, sensor_num):
        try:
            if input_bytes is None or len(input_bytes) != 16:
                return
            
            # Parse binary packet: [4 bytes: timestamp (uint32_t, microseconds)] [4 bytes: X (float)] [4 bytes: Y (float)] [4 bytes: Z (float)]
            timestamp, x, y, z = struct.unpack('<Ifff', input_bytes)  # Little-endian: uint32_t, float, float, float
            
            # Validate data - check for reasonable values
            # Accelerometer values should typically be in range -20 to +20 g
            # Timestamps should be reasonable (not wrapping around)
            if abs(x) > 100 or abs(y) > 100 or abs(z) > 100:
                # Data looks corrupted, skip it
                return
            
            accel_data = [x, y, z]
            
            # Timestamp is in microseconds from Arduino micros()
            timestamp_us = timestamp
            
            # Detect dropped packets by checking for large timestamp gaps
            if self.last_timestamp is not None and sensor_num == 1:
                time_diff = timestamp_us - self.last_timestamp
                # At 3200 Hz, we expect ~312.5 microseconds between packets
                # If gap is > 1000 us, we likely dropped packets
                if time_diff > 1000:
                    estimated_dropped = int((time_diff - 312.5) / 312.5)
                    self.packets_dropped += max(0, estimated_dropped)
            
            if sensor_num == 1:
                self.last_timestamp = timestamp_us
            
            # Calculate time offset on first data point to sync with system time
            # This allows trial_status timestamps to match sensor timestamps
            if self.time_offset is None:
                system_time = time.time()
                # Store offset: sensor_time_us = (system_time - offset) * 1000000
                # So when we have a sensor timestamp in us, we can convert: system_time = (sensor_time_us / 1000000) + offset
                self.time_offset = system_time - (timestamp_us / 1000000.0)
                print(f"[Time Sync] Offset calculated: sensor_time = system_time - {self.time_offset:.3f}")
                print(f"[Time Sync] First sensor timestamp: {timestamp_us} us (Arduino micros)")
            
            # Track first timestamps from each sensor for synchronization
            if sensor_num == 1 and self.sensor1_first_timestamp is None:
                self.sensor1_first_timestamp = timestamp_us
            elif sensor_num == 2 and self.sensor2_first_timestamp is None:
                self.sensor2_first_timestamp = timestamp_us
                # If we have both first timestamps, calculate offset
                if self.sensor1_first_timestamp is not None and self.timestamp_offset_s2_to_s1 is None:
                    # Calculate offset: sensor2_time + offset = sensor1_time
                    # We'll align sensor 2 to sensor 1's time scale
                    self.timestamp_offset_s2_to_s1 = self.sensor1_first_timestamp - self.sensor2_first_timestamp
                    print(f"[Time Sync] Sensor timestamp alignment:")
                    print(f"  Sensor 1 first timestamp: {self.sensor1_first_timestamp} us")
                    print(f"  Sensor 2 first timestamp: {self.sensor2_first_timestamp} us")
                    print(f"  Offset (s2->s1): {self.timestamp_offset_s2_to_s1} us")
                    print(f"  Sensor 2 timestamps will be adjusted by {self.timestamp_offset_s2_to_s1} us to align with Sensor 1")

            # Use lock for thread-safe list operations
            with self.data_lock:
                if sensor_num == 1:
                    self.all_data1.append(accel_data)
                    self.all_data_timestamps1.append(timestamp_us)  # Store in microseconds
                    self.buffer1.append(accel_data)
                    self.buffer_timestamps1.append(timestamp_us)
                    if len(self.buffer1) > self.buffer_size:
                        self.buffer1.pop(0)
                        self.buffer_timestamps1.pop(0)
                    self.packets_received += 1
                elif sensor_num == 2:
                    # Only save sensor 2 data if save_only_sensor1 flag is False
                    if not self.save_only_sensor1:
                        self.all_data2.append(accel_data)
                        self.all_data_timestamps2.append(timestamp_us)  # Store in microseconds
                        # Debug: print first few sensor 2 data points
                        if len(self.all_data2) <= 5:
                            print(f"[Sensor 2] Received data point {len(self.all_data2)}: {accel_data}, timestamp: {timestamp_us}")
                    # Still update buffer for baseline calculation even if not saving
                    self.buffer2.append(accel_data)
                    self.buffer_timestamps2.append(timestamp_us)
                    if len(self.buffer2) > self.buffer_size:
                        self.buffer2.pop(0)
                        self.buffer_timestamps2.pop(0)
                    if not self.save_only_sensor1:
                        self.packets_received += 1
        except Exception as e:
            print(f'error processing data from sensor {sensor_num}: {input_bytes}')

    def baseline_calculate(self, sensor_num):
        if sensor_num == 1:
            with self.data_lock:
                acc_array = np.array(self.buffer1)
            # Filter out corrupted data (values > 100 g are clearly wrong)
            valid_mask = np.all(np.abs(acc_array) < 100, axis=1)
            if np.sum(valid_mask) < 50:  # Need at least 50 valid points
                print(f'{self.sensor1_name} WARNING: Not enough valid data for baseline ({np.sum(valid_mask)} valid points)')
                self.baseline_acc1 = np.array([0, 0, 0])
            else:
                valid_data = acc_array[valid_mask]
                self.baseline_acc1 = np.mean(valid_data, axis=0)
                print(f'{self.sensor1_name} baseline:', self.baseline_acc1)
            self.is_baseline_valid1 = True
        elif sensor_num == 2:
            with self.data_lock:
                acc_array = np.array(self.buffer2)
            # Filter out corrupted data
            valid_mask = np.all(np.abs(acc_array) < 100, axis=1)
            if np.sum(valid_mask) < 50:
                print(f'{self.sensor2_name} WARNING: Not enough valid data for baseline ({np.sum(valid_mask)} valid points)')
                self.baseline_acc2 = np.array([0, 0, 0])
            else:
                valid_data = acc_array[valid_mask]
                self.baseline_acc2 = np.mean(valid_data, axis=0)
                print(f'{self.sensor2_name} baseline:', self.baseline_acc2)
            self.is_baseline_valid2 = True


    def handle_terminal_input(self):
        """Handle terminal input commands in a separate thread"""
        while True:
            try:
                # Use lock to prevent status messages from interfering
                with self.print_lock:
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
                        with self.print_lock:
                            print(f"[Trial Status] Set to: '{message}' at sensor time {sensor_timestamp_us:.1f} us")
                    else:
                        # Store with system time, will be converted later when offset is known
                        system_time = time.time()
                        self.trial_status_history.append((system_time, message))
                        self.current_trial_status = message
                        with self.print_lock:
                            print(f"[Trial Status] Set to: '{message}' (will sync when data arrives)")
                # Ignore empty input
                    
            except EOFError:
                # Handle case where stdin is closed
                break
            except Exception as e:
                with self.print_lock:
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
        """Return the base filename (will overwrite existing files)"""
        return 'acc_data.csv'

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
        print(f"[Save Debug] control2 exists: {self.control2 is not None}, save_only_sensor1: {self.save_only_sensor1}, all_data2 length: {len(self.all_data2)}")
        if self.control2 is not None and not self.save_only_sensor1:
            print(f"[Save Debug] Sensor 2 condition passed, checking data length: {len(self.all_data2)}")
            if len(self.all_data2) > 0:
                print(f"[Save Debug] Saving {len(self.all_data2)} sensor 2 data points")
                for i in range(len(self.all_data2)):
                    baseline = self.baseline_acc2 if self.baseline_acc2 is not None else np.array([0, 0, 0])
                    timestamp = self.all_data_timestamps2[i]
                    # Apply timestamp offset to align sensor 2 with sensor 1's time scale
                    if self.timestamp_offset_s2_to_s1 is not None:
                        timestamp = timestamp + self.timestamp_offset_s2_to_s1
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
            else:
                print(f"[Save Debug] WARNING: Sensor 2 has no data to save! (all_data2 is empty)")
        else:
            print(f"[Save Debug] Sensor 2 NOT being saved - control2: {self.control2 is not None}, save_only_sensor1: {self.save_only_sensor1}")
        
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
    PORT_SENSOR_2 = '/dev/cu.usbmodem1101'   # Not used - single sensor setup

    # Single sensor setup - using Sensor 0x1D
    spec = AccelerationAnalyzer(
        port1=PORT_SENSOR_1,
        port2=PORT_SENSOR_2,
        sensor1_name="Sensor_0x1D",
        sensor2_name="Sensor_0x53",
        save_only_sensor1=False  # Only saving Sensor 1 (0x1D) data
    )
