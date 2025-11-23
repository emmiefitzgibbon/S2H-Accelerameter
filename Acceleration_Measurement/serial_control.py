import serial
import time
import struct

class serial_control:
    def __init__(self, port):
        self.port = port
        print('start a new serial connection (binary format)')
        self.host = serial.Serial(self.port, 2000000, timeout=0.1)  # 2 Mbps, shorter timeout
        self.buffer = b''  # Buffer for incomplete binary packets
        self.PACKET_SIZE = 16  # 4 bytes timestamp + 4 bytes X + 4 bytes Y + 4 bytes Z
        time.sleep(1)

    def send(self, data):
        self.host.write(bytes(data, encoding='utf-8'))

    def receive(self):
        # Read available data - larger buffer for faster reading
        data = self.host.read(5000)  # Read much larger chunks for higher rates
        if len(data) > 0:
            self.buffer += data

        # Binary format: 16 bytes per packet
        # Extract complete packets from buffer
        if len(self.buffer) >= self.PACKET_SIZE:
            # Return first complete packet
            packet = self.buffer[:self.PACKET_SIZE]
            self.buffer = self.buffer[self.PACKET_SIZE:]
            return packet

        return None

    def finish(self):
        print('serial connection ended')
        self.host.close()

