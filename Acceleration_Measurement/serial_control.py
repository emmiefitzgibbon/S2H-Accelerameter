import serial
import time
import struct

class serial_control:
    def __init__(self, port):
        self.port = port
        print('start a new serial connection (binary format)')
        # Increase input buffer size to prevent overflow at high data rates
        # Use very short timeout for non-blocking reads
        self.host = serial.Serial(
            self.port, 
            2000000, 
            timeout=0.001,  # Very short timeout (1ms) for non-blocking reads
            write_timeout=0.1
        )
        # Note: Serial buffer size is typically OS-controlled and can't be changed via pyserial
        # The key is to read frequently enough to prevent overflow
        self.buffer = b''  # Buffer for incomplete binary packets
        self.PACKET_SIZE = 16  # 4 bytes timestamp + 4 bytes X + 4 bytes Y + 4 bytes Z
        time.sleep(1)

    def send(self, data):
        self.host.write(bytes(data, encoding='utf-8'))

    def receive(self):
        # Read ALL available data aggressively to prevent buffer overflow
        # Read in large chunks to minimize system calls
        data = self.host.read(50000)  # Read up to 50KB at a time (3125 packets)
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

