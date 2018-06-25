#!/usr/bin/env python

'''
MIT License

Copyright (c) 2018 Jesus Bamford

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

import serial
import time
import RPi.GPIO as GPIO

# Raspberry Pi 2
# COM_PORT  = '/dev/ttyAMA0'
# Raspberry PI 3 (ttyAMA0 becomes the bluetooth port)
# WARNING: If you're planning on using ttyS0, disable getty
# COM_PORT  = '/dev/ttyS0'

# Select the correct UART port automatically
COM_PORT    = '/dev/serial0'
BAUDRATE    = 115200
BOARD_NUM   = 11

serial_port = None

def checksum(data):
    """
    Compute and return the checksum as an int.
    :param data: list of 20 bytes, in the order they arrived in.
    :return: The checksum computed.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append(data[2 * t] + (data[2 * t + 1] << 8))

    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    check_sum = (chk32 & 0x7FFF) + (chk32 >> 15)  # wrap around to fit into 15 bits
    check_sum &= 0x7FFF  # truncate to 15 bits
    return int(check_sum)


def cleanup():
    """
    Clean up Serial and GPIO port.
    Should be called before closing the program.
    """
    if serial_port is not None and serial_port.is_open:
        serial_port.close()
    serial_port = None

    GPIO.cleanup()


def init():
    """
    Initialize Serial and GPIO Port.
    """
    global BOARD_NUM, COM_PORT, BAUDRATE, serial_port
    serial_port = serial.Serial(port=COM_PORT, baudrate=BAUDRATE, timeout=None)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BOARD_NUM, GPIO.OUT)


def motor_disable():
    """
    Set Voltage LOW on GPIO pin.
    """
    global BOARD_NUM

    GPIO.output(BOARD_NUM, 0)


def motor_enable():
    """
    Set Voltage HIGH on GPIO pin.
    """
    global BOARD_NUM

    GPIO.output(BOARD_NUM, 1)


def run(shared_buffer, lock, msg_pipe):
    """
    Read from LIDAR readings from serial port.
    Stores readings into multiprocessing buffer.
    :param shared_buffer: multiprocessing 1D array of size 720 (360 x 2).
    :param lock: multiprocessing lock.
    :param msg_pipe: pipe to send messages to calling process.
    """
    global serial_port

    if serial_port is None:
        raise TypeError('init() must to be called first')

    motor_enable()

    while True:
        # Do not hog CPU power
        time.sleep(0.00001)
        # Check for any messages in the pipe
        if msg_pipe.poll():
            if msg_pipe.recv() == 0xFF:
                break

        # Packet format = [0xFA, 1-byte index, 2-byte speed, [2-byte flags/distance, 2-byte quality] * 4, 2-byte checksum]
        # All multi-byte values are little endian.
        packet_header = serial_port.read(1)
        if packet_header[0] != 0xFA:
            continue

        packet_index = serial_port.read(1)
        if packet_index[0] < 0xA0 or packet_index[0] > 0xF9:
            continue

        # Packet index | Range = [0,89]
        index = packet_index[0] - 0xA0
		
        # Read the rest of the packet
        data = serial_port.read(20)

        # Verify the packet's integrity
        expected_checksum = data[19] << 8 | data[18]
        actual_checksum = packet_header + packet_index + data[0:18]
        if checksum(actual_checksum) != expected_checksum:
            # Checksum error
            with lock:
                for i in range(4):
                    shared_buffer[8 * index + 2 * i + 0] = 0
                    shared_buffer[8 * index + 2 * i + 1] = -3
            continue

        # Speed in revolutions per minute
        speed_rpm = (data[1] << 8 | data[0]) / 64.0
        # A packet contains 4 distance/reliance readings
        for i in range(4):
            byte_ndx = 4 * i + 2
            # The first 16 bits are two flags + distance
            distance = (data[byte_ndx + 1] << 8) | data[byte_ndx]
            # The second 16 bits are the reliability (higher # = more reliable reading)
            quality  = (data[byte_ndx + 3] << 8) | data[byte_ndx + 2]
            # Look for "invalid data" flag
            if (distance & 0x8000) > 0:
                with lock:
                    # byte 0 contains the error code
                    shared_buffer[8 * index + 2 * i + 0] = data[byte_ndx]
                    shared_buffer[8 * index + 2 * i + 1] = -1
                continue
            # Look for "signal strength warning" flag
            # adding distance might be okay
            elif (distance & 0x4000) > 0:
                with lock:
                    shared_buffer[8 * index + 2 * i + 0] = distance
                    shared_buffer[8 * index + 2 * i + 1] = -2
                continue
            else:
                # Remove flags and write distance/quality to numpy array
                with lock:
                    shared_buffer[8 * index + 2 * i + 0] = distance & 0x3FFF
                    shared_buffer[8 * index + 2 * i + 1] = quality

        if index == 89:
            msg_pipe.send(0x01)

    motor_disable()


if __name__ == "__main__":
    import multiprocessing as mp
    import numpy as np
	
    # Multiprocessing array containing the synchronous state
    shared_buffer = mp.Array('i', 720)
    # The Lidar Data
    # index -> angle
    # data -> (distance in milimeter, integrity)
    lidar_data = np.frombuffer(shared_buffer.get_obj(), dtype=int).reshape((360, 2))
    # Global lock
    g_lock = mp.Lock()
    # Multiprocessing communication pipe
    parent_conn, child_conn = mp.Pipe()

    init()
    p = mp.Process(target=run, args=(shared_buffer, g_lock, child_conn,))
    p.start()
    try:
        while True:
            time.sleep(0.00001)
            # Check if any data in pipe to prevent blocking
            if parent_conn.poll():
                parent_conn.recv()
                with g_lock:
                    distance  = lidar_data[:,0]
                    integrity = lidar_data[:,1]
                    # Filter out errors
                    distance[integrity < 1] = 0
                    print(distance)

    except KeyboardInterrupt:
        parent_conn.send(0xFF)

    p.join()
    cleanup()
