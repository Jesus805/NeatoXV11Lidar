# NeatoXV11Lidar
Multiprocessed Python Neato v11 LIDAR reader.
## Overview
This module allows a Raspberry Pi to read distance readings from a Neato XV-11 LIDAR. The module utilizes multiprocessing to allow true parallel computation without the limitation of Python's [Global Interpreter Lock](https://wiki.python.org/moin/GlobalInterpreterLock). 

This module was written with NumPy in mind. Therefore, the shared buffer contains distance readings at index `2*i` and reliability readings at index `2*i + 1` for `0 <= i <= 359`

There is also code for interfacing with the GPIO to turn the LIDAR on/off. However, GPIO pins do not supply enough current to power the LIDAR, so you will need to create a circuit that allows the GPIO pin to switch a 3v3 power rail via a transistor.

MIT license, (C) 2018 Jesus Bamford <jesus.bamford@gmail.com>

## Running
To execute a LIDAR test-run

``python3 neato_xv11.py``

Note: Only works with Python 3
