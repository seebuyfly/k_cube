#!/usr/bin/python2.7

from __future__ import print_function

import serial
import struct
import sys
import threading
import time
import datetime
import os
from collections import OrderedDict
from time import sleep
from crc16 import crc16xmodem

# windows: check "Load VCP" in APT USB Device (in DeviceManager)
# Then, unplug USB and replug.  The K Cube should now be assigned a COM port
# https://www.thorlabs.com/Software/Motion%20Control/APT_Communications_Protocol.pdf p.290

# SERIAL_DEV = "USB\\VID_0403&PID_FAF0\\69251259"
SERIAL_DEV = "COM3"
FILE_PATH = 'tmp/pos_data.txt'


# NAME = sys.argv[2]
# LOW_LIM = 15
# HIGH_LIM = 19


# write_type None means automatic.
def reg(pretty_name, address, writable, return_type='u', multiplier=1, write_type=None):
    return {'address': address, 'name': pretty_name, 'writable': writable,
            'returns': return_type, 'multiplier': multiplier, 'write_type': write_type}


def extract_int32(x):
    return struct.unpack('<i', x)[0]


class APT:
    dev_id = 0x50
    # internal_per_mm = 34304.0
    busy = False
    queued = False
    waiting_for_response = False
    busy_attempts = 0
    deferred = None
    r_lock = threading.Lock()

    def __init__(self, s_port, file_path, debug=lambda x: None):
        self.s_port = s_port
        self.file_path = file_path
        # ~ self.internal_per_mm = 34304.0
        self.ser = serial.Serial(s_port, 115200, timeout=20,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE)
        self.debug = debug
        self.make_file()

    def __del__(self):
        self.ser.close()

    def wait_for_our_turn(self):
        while True:
            self.r_lock.acquire()
            print("Waiting...")
            tmp = self.waiting_for_response
            self.r_lock.release()
            if tmp:
                sleep(0.25)
            else:
                return

    # def gohome(self, magic_number):
    #     print("Here: %d" % magic_number)
    #     if magic_number != 1337:
    #         print("You didn't put in the magic number.")
    #         return "NO DICE"
    #     # ~ tosend = [0x43, 0x04, 0x01, 0x00, 0x50, 0x01]
    #     tosend = b'\x43\x04\x01\x00\x50\x01';
    #     self.send_it(tosend)
    #     return "HOMING"

    def send_it(self, msg):
        self.debug("Sending Message: %s" % map(ord, list(msg)))
        print("SENDING...")
        self.ser.write(msg)
        print("SENT!")

    def execute_deferred(self):
        if self.busy:
            self.busy_attempts += 1
            self.debug("Busy, deferring %d" % self.busy_attempts)
            threading.Timer(0.25, self.execute_deferred).start()
        else:
            self.deferred()
            self.queued = False
            self.busy_attempts = 0

    # def stop_movement(self):
    #     cmd = self.general_command(cmd=b'\x65\x04', params=b'\x01\x02')  # Channel 1 to stop gracefully (02)
    #     self.send_it(cmd)
    #     if not self.busy:
    #         self.read_ser()

    # def move(self, pos_mm):
    #     if pos_mm < LOW_LIM or pos_mm > HIGH_LIM:
    #         print("OUT OF RANGE!!!")
    #         return
    #     # ~ if self.busy:
    #     # ~ self.debug("Busy, deferring")
    #     # ~ self.deferred = lambda: self.move(pos_mm)
    #     # ~ if not self.queued:
    #     # ~ self.queued = True
    #     # ~ threading.Timer(0.25, self.execute_deferred).start()
    #
    #     # ~ return
    #
    #     # ~ self.busy = True
    #     pos_int = pos_mm * self.internal_per_mm;
    #     print("Nominally moving to: %fmm" % pos_mm)
    #     pos_int_s = struct.pack('<i', pos_int)
    #
    #     cmd = self.general_command(cmd=b'\x53\x04', data=pos_int_s)
    #     print("Command is: %s" % map(ord, list(cmd)))
    #     self.send_it(cmd)
    #
    #     # Async:
    #     if not self.waiting_for_response:
    #         threading.Timer(0, self.read_ser).start()

    def pos(self):
        self.wait_for_our_turn()
        # self.send_it(b'\x11\x04\x01\x00\x50\x01')
        self.send_it(b'\x71\x08\x03\x00\x50\x01')

        def pad(read_data, low_lim, up_lim):
            pad_width = 4
            pad_char = '\0'
            tmp = read_data[low_lim:up_lim]
            tmp = tmp.ljust(pad_width, pad_char)
            return tmp

        r = self.read_ser()

        print(str(r))
        print("data:")
        print(str(r['data']))
        print("bottom")
        print(str(list(r['data'])))

        data = list(r['data'])
        print("x_diff: " + str(data[0:1 + 1]))
        print("y_pos: " + str(data[2:3 + 1]))
        print("sum: " + str(data[4:5 + 1]))

        tmp_x = pad(r['data'], 0, 1 + 1)
        tmp_y = pad(r['data'], 2, 3 + 1)
        tmp_sum = pad(r['data'], 4, 5 + 1)
        tmp_time = datetime.now()
        tmp_time = tmp_time.strftime("%d/%m/%Y %H:%M:%S")
        tmp_data_str = tmp_time + ',' + tmp_x + ',' + tmp_y + ',' + tmp_sum
        print(tmp_data_str)

        with open(self.file_path, 'a') as f:
            f.write(tmp_data_str)

        # # print("x_pos2: " + str(list(r['data'][6:7+1])))
        # # ljust pads to a defined width using a defined character
        # pad_width = 4
        # pad_char = '\0'
        # tmp = r['data'][0:1 + 1]
        # tmp = tmp.ljust(pad_width, pad_char)
        # print("x_pos2: " + str(extract_int32(tmp)))
        # tmp = r['data'][2:3 + 1]
        # tmp = tmp.ljust(pad_width, pad_char)
        # print("y_pos2: " + str(extract_int32(tmp)))
        # tmp = r['data'][4:5 + 1]
        # tmp = tmp.ljust(pad_width, pad_char)
        # print("sum: " + str(extract_int32(tmp)))

    # return extract_int32(r['data'])/self.internal_per_mm

    def pos_every(self, interval_s):
        start_time = time.time()
        while True:
            self.pos()
            time.sleep(interval_s - ((time.time() - start_time) % interval_s))

    @staticmethod
    def general_command(cmd, data='', params=''):
        print("Up top")
        source = b'\x01'  # PC
        dest = b'\x50'  # Generic USB device

        # Bytes 0-1, command:
        s = cmd
        # Bytes 2-3
        if len(data):
            s += struct.pack('<h', len(data) + 2)
            data = b'\x01\x00' + data  # Channel info...does this ever change?
            dest = chr(ord(dest) | 0x80)
        elif len(params) == 2:
            s += params
        else:
            s += b'\x00\x00'

        # Bytes 4-5, and (possibly) beyond
        s += dest + source + data
        print("Final command: %s" % map(ord, list(s)))
        return s

    # See https://www.thorlabs.com/Software/Motion%20Control/APT_Communications_Protocol.pdf, page 30
    @staticmethod
    def parse_response(s):
        a = map(ord, list(s))  # Turn our bytestring into an array, useful for some things...
        r = {'msg_id': a[0:1 + 1], 'param1': a[2], 'param2': a[3]}
        dest = a[4]
        if dest & 0x80:  # We are expecting a data packet with this response
            r['dest'] = 0x80 ^ dest
            r['data_len'] = a[2] + 0x100 * a[3]  # I think!
        else:  # No data packet to follow
            r['dest'] = dest
            r['data_len'] = 0
        r['source'] = a[5]
        r['chan_identity'] = a[6:8]  # Next tends (???) to be channel
        r['data'] = s[8:6 + r['data_len']]  # And now our data, which *included* the two channel bytes. I think...

        return r

    # This is a bit odd, perhaps.
    #
    # Basic idea: we are allowed to write to the device as much as we want for e.g. moving, but
    # and it will only send *one* response when it's done. So no need to block the writing, but
    # at the same time we need to make sure we don't keep reading.
    #
    # Note that we should actually be allowed to poll our progress!

    def read_ser(self):
        self.r_lock.acquire()
        if self.waiting_for_response:
            self.r_lock.release()
            return

        self.waiting_for_response = True
        self.r_lock.release()
        # ~ self.busy = True # Ok this is super sketchy, and relies on the global lock
        # I think...need proper locking/semaphores/yada yada!

        header_raw = self.ser.read(6)
        print("Header read %s\n\t%s)" % (
            map(lambda x: '0x' + format(x, '02x'), map(ord, list(header_raw))), list(header_raw)))

        if not len(header_raw):
            print("NO RESPONSE!")

            return []

        r = self.parse_response(header_raw)
        all_raw = header_raw + self.ser.read(r['data_len'])

        print("Full read %s" % map(lambda x: '0x' + format(x, '02x'), map(ord, list(all_raw))))

        # ~ self.busy = False
        self.r_lock.acquire()
        self.waiting_for_response = False
        self.r_lock.release()
        return self.parse_response(all_raw)

    def make_file(self):
        if not os.path.exists(self.file_path):
            with open(self.file_path, 'w') as f:
                f.write('x_diff,y_diff,sum')
        else:
            print(self.file_path + " already exists.")


# Can also use /dev/ttyUSBx (0 as of writing...but below is more portable?)
apt = APT(SERIAL_DEV, FILE_PATH, debug=lambda x: print('DEBUG: %s' % x))  # print(s) for debugging
apt.pos()

# Create file with heading
# Append position data each time .pos() is called

# Add time stamp when .pos() is called
# Call .pos() at set interval
# Implement time averaging in .pos()?
