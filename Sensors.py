# -*- coding: utf-8 -*-

"""
Sensors.py - Classes to define sensor functionality and attributes
"""

import sys
import telnetlib
import time
import socket
import struct
import math


class Sensor:
    """
    Generic base class for all logging sensors.
    """
    def get_reading(self):
        pass

    def send_cmd(self):
        pass


class VSL(Sensor):
    """
    Vaisala sensor class.
    """
    def __init__(self, host):
        """
        :param host: (str) IP address of unit, eg: '172.16.52.172'.
        """
        self.host = host

    def get_reading(self):
        """
        :return: a tuple of (temperature, humidity) readings.
        """
        with telnetlib.Telnet(self.host) as tn:  # Open connection to Vaisala unit.
            # tn = telnetlib.Telnet(self.host)
            try:
                tn.read_until(b'\r\nHMT330 / 5.16\r\n>')
            except EOFError:
                print('telnet error on setup ', sys.exc_info())
                return 0.0, 0.0
            tn.write(b'send\r')
            time.sleep(1)
            while True:
                try:
                    line = tn.read_until(b'\r\n', timeout=1)
                except EOFError:
                    print('telnet error on reading ', sys.exc_info())
                    return 0.0, 0.0
                if b'C' in line:
                    break
        vsloutput = line.decode('ascii')
        temp = float(vsloutput[37:42])
        hum = float(vsloutput[24:29])
        return temp, hum


class ModBus(Sensor):
    """
    Class for Modbus functions, used by SeaLevel 170E I/O unit 'eIO 170',
    to which a number of analogue sensors or passive devices
    may be attached.

    The communication protocol is Modbus TCP/IP (Ethernet).
    See secn 5.3 'Communicating Via Modbus' in SeaMAX Software API Manual.pdf,
    located in:
    I:/MSL/Private/Electricity/Ongoing/Cryocooler/He-compressor_emergency_cut-out/Ethernet I-O/

    Further reasources:
    https://en.wikipedia.org/wiki/Modbus
    and
    http://www.modbus.org.

    Usage:
    seal170E = ModBus('131.203.9.249') # creates a Modbus object with member functions:
        response (float)= ReadAnalog(start_ch,num_ch) ;
        response (int,int) = ReadCoils(start_ch,num_ch);
        None = WriteCoil(start_ch, state). state in form (int, int) for (relay1, relay2);
        (V_ref, mode, ch_ranges) = GetAnalogConfig();
        None = SetAnalogConfig(V_ref, mode, ch_ranges)
        and open() and close() methods.
    """
    def __init__(self, host):
        self.ip_addr = host
        self.tcp_port = 502
        self.slave_id = 0x01  # This identifies the SeaLevel 170E unit as the only slave.
        self.zero_2bytes = (0x00, 0x00)
        self.one_2bytes = (0x00, 0x01)
        self.six_2bytes = (0x00, 0x06)  # same as (hex(0), hex(6))
        self.full_scale = 4096
        self.half_scale = 2048
        self.ranges = {0: 5.0, 1: -5.0, 2: 10.0, 3: -10.0}  # '5.0' means '0-5V'; '-5.0' means '+/-5V',etc.
        self.sock = None
        self.v_ref = 0
        self.mode = 0
        self.ch_ranges = []

    def open(self):
        """
        Create a TCP/IP socket.
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip_addr, self.tcp_port))
        # Get channel range configuration:
        self.v_ref, self.mode, self.ch_ranges = self.get_analog_config()

    def close(self):
        """
        Close TCP/IP socket.
        """
        self.sock.shutdown(socket.SHUT_RDWR) # close the socket quickly
        self.sock.close()

    def read_analog(self, start_ch, num_ch):  # MODBUS 0x04 - 'Read input registers'
        """
        Read 1 or more sequential analog input channels.
        :param start_ch: First channel to read (int);
        :param num_ch: Number of channels to read(int);
        :return: list of floats.
        """
        fn = 0x04
        data = [(0x00, start_ch), (0x00, num_ch)]

        # Get channel range configuration
        # (v_ref, mode, ch_ranges) = self.get_analog_config()
        v_range = self.ranges[self.ch_ranges[start_ch]]  # should be either 5.0, -5.0, 10.0 or -10.0
        # construct message from packet data
        msg = self.build_msg(fn, data)

        # pack up msg in binary form (stream of bytes suitable for socket)
        sock_out = struct.pack(*msg)  # pass each byte as an argument to struct.pack

        self.sock.send(sock_out)  # send request
        time.sleep(0.1)

        buffer_size = 9 + num_ch * 2
        sock_ret = self.sock.recv(buffer_size)  # recieve (9 + 2*n) bytes response
        reply = struct.unpack(str(buffer_size) + 'B', sock_ret)
        rev_reply = reversed(reply)
        byte_it = iter(rev_reply)
        readings = []
        for chan in range(num_ch):
            lsb = byte_it.__next__() & 0b11111111  # reply[-1] & 0b11111111
            msb = byte_it.__next__() & 0b00001111  # reply[-2] & 0b00001111  # ignore sign and 3 pad bits
            raw_val = float(msb * 256 | lsb)
            if v_range > 0:  # positive range only
                v = v_range * (raw_val / self.full_scale)
            else:  # +/- range
                if raw_val <= self.half_scale:
                    v = -v_range * (raw_val / self.half_scale)
                else:
                    v = -v_range * ((raw_val - self.full_scale) / self.half_scale)
            readings.append(v)
        return readings

    def ReadCoils(self, start_ch, num_ch):  # MODBUS 0x01 - 'Read both coils'
        fn = 0x01
        data = [(0x00, start_ch), (0x00, num_ch)]  # (lsbyte, msbyte)

        # construct message
        msg = self.build_msg(fn, data)
        sock_out = struct.pack(*msg)  # pass each byte as an argument to struct.pack().

        self.sock.send(sock_out)  # send request
        time.sleep(0.1)

        # n_data_bytes = 1 + (num_ch - 1) / 8  # 1 relay/bit => 1 byte per 8 bits
        # int((num_ch - (num_ch-1) % 8 - 1)/8 + 1)
        """
        Each relay is represented by one bit - 0:OFF, 1:ON.
        Need to calculate minimum number (n_data_bytes) of 8-bit bytes to
        hold the states of all num_ch relays. (Eg: for two relays, we only
        need 1 byte - rest of byte is 0-padded, ie: last 6 bits will be 0s):
        """
        n_data_bytes = math.ceil(num_ch/8)
        buffer_size = 9 + n_data_bytes  # Where do 9 bytes come from?
        sock_ret = self.sock.recv(buffer_size)  # recieve response
        reply = struct.unpack(str(buffer_size) + 'B', sock_ret)
        raw_val = reply[-1]  # data bytes only (only have 2 relays, so must be >last< byte)
        states = {0: (0, 0), 1: (1, 0), 2: (0, 1), 3: (1, 1)}
        return states[raw_val]

    def WriteCoil(self, start_ch, state):  # MODBUS 0x05 - 'Write single coil'
        fn = 0x05
        states = {0: 0x00, 1: 0xff}
        data = [(0x00, start_ch), (states[state], 0x00)]

        # construct message from packet data
        msg = self.build_msg(fn, data)
        sock_out = struct.pack(*msg)  # pass each byte as an argument to struct.pack

        self.sock.send(sock_out)  # send request
        time.sleep(0.1)

        buffer_size = 8 + 4  # Response is a copy of the query
        sock_ret = self.sock.recv(buffer_size)  # recieve response
        rply = struct.unpack(str(buffer_size) + 'B', sock_ret)

        return rply

    def get_analog_config(self):  # MODBUS 0x65 (DEC101) - 'Get A/D, D/A configuration'
        """
            Data-frame for request:             Data-frame for response:
                | Function  | 0x64 |              | Function       | 0x64 |
                                                  | Device Config  | ?    |
                                                  | Channels 1-4   | ?    |
                                                  | Channels 5-8   | ?    |
                                                  | Channels 9-12  | ?    |
                                                  | Channels 13-16 | ?    |

            Bit-packing for Device Config Byte:
                               Bit | 7   6   5   4   |   3   2   1   0   |
                Device Config Byte | A/D V-Reference |  A/D channel mode |

            Bit-packing for Channel Config Bytes:
                          Bit   |   7   6   |   5   4   |   3   2   |   1   0   |
                Chan 1-4 byte   | Ch1 range | Ch2 range | Ch3 range | Ch4 range |
                Chan 5-8 byte   | Ch5 range | Ch6 range | Ch7 range | Ch8 range |
                Chan 9-12 byte  | Ch9 range | Ch10 range| Ch11 range| Ch12 range|
                Chan 13-16 byte | Ch13 range| Ch14 range| Ch15 range| Ch16 range|

            :param v_ref:
            :param mode:
            :param ch_ranges:
            :return:
        """
        fn = 0x65
        data = []  # no data to send, just function code

        # construct message from packet data
        msg = self.build_msg(fn, data)
        sock_out = struct.pack(*msg)  # pass each byte as an argument to struct.pack

        self.sock.send(sock_out)  # send request
        time.sleep(0.1)

        buffer_size = 8 + 5  # Last 5 bytes: config, ch1-4, ch4-8, ch9-12, ch13-16
        sock_ret = self.sock.recv(buffer_size)  # recieve response
        rply = struct.unpack(str(buffer_size) + 'B', sock_ret)
        v_ref = rply[-5] >> 4 & 0b1111
        mode = rply[-5] & 0b1111

        r_bytes = rply[-4:]
        ch_ranges = []
        for i in r_bytes:  # for each range byte (4 channels-worth)...
            for j in reversed(range(0, 8, 2)):  # 6,4,2,0
                ch_ranges.append(int(i >> j) & 0b11)
        return v_ref, mode, ch_ranges

    def SetAnalogConfig(self, v_ref, mode, ch_ranges):  # MODBUS 0x64 (DEC100) - 'Set A/D, D/A configuration'
        """
            Data-frame for Request:         Data-frame for response:
                | Function       | 0x64 |       | Function          | 0x64 |
                | Device Config  | ?    |       | Err code (if any) | ?    |
                | Channels 1-4   | ?    |
                | Channels 5-8   | ?    |
                | Channels 9-12  | ?    |
                | Channels 13-16 | ?    |

            Bit-packing for Device Config Byte:
                               Bit| 7   6   5   4   |   3   2   1   0   |
                Device Config Byte| A/D V-Reference |  A/D channel mode |

            Bit-packing for Channel Config Bytes:
                          Bit  |   7   6   |   5   4   |   3   2   |   1   0   |
                Chan 1-4 byte  | Ch1 range | Ch2 range | Ch3 range | Ch4 range |
                Chan 5-8 byte  | Ch5 range | Ch6 range | Ch7 range | Ch8 range |
                Chan 9-12 byte | Ch9 range | Ch10 range| Ch11 range| Ch12 range|
                Chan 13-16 byte| Ch13 range| Ch14 range| Ch15 range| Ch16 range|

            :param v_ref:
            :param mode:
            :param ch_ranges:
            :return:
        """
        fn = 0x64
        data = [(v_ref << 4) | mode]  # repack data into one byte

        # repack 16 x 2-bit channel ranges data into four 8-bit bytes
        for i in range(0, 16, 4):
            r = int(ch_ranges[i]) << 6 | int(ch_ranges[i + 1]) << 4 | int(ch_ranges[i + 2]) << 2 | int(ch_ranges[i + 3])
            data.append(r)

        # construct message from packet data
        msg = self.build_msg(fn, data)
        sock_out = struct.pack(*msg)  # pass each byte as an argument to struct.pack

        self.sock.send(sock_out)  # send request
        time.sleep(0.1)

        buffer_size = 8 + 1  # [header+fn_code] + error code
        sock_ret = self.sock.recv(buffer_size)  # recieve response
        rply = struct.unpack(str(buffer_size) + 'B', sock_ret)
        return rply

    def build_msg(self, fn, data):
        """
        Build query message. The first few bytes simulate a multiple of at least
        3.5 character times before the actual data frame.
        :param fn: Modbus function code;
        :param data: Data associated with function;
        :return: message (list).
        """
        header = [self.zero_2bytes, self.zero_2bytes, self.six_2bytes, self.slave_id, fn]
        head_data = header + data  # Concatenate lists
        msg = ['']  # placeholder for packing format string
        for i in head_data:
            if isinstance(i, int):
                msg.append(i)
            else:
                for j in i:
                    msg.append(j)
        # Write check string:
        msg_len = len(msg) - 1
        msg[0] = str(msg_len) + 'B'
        return msg

