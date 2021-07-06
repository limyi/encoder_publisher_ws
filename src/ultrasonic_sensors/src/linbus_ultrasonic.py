#!/usr/bin/env python
import serial
import time 
import struct
import sys
from bitarray import bitarray # pip install bitarray
import bitarray.util as util
from binascii import hexlify

class Ultrasonic(object):
    def __init__(self, port, debug=False):
        # Class variables
        self.profile = "B"
        self.addresses = []
        self.port = port
        self.profile_dict = {
            "A": 254,
            "B": 253,
            "C": 252}
        # errors = {
        #   0xFF: "OK/No Error"
		# 	0x1: "Checksum Error"
		# 	0x2: "Telegram Timeout"
		# 	0x3: "Telegram Below Threshold"
		# 	0x4: "Telegram Above Thresho-ld="
		# 	0x5: "Parameter Error"
		# 	0x6: "Session Error"
		# 	0x7: "Transmission Error"
		# 	0x8: "EEPROM Error"
		# 	0x9: "OP CODE Error"
		# 	0xA: "Object is read-only"
		# 	0xB: "Temperature Error"}
        
        ########### INITIALISE CLASS INSTANCE #############
        # Initialise port with PySerial, break if unable to initialise
        print("Initialising......")
        try: self.serial = serial.Serial(
				port=port,
				#baudrate=9600,
				baudrate=19200,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				timeout=.6,
				bytesize=serial.EIGHTBITS)
        except ValueError: print("Wrong serial params!")
        
        # Verify device address and ensure device/port working
        if self.serial.is_open:# Initialise port with PySerial, break if unable to initialise
            print("Initialising......")
            try: self.serial = serial.Serial(
                    port=port,

                    baudrate=19200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=.6,
                    bytesize=serial.EIGHTBITS)
            except ValueError: print("Wrong serial params!")
            #except SerialException: print("Unable to connect! Check device or serial port!")


            print('Device connected on ' + self.serial.name)
        else:
            print('Serial port not open!')
        
        # check addresses connected
        self.serial.write(b'\xA8\x00\x00\x43')              
        try: 
            message = self.serial.readline()
            message = bytearray(message)

            # list of addresses
            for i in range(4,len(message),2):
                self.addresses.append(message[i])
            print(self.addresses, message)
            
        except:                                      # Return message != 5 bytes
            print('Address Cast response message corrupted')
    '''
    def _check_error(self,byte):
            return False

    def _checksum(self, data):          ## Recieves a list of bytes (dec) and returns checksum byte (dec)
        init = 82                       ## Todo: pad bytes (decimal) of <2digits
        for values in data:
            init = values ^ init        # first XOR operation
        temp = bin(init)[2:]            # convert to binary string and remove header (b)
        check = []
        for i in range(len(temp)):
            check.append(int(temp[i]))
        checksum = bitarray('00000000') # Init empty bit array object 
        ## The following bit reduction operation is in little endian (reverse from US documentation)
        checksum[2] = check[0]^check[2]^check[4]^check[6]
        checksum[3] = check[1]^check[3]^check[5]^check[7]
        checksum[4] = check[0]^check[1]
        checksum[5] = check[2]^check[3]
        checksum[6] = check[4]^check[5]
        checksum[7] = check[6]^check[7]

        checksum[1] = True              # Required bit 6 == 1 
        checksum[0] = False             # ACK/NACK bit, Reponse == 1 | Transmit == 0 
        return(int(util.ba2hex(checksum), 16))
        return(CHECKSUM_BYTE)
    ''' 
    def setprofile (self, profile):
        if self.profile == profile: print("Profile already set as %s",self.profile)
        else: 
            self.profile = profile
            #print("Profile changed to " + self.profile)
    def getprofile (self):
        return(self.profile)
    '''
    def measure(self, counts=1): 
        profile_dict = {
            "A": 254,
			"B": 253,
			"C": 252}
        
        ################# SYNC_BYTE (8-bits) ##67####################
        # Bit 7->4	|		Bit 3		|			Bit 2->0		
        #   0xA	  	|	  1/0 (R/W)		|	0xSensor_Address (e.g 0x1, 0x2)
        # Convert Bit 7->4 and Bit 2->0 to binary
        # 0xA -> 1010 in binary

        # SYNC BYTE (BINARY):
        # (0xA)  (R)	(0x1)
        # 1010 	1 	 001 (Reading from address 0x1 = 10101001)
        #########################################################
        
        '-----x0A (Header)-----Read------Device Address 001(0x1) to 111(0x7)------'
        SYNC =  "1010"        + "1" +    '{:03b}'.format(self.address)      # SYNC BYTE abstraction in string --> binary
        SYNC_BYTE = int(SYNC, 2)                                            # SYNC BYTE binary --> decimal
        OP_CODE = profile_dict[self.profile]                                # OP CODE abstraction instance variable(profile) --> decimal
        DATA_BYTE = 255 - counts                                            # DATA BYTE abstraction (decimal)
        CHECKSUM_BYTE = self._checksum((SYNC_BYTE,OP_CODE, DATA_BYTE))
            # struct.pack  --> Converts 4 decimals (1 byte) to 4 byte message. [Format in little endian 4 bytes]
        message_send = bytearray([SYNC_BYTE,OP_CODE,DATA_BYTE,CHECKSUM_BYTE])
        self.serial.write(message_send)
        time.sleep(0.01)

        try: 
            raw_message = self.serial.readline()
            print("raw message",raw_message,len(raw_message))
        
        except: print('Measure response message corrupted', len(raw_message), type(raw_message))
        
        if self._check_error(raw_message[5]): pass                           # Checksum in 6th byte
        elif raw_message[4] == b'\x00': 
            #print(0)
            return(0)      # No object detected
        elif raw_message[4] == b'\xff': 
            #print(0)
            return(0)      # No object detected
        elif raw_message[4] == b'\x01': 
            #print(0)
            return(0)      # No object detected
        else:                                                           # Checksum passsed with no error
            message_recieve = struct.unpack('<BBBBBB',raw_message)      # Unpack bytes into tuple
            distance = int(message_recieve[4]) * 1.6
            #print(distance)
            return(distance)
            #time.sleep(1)
    '''
    def setAddress(self, curr_addr, new_addr):
        SYNC =  "1010"        + "0" +    '{:03b}'.format(curr_addr)     # SYNC BYTE abstraction in string --> binary
        SYNC_BYTE = hex(int(SYNC, 2))
        OP_CODE = "0x35"
        DATA_BYTE = "0x0{}".format(str(new_addr))
        CHECKSUM_BYTE = self.get_checkbyte(SYNC_BYTE, OP_CODE, DATA_BYTE)

        message_send = bytearray([self.dec(SYNC_BYTE), self.dec(OP_CODE), self.dec(DATA_BYTE), self.dec(CHECKSUM_BYTE)])
        self.serial.write(message_send)
        try:
            raw_message = self.serial.readline()
            print("New address: {}".format(str(new_addr)))
            self.address = new_addr
        except:
            print("Failed to set new address")

    def off_pwm(self, addr):
        SYNC_BYTE = "0xA{}".format(str(addr))
        OP_CODE = "0x0A"
        DATA_BYTE = "0x01"
        CHECKSUM_BYTE = self.get_checkbyte(SYNC_BYTE, OP_CODE, DATA_BYTE)

        message = bytearray([self.dec(SYNC_BYTE), self.dec(OP_CODE), self.dec(DATA_BYTE), self.dec(CHECKSUM_BYTE)])
        self.serial.write(message)
        try:
            raw_message = self.serial.readline()
            print("PWM turned off")
        except:
            print("Failed to off PWM")

    def get_checkbyte(self, d1, d2, d3):
        SYNC_BYTE = int("0xA0", 16)
        OP_CODE = int("0x00", 16)
        DATA1 = int(d1, 16)
        DATA2 = int(d2, 16)
        DATA3 = int(d3, 16)
        message = bytearray([SYNC_BYTE,OP_CODE,DATA1,DATA2,DATA3])
        self.serial.write(message)
        
        raw_message = self.serial.readline()
        raw_message = bytearray(raw_message)
        check_byte = hex(raw_message[-1])
        # print("Check byte: " + str(check_byte))
        
        return check_byte

    def dec(self, hx):
        return int(hx, 16)

    def measure(self, addr, profile="B", cycles=1):
        addr = int(addr)
        SYNC =  "1010" + "1" + '{:03b}'.format(addr)
        SYNC_BYTE = hex(int(SYNC, 2))
        OP_CODE = hex(self.profile_dict[profile])
        DATA_BYTE = hex(255 - cycles)
        CHECKSUM_BYTE = self.get_checkbyte(SYNC_BYTE, OP_CODE, DATA_BYTE)
        
        message = bytearray([self.dec(SYNC_BYTE), self.dec(OP_CODE), self.dec(DATA_BYTE), self.dec(CHECKSUM_BYTE)])
        self.serial.write(message)
        raw_message = bytearray(self.serial.readline())
        print("Message",message)
        distance = int(raw_message[4]) * 1.6
        return (addr, distance)

    def measure_all(self):
        # returns list of measurements [(addr1, reading1), (addr2, reading2)]
        dist_ls = []
        for addr in self.addresses:
            dist_ls.append(self.measure(addr))
        return dist_ls

    def measure_all_Edit(self):
        hexadecimal_string = "A8000043" ## Read Address
        #hexadecimal_string = "A2FDFE40" ## Read Address
        hexadecimal_string = "A9FA2F75" ## Read Address
        #hexadecimal_string = "A20A0152" ## Read Address
        byte_array = bytearray.fromhex(hexadecimal_string)
        self.serial.write(byte_array)
        raw = self.serial.read(69)##You may need to change this
        info = struct.unpack('<BBBBBB',raw)
        print(info,raw)

        return info
