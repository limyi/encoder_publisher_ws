#!/usr/bin/env python
import serial
import struct
import time
import sys

from binascii import unhexlify
_serialports = {}

class dis(object):
	def __init__(self, port, hexadecimal_input, debug = False):
		self._port = port
		self.debug = debug
		self.hexadecimal_input = hexadecimal_input
		print("Initialization.....")
		if port not in _serialports or not _serialports[port]:
			self._print_debug("Create serial port {}".format(port))
			self.serial = _serialports[port] = serial.Serial(
				port=port,
				baudrate=19200,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				timeout=0.0,
				bytesize=serial.EIGHTBITS
			)
			_print_out("Successfully Connected to the battery, finally...")
		else:
			self._print_debug("Serial port {} already exists".format(port))
			self.serial = _serialports[port]
			if (self.serial.port is None) or (not self.serial.is_open):
				self._print_debug("Serial port {} is closed. Opening.".format(port))
				self.serial.open()
	def _print_debug(self, text):
		if self.debug:
			_print_out("MinimalModbus debug mode. " + text)

	def readDistanceInfo(self):
		try:
			self._print_debug(str(self.serial.isOpen()))
			#print("To test if serial is open")
			if self.serial.isOpen():
				print("Serial is open")


      



				#Sequence to Off PWM RUN STEP 1 to Step 4 in sequence Manually. Change input command to sensor at Step 1. Uncommend Step 2 and run main.py. Repeat Steps until Step 5 or 7
				#hexadecimal_string = "A136557C" # STEP 2: Factory Reset Sensor. Default is Address 7
				#hexadecimal_string = "A7350161" # STEP 3: Change address from 7 to 1
				#hexadecimal_string = "A10A0162" # Step 4: PWM OFF at address 1
				#hexadecimal_string = "A9FCFE73" # Step 5: Check if value is readable at address 1. Should have 6 bytes where distance value is Index[4] multiply by 1.6 READ ADDRESS 1

				#Change address and read new address distance value
				#hexadecimal_string =  		 # Step 6: Change address from 1 to desired. A1350#CheckSum. # is Desired address number. Refer to excel sheet to calculate checksum
				#hexadecimal_string = "ABFCFE52" # Step 7: Read new address distance value. Telegram. A9-AF for Address 1-9, FC, FE, Checksum, Refer to excel for checksum


				#Example Input
				#hexadecimal_string = "A9FCFE73" # READ ADDRESS 1 A9 for Address 1. AA for Address 2, AB for Address 3,.... AF for Address 7
				#hexadecimal_string = "A2350162" # Change address to 2 to 1
				#hexadecimal_string = "A8000043" # Read Address
				#hexadecimal_string = "ABFCFE52" # READ ADDRESS 3
				#hexadecimal_string =            # Read Address1:"A9FCFE73", Address2:"AAFCFE43", Address3:"ABFCFE52", Address4:"ACFCFE70", Address5:"ADFCFE61", Address6:"AEFCFE51", Address7:"AFFCFE40"


				byte_array = bytearray.fromhex(self.hexadecimal_input)    # STEP 1: Change to hexadecimal_string to run sequence to Off PWM at address 1.

				self.serial.write(byte_array)
				raw = self.serial.read(69)       #You may need to change this for other sensors

				#print("Test",raw,len(raw))
				info = struct.unpack('<BBBBBB',raw)
				print(info,info[4]*1.6)		 #Raw Value Multiply by 1.6 to get actual distance value (Sensor properties). Refer to manual for more information

				return info[4]*1.6/100

				
		except:
			self._print_debug("Unable to read from the battery!")
			print("Unable to read from battery")
			info = 0
		return 0


def _print_out(inputstring):

    sys.stdout.write(inputstring + "\n")
    sys.stdout.flush()

