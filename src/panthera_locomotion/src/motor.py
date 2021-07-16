import serial
import time
import keyboard
import serial.tools.list_ports

class RoboteqMotor():
	# 12rpm = 200 units
	def __init__(self, serialnumber):
		self.sn = serialnumber
		p = list(serial.tools.list_ports.grep(self.sn))
		self.port = '/dev/' + p[0].name
		self.ser = serial.Serial(self.port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

		self.wheel_diamter = 0.3
		self.ratio = 1000
		self.initialize()

	def initialize(self):
		self.ser.write("# c\r")                      # Clear buffer
		self.ser.write("?CB\r")                      # select CB for hall sensor or C for encoder
		self.ser.write("# 10\r")                     # read data every 10ms
		self.ser.write("!CB 1 0_!CB 2 0\r")

	def writeSpeed(self, speed):
		rps = self.speed_to_rps(speed)
		cmd = rps * self.ratio
		#self.ser.write("!G {}\r".format(str(rps*60)))
		self.ser.write("!G 1 {}\r".format(str(cmd)))
		#print("Speed: ", cmd)

	def writeTorque(self, data):
		self.ser.write("!G 1 {}\r".format(str(data)))

	def speed_to_rps(self, speed):
		return speed/self.wheel_diamter

	def readCurrent(self):
		self.ser.write("?AC 1\r")
		rpm = self.ser.read(1000)
		print(rpm)

	def writeAcc(self, acc):
		# increase motor spped by 0.1* rpm per s
		# acc is in motor units
		self.ser.write("!AC 1 {}\r".format(str(acc)))

	def e_stop(self):
		self.ser.write("!EX\r")

	def set_mode(self, mode):
		if mode == 0:
			print("open loop mode")
		elif mode == 1:
			print("closed-loop speed mode")
		elif mode == 5:
			print("closed-loop torque mode")
		self.ser.write("^MMOD 1 {}\r".format(str(mode)))

	def set_torque(self, data):
		self.ser.write("!GIQ 1 {}\r".format(str(data*10)))