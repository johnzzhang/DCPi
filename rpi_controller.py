import serial
import sys

# ser = serial.Serial('/dev/ttyACM0',9600) # use with RPi
ser = serial.Serial('COM8',9600) # use with PC
running = True
newest_line = ""
command = ""

while True:
	while True:
		try:
			command = str(input("Would you like to begin a new scan? Answer y to continue "))
			if not (command == 'y'):
				raise ValueError
		except ValueError:
			continue
		break
	running = True
	file_id = str(input("Give this file a unique identifier: "))
	file_name= "scan_" + file_id + ".txt"
	txtfile = open(file_name, "w+")
	ser.write("run".encode())
	while running:
		newest_line = str(ser.readline().decode('utf-8'))[:-1]
		if newest_line[0:14] == "terminate scan":
			txtfile.close()
			print("command received; scan terminated")
			running = False
			break
		else:
			txtfile.write(newest_line)
