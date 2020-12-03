import serial

serialPort = serial.Serial(port = "COM6", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
value = bytes([90])
serialPort.write(value)
serialPort.close()
