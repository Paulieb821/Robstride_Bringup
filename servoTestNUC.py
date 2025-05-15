import sys, serial
print("serial module path:", serial.__file__)
print("serial dir:", dir(serial))
# now try:
ser = serial.Serial("/dev/ttyUSB0", 9600)
print("OK, got:", ser)
