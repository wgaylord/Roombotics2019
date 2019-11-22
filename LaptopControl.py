 import serial
 ser = serial.Serial("COM12",9600)
 waiting = True
 while True:
     if waiting:
         raw_input("Press enter to start!")
         ser.write(1)
     x = ser.read()
     ser.write(x)
