import arduino as ard
import matplotlib.pyplot as plt
import time

portName = "/dev/ttyACM0"

baudRate = 57600

myArduino = ard.Arduino(port=portName, baudrate=baudRate, timeout=0.5)
myArduino.connect()

print ("Sleeping for 1 second...")

print ("Connection test successful.")

print(myArduino.update_pid_L(85,80,0,60))
print(myArduino.update_pid_R(83,80,0,60))


ticks = [0]
x = [1]
inc = 0
lastticks = 0
print(myArduino.reset_encoders())
myArduino.drive(150,150)
while True:
    currentticks = myArduino.get_encoder_counts()[1]
    ticks.append((currentticks - lastticks)*33)
    lastticks = currentticks
    inc += 1
    x.append(inc)
    if inc > 80:
        break
    time.sleep(0.03)

print(ticks)
plt.plot(x,ticks)
plt.show()

myArduino.close()

print ("Shutting down Arduino.")