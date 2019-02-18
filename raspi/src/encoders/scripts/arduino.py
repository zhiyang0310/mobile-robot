from serial import Serial
import threading
import sys

class Arduino:

    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5, motors_reversed=False):

        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
        self.motors_reversed = motors_reversed
        # Keep things thread safe
        self.mutex = threading.Lock()

    def connect(self):
        print("Connecting to Arduino on port", self.port, "...")
        self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
        # The next line is necessary to give the firmware time to wake up.
        self.port.close()
        self.port.open()
        print("Arduino is ready.")
    
    def open(self):
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self):
        ''' Close the serial port.
        '''
        self.port.close()

    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write((cmd + '\r').encode('ascii'))

    def recv(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1).decode()
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > self.timeout:
                return None

        value = value.strip('\r')

        return value
    
    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        values = self.recv().split()
        return values
    

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0
        
        self.send(cmd)
        value = self.recv()

        self.mutex.release()
        return int(value)

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        self.send(cmd)
        values = self.recv_array()

        self.mutex.release()
        return list(map(int,values))

    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        self.port.write((cmd + "\r").encode('ascii'))
        ack = self.recv()

        self.mutex.release()
        return ack == 'OK'

    def update_pid_L(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print ("Updating PID parameters")
        cmd = 'a ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)

    def update_pid_R(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print ("Updating PID parameters")
        cmd = 'b ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2:
            print ("Encoder count was not 2")
            return None
        else:
            if self.motors_reversed:
                values[0], values[1] = -values[0], -values[1]
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')

    def drive(self ,left, right):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        #if self.motors_reversed:
        #    left, right = -left, -right
        cmd = "m "+str(left)+" "+str(right)
        return self.execute_ack(cmd)

    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    def analog_write(self, pin, value):
        cmd = 'x '+str(pin)+' '+str(value)
        return self.execute_ack(cmd)

    def digital_write(self, pin, value):
        cmd = 'x '+str(pin)+' '+str(value)
        return self.execute_ack(cmd)