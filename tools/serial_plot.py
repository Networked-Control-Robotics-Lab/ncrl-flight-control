import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import threading
import serial
import struct
from collections import deque
from datetime import datetime

ser = serial.Serial(
    port='/dev/ttyUSB1',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=0)

print("connected to: " + ser.portstr)

class serial_data_class:
    def __init__(self, max_count):
	self.max_count = max_count
	self.data = deque([0.0] * max_count)

    def add(self, value):
	if len(self.data) < self.max_count:
		self.data.append(val)
	else:
		self.data.pop()
	self.data.appendleft(value)

class serial_plotter_class:
    def __init__(self):
	self.figure = plt.figure(figsize=(14,8))
	self.curve = []
	self.current_curve_count = 0
        self.plot_begin = False;

    def set_graph(curve_count, serial_data):
	self.curve_number = curve_count
	self.serial_data = serial_data

    def create_curve(self, label_name, curve_color):
	for i in range(0, len(self.curve_indexs)):
		if self.curve_indexs[i] == self.current_curve_count:
			self.curve.append(plt.plot(self.serial_data[self.current_curve_count].data, \
				label=label_name, color=curve_color, animated=True)[0])

	self.current_curve_count += 1

    def show_subplot(self):
	plt.grid()
	plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), \
		ncol=3, fancybox=True, shadow=True)

    def animate(self, i):
	for index in range(0, len(self.curve_indexs)):
		self.curve[index].set_ydata( \
			self.serial_data[self.curve_indexs[index]].data)

	return self.curve

    def set_figure(self, message_id):
        if(message_id == 0):
        	plt.subplot(511)
        	plt.ylabel('accel [m/s^2]')
        	plt.ylim([-30, 30])
        	self.create_curve('x (raw)', 'red')		
        	self.create_curve('y (raw)', 'blue')		
        	self.create_curve('z (raw)', 'green')		
        	self.show_subplot()

                plt.subplot(512)
        	plt.ylabel('accel [m/s^2]')
        	plt.ylim([-30, 30])
        	self.create_curve('x (lpf)', 'red')		
        	self.create_curve('y (lpf)', 'blue')		
        	self.create_curve('z (lpf)', 'green')		
        	self.show_subplot()

        	plt.subplot(513)
        	plt.ylabel('gyro [deg/s]')
        	plt.ylim([-450, 450])
        	self.create_curve('x (raw)', 'red')		
        	self.create_curve('y (raw)', 'blue')		
        	self.create_curve('z (raw)', 'green')		
        	self.show_subplot()

                plt.subplot(514)
        	plt.ylabel('gyro [deg/s]')
        	plt.ylim([-450, 450])
        	self.create_curve('x (lpf)', 'red')		
        	self.create_curve('y (lpf)', 'blue')		
        	self.create_curve('z (lpf)', 'green')		
        	self.show_subplot()

                plt.subplot(515)
        	plt.ylabel('temperature')
        	plt.ylim([-25, 100])
        	self.create_curve('temperature', 'red')		
        	self.show_subplot()
        elif (message_id == 1):
                plt.subplot(111)
        	plt.ylabel('gyro [deg/s]')
        	plt.ylim([-450, 450])
        	self.create_curve('roll', 'red')		
        	self.create_curve('pitch', 'blue')		
        	self.create_curve('yaw', 'green')		
        	self.show_subplot()
        elif (message_id == 2):
                plt.subplot(311)
        	plt.ylabel('attitude [deg]')
        	plt.ylim([-450, 450])
        	self.create_curve('roll', 'red')
        	self.create_curve('pitch', 'blue')
        	self.create_curve('yaw', 'green')
        	self.show_subplot()

                plt.subplot(312)
        	plt.ylabel('accel [m/s^2]')
        	plt.ylim([-30, 30])
        	self.create_curve('x (lpf)', 'red')
        	self.create_curve('y (lpf)', 'blue')
        	self.create_curve('z (lpf)', 'green')
        	self.show_subplot()

        	plt.subplot(313)
        	plt.ylabel('gyro [deg/s]')
        	plt.ylim([-450, 450])
        	self.create_curve('x (lpf)', 'red')
        	self.create_curve('y (lpf)', 'blue')
        	self.create_curve('z (lpf)', 'green')
        	self.show_subplot()
        elif (message_id == 3):
         	plt.subplot(211)
        	plt.ylabel('Var(P)')
        	plt.ylim([-5, 5])
        	self.create_curve('P[0][0]', 'red')
        	self.create_curve('P[1][1]', 'blue')
        	self.create_curve('P[2][2]', 'green')
        	self.create_curve('P[3][3]', 'orange')
        	self.show_subplot()

                plt.subplot(212)
        	plt.ylabel('K')
        	plt.ylim([-50, 50])
        	self.create_curve('K[0][0]', 'red')
        	self.create_curve('K[1][1]', 'blue')
        	self.create_curve('K[2][2]', 'green')
        	self.create_curve('K[3][3]', 'orange')
        	self.show_subplot()
        elif (message_id == 4 or message_id == 8):
         	plt.subplot(111)
        	plt.ylabel('Attitude (quaternion)')
        	plt.ylim([-5, 5])
        	self.create_curve('q0', 'red')
        	self.create_curve('q1', 'blue')
        	self.create_curve('q2', 'green')
        	self.create_curve('q3', 'orange')
        	self.show_subplot()
        elif (message_id == 5):
         	plt.subplot(111)
        	plt.ylabel('pid controller debug')
        	plt.ylim([-50, 50])
        	self.create_curve('error', 'red')
                self.create_curve('error derivative', 'orange')
        	self.create_curve('p term', 'yellow')
                self.create_curve('i term', 'green')
                self.create_curve('d term', 'blue')
        	self.create_curve('pid final', 'purple')
        	self.show_subplot()
        elif (message_id == 6):
         	plt.subplot(111)
        	plt.ylabel('motor [thrust %]')
        	plt.ylim([-20, 120])
        	self.create_curve('m1', 'red')
                self.create_curve('m2', 'orange')
        	self.create_curve('m3', 'yellow')
        	self.create_curve('m4', 'purple')
        	self.show_subplot()
        elif (message_id == 7):
         	plt.subplot(111)
        	plt.ylabel('position [cm]')
        	plt.ylim([-200, 200])
        	self.create_curve('x', 'red')
                self.create_curve('y', 'orange')
        	self.create_curve('z', 'yellow')
        	self.show_subplot()
        elif (message_id == 9):
         	plt.subplot(311)
        	plt.ylabel('velocity (raw) [cm/s]')
        	plt.ylim([-200, 200])
        	self.create_curve('vx', 'red')
                self.create_curve('vy', 'orange')
        	self.create_curve('vz', 'yellow')
        	self.show_subplot()

         	plt.subplot(312)
        	plt.ylabel('velocity (lpf) [cm/s]')
        	plt.ylim([-200, 200])
        	self.create_curve('vx', 'red')
                self.create_curve('vy', 'orange')
        	self.create_curve('vz', 'yellow')
        	self.show_subplot()

                plt.subplot(313)
                plt.ylabel('update frequency')
                plt.ylim([0, 200])
                self.create_curve('update freq', 'red')
                self.show_subplot()
        elif (message_id == 10):
         	plt.subplot(111)
        	plt.ylabel('value')
        	plt.ylim([-200, 200])
        	self.create_curve('float variable', 'red')
        	self.show_subplot()
        elif (message_id == 11):
          	plt.subplot(411)
        	plt.ylabel('attitude error [deg]')
        	plt.ylim([-200, 200])
        	self.create_curve('roll', 'red')
                self.create_curve('pitch', 'blue')
        	self.create_curve('yaw', 'green')
        	self.show_subplot()

         	plt.subplot(412)
        	plt.ylabel('attitude rate error [deg/s]')
        	plt.ylim([-200, 200])
        	self.create_curve('wx', 'red')
                self.create_curve('wy', 'blue')
        	self.create_curve('wz', 'green')
        	self.show_subplot()
 
                plt.subplot(413)
                plt.ylabel('feedback')
        	plt.ylim([-3, 3])
        	self.create_curve('Mx', 'red')
                self.create_curve('My', 'blue')
        	self.create_curve('Mz', 'green')
        	self.show_subplot()

                plt.subplot(414)
                plt.ylabel('feedfoward')
        	plt.ylim([-3, 3])
        	self.create_curve('Mx', 'red')
                self.create_curve('My', 'blue')
        	self.create_curve('Mz', 'green')
        	self.show_subplot()

        elif (message_id == 12):
          	plt.subplot(211)
        	plt.ylabel('Moment (N*m)')
        	plt.ylim([-1, 1])
        	self.create_curve('Mx', 'red')
                self.create_curve('My', 'blue')
        	self.create_curve('Mz', 'green')
        	self.show_subplot()

          	plt.subplot(212)
        	plt.ylabel('Moment_rot_frame (N*m)')
        	plt.ylim([-1, 1])
        	self.create_curve('Mx', 'red')
                self.create_curve('My', 'blue')
        	self.create_curve('Mz', 'green')
        	self.show_subplot()

        elif (message_id == 13):
                plt.subplot(211)
        	plt.ylabel('accel [m/s^2]')
        	plt.ylim([-30, 30])
        	self.create_curve('gravity norm', 'red')
        	self.show_subplot()

                plt.subplot(212)
                plt.ylabel('true(1), false(0)')
                plt.ylim([-1.5, 1.5])
                self.create_curve('free fall detected', 'red')

        elif (message_id == 14):
          	plt.subplot(111)
        	plt.ylabel('position error (cm)')
        	plt.ylim([-300, 300])
        	self.create_curve('ex', 'red')
                self.create_curve('ey', 'blue')
        	self.create_curve('ez', 'green')
        	self.show_subplot()

        elif (message_id == 15):
          	plt.subplot(311)
        	plt.ylabel('magnetic induction (uT)')
        	plt.ylim([-10000, 10000])
        	self.create_curve('mx', 'red')
                self.create_curve('my', 'blue')
        	self.create_curve('mz', 'green')
        	self.show_subplot()

          	plt.subplot(312)
        	plt.ylabel('magnetic field strength (uT)')
        	plt.ylim([0, 10000])
        	self.create_curve('mx', 'red')
        	self.show_subplot()

          	plt.subplot(313)
        	plt.ylabel('update frequency')
        	plt.ylim([0, 100])
        	self.create_curve('mx', 'red')
        	self.show_subplot()

    def show_graph(self):
	ani = animation.FuncAnimation(self.figure, self.animate, np.arange(0, 200), \
		interval=0, blit=True)

	plt.show(block=False)

    def serial_receive(self):
        while ser.inWaiting() > 0:
            buffer = []
            checksum = 0

            c = ser.read(1)

            #wait for start byte
            if c == '@':
                #print('start byte received')
                pass
            else:
                continue

            #receive package size
            while ser.inWaiting() == 0:
                continue
            payload_count, =  struct.unpack("B", ser.read(1))
            #print('payload size: %d' %(payload_count))

            #receive message id
            while ser.inWaiting() == 0:
                continue
            _message_id, =  struct.unpack("c", ser.read(1))
            message_id = ord(_message_id)
            print('[%s]received message, id:%d' %(datetime.now().strftime('%H:%M:%S'), message_id))

            #receive payload and calculate checksum
            for i in range(0, payload_count):
                while ser.inWaiting() == 0:
                    continue
                buffer.append(ser.read(1))
                buffer_checksum ,= struct.unpack("B", buffer[i])
                checksum ^= buffer_checksum

            #checksum test
            while ser.inWaiting() == 0:
                continue

            received_checksum ,= struct.unpack("B", ser.read())
            if received_checksum != checksum:
                    print("error: checksum mismatch");
                    return 'fail'
            else:
                    #print("checksum is correct (%d)" %(checksum))
                    pass

            if self.plot_begin == False:
                self.curve_number = payload_count / 4
                self.serial_data = [serial_data_class(200) for i in range(0, self.curve_number)]
                self.curve_indexs = [i for i in range(0, self.curve_number)]
        	self.set_figure(message_id)
                self.plot_begin = True

            for i in range(0, self.curve_number):
                #unpack received data
                binary_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
                float_data = np.asarray(struct.unpack("f", binary_data))
                self.serial_data[i].add(float_data)
                print("received: %f" %(float_data))
            #print("-----------------------------");
            return 'success'

serial_plotter = serial_plotter_class()

class serial_thread(threading.Thread):
	def run(self):
		while True:
		    serial_plotter.serial_receive()

        def join(self):
            super().join()

serial_thread().start()

while serial_plotter.plot_begin == False:
    continue

serial_plotter.show_graph()
