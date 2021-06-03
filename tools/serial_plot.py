import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import threading
import serial
import struct
import time
from collections import deque
from datetime import datetime

ser = serial.Serial(
    port='/dev/ttyUSB1',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=100)

save_csv = False
csv_file = 'serial_log.csv'

if save_csv == True:
    csv_token = open(csv_file, "w")

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
        self.plot_time_last = time.time()
        self.update_rate_last = 0

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
        	plt.ylabel('euler angles[deg]')
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

        elif (message_id == 4 or message_id == 8 or message_id == 31):
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

        elif (message_id == 7 or message_id == 30):
         	plt.subplot(111)
        	plt.ylabel('position [cm]')
        	plt.ylim([-200, 200])
        	self.create_curve('x', 'red')
                self.create_curve('y', 'orange')
        	self.create_curve('z', 'yellow')
        	self.show_subplot()

        elif (message_id == 9 or message_id == 32):
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
        	plt.ylabel('magnetic induction [uT]')
        	plt.ylim([-100, 100])
        	self.create_curve('mx', 'red')
                self.create_curve('my', 'blue')
        	self.create_curve('mz', 'green')
        	self.show_subplot()

          	plt.subplot(312)
        	plt.ylabel('magnetic field strength [uT]')
        	plt.ylim([0, 100])
        	self.create_curve('mx', 'red')
        	self.show_subplot()

          	plt.subplot(313)
        	plt.ylabel('update frequency')
        	plt.ylim([0, 100])
        	self.create_curve('mx', 'red')
        	self.show_subplot()

        elif (message_id == 16):
                plt.subplot(511)
                plt.ylabel('barometer freq')
                plt.ylim([-1.0, 100.0])
                self.create_curve('gps freq', 'red')
                self.show_subplot()

          	plt.subplot(512)
        	plt.ylabel('pressure [mbar]')
        	plt.ylim([-0.75, 1.5])
        	self.create_curve('pressure', 'red')
        	self.show_subplot()

          	plt.subplot(513)
        	plt.ylabel('temperature [deg c]')
        	plt.ylim([-30, 75])
        	self.create_curve('temperature', 'red')
        	self.show_subplot()

          	plt.subplot(514)
        	plt.ylabel('height [m]')
        	plt.ylim([-0.15, 2.0])
        	self.create_curve('height', 'red')
        	self.show_subplot()

          	plt.subplot(515)
        	plt.ylabel('velocity [m/s]')
        	plt.ylim([-5.0, 5.0])
        	self.create_curve('velocity', 'red')
        	self.show_subplot()

        elif (message_id == 17):
          	plt.subplot(211)
        	plt.ylabel('height [m]')
        	plt.ylim([-5.0, 5.0])
        	self.create_curve('h_barometer', 'red')
                self.create_curve('h_optitrack', 'blue')
        	self.show_subplot()

        	plt.subplot(212)
        	plt.ylabel('velocity [m/s]')
        	plt.ylim([-5.0, 5.0])
        	self.create_curve('v_barometer', 'red')
                self.create_curve('v_optitrack', 'blue')
        	self.show_subplot()

        elif (message_id == 18):
           	plt.subplot(521)
        	plt.ylabel('time [ms]')
        	plt.ylim([0.0, 600000.0])
        	self.create_curve('time', 'red')
        	self.show_subplot()

           	plt.subplot(522)
        	plt.ylabel('gps update time [ms]')
        	plt.ylim([0.0, 600000.0])
        	self.create_curve('time', 'red')
        	self.show_subplot()

           	plt.subplot(523)
        	plt.ylabel('accel (lpf) [m/s^2]')
        	plt.ylim([-30.0, 30.0])
        	self.create_curve('ax', 'red')
                self.create_curve('ay', 'blue')
                self.create_curve('az', 'green')
        	self.show_subplot()

           	plt.subplot(524)
        	plt.ylabel('gyro (raw) [rad/s]')
        	plt.ylim([-10.0, 10.0])
        	self.create_curve('wx', 'red')
                self.create_curve('wy', 'blue')
                self.create_curve('wz', 'green')
        	self.show_subplot()

           	plt.subplot(525)
        	plt.ylabel('mag (raw) [uT]')
        	plt.ylim([-100, 100])
        	self.create_curve('mx', 'red')
                self.create_curve('my', 'blue')
                self.create_curve('mz', 'green')
        	self.show_subplot()

           	plt.subplot(526)
        	plt.ylabel('gps [deg]')
        	plt.ylim([-180, 180])
        	self.create_curve('longitude', 'red')
                self.create_curve('latitude', 'blue')
                self.create_curve('height', 'green')
        	self.show_subplot()

           	plt.subplot(527)
        	plt.ylabel('gps velocity [m/s]')
        	plt.ylim([-5.0, 5.0])
        	self.create_curve('vx', 'red')
                self.create_curve('vy', 'blue')
                self.create_curve('vz', 'green')
        	self.show_subplot()

                plt.subplot(528)
        	plt.ylabel('barometer height [m]')
        	plt.ylim([-1.0, 5.0])
        	self.create_curve('height', 'red')
        	self.show_subplot()

                plt.subplot(529)
        	plt.ylabel('barometer velocity [m]')
        	plt.ylim([-10.0, 10.0])
        	self.create_curve('velocity', 'red')
        	self.show_subplot()

        elif (message_id == 19):
                plt.subplot(111)
                plt.ylabel('gps raw position [m/s]')
                plt.ylim([-25.0, 25.0])
                self.create_curve('px', 'red')
                self.create_curve('py', 'blue')
                self.create_curve('pz', 'green')
                self.show_subplot()
             
        elif (message_id == 20):
                plt.subplot(421)
                plt.ylabel('time [ms]')
                plt.ylim([0.0, 600000.0])
                self.create_curve('time', 'red')
                self.show_subplot()

                plt.subplot(422)
                plt.ylabel('sv num')
                plt.ylim([0.0, 50.0])
                self.create_curve('sv num', 'red')
                self.show_subplot()

                plt.subplot(423)
                plt.ylabel('gps freq')
                plt.ylim([0.0, 50.0])
                self.create_curve('gps freq', 'red')
                self.show_subplot()

                plt.subplot(424)
                plt.ylabel('gps position [m]')
                plt.ylim([-50.0, 50.0])
                self.create_curve('px', 'red')
                self.create_curve('py', 'blue')
                self.create_curve('pz', 'green')
                self.show_subplot()

                plt.subplot(425)
                plt.ylabel('ins position [m]')
                plt.ylim([-50.0, 50.0])
                self.create_curve('px', 'red')
                self.create_curve('py', 'blue')
                self.create_curve('pz', 'green')
                self.show_subplot()
           
                plt.subplot(426)
                plt.ylabel('gps velocity [m/s]')
                plt.ylim([-5.0, 5.0])
                self.create_curve('vx', 'red')
                self.create_curve('vy', 'blue')
                self.create_curve('vz', 'green')
                self.show_subplot()

                plt.subplot(427)
                plt.ylabel('ins velocity [m/s]')
                plt.ylim([-5.0, 5.0])
                self.create_curve('vx', 'red')
                self.create_curve('vy', 'blue')
                self.create_curve('vz', 'green')
                self.show_subplot()

        elif (message_id == 21):
          	plt.subplot(511)
        	plt.ylabel('mag [uT]')
        	plt.ylim([-100, 100])
        	self.create_curve('mx', 'red')
                self.create_curve('my', 'blue')
        	self.create_curve('mz', 'green')
        	self.show_subplot()

          	plt.subplot(512)
        	plt.ylabel('mag strength [uT]')
        	plt.ylim([0, 100])
        	self.create_curve('mx', 'red')
        	self.show_subplot()

          	plt.subplot(513)
        	plt.ylabel('update freq')
        	plt.ylim([0, 100])
        	self.create_curve('mx', 'red')
        	self.show_subplot()

                plt.subplot(514)
        	plt.ylabel('quality')
        	plt.ylim([-1.2, 1.2])
        	self.create_curve('1: good 0: bad', 'red')
        	self.show_subplot()

                plt.subplot(515)
        	plt.ylabel('deg')
        	plt.ylim([-200, 200])
        	self.create_curve('compass yaw', 'red')
        	self.create_curve('ahrs yaw', 'blue')
        	self.show_subplot()

        elif (message_id == 22):
          	plt.subplot(111)
        	plt.ylabel('eskf1 P matrix')
        	plt.ylim([-100, 100])
        	self.create_curve('P00', 'blue')
                self.create_curve('P11', 'orange')
        	self.create_curve('P22', 'green')
                self.create_curve('P33', 'red')
                self.create_curve('P44', 'purple')
                self.create_curve('P55', 'brown')
                self.create_curve('P66', 'pink')
                self.create_curve('P77', 'gray')
                self.create_curve('P88', 'cyan')
                self.show_subplot()

        elif (message_id == 33):
          	plt.subplot(111)
        	plt.ylabel('eskf1 P matrix')
        	plt.ylim([-1, 30])
        	self.create_curve('h_acc', 'blue')
                self.create_curve('v_acc', 'red')
                self.show_subplot()


    def show_graph(self):
	ani = animation.FuncAnimation(self.figure, self.animate, np.arange(0, 200), \
		interval=0, blit=True)

	plt.show()

    def save_csv(self, datas):
            for i in range(0, len(datas)):
                data_str = "{:.7f}".format(float(datas[i]))

                if i == (self.curve_number - 1):
                    csv_token.write(data_str)
                    csv_token.write('\n')
                else:
                    csv_token.write(data_str)
                    csv_token.write(',')

    def serial_receive(self):
        buffer = []
        checksum = 0

        c = ser.read(1)

        #wait for start byte
        if c == '@':
            pass
        else:
            return 'fail'

        #receive package size
        payload_count, =  struct.unpack("B", ser.read(1))
        #print('payload size: %d' %(payload_count))

        #receive message id
        _message_id, =  struct.unpack("c", ser.read(1))
        message_id = ord(_message_id)

        buf = ser.read(payload_count + 1)

        #receive payload and calculate checksum
        for i in range(0, payload_count):
            buffer.append(buf[i])
            buffer_checksum ,= struct.unpack("B", buffer[i])
            checksum ^= buffer_checksum

        received_checksum ,= struct.unpack("B", buf[payload_count])
        if received_checksum != checksum:
                print("error: checksum mismatch");
                return 'fail'
        else:
                #print("checksum is correct (%d)" %(checksum))
                pass

        plot_time_now = time.time()
        update_rate = 1.0 / (plot_time_now - self.plot_time_last)
        self.plot_time_last = plot_time_now

        if abs(update_rate - self.update_rate_last) > 50:
            print('[%s]received message #%d' \
                %(datetime.now().strftime('%H:%M:%S'), \
                message_id))
            print("telemetry speed is too slow, the update rate is unknown!")
        else:
            print('[%s]received message #%d (%.1fHz)' \
                %(datetime.now().strftime('%H:%M:%S'), \
                message_id, update_rate))

        self.update_rate_last = update_rate

        if self.plot_begin == False:
            self.curve_number = payload_count / 4
            self.serial_data = [serial_data_class(200) for i in range(0, self.curve_number)]
            self.curve_indexs = [i for i in range(0, self.curve_number)]
            self.set_figure(message_id)
            self.plot_begin = True

        recvd_datas = []

        for i in range(0, self.curve_number):
            #unpack received data
            binary_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
            float_data = np.asarray(struct.unpack("f", binary_data))
            self.serial_data[i].add(float_data)
            print("payload #%d: %f" %(i, float_data))

            recvd_datas.append(float_data)

        if save_csv == True:
            self.save_csv(recvd_datas)

        #print("-----------------------------");

        return 'success'

serial_plotter = serial_plotter_class()

class serial_thread(threading.Thread):
	def run(self):
		while True:
		    serial_plotter.serial_receive()

        def join(self):
            super(serial_thread, self).join()

serial_thread().start()

while serial_plotter.plot_begin == False:
    continue

serial_plotter.show_graph()
