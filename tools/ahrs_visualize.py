import pygame
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import threading
import serial
import struct
from collections import deque
from datetime import datetime
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from math import *

ser = serial.Serial(
    port='/dev/ttyUSB1',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=0)

q = [1.0, 0.0, 0.0, 0.0]

#transformation matrix using quaternion
def q_to_mat4(q0, q1, q2 ,q3):
    return np.array(
        [[1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3), 2*(q0*q2 + q1*q3), 0],
         [2*(q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 - q0*q1), 0],
         [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3, 0],
         [0, 0, 0, 1]],'f')


def quat_to_ypr(q0, q1, q2, q3):
    roll = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    pitch = math.sin(2*(q0*q2 - q3*q1))
    yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    roll *= 180.0 / math.pi
    pitch *= 180.0 / math.pi
    yaw *= 180.0 / math.pi
    return [yaw, pitch, roll]


def gl_draw_text(x, y, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glWindowPos2f(x, y)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def visualize_quaternion_attitude():
    pygame.init()
    pygame.display.set_mode((640, 480), DOUBLEBUF|OPENGL)
    pygame.display.set_caption("loyalty_fc attitude visualization")

    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, 1.0* 640 / 480, 0.1, 100.0)
    glTranslatef(0.0,0.0,-7.0)

    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(q_to_mat4(q[0], q[2], -q[3], q[1]))

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        [yaw, pitch , roll] = quat_to_ypr(q[0], q[1], q[2], q[3])
        gl_draw_text(40, 40, "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
        gl_draw_text(40, 440, "loyalty_fc attitude visualization", 18)
        gl_draw_text(40, 420, "Module to visualize quaternion or Euler angles data", 16)
        gl_draw_text(40, 20, "Press Escape to exit.", 16)

        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(1.0, 0.5, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

        pygame.display.flip()
        pygame.time.wait(10)

def serial_receive():
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
        if message_id != 4 and message_id != 8:
            continue
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

        received_checksum ,= struct.unpack("B", ser.read(1))
        if received_checksum != checksum:
                print("error: checksum mismatch");
                return 'fail'
        else:
                #print("checksum is correct (%d)" %(checksum))
                pass

        for i in range(0, 4):
            #unpack received data
            binary_data = ''.join([buffer[i * 4], buffer[i * 4 + 1], buffer[i * 4 + 2], buffer[i * 4 + 3]])
            float_data = np.asarray(struct.unpack("f", binary_data))
            q[i] = float_data
            print("received: %f" %(float_data))
        #print("-----------------------------");
        return 'success'

class serial_thread(threading.Thread):
	def run(self):
		while True:
			serial_receive()
        def join(self):
            super().join()

serial_thread().start()

visualize_quaternion_attitude()
