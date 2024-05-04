#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty
import socket
import serial
import struct
import time
import numpy
from math import *
from quat import *

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *


CMD_PING = 0x0001
LEN_PING = 2

CRC = 0xCACB

def q_to_mat4(q):
    w, x, y, z = q
    return numpy.array(
        [[1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w, 0],
         [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w, 0],
         [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y, 0],
         [0, 0, 0, 1]], 'f')

axis_verts = (
    (-7.5, 0.0, 0.0),
    ( 7.5, 0.0, 0.0),
    ( 0.0,-7.5, 0.0),
    ( 0.0, 7.5, 0.0),
    ( 0.0, 0.0,-7.5),
    ( 0.0, 0.0, 7.5)
    )

axes = (
    (0,1),
    (2,3),
    (4,5)
    )

axis_colors = (
    (1.0,0.0,0.0), # Red
    (0.0,1.0,0.0), # Green
    (0.0,0.0,1.0)  # Blue
    )


'''
       5____________6
       /           /|
      /           / |
    1/__________2/  |
    |           |   |
    |           |   |
    |           |   7
    |           |  /
    |           | /
    0___________3/
'''

cube_verts = (
    (-3.0,-3.0, 3.0),
    (-3.0, 3.0, 3.0),
    ( 3.0, 3.0, 3.0),
    ( 3.0,-3.0, 3.0),
    (-3.0,-3.0,-3.0),
    (-3.0, 3.0,-3.0),
    ( 3.0, 3.0,-3.0),
    ( 3.0,-3.0,-3.0)
    )

cube_edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,6),
    (5,1),
    (5,4),
    (5,6),
    (7,3),
    (7,4),
    (7,6)
    )

cube_surfaces = (
    (0,1,2,3), # Front
    (3,2,6,7), # Right
    (7,6,5,4), # Left
    (4,5,1,0), # Back
    (1,5,6,2), # Top
    (4,0,3,7)  # Bottom
    )

cube_colors = (
    (0.769,0.118,0.227), # Red
    (  0.0,0.318,0.729), # Blue
    (  1.0,0.345,  0.0), # Orange
    (  0.0, 0.62,0.376), # Green
    (  1.0,  1.0,  1.0), # White
    (  1.0,0.835,  0.0)  # Yellow
    )

def Axis():
    glBegin(GL_LINES)
    for color,axis in zip(axis_colors,axes):
        glColor3fv(color)
        for point in axis:
            glVertex3fv(axis_verts[point])
    glEnd()

def Cube():
    glBegin(GL_QUADS)
    for color,surface in zip(cube_colors,cube_surfaces):
        glColor3fv(color)
        for vertex in surface:
            glVertex3fv(cube_verts[vertex])
    glEnd()

    glBegin(GL_LINES)
    glColor3fv((0.0,0.0,0.0))
    for edge in cube_edges:
        for vertex in edge:
            glVertex3fv(cube_verts[vertex])
    glEnd()

def main():
    print("Hello boss!")

    pygame.init()
    display = (1800,1718)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    # Using depth test to make sure closer colors are shown over further ones
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    # Default view
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (display[0]/display[1]), 0.5, 40)
    glTranslatef(0.0,0.0,-17.5)    

    port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout = 0.12)

    while True:
        rxb = port.read(60)
        #print(rxb)
        orientation = struct.unpack("<2sHHfffffffffffff", rxb[0:58])

        rotation = orientation[3:6]
        quaternion = orientation[6:10]
        gravity = orientation[10:13]
        acceleration = orientation[13:16]

        print(rotation, quaternion, gravity, acceleration)

        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(q_to_mat4([quaternion[0], quaternion[2], quaternion[1], quaternion[3]]))

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        Cube()
        Axis()
        pygame.display.flip()

    seq = 0
    while True:
        cmd = CMD_PING
        len = LEN_PING

        if 0: #seq % 5  == 1:
            ping = struct.pack('<2sHHH', b'mb', cmd, len, seq)
        else:
            ping = struct.pack('<2sHHHH', b'mb', cmd, len, seq, CRC) 

        port.write(ping)

        print("ping #" + str(seq))

        pong = port.read(25)
        print(pong)

        time.sleep(5)

        seq += 1

        

if __name__ == '__main__':
    main()