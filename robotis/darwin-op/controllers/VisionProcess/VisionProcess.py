"""VisionProcess controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from time import time
from controller import Robot, Camera, Display
from controller import Emitter
import struct
import cv2
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
camera = robot.getDevice('camera')
# emitter = robot.getDevice('emitter')
camera.enable(timestep)

print("[%8.3f] Init done" % (robot.getTime()))

while robot.step(timestep) != -1:
    frame = camera.getImage()
    image = np.frombuffer(frame, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    #视觉数据传入pack中即可
    message = struct.pack("ffffffff",155.0,45.0,120.08,20.0,155.0,45.0,120.08,20.0)
    
    # emitter.send(message)
    
    cv2.imshow("image", image)
    cv2.waitKey(1)

pass

# Enter here exit cleanup code.
