#!/usr/bin/env python
#
# This is a NetworkTables client (eg, the DriverStation/coprocessor side).
# You need to tell it the IP address of the NetworkTables server (the
# robot or simulator).
#
# When running, this will continue incrementing the value 'dsTime', and the
# value should be visible to other networktables clients and the robot.
#

import sys
import time
import random
from networktables import NetworkTable

# To see messages from networktables, you must setup logging
#import logging
#logging.basicConfig(level=logging.DEBUG)

if len(sys.argv) < 2:
    print("Error: specify an IP to connect to!")
    print("Going with default: ")
    ip = "roboRIO-1262-FRC.local"
    print(ip)
else:
    ip = sys.argv[1]

NetworkTable.setIPAddress(ip)
NetworkTable.setClientMode()
NetworkTable.initialize()

sd = NetworkTable.getTable("RaspberryPI")

i = 0
while True:
    sd.putNumber("x", i)
    time.sleep(1)
    i += 1
