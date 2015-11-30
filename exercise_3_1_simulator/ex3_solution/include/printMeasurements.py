#!/usr/bin/python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

if len(sys.argv) < 4:
	sys.exit("Please give the paths to three log files.")

logFile0 = open(str(sys.argv[1]), 'r')
logFile1 = open(str(sys.argv[2]), 'r')
logFile2 = open(str(sys.argv[3]), 'r')

#logfile 0
xPos0 = []
yPos0 = []
zPos0 = []
for line in logFile0:
	coords = line.split(" ")
	xPos0.append(float(coords[1]))
	yPos0.append(float(coords[2]))
	zPos0.append(float(coords[3]))
#logfile 1
xPos1 = []
yPos1 = []
zPos1 = []
for line in logFile1:
	coords = line.split(" ")
	xPos1.append(float(coords[1]))
	yPos1.append(float(coords[2]))
	zPos1.append(float(coords[3]))
#logfile 2
xPos2 = []
yPos2 = []
zPos2 = []
for line in logFile2:
	coords = line.split(" ")
	xPos2.append(float(coords[1]))
	yPos2.append(float(coords[2]))
	zPos2.append(float(coords[3]))

#draw
fig = plt.figure()
ax = Axes3D(fig)

ax.plot(xPos0, yPos0, zPos0, color="blue")
ax.plot(xPos1, yPos1, zPos1, color="red")
ax.plot(xPos2, yPos2, zPos2, color="green")
plt.show()
