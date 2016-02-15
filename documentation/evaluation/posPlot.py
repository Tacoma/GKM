#!/usr/bin/python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

if len(sys.argv) < 2:
	sys.exit("Please give the paths to three log files.")

logFile = open(str(sys.argv[1]), 'r')

#evaluation log 
xPos = []
yPos = []
zPos = []
for line in logFile:
	coords = line.split(",")
	xPos.append(float(coords[0]))
	yPos.append(float(coords[1]))
	zPos.append(float(coords[2]))
	
#static plane def
#point
#pxPos = -2.38791
#pyPos = -2.43330
#pzPos = 1.16667
#normal
#pnxPos = 0
#pnyPos = 1
#pnzPos = 0

#calculate distances
#distances = []
#for i in range(0, len(xPos)):
#	distances.append(xPos[i]*pnxPos + yPos[i]*pnyPos + zPos[i]*pnzPos)

#get min and max points
yMin = min(yPos)
yMax = max(yPos)

#draw
fig = plt.figure()
ax = Axes3D(fig)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.plot(xPos, yPos, zPos, color="blue")

plt.plot([0,0], [yMin,yMax], 'ro')
plt.plot([0,0], [yMin,yMax], '--')

plt.title(yMax-yMin)

plt.show()
