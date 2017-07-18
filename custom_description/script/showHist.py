#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from custom_description.msg import Histogram
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import threading

xData = []
yData = []
yBinData = []
yLowThreshold = 0
yHighThreshold = 0
lock = threading.Lock()
numBin = 0
fig = plt.figure()
originPlt = plt.subplot(211)
resultPlt = plt.subplot(212)


def histComeCb(data):
    global xData
    global yData
    global yBinData
    global numBin
    global lock
    global yLowThreshold, yHighThreshold

    # rospy.loginfo("I heard %s", data.xData)
    lock.acquire()
    xData = data.xData
    yData = data.yData
    yBinData = data.yBinData
    yLowThreshold = data.yLowThreshold
    yHighThreshold = data.yHighThreshold
    numBin = data.num_bin
    lock.release()


def listener():
    rospy.init_node('ShowHist', anonymous=False)

    rospy.Subscriber("hist", Histogram, histComeCb)


def update(frame):
    resultPlt.clear()
    originPlt.clear()
    global lock

    lock.acquire()

    originPlt.bar(xData, yData, color='b', width=5)
    originPlt.hlines(yLowThreshold, 0, 180, 'r')
    originPlt.hlines(yHighThreshold, 0, 180, 'r')
    originPlt.set_xticks(
        (15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180))
    originPlt.set_title("Origin Histogram")

    resultPlt.bar(xData, yBinData, color='b', width=5)
    resultPlt.set_xticks(
        (15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180))
    resultPlt.set_title("Marked Histogram")

    lock.release()


def spin():
    rospy.spin()


if __name__ == '__main__':
    listener()

    #spin thread
    spinThread = threading.Thread(target=spin)
    spinThread.start()

    #show
    anim = FuncAnimation(fig, update, interval=1000, blit=False)
    plt.show()
