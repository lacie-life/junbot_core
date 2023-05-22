#!/usr/bin/env python

import os
import signal
import sys
import math 
import time

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import *
from nav_msgs.msg import *


MARKER_TIME_INTERVAL = 1.0
MARKER_TIME_INTERVAL_BIG = 50
MARKER_TIME_INTERVAL_MID = 10



class DataOdometry(object):

    #---------------------------------------------------------------------------
    def __init__(self):
        rospy.loginfo("[DataOdometry] init()")
        self.prevOdom = None
        self.currOdom = None
        self.iniPosX = None
        self.iniPosY = None
        self.initTime = None
        self.lastTime = None
        self.totalTime = 0.0
        self.travDist = 0.0
        self.numSamples = 0
                
        self.dataTime = []
        self.dataTravDist = []
        
        self.dataX = []
        self.dataY = []
        self.dataZ = []
        self.dataVx = []
        self.dataVy = []
        self.dataVz = []
        self.dataAx = []
        self.dataAy = []
        self.dataAz = []

        self.dataBVx = []
        self.dataBVy = []
        self.dataBVz = []

        self.dataR = []
        self.dataP = []
        self.dataH = []
        self.dataVr = []
        self.dataVp = []
        self.dataVh = []
        
        self.markerPointX = []
        self.markerPointY = []
        self.markerPointZ = []
        self.markerPointHeading = []
        self.markerPointTime = []
        self.markerLabelStr = []
        self.markerLabelTime = 0
                

    #---------------------------------------------------------------------------
    def getCurrentPose(self):
        return self.currOdom


    #---------------------------------------------------------------------------
    # Data in the plots: X=north / Y=east / altitude is positive - data conversion is performed

    def odometryHandler(self, msg):
        #Set initial position - this will be point 0 / 0 in the plot
        if (self.currOdom == None):
            self.iniPosX = msg.pose.pose.position.x
            self.iniPosY = msg.pose.pose.position.y
    
        self.currOdom = msg

        self.currOdom.pose.pose.position.x = self.currOdom.pose.pose.position.x - self.iniPosX
        self.currOdom.pose.pose.position.y = self.currOdom.pose.pose.position.y - self.iniPosY

        self.lastTime = msg.header.stamp.to_sec()
        if (self.initTime == None):
            self.initTime = msg.header.stamp.to_sec()
            
        if (self.prevOdom == None):
            self.prevOdom = self.currOdom
       
        #Only store data if minimum time has passed since previous sample
        dt = self.currOdom.header.stamp.to_sec() - self.prevOdom.header.stamp.to_sec()
        if (dt > 0.19):                
            self.totalTime = self.totalTime + dt
            rospy.logdebug(self.totalTime)
            
            #=== TIME
            self.dataTime.append(self.totalTime)

            #=== POSITION
            self.dataX.append(self.currOdom.pose.pose.position.y)
            self.dataY.append(self.currOdom.pose.pose.position.x)
            self.dataZ.append(-self.currOdom.pose.pose.position.z)

            #=== POS DIFF
            dx = (self.currOdom.pose.pose.position.y - self.prevOdom.pose.pose.position.y)
            dy = (self.currOdom.pose.pose.position.x - self.prevOdom.pose.pose.position.x)
            dz = (self.currOdom.pose.pose.position.z - self.prevOdom.pose.pose.position.z)

            self.travDist = self.travDist + math.sqrt(dx*dx + dy*dy)
            self.dataTravDist.append(self.travDist)

            #=== VELOCITY
            vx = (self.currOdom.pose.pose.position.y - self.prevOdom.pose.pose.position.y) / dt
            vy = (self.currOdom.pose.pose.position.x - self.prevOdom.pose.pose.position.x) / dt
            vz = (self.currOdom.pose.pose.position.z - self.prevOdom.pose.pose.position.z) / dt

            #Apply low pass filter for velocity
            prev_vx = 0.0
            prev_vy = 0.0
            prev_vz = 0.0
            if (self.numSamples > 0):
                prev_vx = self.dataVx[self.numSamples - 1]
                prev_vy = self.dataVy[self.numSamples - 1]
                prev_vz = self.dataVz[self.numSamples - 1]

            vx = 0.6*prev_vx + 0.4*vx
            vy = 0.6*prev_vy + 0.4*vy
            vz = 0.6*prev_vz + 0.4*vz
            
            self.dataVx.append(vx)
            self.dataVy.append(vy)
            self.dataVz.append(vz)

            self.dataBVx.append(self.currOdom.twist.twist.linear.x)
            self.dataBVy.append(self.currOdom.twist.twist.linear.y)
            self.dataBVz.append(-self.currOdom.twist.twist.linear.z)

            velXY = math.sqrt(vx*vx + vy*vy)
            velZ  = math.fabs(vz)
            velTotal = math.sqrt(vx*vx + vy*vy + vz*vz)
            
            #=== ACCELERATION
            ax = (self.currOdom.twist.twist.linear.y - self.prevOdom.twist.twist.linear.y) / dt
            ay = (self.currOdom.twist.twist.linear.x - self.prevOdom.twist.twist.linear.x) / dt
            az = (self.currOdom.twist.twist.linear.z - self.prevOdom.twist.twist.linear.z) / dt
                
            #Apply low pass filter for acceleration
            prev_ax = 0.0
            prev_ay = 0.0
            prev_az = 0.0
            if (self.numSamples > 0):
                prev_ax = self.dataAx[self.numSamples - 1]
                prev_ay = self.dataAy[self.numSamples - 1]
                prev_az = self.dataAz[self.numSamples - 1]
                
            ax = 0.6*prev_ax + 0.4*ax
            ay = 0.6*prev_ay + 0.4*ay
            az = 0.6*prev_az + 0.4*az
                
            self.dataAx.append(ax)
            self.dataAy.append(ay)
            self.dataAz.append(az)

            #=== ORIENTATION
            auxOrient = self.currOdom.pose.pose.orientation
            quat = (auxOrient.x, auxOrient.y, auxOrient.z, auxOrient.w)
            [roll, pitch, yaw]  = list(euler_from_quaternion(quat))

            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

            #Yaw between 0 and 360
            yaw_compass = yaw
            if (yaw < 0.0):
                yaw_compass = yaw + 360.0

            self.dataR.append(roll)
            self.dataP.append(pitch)
            self.dataH.append(yaw_compass)

            #=== ANGULAR VELOCITIES
            self.dataVr.append(math.degrees(self.currOdom.twist.twist.angular.x))
            self.dataVp.append(math.degrees(self.currOdom.twist.twist.angular.y))
            self.dataVh.append(math.degrees(self.currOdom.twist.twist.angular.z))
            
            #=== TIME MARKERS
            if (self.totalTime > self.markerLabelTime):
                    self.markerPointX.append( self.currOdom.pose.pose.position.y)
                    self.markerPointY.append( self.currOdom.pose.pose.position.x)
                    self.markerPointZ.append(-self.currOdom.pose.pose.position.z)
                    self.markerPointHeading.append(math.radians(yaw) + math.pi/2.0)
                    self.markerPointTime.append(self.markerLabelTime)
                    self.markerLabelStr.append("{0} s".format(self.markerLabelTime))
                    self.markerLabelTime = self.markerLabelTime + MARKER_TIME_INTERVAL

            #=== UPDATE FOR ITERATION
            self.prevOdom = self.currOdom
            self.numSamples = self.numSamples + 1
                      
        
    #---------------------------------------------------------------------------

    def getTime(self):
        #rospy.loginfo('data odom get time')
        self.totalTime


    def getRangeDataX(self):
        if (len(self.dataX) <= 0):
            return [0, 0]
        minVal = self.dataX[0]
        maxVal = self.dataX[0]
        for val in self.dataX:
            if (minVal > val):
                minVal = val
            if (maxVal < val):
                maxVal = val
        return [minVal, maxVal]


    def getRangeDataY(self):
        if (len(self.dataY) <= 0):
            return [0, 0]
        minVal = self.dataY[0]
        maxVal = self.dataY[0]
        for val in self.dataY:
            if (minVal > val):
                minVal = val
            if (maxVal < val):
                maxVal = val
        return [minVal, maxVal]


    def getRangeDataZ(self):
        if (len(self.dataZ) <= 0):
            return [0, 0]
        minVal = self.dataZ[0]
        maxVal = self.dataZ[0]
        for val in self.dataZ:
            if (minVal > val):
                minVal = val
            if (maxVal < val):
                maxVal = val
        return [minVal, maxVal]



