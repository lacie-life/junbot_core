#!/usr/bin/env python
import os
import signal
import sys
import math
import string
import time
import threading

import rospy
import roslib; roslib.load_manifest('jun_plotter')
import rosbag
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import *
from nav_msgs.msg import *

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
from pylab import *

from data_odometry import *

from docutils.nodes import header
from genpy import rostime


class PlotsGenerator(object):

    #---------------------------------------------------------------------------
    def __init__(self):
        rospy.loginfo("[PlotsGenerator] init()")
                        
        self.dataOdom = DataOdometry()
    
    #---------------------------------------------------------------------------
    def threePlotsOnePage(self, pdf, title, ylabel1, ylabel2, ylabel3, xlabel, xData, yData1, yData2, yData3):

        figure(figsize=(6,6))

        # 1st plot
        plt.subplot(3, 1, 1)
        plt.plot(xData, yData1)

        plt.title(title)
        plt.ylabel(ylabel1)
        plt.grid(True)

        setp(getp(gca(), 'xticklabels'), fontsize='x-small')
        setp(getp(gca(), 'yticklabels'), fontsize='x-small')


        # 2nd plot
        plt.subplot(3, 1, 2)
        plt.plot(xData, yData2)

        plt.ylabel(ylabel2)
        plt.grid(True)
        
        setp(getp(gca(), 'xticklabels'), fontsize='x-small')
        setp(getp(gca(), 'yticklabels'), fontsize='x-small')


        # 3rd plot
        plt.subplot(3, 1, 3)
        plt.plot(xData, yData3)

        plt.ylabel(ylabel3)
        plt.grid(True)

        setp(getp(gca(), 'xticklabels'), fontsize='x-small')
        setp(getp(gca(), 'yticklabels'), fontsize='x-small')


        plt.xlabel(xlabel)

        plt.tight_layout()
        pdf.savefig() # note the format='pdf' argument!
        close()


    #---------------------------------------------------------------------------    
    def altitudeTimePlot(self, pdf, dataTime, dataZ):
        rospy.loginfo('<<< Plot Altitude x Time >>>')

        figure(figsize=(6,6))

        fig = plt.figure()
        ax = fig.add_subplot(111)

        for i in range(len(dataZ)):
            dataZ[i] = dataZ[i]

        plot(dataTime, dataZ)
                
        xlabel('Time (s)')
        ylabel('Altitude sea level (m)')
        title('Altitude x Time')
        grid(True)

        setp(getp(gca(), 'xticklabels'), fontsize='x-small')
        setp(getp(gca(), 'yticklabels'), fontsize='x-small')

        plt.tight_layout()
        pdf.savefig() # note the format='pdf' argument!
        close()


    #---------------------------------------------------------------------------    
    def altitudeTravDistPlot(self, pdf, dataTravDist, dataZ):
        rospy.loginfo('<<< Plot Altitude x Trav Dist >>>')

        figure(figsize=(6,6))

        fig = plt.figure()
        ax = fig.add_subplot(111)

        plot(dataTravDist, dataZ)
        
        xlabel('Distance (m)')
        ylabel('Altitude sea level (m)')
        title('Altitude x Travelled Distance')
        grid(True)

        setp(getp(gca(), 'xticklabels'), fontsize='x-small')
        setp(getp(gca(), 'yticklabels'), fontsize='x-small')

        plt.tight_layout()
        pdf.savefig() # note the format='pdf' argument!
        close()
    

    #---------------------------------------------------------------------------    
    def getXYLimits(self):
        [ODminX, ODmaxX] = self.dataOdom.getRangeDataX()
        rospy.loginfo("ODOM MinX={0} MaxX={1}".format(ODminX, ODmaxX))
        [ODminY, ODmaxY] = self.dataOdom.getRangeDataY()
        rospy.loginfo("ODOM MinY={0} MaxY={1}".format(ODminY, ODmaxY))
        return [ODminX, ODmaxX, ODminY, ODmaxY]


    #---------------------------------------------------------------------------
    #PLOT XY TRAJECTORY
    def plotXYTrajectory(self, pdf):
        rospy.loginfo('<<< Plot XY Trajectory >>>')

        figure(figsize=(6,6))

        fig = plt.figure()
        ax = fig.add_subplot(111)

        #adjust limits to focus on trajectory / waypoints
        #find range in X and Y
        #equalize both ranges
        #calculate new center
        [minX, maxX, minY, maxY] = self.getXYLimits() #use odometry
        rangeX = maxX - minX + 500
        rangeY = maxY - minY + 500
        if (rangeX < rangeY):
            rangeX = rangeY
        if (rangeY < rangeX):
            rangeY = rangeX

        centerX = (maxX + minX) / 2.0
        centerY = (maxY + minY) / 2.0

        rospy.loginfo("Center: X={0} Y={1} / Range: X={2} Y={3}".format(centerX, centerY, rangeX, rangeY))

        # get coordinates to plot altitude map
        spacing  = 20.0
        if (rangeX > 10000.0):
            spacing = 100.0
        if (rangeX > 30000.0):
            spacing = 200.0
            
        #Add executed path
        rospy.loginfo("[PlotsGenerator] Adding executed path...")
        plot(self.dataOdom.dataX, self.dataOdom.dataY, '-', lw=1, color='blue')

        #Add time markers along trajectory - every 50 seconds
        for i in range(len(self.dataOdom.markerPointTime)):
            t = int(self.dataOdom.markerPointTime[i])
            if ((t % 50) == 0):
                rospy.loginfo("Add time marker: {0}".format(t))
                label = self.dataOdom.markerLabelStr[i]
                x = self.dataOdom.markerPointX[i]
                y = self.dataOdom.markerPointY[i]
                xtxt = 35 * math.cos(self.dataOdom.markerPointHeading[i])
                ytxt = 35 * math.sin(self.dataOdom.markerPointHeading[i])
                annotate(label, xy = (x, y), xytext = (xtxt, ytxt), size = 5,
                         textcoords = 'offset points', ha = 'right', va = 'bottom',
                         bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
                         arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'))
        
        #Add plot labels
        xlabel('EAST (m)')
        ylabel('NORTH (m)')
        title('Mission path')
        #grid(True)
        axes().set_aspect('equal', 'datalim')

        xticklabels = getp(gca(), 'xticklabels')
        yticklabels = getp(gca(), 'yticklabels')
        setp(yticklabels, fontsize='x-small')
        setp(xticklabels, fontsize='x-small')


        if (self.dataOdom.initTime != None):
            sttime = time.gmtime(int(self.dataOdom.initTime))
            timeStr = time.strftime("%a, %d %b %Y %H:%M:%S GMT", sttime)
            plt.figtext(0.70, 0.20, timeStr, bbox=dict(facecolor='yellow', alpha=0.75), size='xx-small')        

        #Add legends explaining plotted lines
        txtPosX = 0.12
        txtPosY = 0.90
        plt.figtext(txtPosX, txtPosY, "Executed trajectory", bbox=dict(facecolor='white', alpha=1.0), size='xx-small', color='blue')

        originCoord = 'Origin Coordinates: {0} E / {1} N'.format(self.dataOdom.iniPosY, self.dataOdom.iniPosX)
        plt.figtext(0.2, 0.01, originCoord, bbox=dict(facecolor='gray', alpha=0.5), size='xx-small')
        
        #set plot area limits
        plt.axis((centerX-rangeX/2, centerX+rangeX/2, centerY-rangeY/2, centerY+rangeY/2))

        plt.tight_layout()
        pdf.savefig() # note the format='pdf' argument!
        close()




    #---------------------------------------------------------------------------
    def finish(self, filename):
        rospy.loginfo("[PlotsGenerator] finish({0})".format(filename))
        
        #-----------------------------------------------------------------------
        pdf = PdfPages(filename)


        #-----------------------------------------------------------------------
        #PLOT MISSION (2D top down view)
        self.plotXYTrajectory(pdf)

        #-----------------------------------------------------------------------
        #PLOT ALTITUDE vs TIME
        self.altitudeTimePlot(pdf, self.dataOdom.dataTime, self.dataOdom.dataZ)

        #-----------------------------------------------------------------------
        #PLOT ALTITUDE vs TRAV DISTANCE
        self.altitudeTravDistPlot(pdf, self.dataOdom.dataTravDist, self.dataOdom.dataZ)

        #-----------------------------------------------------------------------
        #PLOT LINEAR VELOCITIES
        self.threePlotsOnePage(pdf, 'Velocity x Time', 'X axis (m/s)', 'Y axis (m/s)', 'Z axis (m/s)', 'time (s)', 
                               self.dataOdom.dataTime, self.dataOdom.dataVx, self.dataOdom.dataVy, self.dataOdom.dataVz)

        #-----------------------------------------------------------------------
        #PLOT ATTITUDE
        self.threePlotsOnePage(pdf, 'Attitude x Time', 'Roll (degrees)', 'Pitch (degrees)', 'Yaw (degrees)', 'time (s)', 
                               self.dataOdom.dataTime, self.dataOdom.dataR, self.dataOdom.dataP, self.dataOdom.dataH)

        #-----------------------------------------------------------------------
        #PLOT ANGULAR VELOCITIES
        self.threePlotsOnePage(pdf, 'Angular Speed x Time', 'Roll (degrees/s)', 'Pitch (degrees/s)', 'Yaw (degrees/s)', 'time (s)', 
                               self.dataOdom.dataTime, self.dataOdom.dataVr, self.dataOdom.dataVp, self.dataOdom.dataVh)

        #-----------------------------------------------------------------------
        d = pdf.infodict()
        d['Title'] = 'Odometry Plots PDF'
        d['Author'] = 'Silvio Maeta - smaeta@andrew.cmu.edu'
        d['Subject'] = 'Odometry data report plots - PDF formate file'
        d['Keywords'] = 'odometry plots'
        d['CreationDate'] = datetime.datetime.today()
        d['ModDate'] = datetime.datetime.today()

        # Remember to close the object - otherwise the file will not be usable
        pdf.close()
                

#-------------------------------------------------------------------------------

if __name__ == '__main__':

    rospy.init_node('generate_plots')

    plotsGen = None
    subPose = None
    
    #load parameters - output file name and odometry topic name
    reportOutputFilename = rospy.get_param('/generate_plots/output_filename')
    pose_topic = rospy.get_param('/generate_plots/pose_topic')

    try:
        #Get mission information and build plot file
        plotsGen = PlotsGenerator()

        #Associate subscribers with proper data handlers                
        subPose = rospy.Subscriber(pose_topic, Odometry, plotsGen.dataOdom.odometryHandler)
            
        #Collect information loop
        r = rospy.Rate(5.0)
        while (not rospy.is_shutdown()):
            r.sleep()
                                
        #unregister subscribers before writing to file
        subPose.unregister()
        
        #Finish processing and generate plot file
        plotsGen.finish(reportOutputFilename)        
        plotsGen = None

    except rospy.ROSInterruptException:
        if (subPose != None):
            subPose.unregister()
        if (plotsGen != None):
            plotsGen.finish(reportOutputFilename)
            plotsGen = None



