"""
Author: Daniel Kahrizi
Date: 29.06.2020
Description: Plotter for radar data.
This script holds a class that can be used to plot radar data.
Input: Inital axis limits and data input queues for radar 1 and radar 2.
"""
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle, Ellipse
import matplotlib
import mplcursors
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from numpy import unique
from numpy import where
import numpy as np
import math
import EllipseNumpy as EN

from queue import Queue
import queue
import numpy as np
import CircleBourke as CB
import RADAR_BINS
import cfarca
import logging
import sys
#import CAN_RADAR_INTERFACE_MAIN as PARSER
import RadarDataStructure as RSTRUCT
import datetime
from pathlib import Path
import time

""" Set the amount of range-bins and doppler-bins that are submitted each radar: Size of the Range-Doppler-Matrix """
size_range = 256
size_doppler = 64

class RDPlot:
    def __init__(self, RDMatrixQ_r1, RDMatrixQ_r2, imageDoExport, imageFoldername):
        global size_range, size_doppler
        #matplotlib.use('agg')
        self.size_range = size_range
        self.size_doppler = size_doppler
        self.q_rdMatrix_r1 = RDMatrixQ_r1
        self.q_rdMatrix_r2 = RDMatrixQ_r2
        self.rdMatrix_r1 = [ [0] * self.size_doppler for _ in range(self.size_range)]
        self.rdMatrix_r2 = [ [0] * self.size_doppler for _ in range(self.size_range)]

        self.imageDoExport = imageDoExport
        self.imageFoldername = imageFoldername
        
        if(imageDoExport == True):
            Path("./data/"+imageFoldername).mkdir(parents=True, exist_ok=True)
            #plt.ioff()
        self.pltcnt = 1

        self.r = np.zeros((self.size_range,self.size_doppler))
        self.g = np.zeros((self.size_range,self.size_doppler))
        self.b = np.zeros((self.size_range,self.size_doppler))
        self.rgb = np.dstack((self.r,self.g,self.b))
        self.flg_rd1Filled = 0
        self.flg_rd2Filled = 0

        self.ax1 = None
        self.ax2 = None
        self.ax3 = None
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(nrows=1, ncols=3,clear=True, sharex=True, sharey=True, figsize=(20, 5))
        self.im1 = self.ax1.imshow(self.rdMatrix_r1, interpolation='none', aspect='equal', vmin=0, vmax=32000)
        self.im2 = self.ax2.imshow(self.rdMatrix_r2, interpolation='none', aspect='equal', vmin=0, vmax=32000)
        self.im3 = self.ax3.imshow(self.rgb, interpolation = 'none', aspect = 'equal')#'auto'
        self.fig.suptitle('Range Doppler Matrix (live view)', fontsize=16)
        self.ax1.set_title('Radar 1')
        self.ax2.set_title('Radar 2')
        self.ax1.set_xlabel('Doppler Bins')
        self.ax1.set_ylabel('Range Bins')
        self.ax2.set_xlabel('Doppler Bins')
        self.ax2.set_ylabel('Range Bins')
        self.ax3.set_title('RGB image of both radars')
        self.ax3.set_xlabel('Doppler Bins')
        self.ax3.set_ylabel('Range Bins')
        self.newDataR1 = False
        self.newDataR2 = False

        self.cfarList_r1 = [] # cfar detection list with range index, doppler index and log 2 magnitude value
        self.cfarList_r2 = []
        self.scat_r1 = []
        self.scat_r2 = []
        
        self.ani = None
        #gui_env = [i for i in matplotlib.rcsetup.interactive_bk]
        #non_gui_backends = matplotlib.rcsetup.non_interactive_bk
        #print("GUIENV:", gui_env, " nonGUIENV:",non_gui_backends)
        




    def animateRDMatrix(self, i):
        startTime = time.time()
        self.readData()
        if(self.newDataR1):
            self.im1.set_array(self.rdMatrix_r1)

            # Remove scatter list from last round
            if(len(self.scat_r1)>0):
                for count in range(0, len(self.scat_r1)):
                    del_scat = self.scat_r1.pop()
                    del_scat.remove()
            # Add fresh scatter list with cfar detection list
            if(len(self.cfarList_r1) > 0):
                for count in range (0, len(self.cfarList_r1)):
                    current_item = self.cfarList_r1.pop()
                    self.scat_r1.append(self.ax1.scatter(current_item[1], current_item[0], c='r', marker = 'o'))

            self.flg_rd1Filled = 1
            self.newDataR1 = False

        if(self.newDataR2):
            self.im2.set_array(self.rdMatrix_r2)

            if(len(self.scat_r2)>0):
                for count in range(0, len(self.scat_r2)):
                    del_scat = self.scat_r2.pop()
                    del_scat.remove()

            if(len(self.cfarList_r2) > 0):
                for count in range (0, len(self.cfarList_r2)):
                    current_item = self.cfarList_r2.pop()
                    self.scat_r2.append(self.ax2.scatter(current_item[1], current_item[0], c='r', marker = '^'))

            self.flg_rd2Filled = 1
            self.newDataR2 = False

        # Create rgb image
        if self.flg_rd1Filled and self.flg_rd2Filled:

            # Convert data for image creation and normalize data to range 0..1 (r/g-channel)
            for rn in range(0,self.size_range):
                self.r[rn] = self.rdMatrix_r1[rn]/65535
                self.g[rn] = self.rdMatrix_r2[rn]/65535

            # Create rgb matrix and save image
            self.rgb = np.dstack((self.r,self.g,self.b))
            #self.im3 = self.ax3.imshow(self.rgb, interpolation= 'none', aspect = 'auto')
            self.im3.set_array(self.rgb)

            if(self.imageDoExport == True):
                timestamp = "./data/" + self.imageFoldername + "/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S_%f")
                timestamp += ".png"
                extent = self.im3.get_window_extent().transformed(self.fig.dpi_scale_trans.inverted())
                #fig.savefig('ax2_figure.png', bbox_inches=extent)
                #plt.savefig(timestamp, format = 'png', bbox_inches ='tight')
                plt.savefig(timestamp, format = 'png', bbox_inches = extent)
               
                #if(self.pltcnt % 10 == 0):
                 #   print("Using:",matplotlib.get_backend())
                    #plt.switch_backend('Agg')
                    #matplotlib.use('agg')
                    #plt.ion()
                    #self.pltcnt=0
                     
                #else:
                 #   print("Using:",matplotlib.get_backend())
                    #plt.switch_backend('GTK3Agg')
                    #matplotlib.use('GTK3Agg')
            #self.pltcnt +=1
                    
            
            self.flg_rd1Filled = 0
            self.flg_rd2Filled = 0
            self.r = np.zeros((self.size_range,self.size_doppler))
            self.g = np.zeros((self.size_range,self.size_doppler))
        #print("Plot needed: ",(time.time() - startTime)*1000, " ms")

        return [self.im1, self.im2, self.im3]

    def startAnimation(self):
        """
        Start the animation app. The frame update is set to 1000 ms
        """
        self.ani = anim.FuncAnimation(self.fig, self.animateRDMatrix, interval=500)  # ms
        #timestamp = "data/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S:%f")
        #myAni.save(timestamp + ".gif", writer='imagemagick', fps=1)
        #mplcursors.cursor(multiple=True)
        self.show()
    
    def terminate(self):
        plt.close('all')
        plt.close()
        if self.ani.event_source is not None:
            self.ani.event_source.stop()
            print("trying to stop the event source")
        plt.close(self.fig)
        plt.close()
        

    def show(self):
        plt.show()
        #print("noshow")


    def readData(self):
        """
            Read in the data from the queues, filled with the range-doppler-matrizes from both radars
        """
        try:
            temp = self.q_rdMatrix_r1.get_nowait()
            self.rdMatrix_r1 = np.fft.fftshift(temp,axes=(1,))
            self.cfarList_r1 = cfarca.CA_CFAR_CALL(self.rdMatrix_r1, self.size_range, self.size_doppler)
            with self.q_rdMatrix_r1.mutex:
                self.q_rdMatrix_r1.queue.clear()
            self.newDataR1 = True
            #print("reveived r1 data: ", self.ptsObj_r1.xs, self.ptsObj_r1.ys)
        except queue.Empty:
            #self.newDataR1 = False
            pass
        try:
            temp = self.q_rdMatrix_r2.get_nowait()
            self.rdMatrix_r2 = np.fft.fftshift(temp,axes=(1,))
            self.cfarList_r2 = cfarca.CA_CFAR_CALL(self.rdMatrix_r2, self.size_range, self.size_doppler)
            with self.q_rdMatrix_r2.mutex:
                self.q_rdMatrix_r2.queue.clear()
            self.newDataR2 = True
            #print("reveived r1 data: ", self.ptsObj_r1.xs, self.ptsObj_r1.ys)
        except queue.Empty:
            #self.newDataR2 = False
            pass

class RadarPlot:
    def __init__(self, xmin, xmax, ymin, ymax, detPointsQ_r1, detClustersQ_r1, detPointsQ_r2, detClustersQ_r2, loggingFilename=None, loggingActive=False, refX=0, refY=0, refRange=0.3, plotPeak=True, plotCOG=True, plotComb=True, plotBins=True, epsCOG=0.3, epsPeak=0.3, epsCombined=0.4):
        # plot settings
        self.plotPeak = plotPeak
        self.plotCOG = plotCOG
        self.plotComb = plotComb
        self.plotBins = plotBins
        #self.refreshInterval = refreshInterval #refresh interval is obtained via startAnimation(..)
        #print("plotBins"+str(self.plotBins))
        # DBSCAN Clustering Settings
        self.epsCOG = epsCOG
        self.epsPeak = epsPeak
        self.epsCombined = epsCombined
        # axis values
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        # input queues for detected points and clusters
        self.detPointsQ_r1 = detPointsQ_r1
        self.detClustersQ_r1 = detClustersQ_r1
        self.detPointsQ_r2 = detPointsQ_r2
        self.detClustersQ_r2 = detClustersQ_r2
        # last received pooints and clusters
        self.lastRecPtsObjR1 = RSTRUCT.DetectedPoints()
        self.lastRecClstrObjR1 = RSTRUCT.DetectedClusters()
        self.lastRecPtsObjR2 = RSTRUCT.DetectedPoints()
        self.lastRecClstrObjR2 = RSTRUCT.DetectedClusters()
        # current points and clusters
        self.ptsObj_r1 = None
        self.clustersObj_r1 = None
        self.ptsObj_r2 = None
        self.clustersObj_r2 = None

        # Create figure and plot with three subplots
        self.ax1 = None
        self.ax2 = None
        self.ax3 = None
        self.ax4 = None
        self.generateSubplots()
        #TODO
        """
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(nrows=1, ncols=4,
                                                                          clear=True, sharex=True, sharey=True, figsize=(20, 5))
       
        self.fig, (self.ax1) = plt.subplots(nrows=1, ncols=1,
                                                                          clear=True, sharex=True, sharey=True, figsize=(20, 5))
        self.ax2 = None
        self.ax3 = None
        self.ax4 = None
         """

        # configure axes and connect interaction callbacks for resizing of the axis
        # TODO
        if(self.ax2 is not None):
            self.ax2.yaxis.set_tick_params(labelleft=True)
            self.ax2.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax2.callbacks.connect('ylim_changed', self.on_ylims_change)
        if(self.ax3 is not None):
            self.ax3.yaxis.set_tick_params(labelleft=True)
            self.ax3.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax3.callbacks.connect('ylim_changed', self.on_ylims_change)
        if(self.ax4 is not None):
            self.ax4.yaxis.set_tick_params(labelleft=True)
            self.ax4.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax4.callbacks.connect('ylim_changed', self.on_ylims_change)
        self.ax1.callbacks.connect('xlim_changed', self.on_xlims_change)
        self.ax1.callbacks.connect('ylim_changed', self.on_ylims_change)

        self.cbar = None
        self.redraw = True

        # images that can be recoloured by colorbar
        self.im1 = None
        self.im2 = None
        self.im3 = None
        self.im4 = None

        self.xP1 = None
        self.yP1 = None
        self.xP2 = None
        self.yP2 = None
        self.xC = None
        self.yC = None
        self.timeCounter = 0
        # settings for data export
        self.referencePointX = refX  # 0.5493*3#0.2679*3
        self.referencePointY = refY
        self.maxDistFromReference = refRange

        self.filename = f"./data/{self.referencePointX}_{self.referencePointY}_measurement_M182.csv"
        if loggingFilename is not None:
            self.filename = "./data/" + str(loggingFilename) + ".csv"
        self.loggingActive = loggingActive
        if(loggingActive):
            with open(self.filename, 'w+') as myFile:
                myFile.write(
                    f"Time[s],xP1[m],yP1[m],xP2[m],yP2[m],xC[m],yC[m]\n")

        #self.show()

    def generateSubplots(self):
        if(self.plotCOG and self.plotPeak and self.plotComb and self.plotBins):
            self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(nrows=1, ncols=4,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG and self.plotPeak and self.plotComb and self.plotBins==False):
            self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(nrows=1, ncols=3,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG and self.plotPeak and self.plotComb==False and self.plotBins==False):
            self.fig, (self.ax1, self.ax2) = plt.subplots(nrows=1, ncols=2,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG and self.plotPeak==False and self.plotComb and self.plotBins):
            self.fig, (self.ax1, self.ax3, self.ax4) = plt.subplots(nrows=1, ncols=3,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG and self.plotPeak==False and self.plotComb and self.plotBins==False):
            self.fig, (self.ax1, self.ax3) = plt.subplots(nrows=1, ncols=2,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG and self.plotPeak==False and self.plotComb==False and self.plotBins==False):
            self.fig, (self.ax1) = plt.subplots(nrows=1, ncols=1,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG==False and self.plotPeak and self.plotComb and self.plotBins):
            self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(nrows=1, ncols=4,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG==False and self.plotPeak and self.plotComb and self.plotBins==False):
            self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(nrows=1, ncols=3,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG==False and self.plotPeak and self.plotComb==False and self.plotBins==False):
            self.fig, (self.ax1, self.ax2) = plt.subplots(nrows=1, ncols=2,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG==False and self.plotPeak==False and self.plotComb and self.plotBins):
            self.fig, (self.ax1, self.ax3, self.ax4) = plt.subplots(nrows=1, ncols=3,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG==False and self.plotPeak==False and self.plotComb and self.plotBins==False):
            self.fig, (self.ax1, self.ax3) = plt.subplots(nrows=1, ncols=2,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))
        elif(self.plotCOG==False and self.plotPeak==False and self.plotComb==False and self.plotBins==False):
            self.fig, (self.ax1) = plt.subplots(nrows=1, ncols=1,
                                                                            clear=True, sharex=True, sharey=True, figsize=(20, 5))

    def ptInRefDist(self, x, y):
        """
            Check, if the point is in a plausible distance to the reference points
        """
        #TODO
        print(f'x:{x}; referencePointX:{self.referencePointX}')
        if(x is None or y is None):
            return False
        if(math.sqrt((x-self.referencePointX)**2 + (y-self.referencePointY)**2) <= self.maxDistFromReference):
            return True
        else:
            return False

    def plotPoint(self, x, y, text):
        """
            TODO
        """
        if(x is not None and y is not None):
            if(math.sqrt((x-self.referencePointX)**2 + (y-self.referencePointY)**2) <= self.maxDistFromReference):
                if(text == "Center"):
                    self.xC = x
                    self.yC = y
                    # print(x)
                    # print(y)

    def plotPoints(self, xs, ys, text):
        """
            Plot the points in the animation
        """
        index = 0
        for x in xs:
            self.plotPoint(x, ys[index], text)
            index += 1

    def startAnimation(self, intervalMS=500):
        """
            Start the animation app. The frame update is set to 300 ms
        """
        myAni = anim.FuncAnimation(
            self.fig, self.animate_noJumping, interval=intervalMS)  # ms
        mplcursors.cursor(multiple=True)
        self.show()

    def checkNone(self, val):
        """
            Check if the value of the input parameter is None
        """
        if(val == None):
            return "#NUM!"
        else:
            return val

    def writeToFile(self):
        """
            Write the current point into a file
        """
        wxP1 = self.checkNone(self.xP1)
        wyP1 = self.checkNone(self.yP1)
        wxP2 = self.checkNone(self.xP2)
        wyP2 = self.checkNone(self.yP2)
        wxC = self.checkNone(self.xC)
        wyC = self.checkNone(self.yC)
        #self.timeCounter = 0
        with open(self.filename, 'a') as myFile:
            myFile.write(
                f"{round(self.timeCounter,3)},{wxP1},{wyP1},{wxP2},{wyP2},{wxC},{wyC}\n")
    def animate(self, i):
        # Read newest data from queue and delete queue data that is older than the one just read.
        # sets weather redraw is neccessary or not:
        self.readData()
        if(self.redraw):
            # clear the axis, set axis limits (user zoomed in/out) and reconnect callbacks for next zoom event
            self.clearAxesAndCbar()
            imagesToColorList = []
            # reconnect scroll event for zooming via mousewheel
            cid = self.fig.canvas.mpl_connect('scroll_event', self.onscroll)

            # draw all detected points and radar-detected clusters from radar 1 and radar 2
            im1, im2 = self.drawAllPointsAndClusters()
            imagesToColorList.extend([im1, im2])


            # Cluster combined data of r1 and r2 and plot center of gravity for each cluster
            #print(f'plotCOG:{self.plotCOG}')
            if(self.plotCOG):
                self.clusterRawPointsAndDrawWeightedCenterOfGravity(
                    epsilon=self.epsCOG)

            # DBSCAN Algorith with scikit learn
            # Find points that are max 0.3m distant to each other and group them into new clusters
            # Find each point in a detected cluster that has the biggest peak value
            # plot this point to ax2 and store those points in ptsMaxPeakObj_rX
            if(self.plotPeak or self.plotComb or (self.plotComb and self.plotBins)):
                ptsMaxPeakObj_r1 = self.calcClusterWithMaxPeakValue(epsilon=self.epsPeak, ptsObj_rX=self.ptsObj_r1)
                ptsMaxPeakObj_r2 = self.calcClusterWithMaxPeakValue(epsilon=self.epsPeak, ptsObj_rX=self.ptsObj_r2)
            if(self.plotPeak):
                im3 = self.plotClusterPointWithMaxPeakValue(
                    ptsMaxPeakObj_rX=ptsMaxPeakObj_r1, markerRadarPoint='o')
                im4 = self.plotClusterPointWithMaxPeakValue(
                    ptsMaxPeakObj_rX=ptsMaxPeakObj_r2, markerRadarPoint='v')
                imagesToColorList.extend([im3, im4])
            # USE DBSCAN TO FIND AND PLOT OBJECTS POINTS THAT ARE DETECTED BY BOTH RADARS

            if(self.plotComb or self.plotBins):
                # 1) combine max peak cluster points into one object
                ptsMaxPeakObj_r12 = RSTRUCT.DetectedPoints()
                ptsMaxPeakObj_r12.extendFromOtherDetectedObject(ptsMaxPeakObj_r1)
                ptsMaxPeakObj_r12.extendFromOtherDetectedObject(ptsMaxPeakObj_r2)
            # 2)cluster -> 20cm / 30cm?
            # 3) check weather each cluster holds exactly one point of radar 1 and one point of radar 2
            # 3.1) fill result object with those points: ptsPairedMaxPeakObj
            # 4) print circles and calculate intersection for points that fulfill 3)
            # those steps are implemented in the following function
            print(f'plotComb:{self.plotComb}')
            if(self.plotComb):
                im5, im6, ptsPairedMaxPeakObj_r12 = self.plotR1R2DetectionOfRLObject(
                    self.epsCombined, ptsMaxPeakObj_r12, ptsMaxPeakObj_r1, ptsMaxPeakObj_r2)
                imagesToColorList.extend([im5, im6])
            print(f'plotBins:{self.plotBins}')
            if(self.plotBins):
                # for each pair of r1/r2 detected point, draw the respective radar bin
                self.plotRadarBins(ptsPairedMaxPeakObj_r12)

            # add colorbar
            self.plotPeakValueColorbar(
                im1, self.ax1, imagesToColorList, lowerLimit=0, upperLimit=6000)
            # configure axes
            self.configureAxes()
            # set space between subplots
            # commented due incompability on ubuntu notebook
            self.fig.tight_layout(pad=3.0)

        if (self.loggingActive):
            self.writeToFile()  # finally write measurement data to file

    def animate_noJumping(self, i):
        # Read newest data from queue and delete queue data that is older than the one just read.
        # sets weather redraw is neccessary or not:
        self.readData()
        if(self.redraw):
            # clear the axis, set axis limits (user zoomed in/out) and reconnect callbacks for next zoom event
            """
            Clear the plots and the color bar
            """
            self.ax1.cla()
            self.ax1.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax1.callbacks.connect('ylim_changed', self.on_ylims_change)
            if self.ax2 is not None:
                self.ax2.cla()
                self.ax2.callbacks.connect('xlim_changed', self.on_xlims_change)
                self.ax2.callbacks.connect('ylim_changed', self.on_ylims_change)
            if self.ax3 is not None:
                self.ax3.cla()
                self.ax3.callbacks.connect('xlim_changed', self.on_xlims_change)
                self.ax3.callbacks.connect('ylim_changed', self.on_ylims_change)
            if self.ax4 is not None:
                self.ax4.cla()
                self.ax4.callbacks.connect('xlim_changed', self.on_xlims_change)
                self.ax4.callbacks.connect('ylim_changed', self.on_ylims_change)
            self.ax1.set_xlim(self.xmin, self.xmax)
            self.ax1.set_ylim(self.ymin, self.ymax)
            if self.cbar is not None:
                # colorbar cannot be removed via cla. therefor remove it manually and rebuild it later
                self.cbar.remove()
                
            imagesToColorList = []
            # reconnect scroll event for zooming via mousewheel
            cid = self.fig.canvas.mpl_connect('scroll_event', self.onscroll)

            # draw all detected points and radar-detected clusters from radar 1 and radar 2
            im1, im2 = self.drawAllPointsAndClusters()
            imagesToColorList.extend([im1, im2])


            # Cluster combined data of r1 and r2 and plot center of gravity for each cluster
            #print(f'plotCOG:{self.plotCOG}')
            if(self.plotCOG):
                self.clusterRawPointsAndDrawWeightedCenterOfGravity(
                    epsilon=self.epsCOG)

            # DBSCAN Algorith with scikit learn
            # Find points that are max 0.3m distant to each other and group them into new clusters
            # Find each point in a detected cluster that has the biggest peak value
            # plot this point to ax2 and store those points in ptsMaxPeakObj_rX
            ptsMaxPeakObj_r1 = RSTRUCT.DetectedPoints()
            ptsMaxPeakObj_r2 = RSTRUCT.DetectedPoints()
            
            if(self.plotPeak or self.plotComb or (self.plotComb and self.plotBins)):
                tmp1 = self.calcClusterWithMaxPeakValue(epsilon=self.epsPeak, ptsObj_rX=self.ptsObj_r1)
                tmp2 = self.calcClusterWithMaxPeakValue(epsilon=self.epsPeak, ptsObj_rX=self.ptsObj_r2)
                ptsMaxPeakObj_r1.extendFromOtherDetectedObject(tmp1)
                ptsMaxPeakObj_r2.extendFromOtherDetectedObject(tmp2)
            if(self.plotPeak):
                im3 = self.plotClusterPointWithMaxPeakValue(
                    ptsMaxPeakObj_rX=ptsMaxPeakObj_r1, markerRadarPoint='o')
                im4 = self.plotClusterPointWithMaxPeakValue(
                    ptsMaxPeakObj_rX=ptsMaxPeakObj_r2, markerRadarPoint='v')
                imagesToColorList.extend([im3, im4])
            # USE DBSCAN TO FIND AND PLOT OBJECTS POINTS THAT ARE DETECTED BY BOTH RADARS

            if(self.plotComb or self.plotBins):
                # 1) combine max peak cluster points into one object
                ptsMaxPeakObj_r12 = RSTRUCT.DetectedPoints()
                ptsMaxPeakObj_r12.extendFromOtherDetectedObject(ptsMaxPeakObj_r1)
                ptsMaxPeakObj_r12.extendFromOtherDetectedObject(ptsMaxPeakObj_r2)
            # 2)cluster -> 20cm / 30cm?
            # 3) check weather each cluster holds exactly one point of radar 1 and one point of radar 2
            # 3.1) fill result object with those points: ptsPairedMaxPeakObj
            # 4) print circles and calculate intersection for points that fulfill 3)
            # those steps are implemented in the following function
            #print(f'plotComb:{self.plotComb}')
            if(self.plotComb):
                im5, im6, ptsPairedMaxPeakObj_r12 = self.plotR1R2DetectionOfRLObject(
                    self.epsCombined, ptsMaxPeakObj_r12, ptsMaxPeakObj_r1, ptsMaxPeakObj_r2)
                imagesToColorList.extend([im5, im6])
            #print(f'plotBins:{self.plotBins}')
            if(self.plotBins):
                # for each pair of r1/r2 detected point, draw the respective radar bin
                self.plotRadarBins(ptsPairedMaxPeakObj_r12)

            # add colorbar
            self.plotPeakValueColorbar(
                im1, self.ax1, imagesToColorList, lowerLimit=0, upperLimit=6000)
            # configure axes
            self.configureAxes()
            # set space between subplots
            # commented due incompability on ubuntu notebook
            self.fig.tight_layout(pad=3.0)

        if (self.loggingActive):
            self.writeToFile()  # finally write measurement data to file

    def animateXYPeak(self, i):
        # Read newest data from queue and delete queue data that is older than the one just read.
        # sets weather redraw is neccessary or not:

        
        self.readData()
        if(self.redraw):
            self.ax1.cla()
            #self.ax2.cla()
            #self.ax3.cla()
            #self.ax4.cla()
            self.ax1.set_xlim(self.xmin, self.xmax)
            self.ax1.set_ylim(self.ymin, self.ymax)

            self.ax1.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax1.callbacks.connect('ylim_changed', self.on_ylims_change)
            #self.ax2.callbacks.connect('xlim_changed', self.on_xlims_change)
            #self.ax2.callbacks.connect('ylim_changed', self.on_ylims_change)
            #self.ax3.callbacks.connect('xlim_changed', self.on_xlims_change)
            #self.ax3.callbacks.connect('ylim_changed', self.on_ylims_change)
            #self.ax4.callbacks.connect('xlim_changed', self.on_xlims_change)
            #self.ax4.callbacks.connect('ylim_changed', self.on_ylims_change)

            if self.cbar is not None:
                # colorbar cannot be removed via cla. therefor remove it manually and rebuild it later
                self.cbar.remove()
  
            imagesToColorList = []
            im1, im2 = self.drawAllPointsAndClusters()
            imagesToColorList.extend([im1, im2])
            self.clusterRawPointsAndDrawWeightedCenterOfGravity(epsilon=self.epsCOG)
            #self.ax1.scatter([0], [0])
            #self.ax2.scatter([0], [0])
            #self.ax3.scatter([0], [0])
            #self.ax4.scatter([0], [0])
            self.plotPeakValueColorbar(im1, self.ax1, imagesToColorList, lowerLimit=0, upperLimit=6000)
            # configure axes
            self.ax1.set_xlabel('x-coordinate [m]')
            self.ax1.set_ylabel('y-coordinate [m]')
            
            #self.ax2.set_xlabel('x-coordinate [m]')
            #self.ax2.set_ylabel('y-coordinate [m]')
            """
            self.ax3.set_xlabel('x-coordinate [m]')
            self.ax3.set_ylabel('y-coordinate [m]')
            self.ax4.set_xlabel('x-coordinate [m]')
            self.ax4.set_ylabel('y-coordinate [m]')
            """
            # connect callback for zoom
            self.ax1.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax1.callbacks.connect('ylim_changed', self.on_ylims_change)
            #self.ax2.callbacks.connect('xlim_changed', self.on_xlims_change)
            #self.ax2.callbacks.connect('ylim_changed', self.on_ylims_change)
            #self.ax3.callbacks.connect('xlim_changed', self.on_xlims_change)
            #self.ax3.callbacks.connect('ylim_changed', self.on_ylims_change)
            #self.ax4.callbacks.connect('xlim_changed', self.on_xlims_change)
            #self.ax4.callbacks.connect('ylim_changed', self.on_ylims_change)
            #mplcursors.cursor(multiple=True)
            # Set title of the figures
            self.fig.suptitle('Detected Objects (live view)', fontsize=16)
            self.ax1.set_title('Radar data: Detected Points, Peak Vals & Clusters')
            #self.ax2.set_title('Max PeakVal Point of each Cluster')
            """
            
            self.ax3.set_title('Intersection of two range circles')
            self.ax4.set_title('Intersection of Radar-Range/Angle-Bins')
            """

            # grid on
            self.ax1.grid()
            #self.ax2.grid()
            #self.ax3.grid()
            #self.ax4.grid()
            #self.fig.tight_layout(pad=3.0)
        """
        self.timeCounter += 0.3  # add 300 ms to internal counter

        if(self.redraw):
            # clear the axis, set axis limits (user zoomed in/out) and reconnect callbacks for next zoom event
            self.clearAxesAndCbar()
            imagesToColorList = []
            # reconnect scroll event for zooming via mousewheel
            cid = self.fig.canvas.mpl_connect('scroll_event', self.onscroll)

            # draw all detected points and radar-detected clusters from radar 1 and radar 2
            im1, im2 = self.drawAllPointsAndClusters()
            imagesToColorList.extend([im1, im2])


            # Cluster combined data of r1 and r2 and plot center of gravity for each cluster
            print(f'plotCOG:{self.plotCOG}')
            if(self.plotCOG):
                self.clusterRawPointsAndDrawWeightedCenterOfGravity(
                    epsilon=self.epsCOG)

            # DBSCAN Algorith with scikit learn
            # Find points that are max 0.3m distant to each other and group them into new clusters
            # Find each point in a detected cluster that has the biggest peak value
            # plot this point to ax2 and store those points in ptsMaxPeakObj_rX
            ptsMaxPeakObj_r1, im3 = self.plotClusterPointWithMaxPeakValue(
                epsilon=self.epsPeak, ptsObj_rX=self.ptsObj_r1, markerRadarPoint='o')
            ptsMaxPeakObj_r2, im4 = self.plotClusterPointWithMaxPeakValue(
                epsilon=self.epsPeak, ptsObj_rX=self.ptsObj_r2, markerRadarPoint='v')
            imagesToColorList.extend([im3, im4])
            # USE DBSCAN TO FIND AND PLOT OBJECTS POINTS THAT ARE DETECTED BY BOTH RADARS
            # 1) combine max peak cluster points into one object
            ptsMaxPeakObj_r12 = RSTRUCT.DetectedPoints()
            ptsMaxPeakObj_r12.extendFromOtherDetectedObject(ptsMaxPeakObj_r1)
            ptsMaxPeakObj_r12.extendFromOtherDetectedObject(ptsMaxPeakObj_r2)
            # 2)cluster -> 20cm / 30cm?
            # 3) check weather each cluster holds exactly one point of radar 1 and one point of radar 2
            # 3.1) fill result object with those points: ptsPairedMaxPeakObj
            # 4) print circles and calculate intersection for points that fulfill 3)
            # those steps are implemented in the following function
            print(f'plotComb:{self.plotComb}')
            if(self.plotComb):
                im5, im6, ptsPairedMaxPeakObj_r12 = self.plotR1R2DetectionOfRLObject(
                    self.epsCombined, ptsMaxPeakObj_r12, ptsMaxPeakObj_r1, ptsMaxPeakObj_r2)
                imagesToColorList.extend([im5, im6])
            print(f'plotBins:{self.plotBins}')
            if(self.plotBins):
                # for each pair of r1/r2 detected point, draw the respective radar bin
                self.plotRadarBins(ptsPairedMaxPeakObj_r12)
            
            # add colorbar
            self.plotPeakValueColorbar(
                im1, self.ax1, imagesToColorList, lowerLimit=0, upperLimit=6000)
            # configure axes
            self.configureAxes()
            # set space between subplots
            # commented due incompability on ubuntu notebook
            self.fig.tight_layout(pad=3.0)
            """
        if (self.loggingActive):
            self.writeToFile()  # finally write measurement data to file


    def show(self):
        plt.show()
    # this is the callback function that handles resizing of the x-axis

    def on_xlims_change(self, evt_axlim_changed):
        self.xmin, self.xmax = evt_axlim_changed.get_xlim()

    # this is the callback function that handles resizing of the y-axis
    def on_ylims_change(self, evt_axlim_changed):
        self.ymin, self.ymax = evt_axlim_changed.get_ylim()

    # this is the callback funtion that handles mouse wheel scrolling events on the plot area
    # mousewheel is herby registered to zoom in / zoom out
    def onscroll(self, event):
        # print(f"step={event.step} x={event.x} y={event.y}" )
        if(event.step > 0):
            step = int(event.step)*0.9
        else:
            step = int(event.step)*-1.1

        self.xmin = step * self.xmin
        self.xmax = step * self.xmax
        self.ymin = step * self.ymin
        self.ymax = step * self.ymax

    # unused function sceletton that can be used to react to button clicks
    # command to register this callback function:
    # cid = fig.canvas.mpl_connect('button_press_event', onclick)

    def onclick(event):
        logging.info('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
                     ('double' if event.dblclick else 'single', event.button,
                      event.x, event.y, event.xdata, event.ydata))
    # unused function sceletton that can be used to react to pick events
    #register: cid = fig.canvas.mpl_connect('pick_event', on_pick)

    def on_pick(event):
        """
            Action on pick event
        """
        line = event.artist
        xdata, ydata = line.get_data()
        ind = event.ind
        logging.info('on pick line:', np.array([xdata[ind], ydata[ind]]).T)

    def readData(self):
        """
            Read in the data from the queues, filled with information about the detected points and clusters from both radars
        """
        try:
            self.ptsObj_r1 = self.detPointsQ_r1.get_nowait()
            self.lastRecPtsObjR1 = self.ptsObj_r1
            with self.detPointsQ_r1.mutex:
                self.detPointsQ_r1.queue.clear()
            #print("reveived r1 data: ", self.ptsObj_r1.xs, self.ptsObj_r1.ys)
            self.redraw = True
        except queue.Empty:
            self.ptsObj_r1 = RSTRUCT.DetectedPoints()
            pass

        try:
            self.clustersObj_r1 = self.detClustersQ_r1.get_nowait()
            self.lastRecClstrObjR1 = self.clustersObj_r1
            with self.detClustersQ_r1.mutex:
                self.detClustersQ_r1.queue.clear()
            self.redraw = True

        except queue.Empty:
            self.clustersObj_r1 = RSTRUCT.DetectedClusters()
            pass

        try:
            self.ptsObj_r2 = self.detPointsQ_r2.get_nowait()
            index = 0
            for x in self.ptsObj_r2.xs:
                # add distance between radar module 1 and 2 to points from radar module 2
                self.ptsObj_r2.xs[index] = x+0.15
                index += 1
            self.lastRecPtsObjR2 = self.ptsObj_r2
            with self.detPointsQ_r2.mutex:
                self.detPointsQ_r2.queue.clear()
            self.redraw = True
            #print("received r2 data: ", self.ptsObj_r2.xs,self.ptsObj_r2.ys)
        except queue.Empty:
            self.ptsObj_r2 = RSTRUCT.DetectedPoints()
            pass

        try:
            self.clustersObj_r2 = self.detClustersQ_r2.get_nowait()
            self.lastRecClstrObjR2 = self.clustersObj_r2
            with self.detClustersQ_r2.mutex:
                self.detClustersQ_r2.queue.clear()
            self.redraw = True
        except queue.Empty:
            self.clustersObj_r2 = RSTRUCT.DetectedClusters()
            pass

        if(self.redraw):
            if(self.ptsObj_r1.isEmpty() and self.clustersObj_r1.isEmpty() and self.ptsObj_r2.isEmpty() and self.clustersObj_r2.isEmpty()):
                # all queues are empty: set current values to last received values. Next time no replotting will be needed. Except new data is available.
                self.ptsObj_r1 = self.lastRecPtsObjR1
                self.clustersObj_r1 = self.lastRecClstrObjR1
                self.ptsObj_r2 = self.lastRecPtsObjR2
                self.clustersObj_r2 = self.lastRecClstrObjR2
                self.redraw = False
            else:
                if(self.ptsObj_r1.isEmpty()):
                    self.ptsObj_r1 = self.lastRecPtsObjR1
                    self.redraw = True
                if(self.clustersObj_r1.isEmpty()):
                    self.clustersObj_r1 = self.lastRecClstrObjR1
                    self.redraw = True
                if(self.ptsObj_r2.isEmpty()):
                    self.ptsObj_r2 = self.lastRecPtsObjR2
                    self.redraw = True
                if (self.clustersObj_r2.isEmpty()):
                    self.clustersObj_r2 = self.lastRecClstrObjR2
                    self.redraw = True

            self.ptsObj_r1.setAllOrigin(0)
            self.ptsObj_r2.setAllOrigin(1)

    def clusterRawPointsAndDrawWeightedCenterOfGravity(self, epsilon=0.3, threshold=4000):
        """
            Find the clusters with the dbscan algorithm, calculate the center of gravity and plot it
        """
        ptsObj_r12 = RSTRUCT.DetectedPoints()
        ptsObj_r12.extendFromOtherDetectedObject(self.ptsObj_r1)
        ptsObj_r12.extendFromOtherDetectedObject(self.ptsObj_r2)

        # Cluster raw point data from both radars with DBSCAN
        if(ptsObj_r12.isEmpty() == False):
            data = ptsObj_r12.getXYArray()
            db = DBSCAN(eps=epsilon, min_samples=2).fit_predict(data)
            clusters = unique(db)

            xsGravity = []
            ysGravity = []
            # DBSCAN Clustering
            self.xC = None
            self.yC = None
            for cluster in clusters:
                row_ix = where(db == cluster)
                # row_ix is array of cluster indices
                # [ [0,1], [2,3] ] meaning points at indices 0 and 1 (of data array)
                # are a cluster 1
                # and points at indices 2 and 3 are cluster 2
                for i in row_ix:  # iterate over the cluster 2d array
                    # print("i:",i)
                    clusterPoints = RSTRUCT.DetectedPoints()
                    for j in i:
                        # add cluster point values to cluster Points object
                        if(ptsObj_r12.peakVals[j] > threshold):
                            clusterPoints.appendX(ptsObj_r12.xs[j])
                            clusterPoints.appendY(ptsObj_r12.ys[j])
                            clusterPoints.appendPeakVal(ptsObj_r12.peakVals[j])
                            clusterPoints.appendOrigins(ptsObj_r12.origins[j])
                    # calculate weighted center of gravity for the whole closter

                    sumClusterOrigins = sum(clusterPoints.origins)
                    # if cluster contains elements from both radars and at least 2 elements in cluster
                    if(sumClusterOrigins < len(clusterPoints.origins) and sumClusterOrigins > 0 and len(clusterPoints.origins) > 1):
                        xCG, yCG = clusterPoints.getPeakValueWeightedCenterOfGravity()
                        #print(clusterPoints.origins, " ", len(clusterPoints.origins), " Sum:", sum(clusterPoints.origins))
                        # save center of gravity to list
                        #print(f"found centers xy:{xCG}/{yCG}")
                        if (self.ptInRefDist(xCG, yCG)):
                            self.xC = xCG
                            self.yC = yCG
                        xsGravity.append(xCG)
                        ysGravity.append(yCG)
                        self.plotPoint(xCG, yCG, "Center")

                # plot center of gravity
                self.ax1.scatter(xsGravity, ysGravity,
                                 color='blue',  marker='p')

    # this functions draws all detected points and clusters from radar 1 and radar 2 on ax1

    def drawAllPointsAndClusters(self):
        """
            Plot all points and clusters (Figure 1 in the documentation)
        """
        # draw cluster radar 1
        rectCount = 0
        while(rectCount < len(self.clustersObj_r1.xsCenter)):
            self.ax1.add_patch(Rectangle((self.clustersObj_r1.xsCenter[rectCount]-self.clustersObj_r1.xsSize[rectCount]/2, self.clustersObj_r1.ysCenter[rectCount]-self.clustersObj_r1.ysSize[rectCount]/2),
                                         self.clustersObj_r1.xsSize[rectCount], self.clustersObj_r1.ysSize[rectCount], fill=False, ec="green", ls="--"))
            rectCount += 1
        # draw cluster radar 2
        rectCount = 0
        while(rectCount < len(self.clustersObj_r2.xsCenter)):
            self.ax1.add_patch(Rectangle((self.clustersObj_r2.xsCenter[rectCount]-self.clustersObj_r2.xsSize[rectCount]/2, self.clustersObj_r2.ysCenter[rectCount]-self.clustersObj_r2.ysSize[rectCount]/2),
                                         self.clustersObj_r2.xsSize[rectCount], self.clustersObj_r2.ysSize[rectCount], fill=False, ec="red", ls="--"))
            rectCount += 1

        # draw x,y of detected objectes
        im1 = self.ax1.scatter(self.ptsObj_r1.xs, self.ptsObj_r1.ys, c=self.ptsObj_r1.peakVals,
                               cmap=plt.cm.get_cmap('viridis_r'), marker='o')
        im2 = self.ax1.scatter(self.ptsObj_r2.xs, self.ptsObj_r2.ys, c=self.ptsObj_r2.peakVals,
                               cmap=plt.cm.get_cmap('viridis_r'), marker='v')

        return im1, im2

    def calcClusterWithMaxPeakValue(self,epsilon, ptsObj_rX):
        ptsMaxPeakObj_rX = RSTRUCT.DetectedPoints()
        if(ptsObj_rX.isEmpty() == False):
            data = ptsObj_rX.getXYArray()
            db = DBSCAN(eps=epsilon, min_samples=1).fit_predict(data)
            clusters = unique(db)
            #print(f'clusters:{clusters}')
            for cluster in clusters:
                row_ix = where(db == cluster)
                clusterPts = []

                for i in row_ix:
                    maxPeakVal = 0
                    (x, y, v, peakVal) = ptsObj_rX.getXYMaxPeakVal(i)
                    #print(f'append: x:{x} y:{y} v:{v} peakVal:{v}')
                    ptsMaxPeakObj_rX.appendX(x)
                    ptsMaxPeakObj_rX.appendY(y)
                    ptsMaxPeakObj_rX.appendV(v)
                    ptsMaxPeakObj_rX.appendPeakVal(peakVal)
                    ptsMaxPeakObj_rX.appendOrigins(0)
        return ptsMaxPeakObj_rX
    # DBSCAN Algorith with scikit learn
    # Find points that are max 0.3m distant to each other and group them into new clusters
    # Find each point in a detected cluster that has the biggest peak value
    # plot this point to ax2 and store those points in ptsMaxPeakObj_rX
    def plotClusterPointWithMaxPeakValue(self, ptsMaxPeakObj_rX, markerRadarPoint='o'):
        """
            Figure 2 in the documentation
        """
        if(ptsMaxPeakObj_rX.isEmpty() == False):
            try:
                image = self.ax2.scatter(ptsMaxPeakObj_rX.xs, ptsMaxPeakObj_rX.ys, c=ptsMaxPeakObj_rX.peakVals,
                                         cmap=plt.cm.get_cmap('viridis_r'), marker=markerRadarPoint)
            except ValueError:
                image = self.ax2.scatter([], [], c=[],
                                     cmap=plt.cm.get_cmap('viridis_r'), marker=markerRadarPoint)
                pass            
        else:
            image = self.ax2.scatter([], [], c=[],
                                     cmap=plt.cm.get_cmap('viridis_r'), marker=markerRadarPoint)

        return image

    def plotR1R2DetectionOfRLObject(self, epsilon, ptsMaxPeakObj_r12, ptsMaxPeakObj_r1, ptsMaxPeakObj_r2):
        """
            Calculate and plot the uncertainty ellipses with their intersection point (figure 3 in the documentation)
        """
        # ptsMaxPeakObj_r12 contains pairs of max peak value points each one from radar 1 and radar 2 (contains reallife objects that were detected by both radars)
        ptsPairedMaxPeakObj_r12 = RSTRUCT.DetectedPoints()
        # 2)find clusters of points that are max. 30cm distant to each other
        if(ptsMaxPeakObj_r12.isEmpty() == False and len(ptsMaxPeakObj_r12.getXYArray()) > 0):
            data = ptsMaxPeakObj_r12.getXYArray()
            print(data)
            print(f'epsilon{epsilon}')
            db = DBSCAN(eps=epsilon, min_samples=1).fit_predict(data)
            clusters = unique(db)
            selectedPoints = RSTRUCT.DetectedPoints()
            # print(clusters)
            for cluster in clusters:
                row_ix = where(db == cluster)
                # row_ix is array of cluster indices
                # [ [0,1], [2,3] ] meaning points at indices 0 and 1 (of data array)
                # are a cluster 1
                # and points at indices 2 and 3 are cluster 2
                for i in row_ix:  # iterate over the cluster 2d array
                    # print("i:",i)
                    # 3)check weather each cluster holds exactly one point of r1 and one point of radar2
                    if(len(i) == 2):  # if cluster contains 2 elements
                        if(i[0] < len(ptsMaxPeakObj_r1.xs) and i[1] >= len(ptsMaxPeakObj_r1.xs)):
                            # check weather first and second point of the cluster are from different radars
                            # (point1=radar1 point2=radar2
                            # 3.1 fill result data
                            xs = [ptsMaxPeakObj_r12.xs[i[0]],
                                  ptsMaxPeakObj_r12.xs[i[1]]]
                            ys = [ptsMaxPeakObj_r12.ys[i[0]],
                                  ptsMaxPeakObj_r12.ys[i[1]]]
                            vs = [ptsMaxPeakObj_r12.vs[i[0]],
                                  ptsMaxPeakObj_r12.vs[i[1]]]
                            peakVals = [ptsMaxPeakObj_r12.peakVals[i[0]],
                                        ptsMaxPeakObj_r12.peakVals[i[1]]]
                            origins = [0, 1]
                            ptsPairedMaxPeakObj_r12.extendAll(
                                xs, ys, vs, peakVals, origins)
                        elif(i[0] >= len(ptsMaxPeakObj_r1.xs) and i[1] < len(ptsMaxPeakObj_r1.xs)):
                            # point1=radar2 point2=radar1
                            # 3.1 fill result data
                            xs = [ptsMaxPeakObj_r12.xs[i[0]],
                                  ptsMaxPeakObj_r12.xs[i[1]]]
                            ys = [ptsMaxPeakObj_r12.ys[i[0]],
                                  ptsMaxPeakObj_r12.ys[i[1]]]
                            vs = [ptsMaxPeakObj_r12.vs[i[0]],
                                  ptsMaxPeakObj_r12.vs[i[1]]]
                            peakVals = [ptsMaxPeakObj_r12.peakVals[i[0]],
                                        ptsMaxPeakObj_r12.peakVals[i[1]]]
                            origins = [1, 0]
                            ptsPairedMaxPeakObj_r12.extendAll(
                                xs, ys, vs, peakVals, origins)
                # ptsPairedMaxPeakObj now holds pairs of points that represent the same object: One Radar 1, Other Radar2
                # plot point pairs
                (xs1, ys1, pks1, xs2, ys2,
                 pks2) = ptsPairedMaxPeakObj_r12.getR1R2DataXY()
                idx = 0
                self.xP1 = None
                self.yP1 = None
                self.xP2 = None
                self.yP2 = None
                for x in xs1:
                    if(x is not None and self.ptInRefDist(x, ys1[idx])):
                        self.xP1 = x
                        self.yP1 = ys1[idx]
                    idx += 1
                idx = 0
                for x in xs2:
                    if(x is not None and self.ptInRefDist(x, ys2[idx])):
                        self.xP2 = x
                        self.yP2 = ys2[idx]
                    idx += 1

                self.plotPoints(xs1, ys1, "PeakR1")
                self.plotPoints(xs2, ys2, "PeakR2")
                im5 = self.ax3.scatter(
                    xs1, ys1, c=pks1, cmap=plt.cm.get_cmap('viridis_r'), marker='o')
                im6 = self.ax3.scatter(
                    xs2, ys2, c=pks2, cmap=plt.cm.get_cmap('viridis_r'), marker='v')

                # 4 plot circles and intersection points
                # plot radar circles
                radar_pos_1 = (0, 0)
                radar_pos_2 = (0.15, 0)

                # Plot marker for the radar positions
                circle_marker_radar_1 = plt.Circle(
                    radar_pos_1, 0.01, color='g', fill=True)
                self.ax3.add_artist(circle_marker_radar_1)
                circle_marker_radar_2 = plt.Circle(
                    radar_pos_2, 0.01, color='r', fill=True)
                self.ax3.add_artist(circle_marker_radar_2)

                if(ptsPairedMaxPeakObj_r12.isEmpty() == False):
                    # Draw Circles
                    index = 0
                    pointDistanceToRadarList = []
                    circleIntersectionPtsX = []
                    circleIntersectionPtsY = []
                    circleIntersectionPts2X = []
                    circleIntersectionPts2Y = []
                    for x in ptsPairedMaxPeakObj_r12.xs:
                        if(x is not None and ptsPairedMaxPeakObj_r12.ys[index] is not None):
                        # if point is from radar 1
                            if(ptsPairedMaxPeakObj_r12.origins[index] < 1):
                                pointDistanceToRadarList.append(
                                    (x**2 + ptsPairedMaxPeakObj_r12.ys[index]**2) ** 0.5)
                                circle_marker = plt.Circle(
                                    radar_pos_1, pointDistanceToRadarList[index], color='g', fill=False)
                                #circle_marker2 = plt.Circle((x,ptsPairedMaxPeakObj_r12.ys[index]), 0.05, color='g', fill=False)

                                angle = 0
                                corrected_angle = 0

                                if x > 0:
                                    corrected_angle = math.atan(
                                        ptsPairedMaxPeakObj_r12.ys[index]/x)/math.pi*180
                                elif x < 0 and ptsPairedMaxPeakObj_r12.ys[index] >= 0:
                                    corrected_angle = (
                                        math.atan(ptsPairedMaxPeakObj_r12.ys[index]/x)/math.pi*180) + 180
                                elif x < 0 and ptsPairedMaxPeakObj_r12.ys[index] < 0:
                                    corrected_angle = (
                                        math.atan(ptsPairedMaxPeakObj_r12.ys[index]/x)/math.pi*180) - 180
                                elif x == 0 and ptsPairedMaxPeakObj_r12.ys[index] > 0:
                                    corrected_angle = 90
                                elif x == 0 and ptsPairedMaxPeakObj_r12.ys[index] < 0:
                                    corrected_angle = -90
                                elif x == 0 and ptsPairedMaxPeakObj_r12.ys[index] == 0:
                                    corrected_angle = None
                                circle_marker2 = Ellipse(xy=(x, ptsPairedMaxPeakObj_r12.ys[index]), width=(
                                    (ptsPairedMaxPeakObj_r12.ys[index]//0.043)*0.043)*math.tan(3.75*math.pi/180)*2, height=0.043*2, angle=(corrected_angle - 90), color='g', fill=False)

                                if(index % 2 == 1):  # Every 2nd iteration
                                    # Intersection of Circles with centers Radar 1 / Radar 2 and Range of respective Detected Point
                                    c1 = CB.Circle(
                                        radar_pos_1[0], radar_pos_1[1], pointDistanceToRadarList[index])
                                    c2 = CB.Circle(
                                        radar_pos_2[0], radar_pos_2[1], pointDistanceToRadarList[index-1])
                                    xyTmp = c1.circle_intersect(c2)

                                    ##################################

                                    # Ellipse with numpy
                                    ellipses = [(x, ptsPairedMaxPeakObj_r12.ys[index], ((ptsPairedMaxPeakObj_r12.ys[index]//0.043*0.043)*math.tan(3.75*math.pi/180)), 0.043, (corrected_angle - 90)), (ptsPairedMaxPeakObj_r12.xs[index-1],
                                                                                                                                                                                                       ptsPairedMaxPeakObj_r12.ys[index-1], ((ptsPairedMaxPeakObj_r12.ys[index-1]//0.043*0.043)*math.tan(3.75*math.pi/180)), 0.043, (corrected_angle - 90))]                                # Calculate a, b coefficients
                                    a, b = EN.ellipse_polyline(ellipses)
                                    # Calculate the intersection points
                                    ell_int_pts = EN.intersections(a, b)
                                    xyTmp2 = ell_int_pts

                                    ##################################
                                    if xyTmp is not None:
                                        for el in xyTmp:
                                            if(xyTmp.index(el) < 2):  # do not plot chordmidpoint
                                                circleIntersectionPtsX.append(
                                                    el[0])
                                                circleIntersectionPtsY.append(
                                                    el[1])
                                    if xyTmp2 is not None:
                                        for el in xyTmp2:
                                            circleIntersectionPts2X.append(el[0])
                                            circleIntersectionPts2Y.append(el[1])
                            else:  # point is from radar 2
                                pointDistanceToRadarList.append(
                                    ((x-radar_pos_2[0])**2 + (ptsPairedMaxPeakObj_r12.ys[index]-radar_pos_2[1])**2) ** 0.5)
                                circle_marker = plt.Circle(
                                    radar_pos_2, pointDistanceToRadarList[index], color='r', fill=False)
                                circle_marker2 = Ellipse(xy=(x, ptsPairedMaxPeakObj_r12.ys[index]), width=(
                                    (ptsPairedMaxPeakObj_r12.ys[index]//0.043)*0.043)*math.tan(3.75*math.pi/180)*2, height=0.043*2, angle=(corrected_angle - 90), color='g', fill=False)

                                if(index % 2 == 1):  # Every 2nd iteration
                                    # Intersection of Circles with centers Radar 1 / Radar 2 and Range of respective Detected Point
                                    c1 = CB.Circle(
                                        radar_pos_1[0], radar_pos_1[1], pointDistanceToRadarList[index-1])
                                    c2 = CB.Circle(
                                        radar_pos_2[0], radar_pos_2[1], pointDistanceToRadarList[index])
                                    xyTmp = c1.circle_intersect(c2)

                                    ##################################
                                    # Ellipse with numpy
                                    ellipses = [(x, ptsPairedMaxPeakObj_r12.ys[index], ((ptsPairedMaxPeakObj_r12.ys[index]//0.043*0.043)*math.tan(3.75*math.pi/180)), 0.043, (corrected_angle - 90)), (ptsPairedMaxPeakObj_r12.xs[index-1],
                                                                                                                                                                                                       ptsPairedMaxPeakObj_r12.ys[index-1], ((ptsPairedMaxPeakObj_r12.ys[index-1]//0.043*0.043)*math.tan(3.75*math.pi/180)), 0.043, (corrected_angle - 90))]                                # Calculate a, b coefficients
                                    a, b = EN.ellipse_polyline(ellipses)
                                    # Calculate the intersection points
                                    ell_int_pts = EN.intersections(a, b)
                                    xyTmp2 = ell_int_pts

                                    ##################################
                                    if xyTmp is not None:
                                        for el in xyTmp:
                                            if(xyTmp.index(el) < 2):  # do not plot chordmidpoint
                                                circleIntersectionPtsX.append(
                                                    el[0])
                                                circleIntersectionPtsY.append(
                                                    el[1])
                                    if xyTmp2 is not None:
                                        for el in xyTmp2:
                                            circleIntersectionPts2X.append(el[0])
                                            circleIntersectionPts2Y.append(el[1])
                            self.ax3.add_artist(circle_marker)
                            self.ax3.add_artist(circle_marker2)

                            index += 1

                    im7 = self.ax3.scatter(
                        circleIntersectionPtsX, circleIntersectionPtsY, marker='x', c='blue')
                    im8 = self.ax3.scatter(
                        circleIntersectionPts2X, circleIntersectionPts2Y, marker='X', c='blue')

        else:
            im5 = self.ax3.scatter(
                [], [], c=[], cmap=plt.cm.get_cmap('viridis_r'), marker='o')
            im6 = self.ax3.scatter(
                [], [], c=[], cmap=plt.cm.get_cmap('viridis_r'), marker='v')

        return im5, im6, ptsPairedMaxPeakObj_r12

    # Detected Point Pairs of Radar 1 and Radar 2. Indices of each array must refer to the same detected real world object/point

    def plotRadarBins(self, detPointsPairedR1R2Obj):
        """
            Calculate and plot the radar bins (figure 4 in the documentation)
        """
        index = 0
        xs1 = []
        ys1 = []
        xs2 = []
        ys2 = []
        for x in detPointsPairedR1R2Obj.xs:
            if(x is not None):
                if(detPointsPairedR1R2Obj.origins[index] == 0):
                    # radar 1 has no x offset (coordinate origin 0,0)
                    xs1, ys1 = RADAR_BINS.getRadarBin(
                        x, detPointsPairedR1R2Obj.ys[index])
                    if(index % 2 == 1):
                        xInt, yInt = RADAR_BINS.getIntesection(xs1, ys1, xs2, ys2)
                        self.ax4.plot(xs1, ys1, color='green')
                        self.ax4.plot(xs2, ys2, color='red')
                        self.ax4.plot(xInt, yInt, color='grey')
                        self.ax4.plot(
                            detPointsPairedR1R2Obj.xs[index], detPointsPairedR1R2Obj.ys[index], marker='o', color='black')
                        self.ax4.plot(
                            detPointsPairedR1R2Obj.xs[index-1], detPointsPairedR1R2Obj.ys[index-1], marker='v', color='black')

                else:
                    # radar 2 has offset (coordinate origin (0.15, 0)
                    xs2, ys2 = RADAR_BINS.getRadarBinOffset(
                        detPointsPairedR1R2Obj.xs[index], detPointsPairedR1R2Obj.ys[index], 0.15)
                    if(index % 2 == 1):
                        xInt, yInt = RADAR_BINS.getIntesection(xs1, ys1, xs2, ys2)
                        self.ax4.plot(xs1, ys1, color='green')
                        self.ax4.plot(xs2, ys2, color='red')
                        self.ax4.plot(xInt, yInt, color='grey')
                        self.ax4.plot(
                            detPointsPairedR1R2Obj.xs[index-1], detPointsPairedR1R2Obj.ys[index-1], marker='o', color='black')
                        self.ax4.plot(
                            detPointsPairedR1R2Obj.xs[index], detPointsPairedR1R2Obj.ys[index], marker='v', color='black')

            index += 1
            #im1 = self.ax4.scatter(detPtsRadar1X, detPtsRadar1Y, c=pks1,cmap=plt.cm.get_cmap('viridis_r'), marker='o')
            #im2 = self.ax4.scatter(xs2, ys2, c=pks2,cmap=plt.cm.get_cmap('viridis_r'), marker='v')

    def plotPeakValueColorbar(self, imageToHostColorbar, axToHostColorbar, imageArrayToRecolor, lowerLimit=0, upperLimit=6000):
        # draw peak value colorbar
        tmpPkVals = []
        tmpPkVals.extend(self.ptsObj_r1.peakVals)
        # fig.colorbar(im1, ax=ax1, label="Peak Value",values=ptsObj_r1.peakVals.extend(ptsObj_r2.peakVals))
        self.cbar = self.fig.colorbar(imageToHostColorbar, ax=axToHostColorbar, label="Peak Value",
                                      values=tmpPkVals.extend(self.ptsObj_r2.peakVals))
        for im in imageArrayToRecolor:
            im.set_clim(lowerLimit, upperLimit)

    def clearAxesAndCbar(self):
        """
            Clear the plots and the color bar
        """
        self.ax1.cla()
        self.ax1.callbacks.connect('xlim_changed', self.on_xlims_change)
        self.ax1.callbacks.connect('ylim_changed', self.on_ylims_change)
        if self.ax2 is not None:
            self.ax2.cla()
            self.ax2.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax2.callbacks.connect('ylim_changed', self.on_ylims_change)
        if self.ax3 is not None:
            self.ax3.cla()
            self.ax3.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax3.callbacks.connect('ylim_changed', self.on_ylims_change)
        if self.ax4 is not None:
            self.ax4.cla()
            self.ax4.callbacks.connect('xlim_changed', self.on_xlims_change)
            self.ax4.callbacks.connect('ylim_changed', self.on_ylims_change)
        self.ax1.set_xlim(self.xmin, self.xmax)
        self.ax1.set_ylim(self.ymin, self.ymax)

        if self.cbar is not None:
            # colorbar cannot be removed via cla. therefor remove it manually and rebuild it later
            self.cbar.remove()

    def configureAxes(self):
        """
            Configure the axis
        """
        # set labels & titles & activate grid
        self.fig.suptitle('Detected Objects (live view)', fontsize=16)
        self.ax1.set_title('Radar data: Detected Points, Peak Vals & Clusters')
        self.ax1.set_xlabel('x-coordinate [m]')
        self.ax1.set_ylabel('y-coordinate [m]')
        self.ax1.grid()
        if self.ax2 is not None:
            self.ax2.set_title('Max PeakVal Point of each Cluster')
            self.ax2.set_xlabel('x-coordinate [m]')
            self.ax2.set_ylabel('y-coordinate [m]')
            self.ax2.grid()
        if self.ax3 is not None:
            self.ax3.set_title('Intersection of two range circles')
            self.ax3.set_xlabel('x-coordinate [m]')
            self.ax3.set_ylabel('y-coordinate [m]')
            self.ax3.grid()
        if self.ax4 is not None:
            self.ax4.set_title('Intersection of Radar-Range/Angle-Bins')
            self.ax4.set_xlabel('x-coordinate [m]')
            self.ax4.set_ylabel('y-coordinate [m]')
            self.ax4.grid()
        # connect callbacks for zoom & hook mpl cursors
        self.ax1.callbacks.connect('xlim_changed', self.on_xlims_change)
        self.ax1.callbacks.connect('ylim_changed', self.on_ylims_change)
        mplcursors.cursor(multiple=True)
        
        
        
        
        
