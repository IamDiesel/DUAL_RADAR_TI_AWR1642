"""
Author: Daniel Kahrizi
Date: 05.07.2020
This script is used by CANReplayGUI_Main.py to load and save the configuration.
Inter Frame Waiting Time: Time between frames that CANReplay will wait/sleep
Inter Message Waiting Time: Time between CAN-Messages that CAN-Replay will wait/sleep
"""
import tkinter as tk
from tkinter import *
from tkinter import filedialog
import tkinter.ttk as tkk
from configparser import ConfigParser
from configparser import NoOptionError, NoSectionError, NoOptionError

import traceback


class Settings(object):
    def __init__(self, parent):
        # Create new window
        self.root = Toplevel(parent)
        self.root.title("Settings")
        
        #Scrollbar  
        resX, resY = self.get_curr_screen_geometry()
        print(resX,resY)
        if(int(resY) > 768):
            resY=700
        else:
            resY= int(2/3*int(resY))
        self.root.geometry(f'600x{resY}')         
        self.wrapper = LabelFrame(self.root) #width=200, height=500
        self.canvas = Canvas(self.wrapper,width=580, height=resY)
        self.canvas.pack(side=LEFT, fill="both", expand="yes")
        self.yScrollbar = ttk.Scrollbar(self.wrapper, orient="vertical", command=self.canvas.yview)
        self.yScrollbar.pack(side=RIGHT, fill="y")
        self.canvas.configure(yscrollcommand=self.yScrollbar.set)
        self.canvas.bind('<Configure>',lambda e: self.canvas.configure(scrollregion = self.canvas.bbox('all')))        
        self.rootScrollableWindow = Frame(self.canvas)#width=200, height=500
        self.wrapper.pack()
        self.canvas.create_window((0,0), window=self.rootScrollableWindow, anchor="nw")
        # Variables
        self.config = ConfigParser()
        self.filename = "settings.cfg"

        # TK Variables
        self.interFrameTimeVar = DoubleVar(self.rootScrollableWindow)
        self.interMessageTimeVar = DoubleVar(self.rootScrollableWindow)
        self.loggingActiveVar = IntVar(self.rootScrollableWindow)
        self.loggingFilenameVar = StringVar(self.rootScrollableWindow)
        self.refXVar = DoubleVar(self.rootScrollableWindow)
        self.refYVar = DoubleVar(self.rootScrollableWindow)
        self.refRangeVar = DoubleVar(self.rootScrollableWindow)
        self.plotPeakPointsVar = IntVar(self.rootScrollableWindow)  # draw points with max peak value per cluster (2nd plot)
        self.plotCenterOfGravityVar = IntVar(self.rootScrollableWindow)  # draw COG in first (left) plot
        self.plotCombinedDetectionVar = IntVar(self.rootScrollableWindow)
        self.plotBinsVar = IntVar(self.rootScrollableWindow)
        self.clusterCOGEpsilonVar = DoubleVar(self.rootScrollableWindow)
        self.clusterCombinedDetectionVar = DoubleVar(self.rootScrollableWindow)
        self.clusterPeakEpsilonVar = DoubleVar(self.rootScrollableWindow)
        self.refreshIntervalVar = DoubleVar(self.rootScrollableWindow)
        self.imageFoldernameVar = StringVar(self.rootScrollableWindow)
        self.imageDoExportVar = IntVar(self.rootScrollableWindow)

        # Read Settings from file
        try:
            self.readSettings()
        except (NoOptionError, NoSectionError, NoOptionError):
            self.setDefaultNoSettings()
            self.readSettings()
            pass
        except Exception as e:
            print("Error while loading settings ..:" + str(e) + traceback.format_exc())
        # loggingFilename = None, loggingActive = False, refX = 0, refY = 0, refRange = 0.3
        # Labels
        labelwidth = 30
        self.categoryReplayLabel = Label(self.rootScrollableWindow, text="CAN-Replay Options:", width=65, height=3, fg="black", anchor='w')
        self.interFrameTimeLabel = Label(self.rootScrollableWindow, text="Inter Frame Waiting Time [s]", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.interMessageTimeLabel = Label(self.rootScrollableWindow, text="Inter Message Waiting Time [s]", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.categoryLoggingLabel = Label(self.rootScrollableWindow, text="CSV-Logging-Options:", width=65, height=3, fg="black", anchor='w')
        self.loggingFilenameLabel = Label(self.rootScrollableWindow, text="Export Filename", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.refXLabel = Label(self.rootScrollableWindow, text="Reference Center Point X", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.refYLabel = Label(self.rootScrollableWindow, text="Reference Center Point Y", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.refRangeLabel = Label(self.rootScrollableWindow, text="Validity-Range from Center Point", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.categoryPlotClusterLabel = Label(self.rootScrollableWindow, text="Clustering and Plotting:", width=65, height=3, fg="black", anchor='w')
        self.clusterCOGEpsilonLabel = Label(self.rootScrollableWindow, text="Epsilon Cluster Center of Gravity", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.clusterPeakEpsilonLabel = Label(self.rootScrollableWindow, text="Epsilon Cluster Maximum Peak Value", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.clusterCombinedDetectionLabel = Label(self.rootScrollableWindow, text="Epsilon Cluster Combined Max. Peak Values", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.categoryGeneralLabel = Label(self.rootScrollableWindow, text="General:", width=65, height=3, fg="black", anchor='w')
        self.refreshIntervalLabel = Label(self.rootScrollableWindow, text="Plot refresh interval [ms]:", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        self.categoryImageLabel = Label(self.rootScrollableWindow, text="Image Export Options:", width=65, height=3, fg="black", anchor='w')
        self.imageFoldernameLabel = Label(self.rootScrollableWindow, text="Image Export Foldername", width=labelwidth, height=1, fg="black", relief="groove", anchor='w')
        # Checkboxes
        self.loggingCheckBox = Checkbutton(self.rootScrollableWindow, text="Log Plot to csv (Max Peak R1/R2, CoG)", variable=self.loggingActiveVar)
        self.plotPeakPointsCheckBox = Checkbutton(self.rootScrollableWindow, text="Plot clustered Peaks", variable=self.plotPeakPointsVar)
        self.plotCenterOfGravityCheckBox = Checkbutton(self.rootScrollableWindow, text="Plot Center of Gravity", variable=self.plotCenterOfGravityVar)
        self.plotCombinedDetectionCheckBox = Checkbutton(self.rootScrollableWindow, text="Plot measurement uncertainty ellipses", variable=self.plotCombinedDetectionVar, command=self.plotDependency)
        self.plotBinsCheckBox = Checkbutton(self.rootScrollableWindow, text="Plot Radar Bins for detected Objects", variable=self.plotBinsVar, command=self.plotDependency)
        self.imageDoExportCheckBox = Checkbutton(self.rootScrollableWindow, text="Export Image Mode On", variable=self.imageDoExportVar)
        # Entry Fields
        self.interFrameTimeEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.interFrameTimeVar)
        self.interMessageTimeEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.interMessageTimeVar)
        self.loggingFilenameEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.loggingFilenameVar)
        self.refXEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.refXVar)
        self.refYEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.refYVar)
        self.refRangeEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.refRangeVar)
        self.clusterCOGEpsilonEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.clusterCOGEpsilonVar)
        self.clusterPeakEpsilonEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.clusterPeakEpsilonVar)
        self.clusterCombinedDetectionEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.clusterCombinedDetectionVar)
        self.refreshIntervalEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.refreshIntervalVar)
        self.imageFoldernameEntry = Entry(self.rootScrollableWindow, width=15, textvariable=self.imageFoldernameVar)
        # Buttons
        self.buttonSave = Button(self.rootScrollableWindow, text="Save", command=self.writeSettings, width=15)
        self.buttonLoad = Button(self.rootScrollableWindow, text="Load", command=self.readSettings, width=15)
        self.buttonDefaults = Button(self.rootScrollableWindow, text="Load Defaults", command=self.setDefault, width=15)
        # Layout
        self.categoryReplayLabel.grid(row=0, columnspan=4, sticky=W)
        self.interFrameTimeLabel.grid(row=1, column=1)
        self.interFrameTimeEntry.grid(row=1, column=2)
        self.interMessageTimeLabel.grid(row=2, column=1)
        self.interMessageTimeEntry.grid(row=2, column=2)
        self.categoryLoggingLabel.grid(row=3, columnspan=4, sticky=W)
        self.loggingCheckBox.grid(row=4, column=1, sticky=W)
        self.loggingFilenameLabel.grid(row=5, column=1)
        self.loggingFilenameEntry.grid(row=5, column=2)
        self.refXLabel.grid(row=6, column=1)
        self.refXEntry.grid(row=6, column=2)
        self.refYLabel.grid(row=7, column=1)
        self.refYEntry.grid(row=7, column=2)
        self.refRangeLabel.grid(row=8, column=1)
        self.refRangeEntry.grid(row=8, column=2)
        self.categoryPlotClusterLabel.grid(row=9, columnspan=4, sticky=W)
        self.plotCenterOfGravityCheckBox.grid(row=10, column=1, sticky=W)
        self.plotPeakPointsCheckBox.grid(row=11, column=1, sticky=W)
        self.plotCombinedDetectionCheckBox.grid(row=12, column=1, sticky=W)
        self.plotBinsCheckBox.grid(row=13, column=1, sticky=W)
        self.clusterCOGEpsilonLabel.grid(row=14, column=1)
        self.clusterCOGEpsilonEntry.grid(row=14, column=2)
        self.clusterPeakEpsilonLabel.grid(row=15, column=1)
        self.clusterPeakEpsilonEntry.grid(row=15, column=2)
        self.clusterCombinedDetectionLabel.grid(row=16, column=1)
        self.clusterCombinedDetectionEntry.grid(row=16, column=2)
        self.refreshIntervalLabel.grid(row=17, column=1)
        self.refreshIntervalEntry.grid(row=17, column=2)

        self.categoryImageLabel.grid(row=18, columnspan=4, sticky=W)
        self.imageDoExportCheckBox.grid(row=19, column=1, sticky=W)
        self.imageFoldernameLabel.grid(row=20, column= 1)
        self.imageFoldernameEntry.grid(row=20, column = 2)


        self.categoryGeneralLabel.grid(row=21, columnspan=4, sticky=W)
        self.buttonDefaults.grid(row=22, column=1)
        self.buttonLoad.grid(row=22, column=2)
        self.buttonSave.grid(row=22, column=3)
        
       
        # Close Option
        self.root.protocol("WM_DELETE_WINDOW", self.onClose)
        # Hide this window on creation
        self.hide()
    
    def get_curr_screen_geometry(self):
        """
        Workaround to get the size of the current screen in a multi-screen setup.

        Returns:
            geometry (str): The standard Tk geometry string.
                [width]x[height]+[left]+[top]
        """
        root = tk.Tk()
        root.update_idletasks()
        root.attributes('-fullscreen', True)
        root.state('iconic')
        geometry = root.winfo_geometry()
        root.destroy()
        geomX, rest = geometry.split("x")
        geomY,_,_ = rest.split("+")
        return (geomX, geomY)

    def readSettings(self):
        self.config.read(self.filename)
        self.interFrameTimeVar.set(self.config.getfloat('replay', 'inter_frame_waiting_time'))
        self.interMessageTimeVar.set(self.config.getfloat('replay', 'inter_message_waiting_time'))
        self.loggingActiveVar.set(self.config.getboolean('logging', 'logging_active'))
        self.loggingFilenameVar.set(self.config.get('logging', 'filename'))
        self.refXVar.set(self.config.getfloat('logging', 'ref_x'))
        self.refYVar.set(self.config.getfloat('logging', 'ref_y'))
        self.refRangeVar.set(self.config.getfloat('logging', 'ref_range'))
        self.plotPeakPointsVar.set(self.config.getboolean('plotting', 'peak_points'))
        self.plotCenterOfGravityVar.set(self.config.getboolean('plotting', 'center_of_gravity'))
        self.plotCombinedDetectionVar.set(self.config.getboolean('plotting', 'combined_detection'))
        self.plotBinsVar.set(self.config.getboolean('plotting', 'radar_bins'))
        self.refreshIntervalVar.set(self.config.getfloat('plotting', 'refresh_interval'))
        self.clusterCOGEpsilonVar.set(self.config.getfloat('clustering', 'eps_center_of_gravity'))
        self.clusterPeakEpsilonVar.set(self.config.getfloat('clustering', 'eps_peak_values'))
        self.clusterCombinedDetectionVar.set(self.config.getfloat('clustering', 'eps_combined_peak_values'))
        self.imageDoExportVar.set(self.config.getboolean('image_export', 'export_image_mode_on'))
        self.imageFoldernameVar.set(self.config.get('image_export', 'foldername'))

    def writeSettings(self):
        self.config.set('replay', 'inter_frame_waiting_time', str(self.interFrameTimeVar.get()))
        self.config.set('replay', 'inter_message_waiting_time', str(self.interMessageTimeVar.get()))
        self.config.set('logging', 'logging_active', str(bool(self.loggingActiveVar.get())))
        self.config.set('logging', 'filename', str(self.loggingFilenameVar.get()))
        self.config.set('logging', 'ref_x', str(self.refXVar.get()))
        self.config.set('logging', 'ref_y', str(self.refYVar.get()))
        self.config.set('logging', 'ref_range', str(self.refRangeVar.get()))
        self.config.set('plotting', 'peak_points', str(bool(self.plotPeakPointsVar.get())))
        self.config.set('plotting', 'center_of_gravity', str(bool(self.plotCenterOfGravityVar.get())))
        self.config.set('plotting', 'combined_detection', str(bool(self.plotCombinedDetectionVar.get())))
        self.config.set('plotting', 'radar_bins', str(bool(self.plotBinsVar.get())))
        self.config.set('plotting', 'refresh_interval', str(self.refreshIntervalVar.get()))
        self.config.set('clustering', 'eps_center_of_gravity', str(self.clusterCOGEpsilonVar.get()))
        self.config.set('clustering', 'eps_peak_values', str(self.clusterPeakEpsilonVar.get()))
        self.config.set('clustering', 'eps_combined_peak_values', str(self.clusterCombinedDetectionVar.get()))
        self.config.set('image_export', 'export_image_mode_on', str(bool(self.imageDoExportVar.get())))
        self.config.set('image_export', 'foldername', str(self.imageFoldernameVar.get()))
        with open(self.filename, 'w') as file:
            self.config.write(file)

    def getConfigParser(self):
        return self.config

    def getSettings(self):
        interFrameTime = self.interFrameTimeVar.get()
        interMessageTime = self.interMessageTimeVar.get()
        loggingActive = bool(self.loggingActiveVar.get())
        filename = self.loggingFilenameVar.get()
        refX = self.refXVar.get()
        refY = self.refYVar.get()
        refRange = self.refRangeVar.get()
        plotPeak = bool(self.plotPeakPointsVar.get())
        plotCOG = bool(self.plotCenterOfGravityVar.get())
        plotComb = bool(self.plotCombinedDetectionVar.get())
        plotBins = bool(self.plotBinsVar.get())
        epsCOG = self.clusterCOGEpsilonVar.get()
        epsPeak = self.clusterPeakEpsilonVar.get()
        epsCombined = self.clusterCombinedDetectionVar.get()
        refreshInterval = self.refreshIntervalVar.get()
        imageExportModeOn = bool(self.imageDoExportVar.get())
        imageFoldername = self.imageFoldernameVar.get()
        return (interMessageTime, interFrameTime, loggingActive, filename, refX, refY, refRange, plotPeak, plotCOG, plotComb, plotBins, epsCOG, epsPeak, epsCombined, refreshInterval, imageExportModeOn, imageFoldername)

    def setDefault(self):
        self.interFrameTimeVar.set(0.0001)
        self.interMessageTimeVar.set(0.0001)
        self.loggingActiveVar.set(0)
        self.loggingFilenameVar.set("measurement")
        self.refXVar.set(0.0)
        self.refYVar.set(1.0)
        self.refRangeVar.set(0.3)
        self.plotPeakPointsVar.set(1)
        self.plotCenterOfGravityVar.set(1)
        self.plotCombinedDetectionVar.set(1)
        self.plotBinsVar.set(1)
        self.clusterCOGEpsilonVar.set(0.3)
        self.clusterPeakEpsilonVar.set(0.3)
        self.clusterCombinedDetectionVar.set(0.4)
        self.refreshIntervalVar.set(500)
        self.imageDoExportVar.set(0)
        self.imageFoldernameVar.set("ImageExport")

    def setDefaultNoSettings(self):
        self.config.add_section('replay')
        self.config.set('replay', 'inter_frame_waiting_time', '0.0001')
        self.config.set('replay', 'inter_message_waiting_time', '0.0001')
        self.config.add_section('logging')
        self.config.set('logging', 'logging_active', 'False')
        self.config.set('logging', 'filename', 'measurement')
        self.config.set('logging', 'ref_x', '0.0')
        self.config.set('logging', 'ref_y', '1.0')
        self.config.set('logging', 'ref_range', '0.3')
        self.config.add_section('plotting')
        self.config.set('plotting', 'peak_points', 'True')
        self.config.set('plotting', 'center_of_gravity', 'True')
        self.config.set('plotting', 'combined_detection', 'True')
        self.config.set('plotting', 'radar_bins', 'True')
        self.config.set('plotting', 'refresh_interval', '500')
        self.config.add_section('clustering')
        self.config.set('clustering', 'eps_center_of_gravity', '0.3')
        self.config.set('clustering', 'eps_peak_values', '0.3')
        self.config.set('clustering', 'eps_combined_peak_values', '0.4')
        self.config.add_section('image_export')
        self.config.set('image_export', 'export_image_mode_on', 'False')
        self.config.set('image_export', 'foldername', 'ImageExport')
        self.writeSettings()
        self.readSettings()

    def plotDependency(self):
        if (self.plotCombinedDetectionVar.get() == 0):
            self.plotBinsVar.set(0)

    def show(self):
        self.root.update()
        self.root.deiconify()

    def hide(self):
        self.root.withdraw()

    def onClose(self):
        self.hide()  # self.root.after_cancel(self)  # self.root.destroy()  # self.root.quit()  # sys.exit()
