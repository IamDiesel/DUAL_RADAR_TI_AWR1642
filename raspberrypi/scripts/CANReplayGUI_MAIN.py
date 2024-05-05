"""
Author: Daniel Kahrizi
Date: 25.05.2020
Description: GUI for CAN-Replay.
This script can be used to replay a log file on the virtual canbus vcan0
To setup vcan0 on raspberry pi use the following commands:
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set vcan0 up

also use sudo pigpiod to start gpio

This script can also open the CAN_PI_MAIN_SCRIPT.py (parsing and plotting Radar data). It can be chosen via checkbox, if the plot script will use can0 or vcan0 as input for the radar data.
"""
import tkinter as tk    # Python GUI library
from tkinter import *
from tkinter import filedialog
import tkinter.ttk as tkk
import queue
from threading import Thread
import CANReplay as CANReplay
import Settings
import time
import os
import datetime
from pathlib import Path

timePerCycle = 1/600000000


def main():
    """
        Main function of the CANReplay GUI. Initializes all queues and calls the mainApp mainloop function.
    """
    # threadQueue = queue.LifoQueue()
    threadQueue = queue.Queue()
    guiQueue = queue.LifoQueue()
    thread_queue_frmCtrl = queue.Queue()
    termination_queue = queue.Queue()
    frame_index_queue = queue.Queue()
    mainApp = MainApp(threadQueue, guiQueue, thread_queue_frmCtrl,
                      termination_queue, frame_index_queue)
    mainApp.protocol("WM_DELETE_WINDOW", mainApp.onClose)
    mainApp.mainloop()


class MainApp(tk.Tk):

    """
        MainApp class containing all settings for the GUI.
    """

    def __init__(self, thread_queue, gui_queue, thread_queue_frmCtrl, termination_queue, frame_index_queue):
        ####### Do something ######
        super(MainApp, self).__init__()

        # general settings
        self.settingsApp = Settings.Settings(self)
        self.filename = ""
        self.thread_queue = thread_queue
        self.gui_queue = gui_queue
        self.thread_queue_frmCtrl = thread_queue_frmCtrl
        self.termination_queue = termination_queue
        self.frame_index_queue = frame_index_queue
        self.threadList = []  # CAN-Replay related threads
        self.threadObjectList = []  # CAN-Replay related objects
        self.plotThreadList = []  # Plot / Main Program related threads
        self.plotObjectList = []  # Plot / Main Porgram related objects
        self.title("CAN Replay virtual CAN-Bus")

        # button creation
        self.buttonframe = Frame(self)
        self.button_open = Button(
            self.buttonframe, text="Open Log File", command=self.browseFiles, width=15)
        self.button_stop = Button(
            self.buttonframe, text="Stop Thread", command=self.stopCANReplayThread, width=15)
        self.button_start = Button(
            self.buttonframe, text="Start Thread", command=self.startCANReplayThread, width=15)
        self.button_settings = Button(
            self.buttonframe, text="Settings", command=self.showSettings, width=15)
        self.button_pause = Button(
            self.buttonframe, text="Pause Replay", command=self.pause, width=15)
        self.button_continue = Button(
            self.buttonframe, text="Continue Replay", command=self.start, width=15)
        self.button_last = Button(
            self.buttonframe, text="Prev. Frame", command=self.lastFrame, width=15)
        self.button_next = Button(
            self.buttonframe, text="Next Frame", command=self.nextFrame, width=15)
        self.button_single = Button(
            self.buttonframe, text="Play Single Frame", command=self.single, width=15)
        self.button_startPlot = Button(
            self.buttonframe, text="Run Plot App", command=self.startPiPlot, width=15)
        self.button_stopPlot = Button(
            self.buttonframe, text="Stop Plot App", command=self.stopPiPlot, width=15)
        self.button_startConfig = Button(
            self.buttonframe, text="Start Radar Cfg", command=self.startConfig, width=15)
        self.button_stopConfig = Button(
            self.buttonframe, text="Stop Radar Cfg", command=self.stopConfig, width=15)
        self.button_startRDPlot = Button(
            self.buttonframe, text="Start RD Plot", command=self.startPiRDPlot, width=15)
        self.button_stopRDPlot = Button(
            self.buttonframe, text="Stop RD Plot", command=self.stopPiRDPlot, width=15)
        self.valSendContinuous = IntVar(self)
        self.checkBoxSC = Checkbutton(self.buttonframe, text="Send continuous mode",
                                      variable=self.valSendContinuous, command=self.push2Queue)

        #   x/y-limits
        self.xminLabel = Label(self.buttonframe, text="x min",
                               width=30, height=1, fg="black", relief="groove")
        self.xmaxLabel = Label(self.buttonframe, text="x max",
                               width=30, height=1, fg="black", relief="groove")
        self.yminLabel = Label(self.buttonframe, text="y min",
                               width=30, height=1, fg="black", relief="groove")
        self.ymaxLabel = Label(self.buttonframe, text="y max",
                               width=30, height=1, fg="black", relief="groove")
        self.xminVar = StringVar(self)
        self.xmaxVar = StringVar(self)
        self.yminVar = StringVar(self)
        self.ymaxVar = StringVar(self)
        self.xminVar.set("-10")
        self.xmaxVar.set("+10")
        self.yminVar.set("0")
        self.ymaxVar.set("10")
        self.xminEntry = Entry(self.buttonframe, width=15,
                               textvariable=self.xminVar)
        self.xmaxEntry = Entry(self.buttonframe, width=15,
                               textvariable=self.xmaxVar)
        self.yminEntry = Entry(self.buttonframe, width=15,
                               textvariable=self.yminVar)
        self.ymaxEntry = Entry(self.buttonframe, width=15,
                               textvariable=self.ymaxVar)

        #   button settings
        self.button_open.grid(column=1, row=1)
        self.button_start.grid(column=2, row=1)
        self.button_stop.grid(column=3, row=1)
        self.button_settings.grid(column=4, row=1)
        self.button_last.grid(column=1, row=2, sticky='ew')
        self.button_continue.grid(column=2, row=2, sticky='ew')
        self.button_single.grid(column=3, row=2, sticky='ew')
        self.button_pause.grid(column=4, row=2, sticky='ew')
        self.button_next.grid(column=5, row=2, sticky='ew')
        self.checkBoxSC.grid(column=3, row=3, sticky='ew')
        self.button_startPlot.grid(column=2, row=4)
        self.button_stopPlot.grid(column=3, row=4, sticky='ew')
        self.button_startConfig.grid(column=5, row=4, sticky='ew')
        self.button_stopConfig.grid(column=6, row=4, sticky='ew')
        self.button_startRDPlot.grid(column=7, row=4, sticky='ew')
        self.button_stopRDPlot.grid(column=8, row=4, sticky='ew')
        self.canCheckboxFrame = Frame(self.buttonframe)
        self.realCANCheckboxVar = IntVar(self)
        self.vCANCheckboxVar = IntVar(self)
        self.checkCAN0 = Checkbutton(self.canCheckboxFrame, text="CAN 0",
                                     variable=self.realCANCheckboxVar, command=self.toggleCheckbuttonVCAN)
        self.checkCAN0.grid(row=0, sticky=W)
        self.checkVCAN0 = Checkbutton(self.canCheckboxFrame, text="VCAN 0",
                                      variable=self.vCANCheckboxVar, command=self.toggleCheckbuttonCAN)
        self.checkVCAN0.grid(row=1, sticky=W)
        self.checkVCAN0.select()

        #   Figure grid
        self.canCheckboxFrame.grid(column=4, row=4, sticky='ew')
        self.xminLabel.grid(column=2, row=5, sticky='ew')
        self.xmaxLabel.grid(column=3, row=5, sticky='ew')
        self.yminLabel.grid(column=4, row=5, sticky='ew')
        self.ymaxLabel.grid(column=5, row=5, sticky='ew')
        self.xminEntry.grid(column=2, row=6, sticky='ew')
        self.xmaxEntry.grid(column=3, row=6, sticky='ew')
        self.yminEntry.grid(column=4, row=6, sticky='ew')
        self.ymaxEntry.grid(column=5, row=6, sticky='ew')

        # Progress bar settings
        self.buttonframe.pack(side=TOP)
        self.bottomframe = Frame(self)
        self.progressframe = Frame(self.bottomframe)
        self.pBarStyle = tkk.Style()
        self.pBarStyle.theme_use('clam')
        self.pBarStyle.configure(
            "Horizontal.TProgressbar", foreground='blue', background='grey', troughcolor='black')
        self.progressBar = tkk.Progressbar(
            self.progressframe, orient=HORIZONTAL, length=100, mode='determinate', style="Horizontal.TProgressbar")
        self.progressBar.pack()
        self.progressBar["maximum"] = 100
        self.progressBar["value"] = 0
        self.label_progress = Label(
            self.progressframe, text="-", width=75, height=1, fg="black", wraplength=300)
        self.label_progress.pack(side=TOP)
        self.progressframe.pack(side=BOTTOM)
        self.labelframe = Frame(self.bottomframe)
        self.label_selected_file = Label(
            self.labelframe, text="No file selected", width=75, height=4, fg="blue", wraplength=300)
        self.labelR1Frame = Label(
            self.progressframe, text="Time R1 (ID 0xC1): [-]", width=30, height=1, fg="black", bg="grey", relief="groove")
        self.labelR2Frame = Label(
            self.progressframe, text="Time R2 (ID 0x81): [-]", width=30, height=1, fg="black", bg="grey", relief="groove")
        self.labelNoneFrame = Label(self.progressframe, text="Unknown",
                                    width=30, height=1, fg="black", bg="grey", relief="groove")
        self.labelR1Frame.pack(side=TOP)
        self.labelR2Frame.pack(side=TOP)
        self.labelNoneFrame.pack(side=TOP)

        # Frame Index Selection
        self.frameIndexSelectionFrame = Frame(self.progressframe)
        self.frameInput = StringVar(self)
        self.frameInput.set("0")
        self.nameEntered = Entry(
            self.frameIndexSelectionFrame, width=15, textvariable=self.frameInput)
        self.setFrameIndexButton = Button(
            self.frameIndexSelectionFrame, text="Jump to Frame Index", command=self.pushFrameIndexToQueue)
        self.nameEntered.pack(side=LEFT)
        self.setFrameIndexButton.pack(side=LEFT)
        self.frameIndexSelectionFrame.pack(side=BOTTOM)

        self.label_selected_file.pack()
        self.labelframe.pack(side=TOP)
        self.bottomframe.pack(side=BOTTOM)

        self.lastReceivedTSR1 = 0
        self.lastReceivedTSR2 = 0

    def pushFrameIndexToQueue(self):
        """
            Put the desired frame index to the queue
        """
        try:
            integerValue = int(str(self.frameInput.get()))
            self.frame_index_queue.put(integerValue)
            print(integerValue)
        except ValueError:
            pass

    # label R1 = 0, label R2 = 1, Other Data label = 2
    def highlightRadarFrame(self, labelNo, textR1, textR2, textOther):
        """
            Label the radar frames
        """
        self.labelR1Frame.configure(bg="grey", text=textR1)
        self.labelR2Frame.configure(bg="grey", text=textR2)
        self.labelNoneFrame.configure(bg="grey", text=textOther)
        if(labelNo == 0):
            self.labelR1Frame.configure(bg="green")
        elif(labelNo == 1):
            self.labelR2Frame.configure(bg="green")
        elif(labelNo == 2):
            self.labelNoneFrame.configure(bg="green")

    def toggleCheckbuttonVCAN(self):
        """
            Toggle the button input, wheather the virtual CAN bus is used
        """
        self.checkVCAN0.toggle()

    def toggleCheckbuttonCAN(self):
        """
            Toggle the button input, wheather the real CAN bus is used
        """
        self.checkCAN0.toggle()

    def browseFiles(self):
        """
            Function to browse the files, e.g. to find the desired log files
        """
        self.filename = filedialog.askopenfilename(
            initialdir="./", title="Open CAN logfile", filetypes=(("Log files", "*.log"), ("All files", "*.")))
        self.label_selected_file.configure(text="Replay File: "+self.filename)

    def update_progressBar(self):
        """
            Update the progress bar, which is displaying
        """
        # Check if there is something in the queue
        try:
            hasCanProgTerminated = self.termination_queue.get_nowait()  # has program terminated?
            if(hasCanProgTerminated == -1):
                print("Finished CAN-Send Task.")
                self.progressBar["value"] = 0
                self.stopCANReplayThread()
        except queue.Empty:
            pass

        try:
            (progress, curFrameIdx, maxFrameIdx, firstFrameTSR1, curFrameTS, maxFrameTSR1,
             firstFrameTSR2, maxFrameTSR2, isR1Data, isR2Data) = self.gui_queue.get_nowait()

            progress = int(progress)

            if(progress >= 0):
                self.progressBar["value"] = progress

                textProgressLabel = "Frame: [" + \
                    str(curFrameIdx) + "/" + str(maxFrameIdx) + "]"
                if(isR1Data == True):
                    curFrameTimeR1 = ((curFrameTS-firstFrameTSR1)
                                      * timePerCycle).__round__(3)
                else:
                    curFrameTimeR1 = (
                        (self.lastReceivedTSR1-firstFrameTSR1)*timePerCycle).__round__(3)
                if(isR2Data == True):
                    curFrameTimeR2 = ((curFrameTS-firstFrameTSR2)
                                      * timePerCycle).__round__(3)
                else:
                    curFrameTimeR2 = (
                        (self.lastReceivedTSR2-firstFrameTSR2)*timePerCycle).__round__(3)
                if(curFrameTimeR1 < 0):
                    curFrameTimeR1 = 0
                if(curFrameTimeR2 < 0):
                    curFrameTimeR2 = 0

                maxFrameTimeR1 = ((maxFrameTSR1-firstFrameTSR1)
                                  * timePerCycle).__round__(3)
                maxFrameTimeR2 = ((maxFrameTSR2-firstFrameTSR2)
                                  * timePerCycle).__round__(3)

                if(isR2Data == True):
                    self.highlightRadarFrame(1, "Time R1 (ID 0xC1): [" + str(curFrameTimeR1) + "/" + str(
                        maxFrameTimeR1) + "] s", "Time R2 (ID 0x81): [" + str(curFrameTimeR2) + "/" + str(maxFrameTimeR2) + "] s", "Other Data")
                elif(isR1Data == True):
                    self.highlightRadarFrame(0, "Time R1 (ID 0xC1): [" + str(curFrameTimeR1) + "/" + str(
                        maxFrameTimeR1) + "] s", "Time R2 (ID 0x81): [" + str(curFrameTimeR2) + "/" + str(maxFrameTimeR2) + "] s ", "Other Data")
                else:
                    self.highlightRadarFrame(2, "Time R1 (ID 0xC1): [" + str(curFrameTimeR1) + "/" + str(
                        maxFrameTimeR1) + "] s", "Time R2 (ID 0x81): [" + str(curFrameTimeR2) + "/" + str(maxFrameTimeR2) + "] s", "Other Data")

                self.label_progress.configure(text=textProgressLabel)
                self.update_idletasks()
                self.update()
                with self.gui_queue.mutex:
                    self.gui_queue.queue.clear()
                if(isR2Data == True):
                    self.lastReceivedTSR2 = curFrameTS
                elif(isR1Data == True):
                    self.lastReceivedTSR1 = curFrameTS
                self.after(500, self.update_progressBar)

        except queue.Empty:
            self.after(500, self.update_progressBar)

    def plotSystemThread(self):
        """
            Create a new thread for the CAN radar interface main function
        """
        usevcan = self.vCANCheckboxVar.get()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(dir_path, 'CAN_RADAR_INTERFACE_MAIN.py')
        xmin = float(str(self.xminVar.get()))
        xmax = float(str(self.xmaxVar.get()))
        ymin = float(str(self.yminVar.get()))
        ymax = float(str(self.ymaxVar.get()))
        (_, _, loggingActive, filename, refX, refY, refRange,plotPeak, plotCOG, plotComb,plotBins, epsCOG, epsPeak, epsCombined, refreshInterval, _, _) = self.settingsApp.getSettings()
        print(f'python3 {file_path} {int(usevcan)} {xmin} {xmax} {ymin} {ymax} {int(loggingActive)} {filename} {refX} {refY} {refRange} {int(plotPeak)} {int(plotCOG)} {int(plotComb)} {int(plotBins)} {epsCOG} {epsPeak} {epsCombined} {int(refreshInterval)}')
        res = os.system(f'python3 {file_path} {int(usevcan)} {xmin} {xmax} {ymin} {ymax} {int(loggingActive)} {filename} {refX} {refY} {refRange} {int(plotPeak)} {int(plotCOG)} {int(plotComb)} {int(plotBins)} {epsCOG} {epsPeak} {epsCombined} {int(refreshInterval)}')
        print("Result OS Call:"+str(res))

    def startPiPlot(self):
        """
            Start the CAN radar interface app
        """
        plotShellThread = Thread(target=self.plotSystemThread)
        self.plotThreadList.append(plotShellThread)
        plotShellThread.start()

    def plotRDSystemThread(self):
        """
            Create a new thread for the CAN radar interface main function
        """
        usevcan = self.vCANCheckboxVar.get()
        (_, _, loggingActive, filename, refX, refY, refRange,plotPeak, plotCOG, plotComb,plotBins, epsCOG, epsPeak, epsCombined, refreshInterval, imageDoExport, imageFilename) = self.settingsApp.getSettings()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(dir_path, 'raspberrypi_scripts_recv_parse_range_doppler.py')
        if(imageDoExport):          
            Path("./data/"+imageFilename).mkdir(parents=True, exist_ok=True)
            logfilepath = dir_path+"/data/"+imageFilename+"/"+datetime.datetime.now().strftime("%d_%m_%Y__%H_%M_%S")+".log"
            print("logfilepath: " + logfilepath)
            if(usevcan):
                res = os.system(f'python3 {file_path} {int(usevcan)} {int(imageDoExport)} {imageFilename} & candump vcan0 > {logfilepath}')
            else:
                #print("DirPath: " + dir_path)
                #print(f'python3 {file_path} {int(usevcan)} {int(imageDoExport)} {imageFilename} ; candump can0 > {logfilepath}')
                res = os.system(f'python3 {file_path} {int(usevcan)} {int(imageDoExport)} {imageFilename} & candump can0 > {logfilepath}')
                #res = os.system(f'candump can0 > {logfilepath}')
        else:
            res = os.system(f'python3 {file_path} {int(usevcan)} {int(imageDoExport)} {imageFilename}')


    def startPiRDPlot(self):
        """
            Start the CRange Doppler Processing + Plot
        """
        plotShellThread = Thread(target=self.plotRDSystemThread)
        self.plotThreadList.append((plotShellThread))
        plotShellThread.start()

    def configSystemThread(self):
        """
            Create a new thread for the CAN radar configuration app
        """
        usevcan = self.vCANCheckboxVar.get()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(dir_path, 'CAN_RADAR_CONFIG_MAIN.py')
        print(f'python3 {file_path} {usevcan}')
        res = os.system(f'python3 {file_path} {usevcan}')
        print("Result OS Call:"+str(res))

    def startConfig(self):
        """
            Start the CAN radar configuration app
        """
        plotShellThread = Thread(target=self.configSystemThread)
        self.plotThreadList.append(plotShellThread)
        plotShellThread.start()

    def stopConfig(self):
        """
            Stop the CAN radar configuration app
        """
        while(len(self.plotThreadList) > 0):
            os.system("pkill -f CAN_RADAR_CONFIG_MAIN.py")
            self.plotThreadList.pop(0).join()

    def stopPiPlot(self):
        """
            Stop the CAN radar interface app
        """
        while(len(self.plotThreadList) > 0):
            os.system("pkill -f CAN_RADAR_INTERFACE_MAIN.py")
            os.system("sudo pigs w 13 0")
            os.system("sudo pigs w 18 0")
            self.plotThreadList.pop(0).join()

    def stopPiRDPlot(self):
        """
            Stop the CAN radar interface app
        """
        while(len(self.plotThreadList) > 0):
            (_, _, loggingActive, filename, refX, refY, refRange,plotPeak, plotCOG, plotComb,plotBins, epsCOG, epsPeak, epsCombined, refreshInterval, imageDoExport, imageFilename) = self.settingsApp.getSettings()
            os.system("pkill -f raspberrypi_scripts_recv_parse_range_doppler.py")
            if(imageDoExport):
                os.system("pkill -f candump")
            os.system("sudo pigs w 13 0")
            os.system("sudo pigs w 18 0")
            self.plotThreadList.pop(0).join()

    def startCANReplayThread(self):
        """
            Start the CAN replay app
        """
        while(self.filename == ""):
            self.browseFiles()
        self.gui_queue = queue.LifoQueue()
        self.threadQueue = queue.Queue()
        self.thread_queue_frmCtrl = queue.Queue()
        self.termination_queue = queue.Queue()
        self.frame_index_queue = queue.Queue()
        currentSettings = self.settingsApp.getSettings()
        canRef = CANReplay.CANSendProgram(self.gui_queue, self.termination_queue, self.thread_queue, self.thread_queue_frmCtrl, self.frame_index_queue, self.filename, currentSettings[0], currentSettings[1])
        self.threadObjectList.append(canRef)
        canThread = Thread(target=canRef.run, kwargs={"contMode": self.valSendContinuous.get()})
        self.threadList.append(canThread)
        self.pBarStyle.configure("Horizontal.TProgressbar", troughcolor='grey')
        canThread.start()
        self.update_progressBar()

    def stopCANReplayThread(self):
        """
            Stop the CAN replay app
        """
        print("trying to stop CAN-Thread")
        if(len(self.threadObjectList) > 0):
            self.threadObjectList.pop(0).terminate()
            time.sleep(0.1)
            self.threadList.pop(0).join()
            print("CAN-Thread Stopped")
            self.pBarStyle.configure(
                "Horizontal.TProgressbar", troughcolor='black')
            self.progressBar["value"] = 0
            self.label_progress.configure(text="-")
            self.highlightRadarFrame(
                -1, "Time R1 (ID 0xC1): [-]", "Time R2 (ID 0x81): [-]", "Other Data")
        else:
            print("all threads were already stopped")

    def push2Queue(self):
        """
            Check, if continous mode is selected
        """
        # print("VALSENDCONTINUOUS IS:" + str(self.valSendContinuous.get()))
        if(self.valSendContinuous.get() == 0):
            self.thread_queue.put(0)
        else:
            self.thread_queue.put(1)

    #   Control functions for the GUI, pause/start the replay app or select the next/last frame

    def pause(self):
        self.thread_queue_frmCtrl.put(0)

    def start(self):
        self.thread_queue_frmCtrl.put(1)

    def lastFrame(self):
        self.thread_queue_frmCtrl.put(2)

    def nextFrame(self):
        self.thread_queue_frmCtrl.put(3)

    def single(self):
        self.thread_queue_frmCtrl.put(4)

    def showSettings(self):
        """
            Display the settings window
        """
        self.settingsApp.show()

    def onClose(self):
        """
            Action, which happens when the app is being closed
        """
        self.after_cancel(self)
        while(len(self.threadObjectList) > 0):
            self.stopCANReplayThread()
        self.stopPiPlot()
        self.stopConfig()
        self.destroy()
        self.quit()
        sys.exit()


if __name__ == '__main__':
    main()
