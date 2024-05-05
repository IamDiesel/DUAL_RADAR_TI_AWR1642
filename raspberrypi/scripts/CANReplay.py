
"""
Author: Daniel Kahrizi
Date: 25.05.2020
Description: This script can be used to replay a log file on the virtual canbus vcan0
To setup vcan0 use the following commands:
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set vcan0 up
"""
import can
import time
from queue import Queue
import queue
import struct   # to build structs, s = char byte, h/H signed/unsigned 2 Byte  short, i/I signed/unsigned 4 Byte integer
import binascii  # to decode a ascii stream to hex values
import traceback


####################
#### CAN-Config ####
####################

bustype = 'socketcan'
channel = 'vcan0'
use_fd = True
use_ext_ID = True
useChirpTimestamp = False
raspi = True


def hexStr2HexInt(stringVal):
    """
        Return the hex integer value of a hex string value
    """
    return int(stringVal, 16)


def hexStrArr2HexIntArr(hexStrArr):
    """
        Return the hex integer array of a hex string array
    """
    hexIntArr = []
    for hexStr in hexStrArr:
        hexIntArr.append(hexStr2HexInt(hexStr))
    return hexIntArr


class CANRecording:
    # TODO Add handling of configuration messages
    """
        Class for the CAN recording app, including the read in routine for the log files
    """

    def __init__(self, filepath, useChirpTimestamps):
        self.filepath = filepath
        self.frames = []
        self.amountMessages = 0
        self.lastTimestampR1 = -2
        self.firstTimestampR1 = -2
        self.lastTimestampR2 = -2
        self.firstTimestampR2 = -2
        self.parseLogFile(useChirpTimestamp)

    def parseLogFile(self, useChirpTimestamp):
        print("Processing log file. Please wait...")
        try:
            file = open(self.filepath, 'r')
            lines = file.readlines()
            self.amountMessages = len(lines)
            messageBlock = []
            frameTimestamp = -1
            frameNumber = -1
            lastMessageWasFrameHeader = False
            index = 0
            length = "/"+str(len(lines))

            for line in lines:
                # each CAN_Message is one line. Iterate over all CAN-Message Lines and bundle the messages in message blocks for each Radarframe. From the message block frame objects are generated and added to frame list "Frames"
                # During parsing the Frame message blocks are identified as follows:
                # The algorithm searches for the first frame header with can-id 0xC1 or 0xC1-0x40 that is not in the first line of the file. Also this message must contain the magic Word. All messages that are read before this frame header are bundled in a messageblock.
                # So usually the first line is a frame header. Since it is the first line no frame object is created at this point. The algorithm continues parsing the following lines (and adding them to the messageblock) until the next frame header is found.
                # At this point all messages excluding the just found 2nd frame header are used to create a frame object which is added to the frame list. The message block list is cleared and ready for the next messages of the block.
                # The algorithm now continues searching for the next frame header and on the way adding all messages in between to the message block.When the next frame header is found the next frame object is generated and added to the list.
                #
                # If the first messages are configuration messeges with no frame header at the beginning these messages will also be bundled in a message block and treated as a frame. The bundle willstart at the first message and end at the first frame header message 0xC1 or 0xC1-0x40 with magic word.
                # Since this algorithm identifies the frameheader by CAN-ID + Magic word, it can handle Extended Frame headers (with chirp timestamps) aswell as normal Frame headers (without chirp timestamps). The only requirement is that the length of messages with 0xC1 or 0xC1-0x40 must be 64 bytes.
                #
                # This algorithm also sets the first and last frame-timestamp that was found for each Radar. These timestamps are also displayed at the GUI.
                curMessage = CANRecording.Message(line)
                magicWord = b'0x00'
                if(index % 10 == 0):
                    # print(str(index)+length)
                    pass

                if(curMessage.getCANID() == 0xC1 or curMessage.getCANID() == 0xC1-0x40):
                    s = struct.Struct('8s 4I Q 2I 8s 2Q')
                    packed = binascii.unhexlify(curMessage.getDataRaw())
                    values = s.unpack(packed)
                    # buffer for the return value, assign the values direct to the buffer
                    magicWord = values[0]
                    #timestamp = values[9]
                # if message is Frameheader of Radar 1
                if(curMessage.getCANID() == 0xC1 or curMessage.getCANID() == 0xC1-0x40):
                    # if first element of file is C1-Frame-Header then wait until next C1 Header is found. All messages will be stored in messageblock
                    if(magicWord == b'\x02\x01\x04\x03\x06\x05\x08\x07' and lines.index(line) != 0):
                        #print("line C1:" + line)
                        # add messages to frame
                        curFrame = CANRecording.Frame(messageBlock)
                        #print(f"TS: {curFrame.getFrameTimestamp()*timePerCycle}")
                        self.frames.append(curFrame)
                        timePerCycle = 1/600000000
                        #frameTime = curFrame.getFrameTimestamp()*timePerCycle
                        #print(f"TS: {frameTime}")
                        # reset message block since Frame is complete (cur message is a Frame Header C1)
                        messageBlock = []
                        #lastMessageWasFrameHeader = True
                        if(curFrame.isRadarOneFrame):
                            # No R1 timestamp found yet
                            if(self.firstTimestampR1 <= -1 and curFrame.getFrameTimestamp() >= 0):
                                self.firstTimestampR1 = curFrame.getFrameTimestamp()
                            self.lastTimestampR1 = curFrame.getFrameTimestamp()
                        elif(curFrame.isRadarTwoFrame):
                            # no R2 timestamp found yet
                            if(self.firstTimestampR2 <= -1 and curFrame.getFrameTimestamp() >= 0):
                                self.firstTimestampR2 = curFrame.getFrameTimestamp()
                            self.lastTimestampR2 = curFrame.getFrameTimestamp()

                else:
                    pass
                    # lastMessageWasFrameHeader = False #just for debug
                messageBlock.append(curMessage)
            timePerCycle = 1/600000000
            index += 1

        except Exception as e:
            print("End application.. :"+str(e) + traceback.format_exc())

    def getFrames(self):
        return self.frames

    def getFrame(self, index):
        return self.frames[index]

    def getAmountFrames(self):
        return len(self.frames)

    class Message:
        """
            Class for the messages used in the replay app, which are send on the virtual CAN-Bus as a replay
        """

        def __init__(self, messageDataRaw):
            dataFieldRaw = messageDataRaw.split(']')
            dataFieldRaw = dataFieldRaw[1]
            dataFieldRaw = dataFieldRaw.replace(" ", "")
            dataFieldRaw = dataFieldRaw.replace("\n", "")
            self.messageDataRaw = dataFieldRaw
            self.messageData = []
            tmp = messageDataRaw.split(']')
            idlen = tmp[0].split(" ")
            canId = idlen[4]
            self.isFrameHeaderMsg = False
            self.frameNumber = -1
            self.frameTimestamp = -1
            data = tmp[1].replace("  ", "").replace("\n", "").split(" ")
            self.messageData.append(hexStrArr2HexIntArr(data))
            self.canIdHex = hexStr2HexInt(canId)

        def getData(self):
            return self.messageData

        def getCANID(self):
            return self.canIdHex

        def getDataRaw(self):
            return self.messageDataRaw

    class Frame:
        """
            Class for the frame used in the replay app. Replay of the recorded frames
        """

        def __init__(self, messageBlock):
            self.messages = messageBlock
            # defines weather this frame has Frame Header with id 0xC1,
            self.isRadarOneFrame = False
            # defines weather this frame has frame header with can id 0xC1-0x40
            self.isRadarTwoFrame = False

            if(self.messages[0].getCANID() == 0xC1 or self.messages[0].getCANID() == 0xC1-0x40):
                if(self.messages[0].getCANID() == 0xC1):
                    self.isRadarOneFrame = True
                elif(self.messages[0].getCANID() == 0xC1-0x40):
                    self.isRadarTwoFrame = True
                # s = struct.Struct('8s 4I Q 2I 8s 66Q') #for extendedFrameHeader with ChirpTimestamps
                s = struct.Struct('8s 4I Q 2I 8s 2Q')
                packed = binascii.unhexlify(self.messages[0].getDataRaw())
                values = s.unpack(packed)
                # buffer for the return value, assign the values direct to the buffer
                magicWord = values[0]
                if(magicWord == b'\x02\x01\x04\x03\x06\x05\x08\x07'):
                    self.frameNumber = values[4]
                    self.frameTimestamp = values[9]
                else:
                    self.frameNumber = -1
                    self.frameTimestamp = -1
                # chirpTimestampArr = values[10:74] #only for extended header with chirp Timestamps
            else:
                self.frameNumber = -1
                self.frameTimestamp = -1

        def getMessages(self):
            return self.messages

        def getFrameNumber(self):
            return self.frameNumber

        def getFrameTimestamp(self):
            return self.frameTimestamp

        def printFrame(self):
            print("****************")
            print(f"isR1:{self.isRadarOneFrame}")
            print(f"isR2:{self.isRadarTwoFrame}")
            frameTime = 1/600000000 * self.frameTimestamp
            print(f"FrameTime:{frameTime}")
            for msg in self.messages:
                print(f"{msg.getCANID()}:{msg.getDataRaw()}")


class CANSendProgram:
    """
        Class for the CAN send module of the replay module. Contains function to replay the recorded messages/frames
    """

    def __init__(self, gui_queue, termination_queue, thread_queue, thread_queue_frmCtrl, frame_index_queue, filename, idleTimeMessage, idleTimeFrame):
        """
        class constructor. Inizializes the object.
        """
        self.running = True
        # 0.010 #idle time between messenges. Fixed idleTime since no timestamps are used.
        self.idleTime = idleTimeMessage
        self.idleTimeFrame = idleTimeFrame  # 2.2
        self.continuous = False
        self.gui_queue = gui_queue
        self.thread_queue = thread_queue
        self.thread_queue_frmCtrl = thread_queue_frmCtrl
        self.termination_queue = termination_queue
        self.frame_index_queue = frame_index_queue
        self.filename = filename
        self.bus = 0  # connection to bus will be made when run() is called
        self.progress = 0
        self.canRecording = CANRecording(filename, useChirpTimestamp)
        self.playing = False
        self.curFrameIdx = 0
        self.curFrame = self.canRecording.getFrame(self.curFrameIdx)
        self.state = "Init"
        self.updateProgressBar()

        # self.connect_to_can_bus()

    def connect_to_can_bus(self):
        """
        Call this function to connect to the CAN-bus. Connects to vcan0 when flag raspi is true.
        Else program will print the CAN Output to the console.
        """

        self.bus = 0
        try:
            # todo uncomment
            if(raspi):
                self.bus = can.interface.Bus(
                    channel=channel, bustype=bustype, fd=use_fd)
            else:
                self.bus = 0
            print("Connected to vcan0 successfully.")
        except:
            print("Couldn't find vcan0.. error")
            exit()

    def sendMessage(self, msg):
        if(raspi):
            message = can.Message(arbitration_id=msg.getCANID(
            ), is_fd=use_fd, is_extended_id=use_ext_ID, data=msg.getData()[0])
            self.bus.send(message)
        else:
            print("["+str(hex(msg.getCANID()))+"]"+msg.getDataRaw())
        self.updateProgressBar()

    def terminate(self):
        """
        Call this function to terminate the CAN-.Send-Thread. -1 is put to GUI-QUEUE which signalizes the gui to delete this thread.
        """
        print("Terminate called")
        self.termination_queue.put(-1)
        self.running = False

    def runReplay(self, lines):
        print("")

    def updateProgressBar(self):
        frames = self.canRecording.getAmountFrames()-1
        if(frames > 0):
            progressPercent = int(self.curFrameIdx/frames*100)
        else:
            progressPercent = 0

        self.gui_queue.put((progressPercent, self.curFrameIdx, self.canRecording.getAmountFrames()-1, self.canRecording.firstTimestampR1, self.curFrame.getFrameTimestamp(),
                            self.canRecording.lastTimestampR1, self.canRecording.firstTimestampR2, self.canRecording.lastTimestampR2, self.curFrame.isRadarOneFrame, self.curFrame.isRadarTwoFrame))

    def resetProgressBar(self):
        self.progress = 0

    def readQueueValues(self):
        resultQThread = -1
        resultQFrmCtrl = -1
        resultQFrmIdx = -1
        try:
            resultQThread = self.thread_queue.get_nowait()
            print("Continuous hit")
        except queue.Empty:
            result = -1
            pass
        try:
            resultQFrmCtrl = self.thread_queue_frmCtrl.get_nowait()
        except queue.Empty:
            resultQFrmCtrl = -1
            pass

        try:
            resultQFrmIdx = self.frame_index_queue.get_nowait()
        except queue.Empty:
            resultQFrmIdx = -1
            pass

        return resultQThread, resultQFrmCtrl, resultQFrmIdx

    def run(self, contMode):
        """
        Mainloop of CAN-Send. This function sends out the can-messages that were read via filedescriptor. If self.continuous is set the file will be sent repeatedtly until thread is killed or self.continous is set
        to false via thread_queue.
        """
        # main processing loop
        try:
            if (contMode == 1):
                self.continuous = True
            elif(contMode == 0):
                self.continuous = False
            self.connect_to_can_bus()
            print("Now entering the main processing loop")

            while(self.running):
                ################################################
                # Update inputs
                ################################################
                qValContinuous, qValFrmCtrl, qValFrmIdx = self.readQueueValues()
                # print("qValContinuous"+str(qValContinuous)+"qValFrm:"+str(qValFrmCtrl))
                # print(self.state)
                if(qValContinuous == 1):
                    self.continuous = True
                    #print("Received continuous True")
                elif(qValContinuous == 0):
                    self.continuous = False
                    #print("Received continuous False")
                maxFrmIdx = len(self.canRecording.getFrames())-1
                if(qValFrmIdx >= 0 and qValFrmIdx <= maxFrmIdx):
                    self.curFrameIdx = qValFrmIdx

                ################################################
                # State Transitions
                ################################################
                if(qValFrmCtrl == 0):
                    self.state = "Pause"
                elif(qValFrmCtrl == 1):
                    self.state = "Start"
                elif(qValFrmCtrl == 2):
                    self.state = "LastFrame"
                elif(qValFrmCtrl == 3):
                    self.state = "NextFrame"
                elif(qValFrmCtrl == 4):
                    self.state = "SingleFrame"

                if(self.state == "Start"):
                    if(self.continuous == True):
                        self.state = "StartContinuous"
                    # elif(self.curFrameIdx >= maxFrmIdx):
                    #    self.state = "Pause"
                elif(self.state == "StartContinuous"):
                    if(self.continuous == False):
                        self.state = "Start"

                ################################################
                # State Actions
                ################################################
                if(self.state == "Init"):
                    pass
                elif(self.state == "Pause"):
                    pass
                elif(self.state == "Start"):
                    if(self.curFrameIdx <= maxFrmIdx):
                        self.curFrame = self.canRecording.getFrames()[
                            self.curFrameIdx]
                        for message in self.curFrame.getMessages():
                            self.sendMessage(message)
                            time.sleep(self.idleTime)
                        time.sleep(self.idleTimeFrame)
                        if(raspi == False):
                            print("*****")
                        self.updateProgressBar()
                        if(self.curFrameIdx < maxFrmIdx):
                            self.curFrameIdx += 1
                        else:
                            self.state = "Pause"

                elif(self.state == "StartContinuous"):
                    # if(self.curFrameIdx == len(self.canRecording.getFrames())):
                    #self.curFrameIdx = 0
                    # self.resetProgressBar()
                    if(self.curFrameIdx <= maxFrmIdx):
                        self.curFrame = self.canRecording.getFrames()[
                            self.curFrameIdx]
                        for message in self.curFrame.getMessages():
                            self.sendMessage(message)
                            time.sleep(self.idleTime)
                        time.sleep(self.idleTimeFrame)

                        if(self.curFrameIdx == 0):
                            self.resetProgressBar()
                        self.updateProgressBar()
                        if(self.curFrameIdx < maxFrmIdx):
                            self.curFrameIdx += 1
                        else:
                            self.curFrameIdx = 0

                elif(self.state == "LastFrame"):
                    if(self.continuous and self.curFrameIdx <= 0):
                        self.curFrameIdx = maxFrmIdx+1
                    if(self.curFrameIdx > 0):
                        self.curFrameIdx -= 1
                        self.curFrame = self.canRecording.getFrames()[
                            self.curFrameIdx]
                        for message in self.curFrame.getMessages():
                            self.sendMessage(message)
                            time.sleep(self.idleTime)
                        self.updateProgressBar()
                    self.state = "Pause"
                elif(self.state == "NextFrame"):
                    if(self.continuous and self.curFrameIdx >= maxFrmIdx):
                        self.curFrameIdx = -1
                    if(self.curFrameIdx < maxFrmIdx):
                        self.curFrameIdx += 1
                        self.curFrame = self.canRecording.getFrames()[
                            self.curFrameIdx]
                        for message in self.curFrame.getMessages():
                            self.sendMessage(message)
                            time.sleep(self.idleTime)
                        self.updateProgressBar()
                    self.state = "Pause"
                elif(self.state == "SingleFrame"):
                    if(self.curFrameIdx <= maxFrmIdx and self.curFrameIdx >= 0):
                        for message in self.curFrame.getMessages():
                            self.sendMessage(message)
                            time.sleep(self.idleTime)
                        self.updateProgressBar()
                    self.state = "Pause"

        except Exception as e:
            print("End application.. CAN Replay :" +
                  str(e) + traceback.format_exc())
