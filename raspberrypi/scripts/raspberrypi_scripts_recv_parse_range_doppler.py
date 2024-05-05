#!/usr/bin/env python3

"""
Author: Steven Duong, Daniel Kahrizi
Date: 19.04.2020
Description:
Test script to listen to the can bus and parse the received messages to a range doppler matrix

TODO: check case first zero missing, second used for first one
# Comment

Good
527 528 0 1

Bad
528 527 0 1 x
527 528 1 0 x 
527 0 528 1 
"""

import can
import numpy as np
import time
import sys
import pigpio
import logging
from threading import Thread
import queue
import Plotter as PLOT
import signal
import wavePWM
import traceback

# can config
bustype = 'socketcan'
channel = 'can0'
channel_backup = 'vcan0'
use_fd = True
#HSE CAN ID
use_ext_ID = False
bus = None

# can id of the range doppler matrix
#can_id_ranDopMat_radar0 = 0x000000DA
#can_id_ranDopMat_radar1 = 0x000000DA-0x40
can_id_ranDopMat_radar0 = 0xDA
can_id_ranDopMat_radar1 = 0xDA-0x40

# buffer size of range doppler matrix
range_size = 256#512
doppler_size = 64#32


def connectToCAN(useVCAN=False):
    global bustype, channel, channel_backup, use_fd, use_ext_ID, bus
    if(useVCAN == False):
        try:
            print("Try to connect to can0.. ")
            bus = can.interface.Bus(
                channel=channel, bustype=bustype, fd=use_fd)
            print("Connected to can0 successfully.")
        except:
            print("Couldn't find CAN0.. error")
            exit()
    else:
        try:
            bus = can.interface.Bus(channel=channel_backup,
                                    bustype=bustype, fd=use_fd)
            print("Connected to vcan0 successfully.")
        except:
            print("Couldn't find vcan0.. error")
            exit()


def parse_message(msg, radar_id, is_first_zero_flag, catched_previous_zero_flag, catched_previous_zero_msg, is_first_one_flag, catched_previous_one_flag, catched_previous_one_msg, catched_previous_527_flag, catched_previous_527_msg, success_oneround, range_doppler_matrix, raw_can_buffer, rd_q1, rd_q2):

    global doppler_size, range_size
    # var
    failed_parse = 0

    # get the current can payload counter
    counter_index = msg.data[0]+msg.data[1]*256

    if (counter_index == 1):
        if is_first_one_flag == 0:
            is_first_one_flag = 1
        else:
            # buffer message with index 1, in case, that message 0 and 1 were swapped
            catched_previous_one_msg = msg
            catched_previous_one_flag = 1
            return (is_first_zero_flag, catched_previous_zero_flag, catched_previous_zero_msg, is_first_one_flag, catched_previous_one_flag, catched_previous_one_msg, catched_previous_527_flag, catched_previous_527_msg, success_oneround)

    # stream is synchronized to msg 0
    if(counter_index == 0):
        if is_first_zero_flag == 0:
            is_first_zero_flag = 1
        else:
            # buffer message with index 0, in case 0 and 528 were swapped
            catched_previous_zero_flag = 1
            catched_previous_zero_msg = msg
            return (is_first_zero_flag, catched_previous_zero_flag, catched_previous_zero_msg, is_first_one_flag, catched_previous_one_flag, catched_previous_one_msg, catched_previous_527_flag, catched_previous_527_msg, success_oneround)
    # enter only, if a loop iteration has been runned once
    if success_oneround == 1:
        success_oneround = 0

        # assign swapped message to the right round
        if catched_previous_one_flag == 1:
            for x in range(catched_previous_one_msg.dlc):
                raw_can_buffer[1][x] = catched_previous_one_msg.data[x]
            catched_previous_one_flag = 0
            catched_previous_one_msg = can.Message()

        if catched_previous_527_flag == 1:
            for x in range(catched_previous_527_msg.dlc):
                raw_can_buffer[msg.data[0] + msg.data[1] *
                               256][x] = catched_previous_527_msg.data[x]
            catched_previous_527_msg = can.Message()
            catched_previous_527_flag = 0

        if catched_previous_zero_flag == 1:
            for x in range(catched_previous_zero_msg.dlc):
                raw_can_buffer[0][x] = catched_previous_zero_msg.data[x]
            catched_previous_zero_flag = 0
            catched_previous_zero_msg = can.Message()

    # fill the message in the right buffer
    for x in range(msg.dlc):
        raw_can_buffer[msg.data[0] + msg.data[1]*256][x] = msg.data[x]

    # received the message with index 528 (last message)
    if(counter_index == 529-1):

        # walk through all messages
        for x in range(529):
            # check if all buffer were filled (completeness)
            if (raw_can_buffer[x][0] + raw_can_buffer[x][1]*256) == x:
                pass
            else:
                # if 528 and 527 were swapped, get next message and check if it is 527
                if x == 527:
                    catched_previous_527_msg = bus.recv()
                    if (catched_previous_527_msg.data[0] + catched_previous_527_msg.data[1]*256) == 527:
                        for x in range(catched_previous_527_msg.dlc):
                            raw_can_buffer[527][x] = catched_previous_527_msg.data[x]
                    else:
                        catched_previous_527_flag = 1  # muss in nächster runde korrigiert werden
                else:
                    # Reset buffer on failed parse
                    raw_can_buffer = [
                        [0 for x in range(64)] for y in range(529)]
                    raw_can_buffer[0][0] = 1
                    raw_can_buffer[0][1] = 1
                    is_first_one_flag = 0
                    is_first_zero_flag = 0
                    success_oneround = 1
                    parse_counter = 0
                    failed_parse = 1
                    break
        # dont parse, leave iteration
        if failed_parse == 1:
            failed_parse = 0
            print("failed parse")
            return (is_first_zero_flag, catched_previous_zero_flag, catched_previous_zero_msg, is_first_one_flag, catched_previous_one_flag, catched_previous_one_msg, catched_previous_527_flag, catched_previous_527_msg, success_oneround)

        # Parse the buffer and fill the range doppler matrix
        parse_counter = 0

        for x in range(529):
            for y in range(32-1):
                #range_doppler_matrix[parse_counter//32][parse_counter %32] = (raw_can_buffer[x][y*2+2]+raw_can_buffer[x][y*2+2+1]*256)
                range_doppler_matrix[parse_counter//doppler_size][parse_counter % doppler_size] = (raw_can_buffer[x][y*2+2]+raw_can_buffer[x][y*2+2+1]*256)
                parse_counter = parse_counter + 1
                if (x == 528 and y == 15):
                    break

        if(radar_id == 0):
            #print("Radar 0")
            rd_q1.put(range_doppler_matrix)
        else:
            #print("Radar 1")
            rd_q2.put(range_doppler_matrix)
        # Do something with the data
        # print(range_doppler_matrix[0])
        # print(range_doppler_matrix[1])
        # print(range_doppler_matrix[511])

        range_size, doppler_size
        range_doppler_matrix = [
            [0 for x in range(doppler_size)] for y in range(range_size)]

        # reset vars on succeed parse
        success_oneround = 1
        is_first_zero_flag = 0
        is_first_one_flag = 0

        # Reset buffer on succeed parse
        raw_can_buffer = [[0 for x in range(64)] for y in range(529)]
        raw_can_buffer[0][0] = 1
        raw_can_buffer[0][1] = 1

    return (is_first_zero_flag, catched_previous_zero_flag, catched_previous_zero_msg, is_first_one_flag, catched_previous_one_flag, catched_previous_one_msg, catched_previous_527_flag, catched_previous_527_msg, success_oneround)


class CAN_ParseRangeDoppler_Program:
    def __init__(self, r1_q, r2_q):
        logging.info("Starting Range Doppler Parser")
        self.running = True
        self.rd_q1 = r1_q
        self.rd_q2 = r2_q

    def terminate(self):
        logging.info("CAN_listener shutdown")
        self.running = False
        #exit(0)

    def run(self):
        collect_message(self.rd_q1, self.rd_q2)


def collect_message(rd_q1, rd_q2):

    # <~~~~~ variables ~~~~~>
    global bus

    is_first_zero_flag_r0 = 0
    is_first_zero_flag_r1 = 0
    catched_previous_zero_flag_r0 = 0
    catched_previous_zero_flag_r1 = 0
    catched_previous_zero_msg_r0 = can.Message()
    catched_previous_zero_msg_r1 = can.Message()

    is_first_one_flag_r0 = 0
    is_first_one_flag_r1 = 0
    catched_previous_one_flag_r0 = 0
    catched_previous_one_flag_r1 = 0
    catched_previous_one_msg_r0 = can.Message()
    catched_previous_one_msg_r1 = can.Message()
    catched_previous_527_flag_r0 = 0
    catched_previous_527_flag_r1 = 0
    catched_previous_527_msg_r0 = can.Message()
    catched_previous_527_msg_r1 = can.Message()
    success_oneround_r0 = 0
    success_oneround_r1 = 0

    global range_size, doppler_size

    range_doppler_matrix_r0 = [
        [0 for x in range(doppler_size)] for y in range(range_size)]
    range_doppler_matrix_r1 = [
        [0 for x in range(doppler_size)] for y in range(range_size)]

    # can buffer
    raw_can_buffer_r0 = [[0 for x in range(64)] for y in range(529)]
    raw_can_buffer_r1 = [[0 for x in range(64)] for y in range(529)]
    # set counter of first message to 11 for false-positive prevention
    raw_can_buffer_r0[0][0] = 1
    raw_can_buffer_r0[0][1] = 1
    raw_can_buffer_r1[0][0] = 1
    raw_can_buffer_r1[0][1] = 1

    while(True):
        # init vars

        message = bus.recv()
        #time.sleep(0.001)
        # call parser function
        try:
            if message.arbitration_id == can_id_ranDopMat_radar0:
                (is_first_zero_flag_r0, catched_previous_zero_flag_r0, catched_previous_zero_msg_r0, is_first_one_flag_r0, catched_previous_one_flag_r0, catched_previous_one_msg_r0, catched_previous_527_flag_r0, catched_previous_527_msg_r0, success_oneround_r0) = parse_message(message, 0,
                                                                                                                                                                                                                                                                                    is_first_zero_flag_r0, catched_previous_zero_flag_r0, catched_previous_zero_msg_r0, is_first_one_flag_r0, catched_previous_one_flag_r0, catched_previous_one_msg_r0, catched_previous_527_flag_r0, catched_previous_527_msg_r0, success_oneround_r0, range_doppler_matrix_r0, raw_can_buffer_r0, rd_q1, rd_q2)
            elif message.arbitration_id == can_id_ranDopMat_radar1:
                (is_first_zero_flag_r1, catched_previous_zero_flag_r1, catched_previous_zero_msg_r1, is_first_one_flag_r1, catched_previous_one_flag_r1, catched_previous_one_msg_r1, catched_previous_527_flag_r1, catched_previous_527_msg_r1, success_oneround_r1) = parse_message(message, 1,
                                                                                                                                                                                                                                                                                    is_first_zero_flag_r1, catched_previous_zero_flag_r1, catched_previous_zero_msg_r1, is_first_one_flag_r1, catched_previous_one_flag_r1, catched_previous_one_msg_r1, catched_previous_527_flag_r1, catched_previous_527_msg_r1, success_oneround_r1, range_doppler_matrix_r1, raw_can_buffer_r1, rd_q1, rd_q2)
            else:
                pass  # unkown id
        except Exception as e:
            print(e)

class hwPWMProgram:
    def __init__(self):
        self.running = True
        self.GPIO = pigpio.pi()
        self.PWM_0 = 12  # Physical Pin #12 (PWM0)
        self.PWM_1 = 13  # Physical Pin #33 (PWM1)
        #self.GPIO.set_pull_up_down(self.PWM_0, pigpio.PUD_DOWN)
        #self.GPIO.set_pull_up_down(self.PWM_1, pigpio.PUD_DOWN)
        self.pwm = None

    def terminate(self):
        """# Pull down the GPIO-Pin and cleanup with stop()
        try:
            self.GPIO.write(self.PWM_0, 0)
        except:
            pass
        time.sleep(0.1)
        try:
            self.GPIO.write(self.PWM_1, 0)
        except:
            pass
        time.sleep(0.1)
        self.GPIO.stop()"""
        #if self.pwm is not None:
        self.pwm.cancel()
        #if self.GPIO is not None:
        self.GPIO.write(self.PWM_0, 0)
        self.GPIO.write(self.PWM_1, 0)
        self.GPIO.stop()
        self.running = False

    def run(self):

        #
        # pigpio uses BROADCOM PIN NUMBERING !!!
        #

        logging.info("Starting the HW PWM task.. ")
        #PWM_0 = 12#18  # Physical Pin #12 (PWM0)
        #PWM_1 = 13  # Physical Pin #33 (PWM1)
        """
        # Set the GPIO-Mode to ALT5 for HW-PWM
        self.GPIO.set_mode(self.PWM_0, pigpio.ALT0)#self.GPIO.set_mode(PWM_0, pigpio.ALT5)
        self.GPIO.set_mode(self.PWM_1, pigpio.ALT0)

        # Start the signal generation
        # 33 Hz, 50% duty cycle
        # 2 Hz
        #time.sleep(2.0)
        self.GPIO.hardware_PWM(self.PWM_0, 1, 500000)
        time.sleep(0.55)  # (shifted by half period time of 34 Hz)
        self.GPIO.hardware_PWM(self.PWM_1, 1, 500000)
        """
        
        self.pwm = wavePWM.PWM(self.GPIO)

        self.pwm.set_frequency(1)
        self.pwm.set_pulse_start_in_micros(self.PWM_0, 0) # 0 delay
        self.pwm.set_pulse_start_in_micros(self.PWM_1, 500000) # 500ms delay
        self.pwm.set_pulse_length_in_micros(self.PWM_0, 500000) #500ms pulse length
        self.pwm.set_pulse_length_in_micros(self.PWM_1, 500000) #500ms pulse length
        self.pwm.update() #start
        
        try:
            # Keep the script running until Ctrl + C are pressed
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("Terminate the HW PWM task.. ")
            self.terminate()
            pass

thread_objects = []
thread_ids = []

def signalHandler(sigNum, frame):
    global thread_objects
    for obj in thread_objects:
        try:
            obj.terminate()
            print("terminated Obj")
        except Exception as e:
            print("could not terminate Obj")
            print("Error :"+str(e) + traceback.format_exc())
            pass
    for thread in thread_ids:
        try:
            thread.join(1.0)
            print("joined thread")
        except Exception as e:
            print("could not join thread")
            print("Error :"+str(e) + traceback.format_exc())
            pass
    #sys.exit(0)

def main():
    global thread_objects
    global thread_ids
    signal.signal(signal.SIGTERM, signalHandler)
    q_rdMatrix_r1 = queue.LifoQueue()
    q_rdMatrix_r2 = queue.LifoQueue()
    #thread_ids = []

    # read command-line parameters
    useVCAN = False
    if(len(sys.argv) > 3):
        logging.info(f"Len:{len(sys.argv)}, value:{sys.argv[1]}")
        useVCAN = bool(int(sys.argv[1]))
        imageExportModeOn = bool(int(sys.argv[2]))
        imageFoldername = str(sys.argv[3])
    # connect to CAN
    connectToCAN(useVCAN)
    #Starting HW-PWM Thread for Synchronization of the 2 radars
    logging.info("Create the hw sync pulse task..")
    hwPWMRef = hwPWMProgram()
    hwPWMThread = Thread(target=hwPWMRef.run)
    hwPWMThread.daemon = False
    hwPWMThread.start()
    thread_ids.append(hwPWMThread)
    thread_objects.append(hwPWMRef)
    # Starting Range-Doppler-Matrix Parser Thread
    canRDParseRef = CAN_ParseRangeDoppler_Program(q_rdMatrix_r1, q_rdMatrix_r2)
    canRDParseThread = Thread(target=canRDParseRef.run)
    canRDParseThread.daemon = True
    canRDParseThread.start()
    thread_ids.append(canRDParseThread)
    thread_objects.append(canRDParseRef)
    
    # collect_message()
    # Running the Plot-Animation
    logging.info("Start plotting the radar data..")
    rdPlotRef = PLOT.RDPlot(q_rdMatrix_r1, q_rdMatrix_r2, imageExportModeOn, imageFoldername)
    thread_objects.append(rdPlotRef)
    rdPlotRef.startAnimation()


if __name__ == '__main__':
    main()
