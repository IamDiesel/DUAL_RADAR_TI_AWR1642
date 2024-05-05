#!/usr/bin/env python

"""
Author: Steven Duong, Daniel Kahrizi
Date: 26.06.2020
Description: Use this script to receive, parse and plot radar data via vcan0 or can0.
Program Parameters: 
usevcan: 1..use vcan0 0..use can0
xmin, xmax, ymin, ymax: Axis limits for the data plot
"""

##############
#### libs ####
##############

import can      # CAN (python-can 3.0)
import time
import struct   # to build structs, s = char byte, h/H signed/unsigned 2 Byte  short, i/I signed/unsigned 4 Byte integer
import binascii  # to decode a ascii stream to hex values

import pigpio
import Plotter as PLOT

import _thread
from threading import Thread
from queue import Queue
import queue
import numpy as np

import RadarDataStructure as RSTRUCT

import logging
import sys

detPointsQ_r1 = queue.LifoQueue()
detClustersQ_r1 = queue.LifoQueue()
detPointsQ_r2 = queue.LifoQueue()
detClustersQ_r2 = queue.LifoQueue()

####################
#### References ####
####################
"""
https://docs.python.org/3.7/library/threading.html
https://stackoverflow.com/questions/5568555/thread-vs-threading
https://docs.python.org/3.7/library/queue.html

"""
################
#### PARAMS ####
################

# Application parameters
CAN_REQ_ID_START = 0    # Start position for UID comparison check
CAN_REQ_ID_END = 128    # End position
# Time before second bus receive function is terminated without getting a msg
CAN_REQ_TIMEOUT = 1

hw_trigger_on = True
send_timestamps = False
use_multi_radar = True

make_logfile = False
#user_log_level = logging.INFO
user_log_level = logging.CRITICAL

# TODO rename, change to applicant var
epsi = 1

####################
#### CAN-Config ####
####################

bus = 0
bustype = 'socketcan'
channel = 'can0'
channel_backup = 'vcan0'
use_fd = True
use_ext_ID = True

################
#### CAN-ID ####
################

# CAN ID of radar
CAN_ID_SENDCONF = 0x000000A1

CAN_MESSAGE_MMWDEMO_HEADER = 0xC1
CAN_MESSAGE_MMWDEMO_PADDING = 0xB1

CAN_MESSAGE_MMWDEMO_DETECTED_POINTS = 0xD1
CAN_MESSAGE_MMWDEMO_RANGE_PROFILE = 0xD2
CAN_MESSAGE_MMWDEMO_NOISE_PROFILE = 0xD3
CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP = 0xD4
CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP = 0xD5
CAN_MESSAGE_MMWDEMO_STATS = 0xD6
CAN_MESSAGE_MMWDEMO_CLUSTERS = 0xD7
CAN_MESSAGE_MMWDEMO_PARKING_ASSIST = 0xD8
CAN_MESSAGE_MMWDEMO_TRACKED_OBJECTS = 0xD9

CAN_SECOND_RADAR_OFFSET = 0x40

##########################
#### GLOBAL VARIABLES ####
##########################

# logging module
if (make_logfile == True):
    filenamestr = time.strftime("%Y%m%d-%H%M%S")
    filenamestr += "_CAN_COMM.log"
    logging.basicConfig(filename=filenamestr, level=user_log_level)
else:
    logging.basicConfig(level=user_log_level)

logging.info('Application start..')

# Queues for the incoming CAN msg
queue_radar_can_msg_r1 = queue.Queue(maxsize=10000)
queue_radar_can_msg_r2 = queue.Queue(maxsize=10000)

# counter and buffer for processing the incoming TLVs
process_tlv_counter_r1 = 0
process_tlv_counter_r2 = 0
process_tlv_string_r1 = ""
process_tlv_string_r2 = ""

# buffer for the extracted TLV header
tlv_type_r1 = 0
tlv_length_r1 = 0
tlv_type_r2 = 0
tlv_length_r2 = 0

# counter for received bytes from current TLV transmission
counter_recv_byte_r1 = 0
counter_recv_byte_r2 = 0

# number of maximal CAN dlc
if use_fd == True:
    const_can_dlc_max = 64
else:
    const_can_dlc_max = 8

if(send_timestamps == 1):
    check_frame_header_up = 576
    check_frame_header_low = 576

else:
    check_frame_header_up = 65
    check_frame_header_low = 64

# flag, set if frame start was catched successfully
# reset, when transmission error occured
flg_catch_frame_start_r1 = 0
flg_catch_frame_start_r2 = 0

# Mutex is taken, when the processing of a received TLV has been started. So a msg with another CAN-ID must occure only after the processing of the current TLV
mutex_process_tlv_r1 = 1
mutex_process_tlv_r2 = 1

# flag, which is used as a token to signal, that currently this type of tlv is processed
flg_can_process_B1_padding = 0
flg_can_process_C1_frameHeader = 0
flg_can_process_D1_detObj = 0
flg_can_process_D6_stats = 0
flg_can_process_D7_clusters = 0
flg_can_process_D8_parking = 0
flg_can_process_D9_trObj = 0

flg_can_process_71_padding = 0
flg_can_process_81_frameHeader = 0
flg_can_process_91_detObj = 0
flg_can_process_96_stats = 0
flg_can_process_97_clusters = 0
flg_can_process_98_parking = 0
flg_can_process_99_trObj = 0

#########################
#### LOCAL FUNCTIONS ####
#########################


def reset_to_framestart(radar_select):
    """
    This function resets all counter and buffers, which are used for the processing of the incoming TLVs.
    Calling this function will also reset the flg_catch_frame_start, so that the running script will catch
    the start of the next frame header
    """
    if(radar_select == 1):
        global process_tlv_counter_r1
        process_tlv_counter_r1 = 0
        global process_tlv_string_r1
        process_tlv_string_r1 = ""
        global flg_catch_frame_start_r1
        flg_catch_frame_start_r1 = 0
        global mutex_process_tlv_r1
        mutex_process_tlv_r1 = 1
        global flg_can_process_B1_padding
        flg_can_process_B1_padding = 0
        global flg_can_process_C1_frameHeader
        flg_can_process_C1_frameHeader = 0
        global flg_can_process_D1_detObj
        flg_can_process_D1_detObj = 0
        global flg_can_process_D6_stats
        flg_can_process_D6_stats = 0
        global flg_can_process_D7_clusters
        flg_can_process_D7_clusters = 0
        global flg_can_process_D8_parking
        flg_can_process_D8_parking = 0
        global flg_can_process_D9_trObj
        flg_can_process_D9_trObj = 0
        global counter_recv_byte_r1
        counter_recv_byte_r1 = 0
    else:
        global process_tlv_counter_r2
        process_tlv_counter_r2 = 0
        global process_tlv_string_r2
        process_tlv_string_r2 = ""
        global flg_catch_frame_start_r2
        flg_catch_frame_start_r2 = 0
        global mutex_process_tlv_r2
        mutex_process_tlv_r2 = 1
        global flg_can_process_71_padding
        flg_can_process_71_padding = 0
        global flg_can_process_81_frameHeader
        flg_can_process_81_frameHeader = 0
        global flg_can_process_91_detObj
        flg_can_process_91_detObj = 0
        global flg_can_process_96_stats
        flg_can_process_96_stats = 0
        global flg_can_process_97_clusters
        flg_can_process_97_clusters = 0
        global flg_can_process_98_parking
        flg_can_process_98_parking = 0
        global flg_can_process_99_trObj
        flg_can_process_99_trObj = 0
        global counter_recv_byte_r2
        counter_recv_byte_r2 = 0


def reset_to_nextFrame(radar_select):
    """
    This function resets all counter and buffers, which are used for the processing of the incoming TLVs.
    Calling this function will also not reset the flg_catch_frame_start, so that the running script will continue
    to process incoming TLVs.
    """
    if(radar_select == 1):
        global process_tlv_counter_r1
        process_tlv_counter_r1 = 0
        global process_tlv_string_r1
        process_tlv_string_r1 = ""
        global mutex_process_tlv_r1
        mutex_process_tlv_r1 = 1
        global flg_can_process_B1_padding
        flg_can_process_B1_padding = 0
        global flg_can_process_C1_frameHeader
        flg_can_process_C1_frameHeader = 0
        global flg_can_process_D1_detObj
        flg_can_process_D1_detObj = 0
        global flg_can_process_D6_stats
        flg_can_process_D6_stats = 0
        global flg_can_process_D7_clusters
        flg_can_process_D7_clusters = 0
        global flg_can_process_D8_parking
        flg_can_process_D8_parking = 0
        global flg_can_process_D9_trObj
        flg_can_process_D9_trObj = 0
        global counter_recv_byte_r1
        counter_recv_byte_r1 = 0
    else:
        global process_tlv_counter_r2
        process_tlv_counter_r2 = 0
        global process_tlv_string_r2
        process_tlv_string_r2 = ""
        global mutex_process_tlv_r2
        mutex_process_tlv_r2 = 1
        global flg_can_process_71_padding
        flg_can_process_71_padding = 0
        global flg_can_process_81_frameHeader
        flg_can_process_81_frameHeader = 0
        global flg_can_process_91_detObj
        flg_can_process_91_detObj = 0
        global flg_can_process_96_stats
        flg_can_process_96_stats = 0
        global flg_can_process_97_clusters
        flg_can_process_97_clusters = 0
        global flg_can_process_98_parking
        flg_can_process_98_parking = 0
        global flg_can_process_99_trObj
        flg_can_process_99_trObj = 0
        global counter_recv_byte_r2
        counter_recv_byte_r2 = 0


def get_tlv_type_length(msg):
    """
    This function uses a passed CAN msg to extract the information from the TLV header.
    It returns the tlv type and length.
    """

    # Take the CAN msg and put the content in a string stream
    loc_string = ""
    for numMsg in range(0, msg.dlc):
        temp = '{:02x}'.format(msg.data[numMsg])
        loc_string += str(temp)

    # buffer for the extracted data, I = 4Byte unsigned int
    s = struct.Struct('2I')
    # unpack the string stream to integer values
    packed = binascii.unhexlify(loc_string)
    values = s.unpack(packed)

    # assign the return variable with the information from the buffer
    tlv_type = values[0]
    tlv_length = values[1]

    return (tlv_type, tlv_length)


def get_num_detObj_format(data):
    """
    This function uses a passed CAN msg to extract the information from tlv payload "header".
    It returns the number of detected objects (description from TI, better is:detected items)
    and the xyzQFormat.
    """
    # buffer for the extracted data, H = 2Byte unsigned integer
    s = struct.Struct('2H')
    # unpack the string stream to integer values
    packed = binascii.unhexlify(data)
    values = s.unpack(packed)

    # assign the return variable with the information from the buffer
    numDetectedObj2 = values[0]
    xyzQFormat = values[1]

    return (numDetectedObj2, xyzQFormat)


def process_TLV_TYPE_CAN_MESSAGE_MMWDEMO_DETECTED_POINTS(radar_select, msg):
    """
    This function uses a passed CAN msg to extract the information from tlv payload.
    It returns four arrays, with the information of the detected points.
    Precisely, it containts the velocity, x and y position, a peakValue of the power of the received signal from detected objects.
    """

    global tlv_type_r1
    global tlv_length_r1
    global process_tlv_counter_r1
    global process_tlv_string_r1
    global counter_recv_byte_r1
    global mutex_process_tlv_r1
    global flg_can_process_D1_detObj

    global tlv_type_r2
    global tlv_length_r2
    global process_tlv_counter_r2
    global process_tlv_string_r2
    global counter_recv_byte_r2
    global mutex_process_tlv_r2
    global flg_can_process_91_detObj

    if(radar_select == 1):
        loc_process_tlv_counter = process_tlv_counter_r1
        loc_mutex_process_tlv = mutex_process_tlv_r1
        loc_counter_recv_byte = counter_recv_byte_r1
        loc_flg_can_process = flg_can_process_D1_detObj
        loc_tlv_length = tlv_length_r1
    else:
        loc_process_tlv_counter = process_tlv_counter_r2
        loc_mutex_process_tlv = mutex_process_tlv_r2
        loc_counter_recv_byte = counter_recv_byte_r2
        loc_flg_can_process = flg_can_process_91_detObj
        loc_tlv_length = tlv_length_r2

    # buffer for the return values
    obj1_speed = []
    obj1_peakVal = []
    obj1_pos_x = []
    obj1_pos_y = []

    # check if mutex is free and if it is the TLV header
    if loc_process_tlv_counter == 0 and loc_mutex_process_tlv == 1:
        try:
            if(radar_select == 1):
                mutex_process_tlv_r1 = 0
                flg_can_process_D1_detObj = 1
                if(msg.dlc < 9):
                    tlv_type_r1, tlv_length_r1 = get_tlv_type_length(msg)
                else:
                    reset_to_framestart(1)
                process_tlv_counter_r1 = process_tlv_counter_r1 + 1
                loc_mutex_process_tlv = mutex_process_tlv_r1
                loc_flg_can_process = flg_can_process_D1_detObj
                loc_process_tlv_counter = process_tlv_counter_r1
                loc_tlv_length = tlv_length_r1
            else:
                mutex_process_tlv_r2 = 0
                flg_can_process_91_detObj = 1
                if(msg.dlc < 9):
                    tlv_type_r2, tlv_length_r2 = get_tlv_type_length(msg)
                else:
                    reset_to_framestart(2)
                process_tlv_counter_r2 = process_tlv_counter_r2 + 1
                loc_mutex_process_tlv = mutex_process_tlv_r2
                loc_flg_can_process = flg_can_process_91_detObj
                loc_process_tlv_counter = process_tlv_counter_r2
                loc_tlv_length = tlv_length_r2
        except Exception as e:
            # Couldn't find TLV header, find next frame start
            logging.info("Couldn't find TLV header.")
            if(radar_select == 1):
                logging.info("Radar 1 failed points")
                logging.info(e)
                reset_to_framestart(1)
            else:
                logging.info("Radar 2 failed points")
                logging.info(e)
                reset_to_framestart(2)
    else:
        # check if the flag for processing of the msg is set. Now process the payload of the TLV
        if loc_counter_recv_byte <= loc_tlv_length and loc_flg_can_process == 1:
            # Check if the expected byte amount is higher than the maximal dlc length, then check if the current msg length is the same value
            if (loc_tlv_length-loc_counter_recv_byte) >= const_can_dlc_max:
                if msg.dlc != const_can_dlc_max:
                    # logging.debug("points dlc")
                    if(radar_select == 1):
                        reset_to_framestart(1)
                    else:
                        reset_to_framestart(2)
                    return(0, 0, 0, 0)
            # Take the CAN msg and put the content in a string stream
            for numMsg in range(0, msg.dlc):
                temp = '{:02x}'.format(msg.data[numMsg])
                if(radar_select == 1):
                    process_tlv_string_r1 += str(temp)
                    counter_recv_byte_r1 = counter_recv_byte_r1 + 1
                    loc_counter_recv_byte = counter_recv_byte_r1
                else:
                    process_tlv_string_r2 += str(temp)
                    counter_recv_byte_r2 = counter_recv_byte_r2 + 1
                    loc_counter_recv_byte = counter_recv_byte_r2
                # Check if amount of received bytes is equal to TLV length (ignore padding bytes)
                if(loc_counter_recv_byte == loc_tlv_length):
                    counter_recv_byte_r1 = 0
                    counter_recv_byte_r2 = 0
                    # Take the tlv_length and calculate the amount of bytes of each item in the TLV payload
                    if(radar_select == 1):
                        tlv_length_single = tlv_length_r1//2
                    else:
                        tlv_length_single = tlv_length_r2//2
                    tlv_length_single = tlv_length_single - 2
                    tlv_length_single = tlv_length_single//4
                    # Build the string for the struct dynamically, so the size of the TLV can change and be processed
                    st = '2H '
                    st += str(tlv_length_single)
                    st += ('h ')
                    st += str(tlv_length_single)
                    st += ('H ')
                    st += str(tlv_length_single)
                    st += ('h ')
                    st += str(tlv_length_single)
                    st += ('h')

                    s = struct.Struct(st)
                    if(radar_select == 1):
                        packed = binascii.unhexlify(process_tlv_string_r1)
                    else:
                        packed = binascii.unhexlify(process_tlv_string_r2)

                    try:
                        values = s.unpack(packed)
                    except Exception as e:
                        logging.info(e)
                        if(radar_select == 1):
                            logging.debug("Unpacked failed points radar 1")
                            reset_to_framestart(1)
                        else:
                            logging.debug("Unpacked failed points radar 2")
                            reset_to_framestart(2)
                        return

                    # assign values to the TLV sub header
                    dataObjDescr_numDectObj = values[0]
                    dataObjDescr_xyzQFormat = 1/(pow(2, values[1]))

                    # assign the values to the TLV payload variables
                    for numObj in range(0, dataObjDescr_numDectObj):
                        if(len(values) > 2 + numObj*4 + 0):
                            obj1_speed.append(
                                values[2 + numObj*4 + 0]*dataObjDescr_xyzQFormat)
                        if(len(values) > 2 + numObj*4 + 1):
                            obj1_peakVal.append(values[2 + numObj*4 + 1])
                        if(len(values) > 2 + numObj*4 + 2):
                            obj1_pos_x.append(
                                values[2 + numObj*4 + 2]*dataObjDescr_xyzQFormat)
                        if(len(values) > 2 + numObj*4 + 3):
                            obj1_pos_y.append(
                                values[2 + numObj*4 + 3]*dataObjDescr_xyzQFormat)

                    if(radar_select == 1):
                        try:
                            reset_to_nextFrame(1)
                            if(obj1_pos_x is not None):
                                detPointsQ_r1.put(RSTRUCT.DetectedPoints(
                                    obj1_pos_x, obj1_pos_y, obj1_speed, obj1_peakVal), block=False)

                        except queue.Full:
                            logging.info("Queue radar 1 points full!")
                            pass
                    else:
                        try:
                            reset_to_nextFrame(2)
                            if(obj1_pos_x is not None):
                                detPointsQ_r2.put(RSTRUCT.DetectedPoints(
                                    obj1_pos_x, obj1_pos_y, obj1_speed, obj1_peakVal), block=False)
                        except queue.Full:
                            logging.info("Queue radar 2 points full!")
                            pass

                    return (obj1_speed, obj1_peakVal, obj1_pos_x, obj1_pos_y)

    return (0, 0, 0, 0)


def process_TLV_TYPE_CAN_MESSAGE_MMWDEMO_CLUSTERS(radar_select, msg):
    """
    This function uses a passed CAN msg to extract the information from tlv payload.
    It returns four arrays, with the information of the detected/calculated clusters.
    Precisely, it containts the center (x,y) of the cluster, and its size in x/y direction
    """

    if(radar_select == 1):
        logging.debug("Case: CAN_MESSAGE_MMWDEMO_CLUSTERS 1")
    else:
        logging.debug("Case: CAN_MESSAGE_MMWDEMO_CLUSTERS 2")

    global tlv_type_r1
    global tlv_length_r1
    global process_tlv_counter_r1
    global process_tlv_string_r1
    global counter_recv_byte_r1
    global mutex_process_tlv_r1
    global flg_can_process_D7_clusters

    global tlv_type_r2
    global tlv_length_r2
    global process_tlv_counter_r2
    global process_tlv_string_r2
    global counter_recv_byte_r2
    global mutex_process_tlv_r2
    global flg_can_process_97_clusters

    if(radar_select == 1):
        loc_process_tlv_counter = process_tlv_counter_r1
        loc_mutex_process_tlv = mutex_process_tlv_r1
        loc_counter_recv_byte = counter_recv_byte_r1
        loc_flg_can_process = flg_can_process_D7_clusters
        loc_tlv_length = tlv_length_r1
    else:
        loc_process_tlv_counter = process_tlv_counter_r2
        loc_mutex_process_tlv = mutex_process_tlv_r2
        loc_counter_recv_byte = counter_recv_byte_r2
        loc_flg_can_process = flg_can_process_97_clusters
        loc_tlv_length = tlv_length_r2

    # buffer for the return value
    obj_xCenter = []
    obj_yCenter = []
    obj_xSize = []
    obj_ySize = []

    # check if mutex is free and if it is the TLV header
    if loc_process_tlv_counter == 0 and loc_mutex_process_tlv == 1:
        try:
            if(radar_select == 1):
                mutex_process_tlv_r1 = 0
                flg_can_process_D7_clusters = 1
                if(msg.dlc < 9):
                    tlv_type_r1, tlv_length_r1 = get_tlv_type_length(msg)
                else:
                    reset_to_framestart(1)
                process_tlv_counter_r1 = process_tlv_counter_r1 + 1
                loc_mutex_process_tlv = mutex_process_tlv_r1
                loc_flg_can_process = flg_can_process_D7_clusters
                loc_process_tlv_counter = process_tlv_counter_r1
                loc_tlv_length = tlv_length_r1
            else:
                mutex_process_tlv_r2 = 0
                flg_can_process_97_clusters = 1
                if(msg.dlc < 9):
                    tlv_type_r2, tlv_length_r2 = get_tlv_type_length(msg)
                else:
                    reset_to_framestart(2)
                process_tlv_counter_r2 = process_tlv_counter_r2 + 1
                loc_mutex_process_tlv = mutex_process_tlv_r2
                loc_flg_can_process = flg_can_process_97_clusters
                loc_process_tlv_counter = process_tlv_counter_r2
                loc_tlv_length = tlv_length_r2
        except Exception as e:
            # Couldn't find TLV header, find next frame start
            logging.info("Couldn't find TLV header.")
            if(radar_select == 1):
                logging.info("Radar 1 failed cluster")
                logging.info(e)
                reset_to_framestart(1)
            else:
                logging.info("Radar 2 failed cluster")
                logging.info(e)
                reset_to_framestart(2)
    else:
        # check if the flag for processing of the msg is set. Now process the payload of the TLV
        if loc_counter_recv_byte <= loc_tlv_length and loc_flg_can_process == 1:
            # Check if the expected byte amount is higher than the maximal dlc length, then check if the current msg length is the same value
            if (loc_tlv_length-loc_counter_recv_byte) >= const_can_dlc_max:
                if msg.dlc != const_can_dlc_max:
                    # logging.debug("Cluster dlc")
                    if(radar_select == 1):
                        reset_to_framestart(1)
                    else:
                        reset_to_framestart(2)
                    return(0, 0, 0, 0)
            # Take the CAN msg and put the content in a string stream
            for numMsg in range(0, msg.dlc):
                temp = '{:02x}'.format(msg.data[numMsg])
                if(radar_select == 1):
                    process_tlv_string_r1 += str(temp)
                    counter_recv_byte_r1 = counter_recv_byte_r1 + 1
                    loc_counter_recv_byte = counter_recv_byte_r1
                else:
                    process_tlv_string_r2 += str(temp)
                    counter_recv_byte_r2 = counter_recv_byte_r2 + 1
                    loc_counter_recv_byte = counter_recv_byte_r2

                # Check if amount of received bytes is equal to TLV length (ignore padding bytes)
                if(loc_counter_recv_byte == loc_tlv_length):

                    # Take the tlv_length and calculate the amount of bytes of each item in the TLV payload
                    if(radar_select == 1):
                        tlv_length_single = tlv_length_r1//2
                    else:
                        tlv_length_single = tlv_length_r2//2

                    tlv_length_single = tlv_length_single - 2
                    tlv_length_single = tlv_length_single//4

                    # Build the string for the struct dynamically, so the size of the TLV can change and be processed
                    st = '2H '
                    st += str(tlv_length_single)
                    st += ('h ')
                    st += str(tlv_length_single)
                    st += ('h ')
                    st += str(tlv_length_single)
                    st += ('h ')
                    st += str(tlv_length_single)
                    st += ('h')

                    s = struct.Struct(st)
                    if(radar_select == 1):
                        packed = binascii.unhexlify(process_tlv_string_r1)
                    else:
                        packed = binascii.unhexlify(process_tlv_string_r2)
                    try:
                        values = s.unpack(packed)
                    except Exception as e:
                        logging.info(e)
                        if(radar_select == 1):
                            logging.debug("Unpacked failed radar cluster 1")
                            reset_to_framestart(1)
                        else:
                            logging.debug("Unpacked failed radar cluster 2")
                            reset_to_framestart(2)
                        return
                    # assign values to the TLV sub header
                    dataObjDescr_numDectObj = values[0]
                    dataObjDescr_xyzQFormat = 1/(pow(2, values[1]))

                    # assign the values to the TLV payload variables
                    for numObj in range(0, dataObjDescr_numDectObj):
                        if(len(values) > 2 + numObj*4 + 0):
                            obj_xCenter.append(
                                values[2 + numObj*4 + 0]*dataObjDescr_xyzQFormat)
                        if(len(values) > 2 + numObj*4 + 1):
                            obj_yCenter.append(
                                values[2 + numObj*4 + 1]*dataObjDescr_xyzQFormat)
                        if(len(values) > 2 + numObj*4 + 2):
                            obj_xSize.append(
                                values[2 + numObj*4 + 2]*dataObjDescr_xyzQFormat)
                        if(len(values) > 2 + numObj*4 + 3):
                            obj_ySize.append(
                                values[2 + numObj*4 + 3]*dataObjDescr_xyzQFormat)

                    if(radar_select == 1):
                        try:
                            reset_to_nextFrame(1)
                            detClustersQ_r1.put(RSTRUCT.DetectedClusters(
                                obj_xCenter, obj_yCenter, obj_xSize, obj_ySize), block=False)
                        except queue.Full:
                            logging.info("Queue radar 1 clusters full!")
                            pass
                    else:
                        try:
                            reset_to_nextFrame(2)
                            detClustersQ_r2.put(RSTRUCT.DetectedClusters(
                                obj_xCenter, obj_yCenter, obj_xSize, obj_ySize), block=False)
                        except queue.Full:
                            logging.info("Queue radar 2points full!")
                            pass
                    return (obj_xCenter, obj_yCenter, obj_xSize, obj_ySize)
    return (0, 0, 0, 0)


def process_TLV_FRAME_HEADER(radar_select, msg):
    """
    This function uses a passed CAN msg to extract the information from frame header.
    """

    logging.debug("Case: CAN_MESSAGE_MMWDEMO_HEADER")
    global process_tlv_counter_r1
    global process_tlv_string_r1
    global flg_catch_frame_start_r1
    global flg_can_process_C1_frameHeader
    global mutex_process_tlv_r1
    global counter_recv_byte_r1

    global process_tlv_counter_r2
    global process_tlv_string_r2
    global flg_catch_frame_start_r2
    global flg_can_process_81_frameHeader
    global mutex_process_tlv_r2
    global counter_recv_byte_r2

    global check_frame_header_up
    global check_frame_header_low

    if(radar_select == 1):
        loc_mutex_process_tlv = mutex_process_tlv_r1
        loc_counter_recv_byte = counter_recv_byte_r1
        loc_flg_can_process = flg_can_process_C1_frameHeader
    else:
        loc_mutex_process_tlv = mutex_process_tlv_r2
        loc_counter_recv_byte = counter_recv_byte_r2
        loc_flg_can_process = flg_can_process_81_frameHeader

    # Check if mutex is free and if it is the frame header. Then collect the CAN msgs for the frame header.
    # 576 (value with timestamps), 65 (value without timestamps)
    if (loc_counter_recv_byte < check_frame_header_up and loc_mutex_process_tlv == 1) or (loc_counter_recv_byte < check_frame_header_up and loc_flg_can_process):
        if(radar_select == 1):
            mutex_process_tlv_r1 = 0
            flg_can_process_C1_frameHeader = 1
            loc_flg_can_process = flg_can_process_C1_frameHeader
        else:
            mutex_process_tlv_r2 = 0
            flg_can_process_81_frameHeader = 1
            loc_flg_can_process = flg_can_process_81_frameHeader

        # Take the CAN msg and put the content in a string stream
        for numMsg in range(0, msg.dlc):
            dat = '{:02x}'.format(msg.data[numMsg])
            if(radar_select == 1):
                process_tlv_string_r1 += str(dat)
            else:
                process_tlv_string_r2 += str(dat)

        if(radar_select == 1):
            counter_recv_byte_r1 = counter_recv_byte_r1 + msg.dlc
            process_tlv_counter_r1 = process_tlv_counter_r1 + 1
            loc_counter_recv_byte = counter_recv_byte_r1
        else:
            counter_recv_byte_r2 = counter_recv_byte_r2 + msg.dlc
            process_tlv_counter_r2 = process_tlv_counter_r2 + 1
            loc_counter_recv_byte = counter_recv_byte_r2

    # Receive of the frame header byte stream is finished. Now process the frame header.
    if loc_counter_recv_byte == check_frame_header_low and loc_flg_can_process:
        # 8 chars, 7*4Byte unsigned integer, 8 chars, 65*4Byte unsigned integer
        if send_timestamps == True:
            s = struct.Struct('8s 4I Q 2I 8s 66Q')
        else:
            s = struct.Struct('8s 4I Q 2I 8s 2Q')
        if(radar_select == 1):
            packed = binascii.unhexlify(process_tlv_string_r1)
        else:
            packed = binascii.unhexlify(process_tlv_string_r2)
        try:
            values = s.unpack(packed)
        except Exception as e:
            logging.info(e)
            if(radar_select == 1):
                logging.info("Unpacked failed header radar 1")
                reset_to_framestart(1)
            else:
                logging.info("Unpacked failed header radar 2")
                reset_to_framestart(2)
            return

        # buffer for the return value, assign the values direct to the buffer
        magicWord = values[0]
        version = values[1]
        totalPacketLen = values[2]
        platform = values[3]
        frameNumber = values[4]
        timeCPUCycles = values[5]
        numDetectedObj = values[6]
        numTLVs = values[7]
        magicWordTrailer = values[8]
        frameTimestamp = values[9]

        if(send_timestamps == True):
            chirpTimestampArr = values[10:74]
        else:
            chirpTimestampArr = []

        if(radar_select == 1):
            flg_catch_frame_start_r1 = 1
            reset_to_nextFrame(1)
        else:
            flg_catch_frame_start_r2 = 1
            reset_to_nextFrame(2)

        return (magicWord, version, totalPacketLen, platform, frameNumber, timeCPUCycles, numDetectedObj, numTLVs, magicWordTrailer, frameTimestamp, chirpTimestampArr)

    return (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


def recv_check_CAN_ID(radar_select, msg):
    """
    This function checks the CAN-ID and decide which sub function to call.
    To prevent errors, all messages will be ignored until a frame header has been detected and processed successfully.
    If any transmission error occures during the processing of a TLV, the current process will be stopped and reset.
    """
    # FrameHeader

    global flg_catch_frame_start_r1
    global counter_recv_byte_r1
    global process_tlv_counter_r1
    global process_tlv_string_r1

    global flg_catch_frame_start_r2
    global counter_recv_byte_r2
    global process_tlv_counter_r2
    global process_tlv_string_r2

    if(radar_select == 1):

        # Throw all messages until first frame header was catched successfully
        if flg_catch_frame_start_r1 == 0:
            logging.debug("Trying to catch frame start radar1..")
            if (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_HEADER):
                ret_frameHeader = process_TLV_FRAME_HEADER(1, msg)
            else:
                # If the programm counter lands here, the received frame was incomplete, throw it and reset the processing variables
                counter_recv_byte_r1 = 0
                process_tlv_string_r1 = ""
                process_tlv_counter_r1 = 0

        else:
            # Now, we have the first frame header (synced)
            # The order of the cases is sorted by the probability of their appearence
            # Case 0xC1: Frame header of TLV
            if (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_HEADER):
                ret_frameHeader_r1 = process_TLV_FRAME_HEADER(1, msg)

            # Case 0xD1: TLV header and payload of detected points
            elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_DETECTED_POINTS):
                ret_detected_points_r1 = process_TLV_TYPE_CAN_MESSAGE_MMWDEMO_DETECTED_POINTS(
                    1, msg)

            # Case 0xD7: TLV header and payload of processed clusters
            elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_CLUSTERS):
                ret_clusters_r1 = process_TLV_TYPE_CAN_MESSAGE_MMWDEMO_CLUSTERS(
                    1, msg)

            # Case 0xB1: just padding bytes
            elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_PADDING):
                # logging.debug("Received padding bytes - throw packet")
                pass

            # Default case: not defined, so do nothing.
            else:
                pass
                # logging.debug("Unused CAN-ID")
    else:
        # Throw all messages until first frame header was catched successfully
        if flg_catch_frame_start_r2 == 0:
            logging.debug("Trying to catch frame start radar2..")
            if (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_HEADER-CAN_SECOND_RADAR_OFFSET):
                ret_frameHeader = process_TLV_FRAME_HEADER(2, msg)
            else:
                # If the programm counter lands here, the received frame was incomplete, throw it and reset the processing variables
                counter_recv_byte_r2 = 0
                process_tlv_string_r2 = ""
                process_tlv_counter_r2 = 0

        else:
            # Now, we have the first frame header (synced)
            # The order of the cases is sorted by the probability of their appearence
            # Case 0xC1: Frame header of TLV
            if (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_HEADER-CAN_SECOND_RADAR_OFFSET):
                ret_frameHeader = process_TLV_FRAME_HEADER(2, msg)

            # Case 0xD1: TLV header and payload of detected points
            elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_DETECTED_POINTS-CAN_SECOND_RADAR_OFFSET):
                ret_detected_points = process_TLV_TYPE_CAN_MESSAGE_MMWDEMO_DETECTED_POINTS(
                    2, msg)

            # Case 0xD7: TLV header and payload of processed clusters
            elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_CLUSTERS-CAN_SECOND_RADAR_OFFSET):
                ret_clusters = process_TLV_TYPE_CAN_MESSAGE_MMWDEMO_CLUSTERS(
                    2, msg)

            # Case 0xB1: just padding bytes
            elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_PADDING-CAN_SECOND_RADAR_OFFSET):
                # logging.debug("Received padding bytes - throw packet")
                pass

            # Default case: not defined, so do nothing.
            else:
                # logging.debug("Unused CAN-ID.")
                pass


class CANProgram:
    def __init__(self, radar_select):
        self.running = True
        self.radar_sel = radar_select

    def terminate(self):
        self.running = False
        logging.info("canprog shut down")

    def run(self):
        # main processing loop
        try:
            logging.info("Now entering the main processing loop")
            logging.debug("Printing all CAN msgs\n")

            while self.running:

                if(self.radar_sel == 1):
                    # Receive CAN message from buffer
                    try:
                        message_r1 = queue_radar_can_msg_r1.get()
                    except queue.Empty:
                        pass
                    else:
                        # Process the received message
                        logging.debug(str(message_r1))
                        recv_check_CAN_ID(1, message_r1)

                else:
                    # Receive CAN message from buffer
                    try:
                        message_r2 = queue_radar_can_msg_r2.get()
                    except queue.Empty:
                        pass
                    else:
                        # Process the received message
                        logging.debug(str(message_r2))
                        recv_check_CAN_ID(2, message_r2)

                # Das Pferd ein wenig zügeln
                # time.sleep(0.001)
        except Exception as e:
            logging.info(e)

            # End of application, deinitialize radar (sensorStop)
            radar_deinit()
            if(self.radar_sel == 1):
                logging.info("End application 1.. ")
            else:
                logging.info("End application 2.. ")


def radar_deinit():
    """
    Call this function to send the command for sensorStop to the radar
    """
    global bus
    global CAN_ID_SENDCONF

    DATA_SENDCONF_sensorStop = [0x73, 0x65,
                                0x6e, 0x73, 0x6f, 0x72, 0x53, 0x74, 0x6f, 0x70]
    CAN_DATA_SENDCONF_sensorStop = can.Message(arbitration_id=CAN_ID_SENDCONF, is_fd=use_fd,
                                               is_extended_id=use_ext_ID, data=DATA_SENDCONF_sensorStop)

    try:
        logging.info("Try to deinit radar sensor..\n")
        # bus.send(CAN_DATA_SENDCONF_sensorStop) #Commented for debug purposes
        logging.info("Deinit of radar successfully.\n")
    except:
        logging.info("Deinit of radar NOT successfully.\n")


def connect_to_can_bus(useVCAN):
    """
    Call this function to connect to the CAN-bus. First, it tries to connect to can0.
    If this was not successfully, then it try to connect to vcan0.
    """

    global bus
    if(useVCAN):
        locChannel = 'vcan0'
    else:
        locChannel = 'can0'
    try:
        logging.info("Try to connect to can0.. ")
        bus = can.interface.Bus(channel=locChannel, bustype=bustype, fd=use_fd)
        logging.info("Connected to can0 successfully. \n")
    except:
        logging.info("Couldn't connect to can0, trying vcan0..")
        try:
            bus = can.interface.Bus(channel=channel_backup,
                                    bustype=bustype, fd=use_fd)
            logging.info("Connected to vcan0 successfully.")
        except:
            logging.critical("Couldn't connect to any CAN bus.. ERROR.")
            exit()


class hwPWMProgram:
    def __init__(self):
        self.running = True
        self.GPIO = pigpio.pi()

    def terminate(self):
        # Pull down the GPIO-Pin and cleanup with stop()

        PWM_0 = 18  # Physical Pin #12 (PWM0)
        PWM_1 = 13  # Physical Pin #33 (PWM1)
        self.GPIO.write(PWM_0, 0)
        self.GPIO.write(PWM_1, 0)
        self.GPIO.stop()
        self.running = False

    def run(self):

        #
        # pigpio uses BROADCOM PIN NUMBERING !!!
        #

        logging.info("Starting the HW PWM task.. ")
        PWM_0 = 18  # Physical Pin #12 (PWM0)
        PWM_1 = 13  # Physical Pin #33 (PWM1)

        # Set the GPIO-Mode to ALT5 for HW-PWM
        self.GPIO.set_mode(PWM_0, pigpio.ALT5)
        self.GPIO.set_mode(PWM_1, pigpio.ALT0)

        # Start the signal generation
        # 33 Hz, 50% duty cycle
        self.GPIO.hardware_PWM(PWM_0, 34, 500000)
        time.sleep(0.014705882)  # (shifted by half period time of 34 Hz)
        self.GPIO.hardware_PWM(PWM_1, 34, 500000)

        try:
            # Keep the script running until Ctrl + C are pressed
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("Terminate the HW PWM task.. ")
            pass


def fill_queue(msg):
    """
    Take a CAN msg and sort it in the right queue (high or low ID)
    """

    global queue_radar_can_msg_r1
    global queue_radar_can_msg_r2

    # Case 0xC1: Frame header of TLV
    if (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_HEADER):
        try:
            queue_radar_can_msg_r1.put(msg)
            return
        except queue.Full:
            logging.info("Radar 1 CAN queue full")
            pass
    elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_HEADER-CAN_SECOND_RADAR_OFFSET):
        try:
            queue_radar_can_msg_r2.put(msg)
            return
        except queue.Full:
            logging.info("Radar 2 CAN queue full")
            pass

    # Case 0xD1: TLV header and payload of detected points
    elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_DETECTED_POINTS):
        try:
            queue_radar_can_msg_r1.put(msg)
            return
        except queue.Full:
            logging.info("Radar 1 CAN queue full")
            pass

    elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_DETECTED_POINTS - CAN_SECOND_RADAR_OFFSET):
        try:
            queue_radar_can_msg_r2.put(msg)
            return
        except queue.Full:
            logging.info("Radar 2 CAN queue full")
            pass

    # Case 0xD7: TLV header and payload of processed clusters
    elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_CLUSTERS):
        queue_radar_can_msg_r1.put(msg)
        try:
            queue_radar_can_msg_r1.put(msg)
            return
        except queue.Full:
            logging.info("Radar 1 CAN queue full")
            pass

    elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_CLUSTERS-CAN_SECOND_RADAR_OFFSET):
        try:
            queue_radar_can_msg_r2.put(msg)
            return
        except queue.Full:
            logging.info("Radar 2 CAN queue full")
            pass

        # Case 0xB1: just padding bytes
    elif (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_PADDING) | (msg.arbitration_id == CAN_MESSAGE_MMWDEMO_PADDING - CAN_SECOND_RADAR_OFFSET):
        return

    # Default case: not defined, so do nothing.
    else:
        logging.debug("Unkown CAN-ID.")
        pass


class CAN_listener:
    """
    Listens to the CAN bus and take the incoming messages from the radar to fill the queues.
    """

    def __init__(self):
        self.running = True

    def terminate(self):
        logging.info("CAN_listener shutdown")
        self.running = False

    def run(self):
        logging.info("Starting the CAN listener task.")
        logging.debug("Printing all incoming CAN msgs.")

        while(True):
            message = bus.recv()
            logging.debug(str(message))
            fill_queue(message)


#########################
#### MAIN FUNCTIONS ####
#########################


def main():
    global bus

    useVCAN = False
    xmin = -10
    xmax = 10
    ymin = 0
    ymax = 10
    loggingActive = False
    loggingFilename = "measurement"
    refX = 0
    refY = 1
    refRange = 0.3
    plotPeak = True
    plotCOG = True
    plotCOG = True
    plotComb = True
    plotBins = True
    epsCOG = 0.3
    epsPeak = 0.3
    epsCombined = 0.4
    refreshInterval = 500
    if(len(sys.argv) > 1):
        logging.info(
            f"Len:{len(sys.argv)}, value:{sys.argv[1]} xmin:{sys.argv[2]} xmax:{sys.argv[3]} ymin:{sys.argv[4]} ymax: {sys.argv[5]}  loggingActive: {sys.argv[6]}  logFilename: {sys.argv[7]}  refX: {sys.argv[8]}  refY: {sys.argv[9]} refRange: {sys.argv[10]}")
        useVCAN = bool(int(sys.argv[1]))
        xmin = float(float(sys.argv[2]))
        xmax = float(float(sys.argv[3]))
        ymin = float(float(sys.argv[4]))
        ymax = float(float(sys.argv[5]))
        loggingActive = bool(int(sys.argv[6]))
        loggingFilename = sys.argv[7]
        refX = float(sys.argv[8])
        refY = float(sys.argv[9])
        refRange = float(sys.argv[10])
        plotPeak = bool(int(sys.argv[11]))
        plotCOG = bool(int(sys.argv[12]))
        plotComb = bool(int(sys.argv[13]))
        plotBins = bool(int(sys.argv[14]))
        epsCOG = float(sys.argv[15])
        epsPeak = float(sys.argv[16])
        epsCombined = float(sys.argv[17])
        refreshInterval = int(sys.argv[18])
        logging.info("Use VCAN: " + str(useVCAN))
        print("plotBinsInterface"+str(plotBins))
        print("Refresh Interval:" + str(refreshInterval))
    # Buffer to store the thread IDs
    thread_ids = []

    logging.info("Starting the main function.. ")

    # Open CAN-interface
    logging.info("Connect to CAN bus.. ")
    connect_to_can_bus(useVCAN)

    # Start HW PWM module in a thread
    if(hw_trigger_on == True):
        logging.info("Create the hw sync pulse task..")
        hwPWMRef = hwPWMProgram()
        hwPWMThread = Thread(target=hwPWMRef.run)
        hwPWMThread.start()
        thread_ids.append(hwPWMThread)

    # Start can listener in a thread
    logging.info("Create the can listener thread.. ")
    canListenerRef = CAN_listener()
    canListenerThread = Thread(target=canListenerRef.run)
    canListenerThread.start()
    thread_ids.append(canListenerThread)
    time.sleep(0.5)

    # Start parse process task for radar 0
    logging.info("Create the can processing task for the first radar..")
    canRef0 = CANProgram(radar_select=1)
    canThread0 = Thread(target=canRef0.run)
    canThread0.start()
    thread_ids.append(canThread0)
    time.sleep(0.5)

    # Start parse process task for radar 1
    if(use_multi_radar == True):
        logging.info("Create the can processing task for the second radar..")
        canRef1 = CANProgram(radar_select=2)
        canThread1 = Thread(target=canRef1.run)
        canThread1.start()
        thread_ids.append(canThread1)

 # Plot Extension
    logging.info("Start plotting the radar data..")
    radarDataPlot = PLOT.RadarPlot(xmin, xmax, ymin, ymax, detPointsQ_r1, detClustersQ_r1, detPointsQ_r2, detClustersQ_r2,
                                   loggingFilename, loggingActive, refX, refY, refRange, plotPeak, plotCOG, plotComb, plotBins, epsCOG, epsPeak, epsCombined)
    radarDataPlot.startAnimation(intervalMS=refreshInterval)


if __name__ == '__main__':
    main()
