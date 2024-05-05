#!/usr/bin/env python

"""
Author: Steven Duong
Date: 01.05.2020
Description: Use this script, to bitwise request the UID of two radar modules. The UID will be then compared to find the UID with the higher/lower UID.
"""

##############
#### libs ####
##############

import can      # CAN (python-can 3.0)
import time
import struct   # to build structs, s = char byte, h/H signed/unsigned 2 Byte  short, i/I signed/unsigned 4 Byte integer
import binascii  # to decode a ascii stream to hex values

import _thread
from threading import Thread
import queue
import numpy as np

import logging
import sys

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
user_log_level = logging.INFO

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
#CAN_ID_SENDCONF = 0x000000A1
CAN_ID_SENDCONF = 0xA1

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

#########################
#### LOCAL FUNCTIONS ####
#########################


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


def gen_msg_can_id_req(position):
    """
        Generate the byte array for the UID request at a given position.

        Parameter:
            position     bit position of the requested UID
    """

    CAN_GEN_MSG = []

    # Append the body of the request byte array
    CAN_GEN_MSG.append(0x43)
    CAN_GEN_MSG.append(0x41)
    CAN_GEN_MSG.append(0x4E)
    CAN_GEN_MSG.append(0x52)
    CAN_GEN_MSG.append(0x45)
    CAN_GEN_MSG.append(0x51)
    CAN_GEN_MSG.append(0x45)
    CAN_GEN_MSG.append(0x58)
    CAN_GEN_MSG.append(0x55)
    CAN_GEN_MSG.append(0x49)
    CAN_GEN_MSG.append(0x44)
    CAN_GEN_MSG.append(0x20)

    asciiHexArray = []

    while(int(position / 10) > 0):
        asciiHexArray.append((position % 10) + 0x30)
        position = int(position/10)

    asciiHexArray.append((position % 10) + 0x30)
    asciiHexArray = asciiHexArray[::-1]

    for item in asciiHexArray:
        CAN_GEN_MSG.append(item)

    return CAN_GEN_MSG


def find_winner():
    """
    Function, which bitwise request the UID of two radar modules. If the function receives to different CAN-IDs, the radar module with the lower UID has been detected.
    Then a CAN msg with the content "CANENDREQEX" will be sent to the radar modules.
    """

    global bus

    logging.info("CAN bus communication during finding the winner ID: ")

    # Byte array for signaling the end of the comparison
    MSG_REQ_END = [0x43, 0x41, 0x4e, 0x45,
                   0x4e, 0x44, 0x45, 0x58, 0x55, 0x49, 0x44]

    # # Generate the CAN msg for signaling the end of the comparison
    CAN_DATA_SEND_REQ_END_MSG = can.Message(
        arbitration_id=CAN_ID_SENDCONF, is_fd=use_fd, is_extended_id=use_ext_ID, data=MSG_REQ_END)

    logging.info("______________________________________")

    # Find the winner ID
    for count in range(CAN_REQ_ID_START, CAN_REQ_ID_END):
        # Generate the CAN msg for the bitwise request
        logging.info(count)
        time.sleep(0.1)
        CAN_DATA_SEND_REQ_UID = can.Message(arbitration_id=CAN_ID_SENDCONF, is_fd=use_fd,
                                            is_extended_id=use_ext_ID, data=gen_msg_can_id_req(count))
        logging.info(str(CAN_DATA_SEND_REQ_UID))
        bus.send(CAN_DATA_SEND_REQ_UID)

        # Receive the answer of the radar modules
        msg_uid0 = bus.recv()
        logging.info(str(msg_uid0))
        # error prevention, if two CAN msgs with the same content and ID was received
        msg_uid1 = bus.recv(CAN_REQ_TIMEOUT)
        logging.info(str(msg_uid1))

        logging.info(msg_uid0.arbitration_id)
        logging.info(msg_uid1.arbitration_id)

        if msg_uid1 is None or msg_uid0 is None:
            continue
        else:
            # If the IDs of the received msgs are the same, continue
            if msg_uid0.arbitration_id == msg_uid1.arbitration_id:
                continue
            # The CAN-ID differs, so the winner was found :)
            else:
                logging.info(str(CAN_DATA_SEND_REQ_END_MSG))
                bus.send(CAN_DATA_SEND_REQ_END_MSG)
                return count

    return None


def configure_radar():
    """
    Call this function to send the command for advFramCfg to the radar, followed by sensorStart  command.
    """
    global bus
    global CAN_ID_SENDCONF

    # CAN configuration msgs for radar
    DATA_SENDCONF_advFrameCfg = [
        0x61, 0x64, 0x76, 0x46, 0x72, 0x61, 0x6d, 0x65, 0x43, 0x66, 0x67]
    DATA_SENDCONF_sensorStart = [
        0x73, 0x65, 0x6e, 0x73, 0x6f, 0x72, 0x53, 0x74, 0x61, 0x72, 0x74]

#   First Param: 0..SW-Sync 1..HW-Sync  Second Param: Frame-Delay 0..20000 where 1lsb = 5ns
    if hw_trigger_on == True:
        DATA_SENDCONF_FrmSync = [0x46, 0x72, 0x6d, 0x53, 0x79, 0x6e,
                                 0x63, 0x20, 0x31, 0x20, 0x30]  # outsource to function?

    else:
        DATA_SENDCONF_FrmSync = [0x46, 0x72, 0x6d, 0x53, 0x79, 0x6e,
                                 0x63, 0x20, 0x30, 0x20, 0x30]  # outsource to function?

    # Pack CAN-msg with  data
    CAN_DATA_SENDCONF_advFrameCfg = can.Message(arbitration_id=CAN_ID_SENDCONF, is_fd=use_fd,
                                                is_extended_id=use_ext_ID, data=DATA_SENDCONF_advFrameCfg)
    CAN_DATA_SENDCONF_sensorStart = can.Message(arbitration_id=CAN_ID_SENDCONF, is_fd=use_fd,
                                                is_extended_id=use_ext_ID, data=DATA_SENDCONF_sensorStart)
    CAN_DATA_SENDCONF_FrmSync = can.Message(arbitration_id=CAN_ID_SENDCONF, is_fd=use_fd,
                                            is_extended_id=use_ext_ID, data=DATA_SENDCONF_FrmSync)
    if send_timestamps == True:
        DATA_SENDCONF_TimeStamps = [
            0x43, 0x68, 0x69, 0x72, 0x70, 0x54, 0x69, 0x6d, 0x65, 0x73, 0x74, 0x61, 0x6d, 0x70]
        CAN_DATA_SENDCONF_TimeStamps = can.Message(
            arbitration_id=CAN_ID_SENDCONF, is_fd=use_fd, is_extended_id=use_ext_ID, data=DATA_SENDCONF_TimeStamps)
        bus.send(CAN_DATA_SENDCONF_TimeStamps)
    else:
        pass

    # Find winner ID
    if (use_multi_radar == True):
        try:
            logging.info(
                "Multisensor setup detected. Try to find the smaller/bigger UID.")
            win_id = find_winner()
            logging.info("Winner ID found at: ")
            logging.info(str(win_id))
        except Exception as e:
            logging.warning("Error code:")
            logging.warning(e)
            logging.warning("No winner ID found.")
    else:
        pass

    try:
        logging.info("Try to configure the radar over CAN.")
        # Send FrmSync
        bus.send(CAN_DATA_SENDCONF_FrmSync)
        logging.debug(str(CAN_DATA_SENDCONF_FrmSync))
        time.sleep(0.5)

        # Send advFrameCfg
        bus.send(CAN_DATA_SENDCONF_advFrameCfg)
        logging.debug(str(CAN_DATA_SENDCONF_advFrameCfg))
        time.sleep(0.5)

        # Send sensorStart
        bus.send(CAN_DATA_SENDCONF_sensorStart)
        logging.debug(str(CAN_DATA_SENDCONF_sensorStart))
        time.sleep(0.5)

        logging.info("Radar configuration successfully")

    except Exception as e:
        logging.warning("Errorcode: ")
        logging.warning(e)
        logging.warning("Radar configuration NOT successfully")


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
        logging.info("Try to deinit the radar sensor..")
        # bus.send(CAN_DATA_SENDCONF_sensorStop) #Commented for debug purposes
        logging.info("Deinit of radar successfully.")
    except:
        logging.warning("Deinit of radar NOT successfully.")

#########################
#### MAIN FUNCTIONS ####
#########################


def main():
    global bus

    useVCAN = False
    if(len(sys.argv) > 1):
        print(f"CONFIG: Len:{len(sys.argv)}, value:{sys.argv[1]}")
        useVCAN = bool(int(sys.argv[1]))
        print("CONFIG: Use VCAN: " + str(useVCAN))

    # Buffer to store the thread IDs
    thread_ids = []

    logging.info("CONFIG: Starting the main function.. ")

    # Open CAN-interface
    logging.info("CONFIG: Connect to CAN bus.. ")
    connect_to_can_bus(useVCAN)

    # Send the configuration to the radar
    logging.info("CONFIG: Send configuration to radar..")
    configure_radar()


if __name__ == '__main__':
    main()
