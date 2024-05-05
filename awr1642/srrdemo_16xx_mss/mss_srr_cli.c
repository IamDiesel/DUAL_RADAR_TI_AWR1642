/*
 *   @file  mss_srr_cli.c
 *
 *   @brief
 *      MSS Minimal CLI Implementation for the SRR TI Design
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

/* Application Include Files: */
#include "mss_srr.h"
#include "../common/srr_config_consts.h"

/* HSE CLI INCLUDE*/
/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32

/**************************************************************************
 *************************** Local Functions ******************************
 **************************************************************************/
static int32_t SRR_MSS_CLISensorStart(int32_t argc, char* argv[]);
static int32_t SRR_MSS_CLISensorStop(int32_t argc, char* argv[]);
static int32_t SRR_MSS_CLIBasicCfg(int32_t argc, char* argv[]);
static int32_t SRR_MSS_CLIAdvancedFrameCfg(int32_t argc, char* argv[]);

/*HSE Proof of Concept Function */
static int32_t hiRadar(int32_t argc, char* argv[]);
/*HSE CAN TEST FUNCTION */
static int32_t sendTestCANMessage(int32_t argc, char* argv[]);

//HSE CAN CONFIGURE CAN ID FUNCTION
static int32_t getUIDBit(int32_t argc, char* argv[]);
static int32_t endUIDExchange(int32_t argc, char* argv[]);

/*HSE CAN TEST FUNCTION*/
static int32_t sendTestCANMessage(int32_t argc, char* argv[]);

/*HSE help function for optional selection of timestamps, configuration over CAN/CLI*/
static int32_t setSendChirpTimestamps(int32_t argc, char * argv[]);

/**************************************************************************
 ****************************** CLI Functions *****************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for starting the sensor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
/*HSE Proof of Concept Function*/
static int32_t hiRadar(int32_t argc, char* argv[])
{
    const uint32_t * serialMemPtrA;
    const uint32_t * serialMemPtrB;

    // Address of the radar's unique ID register
    serialMemPtrA = (const uint32_t *) 0xFFFFE200;
    serialMemPtrB = (const uint32_t *) 0xFFFFE20C;

    CLI_write("Chirp, Chirp..\n");
    CLI_write("ADR:  %x::%x::%x::%x\n", &serialMemPtrA[0], &serialMemPtrA[1],
              &serialMemPtrA[2], &serialMemPtrB[0]);
    CLI_write("VAL:  %x::%x::%x::%x\n", serialMemPtrA[0], serialMemPtrA[1],
              serialMemPtrA[2], serialMemPtrB[0]);

    return 0;
}
//HSE CAN CONFIGURE CAN ID FUNCTION
static int32_t getUIDBit(int32_t argc, char* argv[])
{
    uint32_t canID = 0xA4;
    int32_t canError = 0;
    uint16_t tmpBitIndexUID;
    const uint32_t * serialMemPtrA;
    const uint32_t * serialMemPtrB;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    uint16_t bitIndexUID;

    if (argc != 2) //Parameter must be passed to function
    {
        CLI_write("CLI-Error: Amount of parameters must be 1!\n");
        return -1;
    }
    //CLI_write("CLI:argv[0]%s",argv[0]);
    //CLI_write("CLI:argv[1]%s",argv[1]);
    bitIndexUID = atoi(argv[1]); //use cli parameter that was passed
    tmpBitIndexUID = bitIndexUID;

    serialMemPtrA = (const uint32_t *) 0xFFFFE200;
    serialMemPtrB = (const uint32_t *) 0xFFFFE20C;
    CLI_write("Get UID Function was called...\n");
    CLI_write("ADR:  %x::%x::%x::%x\n", &serialMemPtrA[0], &serialMemPtrA[1],
              &serialMemPtrA[2], &serialMemPtrB[0]);
    CLI_write("VAL:  %x::%x::%x::%x\n", serialMemPtrA[0], serialMemPtrA[1],
              serialMemPtrA[2], serialMemPtrB[0]);

    //bitIndexUID globally defined
    if (bitIndexUID < 32)
    {
        serialMemPtrA = (uint32_t *) 0xFFFFE200;
    }
    else if (bitIndexUID >= 32 && bitIndexUID < 64)
    {
        serialMemPtrA = (uint32_t *) 0xFFFFE204;
        tmpBitIndexUID -= 32;
    }
    else if (bitIndexUID >= 64 && bitIndexUID < 96)
    {
        serialMemPtrA = (uint32_t *) 0xFFFFE208;
        tmpBitIndexUID -= 64;
    }
    else if (bitIndexUID >= 96 && bitIndexUID < 128)
    {
        serialMemPtrA = (uint32_t *) 0xFFFFE20C;
        tmpBitIndexUID -= 96;
    }
    else
        CLI_write("Error Bit index too high for UID");

    if ((0x80000000 >> tmpBitIndexUID) & serialMemPtrA[0])
    {
        CLI_write("BIT %d is 1\n", bitIndexUID);
        memset(padding, 0xAF, 1);
        memset(padding + 1, 0xFE, 1);
        gSrrHSEMSSMCB.isLastRequestedBitIndexOne = true;
    }
    else
    {
        CLI_write("BIT %d is 0\n", bitIndexUID);
        canID++;
        memset(padding, 0xD0, 1);
        memset(padding + 1, 0x0F, 1);
        gSrrHSEMSSMCB.isLastRequestedBitIndexOne = false;
    }

    canError = Can_Transmit_Schedule(canID, padding, 2);

    if (canError >= 0)
        CLI_write("Message sent\n");
    else
        CLI_write("Error while sending CAN test message\n");

    //CLI_write(retVal);
    return 0;
}

static int32_t endUIDExchange(int32_t argc, char* argv[])
{
    if (gSrrHSEMSSMCB.isLastRequestedBitIndexOne == false)
    {
        gSrrHSEMSSMCB.useCANIDOffset = true;
        CLI_write("Using Offset for CAN IDs -0x40..\n");
    }
    else
    {
        gSrrHSEMSSMCB.useCANIDOffset = false;
        CLI_write("No Offset for CAN IDs..\n");
    }
    return 0;
}

/*HSE CAN TEST FUNCTION*/
static int32_t sendTestCANMessage(int32_t argc, char* argv[])
{
    uint32_t canID = 0xB1;
    uint32_t numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN;
    int32_t canError = 0;

    CLI_write("Trying to send CAN Message..");
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    for (int i = 0; i < MMWDEMO_OUTPUT_MSG_SEGMENT_LEN; i = i + 2)
    {
        memset(padding + i, 0xAF, 1);
        memset(padding + i + 1, 0xFE, 1);
    }

    canError = Can_Transmit_Schedule(canID, padding, numPaddingBytes);

    if (canError >= 0)
        CLI_write("Message sent.\n");
    else
        CLI_write("Error while sending CAN test message.\n");

    return 0;
}
static int32_t configHWtriggerAndFrameDelay(int32_t argc, char * argv[])
{
    if (argc != 3)
    {
        CLI_write("CLI-Error: Amount of parameters must be 2!\n");
        return -1;
    }

    if (atoi(argv[1]))  //set HW Trigger to value of cli-parameter
        gSrrHSEMSSMCB.useFrameHWTrigger = true;
    else
        gSrrHSEMSSMCB.useFrameHWTrigger = false;

    gSrrHSEMSSMCB.frameTriggerDelay = atoi(argv[2]); //set HW Trigger to value of cli-parameter

    if (gSrrHSEMSSMCB.frameTriggerDelay > 20000) //as defined at mmwavelink.h
    {
        CLI_write(
                "Error: FrameTriggerDelay may not be > 100 us (20000lsb). Delay set to 0.\n");
        gSrrHSEMSSMCB.frameTriggerDelay = 0;
    }

    CLI_write("HW-Trigger:%d\n", gSrrHSEMSSMCB.useFrameHWTrigger);
    CLI_write("Frame Delay:%d\n", gSrrHSEMSSMCB.frameTriggerDelay * 5);

    if (gSrrHSEMSSMCB.useCANIDOffset == false)
        CLI_write(
                "Frame Delay set to 0, since UID is bigger than UID of 2nd radar\n");

    return 0;
}
static int32_t setSendChirpTimestamps(int32_t argc, char * argv[])
{
    MmwDemo_message message;
    int32_t retVal = 0;

    if (argc != 1)
    {
        CLI_write("CLI-Error: No parameters allowed!\n");
        return -1;
    }

    if (gSrrHSEMSSMCB.sendChirptimestamps == false) //set HW Trigger to value of cli-parameter
        gSrrHSEMSSMCB.sendChirptimestamps = true;
    else
        gSrrHSEMSSMCB.sendChirptimestamps = false;

    memset((void *) &message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_HSE_TOGGLE_SEND_CHIRPTIMESTAMPS;

    retVal = MmwDemo_mboxWrite(&message);

    CLI_write("Send Chirp-Timestamps:%d\n", gSrrHSEMSSMCB.sendChirptimestamps);
    CLI_write("Message sent to DSS with status:%d", retVal);

    return 0;
}
static int32_t SRR_MSS_CLISensorStart(int32_t argc, char* argv[])
{
    MMWave_CalibrationCfg calibrationCfg;
    MmwDemo_message message;
    int32_t errCode;

    if (gSrrMSSMCB.runningStatus == true)
    {
        /* Already running. */
        return 0;
    }

    /* The sensor can only be started; if the link has been configured */
    if (gSrrMSSMCB.cfgStatus == true)
    {
        //HSE: Send message to DSS to inform DSS that sensor Start was called. HSE Configuration must be complete at this point.
        memset((void *) &message, 0, sizeof(MmwDemo_message));
        message.type = MMWDEMO_MSS2DSS_HSE_SENSOR_START_CALLED;
        errCode = MmwDemo_mboxWrite(&message);
        CLI_write("Message sent to DSS that sensorStart was called. Status:%d", errCode);
        /* Initialize the calibration configuration: */
        memset((void *) &calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

        /* Populate the calibration configuration: */
        calibrationCfg.dfeDataOutputMode =
                MMWave_DFEDataOutputMode_ADVANCED_FRAME;
        calibrationCfg.u.chirpCalibrationCfg.enableCalibration = true;
        calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity = true;
        calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

        System_printf("Debug: Sensor will start momentarily. \n");

        /* Start the mmWave: */
        if (MMWave_start(gSrrMSSMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
        {
            /* Error: Unable to start the mmWave control module */
            System_printf("Error: mmWave start failed [Error code %d]\n",
                          errCode);
            return -1;
        }

        gSrrMSSMCB.runningStatus = true;

        return 0;
    }
    else
    {
        /* Invalid CLI use case; doing a sensor start without executing the basic or advanced configuration
         * command. Inform the user and return an error code. */
        System_printf(
                "Error: Please ensure that the XXXCfg CLI command is invoked before starting the sensor\n");
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for stopping the sensor
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t SRR_MSS_CLISensorStop(int32_t argc, char* argv[])
{
    int32_t errCode;

    if (gSrrMSSMCB.runningStatus == false)
    {
        return 0; // Already stopped.
    }

    /* The sensor can only be stopped; if the link has been configured */
    if (gSrrMSSMCB.cfgStatus == true)
    {
        /* Stop the sensor */
        if (MMWave_stop(gSrrMSSMCB.ctrlHandle, &errCode) < 0)
        {
            /* Error: Unable to stop the mmWave control module */
            System_printf("Error: mmWave stop failed [Error code %d]\n",
                          errCode);
            return -1;
        }
        System_printf("Debug: Sensor has been stopped\n");

        /* The link is no longer configured. */
        gSrrMSSMCB.runningStatus = true;
        return 0;
    }
    else
    {
        /* Invalid CLI use case; doing a sensor stop multiple times. Inform the user and return an error code. */
        System_printf(
                "Error: Sensor has already been stopped. Reconfigure and start the sensor if required\n");
        return -1;
    }
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for basic configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t SRR_MSS_CLIBasicCfg(int32_t argc, char* argv[])
{
    MMWave_OpenCfg openCfg;
    int32_t errCode;
    rlProfileCfg_t profileCfg;
    rlChirpCfg_t chirpCfg;
    rlFrameCfg_t frameCfg;
    int32_t retVal;

    if (gSrrMSSMCB.cfgStatus == true)
    {
        /* Radar has already been configured. */
        return 0;
    }

    /* Setup the calibration frequency: */
    openCfg.freqLimitLow = 760U;
    openCfg.freqLimitHigh = 810U;
    openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;

    /* Initialize the minimal configuration: */
    Cfg_ChannelCfgInitParams(&openCfg.chCfg);
    Cfg_LowPowerModeInitParams(&openCfg.lowPowerMode);
    Cfg_ADCOutCfgInitParams(&openCfg.adcOutCfg);

    /* Open the mmWave module: */
    if (MMWave_open(gSrrMSSMCB.ctrlHandle, &openCfg, &errCode) < 0)
    {
        System_printf(
                "Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n",
                errCode);
        return -1;
    }

    /********************************************************************************
     * MMWave Link and BSS is operational now. In minimal mode we have access to all
     * the mmWave Link API to perform the configuration
     *
     * Profile configuration:
     ********************************************************************************/
    Cfg_ProfileCfgInitParams(0U, &profileCfg);
    retVal = rlSetProfileConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, &profileCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf("Error: Unable to configure the profile [Error %d]\n",
                      retVal);
        return -1;
    }

    /********************************************************************************
     * Chirp configuration:
     ********************************************************************************/
    Cfg_ChirpCfgInitParams(0U, &chirpCfg);
    retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, &chirpCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf("Error: Unable to configure the chirp [Error %d]\n",
                      retVal);
        return -1;
    }

    /********************************************************************************
     * Frame configuration:
     ********************************************************************************/
    Cfg_FrameCfgInitParams(&frameCfg);
    retVal = rlSetFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &frameCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf("Error: Unable to configure the frame [Error %d]\n",
                      retVal);
        return -1;
    }

    /* The link has been configured. */
    gSrrMSSMCB.cfgStatus = true;
    System_printf(
            "Debug: Basic configuration completed. Start the sensor...\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for advanced frame configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t SRR_MSS_CLIAdvancedFrameCfg(int32_t argc, char* argv[])
{
    MMWave_OpenCfg openCfg;
    int32_t errCode;
    rlProfileCfg_t profileCfg;
    rlChirpCfg_t chirpCfg;
    rlAdvFrameCfg_t advFrameCfg;
    int32_t retVal;
    int32_t indx;

    if (gSrrMSSMCB.cfgStatus == true)
    {
        /* Radar has already been configured. */
        return 0;
    }

    /* Setup the calibration frequency: */
    openCfg.freqLimitLow = 760U;
    openCfg.freqLimitHigh = 810U;
    openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;

    /* Initialize the minimal configuration: */
    Cfg_ChannelCfgInitParams(&openCfg.chCfg);
    Cfg_LowPowerModeInitParams(&openCfg.lowPowerMode);
    Cfg_ADCOutCfgInitParams(&openCfg.adcOutCfg);

    /* Open the mmWave module: */
    if (MMWave_open(gSrrMSSMCB.ctrlHandle, &openCfg, &errCode) < 0)
    {
        System_printf(
                "Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n",
                errCode);
        return -1;
    }

    /********************************************************************************
     * MMWave Link and BSS is operational now. In minimal mode we have access to all
     * the mmWave Link API to perform the configuration
     *
     * Profile configurations:
     ********************************************************************************/
    for (indx = 0; indx < NUM_PROFILES; indx++)
    {

        Cfg_ProfileCfgInitParams(indx, &profileCfg);
        retVal = rlSetProfileConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U,
                                    &profileCfg);
        if (retVal != RL_RET_CODE_OK)
        {
            System_printf(
                    "Error: Unable to configure the profile %d [Error %d]\n",
                    indx, retVal);
            return -1;
        }

    }

    /********************************************************************************
     * Chirp configurations:
     ********************************************************************************/
    for (indx = 0; indx < NUM_CHIRP_PROG; indx++)
    {
        Cfg_ChirpCfgInitParams(indx, &chirpCfg);
        retVal = rlSetChirpConfig(RL_DEVICE_MAP_INTERNAL_BSS, 1U, &chirpCfg);
        if (retVal != RL_RET_CODE_OK)
        {
            System_printf("Error: Unable to configure chirp %d [Error %d]\n",
                          indx, retVal);
            return -1;
        }
    }

    /* Advanced Frame configuration. */
    Cfg_AdvFrameCfgInitParams(&advFrameCfg);
    retVal = rlSetAdvFrameConfig(RL_DEVICE_MAP_INTERNAL_BSS, &advFrameCfg);
    if (retVal != RL_RET_CODE_OK)
    {
        System_printf("Error: Advanced Frame configuration failed [Error %d]\n",
                      retVal);
        return -1;
    }

    /* The link has been configured. */
    gSrrMSSMCB.cfgStatus = true;
    System_printf("Debug: MMWave has been configured for SRR.\n");
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void SRR_MSS_CLIInit(void)
{

    /* Populate the CLI configuration: */
    gSrrHSEMSSMCB.cliCfg.cliPrompt = "SrrTIDesign:/>";
    gSrrHSEMSSMCB.cliCfg.cliUartHandle = gSrrMSSMCB.commandUartHandle;
    gSrrHSEMSSMCB.cliCfg.taskPriority = 3;
    gSrrHSEMSSMCB.cliCfg.mmWaveHandle = gSrrMSSMCB.ctrlHandle;
    gSrrHSEMSSMCB.cliCfg.enableMMWaveExtension = 0U;
    gSrrHSEMSSMCB.cliCfg.usePolledMode = true;

    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "basicCfg";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "Basic Cfg [Hardcoded Parameters]";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            SRR_MSS_CLIBasicCfg;

    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "advFrameCfg";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "Advanced Frame Cfg [Hardcoded Parameters]";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            SRR_MSS_CLIAdvancedFrameCfg;

    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "sensorStart";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "Start the sensor; ensure that the configuration is completed";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            SRR_MSS_CLISensorStart;

    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "sensorStop";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "Stop the sensor";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            SRR_MSS_CLISensorStop;

    /*HSE PROOF OF CONCEPT FUNCTION*/
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "HiRadar";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "Simple Hello World";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            hiRadar;
    /*HSE CAN TEST FUNCTION */
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "CANTEST";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "CAN Hello world";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            sendTestCANMessage;
    /*HSE CAN ID CONFIG */
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "CANREQEXUID";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "<bit-Index> Get Unique ID at given bit-index";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            getUIDBit;

    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "CANENDEXUID";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "no argument";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            endUIDExchange;

    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "FrmSync";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "<bool HWSYNC><uint32 FrameDelay [5 lsb = 1 ns. 100us max.] Parameter will only be set when UID is smaller> ";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            configHWtriggerAndFrameDelay;

    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].cmd =
            "ChirpTimestamp";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter].helpString =
            "no parameters. Command toggles chirp timestamps (initially deactivated).";
    gSrrHSEMSSMCB.cliCfg.tableEntry[gSrrHSEMSSMCB.cliCommandCounter++].cmdHandlerFxn =
            setSendChirpTimestamps;
    /* Open the CLI: */
    if (CLI_open(&(gSrrHSEMSSMCB.cliCfg)) < 0)
    {
        System_printf("Error: Unable to open the CLI\n");
        return;
    }
    System_printf("Debug: CLI is operational\n");

    /* The link is not configured. */
    gSrrMSSMCB.cfgStatus = false;
    gSrrMSSMCB.runningStatus = false;
    gSrrMSSMCB.isMMWaveOpen = false;
    return;
}

