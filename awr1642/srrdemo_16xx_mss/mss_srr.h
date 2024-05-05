/**
 *   @file  mss_srr.h
 *
 *   @brief
 *      This is the main header file for the MSS for the SRR TI Design 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2017 Texas Instruments, Inc.
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
#ifndef MSS_SRR_H
#define MSS_SRR_H

/* MMWAVE Driver Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/cbuff/cbuff.h>

#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/osal/DebugP.h>

/* BIOS/XDC Include Files */
#include <ti/sysbios/knl/Semaphore.h>
//HSE had to include knl/Event
#include <ti/sysbios/knl/Event.h>

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwave/mmwave.h>
#include "../common/srr_config_consts.h"

//HSE CAN CONFIGURATION
#include <ti/utils/cli/cli.h>
#include <common/mmw_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*HSE CAN Additional CAN DEFINITIONS*/
//HSE SRR Additional TLV Types (Messages from DSS to MSS) "ovverrides MSG_RANGE_PROFILE; NOISE_PROFILE; AZIMUT_STATIC_HEAT_MAP" from  MMwDemo_output_message_type from <(sdk 2.0)>/mmw_output.h
typedef enum HSE_CAN_MSG_TYPE_e
{
    MMWDEMO_OUTPUT_MSG_DETECTED_PTS = 0x01U,
    MMWDEMO_OUTPUT_MSG_CLUSTERS = 0x2U,
    MMWDEMO_OUTPUT_MSG_PARKING_ASSIST = 0x3U,
    MMWDEMO_OUTPUT_MSG_TRACKED_OBJECTS = 0x4U,
    //HSE RANGE DOPPLER
    HSE_OUTPUT_MSG_RANGE_DOPPLER = 0x5U,
    MMWDEMO_OUTPUT_MSG_HEADER = 0x7U,
    MMWDEMO_OUTPUT_MSG_PADDING = 0x8U,
    CAN_MESSAGE_MMWDEMO_MAX = 0x8U /*(messge type (6) + header (1) + padding(1))*/
} HSE_CAN_MSG_TYPE;

#define CAN_FRAME_USE_FD            0x0U
#define CAN_FRAME_USE_CLASSIC       0x1U
#define CAN_FRAME_SEL               CAN_FRAME_USE_FD //00 = native CAN FD frame, 01 = CAN B 2.0 frame

#define CAN_MESSAGE_MMWDEMO_HEADER  0xC1
#define CAN_MESSAGE_MMWDEMO_PADDING 0xB1

#define CAN_MSGOBJ_HEADER   0x7U
#define CAN_MSGOBJ_PADDING  0x8U

typedef enum mmwDemo_can_message_type_e
{
    /*! @brief   List of detected points */
    CAN_MESSAGE_MMWDEMO_DETECTED_POINTS = 0xD1,

    /*! @brief   Range profile */
    CAN_MESSAGE_MMWDEMO_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    CAN_MESSAGE_MMWDEMO_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    CAN_MESSAGE_MMWDEMO_STATS,

    /*HSE EXTENSION FOR SRR DEMO*/
//CLUSTER Message
    CAN_MESSAGE_MMWDEMO_CLUSTERS,

//Park Assist
    CAN_MESSAGE_MMWDEMO_PARKING_ASSIST,

//Tracking Clusters
    CAN_MESSAGE_MMWDEMO_TRACKED_OBJECTS,

//HSE RANGE DOPPLER : Range Doppler Matrix (uint16_t) * 512 (range) * 32 (doppler)
    CAN_MESSAGE_HSE_RANGE_DOPPLER_MATRIX //0xDA (No Offset) or 0x9A (Offset)
} mmwDemo_can_message_type;

/*! @brief   sensor start CLI event */
#define MMWDEMO_CLI_SENSORSTART_EVT                     Event_Id_00

/*! @brief   sensor stop CLI  event */
#define MMWDEMO_CLI_SENSORSTOP_EVT                      Event_Id_01

/*! @brief   sensor frame start CLI  event */
#define MMWDEMO_CLI_FRAMESTART_EVT                      Event_Id_02

/*! @brief   BSS CPUFAULT event */
#define MMWDEMO_BSS_CPUFAULT_EVT                        Event_Id_03

/*! @brief   BSS ESMFAULT event */
#define MMWDEMO_BSS_ESMFAULT_EVT                        Event_Id_04

/*! @brief   Monitoring report event */
#define MMWDEMO_BSS_MONITORING_REP_EVT                  Event_Id_05

/*! @brief   BSS Calibration report event */
#define MMWDEMO_BSS_CALIBRATION_REP_EVT                 Event_Id_06

/*! @brief   start completed event from DSS/MSS */
#define MMWDEMO_DSS_START_COMPLETED_EVT                 Event_Id_07

/*! @brief   stop completed event from DSS */
#define MMWDEMO_DSS_STOP_COMPLETED_EVT                  Event_Id_08

/*! @brief   start failed event from DSS/MSS */
#define MMWDEMO_DSS_START_FAILED_EVT                    Event_Id_09

/* All CLI events */
#define MMWDEMO_CLI_EVENTS                              (MMWDEMO_CLI_SENSORSTART_EVT |    \
                                                         MMWDEMO_CLI_SENSORSTOP_EVT |     \
                                                         MMWDEMO_CLI_FRAMESTART_EVT)

/* All BSS faults events */
#define MMWDEMO_BSS_FAULT_EVENTS                        (MMWDEMO_BSS_CPUFAULT_EVT |     \
                                                         MMWDEMO_BSS_ESMFAULT_EVT )

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MSS_STATS_t
{
    /*! @brief   CLI event for sensorStart */
    uint8_t cliSensorStartEvt;

    /*! @brief   CLI event for sensorStop */
    uint8_t cliSensorStopEvt;

    /*! @brief   CLI event for frameStart */
    uint8_t cliFrameStartEvt;

    /*! @brief   Counter which tracks the number of datapath config event detected
     *           The event is triggered in mmwave config callback function */
    uint8_t datapathConfigEvt;

    /*! @brief   Counter which tracks the number of datapath start event  detected 
     *           The event is triggered in mmwave start callback function */
    uint8_t datapathStartEvt;

    /*! @brief   Counter which tracks the number of datapath stop event detected 
     *           The event is triggered in mmwave stop callback function */
    uint8_t datapathStopEvt;

    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t numFailedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t numCalibrationReports;
} MmwDemo_MSS_STATS;

/**
 * @brief
 *  MSS SRR TI Design Master control block
 *
 * @details
 *  The structure is used to hold information pertinent to the MSS SRR TI Design.
 */
typedef struct Srr_MSS_MCB_t
{
    /**
     * @brief  An token for frame start events. 
     */
    int32_t frameStartToken;

    /**
     * @brief  The number of subframes transmitted derived from  chirp available interrupts. 
     */
    int32_t subframeCntFromChirpInt;

    /**
     * @brief  The number of subframes transmitted derived from the frame start interrupts. 
     */
    int32_t subframeCntFromFrameStart;

    /**
     * @brief  The total number of chirp available interrupts. 
     */
    int32_t chirpIntcumSum;

    /**
     * @brief  A counter for chirp interrupts. It is reset every subframe. 
     */
    int32_t chirpInt;

    /**
     * @brief  The number of chirps per subframe. 
     */
    int32_t numChirpsPerSubframe[NUM_SUBFRAMES];

    /**
     * @brief  An indicator for the current subframe. 
     */
    int32_t subframeId;

    /**
     * @brief  Handle to the DMA to transfer data.
     */
    DMA_Handle dmaHandle;

    /**
     * @brief   Handle to the SOC Module
     */
    SOC_Handle socHandle;

    /*! @brief   UART Logging Handle */
    UART_Handle loggingUartHandle;

    /**
     * @brief   UART Command Handle used to interface with the CLI
     */
    UART_Handle commandUartHandle;

    /**
     * @brief   mmWave control handle use to initialize the link infrastructure
     * which allows communication between the MSS and BSS
     */
    MMWave_Handle ctrlHandle;

    /**
     * @brief   Handle to the ADCBUF Driver
     */
    ADCBuf_Handle adcBufHandle;

    /*!@brief   Handle to the peer Mailbox */
    Mbox_Handle peerMailbox;

    /*! @brief   Semaphore handle for the mailbox communication */
    Semaphore_Handle mboxSemHandle;

    /*! @brief   MSS system event handle */
    Event_Handle eventHandle;

    /*! @brief   MSS system event handle */
    Event_Handle eventHandleNotify;

    /*! @brief   Handle to the CBUFF Module */
    CBUFF_Handle cbuffHandle;

    /*! @brief   Handle to the SW Triggered Session which streams out application data */
    CBUFF_SessionHandle swSessionHandle;

    /**
     * @brief   Handle to the SOC chirp interrupt listener Handle
     */
    SOC_SysIntListenerHandle chirpIntHandle;

    /**
     * @brief   Handle to the SOC chirp interrupt listener Handle
     */
    SOC_SysIntListenerHandle frameStartIntHandle;

    /**
     * @brief   This is a flag which indicates if the mmWave link has been configured
     * or not.
     */
    bool cfgStatus;

    /**
     * @brief   This is a flag which indicates if the radar is tranmsitting or not
     * or not.
     */
    bool runningStatus;

    /**
     * @brief   This is a flag which indicates if the basic radar configuration is completed.
     */
    bool isMMWaveOpen;

    /*! @brief   mmw Demo stats */
    MmwDemo_MSS_STATS stats;
} Srr_MSS_MCB;

/**
 * @brief
 *  MSS SRR HSE Design Master control block
 *
 * @details
 *  The structure is used to hold information pertinent to the Hochschule Esslingen MSS SRR TI Design.
 */
typedef struct Srr_HSE_MSS_MCB_t
{
    /**
     * @brief   This is a flag which indicates weather the last requested Unique ID (UID) bit was 1
     */
    bool isLastRequestedBitIndexOne;
    /**
     * @brief   This is a flag which indicates weather the CAN offset for outgoing messages of -0x40 is used or not
     */
    bool useCANIDOffset;
    /**
     * @brief   This is a flag which indicates weather the frames are toggled via HW-Sync (True) or SW-Sync (False). Value is set via CLI-Command "FrmSync".
     */
    bool useFrameHWTrigger;
    /**
     * @brief   Each frame will be delayed by this value. Max is 100us. Value is set via CLI-Command "FrmSync".
     */
    uint32_t frameTriggerDelay;
    /**
     * @brief   Counter for the amount of implemented CLI-Commands.
     */
    uint16_t cliCommandCounter;
    /**
     * @brief   Handle to the CLI-Structure that holds the CLI-Command function pointers etc.
     */
    CLI_Cfg cliCfg;
    /**
     * @brief  Flags that indicate weather command at index x of cliCfgs command table has been called via CAN.
     */
    bool * receivedCANCommand; //Flags that indicate weather command x has been called via CAN.
    /**
     * @brief  Last command (index 0) and parameters (index 1 upwards) that was passed via CAN-CLI.
     */
    char tokenizedArg0[16];
    char tokenizedArg1[16];
    char tokenizedArg2[16];
    char * tokenizedArgs[3];

    /**
     * @brief  Amount of parameters (+1) that were lastly passed via CAN-CLI and that are stored at tokenizedArgs.
     */
    uint32_t argIndex;
    /**
     * @brief   This is a flag which indicates weather the Chirp-Timestamps are sent via CAN in TLV-Header or not. CLI Command is "ChirpTimestamp".
     */
    bool sendChirptimestamps;

} Srr_HSE_MSS_MCB;

int32_t MmwDemo_mboxWrite(MmwDemo_message * message);

/*******************************************************************************************
 * Extern Global Structures:
 *******************************************************************************************/
extern Srr_MSS_MCB gSrrMSSMCB;
extern Srr_HSE_MSS_MCB gSrrHSEMSSMCB; //global structure holding HSE variables

/*******************************************************************************************
 * Extern IPC API:
 *******************************************************************************************/
extern int32_t IPC_init(Mailbox_Type remoteEndPoint);
extern int32_t IPC_sendChannelCfg(rlChanCfg_t* ptrChannelCfg);
extern int32_t IPC_sendLowPowerModeCfg(rlLowPowerModeCfg_t* ptrLowPowerMode);
extern int32_t IPC_sendADCOutCfg(rlAdcOutCfg_t* ptrADCOutCfg);
extern int32_t IPC_sendProfileCfg(rlProfileCfg_t* ptrProfileCfg);
extern int32_t IPC_sendChirpCfg(rlChirpCfg_t* ptrChirpCfg);
extern int32_t IPC_sendFrameCfg(rlFrameCfg_t* ptrFrameCfg);
extern int32_t IPC_sendAdvFrameCfg(rlAdvFrameCfg_t* ptrAdvFrameCfg);
extern int32_t IPC_sendSensorStart(void);
extern int32_t IPC_sendSensorStop(void);

/*******************************************************************************************
 * Extern CLI API:
 *******************************************************************************************/
extern void SRR_MSS_CLIInit(void);

/*******************************************************************************************
 * Extern CFG API:
 *******************************************************************************************/
extern void Cfg_AdvFrameCfgInitParams(rlAdvFrameCfg_t* ptrAdvFrameCfg);
extern void Cfg_FrameCfgInitParams(rlFrameCfg_t* ptrFrameCfg);
extern void Cfg_ProfileCfgInitParams(uint8_t profileNum,
                                     rlProfileCfg_t* ptrProfileCfg);
extern void Cfg_ChirpCfgInitParams(uint8_t chirpNum, rlChirpCfg_t* ptrChirpCfg);
extern void Cfg_LowPowerModeInitParams(rlLowPowerModeCfg_t* ptrLowPowerMode);
extern void Cfg_ChannelCfgInitParams(rlChanCfg_t* ptrChannelCfg);
extern void Cfg_ADCOutCfgInitParams(rlAdcOutCfg_t* ptrADCOutCfg);

/*HSE CAN Transmit Function*/
int32_t Can_Transmit_Schedule(uint32_t msg_id, uint8_t *txmsg, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* MSS_SRR_H */

