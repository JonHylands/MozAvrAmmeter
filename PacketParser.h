/****************************************************************************
*
*   Copyright (c) 2013 Jon Hylands     <jhylands@mozilla.com>
*
*   @file   PacketParser.h
*
*   @brief  Contains definitions for the ammeter packet parser.
*
****************************************************************************/

#if !defined( PACKET_PARSER_H )
#define PACKET_PARSER_H       /**< Include Guard                             */

/* ---- Include Files ---------------------------------------------------- */

#include <stdint.h>
#include "Config.h"

/* ---- Constants and Types ---------------------------------------------- */

#define PACKET_STATE_IDLE          0   ///< We're waiting for the beginning of the packet
#define PACKET_STATE_1ST_FF_RCVD   1   ///< We've received the 1st 0xFF
#define PACKET_STATE_2ND_FF_RCVD   2   ///< We've received the 2nd 0xFF
#define PACKET_STATE_ID_RCVD       4   ///< We've received the ID
#define PACKET_STATE_LENGTH_RCVD   5   ///< We've received the length
#define PACKET_STATE_COMMAND_RCVD  6   ///< We've received the command

#if !defined( CFG_PACKET_MAX_PARAM )
#   define  CFG_PACKET_MAX_PARAM   8
#endif
#define PACKET_MAX_PARAM       CFG_PACKET_MAX_PARAM

#define PACKET_BROADCAST_ID    0xFE

typedef uint8_t     PACKET_Error_t;
typedef uint8_t     PACKET_State_t;
typedef uint8_t     PACKET_ID_t;
typedef uint8_t     PACKET_Length_t;
typedef uint8_t     PACKET_Command_t;

#define PACKET_ERROR_RESERVED      0x80    ///< Reserved - set to zero
#define PACKET_ERROR_CHECKSUM 0x01    ///< checksum error
#define PACKET_ERROR_NONE          0x00    ///< No Error

typedef struct
{
    PACKET_ID_t        m_id;
    PACKET_Length_t    m_length;
    PACKET_Command_t   m_cmd;
    uint8_t         m_param[ PACKET_MAX_PARAM ];
    uint8_t         m_crc;                      // Placeholder for CRC

} PACKET_Packet_t;

struct PACKET_Instance_s;

typedef struct PACKET_Instance_s PACKET_Instance_t;

typedef void (*PACKET_PacketReceived)( PACKET_Instance_t *instance, PACKET_Packet_t *pkt, PACKET_Error_t err );

struct PACKET_Instance_s
{
    PACKET_ID_t            m_id;       ///< Our ID on the bus, 0xFF if we're just monitoring
    PACKET_State_t         m_state;
    PACKET_Length_t        m_paramIdx;
    PACKET_Packet_t        m_pkt;      ///< Contains the packet that was actually received
    PACKET_PacketReceived  m_pktRcvd;  ///< Ptr to Fn called when a packet is successfully received

    uint8_t             m_logPacket;  ///< Causes Log statements for each packet sent
};

#define PACKET_LogPacket( inst, fmt, args... ) do { if ( inst->m_logPacket ) { Log( fmt, ## args ); }} while (0)

// Instructions

#define PACKET_CMD_SET_ID       0x01
#define PACKET_CMD_START_ASYNC       0x02
#define PACKET_CMD_STOP_ASYNC       0x03
#define PACKET_CMD_ASYNC       0x04
#define PACKET_CMD_TURN_OFF_BATTERY       0x05
#define PACKET_CMD_TURN_ON_BATTERY       0x06
#define PACKET_CMD_SEND_SAMPLE 0x07
#define PACKET_CMD_SAMPLE 0x08
#define PACKET_CMD_SET_CALIBRATION 0x09
#define PACKET_CMD_GET_RAW_SAMPLE 0x0A

/* ---- Variable Externs ------------------------------------------------- */

/* ---- Function Prototypes ---------------------------------------------- */

void PACKET_Init( PACKET_Instance_t *instance );
void PACKET_ProcessChar( PACKET_Instance_t *instance, uint8_t ch );

#endif /* PACKET_PARSER_H */

