/****************************************************************************
*
*   Copyright (c) 2013 Jon Hylands     <jhylands@mozilla.com>
*
*   @file   PacketParser.c
*
*   @brief  Implements a parser for detecting and parsing packets sent to
*           the mozilla ammeter
*
*****************************************************************************/

/* ---- Include Files ----------------------------------------------------- */

#include "PacketParser.h"
#include "Log.h"

#include <string.h>

/* ---- Public Variables -------------------------------------------------- */

/* ---- Private Constants and Types --------------------------------------- */

/* ---- Private Variables ------------------------------------------------- */

/* ---- Private Function Prototypes --------------------------------------- */

#define AccumCrc( inst, ch )    ((inst)->m_pkt.m_crc += (ch))

/* ---- Functions --------------------------------------------------------- */


//***************************************************************************
/**
*   Initializes a PACKET_State_t structure.
*/

void PACKET_Init( PACKET_Instance_t *inst )
{
    memset( inst, 0, sizeof( *inst ));

} // PACKET_Init

//***************************************************************************
/**
*   Runs the packet parsing state machine. When a packet is recognized,
*   the PacketReceived callback will be called.
*/

void PACKET_ProcessChar( PACKET_Instance_t *inst, uint8_t ch )
{
    PACKET_State_t nextState = inst->m_state;

    switch ( nextState )
    {
        case PACKET_STATE_IDLE:    // We're waiting for the beginning of the packet (0xFF)
        {
            if ( ch == 0xFF )
            {
// 				printf("PACKET - switching to PACKET_STATE_1ST_FF_RCVD\n");
                nextState = PACKET_STATE_1ST_FF_RCVD;
            }
            break;
        }

        case PACKET_STATE_1ST_FF_RCVD:  // We've received the 1st 0xFF
        {
            if ( ch == 0xFF )
            {
// 				printf("PACKET - switching to PACKET_STATE_2ND_FF_RCVD\n");
                nextState = PACKET_STATE_2ND_FF_RCVD;
            }
            else
            {
// 				printf("PACKET - switching to PACKET_STATE_IDLE\n");
                nextState = PACKET_STATE_IDLE;
            }
            break;
        }

        case PACKET_STATE_2ND_FF_RCVD:  // We've received the 2nd 0xFF, ch is the ID
        {
            if ( ch == 0xFF )
            {
                // 0xFF is invalid as an ID, so just stay in this state until we receive
                // a non-0xFF

// 				printf("PACKET - staying in PACKET_STATE_2ND_FF_RCVD\n");
                nextState = PACKET_STATE_2ND_FF_RCVD;
                break;
            }
            inst->m_pkt.m_id = ch;
			inst->m_pkt.m_crc = 0;
            AccumCrc( inst, ch );
// 			printf("PACKET - ID: %d - switching to PACKET_STATE_ID_RCVD\n", ch);
            nextState = PACKET_STATE_ID_RCVD;
            break;
        }

        case PACKET_STATE_ID_RCVD:      // We've received the ID, ch is the length
        {
            inst->m_pkt.m_length = ch;
            AccumCrc( inst, ch );
// 			printf("PACKET - Length: %d - switching to PACKET_STATE_LENGTH_RCVD\n", ch);
            nextState = PACKET_STATE_LENGTH_RCVD;
            break;
        }

        case PACKET_STATE_LENGTH_RCVD:  // We've received the length, ch is the command
        {
            inst->m_pkt.m_cmd = ch;
            AccumCrc( inst, ch );
            inst->m_paramIdx = 0;
// 			printf("PACKET - Command: %d - switching to PACKET_STATE_COMMAND_RCVD\n", ch);
            nextState = PACKET_STATE_COMMAND_RCVD;
            break;
        }

        case PACKET_STATE_COMMAND_RCVD: // We've received the command, ch is a param byte or checksum
        {
// 			printf("PACKET - top of PACKET_STATE_COMMAND_RCVD\n");
            if (( inst->m_paramIdx + 2 ) >= inst->m_pkt.m_length )
            {
                PACKET_Error_t err = PACKET_ERROR_NONE;

                // ch is the Checksum

                inst->m_pkt.m_crc = ~inst->m_pkt.m_crc;
// 				printf("PACKET - CRC: %d\n", ch);

                if ( inst->m_pkt.m_crc != ch )
                {
                    // CRC failed

                    err = PACKET_ERROR_CHECKSUM;
//                     printf( "Rcvd Checksum: 0x%02x Expecting: 0x%02x\n", ch, inst->m_pkt.m_crc );
                }

                if ( inst->m_pktRcvd != NULL )
                {
// 					printf("PACKET - full packet received\n");
                    inst->m_pktRcvd( inst, &inst->m_pkt, err );
                }
                nextState = PACKET_STATE_IDLE;
                break;
            }

            if ( inst->m_paramIdx < sizeof( inst->m_pkt.m_param ))
            {
// 				printf("PACKET - parameter byte %d\n", ch);
                AccumCrc( inst, ch );
                inst->m_pkt.m_param[ inst->m_paramIdx ] = ch;
            }
            inst->m_paramIdx++;
            break;
        }
    }

    inst->m_state = nextState;

} // PACKET_ProcessChar

