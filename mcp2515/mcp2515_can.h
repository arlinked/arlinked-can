/*
  mcp2515_can.h
  2014 Copyright (c) Arlinked Inc.  All right reserved.

  Author: toejiang
  Contributor: Cory J. Fowler
  2014-1-16
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef __MCP2515_CAN_H__
#define __MCP2515_CAN_H__

#include "mcp2515_define.h"

#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
private:
    
    /* identifier xxxID either extended (the 29 LSB) or standard (the 11 LSB)     */
    INT8U   m_nExtFlg; 

    /* can id */
    INT32U  m_nID;

    /* data length */
    INT8U   m_nDlc;

    /* data */
    INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];

    /* rtr */
    INT8U   m_nRtr;

    INT8U   m_nfilhit;

    INT8U   SPICS;

private:

    /*
    *  mcp2515 driver function 
    */

    /* reset mcp2515 */
    void mcp2515_reset(void);

    /* read mcp2515's register */
    INT8U mcp2515_readRegister(const INT8U address);
    
    void mcp2515_readRegisterS(const INT8U address, 
	                       INT8U values[], 
                               const INT8U n);

    /* set mcp2515's register */
    void mcp2515_setRegister(const INT8U address,
                             const INT8U value);

    /* set mcp2515's registers */
    void mcp2515_setRegisterS(const INT8U address,
                              const INT8U values[],
                              const INT8U n);
    
    void mcp2515_initCANBuffers(void);

    /* set bit of one register */
    void mcp2515_modifyRegister(const INT8U address,
                                const INT8U mask,
                                const INT8U data);

    /* read mcp2515's Status */
    INT8U mcp2515_readStatus(void);

    /* set mode */
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);

    /* set boadrate */
    INT8U mcp2515_configRate(const INT8U canSpeed);

    /* mcp2515init */
    INT8U mcp2515_init(const INT8U canSpeed);

    /* write can id */
    void mcp2515_write_id( const INT8U mcp_addr,
                               const INT8U ext,
                               const INT32U id );

    /* read can id */
    void mcp2515_read_id( const INT8U mcp_addr,
                                    INT8U* ext,
                                    INT32U* id );

    /* write can msg */
    void mcp2515_write_canMsg( const INT8U buffer_sidh_addr );

    /* read can msg */
    void mcp2515_read_canMsg( const INT8U buffer_sidh_addr);

    /* start transmit */
    void mcp2515_start_transmit(const INT8U mcp_addr);

    /* get Next free txbuf */
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);

    /*********
    *  can operator function
    */    

    /* set message */  
    INT8U setMsg(INT32U id, INT8U ext, INT8U len, INT8U *pData);

    /* clear all message to zero */
    INT8U clearMsg();

    /* read message */
    INT8U readMsg();

   /* send message */
    INT8U sendMsg();

public:
    MCP_CAN(INT8U _CS);

    /* init can */
    INT8U begin(INT8U speedset);

    /* init Masks */
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);

    /* init filters */
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);

    /* send buf */
    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);

    /* read buf */
    INT8U readMsgBuf(INT8U *len, INT8U *buf);

    /* if something received */
    INT8U checkReceive(void);

    /* if something error */
    INT8U checkError(void);

    /* get can id when receive */
    INT32U getCanId(void);
};

extern MCP_CAN CAN;

extern INT8U DEBUG_LOG;

#endif

