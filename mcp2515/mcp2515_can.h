/**
 * File: mcp2515_can.h implementation of mcp2515 can-bus
 * Copyright (C) 2014 Arlinked Inc. All right reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 **/

#ifndef __MCP2515_CAN_H__
#define __MCP2515_CAN_H__

#include "mcp2515_define.h"

#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
private:

	/* identifier xxxID either extended (the 29 LSB) or standard (the 11 LSB) */
	uint8 m_nExtFlg;

	/* can id */
	uint32 m_nID;

	/* data length */
	uint8 m_nDlc;

	/* data */
	uint8 m_nDta[MAX_CHAR_IN_MESSAGE];

	/* rtr */
	uint8 m_nRtr;

	uint8 m_nfilhit;

	uint8 SPICS;

private:

	/*
	* mcp2515 driver function
	*/

	/* reset mcp2515 */
	void mcp2515_reset(void);

	/* read mcp2515's register */
	uint8 mcp2515_readRegister(const uint8 address);

	void mcp2515_readRegisterS(const uint8 address,
			uint8 values[],
			const uint8 n);

	/* set mcp2515's register */
	void mcp2515_setRegister(const uint8 address,
			const uint8 value);

	/* set mcp2515's registers */
	void mcp2515_setRegisterS(const uint8 address,
			const uint8 values[],
			const uint8 n);

	void mcp2515_initCANBuffers(void);

	/* set bit of one register */
	void mcp2515_modifyRegister(const uint8 address,
			const uint8 mask,
			const uint8 data);

	/* read mcp2515's Status */
	uint8 mcp2515_readStatus(void);

	/* set mode */
	uint8 mcp2515_setCANCTRL_Mode(const uint8 newmode);

	/* set boadrate */
	uint8 mcp2515_configRate(const uint8 canSpeed);

	/* mcp2515init */
	uint8 mcp2515_init(const uint8 canSpeed);

	/* write can id */
	void mcp2515_write_id( const uint8 mcp_addr,
			const uint8 ext,
			const uint32 id );

	/* read can id */
	void mcp2515_read_id( const uint8 mcp_addr,
			uint8* ext,
			uint32* id );

	/* write can msg */
	void mcp2515_write_canMsg( const uint8 buffer_sidh_addr );

	/* read can msg */
	void mcp2515_read_canMsg( const uint8 buffer_sidh_addr);

	/* start transmit */
	void mcp2515_start_transmit(const uint8 mcp_addr);

	/* get Next free txbuf */
	uint8 mcp2515_getNextFreeTXBuf(uint8 *txbuf_n);

	/*********
	* can operator function
	*/

	/* set message */
	uint8 setMsg(uint32 id, uint8 ext, uint8 len, uint8 *pData);

	/* clear all message to zero */
	uint8 clearMsg();

	/* read message */
	uint8 readMsg();

	/* send message */
	uint8 sendMsg();

public:
	MCP_CAN(uint8 _CS);

	/* init can */
	uint8 begin(uint8 speedset);

	/* init Masks */
	uint8 init_Mask(uint8 num, uint8 ext, uint32 ulData);

	/* init filters */
	uint8 init_Filt(uint8 num, uint8 ext, uint32 ulData);

	/* send buf */
	uint8 sendMsgBuf(uint32 id, uint8 ext, uint8 len, uint8 *buf);

	/* read buf */
	uint8 readMsgBuf(uint8 *len, uint8 *buf);

	/* if something received */
	uint8 checkReceive(void);

	/* if something error */
	uint8 checkError(void);

	/* get can id when receive */
	uint32 getCanId(void);

	void reportState(uint8 addr);
};

extern MCP_CAN CAN;

extern uint8 CAN_DEBUG_LOG;

#endif

