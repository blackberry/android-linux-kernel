/*
 * Copyright (C) 2015 BlackBerry Limited
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#ifdef CONFIG_BBRY
#include "../slimport_private.h"
#else
#include "../slimport.h"
#endif
#include "../slimport_tx_reg.h"
#include "../slimport_tx_drv.h"
#include "slimport_tx_cec.h"


#define CEC_DEVICE_TYPE CEC_DEVICE_TYPE_PLAYBACK_DEVICE

unchar  g_CECRecvBuf[CEC_RECV_BUF_SIZE];
unchar  *g_pCECRecvHead;
unchar  *g_pCECRecvTail;
unchar  g_CECSendBuf[CEC_SEND_BUF_SIZE];
unchar  *g_pCECSendHead;
unchar  *g_pCECSendTail;

unchar  g_CECRecvBuf_HDMI[CEC_RECV_BUF_SIZE];
unchar  *g_pCECRecvHead_HDMI;
unchar  *g_pCECRecvTail_HDMI;
unchar  g_CECSendBuf_HDMI[CEC_SEND_BUF_SIZE];
unchar  *g_pCECSendHead_HDMI;
unchar  *g_pCECSendTail_HDMI;

unchar bCECStatus = CEC_NOT_READY;


struct tagCECFrame g_CECFrame;
struct tagCECFrame g_CECFrame_HDMI;

unchar g_LogicAddr;




#ifdef CEC_PHYCISAL_ADDRESS_INSERT
unchar downstream_physicaladdrh;
unchar downstream_physicaladdrl;
#endif




void cec_init(void)
{
	unchar c;
	unchar i;

	/*set status to un-initialied.*/
	bCECStatus = CEC_NOT_READY;

	g_LogicAddr = CEC_DEVICE_TYPE;

	/*set uptream CEC logic address
	to 0x0(worked as TV), reset CEC, set to RX*/
	sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, 0x09);


	/*set downtream CEC logic address
	to 0x04(worked as upstream DVD playback)
	//reset cec, enable RX*/
	c = 0x49;
	for (i = 0; i < 5; i++) {
		if (AUX_OK ==
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c))
			break;
		else
			DEV_ERR("RX CEC logic address set fail!\n");
	}


	/*initial receive and transmitter buffer*/
	for (c = 0; c < CEC_RECV_BUF_SIZE; c++) {
		g_CECRecvBuf[c] = 0;
		g_CECRecvBuf_HDMI[c] = 0;
	}

	for (c = 0; c < CEC_SEND_BUF_SIZE; c++) {
		g_CECSendBuf[c] = 0;
		g_CECSendBuf_HDMI[c] = 0;
	}

	g_pCECRecvHead = g_CECRecvBuf;
	g_pCECRecvTail = g_CECRecvBuf;
	g_pCECSendHead = g_CECSendBuf;
	g_pCECSendTail = g_CECSendBuf;

	g_pCECRecvHead_HDMI = g_CECRecvBuf_HDMI;
	g_pCECRecvTail_HDMI = g_CECRecvBuf_HDMI;
	g_pCECSendHead_HDMI = g_CECSendBuf_HDMI;
	g_pCECSendTail_HDMI = g_CECSendBuf_HDMI;

#ifdef CEC_PHYCISAL_ADDRESS_INSERT
	downstream_physicaladdrh = 0xff;
	downstream_physicaladdrl = 0xff;
#endif

	/*set status toinitialied.*/
	bCECStatus = CEC_IS_READY;


}

void cec_status_set(enum CEC_STATUS cStatus)
{
    bCECStatus = cStatus;
}



enum CEC_STATUS cec_status_get(void)
{
	return bCECStatus;
}



unchar downStream_hdmi_cec_writemessage(struct tagCECFrame *pFrame, unchar Len)
{
    unchar t;

	if (bCECStatus == CEC_NOT_READY)
		return 0;

	t = downstream_cec_sendframe((unchar *)pFrame, Len);
	return t;
}

void downstream_hdmi_cec_readmessage(void)
{
	struct tagCECFrame *pFrame;
	unchar len;



	if (bCECStatus == CEC_NOT_READY)
		return;

	pFrame = &g_CECFrame;
	downstream_cec_readfifo();
	while ((len = downstream_cec_recvframe((unchar *)pFrame)) != 0) {
		if (len >= 1)
			upstream_hdmi_cec_writemessage(&g_CECFrame, len);
	}
}


void downstream_cec_readfifo(void)
{
	unchar cstatus, c;
	unchar i;
	unchar FIFOLen;
	uint time_out_counter;

	/*if aux error , abort read*/
	if (AUX_ERR == sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x71, 1, &cstatus))
		return;

	time_out_counter = 0;
	do {
		/*get fifo count*/
		FIFOLen = cstatus&0x1F;

		FIFOLen <<= 1;
		for (i = 0; i < FIFOLen; i++) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x80, 1, &c);
			*g_pCECRecvHead = c;

			if (g_pCECRecvHead ==
				g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvHead = g_CECRecvBuf;
			else
				g_pCECRecvHead++;

			if (g_pCECRecvHead == g_pCECRecvTail)
				DEV_ERR("buffer full!\r\n");

		}
		if (AUX_ERR == sp_tx_aux_dpcdread_bytes
			(0x00, 0x05, 0x71, 1, &cstatus))
			return;

		time_out_counter++;
		if (time_out_counter >= TIME_OUT_THRESHOLD)
			break;
		/*CEC busy or fifo data count not zero, go on reading*/
	} while ((cstatus&0x80)|(cstatus&0x1F));
}

unchar downstream_cec_checkfullframe(void)
{
	unchar *pTmp;
	unchar i;

	uint time_out_counter;

	time_out_counter = 0;

	i = 0;
	pTmp = g_pCECRecvTail;
	/* check if end of buffer*/
	while (pTmp != g_pCECRecvHead) {

		if (*pTmp != 0)
			i++;

		if (pTmp == g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf;
		else
			pTmp++;

		if (pTmp == g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf;
		else
			pTmp++;


		time_out_counter++;
		if (time_out_counter >= TIME_OUT_THRESHOLD)
			break;
	}


	if (i >= 1)
		return 1;
	else
		return 0;

}

unchar downstream_cec_recvframe(unchar *pFrameData)
{
	unchar DataLen;
	uint time_out_counter;
	time_out_counter = 0;


	DataLen = 0;
	if (downstream_cec_checkfullframe()) {
		do {
			if (g_pCECRecvTail == g_pCECRecvHead)
				break;


			if (g_pCECRecvTail ==
				g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail = g_CECRecvBuf;
			else
				g_pCECRecvTail++;

			*pFrameData = *g_pCECRecvTail;
			DataLen++;
			pFrameData++;
			if (g_pCECRecvTail ==
				g_CECRecvBuf + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail = g_CECRecvBuf;
			else
				g_pCECRecvTail++;


			time_out_counter++;
			if (time_out_counter >= TIME_OUT_THRESHOLD)
				break;
		} while (*g_pCECRecvTail == 0);
	}

	return DataLen;
}

unchar downstream_cec_sendframe(unchar *pFrameData, unchar Len)
{
	unchar i, j;
	unchar *pDataTmp;
	unchar LenTmp;
	unchar c;
	uint time_out_counter;
	time_out_counter = 0;


	/* According to CEC 7.1, re-transmission
	can be attempted up to 5 times*/
	for (i = 5; i; i--) {
		for (pDataTmp = pFrameData, LenTmp = Len; LenTmp; LenTmp--) {
			c = *pDataTmp;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x80, 1, &c);
			pDataTmp++;
		}

		/*make sure CEC rx is idle*/
		time_out_counter = 0;
		do {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x71, 1, &c);
			time_out_counter++;
			if (time_out_counter >= TIME_OUT_THRESHOLD)
				break;
		} while (c&0x80);


		/*clear tx done interrupt first*/
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x10, 1, &c);
		c |= 0x80;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x10, 1, &c);


		/* switch to CEC tx mode*/
		for (j = 0; j < 20; j++) {

			c = CEC_DEVICE_TYPE<<4;
			c |= 0x04;

			if (AUX_OK != sp_tx_aux_dpcdwrite_bytes
				(0x00, 0x05, 0x70, 1, &c)) {
				DEV_ERR("switch to cec tx write error!\n");
				continue;
			} else
				break;
		}


		time_out_counter = 0;
		do {
			if (AUX_OK == sp_tx_aux_dpcdread_bytes
				(0x00, 0x05, 0x10, 1, &c)) {
				time_out_counter++;
				if (time_out_counter >= TIME_OUT_THRESHOLD)
					break;
			} else
				break;
		} while (!(c&0x80));

		/*clear the CEC TX done interrupt*/
		c = 0x80;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x10, 1, &c);

		/*Get CEC TX status*/
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x11, 1, &c);
		/*DEV_DBG("DPCD 00511 = %.2x\n",(uint)c);*/

		if ((c&0x03) == 0) {
			break;
		} else {
			/*clear the CEC error interrupt*/
			c = 0x03;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x11, 1, &c);

			/*reset CEC*/
			c = CEC_DEVICE_TYPE << 4;
			c |= 0x05;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c);

		}
	}

	/* switch to CEC rx mode*/
	for (j = 0; j < 20; j++) {

		c = CEC_DEVICE_TYPE<<4;
		c |= 0x08;
		if (AUX_OK !=
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c)) {
			DEV_ERR("switch to cec rx write error!\n");
			continue;
		} else
			break;
	}



	if (i != 0)
		return 0;
	else
		return 1;

}



/*for HDMI RX CEC*/
unchar upstream_hdmi_cec_writemessage(struct tagCECFrame *pFrame, unchar Len)
{
	unchar t;

	if (bCECStatus == CEC_NOT_READY)
		return 0;

	t = upstream_cec_sendframe((unchar *)pFrame, Len);
	return t;
}

void upstream_hdmi_cec_readmessage(void)
{
	struct tagCECFrame *pFrame;
	unchar len;


	if (bCECStatus == CEC_NOT_READY)	{
		DEV_ERR("cec not ready!\n");
		return;
	}


	pFrame = &g_CECFrame_HDMI;
	upstream_cec_readfifo();

	while ((len = upstream_cec_recvframe((unchar *)pFrame)) != 0) {
		if (len >= 1) {
#ifdef CEC_PHYCISAL_ADDRESS_INSERT
			uptream_cec_parsemessage();
#endif
			downStream_hdmi_cec_writemessage(&g_CECFrame_HDMI, len);
		}
	}
}



void upstream_cec_readfifo(void)
{
	unchar cStatus, c;
	unchar i;
	unchar FIFOLen;

	uint time_out_counter;

	time_out_counter = 0;

	sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &cStatus);


	do {

		FIFOLen = cStatus&0x0F;

		if ((FIFOLen == 0) && (cStatus&0x20))
			FIFOLen = 0x10;


		FIFOLen <<= 1;
		for (i = 0; i < FIFOLen; i++) {
			sp_read_reg(RX_P0, HDMI_RX_CEC_FIFO_REG, &c);
			*g_pCECRecvHead_HDMI = c;

			if (g_pCECRecvHead_HDMI ==
				g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvHead_HDMI = g_CECRecvBuf_HDMI;
			else
				g_pCECRecvHead_HDMI++;

			if (g_pCECRecvHead_HDMI == g_pCECRecvTail_HDMI)
				DEV_ERR("buffer full!\r\n");

		}
		sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &cStatus);

		time_out_counter++;
		if (time_out_counter > LOCAL_REG_TIME_OUT_THRESHOLD)
			break;

	} while ((cStatus&0x80)|(cStatus&0x0F));

}

unchar upstream_cec_checkfullframe(void)
{
	unchar *pTmp;
	unchar i;
	uint time_out_counter;

	time_out_counter = 0;

	i = 0;
	pTmp = g_pCECRecvTail_HDMI;
	while (pTmp != g_pCECRecvHead_HDMI) {

		if (*pTmp != 0)
			i++;

		if (pTmp == g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf_HDMI;
		else
			pTmp++;


		if (pTmp == g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
			pTmp = g_CECRecvBuf_HDMI;
		else
			pTmp++;


		time_out_counter++;
		if (time_out_counter >= TIME_OUT_THRESHOLD)
			break;


	}


	if (i >= 1)
		return 1;
	else
		return 0;

}


unchar upstream_cec_recvframe(unchar *pFrameData)
{
	unchar DataLen;
	uint time_out_counter;
	time_out_counter = 0;
	DataLen = 0;


	if (upstream_cec_checkfullframe()) {
		do {
			if (g_pCECRecvTail_HDMI == g_pCECRecvHead_HDMI)
				break;

			if (g_pCECRecvTail_HDMI ==
				g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail_HDMI = g_CECRecvBuf_HDMI;
			else
				g_pCECRecvTail_HDMI++;



			*pFrameData = *g_pCECRecvTail_HDMI;
			DataLen++;
			pFrameData++;

			if (g_pCECRecvTail_HDMI ==
				g_CECRecvBuf_HDMI + CEC_RECV_BUF_SIZE - 1)
				g_pCECRecvTail_HDMI = g_CECRecvBuf_HDMI;
			else
				g_pCECRecvTail_HDMI++;


			time_out_counter++;
			if (time_out_counter >= TIME_OUT_THRESHOLD)
				break;
		} while (*g_pCECRecvTail_HDMI == 0);
	}

	return DataLen;
}



unchar upstream_cec_sendframe(unchar *pFrameData, unchar Len)
{
	unchar i;
	unchar *pDataTmp;
	unchar LenTmp;
	unchar c;
	uint time_out_counter;

	time_out_counter = 0;

	/* According to CEC 7.1, re-transmission
	can be attempted up to 5 times*/
	for (i = 5; i; i--) {
		for (pDataTmp = pFrameData, LenTmp = Len; LenTmp; LenTmp--) {
			c = *pDataTmp;
			sp_write_reg(RX_P0, HDMI_RX_CEC_FIFO_REG, c);
			pDataTmp++;
		}

		time_out_counter = 0;

		do {
			sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &c);
			time_out_counter++;
			if (time_out_counter >= LOCAL_REG_TIME_OUT_THRESHOLD)
				break;

		} while (c&0x80);


		/*clear tx done int first*/
		sp_read_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, &c);
		sp_write_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, (c|0x01));



		/*switch to CEC tx mode*/
		sp_read_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, &c);
		c &= 0xf3;
		c |= 0x04;
		sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, c);

		time_out_counter = 0;
		do {

			sp_read_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, &c);

			time_out_counter++;
			if (time_out_counter >= LOCAL_REG_TIME_OUT_THRESHOLD)
				break;
		} while (!(c&0x01));
		/*clear the CEC TX done interrupt*/
		c = 0x01;
		sp_write_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, c);


		/*Get CEC TX status*/
		sp_read_reg(RX_P0, HDMI_RX_CEC_TX_STATUS_REG, &c);
		if ((c&0x040) == 0) {
			break;
		} else {
			DEV_ERR("send failed, reset CEC\n");

			/*reset CEC*/
			sp_read_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, &c);
			c |= 0x01;
			sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, c);

		}
	}


	/* switch to CEC rx mode*/
	sp_read_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, &c);
	c &= 0xf3;
	c |= 0x08;
	sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, c);

	if (i != 0)
		return 0;
	else
		return 1;

}

#ifdef CEC_PHYCISAL_ADDRESS_INSERT
void downstream_cec_phy_add_set(unchar addr0, unchar addr1)
{
	DEV_DBG("set phy addr[0] = %.2x,phy addr[1] = %.2x,\n",
		(uint)addr0, (uint)addr1);
	downstream_physicaladdrh = addr0;
	downstream_physicaladdrl = addr1;
}
#endif



#ifdef CEC_PHYCISAL_ADDRESS_INSERT
void uptream_cec_parsemessage(void)
{

	switch (g_CECFrame_HDMI.msg.raw[0]) {
	case CEC_OPCODE_REPORT_PHYSICAL_ADDRESS:
	DEV_DBG("parse phy addr[0] = %.2x, phy addr[1] = %.2x\n",
		(uint)downstream_physicaladdrh,
		(uint)downstream_physicaladdrl);
	if ((downstream_physicaladdrh != 0xff)
		&& (downstream_physicaladdrl != 0xff)) {
		g_CECFrame_HDMI.msg.raw[1] = downstream_physicaladdrh;
		g_CECFrame_HDMI.msg.raw[2] = downstream_physicaladdrl;
	}

	break;

	default:
	break;
	}
}
#endif

void Downstream_Report_Physical_Addr(void)
{
	g_CECFrame.header.init = g_LogicAddr;
	g_CECFrame.header.dest = 0x0F;
	g_CECFrame.msg.raw[0] = CEC_OPCODE_REPORT_PHYSICAL_ADDRESS;
	g_CECFrame_HDMI.msg.raw[1] = 0x20;
	g_CECFrame_HDMI.msg.raw[2] = 0x00;
	g_CECFrame_HDMI.msg.raw[3] = CEC_DEVICE_TYPE_PLAYBACK_DEVICE;


	downStream_hdmi_cec_writemessage(&g_CECFrame, 5);

}




