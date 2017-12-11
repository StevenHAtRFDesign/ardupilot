/*
 * AP_EngineMon.cpp
 *
 *  Created on: 29Nov.,2017
 *      Author: snh
 *
 * Module for interfacing to the engine trend monitor(s) and forwarding data to the GCS.
 */

#include <stdio.h>
#include "AP_EngineMon.h"
#include "AP_HAL/System.h"

AP_EngineMon::AP_EngineMon()
{
	_pGCS = NULL;
	_LastTestMsgSent = 0;
}

void AP_EngineMon::SetGCS(GCS *pGCS)
{
	_pGCS = pGCS;

}

void AP_EngineMon::Process(void)
{
	uint64_t Now = AP_HAL::millis64();

	//if ((Now - _LastTestMsgSent) > 3000)
	{
		_LastTestMsgSent = Now;
		printf("sending test message\n");
		SendTestMsg();
	}
}

void AP_EngineMon::SendTestMsg(void)
{
	if (_pGCS != NULL)
	{
		uint8_t n;
		for (n = 0; n < _pGCS->num_gcs(); n++)
		{
			if (_pGCS->chan(n).initialised)
			{
				uint16_t Temp[16];
				printf(" - Initialised\n");
				mavlink_msg_ice_engine_data_send(_pGCS->chan(n).get_chan(),
						0, 0, 0, 0, 0, 0, 0, 0, Temp, Temp, 0, 0, 0, 0, 0, 0);
			}
		}
	}
}

void AP_EngineMon::SetUAVCANNode(uavcan::Node<0> *pUAVCANNode)
{
	_pUAVCANNode = pUAVCANNode;
	if (_pUAVCANNode != nullptr)
	{
		_pUAVCANNode->getDispatcher().registerMessageListener(&_MsgListener);
		uavcan::Subscriber<uavcan::protocol::debug::LogMessage> log_sub(NULL);
	}
}
