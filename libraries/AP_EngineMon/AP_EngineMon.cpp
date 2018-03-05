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
#include "AP_HAL/system.h"

#define AP_ENGINEMON_MAX_QTY_MONS 12
#define AP_ENGINEMON_ID_NONE 0

typedef uint64_t AP_EngineMon_ID_t;

typedef struct
{
	uint16_t ICEMotServoUS;	// 	# Output. Pulse Width Ch0. Petrol Engine Throttle
	uint16_t DCMotServoUS;	// 	# Output ? Pulse Width Ch1 ? Electric Engine Throttle
	float ICERPM;			//  # Input ? Rate  (Freq) Ch0 ? Engine RPM - float
	uint16_t ThrustReqUS;	//  # Input ? Rate (Width) Ch1 ? Thrust Input (Will use CAN later)
	uint16_t IgnCutReqUS;	//	# Input ? Rate(Width) Ch2 ? Ignition Cut (Will use CAN later)  (also will be Pulse counter in Copter Monitor)
	float Duty8BitCh0;		//	# Output ? PWM Duty Ch0 ? LED1 (or other) - % float
	float Duty8BitCh1;		//	# Output ? PWM Duty Ch1 ? LED2 (or other) - % float
	float Duty8BitCh2;		//	# Output ? PWM Duty Ch2 ? LED3 (or other) - % float
	uint16_t ICEVibX[16];	//	# Input ? Noise/Vibration Ch0 ? Engine X Vibration ? uint16[16]
	uint16_t ICEVibY[16];	//	# Input ? Noise/Vibration Ch1 ? Engine Y Vibration ? uint16[16]
	float ICEEngDegC;		//	# Input ? ADC Ch0 ? Engine Temp ? degC  float
	float ICEExhDegC;		//	# Input ? ADC Ch1 ? Exhaust Temp ? degC  float
	float Health;			//	# Inferred - Engine Health - 0-65535
	uint64_t FaultBits;		//	# Fault Bits ? 64 bit fields uint64
	uint32_t ServiceSecs;	//	# Inferred ? Service seconds ? seconds uint32
	bool ICEIgnCut;			//	# Output ? P Chan FET Ch0 ? Ignition Cut ? bool
} AP_EngineMon_StdData_t;

typedef struct
{
	AP_EngineMon_StdData_t SData;
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
} AP_EngineMon_Data_t;

typedef struct
{
	AP_EngineMon_Data_t Data;
	bool DataValid;
	AP_EngineMon_ID_t SrcNodeID;	//0 for empty slot.
} AP_EngineMon_Status_t;

AP_EngineMon_Status_t* AP_EngineMon_GetStatus(AP_EngineMon_ID_t ID);

AP_EngineMon_Status_t gStatus[AP_ENGINEMON_MAX_QTY_MONS];
GCS *gpGCS = NULL;
uint64_t _LastTestMsgSent;
bool _SubscriberRegistered = false;
bool _CANMsgRx = false;

void AP_EngineMon_Init(void)
{
	int n;
	for (n = 0; n < AP_ENGINEMON_MAX_QTY_MONS; n++)
	{
		gStatus[n].SrcNodeID = 0;
	}
}

AP_EngineMon_Status_t* AP_EngineMon_GetStatus(AP_EngineMon_ID_t ID)
{
	int n;
	for (n = 0; n < AP_ENGINEMON_MAX_QTY_MONS; n++)
	{
		if (gStatus[n].SrcNodeID == ID)
		{
			return gStatus + n;
		}
	}

	for (n = 0; n < AP_ENGINEMON_MAX_QTY_MONS; n++)
	{
		if (gStatus[n].SrcNodeID == AP_ENGINEMON_ID_NONE)
		{
			gStatus[n].SrcNodeID = ID;
			return gStatus + n;
		}
	}
	return NULL;
}

void AP_EngineMon_Rx_StdData(const uavcan::ReceivedDataStructure<rfd::equipment::eng_mon::UpdateData_t> &msg)
{
	int n;

	/*if (gpGCS != NULL)
	{
		uint8_t n;
		for (n = 0; n < gpGCS->num_gcs(); n++)
		{
			if (gpGCS->chan(n).initialised)
			{
				uint16_t Temp[16];
				printf(" - Initialised\n");
				mavlink_msg_ice_engine_data_send(gpGCS->chan(n).get_chan(),
					0, 0, 0, 0, 0, 0, 0, 0, Temp, Temp, 0, 0, 0, 0, 0, 0);
			}
		}
	}*/
	//msg.SData.DCMotServoUS.

	AP_EngineMon_Status_t *pStatus = AP_EngineMon_GetStatus(msg.getSrcNodeID().get());
	if (pStatus != NULL)
	{
		pStatus->Data.AccX = msg.AccX;
		pStatus->Data.AccY = msg.AccY;
		pStatus->Data.AccZ = msg.AccZ;
		pStatus->Data.SData.DCMotServoUS = msg.SData.DCMotServoUS;
		pStatus->Data.SData.Duty8BitCh0 = msg.SData.Duty8BitCh0;
		pStatus->Data.SData.Duty8BitCh1 = msg.SData.Duty8BitCh1;
		pStatus->Data.SData.Duty8BitCh2 = msg.SData.Duty8BitCh2;
		pStatus->Data.SData.FaultBits = msg.SData.FaultBits;
		pStatus->Data.SData.Health = msg.SData.Health;
		pStatus->Data.SData.ICEEngDegC = msg.SData.ICEEngDegC;
		pStatus->Data.SData.ICEExhDegC = msg.SData.ICEExhDegC;
		pStatus->Data.SData.ICEIgnCut = msg.SData.ICEIgnCut;
		pStatus->Data.SData.ICEMotServoUS = msg.SData.ICEMotServoUS;
		pStatus->Data.SData.ICERPM = msg.SData.ICERPM;
		for (n = 0; n < 16; n++)
		{
			pStatus->Data.SData.ICEVibX[n] = msg.SData.ICEVibX[n];
			pStatus->Data.SData.ICEVibY[n] = msg.SData.ICEVibY[n];
		}
		pStatus->Data.SData.IgnCutReqUS = msg.SData.IgnCutReqUS;
		pStatus->Data.SData.ServiceSecs = msg.SData.ServiceSecs;
		pStatus->Data.SData.ThrustReqUS = msg.SData.ThrustReqUS;
		pStatus->DataValid = true;
	}


	AP_EngineMon_NotifyCANMsgRx();
}

void AP_EngineMon_SetGCS(GCS *pGCS)
{
	gpGCS = pGCS;
}

/*void AP_EngineMon_SendTestMsg(void)
{
	if (gpGCS != NULL)
	{
		uint8_t n;
		for (n = 0; n < gpGCS->num_gcs(); n++)
		{
			if (gpGCS->chan(n).initialised)
			{
				uint16_t Temp[16];
				printf(" - Initialised\n");
				mavlink_msg_ice_engine_data_send(gpGCS->chan(n).get_chan(),
					0, 0, 0, 0, 0, 0, 0, 0, Temp, Temp, 0, 0, 0, 0, 0, 0);
			}
		}
	}
}*/

void AP_EngineMon_SendData(AP_EngineMon_Status_t *pStatus)
{
	if (gpGCS != NULL)
	{
		uint8_t n;
		for (n = 0; n < gpGCS->num_gcs(); n++)
		{
			if (gpGCS->chan(n).initialised)
			{
				uint16_t Temp[16];
				printf(" - Initialised\n");
				/*mavlink_msg_ice_engine_data_send(gpGCS->chan(n).get_chan(),
					0, 0, 0, 0, 0, 0, 0, 0, Temp, Temp, 0, 0, 0, 0, 0, 0);*/

				mavlink_msg_ice_engine_data_send(gpGCS->chan(n).get_chan(), 0,
					pStatus->Data.SData.ICEMotServoUS,
					pStatus->Data.SData.DCMotServoUS,
					pStatus->Data.SData.ICERPM,
					pStatus->Data.SData.ThrustReqUS,
					pStatus->Data.SData.IgnCutReqUS,
					pStatus->Data.SData.Duty8BitCh0,
					pStatus->Data.SData.Duty8BitCh1,
					pStatus->Data.SData.Duty8BitCh2,
					pStatus->Data.SData.ICEVibX,
					pStatus->Data.SData.ICEVibY,
					pStatus->Data.SData.ICEEngDegC,
					pStatus->Data.SData.ICEExhDegC,
					pStatus->Data.SData.Health,
					pStatus->Data.SData.FaultBits,
					pStatus->Data.SData.ServiceSecs,
					pStatus->Data.SData.ICEIgnCut);


			}
		}
	}

	pStatus->DataValid = false;
}

void AP_EngineMon_Process(void)
{
	int n;
	for (n = 0; n < AP_ENGINEMON_MAX_QTY_MONS; n++)
	{
		if (gStatus[n].SrcNodeID != AP_ENGINEMON_ID_NONE)
		{
			if (gStatus[n].DataValid)
			{
				AP_EngineMon_SendData(gStatus + n);
			}
		}
	}
}

void AP_EngineMon_NotifySubscriberRegistered(void)
{
	_SubscriberRegistered = true;
}

void AP_EngineMon_NotifyCANMsgRx(void)
{
	_CANMsgRx = true;
}

/*AP_EngineMon::AP_EngineMon()
{
	_pGCS = NULL;
	_LastTestMsgSent = 0;


}

void AP_EngineMon::GetUAVCAN()
{
	AP_Compass_UAVCAN *sensor = nullptr;

	if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
		for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
			if (hal.can_mgr[i] != nullptr) {
				AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
				if (uavcan != nullptr) {
					uavcan->_baro_listeners

					uint8_t freemag = uavcan->find_smallest_free_mag_node();
					if (freemag != UINT8_MAX) {
						sensor = new AP_Compass_UAVCAN(compass);
						if (sensor->register_uavcan_compass(i, freemag)) {
							debug_mag_uavcan(2, "AP_Compass_UAVCAN probed, drv: %d, node: %d\n\r", i, freemag);
							return sensor;
						} else {
							delete sensor;
							sensor = nullptr;
						}
					}
				}
			}
		}
	}

	return sensor;
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
}*/
