/*
 * AP_EngineMon.h
 *
 *  Created on: 29Nov.,2017
 *      Author: snh
 */

#ifndef LIBRARIES_AP_ENGINEMON_AP_ENGINEMON_H_
#define LIBRARIES_AP_ENGINEMON_AP_ENGINEMON_H_

#include <stdint.h>
#include "GCS_MAVLink/GCS.h"
#include <uavcan/uavcan.hpp>

class AP_EngineMon
{
public:
	AP_EngineMon();
	void SetGCS(GCS *pGCS);
	void Process(void);
	void SetUAVCANNode(uavcan::Node<0> *pUAVCANNode);
private:
	GCS *_pGCS;
	uint64_t _LastTestMsgSent;
	uavcan::TransferPerfCounter PerfCounter;
	MsgListener *_pMsgListener;
	uavcan::Node<0> *_pUAVCANNode;
	void SendTestMsg(void);
	void RxICEEngineData(void);
};


#endif /* LIBRARIES_AP_ENGINEMON_AP_ENGINEMON_H_ */
