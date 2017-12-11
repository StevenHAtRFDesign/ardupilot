/*
 * AP_EngineMon_Listener.cpp
 *
 *  Created on: 6Dec.,2017
 *      Author: snh
 */

#include "AP_EngineMon_Listener.h"


DataMsgListener::DataMsgListener(uavcan::TransferPerfCounter& perf,
									uavcan::IPoolAllocator& allocator)
	: _DTID(20100),
	  _DTD(uavcan::DataTypeKindMessage, _DTID, _DTS, ""),
	  uavcan::TransferListener(perf, _DTD, 0, _LPA)
{

}

void DataMsgListener::handleIncomingTransfer(uavcan::IncomingTransfer& transfer)
{

}
