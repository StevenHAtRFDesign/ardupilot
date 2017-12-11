/*
 * AP_EngineMon_Listener.h
 *
 *  Created on: 6Dec.,2017
 *      Author: snh
 */

#ifndef LIBRARIES_AP_ENGINEMON_AP_ENGINEMON_LISTENER_H_
#define LIBRARIES_AP_ENGINEMON_AP_ENGINEMON_LISTENER_H_

#include <uavcan/uavcan.hpp>
#include <stdint.h>

class DataMsgListener : public uavcan::TransferListener
{
public:
	DataMsgListener(uavcan::TransferPerfCounter& perf,
			uavcan::IPoolAllocator& allocator);

protected:
	virtual void handleIncomingTransfer(uavcan::IncomingTransfer& transfer);

private:
	uavcan::DataTypeSignature _DTS;
	uavcan::DataTypeID _DTID;
	uavcan::DataTypeDescriptor _DTD;
	uavcan::LimitedPoolAllocator _LPA;
};


#endif /* LIBRARIES_AP_ENGINEMON_AP_ENGINEMON_LISTENER_H_ */
