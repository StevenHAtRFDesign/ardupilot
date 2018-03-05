/*
 * AP_Value.h
 *
 *  Created on: 13Nov.,2017
 *      Author: snh
 */

#ifndef LIBRARIES_AP_VALUE_AP_VALUE_H_
#define LIBRARIES_AP_VALUE_AP_VALUE_H_

template <class T>
class AP_Value
{
public:
	virtual T GetValue(void) = 0;
};



#endif /* LIBRARIES_AP_VALUE_AP_VALUE_H_ */
