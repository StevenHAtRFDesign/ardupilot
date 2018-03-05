/*
 * AP_Function.h
 *
 *  Created on: 22Nov.,2017
 *      Author: snh
 */

#ifndef LIBRARIES_AP_FUNCTION_AP_FUNCTION_H_
#define LIBRARIES_AP_FUNCTION_AP_FUNCTION_H_

template <class T>
class AP_Function
{
public:
	virtual T Function(T) = 0;
};



#endif /* LIBRARIES_AP_FUNCTION_AP_FUNCTION_H_ */
