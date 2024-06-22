/*
 * bit_fiddling.h
 *
 * Created: 3/16/2018 7:09:03 PM
 *  Author: moham
 */ 


#ifndef BIT_FIDDLING_H_
#define BIT_FIDDLING_H_

#define SET_BIT(port,bit_no)			port|=(1<<bit_no)
#define CLEAR_BIT(port,bit_no)			port&=~(1<<bit_no)
#define TOGGLE_BIT(port,bit_no)			port^=(1<<bit_no)
#define GET_BIT(port,bit_no)			(port >> bit_no) & 1
#define CHANGE_BIT(port,bit_no,dir)		port ^= (-dir ^ port) & (1 << bit_no);

#endif /* BIT_FIDDLING_H_ */