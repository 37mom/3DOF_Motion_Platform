/*
 * data_tyoes.h
 *
 * Created: 4/1/2018 10:32:28 AM
 *  Author: moham
 */ 


#ifndef DATA_TYOES_H_
#define DATA_TYOES_H_


/** \ingroup avr_stdint
    8-bit signed type. */

typedef signed char int8_t;

/** \ingroup avr_stdint
    8-bit unsigned type. */

typedef unsigned char uint8_t;

/** \ingroup avr_stdint
    16-bit signed type. */

typedef signed int int16_t;

/** \ingroup avr_stdint
    16-bit unsigned type. */

typedef unsigned int uint16_t;

/** \ingroup avr_stdint
    32-bit signed type. */

typedef signed long int int32_t;

/** \ingroup avr_stdint
    32-bit unsigned type. */

typedef unsigned long int uint32_t;

/** \ingroup avr_stdint
    64-bit signed type.
    \note This type is not available when the compiler
    option -mint8 is in effect. */

typedef signed long long int int64_t;

/** \ingroup avr_stdint
    64-bit unsigned type.
    \note This type is not available when the compiler
    option -mint8 is in effect. */

typedef unsigned long long int uint64_t;


#endif /* DATA_TYOES_H_ */