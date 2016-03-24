/*
 * windows.h
 *
 *  Created on: Aug 10, 2015
 *      Author: robot
 */
#include <stdint.h>
#ifndef WINDOWS_H_
#define WINDOWS_H_

typedef unsigned int DWORD;
typedef unsigned char UCHAR;
typedef uint16_t UINT16;
typedef unsigned int UINT;
typedef long LONG;
typedef long long LONGLONG;




typedef union _LARGE_INTEGER {
  struct {
    DWORD LowPart;
    LONG  HighPart;
  };
  struct {
    DWORD LowPart;
    LONG  HighPart;
  } u;
  LONGLONG QuadPart;
} LARGE_INTEGER, *PLARGE_INTEGER;



#endif /* WINDOWS_H_ */
