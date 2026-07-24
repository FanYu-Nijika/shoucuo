#ifndef _zf_common_int_h_
#define _zf_common_int_h_

#include <stdint.h>
#include "../infineon_libraries/iLLD/TC26B/Tricore/Cpu/Std/Platform_Types.h"

/* Short names commonly used by embedded C projects. */
typedef signed char schar;
typedef unsigned char uchar;
typedef signed short sshort;
typedef unsigned short ushort;
typedef signed int sint;
typedef unsigned int uint;
typedef signed long slong;
typedef unsigned long ulong;
typedef signed long long ll;
typedef unsigned long long ull;

#ifndef TRUE
#define TRUE (1u)
#endif

#ifndef FALSE
#define FALSE (0u)
#endif

#endif
