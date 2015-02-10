/**
********************************************************************************
\file   dualprocshm-linuxkernel.h

\brief  Dual Processor Library Target support Header - For Linux Kernel target

This header file provides specific macros for Linux Kernel CPU.

*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

#ifndef _INC_dualprocshm_winkernel_H_
#define _INC_dualprocshm_winkernel_H_

//#error
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <ndis.h>
#include <ndis-intf.h>
//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

/* Memory size */
// We don't use memory header for windows
#define MAX_COMMON_MEM_SIZE        2048                         ///< Max common memory size
#define MAX_DYNAMIC_BUFF_COUNT     20                           ///< Number of maximum dynamic buffers
#define MAX_DYNAMIC_BUFF_SIZE      MAX_DYNAMIC_BUFF_COUNT * 4   ///< Max dynamic buffer size

/// memory
#define DUALPROCSHM_MALLOC(size)              ExAllocatePool(NonPagedPool, (size))
#define DUALPROCSHM_FREE(ptr)                 ExFreePool(ptr)
#define DUALPROCSHM_MEMCPY(dest, src, siz)    NdisMoveMemory(dest, src, siz)
/// sleep
#define DUALPROCSHM_USLEEP(x)                 NdisMSleep((UINT32)x)

/// IO operations
#define DPSHM_READ8(base)                     READ_REGISTER_UCHAR((UINT8*)base);
#define DPSHM_WRITE8(base, val)               WRITE_REGISTER_UCHAR((UINT8*)base, val);
#define DPSHM_READ16(base)                    READ_REGISTER_USHORT((UINT16*)base);
#define DPSHM_WRITE16(base, val)              WRITE_REGISTER_USHORT((UINT16*)base, val);
#define DPSHM_READ32(base)                    READ_REGISTER_ULONG((UINT32*)base);
#define DPSHM_WRITE32(base, val)              WRITE_REGISTER_ULONG((UINT32*)base, val);
#define DPSHM_ENABLE_INTR(fEnable)

/// cache handling
#define DUALPROCSHM_FLUSH_DCACHE_RANGE(base, range)

#define DUALPROCSHM_INVALIDATE_DCACHE_RANGE(base, range)

#define DPSHM_REG_SYNC_INTR(callback, arg)

#define DPSHM_ENABLE_SYNC_INTR()

#define DPSHM_DISABLE_SYNC_INTR()

#define DPSHM_ENABLE_HOST_SYNC_IRQ()
#define DPSHM_DISABLE_HOST_SYNC_IRQ()

#ifndef NDEBUG
#define TRACE(...)    DbgPrint(__VA_ARGS__)
#else
#define TRACE(...)
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#endif /* _INC_DUALPROCSHM_ARM_H_ */
