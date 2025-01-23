// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.1 (64-bit)
// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XRMK_H
#define XRMK_H



/***************************** Include Files *********************************/
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>

#include "xio.hpp"
#include "uio_map.hpp"
#include "xrm_hw.h"

/**************************** Type Definitions ******************************/
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#define XRmk_WriteReg(BaseAddress, RegOffset, Data) \
  Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XRmk_ReadReg(BaseAddress, RegOffset) \
  Xil_In32((BaseAddress) + (RegOffset))

/************************** Function Prototypes *****************************/
void XRmk_Start(UioMap *rm);
u32 XRmk_IsDone(UioMap *rm);

void XRmk_Set_distMap(UioMap *rm, u64 Data);
void XRmk_Set_x(UioMap *rm, u64 Data);

void XRmk_Set_y(UioMap *rm, u64 Data);

void XRmk_Set_yaw(UioMap *rm, u64 Data);

void XRmk_Set_rays(UioMap *rm, u64 Data);

void XRmk_Set_rays_angle(UioMap *rm, u64 Data);

void XRmk_Set_rm_mode(UioMap *rm, u32 Data);
void XRmk_Set_map_height(UioMap *rm, u32 Data);
void XRmk_Set_map_width(UioMap *rm, u32 Data);
void XRmk_Set_n_particles(UioMap *rm, u32 Data);

void XRmk_Set_orig_x(UioMap *rm, u32 Data);

void XRmk_Set_orig_y(UioMap *rm, u32 Data);

void XRmk_Set_map_resolution(UioMap *rm, u32 Data);

#endif
