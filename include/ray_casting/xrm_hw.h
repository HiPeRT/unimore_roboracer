// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Tool Version Limit: 2022.04
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// control
// 0x000 : Control signals
//         bit 0  - ap_start (Read/Write/COH)
//         bit 1  - ap_done (Read/COR)
//         bit 2  - ap_idle (Read)
//         bit 3  - ap_ready (Read/COR)
//         bit 7  - auto_restart (Read/Write)
//         bit 9  - interrupt (Read)
//         others - reserved
// 0x004 : Global Interrupt Enable Register
//         bit 0  - Global Interrupt Enable (Read/Write)
//         others - reserved
// 0x008 : IP Interrupt Enable Register (Read/Write)
//         bit 0 - enable ap_done interrupt (Read/Write)
//         bit 1 - enable ap_ready interrupt (Read/Write)
//         others - reserved
// 0x00c : IP Interrupt Status Register (Read/COR)
//         bit 0 - ap_done (Read/COR)
//         bit 1 - ap_ready (Read/COR)
//         others - reserved
// 0x010 : Data signal of distMap
//         bit 31~0 - distMap[31:0] (Read/Write)
// 0x014 : Data signal of distMap
//         bit 31~0 - distMap[63:32] (Read/Write)
// 0x018 : reserved
// 0x01c : Data signal of x
//         bit 31~0 - x[31:0] (Read/Write)
// 0x020 : Data signal of x
//         bit 31~0 - x[63:32] (Read/Write)
// 0x024 : reserved
// 0x028 : Data signal of y
//         bit 31~0 - y[31:0] (Read/Write)
// 0x02c : Data signal of y
//         bit 31~0 - y[63:32] (Read/Write)
// 0x030 : reserved
// 0x034 : Data signal of yaw
//         bit 31~0 - yaw[31:0] (Read/Write)
// 0x038 : Data signal of yaw
//         bit 31~0 - yaw[63:32] (Read/Write)
// 0x03c : reserved
// 0x040 : Data signal of rays
//         bit 31~0 - rays[31:0] (Read/Write)
// 0x044 : Data signal of rays
//         bit 31~0 - rays[63:32] (Read/Write)
// 0x048 : reserved
// 0x04c : Data signal of rays_angle
//         bit 31~0 - rays_angle[31:0] (Read/Write)
// 0x050 : Data signal of rays_angle
//         bit 31~0 - rays_angle[63:32] (Read/Write)
// 0x054 : reserved
// 0x058 : Data signal of rm_mode
//         bit 31~0 - rm_mode[31:0] (Read/Write)
// 0x05c : reserved
// 0x060 : Data signal of map_height
//         bit 31~0 - map_height[31:0] (Read/Write)
// 0x064 : reserved
// 0x068 : Data signal of map_width
//         bit 31~0 - map_width[31:0] (Read/Write)
// 0x06c : reserved
// 0x070 : Data signal of n_particles
//         bit 31~0 - n_particles[31:0] (Read/Write)
// 0x074 : reserved
// 0x078 : Data signal of orig_x
//         bit 31~0 - orig_x[31:0] (Read/Write)
// 0x07c : reserved
// 0x080 : Data signal of orig_y
//         bit 31~0 - orig_y[31:0] (Read/Write)
// 0x084 : reserved
// 0x088 : Data signal of map_resolution
//         bit 31~0 - map_resolution[31:0] (Read/Write)
// 0x08c : reserved
// 0x090 : Data signal of private_map_pe0
//         bit 31~0 - private_map_pe0[31:0] (Read/Write)
// 0x094 : Data signal of private_map_pe0
//         bit 31~0 - private_map_pe0[63:32] (Read/Write)
// 0x098 : reserved
// 0x09c : Data signal of private_map_pe1
//         bit 31~0 - private_map_pe1[31:0] (Read/Write)
// 0x0a0 : Data signal of private_map_pe1
//         bit 31~0 - private_map_pe1[63:32] (Read/Write)
// 0x0a4 : reserved
// 0x0a8 : Data signal of private_map_pe2
//         bit 31~0 - private_map_pe2[31:0] (Read/Write)
// 0x0ac : Data signal of private_map_pe2
//         bit 31~0 - private_map_pe2[63:32] (Read/Write)
// 0x0b0 : reserved
// 0x0b4 : Data signal of private_map_pe3
//         bit 31~0 - private_map_pe3[31:0] (Read/Write)
// 0x0b8 : Data signal of private_map_pe3
//         bit 31~0 - private_map_pe3[63:32] (Read/Write)
// 0x0bc : reserved
// 0x0c0 : Data signal of private_map_pe4
//         bit 31~0 - private_map_pe4[31:0] (Read/Write)
// 0x0c4 : Data signal of private_map_pe4
//         bit 31~0 - private_map_pe4[63:32] (Read/Write)
// 0x0c8 : reserved
// 0x0cc : Data signal of private_map_pe5
//         bit 31~0 - private_map_pe5[31:0] (Read/Write)
// 0x0d0 : Data signal of private_map_pe5
//         bit 31~0 - private_map_pe5[63:32] (Read/Write)
// 0x0d4 : reserved
// 0x0d8 : Data signal of private_map_pe6
//         bit 31~0 - private_map_pe6[31:0] (Read/Write)
// 0x0dc : Data signal of private_map_pe6
//         bit 31~0 - private_map_pe6[63:32] (Read/Write)
// 0x0e0 : reserved
// 0x0e4 : Data signal of private_map_pe7
//         bit 31~0 - private_map_pe7[31:0] (Read/Write)
// 0x0e8 : Data signal of private_map_pe7
//         bit 31~0 - private_map_pe7[63:32] (Read/Write)
// 0x0ec : reserved
// 0x0f0 : Data signal of private_map_pe8
//         bit 31~0 - private_map_pe8[31:0] (Read/Write)
// 0x0f4 : Data signal of private_map_pe8
//         bit 31~0 - private_map_pe8[63:32] (Read/Write)
// 0x0f8 : reserved
// 0x0fc : Data signal of private_map_pe9
//         bit 31~0 - private_map_pe9[31:0] (Read/Write)
// 0x100 : Data signal of private_map_pe9
//         bit 31~0 - private_map_pe9[63:32] (Read/Write)
// 0x104 : reserved
// 0x108 : Data signal of private_map_pe10
//         bit 31~0 - private_map_pe10[31:0] (Read/Write)
// 0x10c : Data signal of private_map_pe10
//         bit 31~0 - private_map_pe10[63:32] (Read/Write)
// 0x110 : reserved
// 0x114 : Data signal of private_map_pe11
//         bit 31~0 - private_map_pe11[31:0] (Read/Write)
// 0x118 : Data signal of private_map_pe11
//         bit 31~0 - private_map_pe11[63:32] (Read/Write)
// 0x11c : reserved
// 0x120 : Data signal of private_map_pe12
//         bit 31~0 - private_map_pe12[31:0] (Read/Write)
// 0x124 : Data signal of private_map_pe12
//         bit 31~0 - private_map_pe12[63:32] (Read/Write)
// 0x128 : reserved
// 0x12c : Data signal of private_map_pe13
//         bit 31~0 - private_map_pe13[31:0] (Read/Write)
// 0x130 : Data signal of private_map_pe13
//         bit 31~0 - private_map_pe13[63:32] (Read/Write)
// 0x134 : reserved
// 0x138 : Data signal of private_map_pe14
//         bit 31~0 - private_map_pe14[31:0] (Read/Write)
// 0x13c : Data signal of private_map_pe14
//         bit 31~0 - private_map_pe14[63:32] (Read/Write)
// 0x140 : reserved
// 0x144 : Data signal of private_map_pe15
//         bit 31~0 - private_map_pe15[31:0] (Read/Write)
// 0x148 : Data signal of private_map_pe15
//         bit 31~0 - private_map_pe15[63:32] (Read/Write)
// 0x14c : reserved
// 0x150 : Data signal of private_map_pe16
//         bit 31~0 - private_map_pe16[31:0] (Read/Write)
// 0x154 : Data signal of private_map_pe16
//         bit 31~0 - private_map_pe16[63:32] (Read/Write)
// 0x158 : reserved
// 0x15c : Data signal of private_map_pe17
//         bit 31~0 - private_map_pe17[31:0] (Read/Write)
// 0x160 : Data signal of private_map_pe17
//         bit 31~0 - private_map_pe17[63:32] (Read/Write)
// 0x164 : reserved
// 0x168 : Data signal of private_map_pe18
//         bit 31~0 - private_map_pe18[31:0] (Read/Write)
// 0x16c : Data signal of private_map_pe18
//         bit 31~0 - private_map_pe18[63:32] (Read/Write)
// 0x170 : reserved
// 0x174 : Data signal of private_map_pe19
//         bit 31~0 - private_map_pe19[31:0] (Read/Write)
// 0x178 : Data signal of private_map_pe19
//         bit 31~0 - private_map_pe19[63:32] (Read/Write)
// 0x17c : reserved
// 0x180 : Data signal of private_map_pe20
//         bit 31~0 - private_map_pe20[31:0] (Read/Write)
// 0x184 : Data signal of private_map_pe20
//         bit 31~0 - private_map_pe20[63:32] (Read/Write)
// 0x188 : reserved
// 0x18c : Data signal of private_map_pe21
//         bit 31~0 - private_map_pe21[31:0] (Read/Write)
// 0x190 : Data signal of private_map_pe21
//         bit 31~0 - private_map_pe21[63:32] (Read/Write)
// 0x194 : reserved
// 0x198 : Data signal of private_map_pe22
//         bit 31~0 - private_map_pe22[31:0] (Read/Write)
// 0x19c : Data signal of private_map_pe22
//         bit 31~0 - private_map_pe22[63:32] (Read/Write)
// 0x1a0 : reserved
// 0x1a4 : Data signal of private_map_pe23
//         bit 31~0 - private_map_pe23[31:0] (Read/Write)
// 0x1a8 : Data signal of private_map_pe23
//         bit 31~0 - private_map_pe23[63:32] (Read/Write)
// 0x1ac : reserved
// 0x1b0 : Data signal of private_map_pe24
//         bit 31~0 - private_map_pe24[31:0] (Read/Write)
// 0x1b4 : Data signal of private_map_pe24
//         bit 31~0 - private_map_pe24[63:32] (Read/Write)
// 0x1b8 : reserved
// 0x1bc : Data signal of private_map_pe25
//         bit 31~0 - private_map_pe25[31:0] (Read/Write)
// 0x1c0 : Data signal of private_map_pe25
//         bit 31~0 - private_map_pe25[63:32] (Read/Write)
// 0x1c4 : reserved
// 0x1c8 : Data signal of private_map_pe26
//         bit 31~0 - private_map_pe26[31:0] (Read/Write)
// 0x1cc : Data signal of private_map_pe26
//         bit 31~0 - private_map_pe26[63:32] (Read/Write)
// 0x1d0 : reserved
// 0x1d4 : Data signal of private_map_pe27
//         bit 31~0 - private_map_pe27[31:0] (Read/Write)
// 0x1d8 : Data signal of private_map_pe27
//         bit 31~0 - private_map_pe27[63:32] (Read/Write)
// 0x1dc : reserved
// 0x1e0 : Data signal of private_map_pe28
//         bit 31~0 - private_map_pe28[31:0] (Read/Write)
// 0x1e4 : Data signal of private_map_pe28
//         bit 31~0 - private_map_pe28[63:32] (Read/Write)
// 0x1e8 : reserved
// 0x1ec : Data signal of private_map_pe29
//         bit 31~0 - private_map_pe29[31:0] (Read/Write)
// 0x1f0 : Data signal of private_map_pe29
//         bit 31~0 - private_map_pe29[63:32] (Read/Write)
// 0x1f4 : reserved
// 0x1f8 : Data signal of private_map_pe30
//         bit 31~0 - private_map_pe30[31:0] (Read/Write)
// 0x1fc : Data signal of private_map_pe30
//         bit 31~0 - private_map_pe30[63:32] (Read/Write)
// 0x200 : reserved
// 0x204 : Data signal of private_map_pe31
//         bit 31~0 - private_map_pe31[31:0] (Read/Write)
// 0x208 : Data signal of private_map_pe31
//         bit 31~0 - private_map_pe31[63:32] (Read/Write)
// 0x20c : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XRM_CONTROL_ADDR_AP_CTRL               0x000
#define XRM_CONTROL_ADDR_GIE                   0x004
#define XRM_CONTROL_ADDR_IER                   0x008
#define XRM_CONTROL_ADDR_ISR                   0x00c
#define XRM_CONTROL_ADDR_DISTMAP_DATA          0x010
#define XRM_CONTROL_BITS_DISTMAP_DATA          64
#define XRM_CONTROL_ADDR_X_DATA                0x01c
#define XRM_CONTROL_BITS_X_DATA                64
#define XRM_CONTROL_ADDR_Y_DATA                0x028
#define XRM_CONTROL_BITS_Y_DATA                64
#define XRM_CONTROL_ADDR_YAW_DATA              0x034
#define XRM_CONTROL_BITS_YAW_DATA              64
#define XRM_CONTROL_ADDR_RAYS_DATA             0x040
#define XRM_CONTROL_BITS_RAYS_DATA             64
#define XRM_CONTROL_ADDR_RAYS_ANGLE_DATA       0x04c
#define XRM_CONTROL_BITS_RAYS_ANGLE_DATA       64
#define XRM_CONTROL_ADDR_RM_MODE_DATA          0x058
#define XRM_CONTROL_BITS_RM_MODE_DATA          32
#define XRM_CONTROL_ADDR_MAP_HEIGHT_DATA       0x060
#define XRM_CONTROL_BITS_MAP_HEIGHT_DATA       32
#define XRM_CONTROL_ADDR_MAP_WIDTH_DATA        0x068
#define XRM_CONTROL_BITS_MAP_WIDTH_DATA        32
#define XRM_CONTROL_ADDR_N_PARTICLES_DATA      0x070
#define XRM_CONTROL_BITS_N_PARTICLES_DATA      32
#define XRM_CONTROL_ADDR_ORIG_X_DATA           0x078
#define XRM_CONTROL_BITS_ORIG_X_DATA           32
#define XRM_CONTROL_ADDR_ORIG_Y_DATA           0x080
#define XRM_CONTROL_BITS_ORIG_Y_DATA           32
#define XRM_CONTROL_ADDR_MAP_RESOLUTION_DATA   0x088
#define XRM_CONTROL_BITS_MAP_RESOLUTION_DATA   32
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE0_DATA  0x090
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE0_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE1_DATA  0x09c
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE1_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE2_DATA  0x0a8
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE2_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE3_DATA  0x0b4
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE3_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE4_DATA  0x0c0
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE4_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE5_DATA  0x0cc
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE5_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE6_DATA  0x0d8
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE6_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE7_DATA  0x0e4
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE7_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE8_DATA  0x0f0
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE8_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE9_DATA  0x0fc
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE9_DATA  64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE10_DATA 0x108
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE10_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE11_DATA 0x114
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE11_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE12_DATA 0x120
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE12_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE13_DATA 0x12c
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE13_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE14_DATA 0x138
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE14_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE15_DATA 0x144
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE15_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE16_DATA 0x150
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE16_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE17_DATA 0x15c
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE17_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE18_DATA 0x168
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE18_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE19_DATA 0x174
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE19_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE20_DATA 0x180
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE20_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE21_DATA 0x18c
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE21_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE22_DATA 0x198
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE22_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE23_DATA 0x1a4
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE23_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE24_DATA 0x1b0
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE24_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE25_DATA 0x1bc
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE25_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE26_DATA 0x1c8
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE26_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE27_DATA 0x1d4
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE27_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE28_DATA 0x1e0
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE28_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE29_DATA 0x1ec
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE29_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE30_DATA 0x1f8
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE30_DATA 64
#define XRM_CONTROL_ADDR_PRIVATE_MAP_PE31_DATA 0x204
#define XRM_CONTROL_BITS_PRIVATE_MAP_PE31_DATA 64

