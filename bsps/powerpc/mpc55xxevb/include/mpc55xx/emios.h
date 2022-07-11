/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsPowerPCMPC55XX
 *
 * @brief Enhanced Modular Input Output Subsystem (eMIOS).
 */

/*
 * Copyright (c) 2009-2011 embedded brains GmbH.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LIBCPU_POWERPC_MPC55XX_EMIOS_H
#define LIBCPU_POWERPC_MPC55XX_EMIOS_H

#include <mpc55xx/regs.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef MPC55XX_HAS_EMIOS

/**
 * @name eMIOS - Modes
 *
 * @{
 */

#define MPC55XX_EMIOS_MODE_GPIO_INPUT 0U
#define MPC55XX_EMIOS_MODE_GPIO_OUTPUT 1U
#define MPC55XX_EMIOS_MODE_SAIC 2U
#define MPC55XX_EMIOS_MODE_SAOC 3U
#define MPC55XX_EMIOS_MODE_IPWM 4U
#define MPC55XX_EMIOS_MODE_IPM 5U
#define MPC55XX_EMIOS_MODE_DAOC_SECOND 6U
#define MPC55XX_EMIOS_MODE_DAOC_BOTH 7U
#define MPC55XX_EMIOS_MODE_PEA_ACCU_CONT 8U
#define MPC55XX_EMIOS_MODE_PEA_ACCU_SINGLE 9U
#define MPC55XX_EMIOS_MODE_PEA_COUNT_CONT 10U
#define MPC55XX_EMIOS_MODE_PEA_COUNT_SINGLE 11U
#define MPC55XX_EMIOS_MODE_QDEC_COUNT_DIR 12U
#define MPC55XX_EMIOS_MODE_QDEC_PHASE 13U
#define MPC55XX_EMIOS_MODE_WPTA 14U
#define MPC55XX_EMIOS_MODE_RESERVED_15 15U
#define MPC55XX_EMIOS_MODE_MC_UP_INT_CLK 16U
#define MPC55XX_EMIOS_MODE_MC_UP_EXT_CLK 17U
#define MPC55XX_EMIOS_MODE_RESERVED_18 18U
#define MPC55XX_EMIOS_MODE_RESERVED_19 19U
#define MPC55XX_EMIOS_MODE_MC_UP_DOWN_INT_CLK 20U
#define MPC55XX_EMIOS_MODE_MC_UP_DOWN_EXT_CLK 21U
#define MPC55XX_EMIOS_MODE_MC_UP_DOWN_CHANGE_INT_CLK 22U
#define MPC55XX_EMIOS_MODE_MC_UP_DOWN_CHANGE_EXT_CLK 23U
#define MPC55XX_EMIOS_MODE_OPWFM_B_IMMEDIATE 24U
#define MPC55XX_EMIOS_MODE_OPWFM_B_NEXT_PERIOD 25U
#define MPC55XX_EMIOS_MODE_OPWFM_AB_IMMEDIATE 26U
#define MPC55XX_EMIOS_MODE_OPWFM_AB_NEXT_PERIOD 27U
#define MPC55XX_EMIOS_MODE_OPWMC_TRAIL_TRAIL 28U
#define MPC55XX_EMIOS_MODE_OPWMC_TRAIL_LEAD 29U
#define MPC55XX_EMIOS_MODE_OPWMC_BOTH_TRAIL 30U
#define MPC55XX_EMIOS_MODE_OPWMC_BOTH_LEAD 31U
#define MPC55XX_EMIOS_MODE_OPWM_B_IMMEDIATE 32U
#define MPC55XX_EMIOS_MODE_OPWM_B_NEXT_PERIOD 33U
#define MPC55XX_EMIOS_MODE_OPWM_AB_IMMEDIATE 34U
#define MPC55XX_EMIOS_MODE_OPWM_AB_NEXT_PERIOD 35U
#define MPC55XX_EMIOS_MODE_RESERVED_36 36U
#define MPC55XX_EMIOS_MODE_RESERVED_37 37U
#define MPC55XX_EMIOS_MODE_RESERVED_38 38U
#define MPC55XX_EMIOS_MODE_RESERVED_39 39U
#define MPC55XX_EMIOS_MODE_RESERVED_40 40U
#define MPC55XX_EMIOS_MODE_RESERVED_41 41U
#define MPC55XX_EMIOS_MODE_RESERVED_42 42U
#define MPC55XX_EMIOS_MODE_RESERVED_43 43U
#define MPC55XX_EMIOS_MODE_RESERVED_44 44U
#define MPC55XX_EMIOS_MODE_RESERVED_45 45U
#define MPC55XX_EMIOS_MODE_RESERVED_46 46U
#define MPC55XX_EMIOS_MODE_RESERVED_47 47U
#define MPC55XX_EMIOS_MODE_RESERVED_48 48U
#define MPC55XX_EMIOS_MODE_RESERVED_49 49U
#define MPC55XX_EMIOS_MODE_RESERVED_50 50U
#define MPC55XX_EMIOS_MODE_RESERVED_51 51U
#define MPC55XX_EMIOS_MODE_RESERVED_52 52U
#define MPC55XX_EMIOS_MODE_RESERVED_53 53U
#define MPC55XX_EMIOS_MODE_RESERVED_54 54U
#define MPC55XX_EMIOS_MODE_RESERVED_55 55U
#define MPC55XX_EMIOS_MODE_RESERVED_56 56U
#define MPC55XX_EMIOS_MODE_RESERVED_57 57U
#define MPC55XX_EMIOS_MODE_RESERVED_58 58U
#define MPC55XX_EMIOS_MODE_RESERVED_59 59U
#define MPC55XX_EMIOS_MODE_RESERVED_60 60U
#define MPC55XX_EMIOS_MODE_RESERVED_61 61U
#define MPC55XX_EMIOS_MODE_RESERVED_62 62U
#define MPC55XX_EMIOS_MODE_RESERVED_63 63U
#define MPC55XX_EMIOS_MODE_RESERVED_64 64U
#define MPC55XX_EMIOS_MODE_RESERVED_65 65U
#define MPC55XX_EMIOS_MODE_RESERVED_66 66U
#define MPC55XX_EMIOS_MODE_RESERVED_67 67U
#define MPC55XX_EMIOS_MODE_RESERVED_68 68U
#define MPC55XX_EMIOS_MODE_RESERVED_69 69U
#define MPC55XX_EMIOS_MODE_RESERVED_70 70U
#define MPC55XX_EMIOS_MODE_RESERVED_71 71U
#define MPC55XX_EMIOS_MODE_RESERVED_72 72U
#define MPC55XX_EMIOS_MODE_RESERVED_73 73U
#define MPC55XX_EMIOS_MODE_RESERVED_74 74U
#define MPC55XX_EMIOS_MODE_RESERVED_75 75U
#define MPC55XX_EMIOS_MODE_RESERVED_76 76U
#define MPC55XX_EMIOS_MODE_RESERVED_77 77U
#define MPC55XX_EMIOS_MODE_RESERVED_78 78U
#define MPC55XX_EMIOS_MODE_RESERVED_79 79U
#define MPC55XX_EMIOS_MODE_MCB_UP_INT_CLK 80U
#define MPC55XX_EMIOS_MODE_MCB_UP_EXT_CLK 81U
#define MPC55XX_EMIOS_MODE_RESERVED_82 82U
#define MPC55XX_EMIOS_MODE_RESERVED_83 83U
#define MPC55XX_EMIOS_MODE_MCB_UP_DOWN_ONE_INT_CLK 84U
#define MPC55XX_EMIOS_MODE_MCB_UP_DOWN_ONE_EXT_CLK 85U
#define MPC55XX_EMIOS_MODE_MCB_UP_DOWN_BOTH_INT_CLK 86U
#define MPC55XX_EMIOS_MODE_MCB_UP_DOWN_BOTH_EXT_CLK 87U
#define MPC55XX_EMIOS_MODE_OPWFMB_B 88U
#define MPC55XX_EMIOS_MODE_RESERVED_89 89U
#define MPC55XX_EMIOS_MODE_OPWFMB_AB 90U
#define MPC55XX_EMIOS_MODE_RESERVED_91 91U
#define MPC55XX_EMIOS_MODE_OPWMCB_TRAIL_TRAIL 92U
#define MPC55XX_EMIOS_MODE_OPWMCB_TRAIL_LEAD 93U
#define MPC55XX_EMIOS_MODE_OPWMCB_BOTH_TRAIL 94U
#define MPC55XX_EMIOS_MODE_OPWMCB_BOTH_LEAD 95U
#define MPC55XX_EMIOS_MODE_OPWMB_SECOND 96U
#define MPC55XX_EMIOS_MODE_RESERVED_97 97U
#define MPC55XX_EMIOS_MODE_OPWMB_BOTH 98U
#define MPC55XX_EMIOS_MODE_RESERVED_99 99U
#define MPC55XX_EMIOS_MODE_RESERVED_100 100U
#define MPC55XX_EMIOS_MODE_RESERVED_101 101U
#define MPC55XX_EMIOS_MODE_RESERVED_102 102U
#define MPC55XX_EMIOS_MODE_RESERVED_103 103U
#define MPC55XX_EMIOS_MODE_RESERVED_104 104U
#define MPC55XX_EMIOS_MODE_RESERVED_105 105U
#define MPC55XX_EMIOS_MODE_RESERVED_106 106U
#define MPC55XX_EMIOS_MODE_RESERVED_107 107U
#define MPC55XX_EMIOS_MODE_RESERVED_108 108U
#define MPC55XX_EMIOS_MODE_RESERVED_109 109U
#define MPC55XX_EMIOS_MODE_RESERVED_110 110U
#define MPC55XX_EMIOS_MODE_RESERVED_111 111U
#define MPC55XX_EMIOS_MODE_RESERVED_112 112U
#define MPC55XX_EMIOS_MODE_RESERVED_113 113U
#define MPC55XX_EMIOS_MODE_RESERVED_114 114U
#define MPC55XX_EMIOS_MODE_RESERVED_115 115U
#define MPC55XX_EMIOS_MODE_RESERVED_116 116U
#define MPC55XX_EMIOS_MODE_RESERVED_117 117U
#define MPC55XX_EMIOS_MODE_RESERVED_118 118U
#define MPC55XX_EMIOS_MODE_RESERVED_119 119U
#define MPC55XX_EMIOS_MODE_RESERVED_120 120U
#define MPC55XX_EMIOS_MODE_RESERVED_121 121U
#define MPC55XX_EMIOS_MODE_RESERVED_122 122U
#define MPC55XX_EMIOS_MODE_RESERVED_123 123U
#define MPC55XX_EMIOS_MODE_RESERVED_124 124U
#define MPC55XX_EMIOS_MODE_RESERVED_125 125U
#define MPC55XX_EMIOS_MODE_RESERVED_126 126U
#define MPC55XX_EMIOS_MODE_RESERVED_127 127U

/** @} */

#if MPC55XX_CHIP_FAMILY == 566 || MPC55XX_CHIP_FAMILY == 567
  #define MPC55XX_EMIOS_CHANNEL_NUMBER 32U
#else
  #define MPC55XX_EMIOS_CHANNEL_NUMBER 24U
#endif

#define MPC55XX_EMIOS_VALUE_MAX 0x00ffffffU

#define MPC55XX_EMIOS_IS_CHANNEL_VALID( c) \
  ((unsigned) (c) < MPC55XX_EMIOS_CHANNEL_NUMBER)

#define MPC55XX_EMIOS_IS_CHANNEL_INVALID( c) \
  (!MPC55XX_EMIOS_IS_CHANNEL_VALID( c))

void mpc55xx_emios_initialize( unsigned prescaler);

unsigned mpc55xx_emios_global_prescaler( void);

void mpc55xx_emios_set_global_prescaler( unsigned prescaler);

#endif /* MPC55XX_HAS_EMIOS */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBCPU_POWERPC_MPC55XX_EMIOS_H */
