/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSImplApplConfig
 *
 * @brief This header file evaluates configuration options related to the
 *   scheduler configuration.
 */

/*
 * Copyright (C) 2020 embedded brains GmbH (http://www.embedded-brains.de)
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

#ifndef _RTEMS_CONFDEFS_SCHEDULER_H
#define _RTEMS_CONFDEFS_SCHEDULER_H

#ifndef __CONFIGURATION_TEMPLATE_h
#error "Do not include this file directly, use <rtems/confdefs.h> instead"
#endif

#ifdef CONFIGURE_INIT

#include <rtems/confdefs/percpu.h>

#if !defined(CONFIGURE_SCHEDULER_CBS) \
  && !defined(CONFIGURE_SCHEDULER_EDF) \
  && !defined(CONFIGURE_SCHEDULER_EDF_SMP) \
  && !defined(CONFIGURE_SCHEDULER_PRIORITY) \
  && !defined(CONFIGURE_SCHEDULER_PRIORITY_AFFINITY_SMP) \
  && !defined(CONFIGURE_SCHEDULER_PRIORITY_SMP) \
  && !defined(CONFIGURE_SCHEDULER_SIMPLE) \
  && !defined(CONFIGURE_SCHEDULER_SIMPLE_SMP) \
  && !defined(CONFIGURE_SCHEDULER_STRONG_APA) \
  && !defined(CONFIGURE_SCHEDULER_USER)
  #if defined(RTEMS_SMP) && _CONFIGURE_MAXIMUM_PROCESSORS > 1
    #define CONFIGURE_SCHEDULER_EDF_SMP
  #else
    #define CONFIGURE_SCHEDULER_PRIORITY
  #endif
#endif

#include <rtems/scheduler.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIGURE_MAXIMUM_PRIORITY
  #define CONFIGURE_MAXIMUM_PRIORITY PRIORITY_DEFAULT_MAXIMUM
#endif

#if CONFIGURE_MAXIMUM_PRIORITY != 3 \
  && CONFIGURE_MAXIMUM_PRIORITY != 7 \
  && CONFIGURE_MAXIMUM_PRIORITY != 15 \
  && CONFIGURE_MAXIMUM_PRIORITY != 31 \
  && CONFIGURE_MAXIMUM_PRIORITY != 63 \
  && CONFIGURE_MAXIMUM_PRIORITY != 127 \
  && CONFIGURE_MAXIMUM_PRIORITY != 255
  #error "CONFIGURE_MAXIMUM_PRIORITY must be one of 3, 7, 15, 31, 63, 127, and 255"
#endif

#if CONFIGURE_MAXIMUM_PRIORITY > PRIORITY_DEFAULT_MAXIMUM
  #error "CONFIGURE_SCHEDULER_PRIORITY must be less than or equal to the architecture defined maximum priority"
#endif

#ifdef CONFIGURE_SCHEDULER_PRIORITY
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'U', 'P', 'D', ' ' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER \
      RTEMS_SCHEDULER_PRIORITY( \
        dflt, \
        CONFIGURE_MAXIMUM_PRIORITY + 1 \
      )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_PRIORITY( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_PRIORITY_SMP
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'M', 'P', 'D', ' ' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER \
      RTEMS_SCHEDULER_PRIORITY_SMP( \
        dflt, \
        CONFIGURE_MAXIMUM_PRIORITY + 1 \
      )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_PRIORITY_SMP( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_PRIORITY_AFFINITY_SMP
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'M', 'P', 'A', ' ' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER \
      RTEMS_SCHEDULER_PRIORITY_AFFINITY_SMP( \
        dflt, \
        CONFIGURE_MAXIMUM_PRIORITY + 1 \
      )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_PRIORITY_AFFINITY_SMP( \
        dflt, \
        CONFIGURE_SCHEDULER_NAME \
      )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_STRONG_APA
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'M', 'A', 'P', 'A' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER \
      RTEMS_SCHEDULER_STRONG_APA( \
        dflt, \
        CONFIGURE_MAXIMUM_PRIORITY + 1 \
      )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_STRONG_APA( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_SIMPLE
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'U', 'P', 'S', ' ' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER RTEMS_SCHEDULER_SIMPLE( dflt )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_SIMPLE( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_SIMPLE_SMP
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'M', 'P', 'S', ' ' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER \
      RTEMS_SCHEDULER_SIMPLE_SMP( dflt )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_SIMPLE_SMP( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_EDF
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'U', 'E', 'D', 'F' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER RTEMS_SCHEDULER_EDF( dflt )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_EDF( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_EDF_SMP
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'M', 'E', 'D', 'F' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER RTEMS_SCHEDULER_EDF_SMP( dflt )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_EDF_SMP( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif
#endif

#ifdef CONFIGURE_SCHEDULER_CBS
  #ifndef CONFIGURE_SCHEDULER_NAME
    #define CONFIGURE_SCHEDULER_NAME rtems_build_name( 'U', 'C', 'B', 'S' )
  #endif

  #ifndef CONFIGURE_SCHEDULER_TABLE_ENTRIES
    #define CONFIGURE_SCHEDULER RTEMS_SCHEDULER_CBS( dflt )

    #define CONFIGURE_SCHEDULER_TABLE_ENTRIES \
      RTEMS_SCHEDULER_TABLE_CBS( dflt, CONFIGURE_SCHEDULER_NAME )
  #endif

  #ifndef CONFIGURE_CBS_MAXIMUM_SERVERS
    #define CONFIGURE_CBS_MAXIMUM_SERVERS CONFIGURE_MAXIMUM_TASKS
  #endif

  const uint32_t _Scheduler_CBS_Maximum_servers =
    CONFIGURE_CBS_MAXIMUM_SERVERS;

  Scheduler_CBS_Server
    _Scheduler_CBS_Server_list[ CONFIGURE_CBS_MAXIMUM_SERVERS ];
#endif

#ifdef CONFIGURE_SCHEDULER
  /*
   * Ignore these warnings:
   *
   * - invalid use of structure with flexible array member
   *
   * - struct has no members
   */
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
  CONFIGURE_SCHEDULER;
  #pragma GCC diagnostic pop
#endif

const Scheduler_Control _Scheduler_Table[] = {
  CONFIGURE_SCHEDULER_TABLE_ENTRIES
};

#define _CONFIGURE_SCHEDULER_COUNT RTEMS_ARRAY_SIZE( _Scheduler_Table )

#ifdef RTEMS_SMP

const size_t _Scheduler_Count = _CONFIGURE_SCHEDULER_COUNT;

const Scheduler_Assignment _Scheduler_Initial_assignments[] = {
  #ifdef CONFIGURE_SCHEDULER_ASSIGNMENTS
    CONFIGURE_SCHEDULER_ASSIGNMENTS
  #else
    #define _CONFIGURE_SCHEDULER_ASSIGN \
      RTEMS_SCHEDULER_ASSIGN( \
        0, \
        RTEMS_SCHEDULER_ASSIGN_PROCESSOR_OPTIONAL \
      )
    _CONFIGURE_SCHEDULER_ASSIGN
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 2
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 3
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 4
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 5
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 6
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 7
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 8
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 9
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 10
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 11
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 12
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 13
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 14
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 15
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 16
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 17
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 18
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 19
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 20
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 21
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 22
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 23
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 24
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 25
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 26
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 27
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 28
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 29
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 30
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 31
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #if _CONFIGURE_MAXIMUM_PROCESSORS >= 32
      , _CONFIGURE_SCHEDULER_ASSIGN
    #endif
    #undef _CONFIGURE_SCHEDULER_ASSIGN
  #endif
};

RTEMS_STATIC_ASSERT(
  _CONFIGURE_MAXIMUM_PROCESSORS
    == RTEMS_ARRAY_SIZE( _Scheduler_Initial_assignments ),
  _Scheduler_Initial_assignments
);

#endif /* RTEMS_SMP */

#ifdef __cplusplus
}
#endif

#endif /* CONFIGURE_INIT */

#endif /* _RTEMS_CONFDEFS_SCHEDULER_H */
