//
// QSB USB Device Library for libopencm3
//
// Copyright (c) 2021 Manuel Bleichenbacher
// Licensed under LGPL License https://opensource.org/licenses/LGPL-3.0
// Work derived from libopencm3 (https://github.com/libopencm3/libopencm3)
//

//
// Configuration of active peripheral code and functions
//

#pragma once

#define QSB_STR_ENC_LATIN1 1
#define QSB_STR_ENC_UTF8 2

// Configuration flags:
// QSB_STR_ENC: Selects the encoding of strings provided as input for string descriptors.
//     According to the USB standard, string descriptors are always transmitted with UTF-16 encoding.
//     The QSB library however uses a single byte API. If this flag is not defined, `QSB_STR_UTF8` can be
//     defined alternatively. It selects UTF-8 encoding. If none of the flags is defined, the default is Latin-1.
//     Valid values are:
//         1: Latin-1 (aka ISO-8859-1)
//         2: UTF-8
//
// QSB_BOS_ENABLE: If defined, enables support for USB Binary Device Object Store (BOS)
//     By default, BOS support is not available.
//
//
// QSB_MAX_CONTROL_CALLBACKS: Maximum number of control request callbacks that can be registered.
//     By default, it is 4.
//
// QSB_MAX_SET_CONFIG_CALLBACKS: Maximum number of "Set_Config" request callbackss that can be registered.
//     By default, it is 4.
//
// QSB_WIN_WCID_ENABLE: If defined, enables support for Microsoft WCID descriptors.
//     The WCID descriptors declare that Windows should use the WinUSB drivers. This is good for device
//     with a single interface with number 0. 
//
// QSB_WIN_WCID_VENDOR_CODE: If WCID descriptors is enabled, this macro can be defined to set the
//     vendor code used in the WCID control request. The default value is 0xF0. 

// This copy only supports the USB peripheral of the STM32F0 (16-bit BTABLE, with USB_BCDR).
#if !defined(STM32F0)
#error "This USB stack only supports the STM32F0"
#endif

#ifndef QSB_STR_ENC
#ifdef QSB_STR_UTF8
#define QSB_STR_ENC QSB_STR_ENC_UTF8
#else
#define QSB_STR_ENC QSB_STR_ENC_LATIN1
#endif
#endif

#ifdef QSB_BOS_ENABLE
#define QSB_BOS 1
#else
#define QSB_BOS 0
#endif

#ifndef QSB_MAX_CONTROL_CALLBACKS
#define QSB_MAX_CONTROL_CALLBACKS 4
#endif

#ifndef QSB_MAX_SET_CONFIG_CALLBACKS
#define QSB_MAX_SET_CONFIG_CALLBACKS 4
#endif

#ifdef QSB_WIN_WCID_ENABLE
#define QSB_WIN_WCID 1
#else
#define QSB_WIN_WCID 0
#endif

#ifndef QSB_WIN_WCID_VENDOR_CODE
#define QSB_WIN_WCID_VENDOR_CODE 0xf0
#endif
