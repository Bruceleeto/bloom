/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2014-2021 Paul Cercueil <paul@crapouillou.net>
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>
#include <unistd.h>

#include "deadline.h"

/* A LOG LINE IS NOT GUEST WORK.  These go straight to fprintf, so unbracketed
 * they land on the guest clock as thousands of cycles at whatever point the
 * compiler happened to print -- which is why K only ever wandered while a
 * level was loading. */
#define LIGHTREC_LOG(stream, ...) do {					\
	if (FGL_DL_MEASURE)						\
		fgl_deadline_pause();					\
	fprintf(stream, __VA_ARGS__);					\
	if (FGL_DL_MEASURE)						\
		fgl_deadline_resume();					\
} while (0)

#define NOLOG_L 0
#define ERROR_L 1
#define WARNING_L 2
#define INFO_L 3
#define DEBUG_L 4

#ifndef LOG_LEVEL
#define LOG_LEVEL INFO_L
#endif

// -------------

#ifndef COLOR_DEBUG
#define COLOR_DEBUG   "\e[0;32m"
#endif
#ifndef COLOR_WARNING
#define COLOR_WARNING "\e[01;35m"
#endif
#ifndef COLOR_ERROR
#define COLOR_ERROR   "\e[01;31m"
#endif

#define COLOR_END "\e[0m"

#if (LOG_LEVEL >= DEBUG_L)
# ifdef COLOR_DEBUG
#  define pr_debug(str, ...) do {					\
	if (isatty(STDOUT_FILENO))					\
		LIGHTREC_LOG(stdout, COLOR_DEBUG "DEBUG: " str COLOR_END,	\
			##__VA_ARGS__);					\
	else								\
		LIGHTREC_LOG(stdout, "DEBUG: " str, ##__VA_ARGS__);		\
	} while (0)
# else
#  define pr_debug(...) \
    LIGHTREC_LOG(stdout, "DEBUG: " __VA_ARGS__)
# endif
#else
#define pr_debug(...)
#endif

#if (LOG_LEVEL >= INFO_L)
# ifdef COLOR_INFO
#  define pr_info(str, ...) \
    LIGHTREC_LOG(stdout, COLOR_INFO str COLOR_END, ##__VA_ARGS__)
# else
#  define pr_info(...) \
    LIGHTREC_LOG(stdout, __VA_ARGS__)
# endif
#else
#define pr_info(...)
#endif

#if (LOG_LEVEL >= WARNING_L)
# ifdef COLOR_WARNING
#  define pr_warn(str, ...) do {					\
	if (isatty(STDERR_FILENO))					\
		LIGHTREC_LOG(stderr, COLOR_WARNING "WARNING: " str COLOR_END,\
			##__VA_ARGS__);					\
	else								\
		LIGHTREC_LOG(stderr, "WARNING: " str, ##__VA_ARGS__);	\
	} while (0)
# else
#  define pr_warn(...) \
    LIGHTREC_LOG(stderr, "WARNING: " __VA_ARGS__)
# endif
#else
#define pr_warn(...)
#endif

#if (LOG_LEVEL >= ERROR_L)
# ifdef COLOR_ERROR
#  define pr_err(str, ...) do {						\
	if (isatty(STDERR_FILENO))					\
		LIGHTREC_LOG(stderr, COLOR_ERROR "ERROR: " str COLOR_END,	\
			##__VA_ARGS__);					\
	else								\
		LIGHTREC_LOG(stderr, "ERROR: " str, ##__VA_ARGS__);		\
	} while (0)
# else
#  define pr_err(...) \
    LIGHTREC_LOG(stderr, "ERROR: " __VA_ARGS__)
# endif
#else
#define pr_err(...)
#endif

#endif
