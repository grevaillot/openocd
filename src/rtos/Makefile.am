include $(top_srcdir)/common.mk

METASOURCES = AUTO
noinst_LTLIBRARIES = librtos.la
noinst_HEADERS = \
	rtos.h \
	rtos_standard_stackings.h \
	rtos_ecos_stackings.h \
	linux_header.h \
	rtos_chibios_stackings.h \
	rtos_embkernel_stackings.h \
	rtos_mqx_stackings.h \
	rtos_ucos_iii_stackings.h

librtos_la_SOURCES = \
	rtos.c \
	rtos_standard_stackings.c \
	rtos_ecos_stackings.c  \
	rtos_chibios_stackings.c \
	rtos_embkernel_stackings.c \
	rtos_mqx_stackings.c \
	rtos_ucos_iii_stackings.c \
	FreeRTOS.c \
	ThreadX.c \
	eCos.c \
	linux.c \
	ChibiOS.c \
	embKernel.c \
	mqx.c \
	uCOS-III.c

librtos_la_CFLAGS =
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
librtos_la_CFLAGS += -Wno-sign-compare
endif

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
