#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/khai/ti/bios_6_73_01_01/packages;/home/khai/ti/mathlib_c674x_3_1_2_1/packages
override XDCROOT = /home/khai/ti/xdctools_3_50_08_24_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/khai/ti/bios_6_73_01_01/packages;/home/khai/ti/mathlib_c674x_3_1_2_1/packages;/home/khai/ti/xdctools_3_50_08_24_core/packages;..
HOSTOS = Linux
endif
