#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/charles-datle/ti/bios_6_73_01_01/packages
override XDCROOT = /home/charles-datle/ti/xdctools_3_50_08_24_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/charles-datle/ti/bios_6_73_01_01/packages;/home/charles-datle/ti/xdctools_3_50_08_24_core/packages;..
HOSTOS = Linux
endif
