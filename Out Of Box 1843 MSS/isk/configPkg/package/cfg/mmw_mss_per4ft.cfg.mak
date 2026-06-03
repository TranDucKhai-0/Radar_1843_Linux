# invoke SourceDir generated makefile for mmw_mss.per4ft
mmw_mss.per4ft: .libraries,mmw_mss.per4ft
.libraries,mmw_mss.per4ft: package/cfg/mmw_mss_per4ft.xdl
	$(MAKE) -f /home/khai/workspace_ccstheia/Out Of Box 1843 MSS/src/makefile.libs

clean::
	$(MAKE) -f /home/khai/workspace_ccstheia/Out Of Box 1843 MSS/src/makefile.libs clean

