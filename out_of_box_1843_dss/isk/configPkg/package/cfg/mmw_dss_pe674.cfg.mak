# invoke SourceDir generated makefile for mmw_dss.pe674
mmw_dss.pe674: .libraries,mmw_dss.pe674
.libraries,mmw_dss.pe674: package/cfg/mmw_dss_pe674.xdl
	$(MAKE) -f /home/charles-datle/UNI/Radar_1843_Linux/out_of_box_1843_dss/src/makefile.libs

clean::
	$(MAKE) -f /home/charles-datle/UNI/Radar_1843_Linux/out_of_box_1843_dss/src/makefile.libs clean

