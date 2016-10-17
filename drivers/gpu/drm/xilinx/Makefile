#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

xilinx_drm-y := xilinx_drm_crtc.o xilinx_drm_connector.o xilinx_drm_drv.o \
		xilinx_drm_encoder.o xilinx_drm_fb.o xilinx_drm_gem.o \
		xilinx_drm_plane.o
xilinx_drm-y += xilinx_cresample.o xilinx_osd.o xilinx_rgb2yuv.o xilinx_vtc.o

obj-$(CONFIG_DRM_XILINX) += xilinx_drm.o
obj-$(CONFIG_DRM_XILINX_DP) += xilinx_drm_dp.o
obj-$(CONFIG_DRM_XILINX_DP_SUB) += xilinx_drm_dp_sub.o

obj-$(CONFIG_DRM_XILINX_HDMI) += xilinx-hdmi-tx.o
xilinx-hdmi-tx-objs := xilinx-hdmitx.o
#CFLAGS_xilinx-hdmitx.o += -I$(src)/xilinx-hdmi-tx/ -I$(src)/../../../media/platform/xilinx -Wno-unused-label
CFLAGS_xilinx-hdmitx.o += -I$(src)/xilinx-hdmi-tx/ -Wno-unused-label
#xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xgpio.o
#xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xgpio_extra.o
#xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xgpio_intr.o
#xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_axi4s_remap.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx_intr.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss_coreinit.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx_vsif.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvtc.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvtc_intr.o
#xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvidc.o
#xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvidc_edid.o
#xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvidc_timings_table.o

KBUILD_EXTRA_SYMBOLS += ../../../phy/Modules.symvers

