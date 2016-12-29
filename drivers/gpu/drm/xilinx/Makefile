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

# this builds the stand-alone (DRM-independent) top-level
# useful for testing purposes without DRM specifics
#xilinx-hdmi-tx-objs := xilinx-hdmitx.o

# (X)OR this builds the DRM-dependent top-level
xilinx-hdmi-tx-objs := xilinx_drm_hdmi.o

CFLAGS_xilinx-hdmitx.o += -I$(src)/xilinx-hdmi-tx/ -Wno-unused-label
CFLAGS_xilinx_drm_hdmi.o += -I$(src)/xilinx-hdmi-tx/ -Wno-unused-label
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx_intr.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss_log.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss_coreinit.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx_vsif.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvtc.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvtc_intr.o

# @TODO see if needed
KBUILD_EXTRA_SYMBOLS += ../../../phy/Modules.symvers

