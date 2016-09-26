xilinx-video-objs += xilinx-dma.o xilinx-vip.o xilinx-vipp.o

obj-$(CONFIG_VIDEO_XILINX) += xilinx-video.o
obj-$(CONFIG_VIDEO_XILINX_CFA) += xilinx-cfa.o
obj-$(CONFIG_VIDEO_XILINX_CRESAMPLE) += xilinx-cresample.o
obj-$(CONFIG_VIDEO_XILINX_HLS) += xilinx-hls.o
obj-$(CONFIG_VIDEO_XILINX_RGB2YUV) += xilinx-rgb2yuv.o
obj-$(CONFIG_VIDEO_XILINX_SCALER) += xilinx-scaler.o
obj-$(CONFIG_VIDEO_XILINX_REMAPPER) += xilinx-remapper.o
obj-$(CONFIG_VIDEO_XILINX_SWITCH) += xilinx-switch.o
obj-$(CONFIG_VIDEO_XILINX_TPG) += xilinx-tpg.o
obj-$(CONFIG_VIDEO_XILINX_VTC) += xilinx-vtc.o

obj-$(CONFIG_VIDEO_XILINX_HDMI_RX) := xilinx-hdmi-rx.o
xilinx-hdmi-rx-objs := xilinx-hdmirx.o
CFLAGS_xilinx-hdmirx.o += -I$(src)/xilinx-hdmi-rx/ -Wno-unused-label
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xgpio.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xgpio_extra.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xgpio_intr.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_axi4s_remap.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirx.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirx_intr.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirxss.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirxss_coreinit.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirx_vsif.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xvidc.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xvidc_edid.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xvidc_timings_table.o
