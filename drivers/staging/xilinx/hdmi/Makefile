# mainlining: remove CFLAGS<...> += -I$(src)/include
# see below for further mainlining hints

# V4L2 (HDMI RX)
# mainlining: drivers/media/platform/xilinx/Makefile
obj-$(CONFIG_VIDEO_XILINX_HDMI_RX) += xilinx-hdmi-rx.o

# top-level Linux driver source
xilinx-hdmi-rx-objs := xilinx-hdmirx.o

# the C API around the IP registers
CFLAGS_xilinx-hdmirx.o += -I$(src)/xilinx-hdmi-rx/
CFLAGS_xilinx-hdmirx.o += -I$(src)/include/ -I${src}/../../../media/platform/xilinx
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirx.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirx_intr.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirxss.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirxss_log.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirxss_coreinit.o
xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xv_hdmirx_vsif.o

# WIP
#xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xhdcp22_rx.o
#xilinx-hdmi-rx-objs += xilinx-hdmi-rx/xhdcp22_rx_crypt.o

# DRM (HDMI TX)
# mainlining: drivers/gpu/drm/xilinx/Makefile

obj-$(CONFIG_DRM_XILINX_HDMI) += xilinx-hdmi-tx.o

# top-level Linux driver source
xilinx-hdmi-tx-objs := xilinx_drm_hdmi.o

# the C API around the IP registers
CFLAGS_xilinx_drm_hdmi.o += -I$(src)/xilinx-hdmi-tx/ -Wno-unused-label
CFLAGS_xilinx_drm_hdmi.o += -I$(src)/include -I${src}/../../../gpu/drm/xilinx/
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx_intr.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss_log.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitxss_coreinit.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xv_hdmitx_vsif.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvtc.o
xilinx-hdmi-tx-objs += xilinx-hdmi-tx/xvtc_intr.o

#xilinx-hdmi-tx-hdcp-objs := xilinx-hdmi-tx/xhdcp22_tx.o
#xilinx-hdmi-tx-hdcp-objs += xilinx-hdmi-tx/xhdcp22_tx_crypt.o

# module interdependency during linking
# (probably only needed when building separate modules)
KBUILD_EXTRA_SYMBOLS += ../../../phy/Modules.symvers

# PHY (HDMI RX/TX)
# mainlining: drivers/phy/Makefile

obj-$(CONFIG_PHY_XILINX_VPHY)		+= phy-xilinx-vphy.o
//obj-$(CONFIG_PHY_XILINX_VPHY)		+= phy-xilinx-vphy-hdcp.o

# top-level Linux driver source
phy-xilinx-vphy-objs := phy-vphy.o

CFLAGS_phy-vphy.o += -I$(src)/include

# the C API around the IP registers
CFLAGS_phy-vphy.o += -I$(src)/phy-xilinx-vphy/
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvphy.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvphy_gthe4.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvphy_hdmi.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvphy_hdmi_intr.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvphy_intr.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvphy_log.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xgpio.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xgpio_extra.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xgpio_intr.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xv_axi4s_remap.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvidc.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvidc_edid.o
phy-xilinx-vphy-objs += phy-xilinx-vphy/xvidc_timings_table.o

# WIP HDCP (USE_HDCP)
phy-xilinx-vphy-hdcp-objs := phy-xilinx-vphy/xhdcp1x.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp1x_cipher.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp1x_cipher_intr.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp1x_rx.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp1x_tx.o
#baseline/xhdcp1x_platform.c \
#baseline/xhdcp1x_port.c \
#baseline/xhdcp1x_port_dp_rx.c \
#baseline/xhdcp1x_port_dp_tx.c \
#baseline/xhdcp1x_port_hdmi_rx.c \
#baseline/xhdcp1x_port_hdmi_tx.c \
#baseline/xhdcp1x_port_intr.c \

phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_cipher.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_cipher_sinit.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_cipher_g.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_mmult.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_mmult_sinit.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_mmult_g.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_rng.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_rng_sinit.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_rng_g.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/bigdigits.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xtmrctr.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xtmrctr_l.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xtmrctr_g.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xtmrctr_sinit.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xtmrctr_options.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xtmrctr_intr.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/aes.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/hmac.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/sha1.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/sha2.o

phy-xilinx-vphy-hdcp-objs := phy-xilinx-vphy/xhdcp22_tx.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_tx_crypt.o

phy-xilinx-vphy-hdcp-objs := phy-xilinx-vphy/xhdcp22_rx.o
phy-xilinx-vphy-hdcp-objs += phy-xilinx-vphy/xhdcp22_rx_crypt.o
