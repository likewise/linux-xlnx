all:
	(source ~/xilinx/petalinux-v2016.2-final/settings.sh; \
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- O=/home/leon/sandbox/sidebranch/xilinx/xilinx_zcu102trd_xhdmi/apu/petalinux_bsp/build/linux/kernel/linux-xlnx KCONFIG_CONFIG="/home/leon/sandbox/sidebranch/xilinx/xilinx_zcu102trd_xhdmi/apu/petalinux_bsp/build/linux/kernel/linux-xlnx"/.config -j3 -C /home/leon/sandbox/sidebranch/xilinx/xilinx_zcu102trd_xhdmi/apu/petalinux_bsp/build/linux/kernel/download/linux-xlnx all)

