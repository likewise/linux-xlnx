/*
 * The Video Phy is basically a layer above the GT to configure the GT
 * and implement other logic common to many video connectivity protocols.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <dt-bindings/phy/phy.h>
#include <linux/list.h>
#include <linux/interrupt.h>

#include "linux/phy/phy-vphy.h"

/* baseline driver includes */
#include "phy-xilinx-vphy/xvphy.h"
#include "phy-xilinx-vphy/xil_printf.h"
#include "phy-xilinx-vphy/xstatus.h"

/**
 * struct xvphy_lane - representation of a lane
 * @phy: pointer to the kernel PHY device
 * @type: controller which uses this lane
 * @lane: lane number
 * @protocol: protocol in which the lane operates
 * @ref_clk: enum of allowed ref clock rates for this lane PLL
 * @pll_lock: PLL status
 * @data: pointer to hold private data
 * @refclk_rate: PLL reference clock frequency
 * @share_laneclk: lane number of the clock to be shared
 */
struct xvphy_lane {
	struct phy *phy;
	u8 type;
	u8 lane;
	u8 protocol;
	bool pll_lock;
	/* data is pointer to parent xvphy_dev */
	void *data;
	u32 refclk_rate;
	u32 share_laneclk;
};

/**
 * struct xvphy_dev - representation of a ZynMP GT device
 * @dev: pointer to device
 * @iomem: serdes base address
 * @gtr_mutex: mutex for locking
 * @lanes: pointer to all the lanes
 * @fpd: base address for full power domain devices reset control
 * @lpd: base address for low power domain devices reset control
 * @tx_term_fix: fix for GT issue
 */
struct xvphy_dev {
	struct device *dev;
	void __iomem *iomem;
	int irq;
	/* protects the XVphy baseline against concurrent access */
	struct mutex xvphy_mutex;
	struct xvphy_lane *lanes[4];
	/* bookkeeping for the baseline subsystem driver instance */
	XVphy xvphy;
};

/* template function available to controller in dependent driver */
int xvphy_do_something(struct phy *phy)
{
	struct xvphy_lane *vphy_lane = phy_get_drvdata(phy);
	struct xvphy_dev *vphy_dev = vphy_lane->data;

	dev_dbg(vphy_dev->dev, "xvphy_do_something()\n");
	return 0;
}
EXPORT_SYMBOL_GPL(xvphy_do_something);

/* given the (Linux) phy, return the xvphy */
XVphy *xvphy_get_xvphy(struct phy *phy)
{
	struct xvphy_lane *vphy_lane = phy_get_drvdata(phy);
	struct xvphy_dev *vphy_dev = vphy_lane->data;

	//dev_dbg(vphy_dev->dev, "xvphy_get_xvphy() returns %p\n", &vphy_dev->xvphy);
	return &vphy_dev->xvphy;
}
EXPORT_SYMBOL_GPL(xvphy_get_xvphy);

/* given the (Linux) phy, enter critical section of xvphy baseline code */
void xvphy_mutex_lock(struct phy *phy)
{
	struct xvphy_lane *vphy_lane = phy_get_drvdata(phy);
	struct xvphy_dev *vphy_dev = vphy_lane->data;

	mutex_lock(&vphy_dev->xvphy_mutex);
}
EXPORT_SYMBOL_GPL(xvphy_mutex_lock);

/* given the (Linux) phy, exit critical section of xvphy baseline code */
void xvphy_mutex_unlock(struct phy *phy)
{
	struct xvphy_lane *vphy_lane = phy_get_drvdata(phy);
	struct xvphy_dev *vphy_dev = vphy_lane->data;

	mutex_unlock(&vphy_dev->xvphy_mutex);
}
EXPORT_SYMBOL_GPL(xvphy_mutex_unlock);

#if 0
/* wrapper around XVphy_GetPllType
 *
 * XVphy_PllType XVphy_GetPllType(XVphy *InstancePtr, u8 QuadId,
 *		XVphy_DirectionType Dir, XVphy_ChannelId ChId)
 */
XVphy_PllType xvphy_get_plltype(struct phy *phy, u8 QuadId,
						 XVphy_DirectionType Dir, XVphy_ChannelId ChId)
{
	struct xvphy_lane *vphy_lane = phy_get_drvdata(phy);
	struct xvphy_dev *vphy_dev = vphy_lane->data;

	return XVphy_GetPllType(&vphy_dev->xvphy, QuadId, Dir, ChId);
}
EXPORT_SYMBOL_GPL(xvphy_get_plltype);
#endif

#if 0
/* wrapper around XVphy_SetHdmiCallback
 *
 *void XVphy_SetHdmiCallback(XVphy *InstancePtr,
 *               XVphy_HdmiHandlerType HandlerType,
 *               void *CallbackFunc, void *CallbackRef);
 */
int xvphy_set_hdmi_callback(struct phy *phy, XVphy_HdmiHandlerType HandlerType,
					   void *CallbackFunc, void *CallbackRef)
{
	struct xvphy_lane *vphy_lane = phy_get_drvdata(phy);
	struct xvphy_dev *vphy_dev = vphy_lane->data;

	dev_dbg(vphy_dev->dev, "xvphy_set_callback()\n");

	XVphy_SetHdmiCallback(&vphy_dev->xvphy, HandlerType, CallbackFunc, CallbackRef);
	/* see header file. In case this module is disabled, it will return -ENODEV */
	return 0;
}
EXPORT_SYMBOL_GPL(xvphy_set_hdmi_callback);
#endif

/* instead of wrappers, just export the existing baseline API to our closely related consuming
 * controller drivers. Functions must be called inbetween xvphy_mutex_lock/xvphy_mutex_unlock */
EXPORT_SYMBOL_GPL(XVphy_GetPllType);
EXPORT_SYMBOL_GPL(XVphy_IBufDsEnable);
EXPORT_SYMBOL_GPL(XVphy_SetHdmiCallback);
EXPORT_SYMBOL_GPL(XVphy_HdmiCfgCalcMmcmParam);
EXPORT_SYMBOL_GPL(XVphy_MmcmStart);

static irqreturn_t xvphy_irq_handler(int irq, void *dev_id)
{
	struct xvphy_dev *vphydev;
	u32 IntrStatus;
	BUG_ON(!dev_id);
	vphydev = (struct xvphy_dev *)dev_id;
	BUG_ON(!vphydev);
	if (!vphydev)
		return IRQ_NONE;
	//printk(KERN_DEBUG "xvphy_irq_handler()\n");

	XVphy_IntrDisable(&vphydev->xvphy, XVPHY_INTR_HANDLER_TYPE_TXRESET_DONE |
			XVPHY_INTR_HANDLER_TYPE_RXRESET_DONE |
			XVPHY_INTR_HANDLER_TYPE_CPLL_LOCK |
			XVPHY_INTR_HANDLER_TYPE_QPLL0_LOCK |
			XVPHY_INTR_HANDLER_TYPE_TXALIGN_DONE |
			XVPHY_INTR_HANDLER_TYPE_QPLL1_LOCK |
			XVPHY_INTR_HANDLER_TYPE_TX_CLKDET_FREQ_CHANGE |
			XVPHY_INTR_HANDLER_TYPE_RX_CLKDET_FREQ_CHANGE |
			XVPHY_INTR_HANDLER_TYPE_TX_TMR_TIMEOUT |
			XVPHY_INTR_HANDLER_TYPE_RX_TMR_TIMEOUT);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t xvphy_irq_thread(int irq, void *dev_id)
{
	struct xvphy_dev *vphydev;
	u32 IntrStatus;
	BUG_ON(!dev_id);
	vphydev = (struct xvphy_dev *)dev_id;
	BUG_ON(!vphydev);
	if (!vphydev)
		return IRQ_NONE;

	/* call baremetal interrupt handler with mutex locked */
	mutex_lock(&vphydev->xvphy_mutex);

	IntrStatus = XVphy_ReadReg(vphydev->xvphy.Config.BaseAddr, XVPHY_INTR_STS_REG);
	printk(KERN_DEBUG "XVphy IntrStatus = 0x%08x\n", IntrStatus);

	XVphy_InterruptHandler(&vphydev->xvphy);
	mutex_unlock(&vphydev->xvphy_mutex);

	XVphy_IntrEnable(&vphydev->xvphy, XVPHY_INTR_HANDLER_TYPE_TXRESET_DONE |
		XVPHY_INTR_HANDLER_TYPE_RXRESET_DONE |
		XVPHY_INTR_HANDLER_TYPE_CPLL_LOCK |
		XVPHY_INTR_HANDLER_TYPE_QPLL0_LOCK |
		XVPHY_INTR_HANDLER_TYPE_TXALIGN_DONE |
		XVPHY_INTR_HANDLER_TYPE_QPLL1_LOCK |
		XVPHY_INTR_HANDLER_TYPE_TX_CLKDET_FREQ_CHANGE |
		XVPHY_INTR_HANDLER_TYPE_RX_CLKDET_FREQ_CHANGE |
		XVPHY_INTR_HANDLER_TYPE_TX_TMR_TIMEOUT |
		XVPHY_INTR_HANDLER_TYPE_RX_TMR_TIMEOUT);
	return IRQ_HANDLED;
}

/**
 * xvphy_phy_init - initializes a lane
 * @phy: pointer to kernel PHY device
 *
 * Return: 0 on success or error on failure
 */
static int xvphy_phy_init(struct phy *phy)
{
	BUG_ON(!phy);
	struct xvphy_lane *vphy_lane = NULL;
	printk(KERN_INFO "xvphy_phy_init(%p).\n", phy);

	//printk(KERN_INFO "xvphy_probe() found %d phy lanes from device-tree configuration.\n", index);
	//printk(KERN_INFO "xvphy_probe() found %d phy lanes from device-tree configuration.\n", index);

	return 0;
}

/**
 * xvphy_xlate - provides a PHY specific to a controller
 * @dev: pointer to device
 * @args: arguments from dts
 *
 * Return: pointer to kernel PHY device or error on failure
 *
 *
 */
static struct phy *xvphy_xlate(struct device *dev,
				   struct of_phandle_args *args)
{
	struct xvphy_dev *vphydev = dev_get_drvdata(dev);
	struct xvphy_lane *vphy_lane = NULL;
	struct device_node *phynode = args->np;
	int index;
	int i;
	u8 controller;
	u8 instance_num;

	if (args->args_count != 4) {
		dev_err(dev, "Invalid number of cells in 'phy' property\n");
		return ERR_PTR(-EINVAL);
	}
	if (!of_device_is_available(phynode)) {
		dev_warn(dev, "requested PHY is disabled\n");
		return ERR_PTR(-ENODEV);
	}
	for (index = 0; index < of_get_child_count(dev->of_node); index++) {
		if (phynode == vphydev->lanes[index]->phy->dev.of_node) {
			dev_info(dev, "xvphy_xlate() matched with phy index %d\n", index);
			vphy_lane = vphydev->lanes[index];
			break;
		}
	}
	if (!vphy_lane) {
		dev_err(dev, "failed to find appropriate phy\n");
		return ERR_PTR(-EINVAL);
	}

	/* get type of controller from lanes */
	controller = args->args[0];

	/* get controller instance number */
	instance_num = args->args[1];

	/* Check if lane sharing is required */
	vphy_lane->share_laneclk = args->args[2];

	/* get the required clk rate for controller from lanes */
	vphy_lane->refclk_rate = args->args[3];

	dev_info(dev, "xvphy_xlate() returns phy %p\n", vphy_lane->phy);
	BUG_ON(!vphy_lane->phy);
	return vphy_lane->phy;
#if 0

	/* derive lane type */
	if (xvphy_set_lanetype(vphy_lane, controller, instance_num) < 0) {
		dev_err(vphydev->dev, "Invalid lane type\n");
		return ERR_PTR(-EINVAL);
	}

	/* configures SSC settings for a lane */
	if (xvphy_configure_lane(vphy_lane) < 0) {
		dev_err(vphydev->dev, "Invalid clock rate: %d\n",
						vphy_lane->refclk_rate);
		return ERR_PTR(-EINVAL);
	}
	/*
	 * Check Interconnect Matrix is obeyed i.e, given lane type
	 * is allowed to operate on the lane.
	 */
	for (i = 0; i < CONTROLLERS_PER_LANE; i++) {
		if (icm_matrix[index][i] == gtr_phy->type)
			return gtr_phy->phy;
	}

#ifdef XPAR_XV_HDMITXSS_NUM_INSTANCES
    /* VPHY callback setup */
    XVphy_SetHdmiCallback(&Vphy,
    						XVPHY_HDMI_HANDLER_TXINIT,
    						VphyHdmiTxInitCallback,
    						(void *)&Vphy);
    XVphy_SetHdmiCallback(&Vphy,
    						XVPHY_HDMI_HANDLER_TXREADY,
    						VphyHdmiTxReadyCallback,
    						(void *)&Vphy);
#endif
#ifdef XPAR_XV_HDMIRXSS_NUM_INSTANCES
    XVphy_SetHdmiCallback(&Vphy,
    						XVPHY_HDMI_HANDLER_RXINIT,
    						VphyHdmiRxInitCallback,
    						(void *)&Vphy);
    XVphy_SetHdmiCallback(&Vphy,
    						XVPHY_HDMI_HANDLER_RXREADY,
    						VphyHdmiRxReadyCallback,
    						(void *)&Vphy);
#endif
#endif

	/* Should not reach here */
	return ERR_PTR(-EINVAL);
}

#if 0
/**
 * This typedef contains configuration information for the Video PHY core.
 */
typedef struct {
	u16 DeviceId;			/**< Device instance ID. */
	uintptr_t BaseAddr;			/**< The base address of the core
						instance. */
	XVphy_GtType XcvrType;		/**< VPHY Transceiver Type */
	u8 TxChannels;			/**< No. of active channels in TX */
	u8 RxChannels;			/**< No. of active channels in RX */
	XVphy_ProtocolType TxProtocol;	/**< Protocol which TX is used for. */
	XVphy_ProtocolType RxProtocol;	/**< Protocol which RX is used for. */
	XVphy_PllRefClkSelType TxRefClkSel; /**< TX REFCLK selection. */
	XVphy_PllRefClkSelType RxRefClkSel; /**< RX REFCLK selection. */
	XVphy_SysClkDataSelType TxSysPllClkSel; /**< TX SYSCLK selection. */
	XVphy_SysClkDataSelType RxSysPllClkSel; /**< RX SYSCLK selectino. */
	u8 DruIsPresent;		/**< A data recovery unit (DRU) exists
						in the design .*/
	XVphy_PllRefClkSelType DruRefClkSel; /**< DRU REFCLK selection. */
	XVidC_PixelsPerClock Ppc;	/**< Number of input pixels per
						 clock. */
	u8 TxBufferBypass;		/**< TX Buffer Bypass is enabled in the
						design. */
} XVphy_Config;

#endif

static XVphy_Config config = {
		XPAR_VID_PHY_CONTROLLER_0_DEVICE_ID,
		XPAR_VID_PHY_CONTROLLER_0_BASEADDR,
		XPAR_VID_PHY_CONTROLLER_0_TRANSCEIVER,
		XPAR_VID_PHY_CONTROLLER_0_TX_NO_OF_CHANNELS,
		XPAR_VID_PHY_CONTROLLER_0_RX_NO_OF_CHANNELS,
		XPAR_VID_PHY_CONTROLLER_0_TX_PROTOCOL,
		XPAR_VID_PHY_CONTROLLER_0_RX_PROTOCOL,
		XPAR_VID_PHY_CONTROLLER_0_TX_REFCLK_SEL,
		XPAR_VID_PHY_CONTROLLER_0_RX_REFCLK_SEL,
		XPAR_VID_PHY_CONTROLLER_0_TX_PLL_SELECTION,
		XPAR_VID_PHY_CONTROLLER_0_RX_PLL_SELECTION,
		XPAR_VID_PHY_CONTROLLER_0_NIDRU,
		XPAR_VID_PHY_CONTROLLER_0_NIDRU_REFCLK_SEL,
		XPAR_VID_PHY_CONTROLLER_0_INPUT_PIXELS_PER_CLOCK,
		XPAR_VID_PHY_CONTROLLER_0_TX_BUFFER_BYPASS
};

static void vphy_config_init(XVphy_Config *config, void __iomem *iomem)
{
	config->BaseAddr = (uintptr_t)iomem;
};

static struct phy_ops xvphy_phyops = {
	.init		= xvphy_phy_init,
	.owner		= THIS_MODULE,
};

static int vphy_parse_of(struct xvphy_dev *vphydev)
{
	struct device *dev = vphydev->dev;
	struct device_node *node = dev->of_node;
	(void)dev;
	(void)node;
#if 0
	struct device_node *ports;
	struct device_node *port;
	unsigned int nports = 0;
	bool has_endpoint = false;
#endif

#if 0 // example bool
	bool has_dre = false;
	has_dre = of_property_read_bool(node, "xlnx,include-dre");
#endif
#if 0 // example u32
	u32 value;
	int err;
	err = of_property_read_u32(node, "xlnx,datawidth", &value);
#endif

	return 0;
}

/**
 * xvphy_probe - The device probe function for driver initialization.
 * @pdev: pointer to the platform device structure.
 *
 * Return: 0 for success and error value on failure
 */
static int xvphy_probe(struct platform_device *pdev)
{
	struct device_node *child, *np = pdev->dev.of_node;
	struct xvphy_dev *vphydev;
	struct phy_provider *provider;
	struct phy *phy;
	struct resource *res;
	int lanecount, port = 0, index = 0;
	int ret;
	int i;
	u32 Status;
	u32 Data;

	vphydev = devm_kzalloc(&pdev->dev, sizeof(*vphydev), GFP_KERNEL);
	if (!vphydev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vphydev->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vphydev->iomem))
		return PTR_ERR(vphydev->iomem);

	vphydev->irq = platform_get_irq(pdev, 0);
	if (vphydev->irq <= 0) {
		dev_err(&pdev->dev, "platform_get_irq() failed\n");
		return vphydev->irq;
	}

	/* mutex that protects against concurrent access */
	mutex_init(&vphydev->xvphy_mutex);

	BUG_ON(!np);
	for_each_child_of_node(np, child) {
		struct xvphy_lane *vphy_lane;

		vphy_lane = devm_kzalloc(&pdev->dev, sizeof(*vphy_lane),
					 GFP_KERNEL);
		if (!vphy_lane)
			return -ENOMEM;

		/* Assign lane number to gtr_phy instance */
		vphy_lane->lane = index;

		/* Disable lane sharing as default */
		vphy_lane->share_laneclk = -1;

		BUG_ON(port >= 4);
		/* array of pointer to vphy_lane structs */
		vphydev->lanes[port] = vphy_lane;

		/* create phy device for each lane */
		phy = devm_phy_create(&pdev->dev, child, &xvphy_phyops);
		if (IS_ERR(phy)) {
			dev_err(&pdev->dev, "failed to create PHY\n");
			return PTR_ERR(phy);
		}
		/* array of pointer to phy */
		vphydev->lanes[port]->phy = phy;
		/* where each phy device has vphy_lane as driver data */
		phy_set_drvdata(phy, vphydev->lanes[port]);
		/* and each vphy_lane points back to parent device */
		vphy_lane->data = vphydev;
		port++;
		index++;
	}

	printk(KERN_INFO "xvphy_probe() found %d phy lanes from device-tree configuration.\n", index);

	provider = devm_of_phy_provider_register(&pdev->dev, xvphy_xlate);
	if (IS_ERR(provider)) {
		dev_err(&pdev->dev, "registering provider failed\n");
			return PTR_ERR(provider);
	}

	// Initialize Video PHY
	///// @TODO
	// The GT needs to be initialized after the HDMI RX and TX.
	// The reason for this is the GtRxInitStartCallback
	// calls the RX stream down callback.
	/////

	/* initialize configuration data */
	vphy_config_init(&config, vphydev->iomem);

	/* Initialize HDMI VPHY */
	Status = XVphy_HdmiInitialize(&vphydev->xvphy, 0,
				&config, 100*1000*1000/*@TODO logic clock*/);
	if (Status != XST_SUCCESS) {
		printk(KERN_INFO "HDMI VPHY initialization error\n\r");
		return XST_FAILURE;
	}

	Data = XVphy_GetVersion(&vphydev->xvphy);
	xil_printf("VPhy version : %02d.%02d (%04x)\n\r", ((Data >> 24) & 0xFF), ((Data >> 16) & 0xFF), (Data & 0xFFFF));

	ret = devm_request_threaded_irq(&pdev->dev, vphydev->irq, xvphy_irq_handler, xvphy_irq_thread,
			IRQF_TRIGGER_HIGH /*IRQF_SHARED*/, "xilinx-vphy", vphydev/*dev_id*/);

	if (ret) {
		dev_err(&pdev->dev, "unable to request IRQ %d\n", vphydev->irq);
		return ret;
	}

	vphydev->dev = &pdev->dev;
	/* set a pointer to our driver data */
	platform_set_drvdata(pdev, vphydev);
	printk(KERN_INFO "HDMI VPHY initialization completed\n\r");

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id xvphy_of_match[] = {
	{ .compatible = "xlnx,vphy-1.1" },
	{},
};
MODULE_DEVICE_TABLE(of, xvphy_of_match);

static struct platform_driver xvphy_driver = {
	.probe = xvphy_probe,
	.driver = {
		.name = "xilinx-vphy",
		.of_match_table	= xvphy_of_match,
	},
};
module_platform_driver(xvphy_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leon Woestenberg <leon@sidebranch.com>");
MODULE_DESCRIPTION("Xilinx Vphy driver");

