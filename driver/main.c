// SPDX-License-Identifier: BSD-2-Clause

/*
 * LitePCIe driver
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2020 / EnjoyDigital  / florent@enjoy-digital.fr
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mmtimer.h>
#include <linux/miscdevice.h>
#include <linux/posix-timers.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/log2.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/version.h>

#include "litepcie.h"
#include "csr.h"
#include "config.h"
#include "flags.h"
#include "mem.h"

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE_MINOR_COUNT 32
#define TX_BUF_SIZE (ETHMAC_TX_SLOTS * ETHMAC_SLOT_SIZE)

static u8 mac_addr[] = {0x12, 0x2e, 0x60, 0xbe, 0xef, 0xbb};
struct skb_buffer_priv {
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	u32 tx_len;
	dma_addr_t tx_dma_addr;
	struct sk_buff *tx_skb;
};
struct liteeth {
        void __iomem *base;
        struct net_device *netdev;
        u32 slot_size;
        /* Tx */
        u32 tx_slot;
        u32 num_tx_slots;

        /* Rx */
        u32 num_rx_slots;

	void *tx_buf;
	dma_addr_t tx_buf_dma;
	struct litepcie_device *lpdev;
	struct napi_struct napi;
	struct skb_buffer_priv buffer[];
};

struct litepcie_device {
	struct pci_dev *dev;
	resource_size_t bar0_size;
	phys_addr_t bar0_phys_addr;
	uint8_t *bar0_addr; /* virtual address of BAR0 */
	int irqs;
	struct liteeth *ethdev;
};

static inline uint32_t litepcie_readl(struct litepcie_device *s, uint32_t addr)
{
	uint32_t val;

	val = readl(s->bar0_addr + addr - CSR_BASE);
	return le32_to_cpu(val);
}

static inline void litepcie_writel(struct litepcie_device *s, uint32_t addr, uint32_t val)
{
	return writel(cpu_to_le32(val), s->bar0_addr + addr - CSR_BASE);
}

static void litepcie_enable_interrupt(struct litepcie_device *s, int irq_num)
{
        uint32_t v;

        v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
        v |= (1 << irq_num);
        litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void litepcie_disable_interrupt(struct litepcie_device *s, int irq_num)
{
        uint32_t v;

        v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
        v &= ~(1 << irq_num);
        litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void liteeth_rx_fill(struct liteeth *, u32);
static void liteeth_clear_pending_tx_dma(struct liteeth *);
static int liteeth_open(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	int i;
	netdev_info(netdev,"liteeth_open\n");

	for (i = 0; i < priv->num_rx_slots; i += 1)
		liteeth_rx_fill(priv, i);

	litepcie_writel(priv->lpdev,CSR_ETHMAC_SRAM_WRITER_ENABLE_ADDR,1);
	litepcie_enable_interrupt(priv->lpdev, ETHTX_INTERRUPT);
	litepcie_enable_interrupt(priv->lpdev, ETHRX_INTERRUPT);
	napi_enable(&priv->napi);
	netif_carrier_on(netdev);
	netif_start_queue(netdev);

	return 0;
}

static int liteeth_stop(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	int i;

	netdev_info(netdev,"liteeth_stop\n");
	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	napi_disable(&priv->napi);
	litepcie_writel(priv->lpdev,CSR_ETHMAC_SRAM_WRITER_ENABLE_ADDR,0);
	litepcie_disable_interrupt(priv->lpdev, ETHTX_INTERRUPT);
	litepcie_disable_interrupt(priv->lpdev, ETHRX_INTERRUPT);

	for (i = 0; i < priv->num_rx_slots; i += 1){
		dma_unmap_single(&priv->lpdev->dev->dev, priv->buffer[i].dma_addr, priv->slot_size, DMA_FROM_DEVICE);
		dev_kfree_skb_any(priv->buffer[i].skb);
	}

	liteeth_clear_pending_tx_dma(priv);

	return 0;
}
static void liteeth_clear_pending_tx_dma(struct liteeth *priv)
{
	struct litepcie_device *lpdev = priv->lpdev;
	u32 pending_tx;
	int i;

	pending_tx = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_PENDING_SLOTS_ADDR);
	for (i = 0; i < priv->num_tx_slots; i++)
		if (pending_tx & (1 << i)) {
			if (priv->buffer[i].tx_len) {
				dma_unmap_single(&priv->lpdev->dev->dev, priv->buffer[i].tx_dma_addr, priv->buffer[i].tx_len, DMA_TO_DEVICE);
				dev_kfree_skb_any(priv->buffer[i].tx_skb);
			}
		}

	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_CLEAR_PENDING_ADDR, pending_tx);
}
static int liteeth_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct litepcie_device *lpdev = priv->lpdev;
	bool copy = false;

	/* Reject oversize packets */
	if (unlikely(skb->len > priv->slot_size)) {
		dev_kfree_skb_any(skb);
		netdev->stats.tx_dropped++;
		netdev->stats.tx_errors++;

		return NETDEV_TX_OK;
	}

	if (!litepcie_readl(lpdev,CSR_ETHMAC_SRAM_READER_READY_ADDR))
		goto busy;

	if (litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_PENDING_SLOTS_ADDR) & (1 << priv->tx_slot))
		liteeth_clear_pending_tx_dma(priv);

	if (IS_ALIGNED((unsigned long)skb->data, 4)) {
		priv->buffer[priv->tx_slot].tx_dma_addr = dma_map_single(&priv->lpdev->dev->dev, skb->data, skb->len, DMA_TO_DEVICE);
		priv->buffer[priv->tx_slot].tx_len = skb->len;
		priv->buffer[priv->tx_slot].tx_skb = skb;
	}
	else {
		memcpy_toio(priv->tx_buf + priv->tx_slot * priv->slot_size, skb->data, skb->len);
		priv->buffer[priv->tx_slot].tx_dma_addr = priv->tx_buf_dma + priv->tx_slot * priv->slot_size;
		priv->buffer[priv->tx_slot].tx_len = 0;
		priv->buffer[priv->tx_slot].tx_skb = NULL;
		copy = true;
	}

	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_SLOT_ADDR, priv->tx_slot);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_LENGTH_ADDR, skb->len);
	litepcie_writel(priv->lpdev, CSR_ETHMAC_SRAM_READER_PCIE_HOST_ADDRS_ADDR + (priv->tx_slot << 2), priv->buffer[priv->tx_slot].tx_dma_addr);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_START_ADDR, 1);

	//pci_info(priv->lpdev->dev, "tx slot: %d, virtual addr 0x%llx, physical addr 0x%llx, dma addr 0x%llx, copy %d\n", priv->tx_slot, (void *)skb->data, virt_to_phys(skb->data),priv->buffer[priv->tx_slot].tx_dma_addr, copy);

	priv->tx_slot = (priv->tx_slot + 1) % priv->num_tx_slots;

	netdev->stats.tx_bytes += skb->len;
	netdev->stats.tx_packets++;
	
	if (copy)
		dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;

busy:                
	netif_stop_queue(netdev);

	return NETDEV_TX_BUSY;
}

static void liteeth_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct liteeth *priv = netdev_priv(dev);
	struct litepcie_device *lpdev = priv->lpdev;
	struct netdev_queue *queue = netdev_get_tx_queue(dev, txqueue);
	u32 reg, slots;
	
	slots = litepcie_readl(lpdev,CSR_ETHMAC_SRAM_READER_LEVEL_ADDR);
	reg = litepcie_readl(lpdev,CSR_ETHMAC_SRAM_READER_READY_ADDR);
	netdev_info(dev, "litepcie: liteeth_tx_timeout, reg %u, slots %u\n", reg, slots);
	if (reg)
		netif_tx_wake_queue(queue);
}
static const struct net_device_ops liteeth_netdev_ops = {
	.ndo_open		= liteeth_open,
	.ndo_stop		= liteeth_stop,
	.ndo_start_xmit         = liteeth_start_xmit,
	.ndo_tx_timeout		= liteeth_tx_timeout,
};

static void liteeth_rx_fill(struct liteeth *priv, u32 rx_slot)
{
	struct sk_buff *skb;

	skb = __netdev_alloc_skb_ip_align(priv->netdev, priv->slot_size, GFP_ATOMIC);

	WARN_ON(!IS_ALIGNED((unsigned long)skb->data, 4));
	priv->buffer[rx_slot].skb = skb;
	priv->buffer[rx_slot].dma_addr = dma_map_single(&priv->lpdev->dev->dev, skb->data, priv->slot_size, DMA_FROM_DEVICE);
	//pci_info(priv->lpdev->dev, "rx slot: %d, dma addr 0x%llx\n", rx_slot, priv->buffer[rx_slot].dma_addr);
	litepcie_writel(priv->lpdev, CSR_ETHMAC_SRAM_WRITER_PCIE_HOST_ADDRS_ADDR + (rx_slot << 2), priv->buffer[rx_slot].dma_addr);
}
static void handle_ethrx_interrupt(struct net_device *netdev, u32 rx_slot, u32 len)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct sk_buff *skb;
	unsigned char *data;

	dma_unmap_single(&priv->lpdev->dev->dev, priv->buffer[rx_slot].dma_addr, priv->slot_size, DMA_FROM_DEVICE);

	skb = priv->buffer[rx_slot].skb;

	data = skb_put(skb, len);

	skb->protocol = eth_type_trans(skb, netdev);

	napi_gro_receive(&priv->napi, skb);

	liteeth_rx_fill(priv, rx_slot);

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += len;

	return;
}

static irqreturn_t litepcie_interrupt(int irq, void *data)
{
	struct litepcie_device *lpdev = (struct litepcie_device *) data;
	struct net_device *netdev = lpdev->ethdev->netdev;
	struct liteeth *priv = netdev_priv(netdev);
	u32 rx_pending;
	u32 irq_enable;

	irq_enable = litepcie_readl(lpdev, CSR_PCIE_MSI_ENABLE_ADDR);

	if (irq_enable & (1 << ETHRX_INTERRUPT)) {
		rx_pending = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_WRITER_PENDING_SLOTS_ADDR);
		if (rx_pending != 0) {
			litepcie_disable_interrupt(priv->lpdev, ETHRX_INTERRUPT);
			napi_schedule(&priv->napi);
		}
	}

	if ((irq_enable & (1 << ETHTX_INTERRUPT)) && netif_queue_stopped(netdev) && litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_READY_ADDR))
		netif_wake_queue(netdev);

	return IRQ_HANDLED;
}

static int liteeth_napi_poll(struct napi_struct *napi, int budget)
{
	struct liteeth *priv = container_of(napi, struct liteeth, napi);
	struct litepcie_device *lpdev = priv->lpdev;
	u32 rx_pending, length, clear_mask;
	int work_down, i;

	clear_mask = 0;
	work_down = 0;
	rx_pending = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_WRITER_PENDING_SLOTS_ADDR);
	for (i = 0; i < priv->num_rx_slots; i++) {
		if (rx_pending & (1 << i)) {
			length = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_WRITER_PENDING_LENGTH_ADDR + (i << 2));
			handle_ethrx_interrupt(priv->netdev, i, length);
			clear_mask |= 1 << i;
			work_down += 1;
			if (work_down >= budget)
				break;
		}
	}
	//pci_info(lpdev->dev, "liteeth_napi_poll rx_pending %u, work_down %d, budget %d\n", rx_pending, work_down, budget);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_WRITER_CLEAR_PENDING_ADDR, clear_mask);

	if (work_down < budget && napi_complete_done(napi, work_down))
		litepcie_enable_interrupt(lpdev, ETHRX_INTERRUPT);

	return work_down;
}

static void liteeth_setup_slots(struct liteeth *priv)
{
	priv->num_rx_slots = ETHMAC_RX_SLOTS;
	priv->num_tx_slots = ETHMAC_TX_SLOTS;
	priv->slot_size = ETHMAC_SLOT_SIZE;
}

static int liteeth_init(struct litepcie_device *lpdev)
{
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct liteeth *priv;
	int err;

	pdev = lpdev->dev;
	netdev = devm_alloc_etherdev(&pdev->dev, sizeof(*priv) + sizeof(struct skb_buffer_priv) * ETHMAC_RX_SLOTS);
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, &pdev->dev);

	priv = netdev_priv(netdev);
	priv->netdev = netdev;

	lpdev->ethdev = priv;
	priv->lpdev = lpdev;

	netdev->irq = pci_irq_vector(lpdev->dev, 0);

	liteeth_setup_slots(priv);

	priv->tx_slot = 0;

	priv->tx_buf = dma_alloc_coherent(&pdev->dev, TX_BUF_SIZE, &priv->tx_buf_dma, GFP_ATOMIC);

#if 0
	memcpy(netdev->dev_addr, mac_addr, ETH_ALEN);
#else
	 eth_hw_addr_set(netdev, mac_addr);
#endif

	netdev->netdev_ops = &liteeth_netdev_ops;
	
	netdev->watchdog_timeo = 60 * HZ;
	
#if 0
	netif_napi_add(netdev, &priv->napi, liteeth_napi_poll, NAPI_POLL_WEIGHT);
#else
	netif_napi_add(netdev, &priv->napi, liteeth_napi_poll);
#endif
	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev %d\n", err);
		return err;		//FIXME:add post-clean if error occurs
	}

	netdev_info(netdev, "irq %d slots: tx %d rx %d size %d\n",
		    netdev->irq, priv->num_tx_slots, priv->num_rx_slots, priv->slot_size);

	return 0;
}
static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;
	int irqs = 0;
	uint8_t rev_id;
	int i;
	char fpga_identifier[256];
	struct litepcie_device *litepcie_dev = NULL;

	dev_info(&dev->dev, "\e[1m[Probing device]\e[0m\n");

	litepcie_dev = devm_kzalloc(&dev->dev, sizeof(struct litepcie_device), GFP_KERNEL);
	if (!litepcie_dev) {
		ret = -ENOMEM;
		goto fail1;
	}

	pci_set_drvdata(dev, litepcie_dev);
	litepcie_dev->dev = dev;

	ret = pcim_enable_device(dev);
	if (ret != 0) {
		dev_err(&dev->dev, "Cannot enable device\n");
		goto fail1;
	}

	ret = -EIO;

	/* check device version */
	pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
	if (rev_id != 0) {
		dev_err(&dev->dev, "Unsupported device version %d\n", rev_id);
		goto fail1;
	}

	/* check bar0 config */
	if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM)) {
		dev_err(&dev->dev, "Invalid BAR0 configuration\n");
		goto fail1;
	}

	if (pcim_iomap_regions(dev, BIT(0), LITEPCIE_NAME) < 0) {
		dev_err(&dev->dev, "Could not request regions\n");
		goto fail1;
	}

	litepcie_dev->bar0_addr = pcim_iomap_table(dev)[0];
	if (!litepcie_dev->bar0_addr) {
		dev_err(&dev->dev, "Could not map BAR0\n");
		goto fail1;
	}

	/* show identifier */
	for (i = 0; i < 256; i++)
		fpga_identifier[i] = litepcie_readl(litepcie_dev, CSR_IDENTIFIER_MEM_BASE + i*4);
	dev_info(&dev->dev, "Version %s\n", fpga_identifier);

	pci_set_master(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
	ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
#else
	ret = dma_set_mask(&dev->dev, DMA_BIT_MASK(DMA_ADDR_WIDTH));
#endif
	if (ret) {
		dev_err(&dev->dev, "Failed to set DMA mask\n");
		goto fail1;
	};

	irqs = pci_alloc_irq_vectors(dev, 1, 1, PCI_IRQ_MSI);
	if (irqs < 0) {
		dev_err(&dev->dev, "Failed to enable MSI\n");
		ret = irqs;
		goto fail1;
	}
	dev_info(&dev->dev, "%d MSI IRQs allocated.\n", irqs);

	litepcie_dev->irqs = 0;
	for (i = 0; i < irqs; i++) {
		int irq = pci_irq_vector(dev, i);

		ret = request_irq(irq, litepcie_interrupt, IRQF_SHARED, LITEPCIE_NAME, litepcie_dev);
		if (ret < 0) {
			dev_err(&dev->dev, " Failed to allocate IRQ %d\n", dev->irq);
			while (--i >= 0) {
				irq = pci_irq_vector(dev, i);
				free_irq(irq, dev);
			}
			goto fail2;
		}
		litepcie_dev->irqs += 1;
	}
	
	liteeth_init(litepcie_dev);

	return 0;

fail2:
	pci_free_irq_vectors(dev);
fail1:
	return ret;
}

static void litepcie_pci_remove(struct pci_dev *dev)
{
	int i, irq;
	struct litepcie_device *litepcie_dev;
	struct liteeth *priv;

	litepcie_dev = pci_get_drvdata(dev);
	priv = litepcie_dev->ethdev;

	dev_info(&dev->dev, "\e[1m[Removing device]\e[0m\n");

	/* Disable all interrupts */
	litepcie_writel(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR, 0);
	
	dma_free_coherent(&dev->dev, TX_BUF_SIZE, priv->tx_buf, priv->tx_buf_dma);

	/* Free all interrupts */
	for (i = 0; i < litepcie_dev->irqs; i++) {
		irq = pci_irq_vector(dev, i);
		free_irq(irq, litepcie_dev);
	}
	pci_free_irq_vectors(dev);
}

static const struct pci_device_id litepcie_pci_ids[] = {
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_S7_GEN2_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN2_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X1), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X2), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X4), },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_US_GEN3_X8), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X8),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN2_X16), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X8),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN3_X16), },

	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X1),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X2),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X4),  },
	{ PCI_DEVICE(PCIE_FPGA_VENDOR_ID, PCIE_FPGA_DEVICE_ID_USP_GEN4_X8),  },

	{ 0, }
};
MODULE_DEVICE_TABLE(pci, litepcie_pci_ids);

static struct pci_driver litepcie_pci_driver = {
	.name = LITEPCIE_NAME,
	.id_table = litepcie_pci_ids,
	.probe = litepcie_pci_probe,
	.remove = litepcie_pci_remove,
};


static int __init litepcie_module_init(void)
{
	int ret;
	ret = pci_register_driver(&litepcie_pci_driver);
	if (ret < 0) {
		pr_err(" Error while registering PCI driver\n");
	}

	return ret;
}

static void __exit litepcie_module_exit(void)
{
	pci_unregister_driver(&litepcie_pci_driver);
}


module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
