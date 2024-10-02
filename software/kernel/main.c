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

/* Define the MAC address used for the device */
static u8 mac_addr[] = {0x12, 0x2e, 0x60, 0xbe, 0xef, 0xbb};

/* Structure to hold the buffer private information for SKB */
struct skb_buffer_priv {
	struct sk_buff *skb;  /* Socket buffer */
	dma_addr_t dma_addr;  /* DMA address */
	u32 tx_len;           /* Transmission length */
	dma_addr_t tx_dma_addr; /* DMA address for transmission */
	struct sk_buff *tx_skb; /* Socket buffer for transmission */
};

/* Structure to hold the LiteEth device information */
struct liteeth {
	void __iomem *base;        /* Base I/O memory address */
	struct net_device *netdev; /* Network device */
	u32 slot_size;             /* Slot size */

	/* Tx */
	u32 tx_slot;               /* Transmission slot */
	u32 num_tx_slots;          /* Number of transmission slots */

	/* Rx */
	u32 num_rx_slots;          /* Number of reception slots */

	void *tx_buf;              /* Transmission buffer */
	dma_addr_t tx_buf_dma;     /* DMA address for transmission buffer */
	struct litepcie_device *lpdev; /* LitePCIe device */
	struct napi_struct napi;   /* NAPI structure */
	struct skb_buffer_priv buffer[]; /* Buffer array */
};

/* Structure to hold the LitePCIe device information */
struct litepcie_device {
	struct pci_dev *dev;      /* PCI device */
	resource_size_t bar0_size; /* Size of BAR0 */
	phys_addr_t bar0_phys_addr; /* Physical address of BAR0 */
	uint8_t *bar0_addr;       /* Virtual address of BAR0 */
	int irqs;                 /* Number of IRQs */
	struct liteeth *ethdev;   /* LiteEth device */
};

/* Function to read a 32-bit value from a LitePCIe device register */
static inline uint32_t litepcie_readl(struct litepcie_device *s, uint32_t addr)
{
	uint32_t val;

	val = readl(s->bar0_addr + addr - CSR_BASE);
	return le32_to_cpu(val);
}

/* Function to write a 32-bit value to a LitePCIe device register */
static inline void litepcie_writel(struct litepcie_device *s, uint32_t addr, uint32_t val)
{
	writel(cpu_to_le32(val), s->bar0_addr + addr - CSR_BASE);
}

/* Function to enable a specific interrupt on a LitePCIe device */
static void litepcie_enable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

	/* Read the current interrupt enable register value */
	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);

	/* Set the bit corresponding to the given interrupt number */
	v |= (1 << irq_num);

	/* Write the updated value back to the register */
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

/* Function to disable a specific interrupt on a LitePCIe device */
static void litepcie_disable_interrupt(struct litepcie_device *s, int irq_num)
{
	uint32_t v;

	/* Read the current interrupt enable register value */
	v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);

	/* Clear the bit corresponding to the given interrupt number */
	v &= ~(1 << irq_num);

	/* Write the updated value back to the register */
	litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

/* Forward declarations of functions */
static void liteeth_rx_fill(struct liteeth *, u32);
static void liteeth_clear_pending_tx_dma(struct liteeth *);

/* Function to open the LiteEth network device */
static int liteeth_open(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	int i;
	netdev_info(netdev, "liteeth_open\n");

	/* Fill the RX slots */
	for (i = 0; i < priv->num_rx_slots; i++)
		liteeth_rx_fill(priv, i);

	/* Enable the SRAM writer */
	litepcie_writel(priv->lpdev, CSR_ETHMAC_SRAM_WRITER_ENABLE_ADDR, 1);

	/* Enable the interrupts for TX and RX */
	litepcie_enable_interrupt(priv->lpdev, ETHMAC_TX_INTERRUPT);
	litepcie_enable_interrupt(priv->lpdev, ETHMAC_RX_INTERRUPT);

	/* Enable NAPI */
	napi_enable(&priv->napi);

	/* Indicate that the carrier is on and start the queue */
	netif_carrier_on(netdev);
	netif_start_queue(netdev);

	return 0;
}

/* Function to stop the LiteEth network device */
static int liteeth_stop(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	int i;

	netdev_info(netdev, "liteeth_stop\n");

	/* Stop the queue and indicate that the carrier is off */
	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	/* Disable NAPI */
	napi_disable(&priv->napi);

	/* Disable the SRAM writer */
	litepcie_writel(priv->lpdev, CSR_ETHMAC_SRAM_WRITER_ENABLE_ADDR, 0);

	/* Disable the interrupts for TX and RX */
	litepcie_disable_interrupt(priv->lpdev, ETHMAC_TX_INTERRUPT);
	litepcie_disable_interrupt(priv->lpdev, ETHMAC_RX_INTERRUPT);

	/* Unmap and free the RX slots */
	for (i = 0; i < priv->num_rx_slots; i++) {
		dma_unmap_single(&priv->lpdev->dev->dev, priv->buffer[i].dma_addr, priv->slot_size, DMA_FROM_DEVICE);
		dev_kfree_skb_any(priv->buffer[i].skb);
	}

	/* Clear any pending TX DMA */
	liteeth_clear_pending_tx_dma(priv);

	return 0;
}

/* Function to clear any pending TX DMA on a LiteEth device */
static void liteeth_clear_pending_tx_dma(struct liteeth *priv)
{
	struct litepcie_device *lpdev = priv->lpdev;
	u32 pending_tx;
	int i;

	/* Read the pending TX slots */
	pending_tx = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_PENDING_SLOTS_ADDR);

	/* Iterate through all TX slots */
	for (i = 0; i < priv->num_tx_slots; i++)
		if (pending_tx & (1 << i)) {
			if (priv->buffer[i].tx_len) {
				/* Unmap the DMA address and free the SKB */
				dma_unmap_single(&priv->lpdev->dev->dev, priv->buffer[i].tx_dma_addr, priv->buffer[i].tx_len, DMA_TO_DEVICE);
				dev_kfree_skb_any(priv->buffer[i].tx_skb);
			}
		}

	/* Clear the pending TX slots */
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_CLEAR_PENDING_ADDR, pending_tx);
}

/* Function to start transmitting a packet on the LiteEth network device */
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

	/* Check if SRAM reader is ready */
	if (!litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_READY_ADDR))
		goto busy;

	/* Clear pending TX DMA if necessary */
	if (litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_PENDING_SLOTS_ADDR) & (1 << priv->tx_slot))
		liteeth_clear_pending_tx_dma(priv);

	/* Check if the packet data is 4-byte aligned */
	if (IS_ALIGNED((unsigned long)skb->data, 4)) {
		/* Map the SKB data for DMA */
		priv->buffer[priv->tx_slot].tx_dma_addr = dma_map_single(&priv->lpdev->dev->dev, skb->data, skb->len, DMA_TO_DEVICE);
		priv->buffer[priv->tx_slot].tx_len = skb->len;
		priv->buffer[priv->tx_slot].tx_skb = skb;
	} else {
		/* Copy the SKB data to the transmission buffer */
		memcpy_toio(priv->tx_buf + priv->tx_slot * priv->slot_size, skb->data, skb->len);
		priv->buffer[priv->tx_slot].tx_dma_addr = priv->tx_buf_dma + priv->tx_slot * priv->slot_size;
		priv->buffer[priv->tx_slot].tx_len = 0;
		priv->buffer[priv->tx_slot].tx_skb = NULL;
		copy = true;
	}

	/* Write the necessary registers to start the transmission */
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_SLOT_ADDR, priv->tx_slot);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_LENGTH_ADDR, skb->len);
	litepcie_writel(priv->lpdev, CSR_ETHMAC_SRAM_READER_PCIE_HOST_ADDRS_ADDR + (priv->tx_slot << 2), priv->buffer[priv->tx_slot].tx_dma_addr);
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_READER_START_ADDR, 1);

	/* Update the TX slot */
	priv->tx_slot = (priv->tx_slot + 1) % priv->num_tx_slots;

	/* Update statistics */
	netdev->stats.tx_bytes += skb->len;
	netdev->stats.tx_packets++;

	if (copy)
		dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;

busy:
	/* If the device is busy, stop the queue */
	netif_stop_queue(netdev);

	return NETDEV_TX_BUSY;
}

/* Function to handle TX timeout on the LiteEth network device */
static void liteeth_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct liteeth *priv = netdev_priv(dev);
	struct litepcie_device *lpdev = priv->lpdev;
	struct netdev_queue *queue = netdev_get_tx_queue(dev, txqueue);
	u32 reg, slots;

	/* Read the number of slots and the ready register */
	slots = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_LEVEL_ADDR);
	reg = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_READY_ADDR);
	netdev_info(dev, "litepcie: liteeth_tx_timeout, reg %u, slots %u\n", reg, slots);

	/* If the device is ready, wake the queue */
	if (reg)
		netif_tx_wake_queue(queue);
}

/* Net device operations for LiteEth */
static const struct net_device_ops liteeth_netdev_ops = {
	.ndo_open = liteeth_open,           /* Open operation */
	.ndo_stop = liteeth_stop,           /* Stop operation */
	.ndo_start_xmit = liteeth_start_xmit, /* Start transmit operation */
	.ndo_tx_timeout = liteeth_tx_timeout, /* TX timeout operation */
};

/* Function to fill the RX slots with SKBs */
static void liteeth_rx_fill(struct liteeth *priv, u32 rx_slot)
{
	struct sk_buff *skb;

	/* Allocate an SKB with the specified slot size */
	skb = __netdev_alloc_skb_ip_align(priv->netdev, priv->slot_size, GFP_ATOMIC);

	/* Ensure the SKB data is 4-byte aligned */
	WARN_ON(!IS_ALIGNED((unsigned long)skb->data, 4));
	priv->buffer[rx_slot].skb = skb;

	/* Map the SKB data for DMA */
	priv->buffer[rx_slot].dma_addr = dma_map_single(&priv->lpdev->dev->dev, skb->data, priv->slot_size, DMA_FROM_DEVICE);

	/* Write the DMA address to the corresponding register */
	litepcie_writel(priv->lpdev, CSR_ETHMAC_SRAM_WRITER_PCIE_HOST_ADDRS_ADDR + (rx_slot << 2), priv->buffer[rx_slot].dma_addr);
}

/* Function to handle RX interrupt */
static void handle_ethrx_interrupt(struct net_device *netdev, u32 rx_slot, u32 len)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct sk_buff *skb;
	unsigned char *data;

	/* Unmap the DMA address */
	dma_unmap_single(&priv->lpdev->dev->dev, priv->buffer[rx_slot].dma_addr, priv->slot_size, DMA_FROM_DEVICE);

	/* Get the SKB for the specified RX slot */
	skb = priv->buffer[rx_slot].skb;

	/* Append data to the SKB and set its length */
	data = skb_put(skb, len);

	/* Set the protocol for the SKB */
	skb->protocol = eth_type_trans(skb, netdev);

	/* Pass the SKB to the network stack */
	napi_gro_receive(&priv->napi, skb);

	/* Refill the RX slot */
	liteeth_rx_fill(priv, rx_slot);

	/* Update statistics */
	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += len;

	return;
}

/* Function to handle interrupts for LitePCIe device */
static irqreturn_t litepcie_interrupt(int irq, void *data)
{
	struct litepcie_device *lpdev = (struct litepcie_device *)data;
	struct net_device *netdev = lpdev->ethdev->netdev;
	struct liteeth *priv = netdev_priv(netdev);
	u32 rx_pending;
	u32 irq_enable;

	/* Read the interrupt enable register */
	irq_enable = litepcie_readl(lpdev, CSR_PCIE_MSI_ENABLE_ADDR);

	/* Handle RX interrupt */
	if (irq_enable & (1 << ETHMAC_RX_INTERRUPT)) {
		rx_pending = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_WRITER_PENDING_SLOTS_ADDR);
		if (rx_pending != 0) {
			/* Disable RX interrupt and schedule NAPI */
			litepcie_disable_interrupt(priv->lpdev, ETHMAC_RX_INTERRUPT);
			napi_schedule(&priv->napi);
		}
	}

	/* Handle TX interrupt */
	if ((irq_enable & (1 << ETHMAC_TX_INTERRUPT)) && netif_queue_stopped(netdev) && litepcie_readl(lpdev, CSR_ETHMAC_SRAM_READER_READY_ADDR))
		netif_wake_queue(netdev);

	return IRQ_HANDLED;
}

/* NAPI poll function */
static int liteeth_napi_poll(struct napi_struct *napi, int budget)
{
	struct liteeth *priv = container_of(napi, struct liteeth, napi);
	struct litepcie_device *lpdev = priv->lpdev;
	u32 rx_pending, length, clear_mask;
	int work_done, i;

	clear_mask = 0;
	work_done = 0;
	rx_pending = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_WRITER_PENDING_SLOTS_ADDR);

	/* Process pending RX slots */
	for (i = 0; i < priv->num_rx_slots; i++) {
		if (rx_pending & (1 << i)) {
			/* Read the length of the pending slot */
			length = litepcie_readl(lpdev, CSR_ETHMAC_SRAM_WRITER_PENDING_LENGTH_ADDR + (i << 2));

			/* Handle the RX interrupt for the slot */
			handle_ethrx_interrupt(priv->netdev, i, length);

			/* Update the clear mask and work done count */
			clear_mask |= 1 << i;
			work_done += 1;

			/* If the budget is reached, break the loop */
			if (work_done >= budget)
				break;
		}
	}

	/* Clear the pending RX slots */
	litepcie_writel(lpdev, CSR_ETHMAC_SRAM_WRITER_CLEAR_PENDING_ADDR, clear_mask);

	/* If the work done is less than the budget, complete NAPI and re-enable the interrupt */
	if (work_done < budget && napi_complete_done(napi, work_done))
		litepcie_enable_interrupt(lpdev, ETHMAC_RX_INTERRUPT);

	return work_done;
}

/* Function to setup the slots for LiteEth */
static void liteeth_setup_slots(struct liteeth *priv)
{
	priv->num_rx_slots = ETHMAC_RX_SLOTS;
	priv->num_tx_slots = ETHMAC_TX_SLOTS;
	priv->slot_size = ETHMAC_SLOT_SIZE;
}

/* Function to initialize the LiteEth device */
static int liteeth_init(struct litepcie_device *lpdev)
{
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct liteeth *priv;
	int err;

	pdev = lpdev->dev;

	/* Allocate and initialize the network device */
	netdev = devm_alloc_etherdev(&pdev->dev, sizeof(*priv) + sizeof(struct skb_buffer_priv) * ETHMAC_RX_SLOTS);
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, &pdev->dev);

	priv = netdev_priv(netdev);
	priv->netdev = netdev;

	lpdev->ethdev = priv;
	priv->lpdev = lpdev;

	/* Get the IRQ vector for the device */
	netdev->irq = pci_irq_vector(lpdev->dev, 0);

	/* Setup the RX and TX slots */
	liteeth_setup_slots(priv);

	priv->tx_slot = 0;

	/* Allocate coherent memory for the transmission buffer */
	priv->tx_buf = dma_alloc_coherent(&pdev->dev, TX_BUF_SIZE, &priv->tx_buf_dma, GFP_ATOMIC);

	/* Set the hardware address for the network device */
	eth_hw_addr_set(netdev, mac_addr);

	netdev->netdev_ops = &liteeth_netdev_ops;
	netdev->watchdog_timeo = 60 * HZ;

	/* Add NAPI to the network device */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	netif_napi_add(netdev, &priv->napi, liteeth_napi_poll, 64);
#else
	netif_napi_add(netdev, &priv->napi, liteeth_napi_poll);
#endif

	/* Register the network device */
	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev %d\n", err);
		return err;
	}

	netdev_info(netdev, "irq %d slots: tx %d rx %d size %d\n",
		    netdev->irq, priv->num_tx_slots, priv->num_rx_slots, priv->slot_size);

	return 0;
}

/* Function to probe the LitePCIe PCI device */
static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;
	int irqs = 0;
	uint8_t rev_id;
	int i;
	char fpga_identifier[256];
	struct litepcie_device *litepcie_dev = NULL;

	dev_info(&dev->dev, "\e[1m[Probing device]\e[0m\n");

	/* Allocate memory for the LitePCIe device structure */
	litepcie_dev = devm_kzalloc(&dev->dev, sizeof(struct litepcie_device), GFP_KERNEL);
	if (!litepcie_dev) {
		ret = -ENOMEM;
		goto fail1;
	}

	pci_set_drvdata(dev, litepcie_dev);
	litepcie_dev->dev = dev;

	/* Enable the PCI device */
	ret = pcim_enable_device(dev);
	if (ret != 0) {
		dev_err(&dev->dev, "Cannot enable device\n");
		goto fail1;
	}

	ret = -EIO;

	/* Check the device version */
	pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
	if (rev_id != 0) {
		dev_err(&dev->dev, "Unsupported device version %d\n", rev_id);
		goto fail1;
	}

	/* Check the BAR0 configuration */
	if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM)) {
		dev_err(&dev->dev, "Invalid BAR0 configuration\n");
		goto fail1;
	}

	/* Request and map BAR0 */
	if (pcim_iomap_regions(dev, BIT(0), LITEPCIE_NAME) < 0) {
		dev_err(&dev->dev, "Could not request regions\n");
		goto fail1;
	}

	litepcie_dev->bar0_addr = pcim_iomap_table(dev)[0];
	if (!litepcie_dev->bar0_addr) {
		dev_err(&dev->dev, "Could not map BAR0\n");
		goto fail1;
	}

	/* Read and display the FPGA identifier */
	for (i = 0; i < 256; i++)
		fpga_identifier[i] = litepcie_readl(litepcie_dev, CSR_IDENTIFIER_MEM_BASE + i * 4);
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
	}

	/* Allocate MSI IRQ vectors */
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

		/* Request IRQ */
		ret = request_irq(irq, litepcie_interrupt, IRQF_SHARED, LITEPCIE_NAME, litepcie_dev);
		if (ret < 0) {
			dev_err(&dev->dev, "Failed to allocate IRQ %d\n", dev->irq);
			while (--i >= 0) {
				irq = pci_irq_vector(dev, i);
				free_irq(irq, dev);
			}
			goto fail2;
		}
		litepcie_dev->irqs += 1;
	}

	/* Initialize the LiteEth device */
	liteeth_init(litepcie_dev);

	return 0;

fail2:
	pci_free_irq_vectors(dev);
fail1:
	return ret;
}

/* Function to remove the LitePCIe PCI device */
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

	/* Free the DMA coherent memory */
	dma_free_coherent(&dev->dev, TX_BUF_SIZE, priv->tx_buf, priv->tx_buf_dma);

	/* Free all IRQs */
	for (i = 0; i < litepcie_dev->irqs; i++) {
		irq = pci_irq_vector(dev, i);
		free_irq(irq, litepcie_dev);
	}
	pci_free_irq_vectors(dev);
}

/* PCI device ID table */
static const struct pci_device_id litepcie_pci_ids[] = {
	/* Xilinx */
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_S7_GEN2_X1), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_S7_GEN2_X2), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_S7_GEN2_X4), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_S7_GEN2_X8), },

	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN2_X1), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN2_X2), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN2_X4), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN2_X8), },

	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN3_X1), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN3_X2), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN3_X4), },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_US_GEN3_X8), },

	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN2_X1),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN2_X2),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN2_X4),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN2_X8),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN2_X16), },

	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN3_X1),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN3_X2),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN3_X4),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN3_X8),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN3_X16), },

	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN4_X1),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN4_X2),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN4_X4),  },
	{ PCI_DEVICE(PCIE_XILINX_VENDOR_ID, PCIE_XILINX_DEVICE_ID_USP_GEN4_X8),  },

	/* Lattice */
	{ PCI_DEVICE(PCIE_LATTICE_VENDOR_ID, PCIE_LATTICE_DEVICE_ID_CPNX_GEN3_X4),  },

	/* Gowin */
	{ PCI_DEVICE(PCIE_GOWIN_VENDOR_ID, PCIE_GOWIN_DEVICE_ID_GW5AT_GEN2_X4),  },

	{ 0, }
};
MODULE_DEVICE_TABLE(pci, litepcie_pci_ids);

/* PCI driver structure */
static struct pci_driver litepcie_pci_driver = {
	.name = LITEPCIE_NAME,
	.id_table = litepcie_pci_ids,
	.probe = litepcie_pci_probe,
	.remove = litepcie_pci_remove,
};

/* Module initialization function */
static int __init litepcie_module_init(void)
{
	int ret;
	ret = pci_register_driver(&litepcie_pci_driver);
	if (ret < 0) {
		pr_err("Error while registering PCI driver\n");
	}
	return ret;
}

/* Module exit function */
static void __exit litepcie_module_exit(void)
{
	pci_unregister_driver(&litepcie_pci_driver);
}

module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
