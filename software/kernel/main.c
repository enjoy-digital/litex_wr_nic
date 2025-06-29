/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe WR-NIC driver
 *
 * This file is part of LiteX-WR-NIC.
 *
 * Copyright (C) 2024      / Warsaw University of Technology
 * Copyright (C) 2018-2024 / EnjoyDigital  / <enjoy-digital.fr>
 * Copyright (C) 2022      / Tongchen126   / https://github.com/tongchen126
 *
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
#include <linux/platform_device.h>
#include <linux/version.h>

#if defined(__arm__) || defined(__aarch64__)
#include <linux/dma-direct.h>
#endif

#include <linux/ptp_clock_kernel.h>

#include "litepcie.h"
#include "csr.h"
#include "config.h"
#include "flags.h"
#include "soc.h"

//#define DEBUG_CSR
//#define DEBUG_MSI

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE_MINOR_COUNT 32

#ifndef CSR_BASE
#define CSR_BASE 0x00000000
#endif

/* CHECKME: Move to FPGA/CSR ?*/
static uint8_t liteeth_mac_addr_base[] = {0x10, 0xe2, 0xd5, 0x00, 0x00, 0x00};

/* -----------------------------------------------------------------------------------------------*/
/*                                     Structs and Definitions                                    */
/* -----------------------------------------------------------------------------------------------*/

/* Structure to hold the CSR addresses for an Ethernet MAC */
struct ethmac_csr_addresses {
	uint32_t sram_writer_enable_addr;
	uint32_t sram_writer_pending_slots_addr;
	uint32_t sram_writer_pending_length_addr;
	uint32_t sram_writer_pending_clear_addr;
	uint32_t sram_writer_pcie_host_addrs_addr;
	uint32_t sram_reader_ready_addr;
	uint32_t sram_reader_pending_slots_addr;
	uint32_t sram_reader_pending_clear_addr;
	uint32_t sram_reader_slot_addr;
	uint32_t sram_reader_length_addr;
	uint32_t sram_reader_pcie_host_addrs_addr;
	uint32_t sram_reader_start_addr;
	uint32_t sram_reader_level_addr;
};

/* Structure to hold the parameters for an Ethernet MAC */
struct liteeth_params {
	uint32_t slot_size;
	uint32_t rx_slots;
	uint32_t tx_slots;
	uint8_t  rx_interrupt;
	uint8_t  tx_interrupt;
	const struct ethmac_csr_addresses *csrs;
};

/* Define the CSR addresses for each Ethernet MAC */
static const struct ethmac_csr_addresses ethmac_csrs[] = {
#ifdef CSR_ETHMAC0_BASE
	{   /* ETHMAC0 */
		.sram_writer_enable_addr          = CSR_ETHMAC0_SRAM_WRITER_ENABLE_ADDR,
		.sram_writer_pending_slots_addr   = CSR_ETHMAC0_SRAM_WRITER_PENDING_SLOTS_ADDR,
		.sram_writer_pending_length_addr  = CSR_ETHMAC0_SRAM_WRITER_PENDING_LENGTH_ADDR,
		.sram_writer_pending_clear_addr   = CSR_ETHMAC0_SRAM_WRITER_PENDING_CLEAR_ADDR,
		.sram_writer_pcie_host_addrs_addr = CSR_ETHMAC0_SRAM_WRITER_PCIE_HOST_ADDRS_ADDR,
		.sram_reader_ready_addr           = CSR_ETHMAC0_SRAM_READER_READY_ADDR,
		.sram_reader_pending_slots_addr   = CSR_ETHMAC0_SRAM_READER_PENDING_SLOTS_ADDR,
		.sram_reader_pending_clear_addr   = CSR_ETHMAC0_SRAM_READER_PENDING_CLEAR_ADDR,
		.sram_reader_slot_addr            = CSR_ETHMAC0_SRAM_READER_SLOT_ADDR,
		.sram_reader_length_addr          = CSR_ETHMAC0_SRAM_READER_LENGTH_ADDR,
		.sram_reader_pcie_host_addrs_addr = CSR_ETHMAC0_SRAM_READER_PCIE_HOST_ADDRS_ADDR,
		.sram_reader_start_addr           = CSR_ETHMAC0_SRAM_READER_START_ADDR,
		.sram_reader_level_addr           = CSR_ETHMAC0_SRAM_READER_LEVEL_ADDR,
	},
#endif
#ifdef CSR_ETHMAC1_BASE
	{   /* ETHMAC1 */
		.sram_writer_enable_addr          = CSR_ETHMAC1_SRAM_WRITER_ENABLE_ADDR,
		.sram_writer_pending_slots_addr   = CSR_ETHMAC1_SRAM_WRITER_PENDING_SLOTS_ADDR,
		.sram_writer_pending_length_addr  = CSR_ETHMAC1_SRAM_WRITER_PENDING_LENGTH_ADDR,
		.sram_writer_pending_clear_addr   = CSR_ETHMAC1_SRAM_WRITER_PENDING_CLEAR_ADDR,
		.sram_writer_pcie_host_addrs_addr = CSR_ETHMAC1_SRAM_WRITER_PCIE_HOST_ADDRS_ADDR,
		.sram_reader_ready_addr           = CSR_ETHMAC1_SRAM_READER_READY_ADDR,
		.sram_reader_pending_slots_addr   = CSR_ETHMAC1_SRAM_READER_PENDING_SLOTS_ADDR,
		.sram_reader_pending_clear_addr   = CSR_ETHMAC1_SRAM_READER_PENDING_CLEAR_ADDR,
		.sram_reader_slot_addr            = CSR_ETHMAC1_SRAM_READER_SLOT_ADDR,
		.sram_reader_length_addr          = CSR_ETHMAC1_SRAM_READER_LENGTH_ADDR,
		.sram_reader_pcie_host_addrs_addr = CSR_ETHMAC1_SRAM_READER_PCIE_HOST_ADDRS_ADDR,
		.sram_reader_start_addr           = CSR_ETHMAC1_SRAM_READER_START_ADDR,
		.sram_reader_level_addr           = CSR_ETHMAC1_SRAM_READER_LEVEL_ADDR,
	},
#endif
};

/* Define the parameters for each Ethernet MAC */
static const struct liteeth_params liteeth_params[] = {
#ifdef CSR_ETHMAC0_BASE
	{   /* ETHMAC0 */
		.slot_size    = ETHMAC0_SLOT_SIZE,
		.rx_slots     = ETHMAC0_RX_SLOTS,
		.tx_slots     = ETHMAC0_TX_SLOTS,
		.rx_interrupt = ETHMAC0_RX_INTERRUPT,
		.tx_interrupt = ETHMAC0_TX_INTERRUPT,
		.csrs         = &ethmac_csrs[0],
	},
#endif
#ifdef CSR_ETHMAC1_BASE
	{   /* ETHMAC1 */
		.slot_size    = ETHMAC1_SLOT_SIZE,
		.rx_slots     = ETHMAC1_RX_SLOTS,
		.tx_slots     = ETHMAC1_TX_SLOTS,
		.rx_interrupt = ETHMAC1_RX_INTERRUPT,
		.tx_interrupt = ETHMAC1_TX_INTERRUPT,
		.csrs         = &ethmac_csrs[1],
	},
#endif
};

#define NUM_ETHMACS (sizeof(ethmac_csrs) / sizeof(ethmac_csrs[0]))

/* Structure to hold the buffer private information for SKB */
struct skb_buffer_priv {
	uint32_t rx_len;        /* RX Length */
	struct sk_buff *rx_skb; /* RX Socket buffer */
	dma_addr_t rx_dma_addr; /* RX DMA address */
	uint32_t tx_len;        /* TX Length */
	struct sk_buff *tx_skb; /* TX Socket buffer */
	dma_addr_t tx_dma_addr; /* TX DMA address */
};

/* Structure to hold the LiteEth device information */
struct liteeth_device {
	void __iomem *base;                   /* Base I/O memory address */
	struct net_device *netdev;            /* Network device */
	uint32_t slot_size;                   /* Slot size */

	/* Tx */
	uint32_t tx_slot;                     /* Transmission slot */
	uint32_t num_tx_slots;                /* Number of transmission slots */
	uint8_t  tx_irq_num;                  /* TX Interrupt Number */

	/* Rx */
	uint32_t rx_slot;                     /* Reception slot */
	uint32_t num_rx_slots;                /* Number of reception slots */
	uint8_t  rx_irq_num;                  /* RX Interrupt Number */

	spinlock_t lock;

	void *tx_buf;                         /* Transmission buffer */
	dma_addr_t tx_buf_dma;                /* DMA address for transmission buffer */
	struct litepcie_device *litepcie_dev; /* LitePCIe device */
	struct napi_struct napi;              /* NAPI structure */
	int index;                            /* Index of the Ethernet interface */
	const struct ethmac_csr_addresses *csrs; /* Pointer to CSR addresses */
	struct skb_buffer_priv buffer[];      /* Buffer array */
};

/* Structure to hold the LitePCIe device information */
struct litepcie_device {
	struct pci_dev *dev;                          /* PCI device */
	struct platform_device *uart;                 /* UART platform device */
	struct liteeth_device *liteeth_devs[NUM_ETHMACS]; /* LiteEth devices */
	resource_size_t bar0_size;                    /* Size of BAR0 */
	phys_addr_t bar0_phys_addr;                   /* Physical address of BAR0 */
	uint8_t *bar0_addr;                           /* Virtual address of BAR0 */
	int irqs;                                     /* Number of IRQs */

	/* PTP/PTM */
	spinlock_t tmreg_lock;
	struct ptp_clock *litepcie_ptp_clock;
	struct system_time_snapshot snapshot;
	struct ptp_clock_info ptp_caps;
	u64 t1_prev;
	u64 t4_prev;
};

static int litepcie_major;
static int litepcie_minor_idx;
static struct class *litepcie_class;
static dev_t litepcie_dev_t;

/* -----------------------------------------------------------------------------------------------*/
/*                                 LitePCIe MMAP                                                  */
/* -----------------------------------------------------------------------------------------------*/

/* Function to read a 32-bit value from a LitePCIe device register */
static inline uint32_t litepcie_readl(struct litepcie_device *s, uint32_t addr)
{
	uint32_t val;

	val = readl(s->bar0_addr + addr - CSR_BASE);
#ifdef DEBUG_CSR
	dev_dbg(&s->dev->dev, "csr_read: 0x%08x @ 0x%08x", val, addr);
#endif
	return val;
}

/* Function to write a 32-bit value to a LitePCIe device register */
static inline void litepcie_writel(struct litepcie_device *s, uint32_t addr, uint32_t val)
{
#ifdef DEBUG_CSR
	dev_dbg(&s->dev->dev, "csr_write: 0x%08x @ 0x%08x", val, addr);
#endif
	writel(val, s->bar0_addr + addr - CSR_BASE);
}

/* -----------------------------------------------------------------------------------------------*/
/*                               LitePCIe Interrupts                                              */
/* -----------------------------------------------------------------------------------------------*/

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

/* Function to handle interrupts for LitePCIe device */
static irqreturn_t litepcie_interrupt(int irq, void *data)
{
	struct litepcie_device *litepcie_dev = (struct litepcie_device *)data;
	uint32_t clear_mask = 0, irq_vector, irq_enable;
	int i;

	/* Single MSI */
#ifdef CSR_PCIE_MSI_CLEAR_ADDR
	irq_vector = litepcie_readl(litepcie_dev, CSR_PCIE_MSI_VECTOR_ADDR);
	irq_enable = litepcie_readl(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR);
#else
	/* Handle MSI MultiVector / MSI-X */
	irq_vector = 0;
	for (i = 0; i < litepcie_dev->irqs; i++) {
		if (irq == pci_irq_vector(litepcie_dev->dev, i)) {
			irq_vector = (1 << i);
			break;
		}
	}
	irq_enable = litepcie_readl(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR);
#endif

#ifdef DEBUG_MSI
	dev_dbg(&litepcie_dev->dev->dev, "MSI: 0x%x 0x%x\n", irq_vector, irq_enable);
#endif
	irq_vector &= irq_enable;

	for (i = 0; i < NUM_ETHMACS; i++) {
		struct liteeth_device *liteeth_priv = litepcie_dev->liteeth_devs[i];
		uint32_t rx_pending, tx_ready;
		unsigned long flags;

		if (!liteeth_priv)
			continue;

		/* Acquire spinlock */
		spin_lock_irqsave(&liteeth_priv->lock, flags);

		/* Handle RX interrupt */
		if (irq_vector & (1 << liteeth_priv->rx_irq_num)) {
			clear_mask |= (1 << liteeth_priv->rx_irq_num);
			rx_pending = litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_writer_pending_slots_addr);
			if (rx_pending != 0) {
				/* Schedule NAPI */
				napi_schedule(&liteeth_priv->napi);
			}
		}

		/* Handle TX interrupt */
		if (irq_vector & (1 << liteeth_priv->tx_irq_num)) {
			clear_mask |= (1 << liteeth_priv->tx_irq_num);
			tx_ready = litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_reader_ready_addr);
			if ((tx_ready != 0) && netif_queue_stopped(liteeth_priv->netdev)) {
				netif_wake_queue(liteeth_priv->netdev);
			}
		}

		/* Release spinlock */
		spin_unlock_irqrestore(&liteeth_priv->lock, flags);
	}

#ifdef CSR_PCIE_MSI_CLEAR_ADDR
	litepcie_writel(litepcie_dev, CSR_PCIE_MSI_CLEAR_ADDR, clear_mask);
#endif

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------------------------*/
/*                                   LiteEth / NetDev                                             */
/* -----------------------------------------------------------------------------------------------*/

/* Function to fill the RX slots with SKBs */
static void liteeth_refill_rx_buffer(struct liteeth_device *liteeth_priv, uint32_t rx_slot)
{
	struct sk_buff *skb;

	/* Allocate an SKB with the specified slot size */
	skb = __netdev_alloc_skb_ip_align(liteeth_priv->netdev, liteeth_priv->slot_size, GFP_ATOMIC);
	if (!skb) {
		netdev_err(liteeth_priv->netdev, "Failed to allocate skb for RX\n");
		return;
	}

	/* Ensure the SKB data is 4-byte aligned */
	WARN_ON(!IS_ALIGNED((unsigned long)skb->data, 4));

	/* Store the SKB in the buffer and map for DMA */
	liteeth_priv->buffer[rx_slot].rx_skb = skb;
	liteeth_priv->buffer[rx_slot].rx_dma_addr = dma_map_single(
		&liteeth_priv->litepcie_dev->dev->dev,
		skb->data, liteeth_priv->slot_size,
		DMA_FROM_DEVICE
	);
	if (dma_mapping_error(&liteeth_priv->litepcie_dev->dev->dev, liteeth_priv->buffer[rx_slot].rx_dma_addr)) {
		netdev_err(liteeth_priv->netdev, "Failed to map RX DMA address\n");
		dev_kfree_skb_any(skb);
		return;
	}

	/* Write the DMA address to the corresponding register */
	litepcie_writel(liteeth_priv->litepcie_dev,
		liteeth_priv->csrs->sram_writer_pcie_host_addrs_addr + (rx_slot << 2),
		liteeth_priv->buffer[rx_slot].rx_dma_addr);
}

/* Function to clear any pending TX DMA on a LiteEth device */
static void liteeth_clear_tx_dma(struct liteeth_device *liteeth_priv)
{
	int i;
	struct litepcie_device *litepcie_dev = liteeth_priv->litepcie_dev;
	uint32_t pending_tx;

	/* Read the pending TX slots */
	pending_tx = litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_reader_pending_slots_addr);

	/* Iterate through all TX slots and clear pending transactions */
	for (i = 0; i < liteeth_priv->num_tx_slots; i++) {
		if ((pending_tx & (1 << i)) && liteeth_priv->buffer[i].tx_len) {
			/* Unmap the DMA address and free the SKB */
			dma_unmap_single(&litepcie_dev->dev->dev,
							 liteeth_priv->buffer[i].tx_dma_addr,
							 liteeth_priv->buffer[i].tx_len,
							 DMA_TO_DEVICE);
			dev_kfree_skb_any(liteeth_priv->buffer[i].tx_skb);
		}
	}

	/* Clear the pending TX slots */
	litepcie_writel(litepcie_dev, liteeth_priv->csrs->sram_reader_pending_clear_addr, pending_tx);
}

/* Function to open the LiteEth network device */
static int liteeth_open(struct net_device *netdev)
{
	int i;
	struct liteeth_device *liteeth_priv = netdev_priv(netdev);
	netdev_info(netdev, "liteeth_open\n");

	/* Fill the RX slots */
	for (i = 0; i < liteeth_priv->num_rx_slots; i++)
		liteeth_refill_rx_buffer(liteeth_priv, i);

	/* Enable the SRAM writer */
	litepcie_writel(liteeth_priv->litepcie_dev, liteeth_priv->csrs->sram_writer_enable_addr, 1);

	/* Enable the interrupts for TX and RX */
	litepcie_enable_interrupt(liteeth_priv->litepcie_dev, liteeth_priv->tx_irq_num);
	litepcie_enable_interrupt(liteeth_priv->litepcie_dev, liteeth_priv->rx_irq_num);

	/* Enable NAPI */
	napi_enable(&liteeth_priv->napi);

	/* Indicate that the carrier is on and start the queue */
	netif_carrier_on(netdev);
	netif_start_queue(netdev);

	return 0;
}

/* Function to stop the LiteEth network device */
static int liteeth_stop(struct net_device *netdev)
{
	int i;
	struct liteeth_device *liteeth_priv = netdev_priv(netdev);

	netdev_info(netdev, "liteeth_stop\n");

	/* Stop the queue and indicate that the carrier is off */
	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	/* Disable NAPI */
	napi_disable(&liteeth_priv->napi);

	/* Disable the SRAM writer */
	litepcie_writel(liteeth_priv->litepcie_dev, liteeth_priv->csrs->sram_writer_enable_addr, 0);

	/* Disable the interrupts for TX and RX */
	litepcie_disable_interrupt(liteeth_priv->litepcie_dev, liteeth_priv->tx_irq_num);
	litepcie_disable_interrupt(liteeth_priv->litepcie_dev, liteeth_priv->rx_irq_num);

	/* Unmap and free the RX slots */
	for (i = 0; i < liteeth_priv->num_rx_slots; i++) {
		dma_unmap_single(
			&liteeth_priv->litepcie_dev->dev->dev,
			liteeth_priv->buffer[i].rx_dma_addr,
			liteeth_priv->slot_size,
			DMA_FROM_DEVICE
		);
		dev_kfree_skb_any(liteeth_priv->buffer[i].rx_skb);
	}

	/* Clear any pending TX DMA */
	liteeth_clear_tx_dma(liteeth_priv);

	return 0;
}

/* Function to start transmitting a packet on the LiteEth network device */
static int liteeth_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct liteeth_device  *liteeth_priv = netdev_priv(netdev);
	struct litepcie_device *litepcie_dev = liteeth_priv->litepcie_dev;
	bool copy = false;

	/* Reject oversize packets */
	if (unlikely(skb->len > liteeth_priv->slot_size)) {
		dev_kfree_skb_any(skb);
		netdev->stats.tx_dropped++;
		netdev->stats.tx_errors++;
		return NETDEV_TX_OK;
	}

	/* Check if SRAM reader is ready */
	if (!litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_reader_ready_addr))
		goto busy;

	/* Clear pending TX DMA if necessary */
	if (litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_reader_pending_slots_addr) & (1 << liteeth_priv->tx_slot))
		liteeth_clear_tx_dma(liteeth_priv);

	/* Check if the packet data is 4-byte aligned */
	if (IS_ALIGNED((unsigned long)skb->data, 4)) {
		/* Map the SKB data for DMA */
		liteeth_priv->buffer[liteeth_priv->tx_slot].tx_dma_addr = dma_map_single(
			&liteeth_priv->litepcie_dev->dev->dev,
			skb->data, skb->len,
			DMA_TO_DEVICE
		);
		if (dma_mapping_error(&liteeth_priv->litepcie_dev->dev->dev, liteeth_priv->buffer[liteeth_priv->tx_slot].tx_dma_addr)) {
			netdev_err(netdev, "Failed to map TX DMA address\n");
			dev_kfree_skb_any(skb);
			return NETDEV_TX_OK;
		}
		liteeth_priv->buffer[liteeth_priv->tx_slot].tx_len = skb->len;
		liteeth_priv->buffer[liteeth_priv->tx_slot].tx_skb = skb;
	} else {
		/* Copy the SKB data to the transmission buffer */
		memcpy_toio(liteeth_priv->tx_buf + (liteeth_priv->tx_slot * liteeth_priv->slot_size), skb->data, skb->len);
		liteeth_priv->buffer[liteeth_priv->tx_slot].tx_dma_addr = liteeth_priv->tx_buf_dma + (liteeth_priv->tx_slot * liteeth_priv->slot_size);
		liteeth_priv->buffer[liteeth_priv->tx_slot].tx_len = 0;
		liteeth_priv->buffer[liteeth_priv->tx_slot].tx_skb = NULL;
		copy = true;
	}

	/* Write the necessary registers to start the transmission */
	litepcie_writel(litepcie_dev, liteeth_priv->csrs->sram_reader_slot_addr, liteeth_priv->tx_slot);
	litepcie_writel(litepcie_dev, liteeth_priv->csrs->sram_reader_length_addr, skb->len);
	litepcie_writel(litepcie_dev, liteeth_priv->csrs->sram_reader_pcie_host_addrs_addr + (liteeth_priv->tx_slot << 2), liteeth_priv->buffer[liteeth_priv->tx_slot].tx_dma_addr);
	litepcie_writel(litepcie_dev, liteeth_priv->csrs->sram_reader_start_addr, 1);

	/* Update the TX slot */
	liteeth_priv->tx_slot = (liteeth_priv->tx_slot + 1) % liteeth_priv->num_tx_slots;

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
	struct liteeth_device  *liteeth_priv = netdev_priv(dev);
	struct litepcie_device *litepcie_dev = liteeth_priv->litepcie_dev;
	struct netdev_queue *queue = netdev_get_tx_queue(dev, txqueue);
	uint32_t reg, slots;

	/* Read the number of slots and the ready register */
	slots = litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_reader_level_addr);
	reg   = litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_reader_ready_addr);
	netdev_info(dev, "litepcie: liteeth_tx_timeout, reg %u, slots %u\n", reg, slots);

	/* If the device is ready, wake the queue */
	if (reg)
		netif_tx_wake_queue(queue);
}

/* Net device operations for LiteEth */
static const struct net_device_ops liteeth_netdev_ops = {
	.ndo_open       = liteeth_open,       /* Open operation */
	.ndo_stop       = liteeth_stop,       /* Stop operation */
	.ndo_start_xmit = liteeth_start_xmit, /* Start transmit operation */
	.ndo_tx_timeout = liteeth_tx_timeout, /* TX timeout operation */
};

/* Function to handle RX interrupt */
static void liteeth_rx_interrupt(struct net_device *netdev, uint32_t rx_slot, uint32_t len)
{
	struct liteeth_device *liteeth_priv = netdev_priv(netdev);
	struct sk_buff *skb;
	unsigned char *data;

	/* Unmap the DMA address */
	dma_unmap_single(
		&liteeth_priv->litepcie_dev->dev->dev,
		liteeth_priv->buffer[rx_slot].rx_dma_addr,
		liteeth_priv->slot_size, DMA_FROM_DEVICE
	);

	/* Get the SKB for the specified RX slot */
	skb = liteeth_priv->buffer[rx_slot].rx_skb;

	/* Append data to the SKB and set its length */
	data = skb_put(skb, len);

	/* Set the protocol for the SKB */
	skb->protocol = eth_type_trans(skb, netdev);

	/* Pass the SKB to the network stack */
	napi_gro_receive(&liteeth_priv->napi, skb);

	/* Refill the RX slot */
	liteeth_refill_rx_buffer(liteeth_priv, rx_slot);

	/* Update statistics */
	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += len;

	return;
}

/* NAPI poll function */
static int liteeth_napi_poll(struct napi_struct *napi, int budget)
{
	struct liteeth_device  *liteeth_priv = container_of(napi, struct liteeth_device, napi);
	struct litepcie_device *litepcie_dev = liteeth_priv->litepcie_dev;
	uint32_t rx_pending, length, clear_mask;
	int work_done, i;

	clear_mask = 0;
	work_done  = 0;
	rx_pending = litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_writer_pending_slots_addr);

	/* Process pending RX slots */
	for (i = 0; i < liteeth_priv->num_rx_slots; i++) {
		if (rx_pending & (1 << i)) {
			/* Read the length of the pending slot */
			length = litepcie_readl(litepcie_dev, liteeth_priv->csrs->sram_writer_pending_length_addr + (i << 2));

			/* Handle the RX interrupt for the slot */
			liteeth_rx_interrupt(liteeth_priv->netdev, i, length);

			/* Update the clear mask and work done count */
			clear_mask |= 1 << i;
			work_done += 1;

			/* If the budget is reached, break the loop */
			if (work_done >= budget)
				break;
		}
	}

	/* Clear the pending RX slots */
	litepcie_writel(litepcie_dev, liteeth_priv->csrs->sram_writer_pending_clear_addr, clear_mask);

	/* If the work done is less than the budget, complete NAPI */
	if (work_done < budget) {
		napi_complete_done(napi, work_done);
	}

	return work_done;
}

/* Function to initialize the LiteEth device */
static int liteeth_init(struct litepcie_device *litepcie_dev, int index)
{
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct liteeth_device *liteeth_priv;
	int err;
	uint8_t mac_addr[ETH_ALEN];

	pdev = litepcie_dev->dev;

	/* Allocate and initialize the network device */
	netdev = devm_alloc_etherdev(&pdev->dev, sizeof(*liteeth_priv) + sizeof(struct skb_buffer_priv) * liteeth_params[index].rx_slots);
	if (!netdev)
		return -ENOMEM;

	/* Set the device structure in the network device */
	SET_NETDEV_DEV(netdev, &pdev->dev);

	/* Get the private data associated with the network device */
	liteeth_priv = netdev_priv(netdev);
	liteeth_priv->netdev = netdev;

	/* Assign index and CSR addresses */
	liteeth_priv->index = index;
	liteeth_priv->csrs = liteeth_params[index].csrs;

	/* Link LiteEth and LitePCIe device structures */
	litepcie_dev->liteeth_devs[index] = liteeth_priv;
	liteeth_priv->litepcie_dev = litepcie_dev;

	/* Get the IRQ vector for the device */
	netdev->irq = pci_irq_vector(litepcie_dev->dev, 0);

	/* Setup the RX and TX slots for the LiteEth device */
	liteeth_priv->num_rx_slots = liteeth_params[index].rx_slots;
	liteeth_priv->num_tx_slots = liteeth_params[index].tx_slots;
	liteeth_priv->slot_size    = liteeth_params[index].slot_size;

	/* Initialize the transmission slot index */
	liteeth_priv->tx_slot = 0;

	/* Allocate coherent memory for the transmission buffer */
	liteeth_priv->tx_buf = dma_alloc_coherent(&pdev->dev,
											  liteeth_priv->num_tx_slots * liteeth_priv->slot_size,
											  &liteeth_priv->tx_buf_dma, GFP_ATOMIC);
	if (!liteeth_priv->tx_buf) {
		dev_err(&pdev->dev, "Failed to allocate TX buffer\n");
		return -ENOMEM;
	}

	/* Set the hardware (MAC) address for the network device */
	memcpy(mac_addr, liteeth_mac_addr_base, ETH_ALEN);
	mac_addr[ETH_ALEN - 1] += index;  /* Ensure unique MAC address */
	eth_hw_addr_set(netdev, mac_addr);

	/* Assign interrupt numbers */
	liteeth_priv->tx_irq_num = liteeth_params[index].tx_interrupt;
	liteeth_priv->rx_irq_num = liteeth_params[index].rx_interrupt;

	/* Set the network device operations and watchdog timeout */
	netdev->netdev_ops     = &liteeth_netdev_ops;
	netdev->watchdog_timeo = 60 * HZ;

	/* Add NAPI to the network device for RX polling */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	netif_napi_add(netdev, &liteeth_priv->napi, liteeth_napi_poll, 64);
#else
	netif_napi_add(netdev, &liteeth_priv->napi, liteeth_napi_poll);
#endif

	/* Set the network interface name */
	snprintf(netdev->name, IFNAMSIZ, "liteeth%d", index);

	/* Register the network device with the kernel */
	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev %d\n", err);
		dma_free_coherent(&pdev->dev,
						  liteeth_priv->num_tx_slots * liteeth_priv->slot_size,
						  liteeth_priv->tx_buf,
						  liteeth_priv->tx_buf_dma);
		return err;
	}

	spin_lock_init(&liteeth_priv->lock);

	/* Log information about the network device */
	netdev_info(netdev, "Ethernet MAC%d: irq %d, slots: tx %d rx %d, size %d\n",
				index, netdev->irq, liteeth_priv->num_tx_slots, liteeth_priv->num_rx_slots, liteeth_priv->slot_size);

	return 0;
}

/* -----------------------------------------------------------------------------------------------*/
/*                                       PTP/PTM                                                  */
/* -----------------------------------------------------------------------------------------------*/
#ifdef CSR_PTM_REQUESTER_BASE

/* Time Control Register Addresses */
/* Write Time Low and High Addresses */
#define TIME_CONTROL_WRITE_TIME_L (CSR_TIME_GENERATOR_WRITE_TIME_ADDR + (4))
#define TIME_CONTROL_WRITE_TIME_H (CSR_TIME_GENERATOR_WRITE_TIME_ADDR + (0))

/* Read Time Low and High Addresses */
#define TIME_CONTROL_READ_TIME_L  (CSR_TIME_GENERATOR_READ_TIME_ADDR + (4))
#define TIME_CONTROL_READ_TIME_H  (CSR_TIME_GENERATOR_READ_TIME_ADDR + (0))

/* Time Control Register Flags */
#define TIME_CONTROL_ENABLE       (1 << CSR_TIME_GENERATOR_CONTROL_ENABLE_OFFSET)
#define TIME_CONTROL_READ         (1 << CSR_TIME_GENERATOR_CONTROL_READ_OFFSET)
#define TIME_CONTROL_WRITE        (1 << CSR_TIME_GENERATOR_CONTROL_WRITE_OFFSET)
#define TIME_CONTROL_SYNC_ENABLE  (1 << CSR_TIME_GENERATOR_CONTROL_SYNC_ENABLE_OFFSET)

/* PTM Offset in Nanoseconds (Adjust based on calibration) */
#define PTM_OFFSET_NS (-500) /* FIXME: Adjust based on calibration */

/* PTM Control Register Flags */
#define PTM_CONTROL_ENABLE  (1 << CSR_PTM_REQUESTER_CONTROL_ENABLE_OFFSET)
#define PTM_CONTROL_TRIGGER (1 << CSR_PTM_REQUESTER_CONTROL_TRIGGER_OFFSET)

/* PTM Status Register Flags */
#define PTM_STATUS_VALID    (1 << CSR_PTM_REQUESTER_STATUS_VALID_OFFSET)
#define PTM_STATUS_BUSY     (1 << CSR_PTM_REQUESTER_STATUS_BUSY_OFFSET)

/* PTM Time Registers */
/* T1 Time Low and High (Local Request Timestamp at Requester) */
#define PTM_T1_TIME_L       (CSR_PTM_REQUESTER_T1_TIME_ADDR + (4))
#define PTM_T1_TIME_H       (CSR_PTM_REQUESTER_T1_TIME_ADDR + (0))

/* T2 Time Low and High (Master Response Timestamp at Responder) */
#define PTM_MASTER_TIME_L   (CSR_PTM_REQUESTER_MASTER_TIME_ADDR + (4))
#define PTM_MASTER_TIME_H   (CSR_PTM_REQUESTER_MASTER_TIME_ADDR + (0))

/* T4 Time Low and High (Local Response Receipt Timestamp at Requester) */
#define PTM_T4_TIME_L       (CSR_PTM_REQUESTER_T4_TIME_ADDR + (4))
#define PTM_T4_TIME_H       (CSR_PTM_REQUESTER_T4_TIME_ADDR + (0))

/* Function to read a 64-bit value from two 32-bit registers */
static u64 litepcie_read64(struct litepcie_device *dev, uint32_t addr)
{
	/* Read the high and low 32-bit parts and combine them */
	return (((u64) litepcie_readl(dev, addr) << 32) |
		(litepcie_readl(dev, addr + 4) & 0xffffffff));
}

/* Function to read the current time from the device's time generator */
static int litepcie_read_time(struct litepcie_device *dev, struct timespec64 *ts)
{
	struct timespec64 rd_ts;
	s64 value;

	/* Issue a read command to the time generator */
	litepcie_writel(dev, CSR_TIME_GENERATOR_CONTROL_ADDR,
			(TIME_CONTROL_ENABLE | TIME_CONTROL_READ | TIME_CONTROL_SYNC_ENABLE));

	/* Read the high and low parts of the time value */
	value = (((s64) litepcie_readl(dev, TIME_CONTROL_READ_TIME_H) << 32) |
		(litepcie_readl(dev, TIME_CONTROL_READ_TIME_L) & 0xffffffff));

	/* Adjust the value by subtracting PTM offset */
	value = value - PTM_OFFSET_NS;

	/* Convert the value to timespec64 format */
	rd_ts = ns_to_timespec64(value);
	ts->tv_nsec = rd_ts.tv_nsec;
	ts->tv_sec = rd_ts.tv_sec;

	return 0;
}

/* Function to write a new time to the device's time generator */
static int litepcie_write_time(struct litepcie_device *dev, const struct timespec64 *ts)
{
	s64 value = timespec64_to_ns(ts);

	/* Adjust the value by adding PTM offset */
	value = value + PTM_OFFSET_NS;

	/* Write the low and high parts of the time value */
	litepcie_writel(dev, TIME_CONTROL_WRITE_TIME_L, (value >>  0) & 0xffffffff);
	litepcie_writel(dev, TIME_CONTROL_WRITE_TIME_H, (value >> 32) & 0xffffffff);

	/* Issue a write command to the time generator */
	litepcie_writel(dev, CSR_TIME_GENERATOR_CONTROL_ADDR,
			(TIME_CONTROL_ENABLE | TIME_CONTROL_WRITE | TIME_CONTROL_SYNC_ENABLE));

	return 0;
}

/* PTP clock operation: Get the current time with timestamping */
static int litepcie_ptp_gettimex64(struct ptp_clock_info *ptp,
                   struct timespec64 *ts,
                   struct ptp_system_timestamp *sts)
{
	struct litepcie_device *dev = container_of(ptp, struct litepcie_device,
							   ptp_caps);
	unsigned long flags;

	/* Acquire lock to ensure consistent time reading */
	spin_lock_irqsave(&dev->tmreg_lock, flags);

	/* Capture system timestamps before and after reading device time */
	ptp_read_system_prets(sts);
	litepcie_read_time(dev, ts);
	ptp_read_system_postts(sts);

	/* Release lock */
	spin_unlock_irqrestore(&dev->tmreg_lock, flags);

	return 0;
}

/* PTP clock operation: Set the device time */
static int litepcie_ptp_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct litepcie_device *dev = container_of(ptp, struct litepcie_device,
							   ptp_caps);
	unsigned long flags;

	/* Acquire lock to prevent concurrent access */
	spin_lock_irqsave(&dev->tmreg_lock, flags);

	/* Write the new time to the device */
	litepcie_write_time(dev, ts);

	/* Release lock */
	spin_unlock_irqrestore(&dev->tmreg_lock, flags);

	return 0; // Return success
}

/* PTP clock operation: Adjust the frequency by scaled parts per million */
static int litepcie_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct litepcie_device *dev = container_of(ptp, struct litepcie_device,
							   ptp_caps);
    unsigned long flags;

    /* Acquire lock to prevent concurrent access */
    spin_lock_irqsave(&dev->tmreg_lock, flags);

    /* Convert scaled_ppm (Q16.16) → Signed ppb */
    int64_t ppb = (scaled_ppm * 1000LL) >> 16; /* *1000 / 65536 */

    /* Nominal step = 8 ns << 24 */
    uint32_t add_nom = 0x08000000U;

    /* Compute add_new = add_nom * (1 + ppb / 1e9) */
    uint64_t add_new = div64_u64((uint64_t)add_nom * (1000000000LL + ppb), 1000000000LL);

    /* Update Step with add_new */
    litepcie_writel(dev, CSR_TIME_GENERATOR_TIME_INC_ADDR, (uint32_t) add_new);

    /* Release lock */
    spin_unlock_irqrestore(&dev->tmreg_lock, flags);

    return 0;
}

/* PTP clock operation: Adjust the time by a given delta in nanoseconds */
static int litepcie_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct litepcie_device *dev = container_of(ptp, struct litepcie_device,
							   ptp_caps);
	struct timespec64 now, then = ns_to_timespec64(delta);
	unsigned long flags;

	/* Acquire lock to prevent concurrent access */
	spin_lock_irqsave(&dev->tmreg_lock, flags);

	/* Read current time, add delta, and write back */
	litepcie_read_time(dev, &now);
	now = timespec64_add(now, then);
	litepcie_write_time(dev, &now);

	/* Release lock */
	spin_unlock_irqrestore(&dev->tmreg_lock, flags);
	return 0; // Return success
}

/* Function to obtain a synchronized device and system timestamp */
static int litepcie_phc_get_syncdevicetime(ktime_t *device,
                          struct system_counterval_t *system,
                          void *ctx)
{
	u32 t1_curr_h, t1_curr_l;
	u32 t2_curr_h, t2_curr_l;
	u32 prop_delay;
	u32 reg;
	u64 ptm_master_time;
	struct litepcie_device *dev = ctx;
	u64 t1_curr;
	ktime_t t1, t2_curr;
	int count = 100;

	/* Get a snapshot of system clocks to use as historic value */
	ktime_get_snapshot(&dev->snapshot);

	/* Trigger a PTM request */
	litepcie_writel(dev, CSR_PTM_REQUESTER_CONTROL_ADDR,
		PTM_CONTROL_ENABLE | PTM_CONTROL_TRIGGER);

	/* Wait until PTM request is complete */
	do {
		reg = litepcie_readl(dev, CSR_PTM_REQUESTER_STATUS_ADDR);
		if ((reg & PTM_STATUS_BUSY) == 0)
			break;
	} while (--count);

	if (!count) {
		printk("Exceeded number of tries for PTM cycle\n");
		return -ETIMEDOUT;
	}

	/* Read T1 time (Local Request Timestamp at Requester) */
	t1_curr_l = litepcie_readl(dev, PTM_T1_TIME_L);
	t1_curr_h = litepcie_readl(dev, PTM_T1_TIME_H);
	t1_curr = ((u64)t1_curr_h << 32 | t1_curr_l);
	t1 = ns_to_ktime(t1_curr);

	/* Read T2 time (Master Response Timestamp at Responder) */
	t2_curr_l = litepcie_readl(dev, PTM_MASTER_TIME_L);
	t2_curr_h = litepcie_readl(dev, PTM_MASTER_TIME_H);
	t2_curr = ((u64)t2_curr_h << 32 | t2_curr_l);

	/* Read propagation delay (t3 - t2 from downstream port) */
	prop_delay = litepcie_readl(dev, CSR_PTM_REQUESTER_LINK_DELAY_ADDR);

	/* Compute PTM Master Time */
	ptm_master_time = t2_curr - (((dev->t4_prev - dev->t1_prev) - prop_delay) >> 1);

	/* Set device time */
	*device = t1;

#if IS_ENABLED(CONFIG_X86_TSC) && !defined(CONFIG_UML)
	/* Convert ART (Absolute Reference Time) to TSC (Time Stamp Counter) */
	*system = convert_art_ns_to_tsc(ptm_master_time);
#else
    *system = (struct system_counterval_t) { };
#endif

	/* Store T4 and T1 for next request */
	dev->t4_prev = litepcie_read64(dev, CSR_PTM_REQUESTER_T4_TIME_ADDR);
	dev->t1_prev = t1_curr;

	return 0;
}

/* PTP clock operation: Get cross timestamp between system and device clocks */
static int litepcie_ptp_getcrosststamp(struct ptp_clock_info *ptp,
                      struct system_device_crosststamp *cts)
{
	struct litepcie_device *dev = container_of(ptp, struct litepcie_device,
                               ptp_caps);

	/* Obtain the cross timestamp using the provided helper */
	return get_device_system_crosststamp(litepcie_phc_get_syncdevicetime,
                         dev, &dev->snapshot, cts);
}

/* PTP clock operation: Enable or disable features (not supported) */
static int litepcie_ptp_enable(struct ptp_clock_info __always_unused *ptp,
                 struct ptp_clock_request __always_unused *request,
                 int __always_unused on)
{
    return -EOPNOTSUPP;
}

/* PTP clock capabilities and function pointers */
static struct ptp_clock_info litepcie_ptp_info = {
	.owner          = THIS_MODULE,
	.name           = LITEPCIE_NAME,
	.max_adj        = 1000000000,
	.n_alarm        = 0,
	.n_ext_ts       = 0,
	.n_per_out      = 0,
	.n_pins         = 0,
	.pps            = 0,
	.gettimex64     = litepcie_ptp_gettimex64,
	.settime64      = litepcie_ptp_settime,
	.adjtime        = litepcie_ptp_adjtime,
	.adjfine        = litepcie_ptp_adjfine,
	.getcrosststamp = litepcie_ptp_getcrosststamp,
	.enable         = litepcie_ptp_enable,
};
#endif

/* -----------------------------------------------------------------------------------------------*/
/*                            LitePCIe Probe / Remove / Module                                    */
/* -----------------------------------------------------------------------------------------------*/

/* Function to probe the LitePCIe PCI device */
static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;
	int irqs = 0;
	uint8_t rev_id;
	int i;
	char fpga_identifier[256];
	struct litepcie_device *litepcie_dev = NULL;
#ifdef CSR_UART_XOVER_RXTX_ADDR
	struct resource *tty_res = NULL;
#endif
#ifdef CSR_PTM_REQUESTER_BASE
	int count = 100;
#endif

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

	/* Reset LitePCIe core */
#ifdef CSR_CTRL_RESET_ADDR
	litepcie_writel(litepcie_dev, CSR_CTRL_RESET_ADDR, 1);
	msleep(10);
#endif

	/* Read and display the FPGA identifier */
	for (i = 0; i < 256; i++)
		fpga_identifier[i] = litepcie_readl(litepcie_dev, CSR_IDENTIFIER_MEM_BASE + i * 4);
	dev_info(&dev->dev, "Version %s\n", fpga_identifier);


	/* Enable PTM */
	ret = pci_enable_ptm(dev, NULL);
	if (ret < 0)
		dev_info(&dev->dev, "PCIe PTM not supported by PCIe bus/controller\n");
	else
		dev_info(&dev->dev, "PCIe PTM supported by PCIe bus/controller\n");

	pci_set_master(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
	ret = pci_set_dma_mask(dev, DMA_BIT_MASK(DMA_ADDR_WIDTH));
#else
	ret = dma_set_mask(&dev->dev, DMA_BIT_MASK(DMA_ADDR_WIDTH));
#endif
	if (ret) {
		dev_err(&dev->dev, "Failed to set DMA mask\n");
		goto fail1;
	}

	/* MSI-X */
#ifdef CSR_PCIE_MSI_PBA_ADDR
	irqs = pci_alloc_irq_vectors(dev, 1, 32, PCI_IRQ_MSIX);
#else
	/* MSI Single / MultiVector */
	irqs = pci_alloc_irq_vectors(dev, 1, 32, PCI_IRQ_MSI);
#endif
	if (irqs < 0) {
		dev_err(&dev->dev, "Failed to enable MSI\n");
		ret = irqs;
		goto fail1;
	}
#ifdef CSR_PCIE_MSI_PBA_ADDR
	dev_info(&dev->dev, "%d MSI-X IRQs allocated.\n", irqs);
#else
	dev_info(&dev->dev, "%d MSI IRQs allocated.\n", irqs);
#endif

	litepcie_dev->irqs = 0;
	for (i = 0; i < irqs; i++) {
		int irq = pci_irq_vector(dev, i);

		/* Request IRQ */
		ret = request_irq(irq, litepcie_interrupt, 0, LITEPCIE_NAME, litepcie_dev);
		if (ret < 0) {
			dev_err(&dev->dev, " Failed to allocate IRQ %d\n", irq);
			while (--i >= 0) {
				irq = pci_irq_vector(dev, i);
				free_irq(irq, litepcie_dev);
			}
			goto fail2;
		}
		litepcie_dev->irqs += 1;
	}

	/* Initialize all Ethernet interfaces */
	for (i = 0; i < NUM_ETHMACS; i++) {
		ret = liteeth_init(litepcie_dev, i);
		if (ret) {
			dev_err(&dev->dev, "Failed to initialize LiteEth device %d\n", i);
			goto fail_eth_init;
		}
	}

#ifdef CSR_UART_XOVER_RXTX_ADDR
	tty_res = devm_kzalloc(&dev->dev, sizeof(struct resource), GFP_KERNEL);
	if (!tty_res)
		return -ENOMEM;
	tty_res->start =
		(resource_size_t) litepcie_dev->bar0_addr +
		CSR_UART_XOVER_RXTX_ADDR - CSR_BASE;
	tty_res->flags = IORESOURCE_REG;
	litepcie_dev->uart = platform_device_register_simple("liteuart", litepcie_minor_idx, tty_res, 1);
	if (IS_ERR(litepcie_dev->uart)) {
		ret = PTR_ERR(litepcie_dev->uart);
		goto fail_eth_init;
	}
#endif

	/* PTP */
#ifdef CSR_PTM_REQUESTER_BASE
	litepcie_dev->ptp_caps = litepcie_ptp_info;
	litepcie_dev->litepcie_ptp_clock = ptp_clock_register(&litepcie_dev->ptp_caps, &dev->dev);
	if (IS_ERR(litepcie_dev->litepcie_ptp_clock)) {
		return PTR_ERR(litepcie_dev->litepcie_ptp_clock);
	}

	/* Enable timer (time) counter */
	litepcie_writel(litepcie_dev, CSR_TIME_GENERATOR_CONTROL_ADDR, TIME_CONTROL_ENABLE | TIME_CONTROL_SYNC_ENABLE);

	/* Enable PTM control and start first request */
	litepcie_writel(litepcie_dev, CSR_PTM_REQUESTER_CONTROL_ADDR, PTM_CONTROL_ENABLE | PTM_CONTROL_TRIGGER);
	/* Prepare T1 & T4 for next request */
	do {
		if ((litepcie_readl(litepcie_dev, CSR_PTM_REQUESTER_STATUS_ADDR) & PTM_STATUS_BUSY) == 0)
			break;
	} while (--count);

	litepcie_writel(litepcie_dev, CSR_PTM_REQUESTER_CONTROL_ADDR, PTM_CONTROL_ENABLE | PTM_CONTROL_TRIGGER);
	count = 100;
	do {
		if ((litepcie_readl(litepcie_dev, CSR_PTM_REQUESTER_STATUS_ADDR) & PTM_STATUS_BUSY) == 0)
			break;
	} while (--count);

	litepcie_dev->t4_prev = litepcie_read64(litepcie_dev, CSR_PTM_REQUESTER_T4_TIME_ADDR);

	litepcie_dev->t1_prev = (((u64)litepcie_readl(litepcie_dev, PTM_T1_TIME_L) << 32) |
		litepcie_readl(litepcie_dev, PTM_T1_TIME_H));

	spin_lock_init(&litepcie_dev->tmreg_lock);
#endif

	return 0;

fail_eth_init:
	/* Cleanup any initialized Ethernet interfaces */
	while (--i >= 0) {
		struct liteeth_device *liteeth_priv = litepcie_dev->liteeth_devs[i];
		if (liteeth_priv) {
			unregister_netdev(liteeth_priv->netdev);
			netif_napi_del(&liteeth_priv->napi);
			dma_free_coherent(&dev->dev,
							  liteeth_priv->num_tx_slots * liteeth_priv->slot_size,
							  liteeth_priv->tx_buf,
							  liteeth_priv->tx_buf_dma);
		}
	}
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

	litepcie_dev = pci_get_drvdata(dev);

	dev_info(&dev->dev, "\e[1m[Removing device]\e[0m\n");

	/* Disable all interrupts */
	litepcie_writel(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR, 0);

    /* Unregister PTP */
	if (litepcie_dev->litepcie_ptp_clock) {
		ptp_clock_unregister(litepcie_dev->litepcie_ptp_clock);
		litepcie_dev->litepcie_ptp_clock = NULL;
	}

	/* Unregister and free all Ethernet devices */
	for (i = 0; i < NUM_ETHMACS; i++) {
		struct liteeth_device *liteeth_priv = litepcie_dev->liteeth_devs[i];

		if (liteeth_priv) {
			unregister_netdev(liteeth_priv->netdev);
			netif_napi_del(&liteeth_priv->napi);
			dma_free_coherent(&dev->dev,
							  liteeth_priv->num_tx_slots * liteeth_priv->slot_size,
							  liteeth_priv->tx_buf,
							  liteeth_priv->tx_buf_dma);
		}
	}

	/* Free all IRQs */
	for (i = 0; i < litepcie_dev->irqs; i++) {
		irq = pci_irq_vector(dev, i);
		free_irq(irq, litepcie_dev);
	}

#ifdef CSR_UART_XOVER_RXTX_ADDR
	platform_device_unregister(litepcie_dev->uart);
#endif

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
	.name     = LITEPCIE_NAME,
	.id_table = litepcie_pci_ids,
	.probe    = litepcie_pci_probe,
	.remove   = litepcie_pci_remove,
};

/* Module initialization function */
static int __init litepcie_module_init(void)
{
	int ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
	litepcie_class = class_create(THIS_MODULE, LITEPCIE_NAME);
#else
	litepcie_class = class_create(LITEPCIE_NAME);
#endif
	if (IS_ERR(litepcie_class)) {
		ret = PTR_ERR(litepcie_class);
		pr_err("Failed to create class\n");
		goto fail_create_class;
	}

	ret = alloc_chrdev_region(&litepcie_dev_t, 0, LITEPCIE_MINOR_COUNT, LITEPCIE_NAME);
	if (ret < 0) {
		pr_err("Could not allocate char device\n");
		goto fail_alloc_chrdev_region;
	}
	litepcie_major = MAJOR(litepcie_dev_t);
	litepcie_minor_idx = MINOR(litepcie_dev_t);

	ret = pci_register_driver(&litepcie_pci_driver);
	if (ret < 0) {
		pr_err("Error while registering PCI driver\n");
		goto fail_register;
	}

	return 0;

fail_register:
	unregister_chrdev_region(litepcie_dev_t, LITEPCIE_MINOR_COUNT);
fail_alloc_chrdev_region:
	class_destroy(litepcie_class);
fail_create_class:
	return ret;
}

/* Module exit function */
static void __exit litepcie_module_exit(void)
{
	pci_unregister_driver(&litepcie_pci_driver);
	unregister_chrdev_region(litepcie_dev_t, LITEPCIE_MINOR_COUNT);
	class_destroy(litepcie_class);
}

module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
