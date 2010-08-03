/*
 * JMicron JMC2x0 series PCIe Ethernet Linux Device Driver
 *
 * Copyright 2008 JMicron Technology Corporation
 * http://www.jmicron.com/
 *
 * Author: Guo-Fu Tseng <cooldavid@cooldavid.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * TODO:
 * 	-  Implement MSI-X.
 * 	   Along with multiple RX queue, for CPU load balancing.
 *	-  Decode register dump for ethtool.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/if_vlan.h>
#include "jme.h"

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21)
static struct net_device_stats *
jme_get_stats(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	return &jme->stats;
}
#endif

static int
jme_mdio_read(struct net_device *netdev, int phy, int reg)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int i, val;

        jwrite32(jme, JME_SMI, SMI_OP_REQ |
				smi_phy_addr(phy) |
				smi_reg_addr(reg));

	wmb();
        for (i = JME_PHY_TIMEOUT ; i > 0 ; --i) {
		udelay(1);
		val = jread32(jme, JME_SMI);
		if ((val & SMI_OP_REQ) == 0)
			break;
        }

        if (i == 0) {
		jeprintk(netdev->name, "phy read timeout : %d\n", reg);
		return 0;
        }

	return ((val & SMI_DATA_MASK) >> SMI_DATA_SHIFT);
}

static void
jme_mdio_write(struct net_device *netdev,
				int phy, int reg, int val)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int i;

	jwrite32(jme, JME_SMI, SMI_OP_WRITE | SMI_OP_REQ |
		((val << SMI_DATA_SHIFT) & SMI_DATA_MASK) |
		smi_phy_addr(phy) | smi_reg_addr(reg));

	wmb();
	for (i = JME_PHY_TIMEOUT ; i > 0 ; --i) {
		udelay(1);
		val = jread32(jme, JME_SMI);
		if ((val & SMI_OP_REQ) == 0)
			break;
	}

	if (i == 0)
		jeprintk(netdev->name, "phy write timeout : %d\n", reg);

	return;
}

__always_inline static void
jme_reset_phy_processor(struct jme_adapter *jme)
{
	__u32 val;

	jme_mdio_write(jme->dev,
			jme->mii_if.phy_id,
			MII_ADVERTISE, ADVERTISE_ALL |
			ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

	jme_mdio_write(jme->dev,
			jme->mii_if.phy_id,
			MII_CTRL1000,
			ADVERTISE_1000FULL | ADVERTISE_1000HALF);

	val = jme_mdio_read(jme->dev,
				jme->mii_if.phy_id,
				MII_BMCR);

	jme_mdio_write(jme->dev,
			jme->mii_if.phy_id,
			MII_BMCR, val | BMCR_RESET);

	return;
}

static void
jme_setup_wakeup_frame(struct jme_adapter *jme,
		__u32 *mask, __u32 crc, int fnr)
{
	int i;

	/*
	 * Setup CRC pattern
	 */
	jwrite32(jme, JME_WFOI, WFOI_CRC_SEL | (fnr & WFOI_FRAME_SEL));
	wmb();
	jwrite32(jme, JME_WFODP, crc);
	wmb();

	/*
	 * Setup Mask
	 */
	for(i = 0 ; i < WAKEUP_FRAME_MASK_DWNR ; ++i) {
		jwrite32(jme, JME_WFOI,
				((i << WFOI_MASK_SHIFT) & WFOI_MASK_SEL) |
				(fnr & WFOI_FRAME_SEL));
		wmb();
		jwrite32(jme, JME_WFODP, mask[i]);
		wmb();
	}
}

__always_inline static void
jme_reset_mac_processor(struct jme_adapter *jme)
{
	__u32 mask[WAKEUP_FRAME_MASK_DWNR] = {0,0,0,0};
	__u32 crc = 0xCDCDCDCD;
	int i;

	jwrite32(jme, JME_GHC, jme->reg_ghc | GHC_SWRST);
	udelay(2);
	jwrite32(jme, JME_GHC, jme->reg_ghc);
	jwrite32(jme, JME_RXMCHT_LO, 0x00000000);
	jwrite32(jme, JME_RXMCHT_HI, 0x00000000);
	for(i = 0 ; i < WAKEUP_FRAME_NR ; ++i)
		jme_setup_wakeup_frame(jme, mask, crc, i);
	jwrite32(jme, JME_GPREG0, GPREG0_DEFAULT);
	jwrite32(jme, JME_GPREG1, 0);
}

__always_inline static void
jme_clear_pm(struct jme_adapter *jme)
{
	jwrite32(jme, JME_PMCS, 0xFFFF0000 | jme->reg_pmcs);
	pci_set_power_state(jme->pdev, PCI_D0);
	pci_enable_wake(jme->pdev, PCI_D0, false);
}

static int
jme_reload_eeprom(struct jme_adapter *jme)
{
	__u32 val;
	int i;

	val = jread32(jme, JME_SMBCSR);

	if(val & SMBCSR_EEPROMD)
	{
		val |= SMBCSR_CNACK;
		jwrite32(jme, JME_SMBCSR, val);
		val |= SMBCSR_RELOAD;
		jwrite32(jme, JME_SMBCSR, val);
		mdelay(12);

		for (i = JME_SMB_TIMEOUT; i > 0; --i)
		{
			mdelay(1);
			if ((jread32(jme, JME_SMBCSR) & SMBCSR_RELOAD) == 0)
				break;
		}

		if(i == 0) {
			jeprintk(jme->dev->name, "eeprom reload timeout\n");
			return -EIO;
		}
	}
	else
		return -EIO;

	return 0;
}

static void
jme_load_macaddr(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	unsigned char macaddr[6];
	__u32 val;

	spin_lock(&jme->macaddr_lock);
	val = jread32(jme, JME_RXUMA_LO);
	macaddr[0] = (val >>  0) & 0xFF;
	macaddr[1] = (val >>  8) & 0xFF;
	macaddr[2] = (val >> 16) & 0xFF;
	macaddr[3] = (val >> 24) & 0xFF;
	val = jread32(jme, JME_RXUMA_HI);
	macaddr[4] = (val >>  0) & 0xFF;
	macaddr[5] = (val >>  8) & 0xFF;
        memcpy(netdev->dev_addr, macaddr, 6);
	spin_unlock(&jme->macaddr_lock);
}

__always_inline static void
jme_set_rx_pcc(struct jme_adapter *jme, int p)
{
	switch(p) {
	case PCC_OFF:
		jwrite32(jme, JME_PCCRX0,
			((PCC_OFF_TO << PCCRXTO_SHIFT) & PCCRXTO_MASK) |
			((PCC_OFF_CNT << PCCRX_SHIFT) & PCCRX_MASK));
		break;
	case PCC_P1:
		jwrite32(jme, JME_PCCRX0,
			((PCC_P1_TO << PCCRXTO_SHIFT) & PCCRXTO_MASK) |
			((PCC_P1_CNT << PCCRX_SHIFT) & PCCRX_MASK));
		break;
	case PCC_P2:
		jwrite32(jme, JME_PCCRX0,
			((PCC_P2_TO << PCCRXTO_SHIFT) & PCCRXTO_MASK) |
			((PCC_P2_CNT << PCCRX_SHIFT) & PCCRX_MASK));
		break;
	case PCC_P3:
		jwrite32(jme, JME_PCCRX0,
			((PCC_P3_TO << PCCRXTO_SHIFT) & PCCRXTO_MASK) |
			((PCC_P3_CNT << PCCRX_SHIFT) & PCCRX_MASK));
		break;
	default:
		break;
	}
	wmb();

	if(!(jme->flags & JME_FLAG_POLL))
		dprintk(jme->dev->name, "Switched to PCC_P%d\n", p);
}

static void
jme_start_irq(struct jme_adapter *jme)
{
	register struct dynpcc_info *dpi = &(jme->dpi);

	jme_set_rx_pcc(jme, PCC_P1);
	dpi->cur		= PCC_P1;
	dpi->attempt		= PCC_P1;
	dpi->cnt		= 0;

	jwrite32(jme, JME_PCCTX,
			((PCC_TX_TO << PCCTXTO_SHIFT) & PCCTXTO_MASK) |
			((PCC_TX_CNT << PCCTX_SHIFT) & PCCTX_MASK) |
			PCCTXQ0_EN
		);

	/*
	 * Enable Interrupts
	 */
	jwrite32(jme, JME_IENS, INTR_ENABLE);
}

__always_inline static void
jme_stop_irq(struct jme_adapter *jme)
{
	/*
	 * Disable Interrupts
	 */
	jwrite32(jme, JME_IENC, INTR_ENABLE);
}


__always_inline static void
jme_enable_shadow(struct jme_adapter *jme)
{
	jwrite32(jme,
		 JME_SHBA_LO,
		 ((__u32)jme->shadow_dma & ~((__u32)0x1F)) | SHBA_POSTEN);
}

__always_inline static void
jme_disable_shadow(struct jme_adapter *jme)
{
	jwrite32(jme, JME_SHBA_LO, 0x0);
}

static int
jme_check_link(struct net_device *netdev, int testonly)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	__u32 phylink, ghc, cnt = JME_SPDRSV_TIMEOUT, bmcr;
	char linkmsg[64];
	int rc = 0;

	linkmsg[0] = '\0';
	phylink = jread32(jme, JME_PHY_LINK);

        if (phylink & PHY_LINK_UP) {
		if(!(phylink & PHY_LINK_AUTONEG_COMPLETE)) {
			/*
			 * If we did not enable AN
			 * Speed/Duplex Info should be obtained from SMI
			 */
			phylink = PHY_LINK_UP;

			bmcr = jme_mdio_read(jme->dev,
						jme->mii_if.phy_id,
						MII_BMCR);


			phylink |= ((bmcr & BMCR_SPEED1000) &&
					(bmcr & BMCR_SPEED100) == 0) ?
					PHY_LINK_SPEED_1000M :
					(bmcr & BMCR_SPEED100) ?
					PHY_LINK_SPEED_100M :
					PHY_LINK_SPEED_10M;

			phylink |= (bmcr & BMCR_FULLDPLX) ?
					 PHY_LINK_DUPLEX : 0;

			strcat(linkmsg, "Forced: ");
		}
		else {
			/*
			 * Keep polling for speed/duplex resolve complete
			 */
			while(!(phylink & PHY_LINK_SPEEDDPU_RESOLVED) &&
				--cnt) {

				udelay(1);
				phylink = jread32(jme, JME_PHY_LINK);

			}

			if(!cnt)
				jeprintk(netdev->name,
					"Waiting speed resolve timeout.\n");

			strcat(linkmsg, "ANed: ");
		}

		if(jme->phylink == phylink) {
			rc = 1;
			goto out;
		}
		if(testonly)
			goto out;

		jme->phylink = phylink;

		switch(phylink & PHY_LINK_SPEED_MASK) {
			case PHY_LINK_SPEED_10M:
				ghc = GHC_SPEED_10M;
				strcat(linkmsg, "10 Mbps, ");
				break;
			case PHY_LINK_SPEED_100M:
				ghc = GHC_SPEED_100M;
				strcat(linkmsg, "100 Mbps, ");
				break;
			case PHY_LINK_SPEED_1000M:
				ghc = GHC_SPEED_1000M;
				strcat(linkmsg, "1000 Mbps, ");
				break;
			default:
				ghc = 0;
				break;
		}
                ghc |= (phylink & PHY_LINK_DUPLEX) ? GHC_DPX : 0;

		strcat(linkmsg, (phylink &PHY_LINK_DUPLEX) ?
					"Full-Duplex, " :
					"Half-Duplex, ");

		if(phylink & PHY_LINK_MDI_STAT)
			strcat(linkmsg, "MDI-X");
		else
			strcat(linkmsg, "MDI");

		if(phylink & PHY_LINK_DUPLEX)
			jwrite32(jme, JME_TXMCS, TXMCS_DEFAULT);
		else {
			jwrite32(jme, JME_TXMCS, TXMCS_DEFAULT |
						TXMCS_BACKOFF |
						TXMCS_CARRIERSENSE |
						TXMCS_COLLISION);
			jwrite32(jme, JME_TXTRHD, TXTRHD_TXPEN |
				((0x2000 << TXTRHD_TXP_SHIFT) & TXTRHD_TXP) |
				TXTRHD_TXREN |
				((8 << TXTRHD_TXRL_SHIFT) & TXTRHD_TXRL));
		}

		jme->reg_ghc = ghc;
		jwrite32(jme, JME_GHC, ghc);

		jprintk(netdev->name, "Link is up at %s.\n", linkmsg);
                netif_carrier_on(netdev);
	}
        else {
		if(testonly)
			goto out;

		jprintk(netdev->name, "Link is down.\n");
		jme->phylink = 0;
                netif_carrier_off(netdev);
	}

out:
	return rc;
}

static int
jme_setup_tx_resources(struct jme_adapter *jme)
{
	struct jme_ring *txring = &(jme->txring[0]);

	txring->alloc = dma_alloc_coherent(&(jme->pdev->dev),
				   TX_RING_ALLOC_SIZE(jme->tx_ring_size),
				   &(txring->dmaalloc),
				   GFP_ATOMIC);

	if(!txring->alloc) {
		txring->desc = NULL;
		txring->dmaalloc = 0;
		txring->dma = 0;
		return -ENOMEM;
	}

	/*
	 * 16 Bytes align
	 */
	txring->desc		= (void*)ALIGN((unsigned long)(txring->alloc),
						RING_DESC_ALIGN);
	txring->dma		= ALIGN(txring->dmaalloc, RING_DESC_ALIGN);
	txring->next_to_use	= 0;
	txring->next_to_clean	= 0;
	atomic_set(&txring->nr_free, jme->tx_ring_size);

	/*
	 * Initialize Transmit Descriptors
	 */
	memset(txring->alloc, 0, TX_RING_ALLOC_SIZE(jme->tx_ring_size));
	memset(txring->bufinf, 0,
		sizeof(struct jme_buffer_info) * jme->tx_ring_size);

	return 0;
}

static void
jme_free_tx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *txring = &(jme->txring[0]);
	struct jme_buffer_info *txbi = txring->bufinf;

	if(txring->alloc) {
		for(i = 0 ; i < jme->tx_ring_size ; ++i) {
			txbi = txring->bufinf + i;
			if(txbi->skb) {
				dev_kfree_skb(txbi->skb);
				txbi->skb = NULL;
			}
			txbi->mapping	= 0;
			txbi->len	= 0;
			txbi->nr_desc	= 0;
		}

		dma_free_coherent(&(jme->pdev->dev),
				  TX_RING_ALLOC_SIZE(jme->tx_ring_size),
				  txring->alloc,
				  txring->dmaalloc);

		txring->alloc		= NULL;
		txring->desc		= NULL;
		txring->dmaalloc	= 0;
		txring->dma		= 0;
	}
	txring->next_to_use	= 0;
	txring->next_to_clean	= 0;
	atomic_set(&txring->nr_free, 0);

}

__always_inline static void
jme_enable_tx_engine(struct jme_adapter *jme)
{
	/*
	 * Select Queue 0
	 */
	jwrite32(jme, JME_TXCS, TXCS_DEFAULT | TXCS_SELECT_QUEUE0);

	/*
	 * Setup TX Queue 0 DMA Bass Address
	 */
	jwrite32(jme, JME_TXDBA_LO, (__u64)jme->txring[0].dma & 0xFFFFFFFFUL);
	jwrite32(jme, JME_TXDBA_HI, (__u64)(jme->txring[0].dma) >> 32);
	jwrite32(jme, JME_TXNDA, (__u64)jme->txring[0].dma & 0xFFFFFFFFUL);

	/*
	 * Setup TX Descptor Count
	 */
	jwrite32(jme, JME_TXQDC, jme->tx_ring_size);

	/*
	 * Enable TX Engine
	 */
	wmb();
	jwrite32(jme, JME_TXCS, jme->reg_txcs |
				TXCS_SELECT_QUEUE0 |
				TXCS_ENABLE);

}

__always_inline static void
jme_restart_tx_engine(struct jme_adapter *jme)
{
	/*
	 * Restart TX Engine
	 */
	jwrite32(jme, JME_TXCS, jme->reg_txcs |
				TXCS_SELECT_QUEUE0 |
				TXCS_ENABLE);
}

__always_inline static void
jme_disable_tx_engine(struct jme_adapter *jme)
{
	int i;
	__u32 val;

	/*
	 * Disable TX Engine
	 */
	jwrite32(jme, JME_TXCS, jme->reg_txcs | TXCS_SELECT_QUEUE0);

	val = jread32(jme, JME_TXCS);
	for(i = JME_TX_DISABLE_TIMEOUT ; (val & TXCS_ENABLE) && i > 0 ; --i)
	{
		mdelay(1);
		val = jread32(jme, JME_TXCS);
	}

	if(!i) {
		jeprintk(jme->dev->name, "Disable TX engine timeout.\n");
		jme_reset_mac_processor(jme);
	}


}

static void
jme_set_clean_rxdesc(struct jme_adapter *jme, int i)
{
	struct jme_ring *rxring = jme->rxring;
	register volatile struct rxdesc* rxdesc = rxring->desc;
	struct jme_buffer_info *rxbi = rxring->bufinf;
	rxdesc += i;
	rxbi += i;

	rxdesc->dw[0] = 0;
	rxdesc->dw[1] = 0;
	rxdesc->desc1.bufaddrh	= cpu_to_le32((__u64)rxbi->mapping >> 32);
	rxdesc->desc1.bufaddrl	= cpu_to_le32(
					(__u64)rxbi->mapping & 0xFFFFFFFFUL);
	rxdesc->desc1.datalen	= cpu_to_le16(rxbi->len);
	if(jme->dev->features & NETIF_F_HIGHDMA)
		rxdesc->desc1.flags = RXFLAG_64BIT;
	wmb();
	rxdesc->desc1.flags	|= RXFLAG_OWN | RXFLAG_INT;
}

static int
jme_make_new_rx_buf(struct jme_adapter *jme, int i)
{
	struct jme_ring *rxring = &(jme->rxring[0]);
	struct jme_buffer_info *rxbi = rxring->bufinf + i;
	unsigned long offset;
	struct sk_buff* skb;

	skb = netdev_alloc_skb(jme->dev,
		jme->dev->mtu + RX_EXTRA_LEN);
	if(unlikely(!skb))
		return -ENOMEM;

	if(unlikely(offset =
			(unsigned long)(skb->data)
			& ((unsigned long)RX_BUF_DMA_ALIGN - 1)))
		skb_reserve(skb, RX_BUF_DMA_ALIGN - offset);

	rxbi->skb = skb;
	rxbi->len = skb_tailroom(skb);
	rxbi->mapping = pci_map_page(jme->pdev,
					virt_to_page(skb->data),
					offset_in_page(skb->data),
					rxbi->len,
					PCI_DMA_FROMDEVICE);

	return 0;
}

static void
jme_free_rx_buf(struct jme_adapter *jme, int i)
{
	struct jme_ring *rxring = &(jme->rxring[0]);
	struct jme_buffer_info *rxbi = rxring->bufinf;
	rxbi += i;

	if(rxbi->skb) {
		pci_unmap_page(jme->pdev,
				 rxbi->mapping,
				 rxbi->len,
				 PCI_DMA_FROMDEVICE);
		dev_kfree_skb(rxbi->skb);
		rxbi->skb = NULL;
		rxbi->mapping = 0;
		rxbi->len = 0;
	}
}

static void
jme_free_rx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *rxring = &(jme->rxring[0]);

	if(rxring->alloc) {
		for(i = 0 ; i < jme->rx_ring_size ; ++i)
			jme_free_rx_buf(jme, i);

		dma_free_coherent(&(jme->pdev->dev),
				  RX_RING_ALLOC_SIZE(jme->rx_ring_size),
				  rxring->alloc,
				  rxring->dmaalloc);
		rxring->alloc    = NULL;
		rxring->desc     = NULL;
		rxring->dmaalloc = 0;
		rxring->dma      = 0;
	}
	rxring->next_to_use   = 0;
	rxring->next_to_clean = 0;
}

static int
jme_setup_rx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *rxring = &(jme->rxring[0]);

	rxring->alloc = dma_alloc_coherent(&(jme->pdev->dev),
				   RX_RING_ALLOC_SIZE(jme->rx_ring_size),
				   &(rxring->dmaalloc),
				   GFP_ATOMIC);
	if(!rxring->alloc) {
		rxring->desc = NULL;
		rxring->dmaalloc = 0;
		rxring->dma = 0;
		return -ENOMEM;
	}

	/*
	 * 16 Bytes align
	 */
	rxring->desc		= (void*)ALIGN((unsigned long)(rxring->alloc),
						RING_DESC_ALIGN);
	rxring->dma		= ALIGN(rxring->dmaalloc, RING_DESC_ALIGN);
	rxring->next_to_use	= 0;
	rxring->next_to_clean	= 0;

	/*
	 * Initiallize Receive Descriptors
	 */
	for(i = 0 ; i < jme->rx_ring_size ; ++i) {
		if(unlikely(jme_make_new_rx_buf(jme, i))) {
			jme_free_rx_resources(jme);
			return -ENOMEM;
		}

		jme_set_clean_rxdesc(jme, i);
	}

	return 0;
}

__always_inline static void
jme_enable_rx_engine(struct jme_adapter *jme)
{
	/*
	 * Setup RX DMA Bass Address
	 */
	jwrite32(jme, JME_RXDBA_LO, (__u64)jme->rxring[0].dma & 0xFFFFFFFFUL);
	jwrite32(jme, JME_RXDBA_HI, (__u64)(jme->rxring[0].dma) >> 32);
	jwrite32(jme, JME_RXNDA, (__u64)jme->rxring[0].dma & 0xFFFFFFFFUL);

	/*
	 * Setup RX Descriptor Count
	 */
	jwrite32(jme, JME_RXQDC, jme->rx_ring_size);

	/*
	 * Setup Unicast Filter
	 */
	jme_set_multi(jme->dev);

	/*
	 * Enable RX Engine
	 */
	wmb();
	jwrite32(jme, JME_RXCS, jme->reg_rxcs |
				RXCS_QUEUESEL_Q0 |
				RXCS_ENABLE |
				RXCS_QST);
}

__always_inline static void
jme_restart_rx_engine(struct jme_adapter *jme)
{
	/*
	 * Start RX Engine
	 */
	jwrite32(jme, JME_RXCS, jme->reg_rxcs |
				RXCS_QUEUESEL_Q0 |
				RXCS_ENABLE |
				RXCS_QST);
}


__always_inline static void
jme_disable_rx_engine(struct jme_adapter *jme)
{
	int i;
	__u32 val;

	/*
	 * Disable RX Engine
	 */
	jwrite32(jme, JME_RXCS, jme->reg_rxcs);

	val = jread32(jme, JME_RXCS);
	for(i = JME_RX_DISABLE_TIMEOUT ; (val & RXCS_ENABLE) && i > 0 ; --i)
	{
		mdelay(1);
		val = jread32(jme, JME_RXCS);
	}

	if(!i)
		jeprintk(jme->dev->name, "Disable RX engine timeout.\n");

}

static int
jme_rxsum_ok(struct jme_adapter *jme, __u16 flags)
{
	if(!(flags & (RXWBFLAG_TCPON | RXWBFLAG_UDPON | RXWBFLAG_IPV4)))
		return false;

	if(unlikely((flags & RXWBFLAG_TCPON) &&
	!(flags & RXWBFLAG_TCPCS))) {
		csum_dbg(jme->dev->name, "TCP Checksum error.\n");
		return false;
	}

	if(unlikely((flags & RXWBFLAG_UDPON) &&
	!(flags & RXWBFLAG_UDPCS))) {
		csum_dbg(jme->dev->name, "UDP Checksum error.\n");
		return false;
	}

	if(unlikely((flags & RXWBFLAG_IPV4) &&
	!(flags & RXWBFLAG_IPCS))) {
		csum_dbg(jme->dev->name, "IPv4 Checksum error.\n");
		return false;
	}

	return true;
}

static void
jme_alloc_and_feed_skb(struct jme_adapter *jme, int idx)
{
	struct jme_ring *rxring = &(jme->rxring[0]);
	volatile struct rxdesc *rxdesc = rxring->desc;
	struct jme_buffer_info *rxbi = rxring->bufinf;
	struct sk_buff *skb;
	int framesize;

	rxdesc += idx;
	rxbi += idx;

	skb = rxbi->skb;
	pci_dma_sync_single_for_cpu(jme->pdev,
					rxbi->mapping,
					rxbi->len,
					PCI_DMA_FROMDEVICE);

	if(unlikely(jme_make_new_rx_buf(jme, idx))) {
		pci_dma_sync_single_for_device(jme->pdev,
						rxbi->mapping,
						rxbi->len,
						PCI_DMA_FROMDEVICE);

		++(NET_STAT(jme).rx_dropped);
	}
	else {
		framesize = le16_to_cpu(rxdesc->descwb.framesize)
				- RX_PREPAD_SIZE;

		skb_reserve(skb, RX_PREPAD_SIZE);
		skb_put(skb, framesize);
		skb->protocol = eth_type_trans(skb, jme->dev);

		if(jme_rxsum_ok(jme, rxdesc->descwb.flags))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			skb->ip_summed = CHECKSUM_NONE;


		if(rxdesc->descwb.flags & RXWBFLAG_TAGON) {
			vlan_dbg(jme->dev->name, "VLAN: %04x\n",
					rxdesc->descwb.vlan);
			if(jme->vlgrp) {
				vlan_dbg(jme->dev->name,
					"VLAN Passed to kernel.\n");
				vlan_hwaccel_rx(skb, jme->vlgrp,
					le32_to_cpu(rxdesc->descwb.vlan));
				NET_STAT(jme).rx_bytes += 4;
			}
		}
		else {
			netif_rx(skb);
		}

		if((le16_to_cpu(rxdesc->descwb.flags) & RXWBFLAG_DEST) ==
				RXWBFLAG_DEST_MUL)
			++(NET_STAT(jme).multicast);

		jme->dev->last_rx = jiffies;
		NET_STAT(jme).rx_bytes += framesize;
		++(NET_STAT(jme).rx_packets);
	}

	jme_set_clean_rxdesc(jme, idx);

}



static int
jme_process_receive(struct jme_adapter *jme, int limit)
{
	struct jme_ring *rxring = &(jme->rxring[0]);
	volatile struct rxdesc *rxdesc = rxring->desc;
	int i, j, ccnt, desccnt, mask = jme->rx_ring_mask;

	if(unlikely(!atomic_dec_and_test(&jme->rx_cleaning)))
		goto out_inc;

	if(unlikely(atomic_read(&jme->link_changing) != 1))
		goto out_inc;

	if(unlikely(!netif_carrier_ok(jme->dev)))
		goto out_inc;

	i = rxring->next_to_clean;
	while( limit-- > 0 )
	{
		rxdesc = rxring->desc;
		rxdesc += i;

		if((rxdesc->descwb.flags & RXWBFLAG_OWN) ||
		!(rxdesc->descwb.desccnt & RXWBDCNT_WBCPL))
			goto out;

		desccnt = rxdesc->descwb.desccnt & RXWBDCNT_DCNT;

		rx_dbg(jme->dev->name, "RX: Cleaning %d\n", i);

		if(unlikely(desccnt > 1 ||
		rxdesc->descwb.errstat & RXWBERR_ALLERR)) {

			if(rxdesc->descwb.errstat & RXWBERR_CRCERR)
				++(NET_STAT(jme).rx_crc_errors);
			else if(rxdesc->descwb.errstat & RXWBERR_OVERUN)
				++(NET_STAT(jme).rx_fifo_errors);
			else
				++(NET_STAT(jme).rx_errors);

			if(desccnt > 1) {
				rx_dbg(jme->dev->name,
					"RX: More than one(%d) descriptor, "
					"framelen=%d\n",
					desccnt, le16_to_cpu(rxdesc->descwb.framesize));
				limit -= desccnt - 1;
			}

			for(j = i, ccnt = desccnt ; ccnt-- ; ) {
				jme_set_clean_rxdesc(jme, j);
				j = (j + 1) & (mask);
			}

		}
		else {
			jme_alloc_and_feed_skb(jme, i);
		}

		i = (i + desccnt) & (mask);
	}


out:
	rx_dbg(jme->dev->name, "RX: Stop at %d\n", i);
	rx_dbg(jme->dev->name, "RX: RXNDA offset %d\n",
		(jread32(jme, JME_RXNDA) - jread32(jme, JME_RXDBA_LO))
			>> 4);

	rxring->next_to_clean = i;

out_inc:
	atomic_inc(&jme->rx_cleaning);

	return limit > 0 ? limit : 0;

}

static void
jme_attempt_pcc(struct dynpcc_info *dpi, int atmp)
{
	if(likely(atmp == dpi->cur)) {
		dpi->cnt = 0;
		return;
	}

	if(dpi->attempt == atmp) {
		++(dpi->cnt);
	}
	else {
		dpi->attempt = atmp;
		dpi->cnt = 0;
	}

}

static void
jme_dynamic_pcc(struct jme_adapter *jme)
{
	register struct dynpcc_info *dpi = &(jme->dpi);

	if((NET_STAT(jme).rx_bytes - dpi->last_bytes) > PCC_P3_THRESHOLD)
		jme_attempt_pcc(dpi, PCC_P3);
	else if((NET_STAT(jme).rx_packets - dpi->last_pkts) > PCC_P2_THRESHOLD
	|| dpi->intr_cnt > PCC_INTR_THRESHOLD)
		jme_attempt_pcc(dpi, PCC_P2);
	else
		jme_attempt_pcc(dpi, PCC_P1);

	if(unlikely(dpi->attempt != dpi->cur && dpi->cnt > 5)) {
		jme_set_rx_pcc(jme, dpi->attempt);
		dpi->cur = dpi->attempt;
		dpi->cnt = 0;
	}
}

static void
jme_start_pcc_timer(struct jme_adapter *jme)
{
	struct dynpcc_info *dpi = &(jme->dpi);
	dpi->last_bytes		= NET_STAT(jme).rx_bytes;
	dpi->last_pkts		= NET_STAT(jme).rx_packets;
	dpi->intr_cnt		= 0;
	jwrite32(jme, JME_TMCSR,
		TMCSR_EN | ((0xFFFFFF - PCC_INTERVAL_US) & TMCSR_CNT));
}

__always_inline static void
jme_stop_pcc_timer(struct jme_adapter *jme)
{
	jwrite32(jme, JME_TMCSR, 0);
}

static void
jme_pcc_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;
	struct net_device *netdev = jme->dev;


	if(unlikely(!netif_carrier_ok(netdev) ||
		(atomic_read(&jme->link_changing) != 1)
	)) {
		jme_stop_pcc_timer(jme);
		return;
	}

	if(!(jme->flags & JME_FLAG_POLL))
		jme_dynamic_pcc(jme);

	jme_start_pcc_timer(jme);
}

__always_inline static void
jme_polling_mode(struct jme_adapter *jme)
{
	jme_set_rx_pcc(jme, PCC_OFF);
}

__always_inline static void
jme_interrupt_mode(struct jme_adapter *jme)
{
	jme_set_rx_pcc(jme, PCC_P1);
}

static void
jme_link_change_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;
	struct net_device *netdev = jme->dev;
	int timeout = WAIT_TASKLET_TIMEOUT;
	int rc;

	if(!atomic_dec_and_test(&jme->link_changing))
		goto out;

	if(jme_check_link(netdev, 1) && jme->old_mtu == netdev->mtu)
		goto out;

	jme->old_mtu = netdev->mtu;
	netif_stop_queue(netdev);

	while(--timeout > 0 &&
		(
		atomic_read(&jme->rx_cleaning) != 1 ||
		atomic_read(&jme->tx_cleaning) != 1
		)) {

		mdelay(1);
	}

	if(netif_carrier_ok(netdev)) {
		jme_stop_pcc_timer(jme);
		jme_reset_mac_processor(jme);
		jme_free_rx_resources(jme);
		jme_free_tx_resources(jme);

		if(jme->flags & JME_FLAG_POLL) {
			jme_polling_mode(jme);
			napi_disable(&jme->napi);
		}
	}

	jme_check_link(netdev, 0);
	if(netif_carrier_ok(netdev)) {
		rc = jme_setup_rx_resources(jme);
		if(rc) {
			jeprintk(netdev->name,
				"Allocating resources for RX error"
				", Device STOPPED!\n");
			goto out;
		}


		rc = jme_setup_tx_resources(jme);
		if(rc) {
			jeprintk(netdev->name,
				"Allocating resources for TX error"
				", Device STOPPED!\n");
			goto err_out_free_rx_resources;
		}

		jme_enable_rx_engine(jme);
		jme_enable_tx_engine(jme);

		netif_start_queue(netdev);

		if(jme->flags & JME_FLAG_POLL) {
			napi_enable(&jme->napi);
			jme_interrupt_mode(jme);
		}

		jme_start_pcc_timer(jme);
	}

	goto out;

err_out_free_rx_resources:
	jme_free_rx_resources(jme);
out:
	atomic_inc(&jme->link_changing);
}

static void
jme_rx_clean_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;
	struct dynpcc_info *dpi = &(jme->dpi);

	jme_process_receive(jme, jme->rx_ring_size);
	++(dpi->intr_cnt);

}

static int
jme_poll(struct napi_struct *napi, int budget)
{
	struct jme_adapter *jme = container_of(napi, struct jme_adapter, napi);
	struct net_device *netdev = jme->dev;
	int rest;

	rest = jme_process_receive(jme, budget);

	while(!atomic_dec_and_test(&jme->rx_empty)) {
		++(NET_STAT(jme).rx_dropped);
		jme_restart_rx_engine(jme);
	}
	atomic_inc(&jme->rx_empty);

	if(rest) {
		netif_rx_complete(netdev, napi);
		jme_interrupt_mode(jme);
	}

	return budget - rest;
}

static void
jme_rx_empty_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;

	if(unlikely(atomic_read(&jme->link_changing) != 1))
		return;

	if(unlikely(!netif_carrier_ok(jme->dev)))
		return;

	queue_dbg(jme->dev->name, "RX Queue Full!\n");

	jme_rx_clean_tasklet(arg);
	jme_restart_rx_engine(jme);
}

static void
jme_wake_queue_if_stopped(struct jme_adapter *jme)
{
	struct jme_ring *txring = jme->txring;

	smp_wmb();
	if(unlikely(netif_queue_stopped(jme->dev) &&
	atomic_read(&txring->nr_free) >= (jme->tx_wake_threshold))) {

		queue_dbg(jme->dev->name, "TX Queue Waked.\n");
		netif_wake_queue(jme->dev);

	}

}

static void
jme_tx_clean_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;
	struct jme_ring *txring = &(jme->txring[0]);
	volatile struct txdesc *txdesc = txring->desc;
	struct jme_buffer_info *txbi = txring->bufinf, *ctxbi, *ttxbi;
	int i, j, cnt = 0, max, err, mask;

	if(unlikely(!atomic_dec_and_test(&jme->tx_cleaning)))
		goto out;

	if(unlikely(atomic_read(&jme->link_changing) != 1))
		goto out;

	if(unlikely(!netif_carrier_ok(jme->dev)))
		goto out;

	max = jme->tx_ring_size - atomic_read(&txring->nr_free);
	mask = jme->tx_ring_mask;

	tx_dbg(jme->dev->name, "Tx Tasklet: In\n");

	for(i = txring->next_to_clean ; cnt < max ; ) {

		ctxbi = txbi + i;

		if(likely(ctxbi->skb &&
		!(txdesc[i].descwb.flags & TXWBFLAG_OWN))) {

			err = txdesc[i].descwb.flags & TXWBFLAG_ALLERR;

			tx_dbg(jme->dev->name,
				"Tx Tasklet: Clean %d+%d\n",
				i, ctxbi->nr_desc);

			for(j = 1 ; j < ctxbi->nr_desc ; ++j) {
				ttxbi = txbi + ((i + j) & (mask));
				txdesc[(i + j) & (mask)].dw[0] = 0;

				pci_unmap_page(jme->pdev,
						 ttxbi->mapping,
						 ttxbi->len,
						 PCI_DMA_TODEVICE);

				ttxbi->mapping = 0;
				ttxbi->len = 0;
			}

			dev_kfree_skb(ctxbi->skb);

			cnt += ctxbi->nr_desc;

			if(unlikely(err))
				++(NET_STAT(jme).tx_carrier_errors);
			else {
				++(NET_STAT(jme).tx_packets);
				NET_STAT(jme).tx_bytes += ctxbi->len;
			}

			ctxbi->skb = NULL;
			ctxbi->len = 0;
		}
		else {
			if(!ctxbi->skb)
				tx_dbg(jme->dev->name,
					"Tx Tasklet:"
					" Stopped due to no skb.\n");
			else
				tx_dbg(jme->dev->name,
					"Tx Tasklet:"
					"Stopped due to not done.\n");
			break;
		}

		i = (i + ctxbi->nr_desc) & mask;

		ctxbi->nr_desc = 0;
	}

	tx_dbg(jme->dev->name,
		"Tx Tasklet: Stop %d Jiffies %lu\n",
		i, jiffies);
	txring->next_to_clean = i;

	atomic_add(cnt, &txring->nr_free);

	jme_wake_queue_if_stopped(jme);

out:
	atomic_inc(&jme->tx_cleaning);
}

static void
jme_intr_msi(struct jme_adapter *jme, __u32 intrstat)
{
	/*
	 * Disable interrupt
	 */
	jwrite32f(jme, JME_IENC, INTR_ENABLE);

	/*
	 * Write 1 clear interrupt status
	 */
	jwrite32f(jme, JME_IEVE, intrstat);

	if(intrstat & (INTR_LINKCH | INTR_SWINTR)) {
		tasklet_schedule(&jme->linkch_task);
		goto out_reenable;
	}

	if(intrstat & INTR_TMINTR)
		tasklet_schedule(&jme->pcc_task);

	if(intrstat & (INTR_PCCTXTO | INTR_PCCTX))
		tasklet_schedule(&jme->txclean_task);

	if(jme->flags & JME_FLAG_POLL) {
		if(intrstat & INTR_RX0EMP)
			atomic_inc(&jme->rx_empty);

		if((intrstat & (INTR_PCCRX0TO | INTR_PCCRX0 | INTR_RX0EMP))) {
			if(likely(
			netif_rx_schedule_prep(jme->dev, &jme->napi))) {
				jme_polling_mode(jme);
				__netif_rx_schedule(jme->dev, &jme->napi);
			}
		}
	}
	else {
		if(intrstat & INTR_RX0EMP)
			tasklet_schedule(&jme->rxempty_task);

		if(intrstat & (INTR_PCCRX0TO | INTR_PCCRX0))
			tasklet_schedule(&jme->rxclean_task);
	}

out_reenable:
	/*
	 * Re-enable interrupt
	 */
	jwrite32f(jme, JME_IENS, INTR_ENABLE);


}

static irqreturn_t
jme_intr(int irq, void *dev_id)
{
        struct net_device *netdev = dev_id;
        struct jme_adapter *jme = netdev_priv(netdev);
	__u32 intrstat;

	intrstat = jread32(jme, JME_IEVE);

	/*
	 * Check if it's really an interrupt for us
	 */
        if(unlikely(intrstat == 0))
		return IRQ_NONE;

	/*
	 * Check if the device still exist
	 */
	if(unlikely(intrstat == ~((typeof(intrstat))0)))
                return IRQ_NONE;

	jme_intr_msi(jme, intrstat);

        return IRQ_HANDLED;
}

static irqreturn_t
jme_msi(int irq, void *dev_id)
{
        struct net_device *netdev = dev_id;
        struct jme_adapter *jme = netdev_priv(netdev);
	__u32 intrstat;

	pci_dma_sync_single_for_cpu(jme->pdev,
				    jme->shadow_dma,
				    sizeof(__u32) * SHADOW_REG_NR,
				    PCI_DMA_FROMDEVICE);
	intrstat = jme->shadow_regs[SHADOW_IEVE];
	jme->shadow_regs[SHADOW_IEVE] = 0;

	jme_intr_msi(jme, intrstat);

        return IRQ_HANDLED;
}


static void
jme_reset_link(struct jme_adapter *jme)
{
	jwrite32(jme, JME_TMCSR, TMCSR_SWIT);
}

static void
jme_restart_an(struct jme_adapter *jme)
{
	__u32 bmcr;
	unsigned long flags;

	spin_lock_irqsave(&jme->phy_lock, flags);
	bmcr = jme_mdio_read(jme->dev, jme->mii_if.phy_id, MII_BMCR);
	bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
	jme_mdio_write(jme->dev, jme->mii_if.phy_id, MII_BMCR, bmcr);
	spin_unlock_irqrestore(&jme->phy_lock, flags);
}

static int
jme_request_irq(struct jme_adapter *jme)
{
	int rc;
        struct net_device *netdev = jme->dev;
        irq_handler_t handler = jme_intr;
        int irq_flags = IRQF_SHARED;

        if (!pci_enable_msi(jme->pdev)) {
                jme->flags |= JME_FLAG_MSI;
                handler = jme_msi;
                irq_flags = 0;
        }

        rc = request_irq(jme->pdev->irq, handler, irq_flags, netdev->name,
                          netdev);
        if(rc) {
                jeprintk(netdev->name,
			"Unable to request %s interrupt (return: %d)\n",
			jme->flags & JME_FLAG_MSI ? "MSI":"INTx", rc);

                if(jme->flags & JME_FLAG_MSI) {
                        pci_disable_msi(jme->pdev);
			jme->flags &= ~JME_FLAG_MSI;
		}
        }
	else {
		netdev->irq = jme->pdev->irq;
	}

        return rc;
}

static void
jme_free_irq(struct jme_adapter *jme)
{
        free_irq(jme->pdev->irq, jme->dev);
        if (jme->flags & JME_FLAG_MSI) {
                pci_disable_msi(jme->pdev);
		jme->flags &= ~JME_FLAG_MSI;
		jme->dev->irq = jme->pdev->irq;
        }
}

static int
jme_open(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int rc, timeout = 100;

	while(
		--timeout > 0 &&
		(
		atomic_read(&jme->link_changing) != 1 ||
		atomic_read(&jme->rx_cleaning) != 1 ||
		atomic_read(&jme->tx_cleaning) != 1
		)
	)
		msleep(10);

	if(!timeout) {
		rc = -EBUSY;
		goto err_out;
	}

	jme_clear_pm(jme);
	jme_reset_mac_processor(jme);

	rc = jme_request_irq(jme);
	if(rc)
		goto err_out;

	jme_enable_shadow(jme);
	jme_start_irq(jme);

	if(jme->flags & JME_FLAG_SSET)
		jme_set_settings(netdev, &jme->old_ecmd);
	else
		jme_reset_phy_processor(jme);

	jme_reset_link(jme);

	return 0;

err_out:
	netif_stop_queue(netdev);
	netif_carrier_off(netdev);
	return rc;
}

static void
jme_set_100m_half(struct jme_adapter *jme)
{
	__u32 bmcr, tmp;

	bmcr = jme_mdio_read(jme->dev, jme->mii_if.phy_id, MII_BMCR);
	tmp = bmcr & ~(BMCR_ANENABLE | BMCR_SPEED100 |
		       BMCR_SPEED1000 | BMCR_FULLDPLX);
	tmp |= BMCR_SPEED100;

	if (bmcr != tmp)
		jme_mdio_write(jme->dev, jme->mii_if.phy_id, MII_BMCR, tmp);

	jwrite32(jme, JME_GHC, GHC_SPEED_100M);
}

static void
jme_phy_off(struct jme_adapter *jme)
{
	jme_mdio_write(jme->dev, jme->mii_if.phy_id, MII_BMCR, BMCR_PDOWN);
}


static int
jme_close(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	jme_stop_irq(jme);
	jme_disable_shadow(jme);
	jme_free_irq(jme);

	if(jme->flags & JME_FLAG_POLL)
		napi_disable(&jme->napi);

	tasklet_kill(&jme->linkch_task);
	tasklet_kill(&jme->txclean_task);
	tasklet_kill(&jme->rxclean_task);
	tasklet_kill(&jme->rxempty_task);

	jme_reset_mac_processor(jme);
	jme_free_rx_resources(jme);
	jme_free_tx_resources(jme);
	jme->phylink = 0;
	jme_phy_off(jme);

	return 0;
}

static int
jme_alloc_txdesc(struct jme_adapter *jme,
			struct sk_buff *skb)
{
	struct jme_ring *txring = jme->txring;
	int idx, nr_alloc, mask = jme->tx_ring_mask;

	idx = txring->next_to_use;
	nr_alloc = skb_shinfo(skb)->nr_frags + 2;

	if(unlikely(atomic_read(&txring->nr_free) < nr_alloc))
		return -1;

	atomic_sub(nr_alloc, &txring->nr_free);

	txring->next_to_use = (txring->next_to_use + nr_alloc) & mask;

	return idx;
}

static void
jme_fill_tx_map(struct pci_dev *pdev,
		volatile struct txdesc *txdesc,
		struct jme_buffer_info *txbi,
		struct page *page,
		__u32 page_offset,
		__u32 len,
		__u8 hidma)
{
	dma_addr_t dmaaddr;

	dmaaddr = pci_map_page(pdev,
				page,
				page_offset,
				len,
				PCI_DMA_TODEVICE);

	pci_dma_sync_single_for_device(pdev,
				       dmaaddr,
				       len,
				       PCI_DMA_TODEVICE);

	txdesc->dw[0] = 0;
	txdesc->dw[1] = 0;
	txdesc->desc2.flags	= TXFLAG_OWN;
	txdesc->desc2.flags	|= (hidma)?TXFLAG_64BIT:0;
	txdesc->desc2.datalen	= cpu_to_le16(len);
	txdesc->desc2.bufaddrh	= cpu_to_le32((__u64)dmaaddr >> 32);
	txdesc->desc2.bufaddrl	= cpu_to_le32(
					(__u64)dmaaddr & 0xFFFFFFFFUL);

	txbi->mapping = dmaaddr;
	txbi->len = len;
}

static void
jme_map_tx_skb(struct jme_adapter *jme, struct sk_buff *skb, int idx)
{
	struct jme_ring *txring = jme->txring;
	volatile struct txdesc *txdesc = txring->desc, *ctxdesc;
	struct jme_buffer_info *txbi = txring->bufinf, *ctxbi;
	__u8 hidma = jme->dev->features & NETIF_F_HIGHDMA;
	int i, nr_frags = skb_shinfo(skb)->nr_frags;
	int mask = jme->tx_ring_mask;
	struct skb_frag_struct *frag;
	__u32 len;

	for(i = 0 ; i < nr_frags ; ++i) {
                frag = &skb_shinfo(skb)->frags[i];
		ctxdesc = txdesc + ((idx + i + 2) & (mask));
		ctxbi = txbi + ((idx + i + 2) & (mask));

		jme_fill_tx_map(jme->pdev, ctxdesc, ctxbi, frag->page,
				 frag->page_offset, frag->size, hidma);
	}

	len = skb_is_nonlinear(skb)?skb_headlen(skb):skb->len;
	ctxdesc = txdesc + ((idx + 1) & (mask));
	ctxbi = txbi + ((idx + 1) & (mask));
	jme_fill_tx_map(jme->pdev, ctxdesc, ctxbi, virt_to_page(skb->data),
			offset_in_page(skb->data), len, hidma);

}

static int
jme_expand_header(struct jme_adapter *jme, struct sk_buff *skb)
{
	if(unlikely(skb_shinfo(skb)->gso_size &&
			skb_header_cloned(skb) &&
			pskb_expand_head(skb, 0, 0, GFP_ATOMIC))) {
		dev_kfree_skb(skb);
		return -1;
	}

	return 0;
}

static int
jme_tx_tso(struct sk_buff *skb,
		volatile __u16 *mss, __u8 *flags)
{
	if((*mss = (skb_shinfo(skb)->gso_size << TXDESC_MSS_SHIFT))) {
		*flags |= TXFLAG_LSEN;

		if(skb->protocol == __constant_htons(ETH_P_IP)) {
			struct iphdr *iph = ip_hdr(skb);

			iph->check = 0;
                        tcp_hdr(skb)->check = ~csum_tcpudp_magic(iph->saddr,
								iph->daddr, 0,
								IPPROTO_TCP,
								0);
		}
		else {
			struct ipv6hdr *ip6h = ipv6_hdr(skb);

                        tcp_hdr(skb)->check = ~csum_ipv6_magic(&ip6h->saddr,
								&ip6h->daddr, 0,
								IPPROTO_TCP,
								0);
		}

		return 0;
	}

	return 1;
}

static void
jme_tx_csum(struct sk_buff *skb, __u8 *flags)
{
	if(skb->ip_summed == CHECKSUM_PARTIAL) {
		__u8 ip_proto;

		switch (skb->protocol) {
		case __constant_htons(ETH_P_IP):
			ip_proto = ip_hdr(skb)->protocol;
			break;
		case __constant_htons(ETH_P_IPV6):
			ip_proto = ipv6_hdr(skb)->nexthdr;
			break;
		default:
			ip_proto = 0;
			break;
		}

		switch(ip_proto) {
		case IPPROTO_TCP:
			*flags |= TXFLAG_TCPCS;
			break;
		case IPPROTO_UDP:
			*flags |= TXFLAG_UDPCS;
			break;
		default:
			jeprintk("jme", "Error upper layer protocol.\n");
			break;
		}
	}
}

__always_inline static void
jme_tx_vlan(struct sk_buff *skb, volatile __u16 *vlan, __u8 *flags)
{
	if(vlan_tx_tag_present(skb)) {
		vlan_dbg("jme", "Tag found!(%04x)\n", vlan_tx_tag_get(skb));
		*flags |= TXFLAG_TAGON;
		*vlan = vlan_tx_tag_get(skb);
	}
}

static int
jme_fill_first_tx_desc(struct jme_adapter *jme, struct sk_buff *skb, int idx)
{
	struct jme_ring *txring = jme->txring;
	volatile struct txdesc *txdesc;
	struct jme_buffer_info *txbi;
	__u8 flags;

	txdesc = (volatile struct txdesc*)txring->desc + idx;
	txbi = txring->bufinf + idx;

	txdesc->dw[0] = 0;
	txdesc->dw[1] = 0;
	txdesc->dw[2] = 0;
	txdesc->dw[3] = 0;
	txdesc->desc1.pktsize = cpu_to_le16(skb->len);
	/*
	 * Set OWN bit at final.
	 * When kernel transmit faster than NIC.
	 * And NIC trying to send this descriptor before we tell
	 * it to start sending this TX queue.
	 * Other fields are already filled correctly.
	 */
	wmb();
	flags = TXFLAG_OWN | TXFLAG_INT;
	//Set checksum flags while not tso
	if(jme_tx_tso(skb, &txdesc->desc1.mss, &flags))
		jme_tx_csum(skb, &flags);
	jme_tx_vlan(skb, &txdesc->desc1.vlan, &flags);
	txdesc->desc1.flags = flags;
	/*
	 * Set tx buffer info after telling NIC to send
	 * For better tx_clean timing
	 */
	wmb();
	txbi->nr_desc = skb_shinfo(skb)->nr_frags + 2;
	txbi->skb = skb;
	txbi->len = skb->len;

	return 0;
}

static void
jme_stop_queue_if_full(struct jme_adapter *jme)
{
	struct jme_ring *txring = jme->txring;

	smp_wmb();
	if(unlikely(atomic_read(&txring->nr_free) < (MAX_SKB_FRAGS+2))) {
		netif_stop_queue(jme->dev);
		queue_dbg(jme->dev->name, "TX Queue Paused.\n");
		smp_wmb();
		if (atomic_read(&txring->nr_free) >= (jme->tx_wake_threshold)) {
			netif_wake_queue(jme->dev);
			queue_dbg(jme->dev->name, "TX Queue Fast Waked.\n");
		}
	}

}

/*
 * This function is already protected by netif_tx_lock()
 */
static int
jme_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
        struct jme_adapter *jme = netdev_priv(netdev);
	int idx;

	if(skb_shinfo(skb)->nr_frags) {
		tx_dbg(netdev->name, "Frags: %d Headlen: %d Len: %d MSS: %d Sum:%d\n",
			skb_shinfo(skb)->nr_frags,
			skb_headlen(skb),
			skb->len,
			skb_shinfo(skb)->gso_size,
			skb->ip_summed);
	}

	if(unlikely(jme_expand_header(jme, skb))) {
		++(NET_STAT(jme).tx_dropped);
		return NETDEV_TX_OK;
	}

	idx = jme_alloc_txdesc(jme, skb);

	if(unlikely(idx<0)) {
		netif_stop_queue(netdev);
		jeprintk(netdev->name,
				"BUG! Tx ring full when queue awake!\n");

                return NETDEV_TX_BUSY;
	}

	jme_map_tx_skb(jme, skb, idx);
	jme_fill_first_tx_desc(jme, skb, idx);

	tx_dbg(jme->dev->name, "Xmit: %d+%d\n", idx, skb_shinfo(skb)->nr_frags + 2);

	jwrite32(jme, JME_TXCS, jme->reg_txcs |
				TXCS_SELECT_QUEUE0 |
				TXCS_QUEUE0S |
				TXCS_ENABLE);
	netdev->trans_start = jiffies;

	jme_stop_queue_if_full(jme);

        return NETDEV_TX_OK;
}

static int
jme_set_macaddr(struct net_device *netdev, void *p)
{
        struct jme_adapter *jme = netdev_priv(netdev);
	struct sockaddr *addr = p;
	__u32 val;

	if(netif_running(netdev))
		return -EBUSY;

	spin_lock(&jme->macaddr_lock);
	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	val = addr->sa_data[3] << 24 |
	      addr->sa_data[2] << 16 |
	      addr->sa_data[1] <<  8 |
	      addr->sa_data[0];
	jwrite32(jme, JME_RXUMA_LO, val);
	val = addr->sa_data[5] << 8 |
	      addr->sa_data[4];
	jwrite32(jme, JME_RXUMA_HI, val);
	spin_unlock(&jme->macaddr_lock);

	return 0;
}

static void
jme_set_multi(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	u32 mc_hash[2] = {};
	int i;
	unsigned long flags;

	spin_lock_irqsave(&jme->rxmcs_lock, flags);

	jme->reg_rxmcs |= RXMCS_BRDFRAME | RXMCS_UNIFRAME;

        if (netdev->flags & IFF_PROMISC) {
		jme->reg_rxmcs |= RXMCS_ALLFRAME;
	}
        else if (netdev->flags & IFF_ALLMULTI) {
		jme->reg_rxmcs |= RXMCS_ALLMULFRAME;
	}
	else if(netdev->flags & IFF_MULTICAST) {
		struct dev_mc_list *mclist;
		int bit_nr;

		jme->reg_rxmcs |= RXMCS_MULFRAME | RXMCS_MULFILTERED;
		for (i = 0, mclist = netdev->mc_list;
			mclist && i < netdev->mc_count;
			++i, mclist = mclist->next) {

                        bit_nr = ether_crc(ETH_ALEN, mclist->dmi_addr) & 0x3F;
                        mc_hash[bit_nr >> 5] |= 1 << (bit_nr & 0x1F);
                }

		jwrite32(jme, JME_RXMCHT_LO, mc_hash[0]);
		jwrite32(jme, JME_RXMCHT_HI, mc_hash[1]);
	}

	wmb();
	jwrite32(jme, JME_RXMCS, jme->reg_rxmcs);

	spin_unlock_irqrestore(&jme->rxmcs_lock, flags);
}

static int
jme_change_mtu(struct net_device *netdev, int new_mtu)
{
        struct jme_adapter *jme = netdev_priv(netdev);

	if(new_mtu == jme->old_mtu)
		return 0;

        if (((new_mtu + ETH_HLEN) > MAX_ETHERNET_JUMBO_PACKET_SIZE) ||
                ((new_mtu) < IPV6_MIN_MTU))
                return -EINVAL;

	if(new_mtu > 4000) {
		jme->reg_rxcs &= ~RXCS_FIFOTHNP;
		jme->reg_rxcs |= RXCS_FIFOTHNP_64QW;
		jme_restart_rx_engine(jme);
	}
	else {
		jme->reg_rxcs &= ~RXCS_FIFOTHNP;
		jme->reg_rxcs |= RXCS_FIFOTHNP_128QW;
		jme_restart_rx_engine(jme);
	}

	if(new_mtu > 1900) {
		netdev->features &= ~(NETIF_F_HW_CSUM |
				NETIF_F_TSO |
				NETIF_F_TSO6);
	}
	else {
		if(jme->flags & JME_FLAG_TXCSUM)
			netdev->features |= NETIF_F_HW_CSUM;
		if(jme->flags & JME_FLAG_TSO)
			netdev->features |= NETIF_F_TSO | NETIF_F_TSO6;
	}

        netdev->mtu = new_mtu;
        jme_reset_link(jme);

	return 0;
}

static void
jme_tx_timeout(struct net_device *netdev)
{
        struct jme_adapter *jme = netdev_priv(netdev);

	/*
	 * Reset the link
	 * And the link change will reinitialize all RX/TX resources
	 */
	jme->phylink = 0;
	jme_reset_link(jme);
}

static void
jme_vlan_rx_register(struct net_device *netdev, struct vlan_group *grp)
{
	struct jme_adapter *jme = netdev_priv(netdev);

	jme->vlgrp = grp;
}

static void
jme_get_drvinfo(struct net_device *netdev,
		     struct ethtool_drvinfo *info)
{
        struct jme_adapter *jme = netdev_priv(netdev);

        strcpy(info->driver, DRV_NAME);
        strcpy(info->version, DRV_VERSION);
        strcpy(info->bus_info, pci_name(jme->pdev));
}

static int
jme_get_regs_len(struct net_device *netdev)
{
	return 0x400;
}

static void
mmapio_memcpy(struct jme_adapter *jme, __u32 *p, __u32 reg, int len)
{
	int i;

	for(i = 0 ; i < len ; i += 4)
		p[i >> 2] = jread32(jme, reg + i);

}

static void
jme_get_regs(struct net_device *netdev, struct ethtool_regs *regs, void *p)
{
        struct jme_adapter *jme = netdev_priv(netdev);
	__u32 *p32 = (__u32*)p;

	memset(p, 0, 0x400);

	regs->version = 1;
	mmapio_memcpy(jme, p32, JME_MAC, JME_MAC_LEN);

	p32 += 0x100 >> 2;
	mmapio_memcpy(jme, p32, JME_PHY, JME_PHY_LEN);

	p32 += 0x100 >> 2;
	mmapio_memcpy(jme, p32, JME_MISC, JME_MISC_LEN);

	p32 += 0x100 >> 2;
	mmapio_memcpy(jme, p32, JME_RSS, JME_RSS_LEN);

}

static int
jme_get_coalesce(struct net_device *netdev, struct ethtool_coalesce *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);

	if(jme->flags & JME_FLAG_POLL)
		ecmd->use_adaptive_rx_coalesce = false;
	else
		ecmd->use_adaptive_rx_coalesce = true;

	ecmd->tx_coalesce_usecs = PCC_TX_TO;
	ecmd->tx_max_coalesced_frames = PCC_TX_CNT;

	switch(jme->dpi.cur) {
	case PCC_P1:
		ecmd->rx_coalesce_usecs = PCC_P1_TO;
		ecmd->rx_max_coalesced_frames = PCC_P1_CNT;
		break;
	case PCC_P2:
		ecmd->rx_coalesce_usecs = PCC_P2_TO;
		ecmd->rx_max_coalesced_frames = PCC_P2_CNT;
		break;
	case PCC_P3:
		ecmd->rx_coalesce_usecs = PCC_P3_TO;
		ecmd->rx_max_coalesced_frames = PCC_P3_CNT;
		break;
	default:
		break;
	}

	return 0;
}

static int
jme_set_coalesce(struct net_device *netdev, struct ethtool_coalesce *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	struct dynpcc_info *dpi = &(jme->dpi);

	if(ecmd->use_adaptive_rx_coalesce
	&& (jme->flags & JME_FLAG_POLL)) {
		jme->flags &= ~JME_FLAG_POLL;
		napi_disable(&jme->napi);
		dpi->cur		= PCC_P1;
		dpi->attempt		= PCC_P1;
		dpi->cnt		= 0;
		jme_set_rx_pcc(jme, PCC_P1);
		jme_interrupt_mode(jme);
	}
	else if(!(ecmd->use_adaptive_rx_coalesce)
	&& !(jme->flags & JME_FLAG_POLL)) {
		jme->flags |= JME_FLAG_POLL;
		napi_enable(&jme->napi);
		jme_interrupt_mode(jme);
	}

	return 0;
}

static void
jme_get_pauseparam(struct net_device *netdev,
			struct ethtool_pauseparam *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	unsigned long flags;
	__u32 val;

	ecmd->tx_pause = (jme->reg_txpfc & TXPFC_PF_EN) != 0;
	ecmd->rx_pause = (jme->reg_rxmcs & RXMCS_FLOWCTRL) != 0;

	spin_lock_irqsave(&jme->phy_lock, flags);
        val = jme_mdio_read(jme->dev, jme->mii_if.phy_id, MII_ADVERTISE);
	spin_unlock_irqrestore(&jme->phy_lock, flags);

	ecmd->autoneg =
		(val & (ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM)) != 0;
}

static int
jme_set_pauseparam(struct net_device *netdev,
			struct ethtool_pauseparam *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	unsigned long flags;
	__u32 val;

	if( ((jme->reg_txpfc & TXPFC_PF_EN) != 0) !=
		(ecmd->tx_pause != 0)) {

		if(ecmd->tx_pause)
			jme->reg_txpfc |= TXPFC_PF_EN;
		else
			jme->reg_txpfc &= ~TXPFC_PF_EN;

		jwrite32(jme, JME_TXPFC, jme->reg_txpfc);
	}

	spin_lock_irqsave(&jme->rxmcs_lock, flags);
	if( ((jme->reg_rxmcs & RXMCS_FLOWCTRL) != 0) !=
		(ecmd->rx_pause != 0)) {

		if(ecmd->rx_pause)
			jme->reg_rxmcs |= RXMCS_FLOWCTRL;
		else
			jme->reg_rxmcs &= ~RXMCS_FLOWCTRL;

		jwrite32(jme, JME_RXMCS, jme->reg_rxmcs);
	}
	spin_unlock_irqrestore(&jme->rxmcs_lock, flags);

	spin_lock_irqsave(&jme->phy_lock, flags);
        val = jme_mdio_read(jme->dev, jme->mii_if.phy_id, MII_ADVERTISE);
	if( ((val & (ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM)) != 0) !=
		(ecmd->autoneg != 0)) {

		if(ecmd->autoneg)
			val |= (ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);
		else
			val &= ~(ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

		jme_mdio_write(jme->dev, jme->mii_if.phy_id,
				MII_ADVERTISE, val);
	}
	spin_unlock_irqrestore(&jme->phy_lock, flags);

	return 0;
}

static void
jme_get_wol(struct net_device *netdev,
		struct ethtool_wolinfo *wol)
{
	struct jme_adapter *jme = netdev_priv(netdev);

	wol->supported = WAKE_MAGIC | WAKE_PHY;

	wol->wolopts = 0;

	if(jme->reg_pmcs & (PMCS_LFEN | PMCS_LREN))
		wol->wolopts |= WAKE_PHY;

	if(jme->reg_pmcs & PMCS_MFEN)
		wol->wolopts |= WAKE_MAGIC;

}

static int
jme_set_wol(struct net_device *netdev,
		struct ethtool_wolinfo *wol)
{
	struct jme_adapter *jme = netdev_priv(netdev);

	if(wol->wolopts & (WAKE_MAGICSECURE |
				WAKE_UCAST |
				WAKE_MCAST |
				WAKE_BCAST |
				WAKE_ARP))
		return -EOPNOTSUPP;

	jme->reg_pmcs = 0;

	if(wol->wolopts & WAKE_PHY)
		jme->reg_pmcs |= PMCS_LFEN | PMCS_LREN;

	if(wol->wolopts & WAKE_MAGIC)
		jme->reg_pmcs |= PMCS_MFEN;


	return 0;
}

static int
jme_get_settings(struct net_device *netdev,
		     struct ethtool_cmd *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&jme->phy_lock, flags);
	rc = mii_ethtool_gset(&(jme->mii_if), ecmd);
	spin_unlock_irqrestore(&jme->phy_lock, flags);
	return rc;
}

static int
jme_set_settings(struct net_device *netdev,
		     struct ethtool_cmd *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int rc, fdc=0;
	unsigned long flags;

	if(ecmd->speed == SPEED_1000 && ecmd->autoneg != AUTONEG_ENABLE)
		return -EINVAL;

	if(jme->mii_if.force_media &&
	ecmd->autoneg != AUTONEG_ENABLE &&
	(jme->mii_if.full_duplex != ecmd->duplex))
		fdc = 1;

	spin_lock_irqsave(&jme->phy_lock, flags);
	rc = mii_ethtool_sset(&(jme->mii_if), ecmd);
	spin_unlock_irqrestore(&jme->phy_lock, flags);

	if(!rc && fdc)
		jme_reset_link(jme);

	if(!rc) {
		jme->flags |= JME_FLAG_SSET;
		jme->old_ecmd = *ecmd;
	}

	return rc;
}

static __u32
jme_get_link(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	return jread32(jme, JME_PHY_LINK) & PHY_LINK_UP;
}

static u32
jme_get_rx_csum(struct net_device *netdev)
{
        struct jme_adapter *jme = netdev_priv(netdev);

	return jme->reg_rxmcs & RXMCS_CHECKSUM;
}

static int
jme_set_rx_csum(struct net_device *netdev, u32 on)
{
        struct jme_adapter *jme = netdev_priv(netdev);
	unsigned long flags;

	spin_lock_irqsave(&jme->rxmcs_lock, flags);
	if(on)
		jme->reg_rxmcs |= RXMCS_CHECKSUM;
	else
		jme->reg_rxmcs &= ~RXMCS_CHECKSUM;
	jwrite32(jme, JME_RXMCS, jme->reg_rxmcs);
	spin_unlock_irqrestore(&jme->rxmcs_lock, flags);

	return 0;
}

static int
jme_set_tx_csum(struct net_device *netdev, u32 on)
{
        struct jme_adapter *jme = netdev_priv(netdev);

	if(on) {
		jme->flags |= JME_FLAG_TXCSUM;
		if(netdev->mtu <= 1900)
			netdev->features |= NETIF_F_HW_CSUM;
	}
	else {
		jme->flags &= ~JME_FLAG_TXCSUM;
		netdev->features &= ~NETIF_F_HW_CSUM;
	}

	return 0;
}

static int
jme_set_tso(struct net_device *netdev, u32 on)
{
        struct jme_adapter *jme = netdev_priv(netdev);

        if (on) {
		jme->flags |= JME_FLAG_TSO;
		if(netdev->mtu <= 1900)
			netdev->features |= NETIF_F_TSO | NETIF_F_TSO6;
	}
        else {
		jme->flags &= ~JME_FLAG_TSO;
                netdev->features &= ~(NETIF_F_TSO | NETIF_F_TSO6);
	}

        return 0;
}

static int
jme_nway_reset(struct net_device *netdev)
{
        struct jme_adapter *jme = netdev_priv(netdev);
	jme_restart_an(jme);
	return 0;
}

static const struct ethtool_ops jme_ethtool_ops = {
        .get_drvinfo            = jme_get_drvinfo,
	.get_regs_len		= jme_get_regs_len,
	.get_regs		= jme_get_regs,
	.get_coalesce		= jme_get_coalesce,
	.set_coalesce		= jme_set_coalesce,
        .get_pauseparam		= jme_get_pauseparam,
        .set_pauseparam		= jme_set_pauseparam,
	.get_wol		= jme_get_wol,
	.set_wol		= jme_set_wol,
	.get_settings		= jme_get_settings,
	.set_settings		= jme_set_settings,
	.get_link		= jme_get_link,
	.get_rx_csum		= jme_get_rx_csum,
	.set_rx_csum		= jme_set_rx_csum,
	.set_tx_csum		= jme_set_tx_csum,
	.set_tso		= jme_set_tso,
	.set_sg			= ethtool_op_set_sg,
	.nway_reset             = jme_nway_reset,
};

static int
jme_pci_dma64(struct pci_dev *pdev)
{
        if (!pci_set_dma_mask(pdev, DMA_64BIT_MASK))
                if(!pci_set_consistent_dma_mask(pdev, DMA_64BIT_MASK)) {
		    	dprintk("jme", "64Bit DMA Selected.\n");
			return 1;
		}

        if (!pci_set_dma_mask(pdev, DMA_40BIT_MASK))
                if(!pci_set_consistent_dma_mask(pdev, DMA_40BIT_MASK)) {
		    	dprintk("jme", "40Bit DMA Selected.\n");
			return 1;
		}

        if (!pci_set_dma_mask(pdev, DMA_32BIT_MASK))
                if(!pci_set_consistent_dma_mask(pdev, DMA_32BIT_MASK)) {
		    	dprintk("jme", "32Bit DMA Selected.\n");
			return 0;
		}

	return -1;
}

__always_inline static void
jme_set_phy_ps(struct jme_adapter *jme)
{
	jme_mdio_write(jme->dev, jme->mii_if.phy_id, 26, 0x00001000);
}

static int __devinit
jme_init_one(struct pci_dev *pdev,
	     const struct pci_device_id *ent)
{
	int rc = 0, using_dac;
	struct net_device *netdev;
	struct jme_adapter *jme;

	/*
	 * set up PCI device basics
	 */
	rc = pci_enable_device(pdev);
	if(rc) {
		printk(KERN_ERR PFX "Cannot enable PCI device.\n");
		goto err_out;
	}

	using_dac = jme_pci_dma64(pdev);
	if(using_dac < 0) {
		printk(KERN_ERR PFX "Cannot set PCI DMA Mask.\n");
		rc = -EIO;
		goto err_out_disable_pdev;
	}

	if(!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM)) {
		printk(KERN_ERR PFX "No PCI resource region found.\n");
		rc = -ENOMEM;
		goto err_out_disable_pdev;
	}

	rc = pci_request_regions(pdev, DRV_NAME);
	if(rc) {
		printk(KERN_ERR PFX "Cannot obtain PCI resource region.\n");
		goto err_out_disable_pdev;
	}

	pci_set_master(pdev);

	/*
	 * alloc and init net device
	 */
	netdev = alloc_etherdev(sizeof(*jme));
	if(!netdev) {
		printk(KERN_ERR PFX "Cannot allocate netdev structure.\n");
		rc = -ENOMEM;
		goto err_out_release_regions;
	}
	netdev->open			= jme_open;
	netdev->stop			= jme_close;
	netdev->hard_start_xmit		= jme_start_xmit;
	netdev->set_mac_address		= jme_set_macaddr;
	netdev->set_multicast_list	= jme_set_multi;
	netdev->change_mtu		= jme_change_mtu;
	netdev->ethtool_ops		= &jme_ethtool_ops;
	netdev->tx_timeout		= jme_tx_timeout;
	netdev->watchdog_timeo		= TX_TIMEOUT;
	netdev->vlan_rx_register	= jme_vlan_rx_register;
	NETDEV_GET_STATS(netdev, &jme_get_stats);
	netdev->features		=	NETIF_F_HW_CSUM |
						NETIF_F_SG |
						NETIF_F_TSO |
						NETIF_F_TSO6 |
						NETIF_F_HW_VLAN_TX |
						NETIF_F_HW_VLAN_RX;
	if(using_dac)
		netdev->features	|=	NETIF_F_HIGHDMA;

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

	/*
	 * init adapter info
	 */
	jme = netdev_priv(netdev);
	jme->pdev = pdev;
	jme->dev = netdev;
	jme->old_mtu = netdev->mtu = 1500;
	jme->phylink = 0;
	jme->tx_ring_size = 1 << 10;
	jme->tx_ring_mask = jme->tx_ring_size - 1;
	jme->tx_wake_threshold = 1 << 9;
	jme->rx_ring_size = 1 << 9;
	jme->rx_ring_mask = jme->rx_ring_size - 1;
	jme->regs = ioremap(pci_resource_start(pdev, 0),
			     pci_resource_len(pdev, 0));
	if (!(jme->regs)) {
		printk(KERN_ERR PFX "Mapping PCI resource region error.\n");
		rc = -ENOMEM;
		goto err_out_free_netdev;
	}
	jme->shadow_regs = pci_alloc_consistent(pdev,
					        sizeof(__u32) * SHADOW_REG_NR,
					        &(jme->shadow_dma));
	if (!(jme->shadow_regs)) {
		printk(KERN_ERR PFX "Allocating shadow register mapping error.\n");
		rc = -ENOMEM;
		goto err_out_unmap;
	}

	netif_napi_add(netdev, &jme->napi, jme_poll, jme->rx_ring_size >> 2);

	spin_lock_init(&jme->phy_lock);
	spin_lock_init(&jme->macaddr_lock);
	spin_lock_init(&jme->rxmcs_lock);

	atomic_set(&jme->link_changing, 1);
	atomic_set(&jme->rx_cleaning, 1);
	atomic_set(&jme->tx_cleaning, 1);
	atomic_set(&jme->rx_empty, 1);

	tasklet_init(&jme->pcc_task,
		     &jme_pcc_tasklet,
		     (unsigned long) jme);
	tasklet_init(&jme->linkch_task,
		     &jme_link_change_tasklet,
		     (unsigned long) jme);
	tasklet_init(&jme->txclean_task,
		     &jme_tx_clean_tasklet,
		     (unsigned long) jme);
	tasklet_init(&jme->rxclean_task,
		     &jme_rx_clean_tasklet,
		     (unsigned long) jme);
	tasklet_init(&jme->rxempty_task,
		     &jme_rx_empty_tasklet,
		     (unsigned long) jme);
	jme->mii_if.dev = netdev;
	jme->mii_if.phy_id = 1;
	jme->mii_if.supports_gmii = 1;
	jme->mii_if.mdio_read = jme_mdio_read;
	jme->mii_if.mdio_write = jme_mdio_write;

	jme->dpi.cur = PCC_P1;

	jme->reg_ghc = GHC_DPX | GHC_SPEED_1000M;
	jme->reg_rxcs = RXCS_DEFAULT;
	jme->reg_rxmcs = RXMCS_DEFAULT;
	jme->reg_txpfc = 0;
	jme->reg_pmcs = PMCS_LFEN | PMCS_LREN | PMCS_MFEN;
	jme->flags = JME_FLAG_TXCSUM | JME_FLAG_TSO | JME_FLAG_POLL;

	/*
	 * Get Max Read Req Size from PCI Config Space
	 */
	pci_read_config_byte(pdev, PCI_CONF_DCSR_MRRS, &jme->mrrs);
	switch(jme->mrrs) {
		case MRRS_128B:
			jme->reg_txcs = TXCS_DEFAULT | TXCS_DMASIZE_128B;
			break;
		case MRRS_256B:
			jme->reg_txcs = TXCS_DEFAULT | TXCS_DMASIZE_256B;
			break;
		default:
			jme->reg_txcs = TXCS_DEFAULT | TXCS_DMASIZE_512B;
			break;
	};


	/*
	 * Reset MAC processor and reload EEPROM for MAC Address
	 */
	jme_clear_pm(jme);
	jme_set_phy_ps(jme);
	jme_phy_off(jme);
	jme_reset_mac_processor(jme);
	rc = jme_reload_eeprom(jme);
	if(rc) {
		printk(KERN_ERR PFX
			"Reload eeprom for reading MAC Address error.\n");
		goto err_out_free_shadow;
	}
	jme_load_macaddr(netdev);


	/*
	 * Tell stack that we are not ready to work until open()
	 */
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);

	/*
	 * Register netdev
	 */
	rc = register_netdev(netdev);
	if(rc) {
		printk(KERN_ERR PFX "Cannot register net device.\n");
		goto err_out_free_shadow;
	}

	jprintk(netdev->name,
		"JMC250 gigabit eth %02x:%02x:%02x:%02x:%02x:%02x\n",
	        netdev->dev_addr[0],
	        netdev->dev_addr[1],
	        netdev->dev_addr[2],
	        netdev->dev_addr[3],
	        netdev->dev_addr[4],
	        netdev->dev_addr[5]);

	return 0;

err_out_free_shadow:
	pci_free_consistent(pdev,
			    sizeof(__u32) * SHADOW_REG_NR,
			    jme->shadow_regs,
			    jme->shadow_dma);
err_out_unmap:
	iounmap(jme->regs);
err_out_free_netdev:
	pci_set_drvdata(pdev, NULL);
	free_netdev(netdev);
err_out_release_regions:
	pci_release_regions(pdev);
err_out_disable_pdev:
        pci_disable_device(pdev);
err_out:
	return rc;
}

static void __devexit
jme_remove_one(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct jme_adapter *jme = netdev_priv(netdev);

	unregister_netdev(netdev);
	pci_free_consistent(pdev,
			    sizeof(__u32) * SHADOW_REG_NR,
			    jme->shadow_regs,
			    jme->shadow_dma);
	iounmap(jme->regs);
	pci_set_drvdata(pdev, NULL);
	free_netdev(netdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);

}

static int
jme_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct jme_adapter *jme = netdev_priv(netdev);
	int timeout = 100;

	atomic_dec(&jme->link_changing);

	netif_device_detach(netdev);
	netif_stop_queue(netdev);
	jme_stop_irq(jme);
	jme_free_irq(jme);

	while(--timeout > 0 &&
	(
		atomic_read(&jme->rx_cleaning) != 1 ||
		atomic_read(&jme->tx_cleaning) != 1
	)) {
		mdelay(1);
	}
	if(!timeout) {
		jeprintk(netdev->name, "Waiting tasklets timeout.\n");
		return -EBUSY;
	}
	jme_disable_shadow(jme);

	if(netif_carrier_ok(netdev)) {
		jme_stop_pcc_timer(jme);
		jme_reset_mac_processor(jme);
		jme_free_rx_resources(jme);
		jme_free_tx_resources(jme);
		netif_carrier_off(netdev);
		jme->phylink = 0;

		if(jme->flags & JME_FLAG_POLL) {
			jme_polling_mode(jme);
			napi_disable(&jme->napi);
		}
	}


	pci_save_state(pdev);
	if(jme->reg_pmcs) {
		jme_set_100m_half(jme);
		jwrite32(jme, JME_PMCS, jme->reg_pmcs);
		pci_enable_wake(pdev, PCI_D3hot, true);
		pci_enable_wake(pdev, PCI_D3cold, true);
	}
	else {
		jme_phy_off(jme);
		pci_enable_wake(pdev, PCI_D3hot, false);
		pci_enable_wake(pdev, PCI_D3cold, false);
	}
	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	return 0;
}

static int
jme_resume(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct jme_adapter *jme = netdev_priv(netdev);

	jme_clear_pm(jme);
	pci_restore_state(pdev);

	if(jme->flags & JME_FLAG_SSET)
		jme_set_settings(netdev, &jme->old_ecmd);
	else
		jme_reset_phy_processor(jme);

	jme_reset_mac_processor(jme);
	jme_enable_shadow(jme);
	jme_request_irq(jme);
	jme_start_irq(jme);
	netif_device_attach(netdev);

	atomic_inc(&jme->link_changing);

	jme_reset_link(jme);

	return 0;
}

static struct pci_device_id jme_pci_tbl[] = {
	{ PCI_VDEVICE(JMICRON, 0x250) },
	{ }
};

static struct pci_driver jme_driver = {
        .name           = DRV_NAME,
        .id_table       = jme_pci_tbl,
        .probe          = jme_init_one,
        .remove         = __devexit_p(jme_remove_one),
#ifdef CONFIG_PM
        .suspend        = jme_suspend,
        .resume         = jme_resume,
#endif /* CONFIG_PM */
};

static int __init
jme_init_module(void)
{
	printk(KERN_INFO PFX "JMicron JMC250 gigabit ethernet "
	       "driver version %s\n", DRV_VERSION);
	return pci_register_driver(&jme_driver);
}

static void __exit
jme_cleanup_module(void)
{
	pci_unregister_driver(&jme_driver);
}

module_init(jme_init_module);
module_exit(jme_cleanup_module);

MODULE_AUTHOR("Guo-Fu Tseng <cooldavid@cooldavid.org>");
MODULE_DESCRIPTION("JMicron JMC2x0 PCI Express Ethernet driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, jme_pci_tbl);

