/*
 * JMicron JMC2x0 series PCIe Ethernet Linux Device Driver
 *
 * Copyright 2008 JMicron Technology Corporation
 * http://www.jmicron.com/
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
 * Timeline before release:
 * 	Stage 1: Basic Performance / Capbility fine tune.
 * 	-  Implement PCC -- Dynamic adjustment.
 * 	-  Use NAPI instead of rx_tasklet?
 * 		PCC Support Both Packet Counter and Timeout Interrupt for
 * 		receive and transmit complete, does NAPI really needed?
 * 		I'll add NAPI support anyway..
 * 		For CPU busy and heavy network loading system..
 * 	-  Try setting 64bit DMA with pci_set[_consistent]_dma_mask
 * 	   and set netdev feature flag.
 * 	   (Need to modity transmit descriptor filling policy as well)
 *      -  Use pci_map_page instead of pci_map_single for HIGHMEM support
 *
 * 	Stage 2: Error handling.
 * 	-  Wathch dog
 * 	-  Transmit timeout
 *
 * 	Stage 3: Basic offloading support.
 *      -  Implement scatter-gather offloading.
 *         A system page per RX (buffer|descriptor)?
 *	   Handle fraged sk_buff to TX descriptors.
 * 	-  Implement tx/rx ipv6/ip/tcp/udp checksum offloading
 *
 * 	Stage 4: Basic feature support.
 * 	-  Implement Power Managemt related functions.
 * 	-  Implement Jumboframe.
 * 	-  Implement MSI.
 *
 * 	Stage 5: Advanced offloading support.
 * 	-  Implement VLAN offloading.
 * 	-  Implement TCP Segement offloading.
 *
 * 	Stage 6: CPU Load balancing.
 * 	-  Implement MSI-X.
 * 	   Along with multiple RX queue, for CPU load balancing.
 * 	-  Use Multiple TX Queue for Multiple CPU Transmit
 * 	   Simultaneously Without Lock.
 *
 * 	Stage 7:
 * 	-  Cleanup/re-orginize code, performence tuneing(alignment etc...).
 * 	-  Test and Release 1.0
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
#include "jme.h"

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21)
static struct net_device_stats *jme_get_stats(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	return &jme->stats;
}
#endif

static int jme_mdio_read(struct net_device *netdev, int phy, int reg)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int i, val;

        jwrite32(jme, JME_SMI, SMI_OP_REQ |
				 smi_phy_addr(phy) |
				 smi_reg_addr(reg));

	wmb();
        for (i = JME_PHY_TIMEOUT; i > 0; --i) {
                udelay(1);
                if (((val = jread32(jme, JME_SMI)) & SMI_OP_REQ) == 0)
                        break;
        }

        if (i == 0) {
                jeprintk(netdev->name, "phy read timeout : %d\n", reg);
                return (0);
        }

        return ((val & SMI_DATA_MASK) >> SMI_DATA_SHIFT);
}

static void jme_mdio_write(struct net_device *netdev, int phy, int reg, int val)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int i;

        jwrite32(jme, JME_SMI, SMI_OP_WRITE | SMI_OP_REQ |
            ((val << SMI_DATA_SHIFT) & SMI_DATA_MASK) |
            smi_phy_addr(phy) | smi_reg_addr(reg));

	wmb();
        for (i = JME_PHY_TIMEOUT; i > 0; --i)
	{
                udelay(1);
                if (((val = jread32(jme, JME_SMI)) & SMI_OP_REQ) == 0)
                        break;
        }

        if (i == 0)
                jeprintk(netdev->name, "phy write timeout : %d\n", reg);

        return;
}

static void jme_reset_mac_processor(struct jme_adapter *jme)
{
	__u32 val;

	val = jread32(jme, JME_GHC);
	val |= GHC_SWRST;
	jwrite32(jme, JME_GHC, val);
	udelay(2);
	val &= ~GHC_SWRST;
	jwrite32(jme, JME_GHC, val);
	jwrite32(jme, JME_RXMCHT_LO, 0x00000000);
	jwrite32(jme, JME_RXMCHT_HI, 0x00000000);
	jwrite32(jme, JME_WFODP, 0);
	jwrite32(jme, JME_WFOI, 0);
	jwrite32(jme, JME_GPREG0, GPREG0_DEFAULT);
	jwrite32(jme, JME_GPREG1, 0);
}

__always_inline static void jme_clear_pm(struct jme_adapter *jme)
{
	jwrite32(jme, JME_PMCS, 0xFFFF0000);
	pci_set_power_state(jme->pdev, PCI_D0);
}

static int jme_reload_eeprom(struct jme_adapter *jme)
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

__always_inline static void jme_load_macaddr(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	unsigned char macaddr[6];
	__u32 val;

	val = jread32(jme, JME_RXUMA_LO);
	macaddr[0] = (val >>  0) & 0xFF;
	macaddr[1] = (val >>  8) & 0xFF;
	macaddr[2] = (val >> 16) & 0xFF;
	macaddr[3] = (val >> 24) & 0xFF;
	val = jread32(jme, JME_RXUMA_HI);
	macaddr[4] = (val >>  0) & 0xFF;
	macaddr[5] = (val >>  8) & 0xFF;
        memcpy(netdev->dev_addr, macaddr, 6);
}

__always_inline static void jme_start_irq(struct jme_adapter *jme)
{
	/*
	 * Enable Interrupts
	 */
	jwrite32(jme, JME_IENS, INTR_ENABLE);
}

__always_inline static void jme_stop_irq(struct jme_adapter *jme)
{
	/*
	 * Disable Interrupts
	 */
	jwrite32(jme, JME_IENC, INTR_ENABLE);
}


__always_inline static void jme_enable_shadow(struct jme_adapter *jme)
{
	jwrite32(jme,
		 JME_SHBA_LO,
		 ((__u32)jme->shadow_dma & ~((__u32)0x1F)) | SHBA_POSTEN);
}

__always_inline static void jme_disable_shadow(struct jme_adapter *jme)
{
	jwrite32(jme, JME_SHBA_LO, 0x0);
}

static void jme_check_link(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	__u32 phylink, ghc, cnt = JME_AUTONEG_TIMEOUT;
	char linkmsg[32];

	phylink = jread32(jme, JME_PHY_LINK);

        if (phylink & PHY_LINK_UP) {
		/*
		 * Keep polling for autoneg complete
		 */
		while(!(phylink & PHY_LINK_AUTONEG_COMPLETE) && --cnt > 0) {
			mdelay(1);
			phylink = jread32(jme, JME_PHY_LINK);
		}

		if(!cnt)
			jeprintk(netdev->name, "Waiting autoneg timeout.\n");

		switch(phylink & PHY_LINK_SPEED_MASK) {
			case PHY_LINK_SPEED_10M:
				ghc = GHC_SPEED_10M;
				strcpy(linkmsg, "10 Mbps, ");
				break;
			case PHY_LINK_SPEED_100M:
				ghc = GHC_SPEED_100M;
				strcpy(linkmsg, "100 Mbps, ");
				break;
			case PHY_LINK_SPEED_1000M:
				ghc = GHC_SPEED_1000M;
				strcpy(linkmsg, "1000 Mbps, ");
				break;
			default:
				ghc = 0;
				break;
		}
                ghc |= (phylink & PHY_LINK_DUPLEX) ? GHC_DPX : 0;
		jwrite32(jme, JME_GHC, ghc);
		strcat(linkmsg, (phylink &PHY_LINK_DUPLEX) ?
					"Full-Duplex" :
					"Half-Duplex");

		if(phylink & PHY_LINK_DUPLEX)
			jwrite32(jme, JME_TXMCS, TXMCS_DEFAULT);
		else
			jwrite32(jme, JME_TXMCS, TXMCS_DEFAULT |
						   TXMCS_BACKOFF |
						   TXMCS_CARRIERSENSE |
						   TXMCS_COLLISION);

		jprintk(netdev->name, "Link is up at %s.\n", linkmsg);
                netif_carrier_on(netdev);
	}
        else {
		jprintk(netdev->name, "Link is down.\n");
                netif_carrier_off(netdev);
	}
}

static void jme_link_change_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;
	jme_check_link(jme->dev);
}

static void jme_set_new_txdesc(struct jme_adapter *jme,
			       int i, struct sk_buff *skb)
{
	struct jme_ring *txring = jme->txring;
	register struct TxDesc* txdesc = txring->desc;
	struct jme_buffer_info *txbi = txring->bufinf;
	dma_addr_t dmaaddr;

	txdesc += i;
	txbi += i;

	dmaaddr = pci_map_single(jme->pdev,
				 skb->data,
				 skb->len,
				 PCI_DMA_TODEVICE);

	pci_dma_sync_single_for_device(jme->pdev,
				       dmaaddr,
				       skb->len,
				       PCI_DMA_TODEVICE);
	
	txdesc->dw[0] = 0;
	txdesc->dw[1] = 0;
	txdesc->dw[2] = 0;
	txdesc->desc1.bufaddr = cpu_to_le32(dmaaddr);
	txdesc->desc1.datalen = cpu_to_le16(skb->len);
	txdesc->desc1.pktsize = cpu_to_le16(skb->len);
	/*
	 * Set OWN bit at final.
	 * When kernel transmit faster than NIC last packet sent,
	 * and NIC trying to send this descriptor before we tell
	 * it to start sending this TX queue.
	 * Other fields are already filled correctly.
	 */
	wmb();
	txdesc->desc1.flags = TXFLAG_OWN | TXFLAG_INT;
	txbi->skb = skb;
	txbi->mapping = dmaaddr;
	txbi->len = skb->len;

#ifdef TX_QUEUE_DEBUG
	dprintk(jme->dev->name, "TX Ring Buf Address(%08x,%08x,%d).\n",
		dmaaddr,
		(txdesc->all[12] <<  0) |
		(txdesc->all[13] <<  8) |
		(txdesc->all[14] << 16) |
		(txdesc->all[15] << 24),
		(txdesc->all[4]  <<  0) |
		(txdesc->all[5]  <<  8));
#endif

}


static int jme_setup_tx_resources(struct jme_adapter *jme)
{
	struct jme_ring *txring = &(jme->txring[0]);

	txring->alloc = dma_alloc_coherent(&(jme->pdev->dev),
					   TX_RING_ALLOC_SIZE,
					   &(txring->dmaalloc), 
					   GFP_KERNEL);
	if(!txring->alloc) {
		txring->desc = NULL;
		txring->dmaalloc = 0;
		txring->dma = 0;
		return -ENOMEM;
	}

	/*
	 * 16 Bytes align
	 */
	txring->desc		= (void*)ALIGN((unsigned long)(txring->alloc), RING_DESC_ALIGN);
	txring->dma		= ALIGN(txring->dmaalloc, RING_DESC_ALIGN);
	txring->next_to_use	= 0;
	txring->next_to_clean	= 0;

#ifdef TX_QUEUE_DEBUG
	dprintk(jme->dev->name, "TX Ring Base Address(%08x,%08x).\n",
		(__u32)txring->desc,
		txring->dma);
#endif

	/*
	 * Initiallize Transmit Descriptors
	 */
	memset(txring->alloc, 0, TX_RING_ALLOC_SIZE);
	memset(txring->bufinf, 0, sizeof(struct jme_buffer_info) * RING_DESC_NR);

	return 0;
}

static void jme_free_tx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *txring = &(jme->txring[0]);
	struct jme_buffer_info *txbi = txring->bufinf;

	if(txring->alloc) {
		for(i=0;i<RING_DESC_NR;++i) {
			txbi = txring->bufinf + i;
			if(txbi->skb) {
				dev_kfree_skb(txbi->skb);
				txbi->skb = NULL;
				txbi->mapping = 0;
				txbi->len = 0;
			}
		}

		dma_free_coherent(&(jme->pdev->dev),
				  TX_RING_ALLOC_SIZE,
				  txring->alloc,
				  txring->dmaalloc);
		txring->alloc    = NULL;
		txring->desc     = NULL;
		txring->dmaalloc = 0;
		txring->dma      = 0;
	}
	txring->next_to_use   = 0;
	txring->next_to_clean = 0;

}

__always_inline static void jme_enable_tx_engine(struct jme_adapter *jme)
{
	__u8 mrrs;

	/*
	 * Select Queue 0
	 */
	jwrite32(jme, JME_TXCS, TXCS_DEFAULT | TXCS_SELECT_QUEUE0);

	/*
	 * Setup TX Queue 0 DMA Bass Address
	 */
	jwrite32(jme, JME_TXDBA, jme->txring[0].dma);
	jwrite32(jme, JME_TXNDA, jme->txring[0].dma);

	/*
	 * Setup TX Descptor Count
	 */
	jwrite32(jme, JME_TXQDC, RING_DESC_NR);

	/*
	 * Get Max Read Req Size from PCI Config Space
	 */
	pci_read_config_byte(jme->pdev, PCI_CONF_DCSR_MRRS, &mrrs);
	switch(mrrs) {
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
	 * Enable TX Engine
	 */
	wmb();
	jwrite32(jme, JME_TXCS, jme->reg_txcs |
				TXCS_SELECT_QUEUE0 |
				TXCS_ENABLE);

}

__always_inline static void jme_disable_tx_engine(struct jme_adapter *jme)
{
	int i;
	__u32 val;

	/*
	 * Disable TX Engine
	 */
	jwrite32(jme, JME_TXCS, jme->reg_txcs);

	val = jread32(jme, JME_TXCS);
	for(i = JME_TX_DISABLE_TIMEOUT ; (val & TXCS_ENABLE) && i > 0 ; --i)
	{
		udelay(1);
		val = jread32(jme, JME_TXCS);
	}

	if(!i)
		jeprintk(jme->dev->name, "Disable TX engine timeout.\n");


}

static void jme_set_clean_rxdesc(struct jme_adapter *jme, int i)
{
	struct jme_ring *rxring = jme->rxring;
	register struct RxDesc* rxdesc = rxring->desc;
	struct jme_buffer_info *rxbi = rxring->bufinf;
	rxdesc += i;
	rxbi += i;

	rxdesc->dw[0] = 0;
	rxdesc->dw[1] = 0;
	rxdesc->desc1.bufaddrh = cpu_to_le32(((__u64)rxbi->mapping) >> 32);
	rxdesc->desc1.bufaddrl = cpu_to_le32(rxbi->mapping);
	rxdesc->desc1.datalen = cpu_to_le16(RX_BUF_SIZE);
	wmb();
	rxdesc->desc1.flags = RXFLAG_OWN | RXFLAG_INT;

#ifdef RX_QUEUE_DEBUG
	dprintk(jme->dev->name, "RX Ring Buf Address(%08x,%08x,%d).\n",
		rxbi->mapping,
		(rxdesc->all[12] <<  0) |
		(rxdesc->all[13] <<  8) |
		(rxdesc->all[14] << 16) |
		(rxdesc->all[15] << 24),
		(rxdesc->all[4]  <<  0) |
		(rxdesc->all[5]  <<  8));
#endif

}

static int jme_make_new_rx_buf(struct jme_adapter *jme, int i)
{
	struct jme_ring *rxring = &(jme->rxring[0]);
	struct jme_buffer_info *rxbi = rxring->bufinf;
	unsigned long offset;
	struct sk_buff* skb;

	skb = netdev_alloc_skb(jme->dev, RX_BUF_ALLOC_SIZE);
	if(unlikely(!skb))
		return -ENOMEM;
	if(unlikely(skb_shinfo(skb)->nr_frags)) {
		dprintk(jme->dev->name, "Allocated skb fragged(%d).\n", skb_shinfo(skb)->nr_frags);
		dev_kfree_skb(skb);
		return -ENOMEM;
	}


	if(unlikely(
		offset = 
		(unsigned long)(skb->data)
		& (unsigned long)(RX_BUF_DMA_ALIGN - 1))) {
		skb_reserve(skb, RX_BUF_DMA_ALIGN - offset);
	}

	rxbi += i;
	rxbi->skb = skb;
	rxbi->mapping = pci_map_single(jme->pdev,
				       skb->data,
				       RX_BUF_SIZE,
				       PCI_DMA_FROMDEVICE);

	return 0;
}

static void jme_free_rx_buf(struct jme_adapter *jme, int i)
{
	struct jme_ring *rxring = &(jme->rxring[0]);
	struct jme_buffer_info *rxbi = rxring->bufinf;
	rxbi += i;

	if(rxbi->skb) {
		pci_unmap_single(jme->pdev,
				 rxbi->mapping,
				 RX_BUF_SIZE,
				 PCI_DMA_FROMDEVICE);
		dev_kfree_skb(rxbi->skb);
		rxbi->skb = NULL;
		rxbi->mapping = 0;
	}
}

static int jme_setup_rx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *rxring = &(jme->rxring[0]);

	rxring->alloc = dma_alloc_coherent(&(jme->pdev->dev),
					   RX_RING_ALLOC_SIZE,
					   &(rxring->dmaalloc), 
					   GFP_KERNEL);
	if(!rxring->alloc) {
		rxring->desc = NULL;
		rxring->dmaalloc = 0;
		rxring->dma = 0;
		return -ENOMEM;
	}

	/*
	 * 16 Bytes align
	 */
	rxring->desc		= (void*)ALIGN((unsigned long)(rxring->alloc), RING_DESC_ALIGN);
	rxring->dma		= ALIGN(rxring->dmaalloc, RING_DESC_ALIGN);
	rxring->next_to_use	= 0;
	rxring->next_to_clean	= 0;

#ifdef RX_QUEUE_DEBUG
	dprintk(jme->dev->name, "RX Ring Base Address(%08x,%08x).\n",
		(__u32)rxring->desc,
		rxring->dma);
#endif

	/*
	 * Initiallize Receive Descriptors
	 */
	for(i = 0 ; i < RING_DESC_NR ; ++i) {
		if(unlikely(jme_make_new_rx_buf(jme, i)))
			break;

		jme_set_clean_rxdesc(jme, i);
	}

	/*
	 * Cleanup allocated memories when error
	 */
	if(i != RING_DESC_NR) {
		for(--i ; i >= 0 ; --i)
			jme_free_rx_buf(jme, i);

		dma_free_coherent(&(jme->pdev->dev),
				  RX_RING_ALLOC_SIZE,
				  rxring->alloc,
				  rxring->dmaalloc);
		rxring->alloc    = NULL;
		rxring->desc     = NULL;
		rxring->dmaalloc = 0;
		rxring->dma      = 0;
		return -ENOMEM;
	}

	return 0;
}

static void jme_free_rx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *rxring = &(jme->rxring[0]);

	if(rxring->alloc) {
		for(i = 0 ; i < RING_DESC_NR ; ++i)
			jme_free_rx_buf(jme, i);

		dma_free_coherent(&(jme->pdev->dev),
				  RX_RING_ALLOC_SIZE,
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

__always_inline static void jme_enable_rx_engine(struct jme_adapter *jme)
{
	/*
	 * Setup RX DMA Bass Address
	 */
	jwrite32(jme, JME_RXDBA, jme->rxring[0].dma);
	jwrite32(jme, JME_RXNDA, jme->rxring[0].dma);

	/*
	 * Setup RX Descptor Count
	 */
	jwrite32(jme, JME_RXQDC, RING_DESC_NR);

	/* 
	 * Setup Unicast Filter
	 */
	jme_set_multi(jme->dev);

	/*
	 * Enable RX Engine
	 */

	wmb();
	jwrite32(jme, JME_RXCS, RXCS_DEFAULT |
				RXCS_QUEUESEL_Q0 |
				RXCS_ENABLE |
				RXCS_QST);
}

__always_inline static void jme_restart_rx_engine(struct jme_adapter *jme)
{
	/*
	 * Enable RX Engine
	 */
	jwrite32(jme, JME_RXCS, RXCS_DEFAULT |
				RXCS_QUEUESEL_Q0 |
				RXCS_ENABLE |
				RXCS_QST);
}


__always_inline static void jme_disable_rx_engine(struct jme_adapter *jme)
{
	int i;
	__u32 val;

	/*
	 * Disable RX Engine
	 */
	val = jread32(jme, JME_RXCS);
	val &= ~RXCS_ENABLE;
	jwrite32(jme, JME_RXCS, val);

	val = jread32(jme, JME_RXCS);
	for(i = JME_RX_DISABLE_TIMEOUT ; (val & RXCS_ENABLE) && i > 0 ; --i)
	{
		udelay(1);
		val = jread32(jme, JME_RXCS);
	}

	if(!i)
		jeprintk(jme->dev->name, "Disable RX engine timeout.\n");

}

static void jme_tx_clean_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;
	struct jme_ring *txring = &(jme->txring[0]);
	struct TxDesc *txdesc = txring->desc;
	struct jme_buffer_info *txbi = txring->bufinf, *ctxbi;
	struct sk_buff *skb;
	int i, end;

#ifdef TX_TASKLET_DEBUG
	dprintk(jme->dev->name, "into tasklet\n");
#endif

	end = txring->next_to_use;
	for(i = txring->next_to_clean ; i != end ; ) {
		ctxbi = txbi + i;
		skb = ctxbi->skb;
		if(skb && !(txdesc[i].desc1.flags & TXFLAG_OWN)) {

#ifdef TX_TASKLET_DEBUG
			dprintk(jme->dev->name, "cleaning %d\n", i);
#endif

			pci_unmap_single(jme->pdev,
					 ctxbi->mapping,
					 skb->len,
					 PCI_DMA_TODEVICE);

			dev_kfree_skb(skb);
			prefetch(txbi + i + 1);
			prefetch(txdesc + i + 1);
			ctxbi->skb = NULL;
			ctxbi->mapping = 0;
			ctxbi->len = skb->len;
		}
		else {
			break;
		}

		if(unlikely(++i == RING_DESC_NR))
			i = 0;
	}
	txring->next_to_clean = i;

}

static void jme_process_receive(struct jme_adapter *jme)
{
	struct net_device *netdev = jme->dev;
	struct jme_ring *rxring = &(jme->rxring[0]);
	struct RxDesc *rxdesc = rxring->desc;
	struct jme_buffer_info *rxbi;
	struct sk_buff *skb;
	dma_addr_t buf_dma;
	int i, j, start, cnt, ccnt;
	unsigned int framesize, desccnt;

	/*
	 * Assume that one descriptor per frame,
	 * Should be fixed in the future
	 * (or not? If buffer already large enough to store entire packet.)
	 */

	spin_lock(&jme->recv_lock);
	i = start = rxring->next_to_clean;
	/*
	 * Decide how many descriptors need to be processed
	 * In the worst cast we'll have to process entire queue
	 */
	for(cnt = 0 ; cnt < RING_DESC_NR ; )
	{
		rxdesc = (struct RxDesc*)(rxring->desc) + i;
		if((rxdesc->descwb.flags & RXWBFLAG_OWN) ||
		   !(rxdesc->descwb.desccnt & RXWBDCNT_WBCPL)
		) {
			rxring->next_to_clean = i;
			break;
		}

		desccnt = rxdesc->descwb.desccnt & RXWBDCNT_DCNT;

		if(unlikely((cnt += desccnt) >= RING_DESC_NR)) {
			cnt -= desccnt;
			break;
		}

		if(unlikely((i += desccnt) >= RING_DESC_NR))
			i -= RING_DESC_NR;
	}
	spin_unlock(&jme->recv_lock);

	/*
	 * Process descriptors independently accross cpu
	 * 	--- save for multiple cpu handling
	 */
	for( i = start ; cnt-- ; ) {
		rxdesc = (struct RxDesc*)(rxring->desc) + i;
		desccnt = rxdesc->descwb.desccnt & RXWBDCNT_DCNT;
		rxbi = rxring->bufinf + i;
		if(unlikely(
			/*
			 * Drop and record error packet
			 */
			rxdesc->descwb.errstat & RXWBERR_ALLERR ||
			desccnt > 1)) {
			if(rxdesc->descwb.errstat & RXWBERR_OVERUN)
				++(NET_STAT.rx_fifo_errors);
			else if(rxdesc->descwb.errstat & RXWBERR_CRCERR)
				++(NET_STAT.rx_frame_errors);
			else {
				++(NET_STAT.rx_errors);
#ifdef RX_ERR_DEBUG
				dprintk(netdev->name, "err: %02x\n", rxdesc->descwb.errstat);
#endif
			}

			if(desccnt > 1)
				cnt -= desccnt-1;

			for(j=i,ccnt=desccnt;ccnt--;) {
				jme_set_clean_rxdesc(jme, j);

				if(unlikely(++j == RING_DESC_NR))
					j = 0;
			}
		}
		else {
			/*
			* Pass received packet to kernel
			*/
			skb = rxbi->skb;
			buf_dma = rxbi->mapping;
			pci_dma_sync_single_for_cpu(jme->pdev,
						    buf_dma,
						    RX_BUF_SIZE,
						    PCI_DMA_FROMDEVICE);

			if(unlikely(jme_make_new_rx_buf(jme, i))) {
				pci_dma_sync_single_for_device(jme->pdev,
							       buf_dma,
							       RX_BUF_SIZE,
							       PCI_DMA_FROMDEVICE);
				++(NET_STAT.rx_dropped);
			}
			else {
				framesize = le16_to_cpu(rxdesc->descwb.framesize);

				skb_put(skb, framesize);
				skb->protocol = eth_type_trans(skb, netdev);

				netif_rx(skb);

				if(le16_to_cpu(rxdesc->descwb.flags) & RXWBFLAG_DEST_MUL)
					++(NET_STAT.multicast);

				netdev->last_rx = jiffies;
				NET_STAT.rx_bytes += framesize;
				++(NET_STAT.rx_packets);
			}

			jme_set_clean_rxdesc(jme, i);

#ifdef RX_PKT_DEBUG
			dprintk(netdev->name, "DESCCNT: %u, FSIZE: %u, ADDRH: %08x, "
				"ADDRL: %08x, FLAGS: %04x, STAT: %02x, "
				"DST:%02x:%02x:%02x:%02x:%02x:%02x\n",
				desccnt,
				framesize,
				le32_to_cpu(rxdesc->dw[2]),
				le32_to_cpu(rxdesc->dw[3]),
				le16_to_cpu(rxdesc->descwb.flags),
				rxdesc->descwb.errstat,
				rxbuf[0], rxbuf[1], rxbuf[2],
				rxbuf[3], rxbuf[4], rxbuf[5]);
#endif


		}


		if(unlikely((i+=desccnt) >= RING_DESC_NR))
			i -= RING_DESC_NR;

	}

}

static void jme_rx_clean_tasklet(unsigned long arg)
{
	struct jme_adapter *jme = (struct jme_adapter*)arg;
	
	jme_process_receive(jme);
	if(jme->flags & JME_FLAG_RXQ0_EMPTY) {
		jme_restart_rx_engine(jme);
		jme->flags &= ~JME_FLAG_RXQ0_EMPTY;
	}

}

static irqreturn_t jme_intr(int irq, void *dev_id)
{
        struct net_device *netdev = dev_id;
        struct jme_adapter *jme = netdev_priv(netdev);
	irqreturn_t rc = IRQ_HANDLED;
	__u32 intrstat;

#if USE_IEVE_SHADOW
	pci_dma_sync_single_for_cpu(jme->pdev,
				    jme->shadow_dma,
				    sizeof(__u32) * SHADOW_REG_NR,
				    PCI_DMA_FROMDEVICE);
	intrstat = jme->shadow_regs[SHADOW_IEVE];
	jme->shadow_regs[SHADOW_IEVE] = 0;
#else
	intrstat = jread32(jme, JME_IEVE);
#endif


#ifdef INTERRUPT_DEBUG
	dprintk(netdev->name, "Interrupt received(%08x) @ %lu.\n", intrstat, jiffies);
#endif

	/*
	 * Check if it's really an interrupt for us
	 * and if the device still exist
	 */
        if((intrstat & INTR_ENABLE) == 0) {
		rc = IRQ_NONE;
		goto out;
	}
	if(unlikely(intrstat == ~((typeof(intrstat))0))) {
                rc = IRQ_NONE;
		goto out;
	}


	if(intrstat & INTR_LINKCH) {
		/*
		 * Process Link status change event
		 */
		tasklet_schedule(&jme->linkch_task);
	}

	if(intrstat & INTR_RX0EMP) {
		/*
		 * Process event
		 */
		jme->flags |= JME_FLAG_RXQ0_EMPTY;

                jeprintk(netdev->name, "Ranout of Receive Queue 0.\n");
	}

	if(intrstat & INTR_RX0) {
		/*
		 * Process event
		 */
		tasklet_schedule(&jme->rxclean_task);

#ifdef RX_PKT_DEBUG
                dprintk(netdev->name, "Received From Queue 0.\n");
#endif
	}

	if(intrstat & INTR_TX0) {
		/*
		 * Process event
		 */
		tasklet_schedule(&jme->txclean_task);

#ifdef TX_PKT_DEBUG
                dprintk(netdev->name, "Queue 0 transmit complete.\n");
#endif
	}

        if((intrstat & ~INTR_ENABLE) != 0) {
#ifdef INTERRUPT_DEBUG
		dprintk(netdev->name, "Some interrupt event not handled: %08x\n", intrstat & ~INTR_ENABLE);
#endif
	}

	/*
	 * Deassert interrupts
	 */
	jwrite32(jme, JME_IEVE, intrstat & INTR_ENABLE);

out:
        return rc;
}

static int jme_open(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int rc;

	rc = request_irq(jme->pdev->irq, jme_intr,
			 IRQF_SHARED, netdev->name, netdev);
	if(rc) {
		printk(KERN_ERR PFX "Requesting IRQ error.\n");
		goto err_out;
	}

	rc = jme_setup_rx_resources(jme);
	if(rc) {
		printk(KERN_ERR PFX "Allocating resources for RX error.\n");
		goto err_out_free_irq;
	}


	rc = jme_setup_tx_resources(jme);
	if(rc) {
		printk(KERN_ERR PFX "Allocating resources for TX error.\n");
		goto err_out_free_rx_resources;
	}

	jme_reset_mac_processor(jme);
	jme_check_link(netdev);
	jme_enable_shadow(jme);
	jme_start_irq(jme);
	jme_enable_rx_engine(jme);
	jme_enable_tx_engine(jme);
	netif_start_queue(netdev);

	return 0;

err_out_free_rx_resources:
	jme_free_rx_resources(jme);
err_out_free_irq:
	free_irq(jme->pdev->irq, jme->dev);
err_out:
	netif_stop_queue(netdev);
	netif_carrier_off(netdev);
	return rc;
}

static int jme_close(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	jme_stop_irq(jme);
	jme_disable_shadow(jme);
	free_irq(jme->pdev->irq, jme->dev);

	tasklet_kill(&jme->linkch_task);
	tasklet_kill(&jme->txclean_task);
	tasklet_kill(&jme->rxclean_task);
	jme_disable_rx_engine(jme);
	jme_disable_tx_engine(jme);
	jme_free_rx_resources(jme);
	jme_free_tx_resources(jme);

	return 0;
}

static int jme_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
        struct jme_adapter *jme = netdev_priv(netdev);
	struct jme_ring *txring = &(jme->txring[0]);
	struct TxDesc *txdesc = txring->desc;
	int idx;

	/*
	 * Check if transmit queue is already full
	 * and take one descriptor to use
	 */
	spin_lock(&jme->xmit_lock);
	idx = txring->next_to_use;
	if(unlikely(txdesc[idx].desc1.flags & TXFLAG_OWN)) {
		spin_unlock(&jme->xmit_lock);
#ifdef TX_BUSY_DEBUG
		dprintk(netdev->name, "TX Device busy.\n");
#endif
		return NETDEV_TX_BUSY;
	}
	if(unlikely(++(txring->next_to_use) == RING_DESC_NR))
		txring->next_to_use = 0;
	spin_unlock(&jme->xmit_lock);

	/*
	 * Fill up TX descriptors
	 */
	jme_set_new_txdesc(jme, idx, skb);

	/*
	 * Tell MAC HW to send
	 */
	jwrite32(jme, JME_TXCS, jme->reg_txcs |
				TXCS_SELECT_QUEUE0 |
				TXCS_QUEUE0S |
				TXCS_ENABLE);

#ifdef TX_PKT_DEBUG
	dprintk(netdev->name, "Asked to transmit.\n");
#endif

	NET_STAT.tx_bytes += skb->len;
	++(NET_STAT.tx_packets);
	netdev->trans_start = jiffies;

        return NETDEV_TX_OK;
}

static int jme_set_macaddr(struct net_device *netdev, void *p)
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

static void jme_set_multi(struct net_device *netdev)
{
        struct jme_adapter *jme = netdev_priv(netdev);
	u32 mc_hash[2] = {};
	__u32 val;
	int i;


	spin_lock(&jme->macaddr_lock);
	val = RXMCS_BRDFRAME | RXMCS_UNIFRAME;

        if (netdev->flags & IFF_PROMISC)
		val |= RXMCS_ALLFRAME;
        else if (netdev->flags & IFF_ALLMULTI)
		val |= RXMCS_ALLMULFRAME;
	else if(netdev->flags & IFF_MULTICAST) {
                struct dev_mc_list *mclist;
                int bit_nr;

		val |= RXMCS_MULFRAME | RXMCS_MULFILTERED;
                for (i = 0, mclist = netdev->mc_list;
                     mclist && i < netdev->mc_count;
                     ++i, mclist = mclist->next) {
                        bit_nr = ether_crc(ETH_ALEN, mclist->dmi_addr) & 0x3F;
                        mc_hash[bit_nr >> 5] |= 1 << (bit_nr & 0x1F);
#ifdef SET_MULTI_DEBUG
			dprintk(netdev->name, "Adding MCAddr: "
				"%02x:%02x:%02x:%02x:%02x:%02x (%d)\n",
				mclist->dmi_addr[0],
				mclist->dmi_addr[1],
				mclist->dmi_addr[2],
				mclist->dmi_addr[3],
				mclist->dmi_addr[4],
				mclist->dmi_addr[5],
				bit_nr);
#endif
                }

		jwrite32(jme, JME_RXMCHT_LO, mc_hash[0]);
		jwrite32(jme, JME_RXMCHT_HI, mc_hash[1]);
	}


	wmb();
	jwrite32(jme, JME_RXMCS, val);
	spin_unlock(&jme->macaddr_lock);

#ifdef SET_MULTI_DEBUG
	dprintk(netdev->name, "RX Mode changed: %08x\n", val);
#endif
}

static int jme_change_mtu(struct net_device *dev, int new_mtu)
{
	/*
	 * Do not support MTU change for now.
	 */
	return -EINVAL;
}

static void jme_get_drvinfo(struct net_device *netdev,
                             struct ethtool_drvinfo *info)
{
        struct jme_adapter *jme = netdev_priv(netdev);

        strcpy(info->driver, DRV_NAME);
        strcpy(info->version, DRV_VERSION);
        strcpy(info->bus_info, pci_name(jme->pdev));
}

static int jme_get_settings(struct net_device *netdev,
			     struct ethtool_cmd *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int rc;
	spin_lock(&jme->phy_lock);
	rc = mii_ethtool_gset(&(jme->mii_if), ecmd);
	spin_unlock(&jme->phy_lock);
	return rc;
}

static int jme_set_settings(struct net_device *netdev,
			     struct ethtool_cmd *ecmd)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int rc;
	spin_lock(&jme->phy_lock);
	rc = mii_ethtool_sset(&(jme->mii_if), ecmd);
	spin_unlock(&jme->phy_lock);
	return rc;
}

static u32 jme_get_link(struct net_device *netdev) {
	struct jme_adapter *jme = netdev_priv(netdev);
	return jread32(jme, JME_PHY_LINK) & PHY_LINK_UP;
}

static const struct ethtool_ops jme_ethtool_ops = {
        .get_drvinfo            = jme_get_drvinfo,
	.get_settings		= jme_get_settings,
	.set_settings		= jme_set_settings,
	.get_link		= jme_get_link,
};

static int __devinit jme_init_one(struct pci_dev *pdev,
                                     const struct pci_device_id *ent)
{
	int rc = 0;
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
	netdev = alloc_etherdev(sizeof(struct jme_adapter));
	if(!netdev) {
		rc = -ENOMEM;
		goto err_out_release_regions;
	}
	netdev->open			= jme_open;
	netdev->stop			= jme_close;
	netdev->hard_start_xmit		= jme_start_xmit;
	netdev->irq			= pdev->irq;
	netdev->set_mac_address		= jme_set_macaddr;
	netdev->set_multicast_list	= jme_set_multi;
	netdev->change_mtu		= jme_change_mtu;
	netdev->ethtool_ops		= &jme_ethtool_ops;

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

	/*
	 * init adapter info
	 */
	jme = netdev_priv(netdev);
	jme->pdev = pdev;
	jme->dev = netdev;
	jme->regs = ioremap(pci_resource_start(pdev, 0),
			     pci_resource_len(pdev, 0));
	if (!(jme->regs)) {
		rc = -ENOMEM;
		goto err_out_free_netdev;
	}
	jme->shadow_regs = pci_alloc_consistent(pdev,
					        sizeof(__u32) * SHADOW_REG_NR,
					        &(jme->shadow_dma));
	if (!(jme->shadow_regs)) {
		rc = -ENOMEM;
		goto err_out_unmap;
	}

	spin_lock_init(&jme->xmit_lock);
	spin_lock_init(&jme->recv_lock);
	spin_lock_init(&jme->macaddr_lock);
	spin_lock_init(&jme->phy_lock);
	tasklet_init(&jme->linkch_task,
		     &jme_link_change_tasklet,
		     (unsigned long) jme);
	tasklet_init(&jme->txclean_task,
		     &jme_tx_clean_tasklet,
		     (unsigned long) jme);
	tasklet_init(&jme->rxclean_task,
		     &jme_rx_clean_tasklet,
		     (unsigned long) jme);
	jme->mii_if.dev = netdev;
	jme->mii_if.phy_id = 1;
	jme->mii_if.supports_gmii = 1;
	jme->mii_if.mdio_read = jme_mdio_read;
	jme->mii_if.mdio_write = jme_mdio_write;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,21)
	netdev->get_stats = &(jme_get_stats);
#endif

	/*
	 * Reset MAC processor and reload EEPROM for MAC Address
	 */
	jme_clear_pm(jme);
	jme_reset_mac_processor(jme);
	rc = jme_reload_eeprom(jme);
	if(rc) {
		printk(KERN_ERR PFX "Rload eeprom for reading MAC Address error.\n");
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
		"JMC250 gigabit eth at %llx, %02x:%02x:%02x:%02x:%02x:%02x, IRQ %d\n",
	        (unsigned long long) pci_resource_start(pdev, 0),
	        netdev->dev_addr[0],
	        netdev->dev_addr[1],
	        netdev->dev_addr[2],
	        netdev->dev_addr[3],
	        netdev->dev_addr[4],
	        netdev->dev_addr[5],
	        pdev->irq);

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

static void __devexit jme_remove_one(struct pci_dev *pdev)
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

static struct pci_device_id jme_pci_tbl[] = {
	{ PCI_VDEVICE(JMICRON, 0x250) },
	{ }
};

static struct pci_driver jme_driver = {
        .name           = DRV_NAME,
        .id_table       = jme_pci_tbl,
        .probe          = jme_init_one,
        .remove         = __devexit_p(jme_remove_one),
#if 0
#ifdef CONFIG_PM
        .suspend        = jme_suspend,
        .resume         = jme_resume,
#endif /* CONFIG_PM */
#endif
};

static int __init jme_init_module(void)
{
	printk(KERN_INFO PFX "JMicron JMC250 gigabit ethernet "
	       "driver version %s\n", DRV_VERSION);
	return pci_register_driver(&jme_driver);
}

static void __exit jme_cleanup_module(void)
{
	pci_unregister_driver(&jme_driver);
}

module_init(jme_init_module);
module_exit(jme_cleanup_module);

MODULE_AUTHOR("David Tseng <cooldavid@cooldavid.org>");
MODULE_DESCRIPTION("JMicron JMC2x0 PCI Express Ethernet driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, jme_pci_tbl);


