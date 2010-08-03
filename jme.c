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
 * TODO before release:
 * 	1. Use sk_buff for dma buffer with pci_map_single,
 * 	   and handle scattered sk_buffs (Reduce memory copy)
 * 	2. Try setting 64bit DMA with pci_set[_consistent]_dma_mask
 * 	   and set netdev feature flag.
 * 	3. Implement Power Managemt related functions.
 * 	4. Implement checksum offloading, VLAN offloading,
 * 	   TCP Segement offloading.
 * 	5. Implement Jumboframe.
 * 	6. Implement NAPI option for user.
 * 	7. Implement MSI / MSI-X.
 * 	8. Implement PCC.
 * 	9. Implement QoS according to "priority" attribute in sk_buff
 * 	   with 8 TX priority queue provided by hardware.
 * 	10.Cleanup/re-orginize code, performence tuneing(alignment etc...).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/crc32.h>
#include "jme.h"

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
                dprintk("phy read timeout : %d\n", reg);
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
                dprintk("phy write timeout : %d\n", reg);

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
	jwrite32(jme, JME_RXMCHT, 0x00000000);
	jwrite32(jme, JME_RXMCHT+4, 0x00000000);
	jwrite32(jme, JME_WFODP, 0);
	jwrite32(jme, JME_WFOI, 0);
}

__always_inline static void jme_clear_pm(struct jme_adapter *jme)
{
	jwrite32(jme, JME_PMCS, 0xFFFF0000);
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
			dprintk("eeprom reload timeout\n");
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

	val = jread32(jme, JME_RXUMA);
	macaddr[0] = (val >>  0) & 0xFF;
	macaddr[1] = (val >>  8) & 0xFF;
	macaddr[2] = (val >> 16) & 0xFF;
	macaddr[3] = (val >> 24) & 0xFF;
	val = jread32(jme, JME_RXUMA+4);
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

__always_inline static void jme_check_link(struct net_device *netdev)
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
			printk(KERN_ERR "Waiting autoneg timeout.\n");

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

		jprintk("Link is up at %s.\n", linkmsg);
                netif_carrier_on(netdev);
	}
        else {
		jprintk("Link is down.\n");
                netif_carrier_off(netdev);
	}
}

__always_inline static void jme_set_new_txdesc(struct jme_adapter *jme,
						int i, int framesize)
{
	struct jme_ring *txring = jme->txring;
	struct TxDesc* txdesc = txring->desc;
	
	memset(txdesc + i, 0, TX_DESC_SIZE);
	txdesc[i].desc1.bufaddr = cpu_to_le32(ALIGN(txring->buf_dma[i], 8));
	txdesc[i].desc1.datalen = cpu_to_le16(TX_BUF_SIZE);
	txdesc[i].desc1.pktsize = cpu_to_le16(framesize);
	/*
	 * Set OWN bit at final.
	 * When kernel transmit faster than NIC last packet sent,
	 * and NIC tring to send this descriptor before we tell
	 * it to start sending this TX queue.
	 * Other fields are already filled correctly.
	 */
	wmb();
	txdesc[i].desc1.flags = TXFLAG_OWN | TXFLAG_INT;

	dprintk("TX Ring Buf Address(%08x,%08x,%d).\n",
		txring->buf_dma[i],
		(txdesc[i].all[12] <<  0) |
		(txdesc[i].all[13] <<  8) |
		(txdesc[i].all[14] << 16) |
		(txdesc[i].all[15] << 24),
		(txdesc[i].all[4]  <<  0) |
		(txdesc[i].all[5]  <<  8));

}


__always_inline static int jme_setup_tx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *txring = &(jme->txring[0]);

	txring->alloc = dma_alloc_coherent(&(jme->pdev->dev),
					   TX_RING_ALLOC_SIZE,
					   &(txring->dmaalloc), 
					   GFP_KERNEL);
	if(!txring->alloc)
		return -ENOMEM;

	/*
	 * 16 Bytes align
	 */
	txring->desc		= (void*)ALIGN((unsigned long)(txring->alloc), 16);
	txring->dma		= ALIGN(txring->dmaalloc, 16);
	txring->next_to_use	= 0;
	txring->next_to_clean	= 0;

	dprintk("TX Ring Base Address(%08x,%08x).\n",
		(__u32)txring->desc,
		txring->dma);

	/*
	 * Initiallize Transmit Descriptors
	 */
	memset(txring->alloc, 0, TX_RING_ALLOC_SIZE);
	for(i = 0 ; i < RING_DESC_NR ; ++i) {
		txring->buf_virt[i] = dma_alloc_coherent(&(jme->pdev->dev),
							 TX_BUF_ALLOC_SIZE,
							 &(txring->buf_dma[i]),
							 GFP_KERNEL);
		if(!txring->buf_virt[i])
			break;
	}

	/*
	 * Cleanup allocated memories when error
	 */
	if(i != RING_DESC_NR) {
		for(--i ; i >= 0 ; --i) {
			dma_free_coherent(&(jme->pdev->dev),
					  TX_BUF_ALLOC_SIZE,
					  txring->buf_virt[i],
					  txring->buf_dma[i]);
		}
		dma_free_coherent(&(jme->pdev->dev),
				  TX_RING_ALLOC_SIZE,
				  txring->alloc,
				  txring->dmaalloc);
		txring->alloc    = NULL;
		txring->desc     = NULL;
		txring->dmaalloc = 0;
		txring->dma      = 0;
		return -ENOMEM;
	}


	return 0;
}

__always_inline static void jme_free_tx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *txring = &(jme->txring[0]);

	if(txring->alloc) {
		for(i = 0 ; i < RING_DESC_NR ; ++i) {
			if(txring->buf_virt[i]) {
				dma_free_coherent(&(jme->pdev->dev),
						  TX_BUF_ALLOC_SIZE,
						  txring->buf_virt[i],
						  txring->buf_dma[i]);
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
	 * Enable TX Engine
	 */
	wmb();
	jwrite32(jme, JME_TXCS, TXCS_DEFAULT |
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
	jwrite32(jme, JME_TXCS, TXCS_DEFAULT);

	val = jread32(jme, JME_TXCS);
	for(i = JME_TX_DISABLE_TIMEOUT ; (val & TXCS_ENABLE) && i > 0 ; --i)
	{
		udelay(1);
		val = jread32(jme, JME_TXCS);
	}

	if(!i)
		printk(KERN_ERR "Disable TX engine timeout.\n");


}

__always_inline static void jme_set_clean_rxdesc(struct jme_adapter *jme,
						  int i)
{
	struct jme_ring *rxring = jme->rxring;
	struct RxDesc* rxdesc = rxring->desc;
	
	memset(rxdesc + i, 0, RX_DESC_SIZE);
	rxdesc[i].desc1.bufaddrl = cpu_to_le32(ALIGN(rxring->buf_dma[i], 8));
	rxdesc[i].desc1.datalen = cpu_to_le16(RX_BUF_SIZE);
	wmb();
	rxdesc[i].desc1.flags = RXFLAG_OWN | RXFLAG_INT;

#ifdef RX_QUEUE_DEBUG
	dprintk("RX Ring Buf Address(%08x,%08x,%d).\n",
		rxring->buf_dma[i],
		(rxdesc[i].all[12] <<  0) |
		(rxdesc[i].all[13] <<  8) |
		(rxdesc[i].all[14] << 16) |
		(rxdesc[i].all[15] << 24),
		(rxdesc[i].all[4]  <<  0) |
		(rxdesc[i].all[5]  <<  8));
#endif

}

__always_inline static int jme_setup_rx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *rxring = &(jme->rxring[0]);

	rxring->alloc = dma_alloc_coherent(&(jme->pdev->dev),
					   RX_RING_ALLOC_SIZE,
					   &(rxring->dmaalloc), 
					   GFP_KERNEL);
	if(!rxring->alloc)
		return -ENOMEM;

	/*
	 * 16 Bytes align
	 */
	rxring->desc		= (void*)ALIGN((unsigned long)(rxring->alloc), 16);
	rxring->dma		= ALIGN(rxring->dmaalloc, 16);
	rxring->next_to_use	= 0;
	rxring->next_to_clean	= 0;

#ifdef RX_QUEUE_DEBUG
	dprintk("RX Ring Base Address(%08x,%08x).\n",
		(__u32)rxring->desc,
		rxring->dma);
#endif

	/*
	 * Initiallize Receive Descriptors
	 */
	for(i = 0 ; i < RING_DESC_NR ; ++i) {
		rxring->buf_virt[i] = dma_alloc_coherent(&(jme->pdev->dev),
							 RX_BUF_ALLOC_SIZE,
							 &(rxring->buf_dma[i]),
							 GFP_KERNEL);
		if(!rxring->buf_virt[i])
			break;

		jme_set_clean_rxdesc(jme, i);
	}

	/*
	 * Cleanup allocated memories when error
	 */
	if(i != RING_DESC_NR) {
		for(--i ; i >= 0 ; --i) {
			dma_free_coherent(&(jme->pdev->dev),
					  RX_BUF_ALLOC_SIZE,
					  rxring->buf_virt[i],
					  rxring->buf_dma[i]);
		}
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

__always_inline static void jme_free_rx_resources(struct jme_adapter *jme)
{
	int i;
	struct jme_ring *rxring = &(jme->rxring[0]);

	if(rxring->alloc) {
		for(i = 0 ; i < RING_DESC_NR ; ++i) {
			if(rxring->buf_virt[i]) {
				dma_free_coherent(&(jme->pdev->dev),
						  RX_BUF_ALLOC_SIZE,
						  rxring->buf_virt[i],
						  rxring->buf_dma[i]);
			}
		}

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
	__u32 val;

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
	val = jread32(jme, JME_RXCS);
	val |= RXCS_ENABLE | RXCS_QST;
	jwrite32(jme, JME_RXCS, val);
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
		printk(KERN_ERR "Disable RX engine timeout.\n");

}

__always_inline static void jme_process_tx_complete(struct net_device *netdev)
{
	/*
	 * Clear sk_buff here in the future
	 * (Allowing NIC directly DMA with sk_buff kernel requested to send)
	 */
}

__always_inline static void jme_process_receive(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	struct jme_ring *rxring = &(jme->rxring[0]);
	struct RxDesc *rxdesc;
	__u8 *rxbuf;
	struct sk_buff *skb;
	int i, start, cnt;
	int framesize, desccnt;

	/*
	 * Assume that one descriptor per frame,
	 * Should be fixed in the future
	 * (or not? If buffer already large enough to store entire packet.)
	 */

	rxdesc = rxring->desc;

	spin_lock(&jme->recv_lock);
	i = start = rxring->next_to_clean;
	/*
	 * Decide how many descriptors need to be processed
	 * We have to process entire queue in worst case
	 */
	for(cnt = 0 ; cnt < RING_DESC_NR ; ++cnt)
	{
		if(rxdesc[i].descwb.flags & RXWBFLAG_OWN) {
			rxring->next_to_clean = i;
			break;
		}

		if(unlikely(++i == RING_DESC_NR))
			i = 0;
	}
	spin_unlock(&jme->recv_lock);

	/*
	 * Process descriptors independently accross cpu
	 * 	--- save for multiple cpu handling
	 */
	for( i = start ; cnt-- ; ) {
		/*
		* Pass received packet to kernel
		*/
		rxbuf = (void*)ALIGN((unsigned long)(rxring->buf_virt[i]), 8);
		desccnt = rxdesc[i].descwb.desccnt & RXWBDCNT_DCNT;
		framesize = le16_to_cpu(rxdesc[i].descwb.framesize);
		skb = dev_alloc_skb(framesize);
		if(!skb) {
			printk(KERN_ERR PFX "Out of memory.\n");
			++(netdev->stats.rx_dropped);
		}
		else {
			skb_put(skb, framesize);
			skb_copy_to_linear_data(skb, rxbuf, framesize);
			skb->protocol = eth_type_trans(skb, netdev);
			netif_rx(skb);

			netdev->last_rx = jiffies;
			netdev->stats.rx_bytes += framesize;
			++(netdev->stats.rx_packets);
		}

		dprintk("DESCCNT: %u, FSIZE: %u, ADDRH: %08x, "
			"ADDRL: %08x, FLAGS: %04x, STAT: %02x, "
			"DST:%02x:%02x:%02x:%02x:%02x:%02x, "
			"DSTCRC: %d\n",
			desccnt,
			framesize,
			le32_to_cpu(rxdesc[i].dw[2]),
			le32_to_cpu(rxdesc[i].dw[3]),
			le16_to_cpu(rxdesc[i].descwb.flags),
			rxdesc[i].descwb.stat,
			rxbuf[0], rxbuf[1], rxbuf[2],
			rxbuf[3], rxbuf[4], rxbuf[5],
			ether_crc(ETH_ALEN, rxbuf) & 0x3F);


		/*
		* Cleanup descriptor for next receive
		*/
		jme_set_clean_rxdesc(jme, i);

		if(unlikely(++i == RING_DESC_NR))
			i = 0;
	}

}

static irqreturn_t jme_intr(int irq, void *dev_id)
{
        struct net_device *netdev = dev_id;
        struct jme_adapter *jme = netdev_priv(netdev);
	irqreturn_t rc = IRQ_HANDLED;
	__u32 intrstat = jread32(jme, JME_IEVE);
#ifdef RX_QUEUE_DEBUG
	__u32 val;
#endif

#if 0
	/*
	 * Don't disable interrupt, the driver should be
	 * working fine with multiple interrupt handling
	 * at the same time. (When Multi-core CPU)
	 */

	/*
	 * Temporary disable all Interrupts From Our NIC
	 */
	jwrite32(jme, JME_IENC, INTR_ENABLE);
	wmb();
#endif

	dprintk("Interrupt received(%08x).\n", intrstat);


	/*
	 * Check if it's really an interrupt for us
	 * and if the device still exist
	 */
        if((intrstat & INTR_ENABLE) == 0 || intrstat == ~0) {
                rc = IRQ_NONE;
		goto out;
	}

	if(intrstat & INTR_LINKCH) {
		/*
		 * Process Link status change event
		 */
		jme_check_link(netdev);

		/*
		 * Write 1 clear Link status change Interrupt
		 */
		jwrite32(jme, JME_IEVE, INTR_LINKCH);
	}

	if(intrstat & INTR_RX0) {
		/*
		 * Process event
		 */
		jme_process_receive(netdev);

		/*
		 * Write 1 clear Interrupt
		 */
		jwrite32(jme, JME_IEVE, INTR_RX0);

                dprintk("Received From Queue 0.\n");

#ifdef RX_QUEUE_DEBUG
		//Poll out the Receive Queue Next Descriptor Address/Status
		val = jread32(jme, JME_RXCS);
		val |= RXCS_QST;
		jwrite32(jme, JME_RXCS, val);
		wmb();
		val = jread32(jme, JME_RXNDA);
                dprintk("NEXT_RX_DESC.(%08x)\n", val);
#endif

	}

	if(intrstat & INTR_RX0EMP) {
		/*
		 * Write 1 clear Interrupt
		 */
		jwrite32(jme, JME_IEVE, INTR_RX0EMP);

                dprintk("Received Queue 0 is running-out.\n");
	}

	if(intrstat & INTR_TX0) {
		/*
		 * Process event
		 */
		jme_process_tx_complete(netdev);

		/*
		 * Write 1 clear Interrupt
		 */
		jwrite32(jme, JME_IEVE, INTR_TX0);

                dprintk("Queue 0 transmit complete.\n");
	}

out:

#if 0
	/*
	 * Re-enable interrupts
	 */
	wmb();
	jwrite32(jme, JME_IENS, INTR_ENABLE);
#endif
        return rc;
}

static int jme_open(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);
	int CHECK_VAR;

	CHECK_AND_GOTO(request_irq(jme->pdev->irq, jme_intr, IRQF_SHARED,
				   netdev->name, netdev),
		       err_out,
		       "Requesting IRQ error.")

	CHECK_AND_GOTO(jme_setup_rx_resources(jme),
		       err_out_free_irq,
		       "Error allocating resources for RX.")

	CHECK_AND_GOTO(jme_setup_tx_resources(jme),
		       err_out_free_rx_resources,
		       "Error allocating resources for TX.")

	jme_reset_mac_processor(jme);
	jme_check_link(netdev);
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
	return CHECK_VAR;
}

static int jme_close(struct net_device *netdev)
{
	struct jme_adapter *jme = netdev_priv(netdev);

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	jme_stop_irq(jme);
	free_irq(jme->pdev->irq, jme->dev);

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
		return NETDEV_TX_BUSY;
	}
	if(unlikely(++(txring->next_to_use) == RING_DESC_NR))
		txring->next_to_use = 0;
	spin_unlock(&jme->xmit_lock);


	/*
	 * Fill up TX descriptors
	 */
	skb_copy_from_linear_data(skb,
				  (void*)ALIGN((unsigned long)(txring->buf_virt[idx]), 8),
				  skb->len);
	jme_set_new_txdesc(jme, idx, skb->len);

	/*
	 * Since still using copy now. we could free it here.
	 */
	dev_kfree_skb(skb);

	/*
	 * Tell MAC HW to send
	 */
	jwrite32(jme, JME_TXCS, TXCS_QUEUE0S |
				  TXCS_DEFAULT |
				  TXCS_SELECT_QUEUE0 |
				  TXCS_ENABLE);

	netdev->stats.tx_bytes += skb->len;
	++(netdev->stats.tx_packets);
	netdev->trans_start = jiffies;

        return 0;
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
	jwrite32(jme, JME_RXUMA, val);
	val = addr->sa_data[5] << 8 |
	      addr->sa_data[4];
	jwrite32(jme, JME_RXUMA+4, val);
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
			dprintk("Adding MCAddr: "
				"%02x:%02x:%02x:%02x:%02x:%02x (%d)\n",
				mclist->dmi_addr[0],
				mclist->dmi_addr[1],
				mclist->dmi_addr[2],
				mclist->dmi_addr[3],
				mclist->dmi_addr[4],
				mclist->dmi_addr[5],
				bit_nr);
                }

		jwrite32(jme, JME_RXMCHT, mc_hash[0]);
		jwrite32(jme, JME_RXMCHT+4, mc_hash[1]);
	}


	wmb();
	jwrite32(jme, JME_RXMCS, val);
	spin_unlock(&jme->macaddr_lock);

	dprintk("RX Mode changed: %08x\n", val);
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
	int CHECK_VAR = 0;
	struct net_device *netdev;
	struct jme_adapter *jme;
	DECLARE_MAC_BUF(mac);

	/*
	 * set up PCI device basics
	 */
	CHECK_AND_GOTO(pci_enable_device(pdev),
		       err_out,
		       "Cannot enable PCI device.")

	CHECK_AND_GOTO(!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM),
		       err_out_disable_pdev,
		       "No PCI resource region found.")

	CHECK_AND_GOTO(pci_request_regions(pdev, DRV_NAME),
		       err_out_disable_pdev,
		       "Cannot obtain PCI resource region.")

	pci_set_master(pdev);

	/*
	 * alloc and init net device
	 */
	netdev = alloc_etherdev(sizeof(struct jme_adapter));
	if(!netdev) {
		CHECK_VAR = -ENOMEM;
		goto err_out_disable_pdev;
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
	if (!jme->regs) {
		rc = -ENOMEM;
		goto err_out_free_netdev;
	}
	spin_lock_init(&jme->xmit_lock);
	spin_lock_init(&jme->recv_lock);
	spin_lock_init(&jme->macaddr_lock);
	spin_lock_init(&jme->phy_lock);
	jme->mii_if.dev = netdev;
	jme->mii_if.phy_id = 1;
	jme->mii_if.supports_gmii = 1;
	jme->mii_if.mdio_read = jme_mdio_read;
	jme->mii_if.mdio_write = jme_mdio_write;

	/*
	 * Reset MAC processor and reload EEPROM for MAC Address
	 */
	jme_clear_pm(jme);
	jme_reset_mac_processor(jme);
	CHECK_AND_GOTO(jme_reload_eeprom(jme),
		       err_out_unmap,
		       "Rload eeprom for reading MAC Address error.");
	jme_load_macaddr(netdev);


	/*
	 * Tell stack that we are not ready to work until open()
	 */
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);

	/*
	 * Register netdev
	 */
	CHECK_AND_GOTO(register_netdev(netdev),
	               err_out_unmap,
		       "Cannot register net device.")

	printk(KERN_INFO "%s: JMC250 gigabit eth at %llx, %s, IRQ %d\n",
	       netdev->name,
	       (unsigned long long) pci_resource_start(pdev, 0),
	       print_mac(mac, netdev->dev_addr),
	       pdev->irq);

	pci_set_drvdata(pdev, netdev);

	return 0;

err_out_unmap:
	iounmap(jme->regs);
err_out_free_netdev:
	pci_set_drvdata(pdev, NULL);
	free_netdev(netdev);
err_out_disable_pdev:
        pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
err_out:
	return CHECK_VAR;
}

static void __devexit jme_remove_one(struct pci_dev *pdev)
{ 
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct jme_adapter *jme = netdev_priv(netdev);

	unregister_netdev(netdev);
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
	printk(KERN_INFO "jme: JMicron JMC250 gigabit ethernet "
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


