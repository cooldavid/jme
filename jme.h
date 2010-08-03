
#define DRV_NAME	"jme"
#define DRV_VERSION	"0.1"
#define PFX DRV_NAME	": "

#ifdef DEBUG
#define dprintk(fmt, args...) \
        printk(KERN_DEBUG PFX "%s: " fmt, jme->dev->name, ## args);
#else
#define dprintk(fmt, args...)
#endif

#define jprintk(fmt, args...) \
        printk(KERN_INFO PFX "%s: " fmt, jme->dev->name, ## args);

#undef RX_QUEUE_DEBUG

#define DEFAULT_MSG_ENABLE        \
	(NETIF_MSG_DRV          | \
	 NETIF_MSG_PROBE        | \
	 NETIF_MSG_LINK         | \
	 NETIF_MSG_TIMER        | \
	 NETIF_MSG_RX_ERR       | \
	 NETIF_MSG_TX_ERR)

#define CHECK_VAR rc
#define CHECK_AND_GOTO(fun, label, msg)		\
	CHECK_VAR = fun;			\
	if(CHECK_VAR)				\
	{					\
		printk(KERN_ERR PFX msg "\n");	\
		goto label;			\
	}

/*
 * TX/RX Descriptors
 */
#define RING_DESC_NR		512 /* Must be power of 2 */
#define TX_DESC_SIZE		16
#define TX_RING_NR		8
#define TX_RING_ALLOC_SIZE	(RING_DESC_NR * TX_DESC_SIZE) + TX_DESC_SIZE
#define TX_RING_SIZE		(RING_DESC_NR * TX_DESC_SIZE)

#define TX_BUF_DMA_ALIGN	8
#define TX_BUF_SIZE		1600
#define TX_BUF_ALLOC_SIZE	TX_BUF_SIZE + TX_BUF_DMA_ALIGN

struct TxDesc {
	union {
		__u8  all[16];
		__u32 dw[4];
		struct {
			/* DW0 */
			__u16 vlan;
			__u8 rsv1;
			__u8 flags;

			/* DW1 */
			__u16 datalen;
			__u16 mss;

			/* DW2 */
			__u16 pktsize;
			__u16 rsv2;

			/* DW3 */
			__u32 bufaddr;
		} desc1;
	};
};
enum jme_txdesc_flag_bits {
	TXFLAG_OWN	= 0x80,
	TXFLAG_INT	= 0x40,
	TXFLAG_TCPCS	= 0x10,
	TXFLAG_UDPCS	= 0x08,
	TXFLAG_IPCS	= 0x04,
	TXFLAG_LSEN	= 0x02,
	TXFLAG_TAGON	= 0x01,
};


#define RX_DESC_SIZE		16
#define RX_RING_NR		4
#define RX_RING_ALLOC_SIZE	(RING_DESC_NR * RX_DESC_SIZE) + RX_DESC_SIZE
#define RX_RING_SIZE		(RING_DESC_NR * RX_DESC_SIZE)

#define RX_BUF_DMA_ALIGN	8
#define RX_BUF_SIZE		1600
#define RX_BUF_ALLOC_SIZE	RX_BUF_SIZE + RX_BUF_DMA_ALIGN

struct RxDesc {
	union {
		__u8   all[16];
		__le32 dw[4];
		struct {
			/* DW0 */
			__le16 rsv2;
			__u8 rsv1;
			__u8 flags;

			/* DW1 */
			__le16 datalen;
			__le16 wbcpl;

			/* DW2 */
			__le32 bufaddrh;

			/* DW3 */
			__le32 bufaddrl;
		} desc1;
		struct {
			/* DW0 */
			__le16 vlan;
			__le16 flags;

			/* DW1 */
			__le16 framesize;
			__u8 stat;
			__u8 desccnt;

			/* DW2 */
			__le32 rsshash;

			/* DW3 */
			__u8   hashfun; 
			__u8   hashtype; 
			__le16 resrv;
		} descwb;
	};
};
enum jme_rxdesc_flags_bits {
	RXFLAG_OWN	= 0x80,
	RXFLAG_INT	= 0x40,
	RXFLAG_64BIT	= 0x20,
};
enum jme_rxwbdesc_flags_bits {
	RXWBFLAG_OWN	= 0x8000,
	RXWBFLAG_INT	= 0x4000,
	RXWBFLAG_MF	= 0x2000,
	RXWBFLAG_64BIT	= 0x2000,
	RXWBFLAG_TCPON	= 0x1000,
	RXWBFLAG_UDPON	= 0x0800,
	RXWBFLAG_IPCS	= 0x0400,
	RXWBFLAG_TCPCS	= 0x0200,
	RXWBFLAG_UDPCS	= 0x0100,
	RXWBFLAG_TAGON	= 0x0080,
	RXWBFLAG_IPV4	= 0x0040,
	RXWBFLAG_IPV6	= 0x0020,
	RXWBFLAG_PAUSE	= 0x0010,
	RXWBFLAG_MAGIC	= 0x0008,
	RXWBFLAG_WAKEUP	= 0x0004,
	RXWBFLAG_DEST	= 0x0003,
};
enum jme_rxwbdesc_desccnt_mask {
	RXWBDCNT_WBCPL	= 0x80,
	RXWBDCNT_DCNT	= 0x7F,
};

struct jme_ring {
        void *alloc;		/* pointer to allocated memory */
        void *desc;		/* pointer to ring memory  */
        dma_addr_t dmaalloc;	/* phys address of ring alloc */
        dma_addr_t dma;		/* phys address for ring dma */

	/* Virtual addresses for each buffer of Desc */
	void*      buf_virt[RING_DESC_NR];
	/* Physical addresses for each buffer of Desc */
	dma_addr_t buf_dma[RING_DESC_NR];

        u16 next_to_use;
        u16 next_to_clean;

	/*
	 * Kernel requested TX sk_buffs
	 * should be cleared after tx complete
	 */
	struct sk_buff *tx_skb[RING_DESC_NR];
};

/*
 * Jmac Adapter Private data
 */
struct jme_adapter {
        struct pci_dev          *pdev;
        struct net_device       *dev;
        void __iomem            *regs;
	struct mii_if_info	mii_if;
	struct jme_ring		rxring[RX_RING_NR];
	struct jme_ring		txring[TX_RING_NR];
	spinlock_t		xmit_lock;
	spinlock_t		recv_lock;
	spinlock_t		macaddr_lock;
	spinlock_t		phy_lock;
};

/*
 * MMaped I/O Resters
 */
enum jme_iomap_offsets {
	JME_MAC	= 0x0000,
	JME_PHY	= 0x0400,
	JME_MISC	= 0x0800,
	JME_RSS	= 0x0C00,
};

enum jme_iomap_regs {
	JME_TXCS	= JME_MAC | 0x00, /* Transmit Control and Status */
	JME_TXDBA	= JME_MAC | 0x04, /* Transmit Queue Desc Base Addr */
	JME_TXQDC	= JME_MAC | 0x0C, /* Transmit Queue Desc Count */
	JME_TXNDA	= JME_MAC | 0x10, /* Transmit Queue Next Desc Addr */
	JME_TXMCS	= JME_MAC | 0x14, /* Transmit MAC Control Status */
	JME_TXPFC	= JME_MAC | 0x18, /* Transmit Pause Frame Control */
	JME_TXTRHD	= JME_MAC | 0x1C, /* Transmit Timer/Retry@Half-Dup */

	JME_RXCS	= JME_MAC | 0x20, /* Receive Control and Status */
	JME_RXDBA	= JME_MAC | 0x24, /* Receive Queue Desc Base Addr */
	JME_RXQDC	= JME_MAC | 0x2C, /* Receive Queue Desc Count */
	JME_RXNDA	= JME_MAC | 0x30, /* Receive Queue Next Desc Addr */
	JME_RXMCS	= JME_MAC | 0x34, /* Receive MAC Control Status */
	JME_RXUMA	= JME_MAC | 0x38, /* Receive Unicast MAC Address */
	JME_RXMCHT	= JME_MAC | 0x40, /* Receive Multicast Addr HashTable */
	JME_WFODP	= JME_MAC | 0x48, /* Wakeup Frame Output Data Port */
	JME_WFOI	= JME_MAC | 0x4C, /* Wakeup Frame Output Interface */

	JME_SMI		= JME_MAC | 0x50, /* Station Management Interface */
	JME_GHC		= JME_MAC | 0x54, /* Global Host Control */
	JME_PMCS	= JME_MAC | 0x60, /* Power Management Control/Stat */


	JME_PHY_CS	= JME_PHY | 0x28, /* PHY Control and Status Register */
	JME_PHY_LINK	= JME_PHY | 0x30, /* PHY Link Status Register */
	JME_SMBCSR	= JME_PHY | 0x40, /* SMB Control and Status */


	JME_IEVE	= JME_MISC| 0x20, /* Interrupt Event Status */
	JME_IREQ	= JME_MISC| 0x24, /* Interrupt Req Status (For Debug) */
	JME_IENS	= JME_MISC| 0x28, /* Interrupt Enable - Setting Port */
	JME_IENC	= JME_MISC| 0x2C, /* Interrupt Enable - Clearing Port */
};

/*
 * TX Control/Status Bits
 */
enum jme_txcs_bits {
	TXCS_QUEUE7S	= 0x00008000,
	TXCS_QUEUE6S	= 0x00004000,
	TXCS_QUEUE5S	= 0x00002000,
	TXCS_QUEUE4S	= 0x00001000,
	TXCS_QUEUE3S	= 0x00000800,
	TXCS_QUEUE2S	= 0x00000400,
	TXCS_QUEUE1S	= 0x00000200,
	TXCS_QUEUE0S	= 0x00000100,
	TXCS_FIFOTH	= 0x000000C0,
	TXCS_DMASIZE	= 0x00000030,
	TXCS_BURST	= 0x00000004,
	TXCS_ENABLE	= 0x00000001,
};
enum jme_txcs_value {
	TXCS_FIFOTH_16QW	= 0x000000C0,
	TXCS_FIFOTH_12QW	= 0x00000080,
	TXCS_FIFOTH_8QW		= 0x00000040,
	TXCS_FIFOTH_4QW		= 0x00000000,

	TXCS_DMASIZE_64B	= 0x00000000,
	TXCS_DMASIZE_128B	= 0x00000010,
	TXCS_DMASIZE_256B	= 0x00000020,
	TXCS_DMASIZE_512B	= 0x00000030,

	TXCS_SELECT_QUEUE0	= 0x00000000,
	TXCS_SELECT_QUEUE1	= 0x00010000,
	TXCS_SELECT_QUEUE2	= 0x00020000,
	TXCS_SELECT_QUEUE3	= 0x00030000,
	TXCS_SELECT_QUEUE4	= 0x00040000,
	TXCS_SELECT_QUEUE5	= 0x00050000,
	TXCS_SELECT_QUEUE6	= 0x00060000,
	TXCS_SELECT_QUEUE7	= 0x00070000,

	TXCS_DEFAULT		= TXCS_FIFOTH_4QW |
				  TXCS_DMASIZE_512B | 
				  TXCS_BURST,
};
#define JME_TX_DISABLE_TIMEOUT 200 /* 200 usec */

/*
 * TX MAC Control/Status Bits
 */
enum jme_txmcs_bit_masks {
	TXMCS_IFG2		= 0xC0000000,
	TXMCS_IFG1		= 0x30000000,
	TXMCS_TTHOLD		= 0x00000300,
	TXMCS_FBURST		= 0x00000080,
	TXMCS_CARRIEREXT	= 0x00000040,
	TXMCS_DEFER		= 0x00000020,
	TXMCS_BACKOFF		= 0x00000010,
	TXMCS_CARRIERSENSE	= 0x00000008,
	TXMCS_COLLISION		= 0x00000004,
	TXMCS_CRC		= 0x00000002,
	TXMCS_PADDING		= 0x00000001,
};
enum jme_txmcs_values {
	TXMCS_IFG2_6_4		= 0x00000000,
	TXMCS_IFG2_8_5		= 0x40000000,
	TXMCS_IFG2_10_6		= 0x80000000,
	TXMCS_IFG2_12_7		= 0xC0000000,

	TXMCS_IFG1_8_4		= 0x00000000,
	TXMCS_IFG1_12_6		= 0x10000000,
	TXMCS_IFG1_16_8		= 0x20000000,
	TXMCS_IFG1_20_10	= 0x30000000,

	TXMCS_TTHOLD_1_8	= 0x00000000,
	TXMCS_TTHOLD_1_4	= 0x00000100,
	TXMCS_TTHOLD_1_2	= 0x00000200,
	TXMCS_TTHOLD_FULL	= 0x00000300,

	TXMCS_DEFAULT		= TXMCS_IFG2_8_5 |
				  TXMCS_IFG1_16_8 |
				  TXMCS_TTHOLD_FULL |
				  TXMCS_DEFER |
				  TXMCS_CRC |
				  TXMCS_PADDING,
};


/*
 * RX Control/Status Bits
 */
enum jme_rxcs_bits {
	RXCS_QST	= 0x00000004,
	RXCS_ENABLE	= 0x00000001,
};
#define JME_RX_DISABLE_TIMEOUT 200 /* 200 usec */

/*
 * RX MAC Control/Status Bits
 */
enum jme_rxmcs_bits {
	RXMCS_ALLFRAME		= 0x00000800,
	RXMCS_BRDFRAME		= 0x00000400,
	RXMCS_MULFRAME		= 0x00000200,
	RXMCS_UNIFRAME		= 0x00000100,
	RXMCS_ALLMULFRAME	= 0x00000080,
	RXMCS_MULFILTERED	= 0x00000040,
};

/*
 * SMI Related definitions
 */
enum jme_smi_bit_mask
{
	SMI_DATA_MASK		= 0xFFFF0000,
	SMI_REG_ADDR_MASK	= 0x0000F800,
	SMI_PHY_ADDR_MASK	= 0x000007C0,
	SMI_OP_WRITE		= 0x00000020,
	SMI_OP_REQ		= 0x00000010, /* Set to 1, after req done it'll be cleared to 0 */
	SMI_OP_MDIO		= 0x00000008, /* Software assess In/Out */
	SMI_OP_MDOE		= 0x00000004, /* Software Output Enable */
	SMI_OP_MDC		= 0x00000002, /* Software CLK Control */
	SMI_OP_MDEN		= 0x00000001, /* Software access Enable */
};
enum jme_smi_bit_shift
{
	SMI_DATA_SHIFT		= 16,
	SMI_REG_ADDR_SHIFT	= 11,
	SMI_PHY_ADDR_SHIFT	= 6,
};
__always_inline __u32 smi_reg_addr(int x)
{
        return (((x) << SMI_REG_ADDR_SHIFT) & SMI_REG_ADDR_MASK);
}
__always_inline __u32 smi_phy_addr(int x)
{
        return (((x) << SMI_PHY_ADDR_SHIFT) & SMI_PHY_ADDR_MASK);
}
#define JME_PHY_TIMEOUT 1000 /* 1000 usec */

/*
 * Global Host Control
 */
enum jme_ghc_bit_mask {
	GHC_SWRST	= 0x40000000,
	GHC_DPX		= 0x00000040,
	GHC_SPEED	= 0x00000030,
	GHC_LINK_POLL	= 0x00000001,
};
enum jme_ghc_speed_val {
	GHC_SPEED_10M	= 0x00000010,
	GHC_SPEED_100M	= 0x00000020,
	GHC_SPEED_1000M	= 0x00000030,
};

/*
 * Giga PHY Status Registers 
 */
enum jme_phy_link_bit_mask {
	PHY_LINK_SPEED_MASK		= 0x0000C000,
	PHY_LINK_DUPLEX			= 0x00002000,
	PHY_LINK_SPEEDDPU_RESOLVED	= 0x00000800,
	PHY_LINK_UP			= 0x00000400,
	PHY_LINK_AUTONEG_COMPLETE	= 0x00000200,
};
enum jme_phy_link_speed_val {
	PHY_LINK_SPEED_10M		= 0x00000000,
	PHY_LINK_SPEED_100M		= 0x00004000,
	PHY_LINK_SPEED_1000M		= 0x00008000,
};
#define JME_AUTONEG_TIMEOUT	500	/* 500 ms */

/*
 * SMB Control and Status
 */
enum jme_smbcsr_bit_mask
{
	SMBCSR_CNACK	= 0x00020000,
	SMBCSR_RELOAD	= 0x00010000,
	SMBCSR_EEPROMD	= 0x00000020,
};
#define JME_SMB_TIMEOUT 10 /* 10 msec */


/*
 * Interrupt Status Bits
 */
enum jme_interrupt_bits
{
	INTR_SWINTR	= 0x80000000,
	INTR_TMINTR	= 0x40000000,
	INTR_LINKCH	= 0x20000000,
	INTR_PAUSERCV	= 0x10000000,
	INTR_MAGICRCV	= 0x08000000,
	INTR_WAKERCV	= 0x04000000,
	INTR_PCCRX0TO	= 0x02000000,
	INTR_PCCRX1TO	= 0x01000000,
	INTR_PCCRX2TO	= 0x00800000,
	INTR_PCCRX3TO	= 0x00400000,
	INTR_PCCTXTO	= 0x00200000,
	INTR_PCCRX0	= 0x00100000,
	INTR_PCCRX1	= 0x00080000,
	INTR_PCCRX2	= 0x00040000,
	INTR_PCCRX3	= 0x00020000,
	INTR_PCCTX	= 0x00010000,
	INTR_RX3EMP	= 0x00008000,
	INTR_RX2EMP	= 0x00004000,
	INTR_RX1EMP	= 0x00002000,
	INTR_RX0EMP	= 0x00001000,
	INTR_RX3	= 0x00000800,
	INTR_RX2	= 0x00000400,
	INTR_RX1	= 0x00000200,
	INTR_RX0	= 0x00000100,
	INTR_TX7	= 0x00000080,
	INTR_TX6	= 0x00000040,
	INTR_TX5	= 0x00000020,
	INTR_TX4	= 0x00000010,
	INTR_TX3	= 0x00000008,
	INTR_TX2	= 0x00000004,
	INTR_TX1	= 0x00000002,
	INTR_TX0	= 0x00000001,
};
static const __u32 INTR_ENABLE = INTR_LINKCH |
				 INTR_RX0EMP |
				 INTR_RX0 |
				 INTR_TX0;

/*
 * Read/Write MMaped I/O Registers
 */
__always_inline __u32 jread32(struct jme_adapter *jme, __u32 reg)
{
	return le32_to_cpu(readl(jme->regs + reg));
}
__always_inline void jwrite32(struct jme_adapter *jme, __u32 reg, __u32 val)
{
	writel(cpu_to_le32(val), jme->regs + reg);
}
__always_inline void jwrite32f(struct jme_adapter *jme, __u32 reg, __u32 val)
{
	/*
	 * Read after write should cause flush
	 */
	writel(cpu_to_le32(val), jme->regs + reg);
	readl(jme->regs + reg);
}

/*
 * Function prototypes for ethtool
 */
static void jme_get_drvinfo(struct net_device *netdev,
			     struct ethtool_drvinfo *info);
static int jme_get_settings(struct net_device *netdev,
			     struct ethtool_cmd *ecmd);
static int jme_set_settings(struct net_device *netdev,
			     struct ethtool_cmd *ecmd);
static u32 jme_get_link(struct net_device *netdev);


/*
 * Function prototypes for netdev
 */
static int jme_open(struct net_device *netdev);
static int jme_close(struct net_device *netdev);
static int jme_start_xmit(struct sk_buff *skb, struct net_device *netdev);
static int jme_set_macaddr(struct net_device *netdev, void *p);
static void jme_set_multi(struct net_device *netdev);

