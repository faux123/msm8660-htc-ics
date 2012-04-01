/*
 *  Abstract:
 *      Debug Flags for SDIO_AL
 *
 *  Working environment:
 *      Android/LTE 8x60 projects
 *
 *  Referenced documents:
 *      N/A
 *
 *  Revision history:
 *      Trial12MAY2011 --Bert Lin--
 */
#ifndef __SDIODBG_8X60_H__
#define __SDIODBG_8X60_H__

/* external module flags */
#define DBG_DMUX        BIT(31)
#define DBG_RMNET       BIT(30)
#define DBG_CMUX        BIT(29)
#define DBG_CTL         BIT(28)

#define DBG_RPC         BIT(27)
#define DBG_MDM         BIT(26)

/* inter-module flags */
#define DBG_LAWDATA     BIT(5)
#define DBG_MEMCPY      BIT(4)

#define DBG_ALDEBUG     BIT(3)
#define DBG_DATA        BIT(2)
#define DBG_LPM         BIT(1)

#endif /* __SDIODBG_8X60_H__ */
