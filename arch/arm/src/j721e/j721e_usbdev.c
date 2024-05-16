/****************************************************************************
 * arch/arm/src/j721e/j721e_usbdev.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "chip.h"
#include "arm_internal.h"
#include "j721e_usbdev.h"

#include "hardware/j721e_resets.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USBDEV_SETUP_MAXDATASIZE
#  define CONFIG_USBDEV_SETUP_MAXDATASIZE (CONFIG_USBDEV_EP0_MAXSIZE * 4)
#endif

/* Debug ********************************************************************/

/* Trace error codes */

#define J721E_TRACEERR_ALLOCFAIL         0x0001
#define J721E_TRACEERR_BINDFAILED        0x0002
#define J721E_TRACEERR_DRIVER            0x0003
#define J721E_TRACEERR_EPREAD            0x0004
#define J721E_TRACEERR_EWRITE            0x0005
#define J721E_TRACEERR_INVALIDPARMS      0x0006
#define J721E_TRACEERR_IRQREGISTRATION   0x0007
#define J721E_TRACEERR_NULLPACKET        0x0008
#define J721E_TRACEERR_NULLREQUEST       0x0009
#define J721E_TRACEERR_REQABORTED        0x000a
#define J721E_TRACEERR_STALLEDCLRFEATURE 0x000b
#define J721E_TRACEERR_STALLEDISPATCH    0x000c
#define J721E_TRACEERR_STALLEDGETST      0x000d
#define J721E_TRACEERR_STALLEDGETSTEP    0x000e
#define J721E_TRACEERR_STALLEDGETSTRECIP 0x000f
#define J721E_TRACEERR_STALLEDREQUEST    0x0010
#define J721E_TRACEERR_STALLEDSETFEATURE 0x0011
#define J721E_TRACEERR_TXREQLOST         0x0012
#define J721E_TRACEERR_RXREQLOST         0x0013

/* Trace interrupt codes */

#define J721E_TRACEINTID_GETSTATUS       1
#define J721E_TRACEINTID_GETIFDEV        2
#define J721E_TRACEINTID_CLEARFEATURE    3
#define J721E_TRACEINTID_SETFEATURE      4
#define J721E_TRACEINTID_TESTMODE        5
#define J721E_TRACEINTID_SETADDRESS      6
#define J721E_TRACEINTID_GETSETDESC      7
#define J721E_TRACEINTID_GETSETIFCONFIG  8
#define J721E_TRACEINTID_SYNCHFRAME      9
#define J721E_TRACEINTID_DISPATCH       10
#define J721E_TRACEINTID_GETENDPOINT    11
#define J721E_TRACEINTID_HANDLEZLP      12
#define J721E_TRACEINTID_USBINTERRUPT   13
#define J721E_TRACEINTID_INTR_BUSRESET  14
#define J721E_TRACEINTID_INTR_BUFFSTAT  15
#define J721E_TRACEINTID_INTR_SETUP     16
#define J721E_TRACEINTID_EPOUTQEMPTY    17

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(J721E_TRACEERR_ALLOCFAIL),
  TRACE_STR(J721E_TRACEERR_BINDFAILED),
  TRACE_STR(J721E_TRACEERR_DRIVER),
  TRACE_STR(J721E_TRACEERR_EPREAD),
  TRACE_STR(J721E_TRACEERR_EWRITE),
  TRACE_STR(J721E_TRACEERR_INVALIDPARMS),
  TRACE_STR(J721E_TRACEERR_IRQREGISTRATION),
  TRACE_STR(J721E_TRACEERR_NULLPACKET),
  TRACE_STR(J721E_TRACEERR_NULLREQUEST),
  TRACE_STR(J721E_TRACEERR_REQABORTED),
  TRACE_STR(J721E_TRACEERR_STALLEDCLRFEATURE),
  TRACE_STR(J721E_TRACEERR_STALLEDISPATCH),
  TRACE_STR(J721E_TRACEERR_STALLEDGETST),
  TRACE_STR(J721E_TRACEERR_STALLEDGETSTEP),
  TRACE_STR(J721E_TRACEERR_STALLEDGETSTRECIP),
  TRACE_STR(J721E_TRACEERR_STALLEDREQUEST),
  TRACE_STR(J721E_TRACEERR_STALLEDSETFEATURE),
  TRACE_STR(J721E_TRACEERR_TXREQLOST),
  TRACE_STR(J721E_TRACEERR_RXREQLOST),
  TRACE_STR_END
};

const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(J721E_TRACEINTID_GETSTATUS),
  TRACE_STR(J721E_TRACEINTID_GETIFDEV),
  TRACE_STR(J721E_TRACEINTID_CLEARFEATURE),
  TRACE_STR(J721E_TRACEINTID_SETFEATURE),
  TRACE_STR(J721E_TRACEINTID_TESTMODE),
  TRACE_STR(J721E_TRACEINTID_SETADDRESS),
  TRACE_STR(J721E_TRACEINTID_GETSETDESC),
  TRACE_STR(J721E_TRACEINTID_GETSETIFCONFIG),
  TRACE_STR(J721E_TRACEINTID_SYNCHFRAME),
  TRACE_STR(J721E_TRACEINTID_DISPATCH),
  TRACE_STR(J721E_TRACEINTID_GETENDPOINT),
  TRACE_STR(J721E_TRACEINTID_HANDLEZLP),
  TRACE_STR(J721E_TRACEINTID_USBINTERRUPT),
  TRACE_STR(J721E_TRACEINTID_INTR_BUSRESET),
  TRACE_STR(J721E_TRACEINTID_INTR_BUFFSTAT),
  TRACE_STR(J721E_TRACEINTID_INTR_SETUP),
  TRACE_STR(J721E_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR_END
};
#endif

/* Hardware interface *******************************************************/

/* Hardware dependent sizes and numbers */

#define J721E_EP0MAXPACKET     64        /* EP0 max packet size */
#define J721E_BULKMAXPACKET    64        /* Bulk endpoint max packet */
#define J721E_INTRMAXPACKET    64        /* Interrupt endpoint max packet */
#define J721E_ISOMAXPACKET     1023      /* Isochronous max packet size */

/* USB endpoint number conversion macros
 * EPINDEX: eplist[] index (the endpoint list in j721e_usbdev_s)
 *   0 - Endpoint  0 IN
 *   1 - Endpoint  0 OUT
 *   2 - Endpoint  1 (IN or OUT - depends on the endpoint configuration)
 *   3 - Endpoint  2 (IN or OUT - depends on the endpoint configuration)
 *   4 - Endpoint  3 (IN or OUT - depends on the endpoint configuration)
 *     :
 *  15 - Endpoint 14 (IN or OUT - depends on the endpoint configuration)
 *  16 - Endpoint 15 (IN or OUT - depends on the endpoint configuration)
 *
 * DPINDEX: J721E DPSRAM control index
 *   0 - Endpoint  0 IN
 *   1 - Endpoint  0 OUT
 *   2 - Endpoint  1 IN
 *   3 - Endpoint  1 OUT
 *   4 - Endpoint  2 IN
 *   5 - Endpoint  2 OUT
 *     :
 *  30 - Endpoint 15 IN
 *  31 - Endpoint 15 OUT
 */

#define J721E_EPINDEX(eplog)   (USB_EPNO(eplog) == 0 ? \
                                 (USB_ISEPIN(eplog) ? 0 : 1) : \
                                 (USB_EPNO(eplog) + 1))
#define J721E_DPINDEX(eplog)   (USB_EPNO(eplog) * 2 + USB_ISEPOUT(eplog))
#define J721E_DPTOEP(index)    ((index) < 2 ? (index) : (index) / 2 + 1)

#define J721E_NENDPOINTS       (16 + 1)  /* EP0 IN, EP0 OUT, EP1..EP15 */

/* Request queue operations *************************************************/

#define j721e_rqempty(ep)      ((ep)->head == NULL)
#define j721e_rqpeek(ep)       ((ep)->head)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* USB Zero Length Packet type */

enum j721e_zlp_e
{
  J721E_ZLP_NONE = 0,        /* Don't send/receive Zero Length Packet */
  J721E_ZLP_IN_REPLY,        /* Receive ZLP to reply IN transfer */
  J721E_ZLP_OUT_REPLY,       /* Send ZLP to reply OUT transfer */
};

/* A container for a request so that the request make be retained in a list */

struct j721e_req_s
{
  struct usbdev_req_s req;        /* Standard USB request */
  struct j721e_req_s *flink;     /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct j721e_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct j721e_ep_s.
   */

  struct usbdev_ep_s ep;          /* Standard endpoint structure */

  /* J721E-specific fields */

  struct j721e_usbdev_s *dev;    /* Reference to private driver data */
  struct j721e_req_s *head;      /* Request list for this endpoint */
  struct j721e_req_s *tail;
  uint8_t *data_buf;              /* DPSRAM buffer address */
  uint32_t ep_ctrl;               /* DPSRAM EP control register address */
  uint32_t buf_ctrl;              /* DPSRAM buffer control register address */
  int     next_pid;               /* Next PID 0:DATA0, 1:DATA1 */
  uint8_t type;                   /* 0:cont, 1:iso, 2:bulk, 3:int */
  uint8_t epphy;                  /* Physical EP address */
  bool    txnullpkt;              /* Null packet needed at end of transfer */
  bool    in;                     /* in = true, out = false */
  bool    stalled;                /* The EP is stalled */
  bool    pending_stall;          /* Pending stall request */
};

/* This structure encapsulates the overall driver state */

struct j721e_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct j721e_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* J721E-specific fields */

  uint16_t next_offset;       /* Unused DPSRAM buffer offset */
  uint8_t  dev_addr;          /* USB device address */
  enum j721e_zlp_e zlp_stat; /* Pending EP0 ZLP status */
  uint16_t used;              /* used epphy */
  bool stalled;
  bool selfpowered;           /* 1: Device is self powered */

  /* EP0 SETUP data buffering.
   *
   * ctrl
   *   The 8-byte SETUP request is received on the EP0 OUT endpoint and is
   *   saved.
   *
   * ep0data
   *   For OUT SETUP requests, the SETUP data phase must also complete before
   *   the SETUP command can be processed.
   *
   * ep0datlen
   *   Length of OUT DATA received in ep0data[]
   */

  struct usb_ctrlreq_s ctrl;  /* Last EP0 request */

  uint8_t ep0data[CONFIG_USBDEV_SETUP_MAXDATASIZE];
  uint16_t ep0datlen;
  uint16_t ep0reqlen;

  /* The endpoint list */

  struct j721e_ep_s eplist[J721E_NENDPOINTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Request queue operations *************************************************/

static struct
j721e_req_s *j721e_rqdequeue(struct j721e_ep_s *privep);
static void j721e_rqenqueue(struct j721e_ep_s *privep,
                             struct j721e_req_s *req);

/* Low level data transfers and request operations */

static void j721e_update_buffer_control(struct j721e_ep_s *privep,
                                         uint32_t and_mask,
                                         uint32_t or_mask);
static int j721e_epwrite(struct j721e_ep_s *privep, uint8_t *buf,
                          uint16_t nbytes);
static int j721e_epread(struct j721e_ep_s *privep, uint16_t nbytes);
static void j721e_abortrequest(struct j721e_ep_s *privep,
                                struct j721e_req_s *privreq,
                                int16_t result);
static void j721e_reqcomplete(struct j721e_ep_s *privep, int16_t result);
static void j721e_txcomplete(struct j721e_ep_s *privep);
static int j721e_wrrequest(struct j721e_ep_s *privep);
static void j721e_rxcomplete(struct j721e_ep_s *privep);
static int j721e_rdrequest(struct j721e_ep_s *privep);

static void j721e_handle_zlp(struct j721e_usbdev_s *priv);

static void j721e_cancelrequests(struct j721e_ep_s *privep);
static struct j721e_ep_s *
j721e_epfindbyaddr(struct j721e_usbdev_s *priv, uint16_t eplog);
static void j721e_dispatchrequest(struct j721e_usbdev_s *priv);
static void j721e_ep0setup(struct j721e_usbdev_s *priv);

/* Interrupt handling */

static void j721e_usbintr_setup(struct j721e_usbdev_s *priv);
static void j721e_usbintr_ep0out(struct j721e_usbdev_s *priv,
                                  struct j721e_ep_s *privep);
static bool j721e_usbintr_buffstat(struct j721e_usbdev_s *priv);
static void j721e_usbintr_busreset(struct j721e_usbdev_s *priv);
static int j721e_usbinterrupt(int irq, void *context, void *arg);

/* Endpoint methods */

static int j721e_epconfigure(struct usbdev_ep_s *ep,
                              const struct usb_epdesc_s *desc,
                              bool last);

static int j721e_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *j721e_epallocreq(struct usbdev_ep_s
                                                  *ep);
static void j721e_epfreereq(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req);
static int j721e_epsubmit(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *privreq);
static int j721e_epcancel(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *privreq);
static int j721e_epstall_exec(struct usbdev_ep_s *ep);
static int j721e_epstall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller methods */

static struct usbdev_ep_s *j721e_allocep(struct usbdev_s *dev,
                                          uint8_t epno, bool in,
                                          uint8_t eptype);
static void j721e_freeep(struct usbdev_s *dev,
                          struct usbdev_ep_s *ep);
static int j721e_getframe(struct usbdev_s *dev);
static int j721e_wakeup(struct usbdev_s *dev);
static int j721e_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int j721e_pullup(struct usbdev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Endpoint methods */

static const struct usbdev_epops_s g_epops =
{
  .configure   = j721e_epconfigure,
  .disable     = j721e_epdisable,
  .allocreq    = j721e_epallocreq,
  .freereq     = j721e_epfreereq,
  .submit      = j721e_epsubmit,
  .cancel      = j721e_epcancel,
  .stall       = j721e_epstall,
};

/* USB controller device methods */

static const struct usbdev_ops_s g_devops =
{
  .allocep     = j721e_allocep,
  .freeep      = j721e_freeep,
  .getframe    = j721e_getframe,
  .wakeup      = j721e_wakeup,
  .selfpowered = j721e_selfpowered,
  .pullup      = j721e_pullup,
};

static struct j721e_usbdev_s g_usbdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: j721e_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 ****************************************************************************/

static struct
j721e_req_s *j721e_rqdequeue(struct j721e_ep_s *privep)
{
  struct j721e_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: j721e_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 ****************************************************************************/

static void j721e_rqenqueue(struct j721e_ep_s *privep,
                             struct j721e_req_s *req)
{
  req->flink = NULL;
  if (!privep->head)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }
}

/****************************************************************************
 * Name: j721e_update_buffer_control
 *
 * Description:
 *   Update DPSRAM buffer control register
 *
 ****************************************************************************/

static void j721e_update_buffer_control(struct j721e_ep_s *privep,
                                         uint32_t and_mask,
                                         uint32_t or_mask)
{
  uint32_t value = 0;

  if (and_mask)
    {
      value = getreg32(privep->buf_ctrl) & and_mask;
    }

  if (or_mask)
    {
      value |= or_mask;
    }

  putreg32(value, privep->buf_ctrl);
}

/****************************************************************************
 * Name: j721e_epwrite
 *
 * Description:
 *   Endpoint write (IN)
 *
 ****************************************************************************/

static int j721e_epwrite(struct j721e_ep_s *privep, uint8_t *buf,
                          uint16_t nbytes)
{
  uint32_t val;
  irqstate_t flags;

  /* Copy the transmit data into DPSRAM */

  memcpy(privep->data_buf, buf, nbytes);

  val = nbytes |
        J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_AVAIL |
        J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_FULL |
        (privep->next_pid ?
         J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA1_PID : 0);

  privep->next_pid = 1 - privep->next_pid;    /* Invert DATA0 <-> DATA1 */

  /* Start the transfer */

  flags = spin_lock_irqsave(NULL);
  j721e_update_buffer_control(privep, 0, val);
  spin_unlock_irqrestore(NULL, flags);

  return nbytes;
}

/****************************************************************************
 * Name: j721e_epread
 *
 * Description:
 *   Endpoint read (OUT)
 *
 ****************************************************************************/

static int j721e_epread(struct j721e_ep_s *privep, uint16_t nbytes)
{
  uint32_t val;
  irqstate_t flags;

  val = nbytes |
        J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_AVAIL |
        (privep->next_pid ?
         J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_DATA1_PID : 0);

  privep->next_pid = 1 - privep->next_pid;    /* Invert DATA0 <-> DATA1 */

  /* Start the transfer */

  flags = spin_lock_irqsave(NULL);
  j721e_update_buffer_control(privep, 0, val);
  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Name: j721e_abortrequest
 *
 * Description:
 *   Discard a request
 *
 ****************************************************************************/

static void j721e_abortrequest(struct j721e_ep_s *privep,
                                struct j721e_req_s *privreq,
                                int16_t result)
{
  usbtrace(TRACE_DEVERROR(J721E_TRACEERR_REQABORTED),
          (uint16_t)privep->epphy);

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 * Name: j721e_reqcomplete
 *
 * Description:
 *   Handle termination of a request.
 *
 ****************************************************************************/

static void j721e_reqcomplete(struct j721e_ep_s *privep, int16_t result)
{
  struct j721e_req_s *privreq;
  int stalled = privep->stalled;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = enter_critical_section();
  privreq = j721e_rqdequeue(privep);
  leave_critical_section(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      if (privep->epphy == 0)
        {
          privep->stalled = privep->dev->stalled;
        }

      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Restore the stalled indication */

      privep->stalled = stalled;
    }
}

/****************************************************************************
 * Name: j721e_txcomplete
 *
 * Description:
 *   Transfer is completed.
 *
 ****************************************************************************/

static void j721e_txcomplete(struct j721e_ep_s *privep)
{
  struct j721e_req_s *privreq;

  privreq = j721e_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_TXREQLOST), privep->epphy);
    }
  else
    {
      privreq->req.xfrd += getreg32(privep->buf_ctrl)
                           & J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN_MASK;

      if (privreq->req.xfrd >= privreq->req.len && !privep->txnullpkt)
        {
          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          privep->txnullpkt = 0;
          j721e_reqcomplete(privep, OK);
        }
    }

  j721e_wrrequest(privep);
}

/****************************************************************************
 * Name: j721e_wrrequest
 *
 * Description:
 *   Send from the next queued write request
 *
 ****************************************************************************/

static int j721e_wrrequest(struct j721e_ep_s *privep)
{
  struct j721e_req_s *privreq;
  uint8_t *buf;
  int nbytes;
  int bytesleft;

  /* Check the request from the head of the endpoint request queue */

  privreq = j721e_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_NULLREQUEST), 0);
      return OK;
    }

  /* Ignore any attempt to send a zero length packet on anything but EP0IN */

  if (privreq->req.len == 0)
    {
      if (privep->epphy == 0)
        {
          j721e_epwrite(privep, NULL, 0);
        }
      else
        {
          usbtrace(TRACE_DEVERROR(J721E_TRACEERR_NULLPACKET), 0);
        }

      return OK;
    }

  /* Get the number of bytes left to be sent in the packet */

  bytesleft = privreq->req.len - privreq->req.xfrd;

  /* Send the next packet if (1) there are more bytes to be sent, or
   * (2) the last packet sent was exactly maxpacketsize (bytesleft == 0)
   */

  usbtrace(TRACE_WRITE(privep->epphy), (uint16_t)bytesleft);
  if (bytesleft > 0 || privep->txnullpkt)
    {
      /* Try to send maxpacketsize -- unless we don't have that many
       * bytes to send.
       */

      privep->txnullpkt = 0;
      if (bytesleft > privep->ep.maxpacket)
        {
          nbytes = privep->ep.maxpacket;
        }
      else
        {
          nbytes = bytesleft;
          if ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
            {
              privep->txnullpkt = (bytesleft == privep->ep.maxpacket);
            }
        }

      /* Send the largest number of bytes that we can in this packet */

      buf = privreq->req.buf + privreq->req.xfrd;
      j721e_epwrite(privep, buf, nbytes);
    }

  return OK;
}

/****************************************************************************
 * Name: j721e_rxcomplete
 *
 * Description:
 *   Notify the upper layer and continue to next receive request.
 *
 ****************************************************************************/

static void j721e_rxcomplete(struct j721e_ep_s *privep)
{
  struct j721e_req_s *privreq;
  uint16_t nrxbytes;

  nrxbytes = getreg32(privep->buf_ctrl)
             & J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN_MASK;

  privreq = j721e_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_RXREQLOST), privep->epphy);
      return;
    }

  memcpy(privreq->req.buf + privreq->req.xfrd, privep->data_buf, nrxbytes);

  privreq->req.xfrd += nrxbytes;

  if (privreq->req.xfrd >= privreq->req.len ||
      nrxbytes < privep->ep.maxpacket)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      j721e_reqcomplete(privep, OK);
    }

  j721e_rdrequest(privep);
}

/****************************************************************************
 * Name: j721e_rdrequest
 *
 * Description:
 *   Receive to the next queued read request
 *
 ****************************************************************************/

static int j721e_rdrequest(struct j721e_ep_s *privep)
{
  struct j721e_req_s *privreq;

  /* Check the request from the head of the endpoint request queue */

  privreq = j721e_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_EPOUTQEMPTY), 0);
      return OK;
    }

  /* Receive the next packet */

  usbtrace(TRACE_READ(privep->epphy), privreq->req.len);

  return j721e_epread(privep, privreq->req.len);
}

/****************************************************************************
 * Name: j721e_handle_zlp
 *
 * Description:
 *   Handle Zero Length Packet to reply to the control transfer
 *
 ****************************************************************************/

static void j721e_handle_zlp(struct j721e_usbdev_s *priv)
{
  struct j721e_ep_s *privep = NULL;

  switch (priv->zlp_stat)
    {
      case J721E_ZLP_NONE:
        return;

      case J721E_ZLP_IN_REPLY:

        /* Reply to control IN  : receive ZLP from EP0 (0x00) */

        privep = &priv->eplist[J721E_EPINDEX(0x00)];
        break;

      case J721E_ZLP_OUT_REPLY:

        /* Reply to control OUT : send ZLP to EP0 (0x80) */

        privep = &priv->eplist[J721E_EPINDEX(0x80)];
        break;

      default:
        DEBUGPANIC();
    }

  usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_HANDLEZLP), privep->ep.eplog);
  privep->next_pid = 1;   /* ZLP is always sent by DATA1 packet */

  if (priv->zlp_stat == J721E_ZLP_IN_REPLY)
    {
      j721e_epread(privep, 0);
    }
  else
    {
      j721e_epwrite(privep, NULL, 0);
    }

  priv->zlp_stat = J721E_ZLP_NONE;
}

/****************************************************************************
 * Name: j721e_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 ****************************************************************************/

static void j721e_cancelrequests(struct j721e_ep_s *privep)
{
  while (!j721e_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (j721e_rqpeek(privep))->req.xfrd);
      j721e_reqcomplete(privep, -ESHUTDOWN);
    }
}

/****************************************************************************
 * Name: j721e_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 ****************************************************************************/

static struct j721e_ep_s *
j721e_epfindbyaddr(struct j721e_usbdev_s *priv, uint16_t eplog)
{
  return &priv->eplist[J721E_EPINDEX(eplog)];
}

/****************************************************************************
 * Name: j721e_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver
 *
 ****************************************************************************/

static void j721e_dispatchrequest(struct j721e_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        priv->ep0data, priv->ep0datlen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(J721E_TRACEERR_STALLEDISPATCH),
                   priv->ctrl.req);
          priv->stalled = true;
        }

      if (!priv->stalled && USB_REQ_ISOUT(priv->ctrl.type))
        {
          priv->zlp_stat = J721E_ZLP_NONE; /* already sent */
        }
    }
}

/****************************************************************************
 * Name: j721e_ep0setup
 *
 * Description:
 *   USB control EP setup event
 *
 ****************************************************************************/

static void j721e_ep0setup(struct j721e_usbdev_s *priv)
{
  struct j721e_ep_s *ep0 = &priv->eplist[0];
  struct j721e_ep_s *privep;
  uint16_t index;
  uint16_t value;
  uint16_t len;

  usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_INTR_SETUP), 0);

  /* Assume NOT stalled */

  ep0->stalled  = 0;
  priv->stalled = 0;

  /* Extract the little-endian 16-bit values to host order */

  index = GETUINT16(priv->ctrl.index);
  value = GETUINT16(priv->ctrl.value);
  len = GETUINT16(priv->ctrl.len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        priv->ctrl.type, priv->ctrl.req, value, index, len);

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      j721e_dispatchrequest(priv);
    }
  else
    {
      /* Handle standard request.  Pick off the things of interest to the
       * USB device controller driver; pass what is left to the class driver
       */

      switch (priv->ctrl.req)
        {
          case USB_REQ_GETSTATUS:
            {
              /* type:  device-to-host;
               *        recipient = device,
               *        interface,
               *        endpoint
               * value: 0
               * index: zero interface endpoint
               * len:   2; data = status
               */

              usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_GETSTATUS),
                       priv->ctrl.req);

              if (len != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
                  value != 0)
                {
                  usbtrace(TRACE_DEVERROR(J721E_TRACEERR_STALLEDGETST),
                           priv->ctrl.req);
                  priv->stalled = true;
                }
              else
                {
                  switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
                    {
                      case USB_REQ_RECIPIENT_ENDPOINT:
                        {
                          usbtrace(TRACE_INTDECODE(
                                   J721E_TRACEINTID_GETENDPOINT),
                                   0);
                          privep = j721e_epfindbyaddr(priv, index);
                          if (!privep)
                            {
                              usbtrace(
                                TRACE_DEVERROR(
                                J721E_TRACEERR_STALLEDGETSTEP),
                                priv->ctrl.type);
                              priv->stalled = true;
                            }
                        }
                        break;

                      case USB_REQ_RECIPIENT_DEVICE:
                      case USB_REQ_RECIPIENT_INTERFACE:
                        usbtrace(TRACE_INTDECODE(
                                 J721E_TRACEINTID_GETIFDEV),
                                 0);
                        break;

                      default:
                        {
                          usbtrace(TRACE_DEVERROR(
                                   J721E_TRACEERR_STALLEDGETSTRECIP),
                                   priv->ctrl.type);
                          priv->stalled = true;
                        }
                        break;
                    }
                }
            }
            break;

          case USB_REQ_CLEARFEATURE:
            {
              /* type:  host-to device;
               *        recipient = device,
               *        interface or endpoint
               * value: feature selector
               * index: zero interface endpoint;
               * len:   zero, data = none
               */

              usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_CLEARFEATURE),
                       (uint16_t)priv->ctrl.req);
              if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                  USB_REQ_RECIPIENT_ENDPOINT)
                {
                  j721e_dispatchrequest(priv);
                }
              else if (value == USB_FEATURE_ENDPOINTHALT &&
                       len == 0 &&
                       (privep = j721e_epfindbyaddr(priv, index)) != NULL)
                {
                  j721e_epstall(&privep->ep, true);
                  j721e_epwrite(ep0, NULL, 0);
                }
              else
                {
                  usbtrace(TRACE_DEVERROR(J721E_TRACEERR_STALLEDCLRFEATURE),
                           priv->ctrl.type);
                  priv->stalled = true;
                }
            }
            break;

          case USB_REQ_SETFEATURE:
            {
              /* type:  host-to-device;
               *        recipient = device,
               *        interface,
               *        endpoint
               * value: feature selector
               * index: zero interface endpoint;
               * len:   0; data = none
               */

              usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_SETFEATURE),
                       priv->ctrl.req);
              if (priv->ctrl.type == USB_REQ_RECIPIENT_DEVICE &&
                  value == USB_FEATURE_TESTMODE)
                {
                  usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_TESTMODE),
                           index);
                }
              else if (priv->ctrl.type != USB_REQ_RECIPIENT_ENDPOINT)
                {
                  j721e_dispatchrequest(priv);
                }
              else if (value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                       (privep = j721e_epfindbyaddr(priv, index)) != NULL)
                {
                  j721e_epstall(&privep->ep, true);
                  j721e_epwrite(ep0, NULL, 0);
                }
              else
                {
                  usbtrace(TRACE_DEVERROR(J721E_TRACEERR_STALLEDSETFEATURE),
                           priv->ctrl.type);
                  priv->stalled = true;
                }
            }
            break;

          case USB_REQ_SETADDRESS:
            {
              /* type:  host-to-device; recipient = device
               * value: device address
               * index: 0
               * len:   0; data = none
               */

              usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_SETADDRESS), value);
              priv->dev_addr = value & 0xff;
            }
            break;

          case USB_REQ_GETDESCRIPTOR:
            /* type:  device-to-host; recipient = device
             * value: descriptor type and index
             * index: 0 or language ID;
             * len:   descriptor len; data = descriptor
             */

          case USB_REQ_SETDESCRIPTOR:
            /* type:  host-to-device; recipient = device
             * value: descriptor type and index
             * index: 0 or language ID;
             * len:   descriptor len; data = descriptor
             */

            {
              usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_GETSETDESC),
                       priv->ctrl.req);
              j721e_dispatchrequest(priv);
            }
            break;

          case USB_REQ_GETCONFIGURATION:
            /* type:  device-to-host; recipient = device
             * value: 0;
             * index: 0;
             * len:   1; data = configuration value
             */

          case USB_REQ_SETCONFIGURATION:
            /* type:  host-to-device; recipient = device
             * value: configuration value
             * index: 0;
             * len:   0; data = none
             */

          case USB_REQ_GETINTERFACE:
            /* type:  device-to-host; recipient = interface
             * value: 0
             * index: interface;
             * len:   1; data = alt interface
             */

          case USB_REQ_SETINTERFACE:
            /* type:  host-to-device; recipient = interface
             * value: alternate setting
             * index: interface;
             * len:   0; data = none
             */

            {
              usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_GETSETIFCONFIG),
                       priv->ctrl.req);
              j721e_dispatchrequest(priv);
            }
            break;

          case USB_REQ_SYNCHFRAME:
              /* type:  device-to-host; recipient = endpoint
               * value: 0
               * index: endpoint;
               * len:   2; data = frame number
               */

            {
              usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_SYNCHFRAME), 0);
              break;
            }

          default:
            {
              usbtrace(TRACE_DEVERROR(J721E_TRACEERR_STALLEDREQUEST),
                       priv->ctrl.req);
              priv->stalled = true;
            }
            break;
        }
    }

  /* Check if the setup processing resulted in a STALL */

  if (priv->stalled)
    {
      j721e_epstall(&priv->eplist[0].ep, false);
      j721e_epstall(&priv->eplist[1].ep, false);
    }
  else if (priv->zlp_stat != J721E_ZLP_NONE)
    {
      j721e_handle_zlp(priv);
    }
}

/****************************************************************************
 * Name: j721e_usbintr_setup
 *
 * Description:
 *   Handle USB SETUP_REQ interrupt
 *
 ****************************************************************************/

static void j721e_usbintr_setup(struct j721e_usbdev_s *priv)
{
  uint16_t len;

  /* Read USB control request data */

  memcpy(&priv->ctrl, (void *)J721E_USBCTRL_DPSRAM_SETUP_PACKET,
         USB_SIZEOF_CTRLREQ);
  len = GETUINT16(priv->ctrl.len);

  /* Reset PID and stall status in setup stage */

  priv->eplist[0].next_pid = 1;
  priv->eplist[1].next_pid = 1;
  priv->eplist[0].stalled = false;
  priv->eplist[1].stalled = false;

  /* ZLP type in status stage */

  priv->zlp_stat = USB_REQ_ISIN(priv->ctrl.type) ? J721E_ZLP_IN_REPLY :
                                                   J721E_ZLP_OUT_REPLY;

  if (USB_REQ_ISOUT(priv->ctrl.type) && len != priv->ep0datlen)
    {
      /* Receive the subsequent OUT data for the setup */

      priv->ep0reqlen = len;
      j721e_epread(&priv->eplist[J721E_EPINDEX(0x00)], len);
    }
  else
    {
      /* Start the setup */

      priv->ep0reqlen = 0;
      j721e_ep0setup(priv);
    }
}

/****************************************************************************
 * Name: j721e_usbintr_ep0out
 *
 * Description:
 *   Handle the end of EP0OUT data transfer
 *
 ****************************************************************************/

static void j721e_usbintr_ep0out(struct j721e_usbdev_s *priv,
                                  struct j721e_ep_s *privep)
{
  int len;

  len = getreg32(privep->buf_ctrl)
        & J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_LEN_MASK;

  if (len == 0)
    {
      privep->next_pid = 1;
      priv->ep0datlen = 0;
      return;
    }

  memcpy(priv->ep0data + priv->ep0datlen, privep->data_buf, len);
  priv->ep0datlen += len;

  if (priv->ep0datlen == priv->ep0reqlen)
    {
      priv->zlp_stat = J721E_ZLP_NONE;
      j721e_ep0setup(priv);
      priv->ep0datlen = 0;
    }
  else
    {
      j721e_epread(privep, J721E_EP0MAXPACKET);
    }
}

/****************************************************************************
 * Name: j721e_usbintr_buffstat
 *
 * Description:
 *   Handle USB BUFF_STATUS interrupt
 *
 ****************************************************************************/

static bool j721e_usbintr_buffstat(struct j721e_usbdev_s *priv)
{
  uint32_t stat = getreg32(J721E_USBCTRL_REGS_BUFF_STATUS);
  uint32_t bit;
  int i;
  struct j721e_ep_s *privep;

  if (stat == 0)
    {
      return false;
    }

  usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_INTR_BUFFSTAT), stat & 0xffff);

  bit = 1;
  for (i = 0; i < 32 && stat != 0; i++)
    {
      if (stat & bit)
        {
          clrbits_reg32(bit, J721E_USBCTRL_REGS_BUFF_STATUS);
          privep = &priv->eplist[J721E_DPTOEP(i)];

          if (i == 1)
            {
              j721e_usbintr_ep0out(priv, privep);
            }
          else
            {
              if (i == 0 && priv->dev_addr != 0)
                {
                  putreg32(priv->dev_addr, J721E_USBCTRL_REGS_ADDR_ENDP);
                  priv->dev_addr = 0;
                }

              if (privep->in)
                {
                  if (!j721e_rqempty(privep))
                    {
                      j721e_txcomplete(privep);
                    }
                  else if (privep->pending_stall)
                    {
                      j721e_epstall_exec(&privep->ep);
                    }
                }
              else
                {
                  j721e_rxcomplete(privep);
                }
            }

          stat &= ~bit;
        }

      bit <<= 1;
    }

  return true;
}

/****************************************************************************
 * Name: j721e_usbintr_busreset
 *
 * Description:
 *   Handle USB BUS_RESET interrupt
 *
 ****************************************************************************/

static void j721e_usbintr_busreset(struct j721e_usbdev_s *priv)
{
  int i;

  usbtrace(TRACE_INTDECODE(J721E_TRACEINTID_INTR_BUSRESET), 0);

  putreg32(0, J721E_USBCTRL_REGS_ADDR_ENDP);
  priv->dev_addr = 0;
  priv->zlp_stat = J721E_ZLP_NONE;
  priv->next_offset = J721E_USBCTRL_DPSRAM_DATA_BUF_OFFSET;

  for (i = 0; i < J721E_NENDPOINTS; i++)
    {
      struct j721e_ep_s *privep = &g_usbdev.eplist[i];

      j721e_cancelrequests(privep);
    }

  j721e_pullup(&g_usbdev.usbdev, false);
  if (g_usbdev.driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  clrbits_reg32(J721E_USBCTRL_REGS_SIE_STATUS_BUS_RESET,
                J721E_USBCTRL_REGS_SIE_STATUS);
}

/****************************************************************************
 * Name: j721e_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int j721e_usbinterrupt(int irq, void *context, void *arg)
{
  struct j721e_usbdev_s *priv = (struct j721e_usbdev_s *)arg;
  uint32_t stat;

  stat = getreg32(J721E_USBCTRL_REGS_INTS);

  usbtrace(TRACE_INTENTRY(J721E_TRACEINTID_USBINTERRUPT), 0);

  if (stat & J721E_USBCTRL_REGS_INTR_BUFF_STATUS)
    {
      while (j721e_usbintr_buffstat(priv))
        ;
    }

  if (stat & J721E_USBCTRL_REGS_INTR_SETUP_REQ)
    {
      clrbits_reg32(J721E_USBCTRL_REGS_SIE_STATUS_SETUP_REC,
                    J721E_USBCTRL_REGS_SIE_STATUS);

      j721e_usbintr_setup(priv);
    }

  if (stat & J721E_USBCTRL_REGS_INTR_BUS_RESET)
    {
      clrbits_reg32(J721E_USBCTRL_REGS_SIE_STATUS_BUS_RESET,
                    J721E_USBCTRL_REGS_SIE_STATUS);

      j721e_usbintr_busreset(priv);
    }

  usbtrace(TRACE_INTEXIT(J721E_TRACEINTID_USBINTERRUPT), 0);

  return OK;
}

/****************************************************************************
 * Endpoint Methods
 ****************************************************************************/

/****************************************************************************
 * Name: j721e_epconfigure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this is the last endpoint to be configured.  Some
 *          hardware needs to take special action when all of the endpoints
 *          have been configured.
 *
 ****************************************************************************/

static int j721e_epconfigure(struct usbdev_ep_s *ep,
                              const struct usb_epdesc_s *desc, bool last)
{
  struct j721e_ep_s *privep = (struct j721e_ep_s *)ep;
  struct j721e_usbdev_s *priv = privep->dev;
  int eptype;
  uint16_t maxpacket;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  eptype = desc->attr & USB_EP_ATTR_XFERTYPE_MASK;
  maxpacket = GETUINT16(desc->mxpacketsize);

  uinfo("config: EP%d %s %d maxpacket=%d\n", privep->epphy,
        privep->in ? "IN" : "OUT", eptype, maxpacket);

  if (desc)
    {
      privep->ep.maxpacket = GETUINT16(desc->mxpacketsize);
    }

  if (privep->epphy != 0)
    {
      /* Configure the EP data buffer address
       * (No need for EP0 because it has the dedicated buffer)
       */

      privep->data_buf = (uint8_t *)(J721E_USBCTRL_DPSRAM_BASE +
                                     priv->next_offset);
      priv->next_offset =
                     (priv->next_offset + privep->ep.maxpacket + 63) & ~63;

      /* Enable EP */

      putreg32(J721E_USBCTRL_DPSRAM_EP_CTRL_ENABLE |
               J721E_USBCTRL_DPSRAM_EP_CTRL_INT_1BUF |
               (eptype << J721E_USBCTRL_DPSRAM_EP_CTRL_EP_TYPE_SHIFT) |
               ((uint32_t)privep->data_buf &
                J721E_USBCTRL_DPSRAM_EP_CTRL_EP_ADDR_MASK),
               privep->ep_ctrl);
    }

  return OK;
}

/****************************************************************************
 * Name: j721e_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int j721e_epdisable(struct usbdev_ep_s *ep)
{
  struct j721e_ep_s *privep = (struct j721e_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPDISABLE, privep->epphy);
  uinfo("EP%d\n", privep->epphy);

  flags = enter_critical_section();

  privep->ep.maxpacket = 64;
  privep->stalled = false;
  privep->next_pid = 0;
  putreg32(0, privep->buf_ctrl);

  /* Cancel all queued requests */

  j721e_cancelrequests(privep);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: j721e_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static struct usbdev_req_s *j721e_epallocreq(struct usbdev_ep_s *ep)
{
  struct j721e_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, ((struct j721e_ep_s *)ep)->epphy);

  privreq = (struct j721e_req_s *)
            kmm_malloc(sizeof(struct j721e_req_s));

  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct j721e_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: j721e_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void j721e_epfreereq(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req)
{
  struct j721e_req_s *privreq = (struct j721e_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((struct j721e_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

/****************************************************************************
 * Name: j721e_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int j721e_epsubmit(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *req)
{
  struct j721e_req_s *privreq = (struct j721e_req_s *)req;
  struct j721e_ep_s *privep = (struct j721e_ep_s *)ep;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->ep.eplog);

  req->result = -EINPROGRESS;
  req->xfrd = 0;

  flags = enter_critical_section();

  if (privep->stalled && privep->in)
    {
      j721e_abortrequest(privep, privreq, -EBUSY);
      ret = -EBUSY;
    }

  /* Handle IN (device-to-host) requests */

  else if (privep->in)
    {
      /* Add the new request to the request queue for the IN endpoint */

      bool empty = j721e_rqempty(privep);

      j721e_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);

      if (empty)
        {
          j721e_wrrequest(privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      bool empty = j721e_rqempty(privep);

      privep->txnullpkt = 0;
      j721e_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);

      /* This there a incoming data pending the availability of a request? */

      if (empty)
        {
          ret = j721e_rdrequest(privep);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: j721e_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int j721e_epcancel(struct usbdev_ep_s *ep,
                           struct usbdev_req_s *req)
{
  struct j721e_ep_s *privep = (struct j721e_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  /* Remove request from req_queue */

  flags = enter_critical_section();
  j721e_cancelrequests(privep);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: j721e_epstall_exec
 *
 * Description:
 *   Stall endpoint immediately
 *
 ****************************************************************************/

static int j721e_epstall_exec(struct usbdev_ep_s *ep)
{
  struct j721e_ep_s *privep = (struct j721e_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_EPSTALL, privep->epphy);

  flags = spin_lock_irqsave(NULL);

  if (privep->epphy == 0)
    {
      setbits_reg32(privep->in ?
                     J721E_USBCTRL_REGS_EP_STALL_ARM_EP0_IN :
                     J721E_USBCTRL_REGS_EP_STALL_ARM_EP0_OUT,
                    J721E_USBCTRL_REGS_EP_STALL_ARM);
    }

  j721e_update_buffer_control(privep,
                    0,
                    J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_STALL);

  privep->pending_stall = false;

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

/****************************************************************************
 * Name: j721e_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 ****************************************************************************/

static int j721e_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct j721e_ep_s *privep = (struct j721e_ep_s *)ep;
  struct j721e_usbdev_s *priv = privep->dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  if (resume)
    {
      usbtrace(TRACE_EPRESUME, privep->epphy);
      privep->stalled = false;
      if (privep->epphy == 0)
        {
          clrbits_reg32(privep->in ?
                         J721E_USBCTRL_REGS_EP_STALL_ARM_EP0_IN :
                         J721E_USBCTRL_REGS_EP_STALL_ARM_EP0_OUT,
                        J721E_USBCTRL_REGS_EP_STALL_ARM);
        }

      j721e_update_buffer_control(privep,
                        ~(J721E_USBCTRL_DPSRAM_EP_BUFF_CTRL_STALL),
                        0);

      privep->next_pid = 0;
      priv->zlp_stat = J721E_ZLP_NONE;
    }
  else
    {
      privep->stalled = true;

      if (privep->epphy == 0 && !j721e_rqempty(privep))
        {
          /* EP0 IN Transfer ongoing : postpone the stall until the end */

          privep->pending_stall = true;
        }
      else
        {
          /* Stall immediately */

          j721e_epstall_exec(ep);
        }

      priv->zlp_stat = J721E_ZLP_NONE;
    }

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Device Methods
 ****************************************************************************/

/****************************************************************************
 * Name: j721e_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).
 *            Zero means that any endpoint matching the other requirements
 *            will suffice.  The assigned endpoint can be found in the eplog
 *            field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.
 *            One of {USB_EP_ATTR_XFER_ISOC,
 *                    USB_EP_ATTR_XFER_BULK,
 *                    USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static struct usbdev_ep_s *j721e_allocep(struct usbdev_s *dev,
                                          uint8_t eplog, bool in,
                                          uint8_t eptype)
{
  struct j721e_usbdev_s *priv = (struct j721e_usbdev_s *)dev;
  struct j721e_ep_s *privep;
  int epphy;
  int epindex;
  int dpindex;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  epphy = USB_EPNO(eplog);
  epindex = J721E_EPINDEX(eplog);
  dpindex = J721E_DPINDEX(eplog);

  if ((priv->used & 1 << epphy) && (epphy != 0))
    {
      uinfo("ep is still used\n");
      return NULL;
    }

  priv->used |= 1 << epphy;

  privep = &priv->eplist[epindex];
  privep->in = in;
  privep->type = eptype;
  privep->epphy = epphy;
  privep->ep.eplog = eplog;

  privep->next_pid = 0;
  privep->stalled = false;
  privep->buf_ctrl = J721E_USBCTRL_DPSRAM_EP_BUF_CTRL(dpindex);

  if (epphy == 0)
    {
      privep->data_buf = (uint8_t *)J721E_USBCTRL_DPSRAM_EP0_BUF_0;
      privep->ep_ctrl = 0;
    }
  else
    {
      privep->ep_ctrl = J721E_USBCTRL_DPSRAM_EP_CTRL(dpindex);
    }

  return &privep->ep;
}

/****************************************************************************
 * Name: j721e_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void j721e_freeep(struct usbdev_s *dev,
                          struct usbdev_ep_s *ep)
{
  struct j721e_usbdev_s *priv = (struct j721e_usbdev_s *)dev;
  struct j721e_ep_s *privep = (struct j721e_ep_s *)ep;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  priv->used &= ~(1 << privep->epphy);
}

/****************************************************************************
 * Name: j721e_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int j721e_getframe(struct usbdev_s *dev)
{
  usbtrace(TRACE_DEVGETFRAME, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  return (int)(getreg32(J721E_USBCTRL_REGS_SOF_RD) &
               J721E_USBCTRL_REGS_SOF_RD_COUNT_MASK);
}

/****************************************************************************
 * Name: j721e_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 ****************************************************************************/

static int j721e_wakeup(struct usbdev_s *dev)
{
  usbtrace(TRACE_DEVWAKEUP, 0);

  setbits_reg32(J721E_USBCTRL_REGS_SIE_CTRL_RESUME,
                J721E_USBCTRL_REGS_SIE_CTRL);

  return OK;
}

/****************************************************************************
 * Name: j721e_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 ****************************************************************************/

static int j721e_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct j721e_usbdev_s *priv = (struct j721e_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: j721e_pullup
 *
 * Description:
 *    Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

static int j721e_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  if (enable)
    {
      setbits_reg32(J721E_USBCTRL_REGS_SIE_CTRL_PULLUP_EN,
                    J721E_USBCTRL_REGS_SIE_CTRL);
    }
  else
    {
      clrbits_reg32(J721E_USBCTRL_REGS_SIE_CTRL_PULLUP_EN,
                    J721E_USBCTRL_REGS_SIE_CTRL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usbinitialize
 *
 * Description:
 *   Initialize the USB driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  int i;

  usbtrace(TRACE_DEVINIT, 0);

  putreg32(0, J721E_USBCTRL_REGS_ADDR_ENDP);

  /* Initialize driver instance */

  memset(&g_usbdev, 0, sizeof(struct j721e_usbdev_s));

  g_usbdev.usbdev.ops = &g_devops;
  g_usbdev.usbdev.ep0 = &g_usbdev.eplist[0].ep;

  g_usbdev.dev_addr = 0;
  g_usbdev.next_offset = J721E_USBCTRL_DPSRAM_DATA_BUF_OFFSET;

  for (i = 0; i < J721E_NENDPOINTS; i++)
    {
      g_usbdev.eplist[i].ep.ops = &g_epops;
      g_usbdev.eplist[i].ep.maxpacket = 64;
      g_usbdev.eplist[i].dev = &g_usbdev;
      g_usbdev.eplist[i].epphy = 0;
      g_usbdev.eplist[i].head = NULL;
      g_usbdev.eplist[i].tail = NULL;
      g_usbdev.eplist[i].ep.eplog = 0;
    }

  if (irq_attach(J721E_USBCTRL_IRQ, j721e_usbinterrupt, &g_usbdev) != 0)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_IRQREGISTRATION),
               (uint16_t)J721E_USBCTRL_IRQ);
      return;
    }
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  int ret = -1;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* Hook up the driver */

  g_usbdev.driver = driver;

  setbits_reg32(J721E_RESETS_RESET_USBCTRL, J721E_RESETS_RESET);
  clrbits_reg32(J721E_RESETS_RESET_USBCTRL, J721E_RESETS_RESET);

  memset((void *)J721E_USBCTRL_DPSRAM_BASE, 0, 0x1000);

  putreg32(J721E_USBCTRL_REGS_USB_MUXING_SOFTCON |
           J721E_USBCTRL_REGS_USB_MUXING_TO_PHY,
           J721E_USBCTRL_REGS_USB_MUXING);
  putreg32(J721E_USBCTRL_REGS_USB_PWR_VBUS_DETECT |
           J721E_USBCTRL_REGS_USB_PWR_VBUS_DETECT_OVERRIDE_EN,
           J721E_USBCTRL_REGS_USB_PWR);

  j721e_allocep(&g_usbdev.usbdev, 0x00, 0, USB_EP_ATTR_XFER_CONTROL);
  j721e_allocep(&g_usbdev.usbdev, 0x80, 1, USB_EP_ATTR_XFER_CONTROL);

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
      return ret;
    }

  g_usbdev.usbdev.speed = USB_SPEED_FULL;

  putreg32(J721E_USBCTRL_REGS_MAIN_CTRL_CONTROLLER_EN,
           J721E_USBCTRL_REGS_MAIN_CTRL);

  /* Enable interrupt */

  putreg32(J721E_USBCTRL_REGS_SIE_CTRL_EP0_INT_1BUF,
           J721E_USBCTRL_REGS_SIE_CTRL);
  putreg32(J721E_USBCTRL_REGS_INTR_BUFF_STATUS |
           J721E_USBCTRL_REGS_INTR_BUS_RESET |
           J721E_USBCTRL_REGS_INTR_SETUP_REQ,
           J721E_USBCTRL_REGS_INTE);

  up_enable_irq(J721E_USBCTRL_IRQ);

  return OK;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver. If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  struct j721e_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(J721E_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_DEVUNREGISTER, 0);

  flags = spin_lock_irqsave(NULL);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable interrupts */

  up_disable_irq(J721E_USBCTRL_IRQ);

  /* Disconnect device */

  j721e_pullup(&priv->usbdev, false);

  /* Unhook the driver */

  priv->driver = NULL;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}
