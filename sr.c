#include <stdlib.h>
#include <stdio.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 12     /* the min sequence space for SR must be at least 2*windowsize */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

#ifndef bool
#define bool int
#define true 1
#define false 0
#endif

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ )
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int timers[WINDOWSIZE];         /* indicates if timer is running for packet */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int A_base;                     /* base of the sender window */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    windowcount++;
    
    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer for this packet if not already running */
    if (timers[windowlast] == 0) {
      starttimer(A, RTT);
      timers[windowlast] = 1;
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int i, pos;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* Find position of this ACK in our window buffer */
    pos = -1;
    for (i = 0; i < windowcount; i++) {
      if (buffer[(windowfirst + i) % WINDOWSIZE].seqnum == packet.acknum) {
        pos = i;
        break;
      }
    }

    /* If packet is in our window, process ACK */
    if (pos != -1) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", packet.acknum);
      new_ACKs++;

      /* Stop timer for this packet */
      stoptimer(A);
      timers[(windowfirst + pos) % WINDOWSIZE] = 0;
      
      /* Mark this packet as ACKed by moving it out of window */
      if (pos == 0) {
        /* It's the first packet in window, slide window */
        windowfirst = (windowfirst + 1) % WINDOWSIZE;
        windowcount--;
        
        /* See if we can slide further (cumulative effect) */
        while (windowcount > 0 && timers[windowfirst] == 2) {
          windowfirst = (windowfirst + 1) % WINDOWSIZE;
          windowcount--;
        }
        
        /* Update A_base */
        if (windowcount > 0) {
          A_base = buffer[windowfirst].seqnum;
        } else {
          A_base = A_nextseqnum;
        }
      } else {
        /* Mark this packet as acked but can't remove it yet */
        timers[(windowfirst + pos) % WINDOWSIZE] = 2; /* 2 means ACKed but not removed */
      }
      
      /* If there are still packets in window, restart timer */
      if (windowcount > 0) {
        for (i = 0; i < windowcount; i++) {
          if (timers[(windowfirst + i) % WINDOWSIZE] == 1) {
            starttimer(A, RTT);
            break;
          }
        }
      }
    } else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
    }
  } else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;
  
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* Resend all packets that have timers running */
  for (i = 0; i < windowcount; i++) {
    if (timers[(windowfirst + i) % WINDOWSIZE] == 1) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[(windowfirst + i) % WINDOWSIZE].seqnum);
      
      tolayer3(A, buffer[(windowfirst + i) % WINDOWSIZE]);
      packets_resent++;
    }
  }
  
  /* Restart timer */
  if (windowcount > 0) {
    starttimer(A, RTT);
  }
}



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  A_base = 0;        /* Window base starts at 0 */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.
                     new packets are placed in winlast + 1
                     so initially this is set to -1
                   */
  windowcount = 0;
  
  /* initialize timer status array */
  for (i = 0; i < WINDOWSIZE; i++) {
    timers[i] = 0;
  }
}



/********* Receiver (B) variables and procedures ************/

static struct pkt B_buffer[SEQSPACE]; /* buffer for out-of-order packets */
static int B_received[SEQSPACE];     /* track which sequence numbers have been received */
static int B_nextseqnum;             /* the sequence number for the next packets sent by B */
static int B_base;                   /* base of the receiving window */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int in_window;

  /* Check if packet is in receiving window */
  in_window = 0;
  if ((packet.seqnum >= B_base) && (packet.seqnum < B_base + WINDOWSIZE)) {
    in_window = 1;
  } else if ((B_base + WINDOWSIZE > SEQSPACE) && 
             (packet.seqnum < (B_base + WINDOWSIZE) % SEQSPACE)) {
    /* Handle window wrap-around */
    in_window = 1;
  }

  /* if not corrupted and received packet is in window */
  if (!IsCorrupted(packet) && in_window) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    
    /* If not already received, store it */
    if (B_received[packet.seqnum] == 0) {
      /* Mark as received and store packet */
      B_received[packet.seqnum] = 1;
      B_buffer[packet.seqnum] = packet;
      
      /* If packet is at window base, deliver it and any consecutive packets */
      if (packet.seqnum == B_base) {
        packets_received++;
        tolayer5(B, packet.payload);
        
        /* Try to advance window and deliver buffered packets in sequence */
        B_base = (B_base + 1) % SEQSPACE;
        while (B_received[B_base] == 1) {
          packets_received++;
          tolayer5(B, B_buffer[B_base].payload);
          B_received[B_base] = 0; /* Reset for reuse */
          B_base = (B_base + 1) % SEQSPACE;
        }
      }
    }
    
    /* Send ACK for this packet */
    sendpkt.acknum = packet.seqnum;
  } else if (IsCorrupted(packet)) {
    /* packet is corrupted, don't send ACK */
    if (TRACE > 0)
      printf("----B: packet corrupted, don't send ACK!\n");
    return;
  } else {
    /* Packet outside window - likely a duplicate that was already ACKed and delivered */
    if (TRACE > 0)
      printf("----B: packet outside the window, send ACK!\n");
    sendpkt.acknum = packet.seqnum;
  }

  /* Create ACK packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  /* Fill payload with 0's */
  for (i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';

  /* Computer checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt);

  /* Send out packet */
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  
  B_nextseqnum = 1;
  B_base = 0;
  
  /* Initialize received array */
  for (i = 0; i < SEQSPACE; i++) {
    B_received[i] = 0;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
