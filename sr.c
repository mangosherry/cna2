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
static int timer_status[WINDOWSIZE];   /* array to track which packets have timers */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int ack_received[SEQSPACE];     /* track which sequence numbers have been acked */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    windowcount++;
    
    /* set timer for this packet */
    timer_status[windowlast] = 1;
    
    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if not already running */
    starttimer(A, RTT);

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
  int i, seq_pos = -1;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* mark this sequence number as acknowledged */
    ack_received[packet.acknum] = 1;
    
    /* find the position of this ACK in our window */
    for (i = 0; i < windowcount; i++) {
      if (buffer[(windowfirst + i) % WINDOWSIZE].seqnum == packet.acknum) {
        seq_pos = i;
        break;
      }
    }
    
    /* if found in window, mark as new ACK */
    if (seq_pos != -1) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", packet.acknum);
      new_ACKs++;
      
      /* stop timer for this packet */
      stoptimer(A);
      timer_status[(windowfirst + seq_pos) % WINDOWSIZE] = 0;
      
      /* If it's the first packet in window, we can slide window */
      if (seq_pos == 0) {
        /* slide window until reaching unacked packet */
        i = 0;
        while (i < windowcount && ack_received[buffer[(windowfirst + i) % WINDOWSIZE].seqnum] == 1) {
          windowfirst = (windowfirst + 1) % WINDOWSIZE;
          windowcount--;
          i++;
        }
      }
      
      /* restart timer if needed for oldest unacked packet */
      if (windowcount > 0) {
        starttimer(A, RTT);
      }
    }
    else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* Only resend the oldest unacked packet */
  if (windowcount > 0) {
    if (TRACE > 0)
      printf("---A: resending packet %d\n", buffer[windowfirst].seqnum);
    
    tolayer3(A, buffer[windowfirst]);
    packets_resent++;
    
    /* restart timer */
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
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.
                     new packets are placed in winlast + 1
                     so initially this is set to -1
                   */
  windowcount = 0;
  
  /* initialize ack tracking array */
  for (i = 0; i < SEQSPACE; i++) {
    ack_received[i] = 0;
  }
  
  /* initialize timer status array */
  for (i = 0; i < WINDOWSIZE; i++) {
    timer_status[i] = 0;
  }
}



/********* Receiver (B) variables and procedures ************/

static int expectedseqnum;             /* the sequence number expected next by the receiver */
static int B_nextseqnum;               /* the sequence number for the next packets sent by B */
static struct pkt B_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static int B_received[SEQSPACE];       /* track which sequence numbers have been received */
static int B_window_base;              /* base of the receiving window */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int in_window = 0;
  int buffer_index;

  /* check if packet is in receiving window */
  int packet_seqnum = packet.seqnum;
  int window_end = (B_window_base + WINDOWSIZE - 1) % SEQSPACE;
  
  /* check if packet is within the window */
  if (B_window_base <= window_end) {
    /* window hasn't wrapped around */
    in_window = (packet_seqnum >= B_window_base && packet_seqnum <= window_end);
  } else {
    /* window has wrapped around */
    in_window = (packet_seqnum >= B_window_base || packet_seqnum <= window_end);
  }
  
  /* if not corrupted and within receiving window */
  if (!IsCorrupted(packet) && in_window) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    
    /* mark this packet as received */
    B_received[packet.seqnum] = 1;
    
    /* store the packet */
    buffer_index = (packet.seqnum - B_window_base + SEQSPACE) % SEQSPACE;
    if (buffer_index < WINDOWSIZE) {
      B_buffer[buffer_index] = packet;
    }
    
    /* if it's the base of the window, deliver it and any consecutive packets */
    if (packet.seqnum == B_window_base) {
      packets_received++;
      tolayer5(B, packet.payload);
      
      /* slide window and deliver buffered packets in sequence */
      B_window_base = (B_window_base + 1) % SEQSPACE;
      i = 1;
      while (B_received[B_window_base] == 1) {
        packets_received++;
        tolayer5(B, B_buffer[(i-1) % WINDOWSIZE].payload);
        B_received[B_window_base] = 0; /* reset for future reuse */
        B_window_base = (B_window_base + 1) % SEQSPACE;
        i++;
      }
    }
    
    /* Send ACK for the received packet */
    sendpkt.acknum = packet.seqnum;
  }
  else if (IsCorrupted(packet)) {
    /* packet is corrupted, don't send an ACK */
    if (TRACE > 0)
      printf("----B: packet corrupted, don't send ACK!\n");
    return;
  }
  else {
    /* packet outside window - could be a retransmission of an already ACKed packet */
    if (TRACE > 0)
      printf("----B: packet outside the window, send ACK!\n");
    sendpkt.acknum = packet.seqnum;
  }

  /* create ACK packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  /* we don't have any data to send.  fill payload with 0's */
  for (i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';

  /* computer checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt);

  /* send out packet */
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  
  expectedseqnum = 0;
  B_nextseqnum = 1;
  B_window_base = 0;
  
  /* initialize received array */
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
