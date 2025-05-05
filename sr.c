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

int IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (0);
  else
    return (1);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[SEQSPACE];
static int acked[SEQSPACE];
static int base;
static int nextseqnum;
static int timer_active;

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  if (((nextseqnum - base + SEQSPACE) % SEQSPACE) < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    sendpkt.seqnum = nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    buffer[nextseqnum % SEQSPACE] = sendpkt;
    acked[nextseqnum % SEQSPACE] = 0;

    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    if (!timer_active) {
      starttimer(A, RTT);
      timer_active = 1;
    }

    nextseqnum = (nextseqnum + 1) % SEQSPACE;
  }
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
  int win_start = base;
  int win_end = (base + WINDOWSIZE) % SEQSPACE;
  int in_window = 0;
  int ack = packet.acknum;

  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", ack);
    total_ACKs_received++;

    if (win_start < win_end)
      in_window = (ack >= win_start && ack < win_end);
    else
      in_window = (ack >= win_start || ack < win_end);

    if (in_window && !acked[ack % SEQSPACE]) {
      acked[ack % SEQSPACE] = 1;
      new_ACKs++;
      
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", ack);
      
      stoptimer(A);
      
      while (acked[base % SEQSPACE]) {
        acked[base % SEQSPACE] = 0;
        base = (base + 1) % SEQSPACE;
      }
      
      if (base != nextseqnum) {
        starttimer(A, RTT);
        timer_active = 1;
      } else {
        timer_active = 0;
      }
    }
    else if (in_window && acked[ack % SEQSPACE]) {
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
  int i;
  
  if (base == nextseqnum) {
    timer_active = 0;
    return;
  }
  
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");
  
  for (i = 0; i < WINDOWSIZE; i++) {
    int seq = (base + i) % SEQSPACE;
    if (!acked[seq % SEQSPACE] && seq != nextseqnum) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", seq);
      
      tolayer3(A, buffer[seq % SEQSPACE]);
      packets_resent++;
      
      starttimer(A, RTT);
      timer_active = 1;
      return;
    }
  }
  
  timer_active = 0;
}



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  
  base = 0;
  nextseqnum = 0;
  timer_active = 0;
  
  for (i = 0; i < SEQSPACE; i++) {
    acked[i] = 0;
  }
}



/********* Receiver (B) variables and procedures ************/

static int expected;

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  
  if (!IsCorrupted(packet)) {
    if (packet.seqnum == expected) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      
      packets_received++;
      tolayer5(B, packet.payload);
      
      expected = (expected + 1) % SEQSPACE;
    } else {
      if (TRACE > 0)
        printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    }
  } else {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
  }
  
  sendpkt.seqnum = NOTINUSE;
  sendpkt.acknum = packet.seqnum;
  
  for (i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';
  
  sendpkt.checksum = ComputeChecksum(sendpkt);
  
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expected = 0;
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
