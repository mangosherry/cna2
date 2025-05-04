#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
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
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
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

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

#define MAX_SEQ 256

static struct pkt buffer[MAX_SEQ];
static bool acked[MAX_SEQ];
static bool timer_active[MAX_SEQ];
static float timer_start[MAX_SEQ];

static int base = 0;
static int nextseqnum = 0;

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  if (nextseqnum < base + WINDOWSIZE) {
    struct pkt pkt;
    pkt.seqnum = nextseqnum;
    pkt.acknum = NOTINUSE;
    for (int i = 0; i < 20; i++)
      pkt.payload[i] = message.data[i];
    pkt.checksum = ComputeChecksum(pkt);
    buffer[nextseqnum % MAX_SEQ] = pkt;
    acked[nextseqnum % MAX_SEQ] = false;
    tolayer3(A, pkt);
    packets_sent++;

    if (!timer_active[nextseqnum % MAX_SEQ]) {
      timer_active[nextseqnum % MAX_SEQ] = true;
      timer_start[nextseqnum % MAX_SEQ] = get_sim_time();
    }

    nextseqnum++;
  } else {
    window_full++;
  }
}



/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  if (!IsCorrupted(packet)) {
    int seq = packet.acknum % MAX_SEQ;
    if (!acked[seq]) {
      acked[seq] = true;
      timer_active[seq] = false;
      total_ACKs_received++;
      new_ACKs++;

      if (seq == base % MAX_SEQ) {
        while (acked[base % MAX_SEQ]) {
          base++;
        }
      }
    }
  }
}


/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  float now = get_sim_time();
  for (int i = base; i < nextseqnum; i++) {
    int seq = i % MAX_SEQ;
    if (timer_active[seq] && (now - timer_start[seq] >= RTT)) {
      tolayer3(A, buffer[seq]);
      packets_resent++;
      timer_start[seq] = now;
    }
  }
}




/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.
		     new packets are placed in winlast + 1
		     so initially this is set to -1
		   */
  windowcount = 0;
}



/********* Receiver (B)  variables and procedures ************/

#define RECV_WINDOW_SIZE 6
#define MAX_SEQ 1000

static int recv_base;                    
static struct pkt recv_buffer[MAX_SEQ];  
static bool received[MAX_SEQ];           
static int B_nextseqnum;                 

void B_input(struct pkt packet)
{
  struct pkt ack_pkt;
  int i;

  if (IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----B: corrupted packet received, ignoring\n");
    return;
  }

  int seq = packet.seqnum;


  if (seq >= recv_base && seq < recv_base + RECV_WINDOW_SIZE) {
    if (!received[seq]) {
      received[seq] = true;
      recv_buffer[seq] = packet;

      if (TRACE > 0)
        printf("----B: received packet %d and buffered\n", seq);
    }

    
    ack_pkt.seqnum = B_nextseqnum;
    ack_pkt.acknum = seq;
    B_nextseqnum = (B_nextseqnum + 1) % 2;
    for (i = 0; i < 20; i++) ack_pkt.payload[i] = 0;
    ack_pkt.checksum = ComputeChecksum(ack_pkt);
    tolayer3(B, ack_pkt);

    
    while (received[recv_base]) {
      tolayer5(B, recv_buffer[recv_base].payload);
      received[recv_base] = false;
      recv_base++;
    }
  }
  else if (seq < recv_base) {
    
    if (TRACE > 0)
      printf("----B: duplicate packet %d received, resend ACK\n", seq);
    ack_pkt.seqnum = B_nextseqnum;
    ack_pkt.acknum = seq;
    B_nextseqnum = (B_nextseqnum + 1) % 2;
    for (i = 0; i < 20; i++) ack_pkt.payload[i] = 0;
    ack_pkt.checksum = ComputeChecksum(ack_pkt);
    tolayer3(B, ack_pkt);
  }
  else {
    if (TRACE > 0)
      printf("----B: packet %d outside window, dropped\n", seq);
  }
}

void B_init(void)
{
  recv_base = 0;
  B_nextseqnum = 1;
  for (int i = 0; i < MAX_SEQ; i++) {
    received[i] = false;
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
