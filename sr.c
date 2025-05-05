#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

#define RTT  16.0
#define WINDOWSIZE 6
#define SEQSPACE 256
#define NOTINUSE (-1)

int ComputeChecksum(struct pkt packet) {
    int checksum = packet.seqnum + packet.acknum;
    for (int i = 0; i < 20; i++)
        checksum += (int)(packet.payload[i]);
    return checksum;
}

bool IsCorrupted(struct pkt packet) {
    return packet.checksum != ComputeChecksum(packet);
}

/********* Sender (A) variables and functions ************/
static struct pkt buffer[SEQSPACE];
static bool acked[SEQSPACE];
static int base = 0;
static int nextseqnum = 0;

void A_output(struct msg message) {
    if (nextseqnum < base + WINDOWSIZE) {
        struct pkt pkt;
        pkt.seqnum = nextseqnum;
        pkt.acknum = NOTINUSE;
        for (int i = 0; i < 20; i++)
            pkt.payload[i] = message.data[i];
        pkt.checksum = ComputeChecksum(pkt);
        buffer[nextseqnum % SEQSPACE] = pkt;
        acked[nextseqnum % SEQSPACE] = false;
        tolayer3(A, pkt);
        if (base == nextseqnum)
            starttimer(A, RTT);
        nextseqnum++;
    }
}

void A_input(struct pkt packet) {
    if (!IsCorrupted(packet)) {
        int seq = packet.acknum % SEQSPACE;
        acked[seq] = true;
        while (acked[base % SEQSPACE]) {
            acked[base % SEQSPACE] = false;
            base++;
        }
        stoptimer(A);
        if (base != nextseqnum)
            starttimer(A, RTT);
    }
}

void A_timerinterrupt(void) {
    starttimer(A, RTT);
    for (int i = base; i < nextseqnum; i++)
        tolayer3(A, buffer[i % SEQSPACE]);
}

void A_init(void) {
    base = 0;
    nextseqnum = 0;
    for (int i = 0; i < SEQSPACE; i++)
        acked[i] = false;
}

/********* Receiver (B) variables and functions ************/
static int recv_base;
static struct pkt recv_buffer[SEQSPACE];
static bool received[SEQSPACE];

void B_input(struct pkt packet) {
    if (!IsCorrupted(packet)) {
        int seq = packet.seqnum % SEQSPACE;
        if (seq >= recv_base && seq < recv_base + WINDOWSIZE) {
            received[seq] = true;
            recv_buffer[seq] = packet;
            struct pkt ack_pkt;
            ack_pkt.acknum = seq;
            ack_pkt.seqnum = NOTINUSE;
            for (int i = 0; i < 20; i++)
                ack_pkt.payload[i] = 0;
            ack_pkt.checksum = ComputeChecksum(ack_pkt);
            tolayer3(B, ack_pkt);

            while (received[recv_base]) {
                tolayer5(B, recv_buffer[recv_base].payload);
                received[recv_base] = false;
                recv_base++;
            }
        } else if (seq < recv_base) {
            struct pkt ack_pkt;
            ack_pkt.acknum = seq;
            ack_pkt.seqnum = NOTINUSE;
            for (int i = 0; i < 20; i++)
                ack_pkt.payload[i] = 0;
            ack_pkt.checksum = ComputeChecksum(ack_pkt);
            tolayer3(B, ack_pkt);
        }
    }
}

void B_init(void) {
    recv_base = 0;
    for (int i = 0; i < SEQSPACE; i++)
        received[i] = false;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}

