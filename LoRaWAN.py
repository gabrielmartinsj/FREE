#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
 An updated version of LoRaSim 0.2.1 to simulate collisions in confirmable
 LoRaWAN with a datasize target
 author: Khaled Abdelfadeel khaled.abdelfadeel@mycit.ie
"""

"""
 SYNOPSIS:
   ./confirmablelorawan.py <nodes> <avgsend> <datasize> <maxDist> <number of BS> <collision> <randomseed> 
 DESCRIPTION:
    nodes
        number of nodes to simulate
    avgsend
        average sending interval in seconds
    datasize
        Size of data that each device sends in bytes
    collision
        0   simplified check. Two packets collide when they arrive at the same time, on the same frequency and SF
        1   considers the capture effect
        2   consider the Non-orthognality SFs effect and capture effect
    randomseed
        random seed
 OUTPUT
    The result of every simulation run will be appended to a file named expX.dat,
    whereby X is the experiment number. The file contains a space separated table
    of values for nodes, collisions, transmissions and total energy spent. The
    data file can be easily plotted using e.g. gnuplot.
"""

# from typing_extensions import Required
import simpy
import random
import numpy as np
import math
import sys
import re
import matplotlib.pyplot as plt
import os
import operator
import BS_coord
import time
# turn on/off graphics
graphics = 0

# this is an array with measured values for sensitivity
# see paper, Table 3
#sf7 = np.array([7,-126.5,-124.25,-120.75])
#sf8 = np.array([8,-127.25,-126.75,-124.0])
#sf9 = np.array([9,-131.25,-128.25,-127.5])
#sf10 = np.array([10,-132.75,-130.25,-128.75])
#sf11 = np.array([11,-134.5,-132.75,-130])
#sf12 = np.array([12,-133.25,-132.25,-132.25])
sf7 = np.array([7,-124,-120,-117.0])
sf8 = np.array([8,-127,-123,-120.0])
sf9 = np.array([9,-130,-126,-123.0])
sf10 = np.array([10,-133,-129,-126.0])
sf11 = np.array([11,-135,-131.52,-128.51])
sf12 = np.array([12,-137,-134,-131.0])

sensi = np.array([sf7,sf8,sf9,sf10,sf11,sf12])

IS7 = np.array([1,-8,-9,-9,-9,-9])
IS8 = np.array([-11,1,-11,-12,-13,-13])
IS9 = np.array([-15,-13,1,-13,-14,-15])
IS10 = np.array([-19,-18,-17,1,-17,-18])
IS11 = np.array([-22,-22,-21,-20,1,-20])
IS12 = np.array([-25,-25,-25,-24,-23,1])
IsoThresholds = np.array([IS7,IS8,IS9,IS10,IS11,IS12])

# Bandwidth
Bandwidth = 125
# Coding Rate
CodingRate = 1
# packet size per SFs
PcktLength_SF = [20,20,20,20,20,20]
LorawanHeader = 7
# last time the gateway acked a package
nearstACK1p = np.zeros((8,3)) # 3 channels with 1% duty cycle for each BS
nearstACK10p = [0]*8 # one channel with 10% duty cycle for each BS
AckMessLen = 0

#
# packet error model assumming independent Bernoulli
#
from scipy.stats import norm
def ber_reynders(eb_no, sf):
    """Given the energy per bit to noise ratio (in db), compute the bit error for the SF"""
    return norm.sf(math.log(sf, 12)/math.sqrt(2)*eb_no)

def ber_reynders_snr(snr, sf, bw, cr):
    """Compute the bit error given the SNR (db) and SF"""
    Temp = [4.0/5,4.0/6,4.0/7,4.0/8]
    CR = Temp[cr-1]
    BW = bw*1000.0
    eb_no =  snr - 10*math.log10(BW/2**sf) - 10*math.log10(sf) - 10*math.log10(CR) + 10*math.log10(BW)
    return ber_reynders(eb_no, sf)

def per(sf,bw,cr,rssi,pl):
    snr = rssi  +174 - 10*math.log10(bw) - 6
    return 1 - (1 - ber_reynders_snr(snr, sf, bw, cr))**(pl*8)

#
# check for collisions at base station
# Note: called before a packet (or rather node) is inserted into the list
def checkcollision(packet):
    col = 0 # flag needed since there might be several collisions for packet
    processing = 0
    for i in range(0,len(packetsAtBS)):
        if packetsAtBS[i].packet.processed == 1:
            processing = processing + 1
    if (processing > maxBSReceives):
        # print "too long:", len(packetsAtBS)
        packet.processed = 0
    else:
        packet.processed = 1

    if packetsAtBS:
        ubs=[0]*numberBS
        # print "CHECK node {} (sf:{} bw:{} freq:{:.6e}) others: {}".format(
            #  packet.nodeid, packet.sf, packet.bw, packet.freq,
            #  len(packetsAtBS))
        for other in packetsAtBS:
            if other.nodeid != packet.nodeid:
            #    print ">> node {} (sf:{} bw:{} freq:{:.6e})".format(
                #    other.nodeid, other.packet.sf, other.packet.bw, other.packet.freq)
               if(full_collision == 1 or full_collision == 2):
                   if frequencyCollision(packet, other.packet) \
                   and timingCollision(packet, other.packet):
                       # check who collides in the power domain
                        if (full_collision == 1):
                          # Capture effect
                          c, ubs = powerCollision_1(packet, other.packet)
                        else:
                          # Capture + Non-orthognalitiy SFs effects
                          c = powerCollision_2(packet, other.packet)

                        # for value in ubs:
                        #     if value == 3:
                        #         packet.collided = 1
                        #         other.packet.collided = 1
                        #         col = 1
                        #     elif value == 2:
                        #         other.packet.collided = 1
                        #     elif value == 1:
                        #         packet.collided = 1
                        #         col = 1
                       # mark all the collided packets
                       # either this one, the other one, or both
                    #    for p in c:
                    #       p.collided = 1
                    #       if p == packet:
                    #          col = 1
                   else:
                       # no freq or timing collision, all fine
                       pass
               else:
                   # simple collision
                   if frequencyCollision(packet, other.packet) \
                   and sfCollision(packet, other.packet):
                       packet.collided = 1
                       other.packet.collided = 1  # other also got lost, if it wasn't lost already
                       col = 1
        return col, np.array(ubs)
    return col, np.zeros(numberBS)

# check if the gateway can ack this packet at any of the receive windows
# based on the duty cycle
def checkACK(packet):
    global  nearstACK1p
    global  nearstACK10p
    # check ack in the first window
    chanlindex=[872000000, 864000000, 860000000].index(packet.freq)
    timeofacking = env.now + 1  # one sec after receiving the packet
    if (timeofacking >= nearstACK1p[np.argmax(abs(packet.highestRSSI[-1])),chanlindex]):
        # this packet can be acked
        packet.acked = 1
        # tempairtime = airtime(packet.sf, CodingRate, AckMessLen+LorawanHeader, Bandwidth)
        tempairtime = airtime(12, CodingRate, AckMessLen+LorawanHeader, Bandwidth)
        nearstACK1p[np.argmax(abs(packet.highestRSSI[-1])),chanlindex] = timeofacking+(tempairtime/0.01)
        nodes[packet.nodeid].rxtime += tempairtime
        nodes[packet.nodeid].downlinkrcvd.append([timeofacking, 1])
        return packet.acked
    else:
        # this packet can not be acked
        packet.acked = 0
        Tsym = (2**packet.sf)/(Bandwidth*1000) # sec
        Tpream = (8 + 4.25)*Tsym
        nodes[packet.nodeid].rxtime += Tpream

    # chcek ack in the second window
    timeofacking = env.now + 2  # two secs after receiving the packet
    if (timeofacking >= nearstACK10p[np.argmax(abs(packet.highestRSSI[-1]))]):
        # this packet can be acked
        packet.acked = 1
        tempairtime = airtime(12, CodingRate, AckMessLen+LorawanHeader, Bandwidth)
        nearstACK10p[np.argmax(abs(packet.highestRSSI[-1]))] = timeofacking+(tempairtime/0.1)
        nodes[packet.nodeid].rxtime += tempairtime
        nodes[packet.nodeid].downlinkrcvd.append([timeofacking, 2])
        return packet.acked
    else:
        # this packet can not be acked
        packet.acked = 0
        Tsym = (2.0**12)/(Bandwidth*1000.0) # sec
        Tpream = (8 + 4.25)*Tsym
        nodes[packet.nodeid].rxtime += Tpream
        return packet.acked
#
# frequencyCollision, conditions
#
#        |f1-f2| <= 120 kHz if f1 or f2 has bw 500
#        |f1-f2| <= 60 kHz if f1 or f2 has bw 250
#        |f1-f2| <= 30 kHz if f1 or f2 has bw 125
def frequencyCollision(p1,p2):
    if (abs(p1.freq-p2.freq)<=120) and (p1.bw==500 or p2.bw==500) and np.array(p1.BS==p2.BS).sum()>=len(p1.BS):
        # print "frequency coll 500"
        return True
    elif (abs(p1.freq-p2.freq)<=60) and (p1.bw==250 or p2.bw==250) and np.array(p1.BS==p2.BS).sum()>=len(p1.BS):
        # print "frequency coll 250"
        return True
    else:
        if (abs(p1.freq-p2.freq)<=30) and np.array(p1.BS==p2.BS).sum()>=len(p1.BS):
            # Second statement verifies if p2 is received in all BSs that p1 is recevied
            # print "frequency coll 125"
            return True
        #else:
    # print "no frequency coll"
    return False

def sfCollision(p1, p2):
    if p1.sf == p2.sf and np.array(p1.BS==p2.BS).sum()>=len(p1.BS):
        # Second statement verifies if p2 is received in all BSs that p1 is recevied
        # print "collision sf node {} and node {}".format(p1.nodeid, p2.nodeid)
        # p2 may have been lost too, will be marked by other checks
        return True
    # print "no sf collision"
    return False

# check only the capture between the same spreading factor
def powerCollision_1(p1, p2):
    #powerThreshold = 6
    # print "pwr: node {0.nodeid} {0.rssi:3.2f} dBm node {1.nodeid} {1.rssi:3.2f} dBm; diff {2:3.2f} dBm".format(p1, p2, round(p1.rssi - p2.rssi,2))
    UnsafeBS = [0]*numberBS
    if p1.sf == p2.sf:
        # p_1 = 0
        # p_2 = 0
        # p_12 = 0
        for bs in p1.BS:
            if abs(p1.rssi[bs] - p2.rssi[bs]) < IsoThresholds[p1.sf-7][p2.sf-7]:
                    # print "collision pwr both node {} and node {}".format(p1.nodeid, p2.nodeid)
                    # packets are too close to each other, both collide
                    # return both pack ets as casualties
                    UnsafeBS[bs] = 3
                    # return (p1, p2)

            elif p1.rssi[bs] - p2.rssi[bs] < IsoThresholds[p1.sf-7][p2.sf-7]:
                    # p2 overpowered p1, return p1 as casualty
                    # print "collision pwr node {} overpowered node {}".format(p2.nodeid, p1.nodeid)
                    # p_1 += 1
                    UnsafeBS[bs] = 1
                    #collided.append([p1,bs])
                    #return (p1,)
            #    print "p1 wins, p2 lost"
            # p2 was the weaker packet, return it as a casualty
            else:
                # p_2 += 1
                UnsafeBS[bs] = 2
                #collided.append([p2,bs])
            #return (p2,)

        return (), UnsafeBS
        # if p_1 > p_2 and p_1 > p_12:
        #     return (p1,), UnsafeBS
        # elif p_2 > p_1 and p_2 > p_12:
        #     return (p2,), UnsafeBS
        # else:
        #     return (p1,p2), UnsafeBS

    else:
        return (), UnsafeBS

# check the capture effect and checking the effect of pesudo-orthognal SFs
def powerCollision_2(p1, p2):
    #powerThreshold = 6
    # print "SF: node {0.nodeid} {0.sf} node {1.nodeid} {1.sf}".format(p1, p2)
    # print "pwr: node {0.nodeid} {0.rssi:3.2f} dBm node {1.nodeid} {1.rssi:3.2f} dBm; diff {2:3.2f} dBm".format(p1, p2, round(p1.rssi - p2.rssi,2))
    for bs in p1.BS:
        if p1.sf == p2.sf:
            if abs(p1.rssi[bs] - p2.rssi[bs]) < IsoThresholds[p1.sf-7][p2.sf-7]:
                #    print "collision pwr both node {} and node {}".format(p1.nodeid, p2.nodeid)
                # packets are too close to each other, both collide
                # return both packets as casualties
                return (p1, p2)
            elif p1.rssi[bs] - p2.rssi[bs] < IsoThresholds[p1.sf-7][p2.sf-7]:
                # p2 overpowered p1, return p1 as casualty
                #    print "collision pwr node {} overpowered node {}".format(p2.nodeid, p1.nodeid)
                #    print "capture - p2 wins, p1 lost"
                return (p1,)
            #    print "capture - p1 wins, p2 lost"
            # p2 was the weaker packet, return it as a casualty
            return (p2,)
        else:
            if p1.rssi[bs]-p2.rssi[bs] > IsoThresholds[p1.sf-7][p2.sf-7]:
                #   print "P1 is OK"
                if p2.rssi[bs]-p1.rssi[bs] > IsoThresholds[p2.sf-7][p1.sf-7]:
                    #   print "p2 is OK"
                    return ()
                else:
                    #   print "p2 is lost"
                    return (p2,)
            else:
                #    print "p1 is lost"
                if p2.rssi[bs]-p1.rssi[bs] > IsoThresholds[p2.sf-7][p1.sf-7]:
                    #    print "p2 is OK"
                    return (p1,)
                else:
                    #    print "p2 is lost"
                    return (p1,p2)


def timingCollision(p1, p2):
    # assuming p1 is the freshly arrived packet and this is the last check
    # we've already determined that p1 is a weak packet, so the only
    # way we can win is by being late enough (only the first n - 5 preamble symbols overlap)

    # assuming 8 preamble symbols
    Npream = 8

    # we can lose at most (Npream - 5) * Tsym of our preamble
    Tpreamb = 2**p1.sf/(1.0*p1.bw) * (Npream - 5)

    # check whether p2 ends in p1's critical section
    p2_end = p2.addTime + p2.rectime
    p1_cs = env.now + (Tpreamb/1000.0)  # to sec
    # print "collision timing node {} ({},{},{}) node {} ({},{})".format(
        # p1.nodeid, env.now - env.now, p1_cs - env.now, p1.rectime,
        # p2.nodeid, p2.addTime - env.now, p2_end - env.now
    # )
    if p1_cs < p2_end and np.array(p1.BS==p2.BS).sum()>=len(p1.BS):
        # p1 collided with p2 and lost
        # print "not late enough"
        return True
    # print "saved by the preamble"
    return False

# this function computes the airtime of a packet
# according to LoraDesignGuide_STD.pdf
#
def airtime(sf,cr,pl,bw):
    H = 0        # implicit header disabled (H=0) or not (H=1)
    DE = 0       # low data rate optimization enabled (=1) or not (=0)
    Npream = 8   # number of preamble symbol (12.25  from Utz paper)

    if bw == 125 and sf in [11, 12]:
        # low data rate optimization mandated for BW125 with SF11 and SF12
        DE = 1
    if sf == 6:
        # can only have implicit header with SF6
        H = 1

    Tsym = (2.0**sf)/bw  # msec
    Tpream = (Npream + 4.25)*Tsym
    # print "sf", sf, " cr", cr, "pl", pl, "bw", bw
    payloadSymbNB = 8 + max(math.ceil((8.0*pl-4.0*sf+28+16-20*H)/(4.0*(sf-2*DE)))*(cr+4),0)
    Tpayload = payloadSymbNB * Tsym
    return ((Tpream + Tpayload)/1000.0)  # to secs
#
# this function creates a node
#
class myNode():
    def __init__(self, nodeid, bs, period, datasize, ADR, ADRtype, margin_db):
        self.nodeid = nodeid
        self.buffer = datasize
        self.bs = bs
        self.first = 1
        self.period = period
        self.lstretans = 0
        self.sent = 0
        self.coll = 0
        self.lost = 0
        self.noack = 0
        self.acklost = 0
        self.recv = 0
        self.losterror = 0
        self.rxtime = 0
        self.x = 0
        self.y = 0
        self.uplink = []
        self.downlinkrcvd = []
        self.edclass = "A"
        self.highestRSSI = []
        self.availableBS = []
        self.ADR = ADR
        self.RSSIhist = []
        if self.ADR:
            self.ADRtype = ADRtype
            self.DER_inst = []
            self.ADR_counter = [0,0]
            self.margin_db = margin_db
            self.SNRm_dict = {
                "ADR-TTN": self.SNR_ADR_TTN,
                "ADR+": self.SNR_ADR_plus,
                "ADRx": self.SNR_ADRx
            }
            self.required_SNR = {
            7: -7.5,
            8: -10,
            9: -12.5,
            10: -15,
            11: -17.5,
            12: -20
            }
            self.SNRhist = []
            self.ADRsf = 0
            self.ADRtx = 0
            self.ADRcommand = 0
            self.firstADR = False
            self.DER_ref = DER_ref
        self.last_count = -1
        self.frame_counting = []
        # this is very complex prodecure for placing nodes
        # and ensure minimum distance between each pair of nodes

        found = 0
        rounds = 0
        global nodes
        while (found == 0 and rounds < 100):
            a = random.random()
            b = random.random()
            if b<a:
                a,b = b,a
            posx = b*maxDist*math.cos(2*math.pi*a/b)+maxDist
            posy = b*maxDist*math.sin(2*math.pi*a/b)+maxDist
            if len(nodes) > 0:
                for index, n in enumerate(nodes):
                    dist = np.sqrt(((abs(n.x-posx))**2)+((abs(n.y-posy))**2))
                    if dist >= 10:
                        found = 1
                        self.x = posx
                        self.y = posy
                    else:
                        rounds = rounds + 1
                        if rounds == 100:
                            # print "could not place new node, giving up"
                            exit(-1)
            else:
                # print "first node"
                self.x = posx
                self.y = posy
                found = 1
        self.dist = np.sqrt((self.x-bsx)*(self.x-bsx)+(self.y-bsy)*(self.y-bsy))
        # print('node %d' %nodeid, "x", self.x, "y", self.y, "dist: ", self.dist)

        self.txpow = 0

        # graphics for node
        global graphics
        if (graphics == 1):
            global ax
            ax.add_artist(plt.Circle((self.x, self.y), 2, fill=True, color='blue'))
    
    def SNR_ADR_TTN(self, SNR):
        return np.max(SNR)

    def SNR_ADR_plus(self, SNR):
        return np.mean(SNR)

    def SNR_ADRx(self, SNR):
        SNRm = np.mean(SNR)
        DER_inst = len(self.frame_counting)/float(self.frame_counting[-1]-self.frame_counting[0] + 1)
        self.newMargindB(DER_inst)
        return SNRm

    def newMargindB(self, DER_inst):
        node.DER_inst.append(DER_inst)
        if DER_inst < self.DER_ref:
            self.margin_db = min(self.margin_db + 5, 30)
        elif DER_inst > self.DER_ref * 1.15:
            self.margin_db = max(self.margin_db - 2.5, 5)
        else:
            self.margin_db = self.margin_db

    def calc_ADR_NS(self):
        if self.packet.frameid > self.last_count:
            if env.now < datasize/2:#self.buffer <= datasize/2:
                self.recv = self.recv + 1
            self.last_count = self.packet.frameid
            self.RSSIhist.append(max(self.packet.rssi))
            self.frame_counting.append(self.packet.frameid)

        else:
            if max(self.packet.rssi) > self.RSSIhist[-1]:
                self.RSSIhist[-1]=max(self.packet.rssi)

        while len(self.RSSIhist) > 20:
            self.RSSIhist.pop(0)

        while len(self.frame_counting) > 20:
            self.frame_counting.pop(0)

        if len(self.RSSIhist) == 20 and self.ADR:
            self.firstADR = True
            self.ADRsf = self.packet.sf
            self.ADRtx = self.packet.txpow
            SNR = np.array(self.RSSIhist) + 174 - 10 * math.log10(self.packet.bw*1e3)
            SNRm = self.SNRm_dict[self.ADRtype](SNR)
            # print(SNRm)
            # print(np.max(SNR))
            # print("")
            self.SNRhist.append(SNRm)
            Nstep = int((SNRm - self.margin_db - self.required_SNR[self.ADRsf])/3)
            # print("in:\nnode {} SF: {}, PT: {}, NSTEP: {}".format(self.nodeid,self.ADRsf,self.ADRtx,Nstep))
            if Nstep < 0:
                while(self.ADRtx < 14 and Nstep < 0):
                    self.ADRtx += 2
                    Nstep += 1
            elif Nstep > 0:
                while(self.ADRsf > 7 and Nstep > 0):
                    self.ADRsf -= 1
                    Nstep -= 1
                while(self.ADRtx > 2 and Nstep > 0):
                    self.ADRtx -= 2
                    Nstep -= 1
            self.ADRcommand = self.packet.frameid
            self.RSSIhist = [max(self.packet.rssi)]

        if self.packet.acked==1 and self.packet.acklost==0 and self.ADR: # ACK is sent and packet isnt lost
            if self.ADRsf != 0:
                self.packet.sf = self.ADRsf
                self.packet.txpow = self.ADRtx
                # delay = self.packet.frameid - self.ADRcommand
            # print("SF: {}, PT: {}, delay: {}\nout".format(self.packet.sf ,self.packet.txpow,delay))

    def calc_ADR_ED(self):
        ADRsf = self.packet.sf
        ADRtx = self.packet.txpow
        
        if self.ADR:
            if self.packet.acked==1 and self.packet.acklost==0: # ACK is sent and packet isnt lost
                self.ADR_counter = [0, 0]
            else:                                               # wether ACK was not sent or packet was lost is irrelevant
                self.ADR_counter[1] += 1

            if self.ADR_counter == [0,64] or self.ADR_counter == [1,32]:
                self.ADR_counter = [1, 0]
                if self.ADR:
                    if ADRtx < 14:
                        ADRtx += 2
                    elif ADRsf < 12:
                        ADRsf += 1
                    print("ED{}-side: SF: {}->{}, TX: {}->{}".format(self.nodeid,self.packet.sf,ADRsf,self.packet.txpow,ADRtx))

        self.packet.sf = ADRsf
        self.packet.txpow = ADRtx

class assignParameters():
    def __init__(self, nodeid, distance):
        global Ptx
        global gamma
        global d0
        global var
        global Lpld0
        global GL

        self.nodeid = nodeid
        self.bw = Bandwidth
        self.cr = CodingRate
        self.sf = 12#np.random.randint(7,13)
        self.txpow = 14#np.random.randint(1,8)*2
        self.rectime = airtime(self.sf, self.cr, LorawanHeader+PcktLength_SF[self.sf-7], self.bw)
        self.freq = 864000000#random.choice([872000000, 864000000, 860000000])

        global SFdistribution, CRdistribution, TXdistribution, BWdistribution
        SFdistribution[self.sf-7]+=1
        CRdistribution[self.cr-1]+=1
        TXdistribution[int(self.txpow)-2]+=1
        if self.bw==125:
            BWdistribution[0]+=1
        elif self.bw==250:
            BWdistribution[1]+=1
        else:
            BWdistribution[2]+=1

#
# this function creates a packet (associated with a node)
# it also sets all parameters, currently random
#
class myPacket():
    def __init__(self, nodeid, freq, sf, bw, cr, txpow, distance):
        global gamma
        global d0
        global var
        global Lpld0
        global GL

        self.nodeid = nodeid
        self.freq = freq
        self.sf = sf
        self.bw = bw
        self.cr = cr
        self.txpow = txpow
        # transmission range, needs update XXX
        self.transRange = 150
        self.pl = LorawanHeader+PcktLength_SF[self.sf-7]
        self.symTime = (2.0**self.sf)/self.bw
        self.arriveTime = 0

        self.rssi = np.array([])
        self.BS = np.arange(numberBS)
        # print "node id", self.nodeid, "symTime ", self.symTime, "rssi", self.rssi
        self.rectime = airtime(self.sf,self.cr,self.pl,self.bw)
        # print "rectime node ", self.nodeid, "  ", self.rectime
        # denote if packet is collided
        self.collided = 0
        self.processed = 0
        self.lost = False
        self.perror = False
        self.acked = 0
        self.acklost = 0
        self.highestRSSI = []
        self.availableBS = []
        self.frameid = -1

#
# main discrete event loop, runs for each node
# a global list of packet being processed at the gateway
# is maintained
#
def dBm2lin(value):
    return 1e-3*10**(value/10)

def lin2dBm(value):
    return 10*np.log10(value/1e-3)
global maxretr
maxretr = 0
def transmit(env,node):
    while env.now < datasize: #node.buffer > 0.0:
        # print(env.now)
        h = np.random.rayleigh(np.sqrt(2/np.pi))
        node.packet.rssi = lin2dBm(h**2 * dBm2lin(node.packet.txpow + GL - Lpld0 - 10*gamma*np.log10(node.dist/d0)))#node.packet.txpow - Lpld0 - 10*gamma*math.log10(node.dist/d0)# - np.random.normal(-var, var)
        # add maximum number of retransmissions
        if (node.lstretans and node.lstretans <= maxretr):
            node.first = 0
            node.buffer += PcktLength_SF[node.parameters.sf-7]
            # the randomization part (2 secs) to resove the collisions among retrasmissions
            yield env.timeout(max(2+airtime(12, CodingRate, AckMessLen+LorawanHeader, Bandwidth), float(node.packet.rectime*((1-0.01)/0.01)))+(random.expovariate(1.0/float(2000))/1000.0))
        else:
            node.first = 0
            node.lstretans = 0
            node.packet.frameid += 1
            nexttx = random.expovariate(1.0/float(node.period))
            while(nexttx < airtime(12, CodingRate, AckMessLen+LorawanHeader, Bandwidth)/0.01): # fixed DC
                nexttx = random.expovariate(1.0/float(node.period))
            if env.now < datasize/2: #node.buffer <= datasize/2:
                node.sent = node.sent + 1
            yield env.timeout(nexttx)

        node.buffer -= PcktLength_SF[node.parameters.sf-7]
        # print "node {0.nodeid} buffer {0.buffer} bytes".format(node)

        # time sending and receiving
        # packet arrives -> add to base station
        node.uplink.append([env.now, node.parameters.sf, node.parameters.txpow]) # Registers the time in which a node starts an uplink
        if (node in packetsAtBS):
            print ("ERROR: packet already in")
        else:
            sensitivity = sensi[node.packet.sf - 7, [125,250,500].index(node.packet.bw) + 1]
            node.packet.BS = np.where(node.packet.rssi>=sensitivity)[0]
            if (node.packet.rssi < sensitivity).all():
                # print "node {}: packet will be lost".format(node.nodeid)
                node.packet.lost = True
            else:
                node.packet.lost = False
                node.packet.perror = False
                # if (per(node.packet.sf,node.packet.bw,node.packet.cr,max(node.packet.rssi),node.packet.pl) < random.uniform(0,1)):
                #     # OK CRC
                #     node.packet.perror = False
                # else:
                #     # Bad CRC
                #     node.packet.perror = True
                # adding packet if no collision
                is_col, unsafeBS = checkcollision(node.packet)
                # if (is_col==1):
                    # node.packet.collided = 1
                # else:
                    # node.packet.collided = 0
                RSSI_id = np.array(node.packet.rssi>=sensitivity) * (np.array(unsafeBS == 0)+np.array(unsafeBS == 2))
                if RSSI_id.sum() > 0:#len(RSSI_id):
                    node.packet.collided = 0
                else:
                    node.packet.collided = 1
                node.packet.rssi = node.packet.rssi*RSSI_id - np.logical_not(RSSI_id)*1000
                node.packet.highestRSSI.append(np.array(node.packet.rssi))
                node.packet.availableBS.append((RSSI_id).astype(int))
                # if(not node.packet.perror and node.packet.collided == 0):
                    
                packetsAtBS.append(node)
                node.packet.addTime = env.now

        yield env.timeout(node.packet.rectime)

        if (node.packet.lost == False\
                and node.packet.perror == False\
                and node.packet.collided == False\
                ):

            if checkACK(node.packet):
                node.packet.acked = 1
                # the packet can be acked
                # check if the ack is lost or not
                chosen_BS = np.argmax(node.packet.highestRSSI[-1])
                h = np.random.rayleigh(np.sqrt(2/np.pi))
                downlinkPr = lin2dBm( h**2 * dBm2lin(14 +GL- Lpld0 - 10*gamma*math.log10(node.dist[chosen_BS]/d0)) )
                #if((14 - Lpld0 - 10*gamma*math.log10(node.dist/d0) - np.random.normal(-var, var)) > sensi[12-7, 1]): # SF12 and Tx14
                # the ack is not lost
                if downlinkPr >= sensi[12-7,1]:
                    node.packet.acklost = 0
                    # print("node {} acked! RSSI: {:.2f}".format(node.nodeid,downlinkPr))

                else:
                # ack is lost
                    # print("node {} ack lost RSSI: {:.2f}".format(node.nodeid,downlinkPr))
                    node.downlinkrcvd.pop()
                    node.packet.acklost = 1
            else:
                node.packet.acked = 0

            # if node.ADR:
            # if node.packet.frameid > node.last_count:
            #     node.recv = node.recv + 1

            node.calc_ADR_NS()

        node.calc_ADR_ED()
            



        if node.packet.processed == 1:
            global nrProcessed
            nrProcessed = nrProcessed + 1
        if node.packet.lost:
            #node.buffer += PcktLength_SF[node.parameters.sf-7]
            # print "node {0.nodeid} buffer {0.buffer} bytes".format(node)
            node.lost = node.lost + 1
            global nrLost
            node.lstretans += 1
            nrLost += 1
        elif node.packet.perror:
            # print "node {0.nodeid} buffer {0.buffer} bytes".format(node)
            node.losterror = node.losterror + 1
            global nrLostError
            nrLostError += 1
        elif node.packet.collided == 1:
            #node.buffer += PcktLength_SF[node.parameters.sf-7]
            # print "node {0.nodeid} buffer {0.buffer} bytes".format(node)
            if node.lstretans >= maxretr and env.now <= datasize/2:#node.buffer <= datasize/2:
                node.coll = node.coll + 1
                global nrCollisions
                nrCollisions = nrCollisions +1
            node.lstretans += 1
        elif node.packet.acked == 0:
            #node.buffer += PcktLength_SF[node.parameters.sf-7]
            # print "node {0.nodeid} buffer {0.buffer} bytes".format(node)
            node.noack = node.noack + 1
            node.lstretans += 1
            global nrNoACK
            nrNoACK += 1
        elif node.packet.acklost == 1:
            #node.buffer += PcktLength_SF[node.parameters.sf-7]
            # print "node {0.nodeid} buffer {0.buffer} bytes".format(node)
            node.acklost = node.acklost + 1
            node.lstretans += 1
            global nrACKLost
            nrACKLost += 1
        else:
            # node.recv = node.recv + 1
            node.lstretans = 0
            global nrReceived
            nrReceived = nrReceived + 1

        # complete packet has been received by base station
        # can remove it
        if (node in packetsAtBS):
            packetsAtBS.remove(node)
            # reset the packet
        node.packet.collided = 0
        node.packet.processed = 0
        node.packet.lost = False
        node.packet.acked = 0
        node.packet.acklost = 0

#
# "main" program
#

# get arguments
ADRtype = "none"
margin_db = 0
if len(sys.argv) >= 6:
    nrNodes = int(sys.argv[1])
    avgSendTime = int(sys.argv[2])
    datasize = float(sys.argv[3])*60*60*24#int(sys.argv[3])
    maxDist = float(sys.argv[4])
    numberBS = int(sys.argv[5])
    ADR = bool(int(sys.argv[6]))
    if ADR and len(sys.argv) >= 10:
        ADRtype = sys.argv[7]
        margin_db = float(sys.argv[8])
        DER_ref = float(sys.argv[9])
        full_collision = int(sys.argv[10])
        Rnd = random.seed(int(sys.argv[11]))
        rs = int(sys.argv[11])
    else:
        full_collision = int(sys.argv[7])
        Rnd = random.seed(int(sys.argv[8]))
        rs = int(sys.argv[8])
    print ("Nodes:", nrNodes)
    print ("DataSize [bytes]", datasize)
    print ("AvgSendTime (exp. distributed):",avgSendTime)
    print ("Number of BS: ", numberBS)
    print ("Full Collision: ", full_collision)
    print ("Random Seed: ", rs)
else:
    print ("usage: ./confirmablelorawan <nodes> <avgsend> <datasize> <collision> <randomseed>")
    exit(-1)

# global stuff
nodes = []
nodeder1 = [0 for i in range(0,nrNodes)]
nodeder2 = [0 for i in range(0,nrNodes)]
tempdists = [0 for i in range(0,nrNodes)]
packetsAtBS = []
SFdistribution = [0 for x in range(0,6)]
BWdistribution = [0 for x in range(0,3)]
CRdistribution = [0 for x in range(0,4)]
TXdistribution = [0 for x in range(0,13)]
env = simpy.Environment()

# maximum number of packets the BS can receive at the same time
maxBSReceives = 8

# max distance: 300m in city, 3000 m outside (5 km Utz experiment)
# also more unit-disc like according to Utz
bsId = 1
nrCollisions = 0
nrReceived = 0
nrProcessed = 0
nrLost = 0
nrLostError = 0
nrNoACK = 0
nrACKLost = 0

Ptx = 9.75
gamma = 2.32 #2.08
d0 = 1000 #40.0
var = 7.8 #2.0
Lpld0 = 128.95 #127.41
GL = 0
minsensi = np.amin(sensi[:,[125,250,500].index(Bandwidth) + 1])
Lpl = Ptx - minsensi
# maxDist = d0*(10**((Lpl-Lpld0)/(10.0*gamma)))
print ("maxDist:", maxDist)

# base station placement
BS_coordinates = BS_coord.base_coordinates(numberBS,maxDist)
bsx = BS_coordinates[:,0]#maxDist+10
bsy = BS_coordinates[:,1]#maxDist+10
# xmax = bsx + maxDist + 10
# ymax = bsy + maxDist + 10

# prepare graphics and add sink
if (graphics == 1):
    plt.ion()
    plt.figure()
    ax = plt.gcf().gca()
    # XXX should be base station position
    ax.add_artist(plt.Circle((bsx, bsy), 3, fill=True, color='green'))
    ax.add_artist(plt.Circle((bsx, bsy), maxDist, fill=False, color='green'))

start = time.time()
for i in range(0,nrNodes):
    # myNode takes period (in ms), base station id packetlen (in Bytes)
    node = myNode(i,bsId, avgSendTime, datasize, ADR, ADRtype, margin_db)
    nodes.append(node)
    node.parameters = assignParameters(node.nodeid, node.dist)
    node.packet = myPacket(node.nodeid, node.parameters.freq, node.parameters.sf, node.parameters.bw, node.parameters.cr, node.parameters.txpow, node.dist)
    env.process(transmit(env,node))


#prepare show
if (graphics == 1):
    plt.xlim([0, xmax])
    plt.ylim([0, ymax])
    plt.draw()
    plt.show()

# start simulation
env.run(until=datasize)
end = time.time()

print("time",(end-start))
#env.run(until=simtime)

# print stats and save into file
#print "nrCollisions ", nrCollisions

# compute energy
# Transmit consumption in mA from -2 to +17 dBm
TX = [22, 22, 22, 23,                                      # RFO/PA0: -2..1
      24, 24, 24, 25, 25, 25, 25, 26, 31, 32, 34, 35, 44,  # PA_BOOST/PA1: 2..14
      82, 85, 90,                                          # PA_BOOST/PA1: 15..17
      105, 115, 125]                                       # PA_BOOST/PA1+PA2: 18..20
RX = 16
V = 3.0     # voltage XXX
sent = sum(n.sent for n in nodes)
recv = sum(n.recv for n in nodes)
energy = sum(((node.packet.rectime * node.sent * TX[int(node.packet.txpow)+2])+(node.rxtime * RX)) * V  for node in nodes)  / 1e3

print ("energy (in J): ", energy)
print ("sent packets: ", sent)
print ("collisions: ", nrCollisions)
print ("received packets: ", recv)
print ("processed packets: ", nrProcessed)
print ("lost packets: ", nrLost)
print ("Bad CRC: ", nrLostError)
print ("NoACK packets: ", nrNoACK)
# data extraction rate
der1 = (sent-nrCollisions)/float(sent) if sent!=0 else 0
print ("DER:", der1)
der2 = (recv)/float(sent) if sent!=0 else 0
print ("DER method 2:", der2)

# data extraction rate per node
for i in range(0,nrNodes):
    tempdists[i] = nodes[i].dist
    nodeder1[i] = ((nodes[i].sent-nodes[i].coll)/(float(nodes[i].sent)) if float(nodes[i].sent)!=0 else 0)
    nodeder2[i] = (nodes[i].recv/(float(nodes[i].sent)) if float(nodes[i].sent)!=0 else 0)
# calculate the fairness indexes per node
nodefair1 = (sum(nodeder1)**2/(nrNodes*sum([i*float(j) for i,j in zip(nodeder1,nodeder1)])) if (sum([i*float(j) for i,j in zip(nodeder1,nodeder1)]))!=0 else 0)
nodefair2 = (sum(nodeder2)**2/(nrNodes*sum([i*float(j) for i,j in zip(nodeder2,nodeder2)])) if (sum([i*float(j) for i,j in zip(nodeder2,nodeder2)]))!=0 else 0)

print ("============================")
print ("SFdistribution: ", SFdistribution)
print ("BWdistribution: ", BWdistribution)
print ("CRdistribution: ", CRdistribution)
print ("TXdistribution: ", TXdistribution)
print ("CollectionTime: ", env.now)

# save experiment data into a dat file that can be read by e.g. gnuplot
# name of file would be:  exp0.dat for experiment 0
fname = str("confirmablelorawan") + ".dat"
print (fname)
if os.path.isfile(fname):
     res= "\n" + str(sys.argv[5]) + ", " + str(full_collision) + ", " + str(nrNodes) + ", " + str(avgSendTime) + ", " + str(datasize) + ", " + str(sent) + ", "  + str(nrCollisions) + ", "  + str(nrLost) + ", "  + str(nrLostError) + ", " +str(nrNoACK) + ", " +str(nrACKLost) + ", " + str(env.now)+ ", " + str(der1) + ", " + str(der2)  + ", " + str(energy) + ", "  + str(nodefair1) + ", "  + str(nodefair2) + ", "  + str(SFdistribution)
else:
     res = "#randomseed, collType, nrNodes, TransRate, DataSize, nrTransmissions, nrCollisions, nrlost, nrlosterror, nrnoack, nracklost, CollectionTime, DER1, DER2, OverallEnergy, nodefair1, nodefair2, sfdistribution\n" + str(sys.argv[5]) + ", " + str(full_collision) + ", " + str(nrNodes) + ", " + str(avgSendTime) + ", " + str(datasize) + ", " + str(sent) + ", "  + str(nrCollisions) + ", "  + str(nrLost) + ", "  + str(nrLostError) + ", " +str(nrNoACK) + ", " +str(nrACKLost) + ", " + str(env.now)+ ", " + str(der1) + ", " + str(der2)  + ", " + str(energy) + ", "  + str(nodefair1) + ", "  + str(nodefair2) + ", "  + str(SFdistribution)
newres=re.sub('[^#a-zA-Z0-9 \n\.]','',res)
print (newres)
with open(fname, "a") as myfile:
    myfile.write(newres)
myfile.close()

# import csv
# for node in nodes:
#   with open('UL'+str(node.nodeid)+'.csv', 'w') as csv_file:
#     csv_reader = csv.writer(csv_file)
#     for transmission in node.uplink:
#       csv_reader.writerow(transmission)
#   with open('DL'+str(node.nodeid)+'.csv', 'w') as csv_file:
#     csv_reader = csv.writer(csv_file)
#     for reception in node.downlinkrcvd:
#       csv_reader.writerow(reception)
x = np.zeros(len(nodes))
y = np.zeros(len(nodes))
der = nodeder2#np.zeros(len(nodes))
number_BS = np.zeros(len(nodes))
avdist = np.zeros(len(nodes))
sf = [0,0,0,0,0,0]
tx = [0,0,0,0,0,0,0]
sfd = {
    7:  0,
    8:  1,
    9:  2,
    10: 3,
    11: 4,
    12: 5
}

txd = {
    2:  0,
    4:  1,
    6:  2,
    8:  3,
    10: 4,
    12: 5,
    14: 6
}

smargins = {
    5:      0,
    7.5:    1,
    10:     2,
    12.5:   3,
    15:     4,
    17.5:   5,
    20:     6,
    22.5:   7,
    25:     8,
    27.5:   9,
    30:    10
}
margins = [0]*11
margin_db = np.ones(len(nodes))
for i in range(len(nodes)):
    sf[sfd[nodes[i].packet.sf]]+=1
    tx[txd[nodes[i].packet.txpow]]+=1
    x[i] = nodes[i].x
    y[i] = nodes[i].y
    if ADR:
        margins[smargins[nodes[i].margin_db]]+=1
        margin_db[i] = nodes[i].margin_db
    # der[i] = nodes[i].noack/(nodes[i].sent)#nodeder2[i]
    number_BS[i] = np.average(nodes[i].availableBS)
    avdist[i] = np.average(nodes[i].dist)
    # number_BS[i] = nodes[i].highestRSSI[]
plt.scatter(x,y,c=der)
plt.title("DER: {:.2f}% - {}".format(np.average(der)*100, ADRtype))
plt.scatter(bsx, bsy, s=100, marker='*',c='red')
plt.colorbar()
fname="file{}-{}-DER.png".format(numberBS,ADRtype)
plt.savefig(fname,dpi=200)
plt.show()

if ADR:
    plt.scatter(avdist,margin_db,c=der)
    plt.colorbar()
    fname="file{}-{}-margins.png".format(numberBS,ADRtype)
    plt.savefig(fname,dpi=200)
    plt.show()
if not ADR:
    DER_ref = 0.8
plt.scatter(avdist,der,c=margin_db)
plt.colorbar()
plt.plot([min(avdist),max(avdist)],
         [DER_ref, DER_ref],c='red',linestyle='dashed')
fname="file{}-{}-DER_nodes.png".format(numberBS,ADRtype)
plt.savefig(fname,dpi=200)
plt.show()

print(sf)
print(tx)
print(margins)
print((np.array(der)>=DER_ref).sum()/len(der))
# plt.scatter(avdist,number_BS,alpha=0.5,c=der)
# plt.colorbar()
# plt.show()
print ("Finish")

# print(np.array(nodes[0].highestRSSI))
# print(nodes[0].availableBS)
# print(der[0])