/*
 *  new-code.cc
 *
 *
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
//#include "ns3/visualizer-module.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/random-variable-stream.h"
//#include "GaloisField.h"
#include "Utils.h"
#include "bloom_filter.hpp"
#include "glpk.h"
//#include "simplex.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <exception>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <list>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <assert.h>
#include "MyNCApp.h"

//////////////////////////////////////////////


////////////////////////////////////////////////



////////////////////////////////////////////////

using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NetworkCoding");

static const std::size_t BUFFER_SIZE = 10;
static const std::size_t WAITING_LIST_SIZE = 50;
static const std::size_t MAX_VARLIST_SIZE = 50;
static const std::size_t MaxNumberOfCoeff=10;
//static const std::size_t DECODING_BUFF_SIZE = MAX_VARLIST_SIZE;
static const std::size_t DECODED_BUFF_SIZE = 600;
static const std::size_t MAX_DELIVERED_LIST_SIZE =600;
static const std::size_t VAR_LIFETIME=900;
static const float NEIGHBOR_TIMER = 1.2;
static const double BEACON_PERIOD = 1;
static const double FORWARD_PERIOD = 0.08;
static const double SIMULATION_TIME = 3600;
static const double NUMBER_OF_NODES = 40;
static const double RADIO_RANGE = 50;
//static const double MEAN = 1;
static const int bigThreshold = 300;
//static const std::size_t smallThreshold = 20;

static const double DFPP = 0.005;
static const int variableWeight=1;
static const int unreceivedWeight=10;
static const int neighborWeight=10;
// Aging constants parameters declaration:
//static const float K0 = 25.0;
//static const float K1 = 10.0;
//static const float K2 = 0.25;
//static const float MAX_TTL = 255;
/////////////////////////////////////////////////////////


TypeId PktTypeTag::GetTypeId (void) {
  static TypeId tid = TypeId ("ns3::PktTypeTag")
    .SetParent<Tag> ()
    .AddConstructor<PktTypeTag> ()
    .AddAttribute ("Packet Type",
                   "A Packet Type",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&PktTypeTag::GetType),
                   MakeUintegerChecker<uint8_t> ())
    ;
  return tid;
}

TypeId PktTypeTag::GetInstanceTypeId (void) const {
  return GetTypeId ();
}

uint32_t PktTypeTag::GetSerializedSize (void) const {
  return 1;
}

void PktTypeTag::Serialize (TagBuffer i) const {
  i.WriteU8 (m_PacketType);
}

void  PktTypeTag::Deserialize (TagBuffer i) {
  m_PacketType = i.ReadU8 ();
}

void PktTypeTag::Print (std::ostream &os) const {
  os << "Type=" << (uint32_t)m_PacketType;
}

void PktTypeTag::SetType (uint8_t type) {
   m_PacketType = type;
}

uint8_t PktTypeTag::GetType (void) const {
  return m_PacketType;
}

Neighbor::Neighbor () {
  firstReceptionTime = 0;
  lastReceptionTime = 0;
  neighborReceivedPackets = 0;
  neighborReceivedStatusFeedbacks = 0;
  neighborReceivedBeacons = 0;
  neighborForwardedPackets = 0;
  neighborId = 0;
  neighborhoodSize = 0;
  neighborDecodingBufSize = 0;
  neighborRemainingCapacity = 0;
  neighborDecodingFilter = CreateObject<MyBloom_filter> (20, 0.01, 0);
  smallNeighborDecodedFilter = CreateObject<MyBloom_filter> (20, 0.01, 0);
  bigNeighborDecodedFilter = CreateObject<MyBloom_filter> (20, 0.01, 0);
  neighborReceivedFilter= CreateObject<MyBloom_filter> (20, 0.01, 0);
  punished=false;
  numConsDrop=0;
  numConsRedundant=0;
}

Neighbor::~Neighbor () {}


StatusFeedbackHeader::StatusFeedbackHeader() {
  m_decodedInsertedElementCount=0;
  m_decodingInsertedElementCount=0;
  m_remainingCapacity=0;
  m_decodingBufSize=0;
  m_decodedTableSize=0;
  m_decodingTableSize=0;
  m_decodedBitTable=NULL;
  m_decodingBitTable=NULL;
}


StatusFeedbackHeader::~StatusFeedbackHeader()
{
}


uint32_t StatusFeedbackHeader::GetSerializedSize (void) const {
  if (m_panicked) {
    return 21 + (m_decodedTableSize + m_decodingTableSize) / BITS_PER_CHAR+10*m_linCombSize;
  }
  return 20 + (m_decodedTableSize + m_decodingTableSize) / BITS_PER_CHAR+10*m_linCombSize;
}

void StatusFeedbackHeader::Serialize (Buffer::Iterator start) const {
  Buffer::Iterator i = start;
  i.WriteU8 (m_packetType);
  i.WriteU8 (m_nodeId);
  i.WriteU8 (m_destId);
  i.WriteU16(m_pktIndex);
  i.WriteU8 (m_neighborhoodSize);
  i.WriteU8 (m_remainingCapacity);
  i.WriteU8 (m_decodingBufSize);
  i.WriteU16 (m_decodedTableSize);
  for (std::size_t j=0; j < (m_decodedTableSize / BITS_PER_CHAR); j++) {
      i.WriteU8 (m_decodedBitTable[j]);
  }
  i.WriteU16 (m_decodedInsertedElementCount);
  i.WriteU8 (m_decodedSaltCount);
  i.WriteU16 (m_decodingTableSize);
  for (std::size_t j=0; j < (m_decodingTableSize / BITS_PER_CHAR); j++) {
      i.WriteU8 (m_decodingBitTable[j]);
  }
  i.WriteU16 (m_decodingInsertedElementCount);
  i.WriteU8 (m_decodingSaltCount);
  i.WriteU8 (m_linCombSize);
  for (std::size_t j=0; j < m_linCombSize; j++) {
    i.WriteU8(m_linComb[j].coeff);
    i.WriteU8(m_linComb[j].nodeId);
    i.WriteU16(m_linComb[j].index);
    i.WriteU8(m_linComb[j].dstId);
    i.WriteU32(m_linComb[j].genTime);
  }
  if (m_panicked) {
    i.WriteU8(1);
    i.WriteU8(m_panickingNode);
  } else
    i.WriteU8(0);
}

uint32_t StatusFeedbackHeader::Deserialize (Buffer::Iterator start) {
  Buffer::Iterator i = start;
  m_packetType = i.ReadU8 ();
  m_nodeId = i.ReadU8 ();
  m_destId =i.ReadU8 ();
  m_pktIndex=i.ReadU16();
  m_neighborhoodSize = i.ReadU8 ();
  m_remainingCapacity = i.ReadU8 ();
  m_decodingBufSize = i.ReadU8 ();
  m_decodedTableSize = i.ReadU16 ();
  if (m_decodedBitTable!=NULL) {
    delete[] m_decodedBitTable;
  }
  m_decodedBitTable = new unsigned char[m_decodedTableSize /BITS_PER_CHAR];
  for (std::size_t j=0; j < (m_decodedTableSize / BITS_PER_CHAR); j++) {
      m_decodedBitTable[j]=i.ReadU8();
  }
  m_decodedInsertedElementCount=i.ReadU16 ();
  m_decodedSaltCount=i.ReadU8();
  m_decodingTableSize=i.ReadU16 ();
  if (m_decodingBitTable!=NULL) {
    delete[] m_decodingBitTable;
  }
  m_decodingBitTable = new unsigned char[m_decodingTableSize /BITS_PER_CHAR];
  for (std::size_t j=0; j < (m_decodingTableSize / BITS_PER_CHAR); j++) {
      m_decodingBitTable[j]=i.ReadU8();
  }
  m_decodingInsertedElementCount = i.ReadU16 ();
  m_decodingSaltCount=i.ReadU8();
  m_linCombSize=i.ReadU8 ();
  LinearCombination lc;
  for (std::size_t j=0; j < m_linCombSize; j++) {
    lc.coeff=i.ReadU8();
    lc.nodeId=i.ReadU8();
    lc.index=i.ReadU16();
    lc.dstId= i.ReadU8();
    lc.genTime=i.ReadU32();
    m_linComb.push_back(lc);
  }
  m_panicked=(i.ReadU8()==1);
  if (m_panicked) {
    m_panickingNode=i.ReadU8();
  }
  return GetSerializedSize ();
}

TypeId StatusFeedbackHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::StatusFeedbackHeader")
  .SetParent<Header> ()
  .AddConstructor<StatusFeedbackHeader> ()
  ;
  return tid;
}

void StatusFeedbackHeader::SetPacketType (uint8_t type)
{
  m_packetType = type;
}

void StatusFeedbackHeader::SetNodeId (uint8_t id) {
  m_nodeId = id;
}

void StatusFeedbackHeader::SetPktIndex(uint16_t pktIndex){
  m_pktIndex=pktIndex;
}

uint16_t StatusFeedbackHeader::GetPktIndex(){
  return m_pktIndex;
}

uint8_t StatusFeedbackHeader::GetPacketType (void) const {
  return m_packetType;
}

uint8_t StatusFeedbackHeader::GetNodeId (void) const {
  return m_nodeId;
}

uint8_t StatusFeedbackHeader::GetDestination (void) const {
  return m_destId;
}

void
StatusFeedbackHeader::SetDestination (uint8_t destination) {
  m_destId = destination;
}

TypeId StatusFeedbackHeader::GetInstanceTypeId (void) const {
  return GetTypeId ();
}

void StatusFeedbackHeader::Print (std::ostream &os) const {
  os<<"Source="<< m_nodeId <<" Destination=" << m_destId
  <<" nodeId="<< (int)m_nodeId
  <<" m_linCombSize = "<<m_linCombSize;
	for (uint8_t k=0; k<m_linCombSize; k++){
    std::cout <<" m_linComb["<<k<<"].pktId = "<<m_linComb[k].nodeId<<":"<<m_linComb[k].index <<" m_linComb["<<k<<"].coeff = "<<(int)m_linComb[k].coeff;
	}
}

void StatusFeedbackHeader::SetNeighborhoodSize (uint8_t size) {
    m_neighborhoodSize = size;
}

void StatusFeedbackHeader::SetNeighborDecodingBufSize (uint8_t size) {
    m_decodingBufSize = size;
}

void StatusFeedbackHeader::SetRemainingCapacity (uint8_t remainingCapacity) {
    m_remainingCapacity = remainingCapacity;
}

void StatusFeedbackHeader::SetLinearCombinationSize () {
	m_linCombSize = m_linComb.size();
}


uint8_t StatusFeedbackHeader::GetNeighborhoodSize (void) const {
  return m_neighborhoodSize;
}

uint8_t StatusFeedbackHeader::GetNeighborDecodingBufSize (void) const {
  return m_decodingBufSize;
}

uint8_t StatusFeedbackHeader::GetRemainingCapacity (void) const {
    return m_remainingCapacity;
}

uint8_t StatusFeedbackHeader::GetLinearCombinationSize (void) const {
	return m_linCombSize;
}

void StatusFeedbackHeader::PutDecodedBloomFilter (Ptr<MyBloom_filter> nodeFilterPointer) {
  m_decodedTableSize = nodeFilterPointer->size();
  m_decodedSaltCount= nodeFilterPointer->salt_count();
  m_decodedRandomSeed= nodeFilterPointer->seed();
  m_decodedBitTable = new unsigned char[m_decodedTableSize / BITS_PER_CHAR];
  std::copy (nodeFilterPointer->table(), nodeFilterPointer->table() + (m_decodedTableSize / BITS_PER_CHAR), m_decodedBitTable);
  m_decodedInsertedElementCount = nodeFilterPointer->element_count();
}

void StatusFeedbackHeader::PutDecodingBloomFilter (Ptr<MyBloom_filter> nodeFilterPointer) {
  m_decodingTableSize = nodeFilterPointer->size();
  m_decodingSaltCount= nodeFilterPointer->salt_count();
  m_decodingRandomSeed= nodeFilterPointer->seed();
  m_decodingBitTable = new unsigned char[m_decodingTableSize / BITS_PER_CHAR];
  std::copy (nodeFilterPointer->table(), nodeFilterPointer->table() + (m_decodingTableSize / BITS_PER_CHAR), m_decodingBitTable);
  m_decodingInsertedElementCount = nodeFilterPointer->element_count();
}

Ptr<MyBloom_filter> StatusFeedbackHeader::GetDecodedBloomFilter (const std::size_t predictedElementCount, const double falsePositiveProbability) const {
  Ptr<MyBloom_filter> filterPointer;
  filterPointer = CreateObject<MyBloom_filter> (predictedElementCount, falsePositiveProbability, m_pktIndex);
  filterPointer->bit_table_ = new unsigned char[m_decodedTableSize / BITS_PER_CHAR];
  std::copy(m_decodedBitTable, m_decodedBitTable + (m_decodedTableSize / BITS_PER_CHAR), filterPointer->bit_table_);
  filterPointer->inserted_element_count_ = m_decodedInsertedElementCount;
  return filterPointer;
}

Ptr<MyBloom_filter> StatusFeedbackHeader::GetDecodedBloomFilter () const {
  Ptr<MyBloom_filter> filterPointer;

  filterPointer = CreateObject<MyBloom_filter> (m_decodedTableSize, m_decodedSaltCount, m_pktIndex, m_decodedBitTable);
  filterPointer->inserted_element_count_ = m_decodedInsertedElementCount;
  return filterPointer;
}

Ptr<MyBloom_filter>  StatusFeedbackHeader::GetDecodingBloomFilter (const std::size_t predictedElementCount, const double falsePositiveProbability) const {
  Ptr<MyBloom_filter> filterPointer;
  filterPointer = CreateObject<MyBloom_filter> (predictedElementCount, falsePositiveProbability, GetNodeId());
  filterPointer->bit_table_ = new unsigned char[m_decodingTableSize / BITS_PER_CHAR];
  std::copy(m_decodingBitTable, m_decodingBitTable + (m_decodingTableSize / BITS_PER_CHAR), filterPointer->bit_table_);
  filterPointer->inserted_element_count_ = m_decodingInsertedElementCount;
  return filterPointer;
}

Ptr<MyBloom_filter> StatusFeedbackHeader::GetDecodingBloomFilter () const {
  Ptr<MyBloom_filter> filterPointer;
  filterPointer = CreateObject<MyBloom_filter> (m_decodingTableSize, m_decodingSaltCount,GetNodeId(), m_decodingBitTable);
  filterPointer->inserted_element_count_ = m_decodingInsertedElementCount;
  return filterPointer;
}

BeaconHeader::BeaconHeader() {
  m_eBFInsertedElementCount=0;
  m_eBFTableSize=0;
  m_eBFBitTable=NULL;
}


BeaconHeader::~BeaconHeader() {

}


uint32_t BeaconHeader::GetSerializedSize (void) const {
  if (m_panicked) {
    return 25 + (m_decodedTableSize + m_decodingTableSize+ m_eBFTableSize) / BITS_PER_CHAR;
  }
  return 24 + (m_decodedTableSize + m_decodingTableSize+ m_eBFTableSize) / BITS_PER_CHAR;
}

void BeaconHeader::Serialize (Buffer::Iterator start) const {
  Buffer::Iterator i = start;
  i.WriteU8 (m_packetType);
  i.WriteU8 (m_nodeId);
  i.WriteU8 (m_destId);
  i.WriteU16 (m_pktIndex);
  i.WriteU8 (m_neighborhoodSize);
  i.WriteU8 (m_remainingCapacity);
  i.WriteU8 (m_decodingBufSize);
  i.WriteU16 (m_decodedTableSize);
  for (std::size_t j=0; j < (m_decodedTableSize / BITS_PER_CHAR); j++) {
      i.WriteU8 (m_decodedBitTable[j]);
  }
  i.WriteU16 (m_decodedInsertedElementCount);
  i.WriteU8 (m_decodedSaltCount);
  i.WriteU16 (m_decodingTableSize);
  for (std::size_t j=0; j < (m_decodingTableSize / BITS_PER_CHAR); j++) {
      i.WriteU8 (m_decodingBitTable[j]);
  }
  i.WriteU16 (m_decodingInsertedElementCount);
  i.WriteU8 (m_decodingSaltCount);
  i.WriteU16 (m_eBFTableSize);
  for (std::size_t j=0; j < (m_eBFTableSize / BITS_PER_CHAR); j++) {
      i.WriteU8 (m_eBFBitTable[j]);
  }
  i.WriteU16 (m_eBFInsertedElementCount);
  i.WriteU8 (m_eBFSaltCount);
  if (m_panicked) {
    i.WriteU8(1);
    i.WriteU8(m_panickingNode);
  } else
    i.WriteU8(0);
}

uint32_t BeaconHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_packetType = i.ReadU8 ();
  m_nodeId = i.ReadU8 ();
  m_destId =i.ReadU8 ();
  m_pktIndex = i.ReadU16();
  m_neighborhoodSize = i.ReadU8 ();
  m_remainingCapacity = i.ReadU8 ();
  m_decodingBufSize = i.ReadU8 ();
  m_decodedTableSize = i.ReadU16 ();
  if (m_decodedBitTable!=NULL) {
    delete[] m_decodedBitTable;
  }
  m_decodedBitTable = new unsigned char[m_decodedTableSize /BITS_PER_CHAR];
  for (std::size_t j=0; j < (m_decodedTableSize / BITS_PER_CHAR); j++) {
    m_decodedBitTable[j]=i.ReadU8();
  }
  m_decodedInsertedElementCount=i.ReadU16 ();
  m_decodedSaltCount=i.ReadU8 ();
  m_decodingTableSize=i.ReadU16 ();
  if (m_decodingBitTable!=NULL) {
    delete[] m_decodingBitTable;
  }
  m_decodingBitTable = new unsigned char[m_decodingTableSize /BITS_PER_CHAR];
  for (std::size_t j=0; j < (m_decodingTableSize / BITS_PER_CHAR); j++) {
    m_decodingBitTable[j]=i.ReadU8();
  }
  m_decodingInsertedElementCount = i.ReadU16 ();
  m_decodingSaltCount=i.ReadU8 ();
  m_eBFTableSize = i.ReadU16 ();
  if (m_eBFBitTable!=NULL) {
    delete[] m_eBFBitTable;
  }
  m_eBFBitTable = new unsigned char[m_eBFTableSize /BITS_PER_CHAR];
  for (std::size_t j=0; j < (m_eBFTableSize / BITS_PER_CHAR); j++)
  {
    m_eBFBitTable[j] = i.ReadU8 ();
  }
  m_eBFInsertedElementCount = i.ReadU16 ();
  m_eBFSaltCount=i.ReadU8 ();
// we return the number of bytes effectively read.
  m_panicked=(i.ReadU8()==1);
  if (m_panicked) {
    m_panickingNode=i.ReadU8();
  }
  return GetSerializedSize ();
}

void BeaconHeader::PuteBF (Ptr<MyBloom_filter> nodeFilterPointer) {
  m_eBFTableSize = nodeFilterPointer->size();
  m_eBFSaltCount= nodeFilterPointer->salt_count();
  m_eBFRandomSeed= nodeFilterPointer->seed();
  m_eBFBitTable = new unsigned char[m_eBFTableSize / BITS_PER_CHAR];
  std::copy (nodeFilterPointer->table(), nodeFilterPointer->table() + (m_eBFTableSize / BITS_PER_CHAR), m_eBFBitTable);
  m_eBFInsertedElementCount = nodeFilterPointer->element_count();
}

Ptr<MyBloom_filter>  BeaconHeader::GeteBF () const {
  Ptr<MyBloom_filter> filterPointer;
  filterPointer = CreateObject<MyBloom_filter> (m_eBFTableSize, m_eBFSaltCount, GetNodeId(), m_eBFBitTable);
  filterPointer->inserted_element_count_ = m_eBFInsertedElementCount;
  return filterPointer;
}

TypeId BeaconHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void BeaconHeader::Print (std::ostream &os) const
{
  os<<"Source="<< m_nodeId <<" Destination=" << m_destId
  <<" nodeId="<< (int)m_nodeId
  <<" m_linCombSize = "<<m_linCombSize;
	for (uint8_t k=0; k<m_linCombSize; k++){
    std::cout <<" m_linComb["<<k<<"].pktId = "<<m_linComb[k].nodeId<<":"<<m_linComb[k].index <<" m_linComb["<<k<<"].coeff = "<<(int)m_linComb[k].coeff;
	}
}

DecodedPacketStorage::DecodedPacketStorage(Ptr<NetworkCodedDatagram> nc){
  MapType::iterator it=nc->m_coefsList.begin();
  attribute.m_nodeId=it->second.m_nodeId;
  attribute.m_index=it->second.m_index;
	attribute.m_destId=it->second.m_destId;
	attribute.m_genTime=it->second.GetGenTime();
	attribute.m_receptionNum=0;
	attribute.m_sendingNum=0;
  ncDatagram=nc;
}

DecodedPacketStorage::~DecodedPacketStorage() {

}

MyNCApp::MyNCApp(): Application()
{
  m_amSource=false;
  m_idle=true;
  m_changed=false;
  m_degraded=false;
  m_port=5000;
  m_pktSize=1000;
  MCLU=10;
//  predictedElementCount=PEC;
  falsePositiveProbability=DFPP;
  m_running=false;
  m_pktIndex=0;
	/*
	 p(x) = 1x^8+1x^7+0x^6+0x^5+0x^4+0x^3+1x^2+1x^1+1x^0
	 1    1    0    0    0    0    1    1    1
	 */
	unsigned int poly[9] = {1,1,1,0,0,0,0,1,1};
	try {
    m_nodeGaloisField = new galois::GaloisField (8, poly);
  }
	catch (std::bad_alloc&)
  {
    std::cout << "Error allocating memory to nodeGaloisField field." << std::endl;
  }
  nDuplicateRec=0;
	m_buffer. clear ();
	m_decodedBuf. clear ();
  smalldecodedBFIndex=0;
  m_decodedList.clear ();
	m_decodingBuf. clear ();
	m_varList. clear ();
	m_deliveredList.clear ();
	m_packetInterval=0.08;
	m_beaconInterval=1.0;
	double mean = 1;
	expVar= CreateObject<ExponentialRandomVariable> ();
	expVar->SetAttribute ("Mean", DoubleValue (mean));
	double l = 0;
	double s = 1;
	uniVar=CreateObject<UniformRandomVariable> ();
	uniVar->SetAttribute ("Min", DoubleValue (l));
	uniVar->SetAttribute ("Max", DoubleValue (s));
  m_inPanicMode=false;
  m_panicked=false;
  m_oldwaitingList=0;
}

MyNCApp::~MyNCApp()
{}

void MyNCApp::MakeSource(){
	m_amSource=true;
	m_nGeneratedPackets=0;
	m_nInjectedPackets=0;
	m_nErasedElements=0;
  m_waitingList.clear ();
}


void
MyNCApp::SetupSockets () {
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  sinkSock = Socket::CreateSocket (GetNode(), tid);
	sourceSock = Socket::CreateSocket (GetNode(), tid);
  beaconSock=sourceSock;
  m_myNCAppIp = GetNode()->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
	InetSocketAddress local = InetSocketAddress (m_myNCAppIp, m_port);
  sinkSock->Bind (local);
  sinkSock->SetRecvCallback (MakeCallback (&MyNCApp::Receive, this));
	InetSocketAddress remote = InetSocketAddress (Ipv4Address ("10.1.1.255"), m_port);
	sourceSock->SetAllowBroadcast (true);
	sourceSock->Connect (remote);
}

void
MyNCApp::SetupBeacon () {
  nGeneratedBeacons=0;
  nReceivedBeacons=0;
  double expireTime = 0.5*expVar->GetValue();
  Simulator::Schedule (Seconds (expireTime), &MyNCApp::GenerateBeacon, this, true);
}

void
MyNCApp::GenerateBeacon (bool isperm) {
  PktTypeTag tag;
	std::map<uint8_t,Neighbor>::iterator it;
  int sizeDecodedList, sizeDeliveredList;

	Time now = Simulator::Now ();
	// First check if there is inactive neighbors
  int nodeId;
	for (it=m_neighborhood.begin(); it!=m_neighborhood.end();) {
		if (now.GetSeconds() - it->second.lastReceptionTime > NEIGHBOR_TIMER) {
      nodeId=it->first;
			NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" deleted neighbor "<<(int)it->second.neighborId<<" in "<<m_myNodeId<<"'s neighborhood !");
      it++;
      if (it->second.panicked) {
        if (m_inPanicMode){
          m_inPanicMode=false;
          NS_LOG_UNCOND("t= "<<now.GetSeconds()<<" NID: "<<m_myNodeId<<" out of panic mode from "<<(int)it->first);
          std::map<uint8_t, Neighbor>::iterator itr;
          for(itr=it;itr!=m_neighborhood.end();itr++){
            if (itr->second.panicked) {
              m_inPanicMode=true;
              break;
            }
          }
        }
      }
			m_neighborhood.erase(nodeId);
		} else {
      ++it;
    }
	}
  m_myNeighborhoodSize=m_neighborhood.size();
  if (m_neighborhood.empty()){//no more neighbors
		m_idle=true;
    NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" Node : " << m_myNodeId<<" Go to sleep");
	}
	NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" broadcast B"<<StringConcat(m_myNodeId,m_pktIndex));

  Ptr<Packet> beaconPacket = Create<Packet> ();
  Ptr<MyBloom_filter> decodingFilter =CreateObject<MyBloom_filter> (std::max((int)m_varList.size(),1), DFPP ,m_myNodeId);
  sizeDecodedList = std::min((int)bigThreshold, (int)m_decodedList.size()) + MAX_VARLIST_SIZE;
	Ptr<MyBloom_filter> decodedFilter = CreateObject<MyBloom_filter> (sizeDecodedList, DFPP , m_pktIndex);
  sizeDeliveredList = std::min((int)bigThreshold, std::max((int)m_deliveredList.size(), 1));
	Ptr<MyBloom_filter> eBF = CreateObject<MyBloom_filter> (std::max(10,sizeDeliveredList) , DFPP/100 , m_myNodeId);

  tag.SetType(0);
  beaconPacket->AddPacketTag(tag);
  BeaconHeader beaconHeader;
  beaconHeader.SetDestination (255);// means broadcast
  beaconHeader.SetPacketType (0);
  beaconHeader.SetNodeId (m_myNodeId);
  beaconHeader.SetPktIndex(m_pktIndex++);
  std::map<std::string, NCAttribute >::iterator itr;
	for (itr=m_varList.begin();itr!=m_varList.end();itr++)
	{
	    //No No ! we insert string in BFs !!!
		decodingFilter->insert(itr->first);
	}
	std::vector<Ptr<DecodedPacketStorage> >::reverse_iterator itr2=m_decodedBuf.rbegin();
  for (int i=0; (i<sizeDecodedList) && (itr2!=m_decodedBuf.rend()) ; itr2++, i++) {
    if (!((*itr2)->attribute.m_destReceived))
      decodedFilter->insert ((*itr2)->attribute.Key());
  }
  smalldecodedBFIndex=std::max((int)m_decodedBuf.size()-1,0);
  std::vector<std::string>::iterator itr3=m_deliveredList.begin();
  std::string str;
  for(int i=0; ((i<sizeDeliveredList) && (itr3!=m_deliveredList.end())); i++, itr3++){
    str=*itr3;
    eBF->insert (str);
  }

	beaconHeader.PutDecodingBloomFilter (decodingFilter);
	beaconHeader.PutDecodedBloomFilter (decodedFilter);
	beaconHeader.PuteBF(eBF);
	beaconHeader.SetNeighborDecodingBufSize ((uint8_t)m_decodingBuf.size());
	beaconHeader.SetRemainingCapacity((uint8_t) MAX_VARLIST_SIZE - m_varList.size()-1);
	beaconHeader.SetNeighborhoodSize((uint16_t) m_neighborhood.size());
  if (m_panicked){
    beaconHeader.m_panicked=true;
    beaconHeader.m_panickingNode=m_panickingNode;
  } else {
    beaconHeader.m_panicked=false;
  }

	beaconPacket-> AddHeader (beaconHeader);
//	delete tempFilter1;
//	delete tempFilter2;
	sourceSock-> Send (beaconPacket);
	nGeneratedBeacons++;
  RemoveOldVariables();
  if (isperm){
	  Simulator::Schedule (Seconds (m_beaconInterval), &MyNCApp::GenerateBeacon, this,isperm);
  }
}

void MyNCApp::Start(){
  m_running = true;
  m_myNodeId=GetNode()->GetId();
  nGeneratedBeacons=0;
  nReceivedBeacons=0;
  nDuplicateRec=0;
  nSrcForwardedPackets=0;
  nForwardedPackets=0;
  nReceivedPackets=0;
  nReceivedBytes=0;
  packetDelay=0;
  nDroppedPackets=0;
  oldestDiscardedNum=0;
  m_rank=0;
  m_rcvCounter=counter=0;
  m_myNeighborhoodSize=0;
  SetupSockets();
  SetupBeacon();
  if (m_amSource){
		GeneratePacket();
	}
}

void
MyNCApp::Stop () {
  m_running = false;
  if (sourceSock) {
    sourceSock->Close();
  }
  if (sinkSock) {
    sinkSock->Close();
  }
}

void MyNCApp::UpdateNeighborListBeacon(BeaconHeader header, Ipv4Address senderIp) {
	Time now = Simulator::Now ();
	std::map<uint8_t, Neighbor>::iterator listIterator;
	std::string tmpStr;

  listIterator=m_neighborhood.find(header.GetNodeId ());
  if (listIterator!=m_neighborhood.end()){
    listIterator->second.lastReceptionTime = now.GetSeconds();
    listIterator->second.neighborRemainingCapacity = header.GetRemainingCapacity();
    listIterator->second.neighborhoodSize = header.GetNeighborhoodSize();
    listIterator->second.neighborDecodingBufSize = header.GetNeighborDecodingBufSize();
    listIterator->second.neighborDecodingFilter = header.GetDecodingBloomFilter ();
    listIterator->second.bigNeighborDecodedFilter = header.GetDecodedBloomFilter ();
    listIterator->second.neighborReceivedFilter= header.GeteBF();
    listIterator->second.neighborReceivedBeacons++;
    if (header.m_panicked){
      if (!listIterator->second.panicked){
        listIterator->second.panicked=true;
        if (header.m_panickingNode==m_myNodeId){
          m_inPanicMode=true;
          NS_LOG_UNCOND("t= "<<now.GetSeconds()<<" NID: "<<m_myNodeId<<" in panic mode from "<<(int)listIterator->first);
        }
      }
    } else {
      if (listIterator->second.panicked){
        listIterator->second.panicked=false;
        if (m_inPanicMode){
          m_inPanicMode=false;
          NS_LOG_UNCOND("t= "<<now.GetSeconds()<<" NID: "<<m_myNodeId<<" out of panic mode from "<<(int)listIterator->first);
          std::map<uint8_t, Neighbor>::iterator itr;
          for(itr=m_neighborhood.begin();itr!=m_neighborhood.end();itr++){
            if (itr->second.panicked) {
              m_inPanicMode=true;
              break;
            }
          }
        }
      }
    }
  } else {
		Neighbor neighbor;
		neighbor.firstReceptionTime = neighbor.lastReceptionTime = now.GetSeconds ();
		neighbor.neighborRemainingCapacity = header.GetRemainingCapacity();
		neighbor.neighborId = header.GetNodeId ();
		neighbor.neighborIp = senderIp;
		neighbor.neighborhoodSize = header.GetNeighborhoodSize ();
		neighbor.neighborDecodingBufSize = header.GetNeighborDecodingBufSize();
    neighbor.neighborDecodingFilter =header.GetDecodingBloomFilter ();
    neighbor.bigNeighborDecodedFilter =  header.GetDecodedBloomFilter ();
    neighbor.neighborReceivedFilter= header.GeteBF();
    neighbor.neighborReceivedBeacons++;
    if (header.m_panicked){
      neighbor.panicked=true;
      if (header.m_panickingNode==m_myNodeId){
        m_inPanicMode=true;
        NS_LOG_UNCOND("t= "<<now.GetSeconds()<<" NID: "<<m_myNodeId<<" in panic mode from "<<(int)listIterator->first);
      }
    } else {
      neighbor.panicked=false;
    }
    NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" nodeId "<<(int)(neighbor.neighborId)<<" Became neighbor of nodeId "<<m_myNodeId);
  	m_neighborhood[header.m_nodeId]=neighbor;
    GenerateBeacon(false);
    if (m_idle) {
      m_idle=false;//awake the node to send data
      NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" Node : " << m_myNodeId<<" Get awake");
      m_changed=true;
      Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
    }
	}
	m_myNeighborhoodSize=m_neighborhood.size();
	if (m_amSource){
    CheckWaitingList(header.m_nodeId);
	}
}

void MyNCApp::UpdateNeighborListStatus(StatusFeedbackHeader header, Ipv4Address senderIp) {
	Time now = Simulator::Now ();
	std::map<uint8_t, Neighbor>::iterator listIterator;
	std::string tmpStr;

  listIterator=m_neighborhood.find(header.GetNodeId ());
  if (listIterator!=m_neighborhood.end()){
    listIterator->second.lastReceptionTime = now.GetSeconds();
    listIterator->second.neighborRemainingCapacity = header.GetRemainingCapacity();
    listIterator->second.neighborhoodSize = header.GetNeighborhoodSize();
    listIterator->second.neighborDecodingBufSize = header.GetNeighborDecodingBufSize();
    if (header.m_panicked){
      listIterator->second.panicked=true;
      if (header.m_panickingNode==m_myNodeId){
        m_inPanicMode=true;
        NS_LOG_UNCOND("t= "<<now.GetSeconds()<<" NID: "<<m_myNodeId<<" in panic mode from "<<(int)listIterator->first);
      }
    } else {
      if (listIterator->second.panicked){
        listIterator->second.panicked=false;
        if (m_inPanicMode){
          std::map<uint8_t, Neighbor>::iterator itr;
          m_inPanicMode=false;
          NS_LOG_UNCOND("t= "<<now.GetSeconds()<<" NID: "<<m_myNodeId<<" out of panic mode from "<<(int)listIterator->first);
          for(itr=m_neighborhood.begin();itr!=m_neighborhood.end();itr++){
            if (itr->second.panicked) {
              m_inPanicMode=true;
              break;
            }
          }
        }
      }
    }
    switch (header.GetPacketType ())  {
      case 1: {
        listIterator->second.neighborDecodingFilter = header.GetDecodingBloomFilter ();
        listIterator->second.smallNeighborDecodedFilter = header.GetDecodedBloomFilter ();
        listIterator->second.neighborReceivedPackets++;
        break;
      }
      case 2: {
        listIterator->second.neighborReceivedStatusFeedbacks++;
        listIterator->second.neighborDecodingFilter = header.GetDecodingBloomFilter ();
        listIterator->second.smallNeighborDecodedFilter = header.GetDecodedBloomFilter ();
        break;
      }
    }
	} else {
		Neighbor neighbor;
		neighbor.firstReceptionTime = neighbor.lastReceptionTime = now.GetSeconds ();
		neighbor.neighborRemainingCapacity = header.GetRemainingCapacity();
		neighbor.neighborId = header.GetNodeId ();
		neighbor.neighborIp = senderIp;
		neighbor.neighborhoodSize = header.GetNeighborhoodSize ();
		neighbor.neighborDecodingBufSize = header.GetNeighborDecodingBufSize();
    NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" nodeId "<<(int)(neighbor.neighborId)<<" Became neighbor of nodeId "<<m_myNodeId);
    GenerateBeacon(false);
    if (header.m_panicked){
      neighbor.panicked=true;
      if (header.m_panickingNode==m_myNodeId){
        m_inPanicMode=true;
        NS_LOG_UNCOND("t= "<<now.GetSeconds()<<" NID: "<<m_myNodeId<<" in panic mode from "<<(int)listIterator->first);
      }
    } else {
      neighbor.panicked=false;
    }
    switch (header.GetPacketType ())  {
      case 1: {
        neighbor.neighborDecodingFilter = header.GetDecodingBloomFilter ();
  			neighbor.smallNeighborDecodedFilter = header.GetDecodedBloomFilter ();
        neighbor.neighborReceivedPackets++;
        break;
      }
      case 2: {
        neighbor.neighborDecodingFilter = header.GetDecodingBloomFilter ();
  			neighbor.smallNeighborDecodedFilter = header.GetDecodedBloomFilter ();
        neighbor.neighborReceivedStatusFeedbacks++;
        break;
      }
    }
    m_neighborhood[header.m_nodeId]=neighbor;
		if (m_idle) {
			m_idle=false;//awake the node to send data
      NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" Node : " << m_myNodeId<<" Get awake");
      m_changed=true;
			Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
		}
	}
	m_myNeighborhoodSize=m_neighborhood.size();
  if (m_amSource){
    CheckWaitingList(header.m_nodeId);
	}
}

bool MyNCApp::UpdateNeighorhoodEst(MapType coefsList){
  std::map<uint8_t, Neighbor>::iterator listIterator;
  MapType::iterator it;
  int utility=0;
  std::string pktId;
	//we assume any forward packet is entering into all neighbors varList and therefore appears in neighborDecodingFilter
  for (listIterator=m_neighborhood.begin(); listIterator!=m_neighborhood.end(); listIterator++) {
    listIterator->second.neighborForwardedPackets++;
    listIterator->second.neighborDecodingBufSize++;
    if (coefsList.size()==1) {
      pktId=coefsList.begin()->first;
      listIterator->second.bigNeighborDecodedFilter->insert(pktId);
      utility++;
    } else {
      for (it=coefsList.begin();it!=coefsList.end();it++){
        pktId=it->first;
        if (!listIterator->second.smallNeighborDecodedFilter->contains(pktId))  {
          if (!listIterator->second.bigNeighborDecodedFilter->contains(pktId)) {
            if (!listIterator->second.neighborDecodingFilter->contains(pktId)) {
              listIterator->second.neighborRemainingCapacity--;
              listIterator->second.neighborDecodingFilter->insert(pktId);
            }
          }
        }
      }
      if (listIterator->second.neighborDecodingBufSize<MAX_VARLIST_SIZE-listIterator->second.neighborRemainingCapacity){
        utility++;
      }
    }
	}
  if (utility>0){
    return true;
  }
  return false;
}

void MyNCApp::Receive (Ptr<Socket> socket)
{
	Time now = Simulator::Now();
	Address from;
	Ptr<Packet> packetIn = socket -> RecvFrom (from);
	Ipv4Address senderIp = InetSocketAddress::ConvertFrom (from).GetIpv4 ();
  PktTypeTag tag;
  packetIn->RemovePacketTag(tag);
  uint8_t pktType=tag.GetType();
  //std::list<Neighbor> itr;
  switch(pktType){
    case 0: {
      BeaconHeader bcnHeader;
      nReceivedBeacons++;
      packetIn-> RemoveHeader(bcnHeader);
      NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" Received B"<<StringConcat(bcnHeader.m_nodeId,bcnHeader.m_pktIndex)<<" in  "<<m_myNodeId);
      UpdateNeighborListBeacon(bcnHeader, senderIp);
      RemoveDeliveredPackets(bcnHeader.m_nodeId);
      break;
      if (!m_changed) {
        m_changed=true;
        Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
      }
    }
    case 1: {
      StatusFeedbackHeader statusHeader;
      packetIn-> RemoveHeader(statusHeader);
      NS_LOG_UNCOND("t = "<< now.GetSeconds ()<< " Received D"<<StringConcat(statusHeader.m_nodeId,statusHeader.m_pktIndex) <<" in "<<m_myNodeId);
      UpdateNeighborListStatus(statusHeader, senderIp);
      if (!m_changed){
        m_changed=true;
        Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
      }
      Ptr<NetworkCodedDatagram> nc= CreateObject<NetworkCodedDatagram>();
      CoefElt coef;
      for (size_t j=0 ;j < statusHeader.GetLinearCombinationSize (); j++) {
        coef.SetCoef (statusHeader.m_linComb[j].coeff);
        coef.SetDestination (statusHeader.m_linComb[j].dstId);
        coef.SetGenTime(statusHeader.m_linComb[j].genTime);
        coef.SetNodeId(statusHeader.m_linComb[j].nodeId);
        coef.SetIndex(statusHeader.m_linComb[j].index);
        nc ->m_coefsList[statusHeader.m_linComb[j].Key()]=coef;
      }
      std::map<uint8_t, Neighbor>::iterator itr=m_neighborhood.find(statusHeader.m_nodeId);
      m_rcvCounter++;
      Reduce(*nc);
      if(nc->m_coefsList.size() == 0) {
        nDuplicateRec++;
        NS_LOG_UNCOND("t = " << Simulator::Now().GetSeconds() << " NODE ID: "<< m_myNodeId<<" ACT: Useless packet!!!");
        if (!(m_changed)){
          //send a packet announcing the current state
          Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
        }
        return;
      }
      if (nc->m_coefsList.size()==1){//Easy to decode
        std::map<std::string, NCAttribute>::iterator it=m_varList.find(nc->m_coefsList.begin()->first);
        if (it==m_varList.end()){
          uint8_t pivot=nc->m_coefsList.begin()->second.GetCoef();
          assert(pivot!=0);
          if (pivot!=1){
            uint8_t alpha=m_nodeGaloisField->div(1,pivot);
            nc->Product(alpha,m_nodeGaloisField);
          } // nc is completely decoded
          insertIntoDecodedBuf(nc);
          return;
        }
      }
      if (CheckCapacity(*nc)) {
        Decode (nc);
        if (itr->second.punished){
            itr->second.punished=false;
            itr->second.numConsDrop=0;
            m_panicked=false;
            std::map<uint8_t, Neighbor>::iterator listIterator;
            for (listIterator=m_neighborhood.begin();listIterator!=m_neighborhood.end();listIterator++){
              if (listIterator->second.punished) {
                m_panicked=true;
                break;
              }
            }
            if (!m_panicked)
              NS_LOG_UNCOND("t = "<< Simulator::Now().GetSeconds()<<" NID: "<< m_myNodeId<<" end panic!");
        }
      } else {
        nDroppedPackets++;
        NS_LOG_UNCOND("Dropped packet at NODE ID:" <<m_myNodeId);
        int threshold=1;
        if (itr->second.numConsDrop<=threshold){
          itr->second.numConsDrop++;
        } else {
          itr->second.punished=true;
          m_panicked=true;
          m_panickingNode=statusHeader.GetNodeId();
          NS_LOG_UNCOND("t = "<< Simulator::Now().GetSeconds()<<" NID: "<< m_myNodeId<<" panicked !");
        }
        return;
      }
      break;
    }
    case 2:{
      StatusFeedbackHeader statusHeader;
      packetIn-> RemoveHeader(statusHeader);
      NS_LOG_UNCOND("t = "<< now.GetSeconds ()<< " Received S"<<StringConcat(statusHeader.m_nodeId,statusHeader.m_pktIndex) <<" in "<<m_myNodeId);
      UpdateNeighborListStatus(statusHeader, senderIp);
      if (!m_changed){
        m_changed=true;
        Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
      }
      break;
    }
  }
}

void MyNCApp::Forward ()
{
  PktTypeTag tag;
  Time now = Simulator::Now();
  bool sendStatus=false;
  if (!m_idle) {
    std::string tmpStr;
    int sizeDecodedList;
    StatusFeedbackHeader lcHeader;
    Ptr<MyBloom_filter> decodingFilter=CreateObject<MyBloom_filter> (MAX_VARLIST_SIZE, DFPP , m_myNodeId);
    sizeDecodedList=max((int)m_decodedBuf.size()-smalldecodedBFIndex,1);
    Ptr<MyBloom_filter> decodedFilter = CreateObject<MyBloom_filter> (sizeDecodedList, DFPP , m_pktIndex);
    //Ptr<MyBloom_filter> eBF = CreateObject<MyBloom_filter> (predictedElementCount, falsePositiveProbability , m_myNodeId);
    std::map<std::string, NCAttribute >::iterator itr;
    for (itr=m_varList.begin();itr!=m_varList.end();itr++) {
      decodingFilter->insert(itr->first);
    }
    std::vector<Ptr<DecodedPacketStorage> >::iterator itr2;
    std::string str;
    for (itr2=m_decodedBuf.begin()+smalldecodedBFIndex;itr2!=m_decodedBuf.end();itr2++) {
      str=(*itr2)->attribute.Key();
      decodedFilter->insert (str);
    }
    Ptr<Packet> lcPacket = Create<Packet> (m_pktSize);
    if (!m_decodedBuf.empty () || !m_decodingBuf.empty()) {
      Ptr<NetworkCodedDatagram> tmpEncDatagram;
      tmpEncDatagram = CreateObject<NetworkCodedDatagram>();
      if (m_inPanicMode){
        tmpEncDatagram =PanicEncode();
      } else {
        tmpEncDatagram = Encode();
      }
      if (tmpEncDatagram!= NULL && !tmpEncDatagram->IsNull()) {
        lcHeader.SetPacketType (1);
        lcHeader.SetNodeId (m_myNodeId);
        lcHeader.SetDestination(255);
        lcHeader.SetPktIndex(m_pktIndex++);
        lcHeader.PutDecodingBloomFilter (decodingFilter);
        lcHeader.PutDecodedBloomFilter (decodedFilter);
        lcHeader.SetNeighborhoodSize ((uint16_t) m_neighborhood.size());
        lcHeader.SetNeighborDecodingBufSize ((uint8_t)m_decodingBuf.size());
        lcHeader.SetRemainingCapacity (std::max((int)(MAX_VARLIST_SIZE - m_varList.size()-1),0));
        if (m_panicked){
          lcHeader.m_panicked=true;
          lcHeader.m_panickingNode=m_panickingNode;
        } else {
          lcHeader.m_panicked=false;
        }
        //the line below has changed in the forthcoming: each var should have its own genTime
        //lcHeader.SetTime(tmpEncDatagram->m_genTime);//Write the datagram's generation time in header
        MapType::iterator it;
        LinearCombination lc;
        lcHeader.m_linComb.clear();
        if (UpdateNeighorhoodEst(tmpEncDatagram->m_coefsList)) {
          for (it=tmpEncDatagram->m_coefsList.begin (); it!=tmpEncDatagram->m_coefsList.end (); it++) {
            lc.nodeId = (*it).second.GetNodeId();
            lc.index=(*it).second.GetIndex();
            if (it->second.GetNodeId()==m_myNodeId) {// we are the source of the packet so we have to update waitingList to do backpressure
              UpdateWaitingList(lc.Key());
            }
            lc.coeff = (*it).second.GetCoef();
            lc.dstId = (*it).second.GetDestination();
            lc.genTime= (*it).second.GetGenTime();
            lcHeader.m_linComb.push_back (lc);
          }
          lcHeader.SetLinearCombinationSize ();
          lcPacket->AddHeader (lcHeader);
          lcPacket->RemoveAllPacketTags ();
          lcPacket->RemoveAllByteTags ();
          tag.SetType(1);
          lcPacket->AddPacketTag(tag);
          sourceSock->Send (lcPacket);
          NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" packet broadcast D"<<StringConcat(m_myNodeId,m_pktIndex-1));
          Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
        } else {
          sendStatus=true;
        }
      } else {
        sendStatus=true;
      }
    } else {
      sendStatus=true;
    }
    if (sendStatus){
      StatusFeedbackHeader statusFbHeader;
      statusFbHeader.SetPacketType (2);
      statusFbHeader.SetNodeId (m_myNodeId);
      statusFbHeader.SetDestination(255);
      statusFbHeader.SetPktIndex(m_pktIndex++);
      statusFbHeader.PutDecodingBloomFilter (decodingFilter);
      statusFbHeader.PutDecodedBloomFilter (decodedFilter);
      statusFbHeader.SetNeighborhoodSize ((uint16_t) m_neighborhood.size());
      statusFbHeader.SetNeighborDecodingBufSize ((uint8_t)m_decodingBuf.size());
      statusFbHeader.SetRemainingCapacity (std::max((int)(MAX_VARLIST_SIZE - m_varList.size()-1),0));
      statusFbHeader.SetLinearCombinationSize();
      if (m_panicked){
        statusFbHeader.m_panicked=true;
        statusFbHeader.m_panickingNode=m_panickingNode;
      } else {
        statusFbHeader.m_panicked=false;
      }
      lcPacket->AddHeader (statusFbHeader);
      lcPacket->RemoveAllPacketTags ();
      lcPacket->RemoveAllByteTags ();
      tag.SetType(2);
      lcPacket->AddPacketTag(tag);
      sourceSock->Send (lcPacket);
      NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" packet broadcast S"<<StringConcat(m_myNodeId,m_pktIndex-1));
      NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" Node " <<m_myNodeId<<" changed=false");
      m_changed=false;
    }
  }
}

Ptr<NetworkCodedDatagram> MyNCApp::PanicEncode () {
  int L,l,len;
  Time now = Simulator::Now ();

  L = m_decodedBuf.size();
  if (m_degraded) {
    len = m_decodedBuf.size();
    l = m_decodedBuf.size();
  } else {
    len = m_decodedBuf.size()+m_decodingBuf.size();
    l = m_decodedBuf.size() + m_varList.size();
  }
  std::map<uint8_t, Neighbor>::iterator listIterator;
  // For encoding we should have received packets
  if (l==0){
    return NULL;
  }
  assert(len !=0);

  bool destNeighbor=false;
  std::vector<double> var,F;
  F.resize(len);
  std::vector<Ptr<DecodedPacketStorage> >::iterator itr;
  for (listIterator=m_neighborhood.begin(); listIterator!=m_neighborhood.end(); listIterator++) {
    if (listIterator->second.panicked){
      //let's first iterate over the decoded packets
      int i=0;
      for (itr=m_decodedBuf.begin();itr!=m_decodedBuf.end();itr++) {
        std::string tmpStr = (*itr)->attribute.Key();
        if (!(listIterator->second.smallNeighborDecodedFilter->contains(tmpStr)) &&
          !(listIterator->second.bigNeighborDecodedFilter->contains(tmpStr)) &&
          !((*itr)->attribute.m_destReceived)&&
          (now.GetMilliSeconds()-(*itr)->attribute.GetGenTime()<VAR_LIFETIME*1000)) {
          if (listIterator->second.neighborDecodingFilter->contains(tmpStr)) {
            F.at(i)=F.at(i) + 10;
          } else {
            F.at(i)=F.at(i) + 1;
          }
          if ((*itr)->attribute.m_destId ==listIterator->second.neighborId){
            destNeighbor=true;
            F.at(i)=F.at(i) + 5;
          }
        }
        i++;
      }//for loop over decoded packets
    }
  }//end of iteration over the neighborhood
  std::vector<double>::iterator maxElement = std::max_element(F.begin(),F.end());
  int index=maxElement-F.begin();
  if (F[index]>0)
    return m_decodedBuf[index]->ncDatagram;
  //case 2: nothing new to send
  NS_LOG_UNCOND("Blocked situation for "<<m_myNodeId);
  return NULL;
}


Ptr<NetworkCodedDatagram> MyNCApp::Encode () {
  int L,l,len;
  int coef;//choice, order;
  NetworkCodedDatagram g;
  Ptr<NetworkCodedDatagram> nc= CreateObject<NetworkCodedDatagram>();
  Time now = Simulator::Now ();

  L = m_decodedBuf.size();
  if (m_degraded) {
    len = m_decodedBuf.size();
    l = m_decodedBuf.size();
  } else {
    len = m_decodedBuf.size()+m_decodingBuf.size();
    l = m_decodedBuf.size() + m_varList.size();
  }
  std::map<uint8_t, Neighbor>::iterator listIterator;
  // For encoding we should have received packets
  if (l==0){
    return NULL;
  }
  assert(len !=0);

  int numVar=0;
  int index=0;
  double val;
  bool destNeighbor=false;
  std::vector<double> var,F, A, B;
  m_lpMatrix.A.clear();
  m_lpMatrix.SetDimensions((int) m_neighborhood.size(), l);
  B.clear();
  B.resize(m_neighborhood.size());
  F.resize(len);
  A.resize(len);
  for (int i=0;i<len;A.at(i++)=0);
  std::vector<Ptr<DecodedPacketStorage> >::iterator itr;
//  std::vector<int> indices(m_decodedBuf.size());
//  for (int j=0;j<indices.size();j++){
//    indices[j]=j;
//  }
//  std::vector<int>::iterator it;
//  std::random_shuffle ( indices.begin(), indices.end() );
  for (listIterator=m_neighborhood.begin(); listIterator!=m_neighborhood.end(); listIterator++) {
    B.at(index)=listIterator->second.neighborRemainingCapacity/m_neighborhood.size();
    //let's first iterate over the decoded packets
    int i=0;
    for (itr=m_decodedBuf.begin();itr!=m_decodedBuf.end();itr++) {

//    for (it=indices.begin();it!=indices.end();it++) {
//      itr=m_decodedBuf.begin()+(*it);
      std::string tmpStr = (*itr)->attribute.Key();
      F.at(i)=0;
      if (!(listIterator->second.smallNeighborDecodedFilter->contains(tmpStr)) &&
      !(listIterator->second.bigNeighborDecodedFilter->contains(tmpStr)) &&
      !((*itr)->attribute.m_destReceived) &&
      (now.GetMilliSeconds()-(*itr)->attribute.GetGenTime()<VAR_LIFETIME*1000)) {
        if (listIterator->second.neighborDecodingFilter->contains(tmpStr)) {
          F.at(i)=F.at(i) + variableWeight;
          //X.at(i)=1;
          //add the constraints here
          //constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
          //X.at(i)=0;
          A.at(i)=1;
        } else {
          F.at(i)=F.at(i) + unreceivedWeight;
          //X.at(i)=1;
          //add the constraints here
          //constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
          //X.at(i)=0;
          m_lpMatrix.SetValue(index,i,1);
          A.at(i)=1;
        }
        if ((*itr)->attribute.m_destId ==listIterator->second.neighborId){
          //Trick for faster delivery. If a destination is in your neighborhood send the packet without NC.
          //          destNeighbor=true;
          //          nc=(*itr)->ncDatagram;
          //          return nc;
          destNeighbor=true;
          F.at(i)=F.at(i) + neighborWeight;
          A.at(i)=1;
        }
      }
      i++;
    }//for loop over decoded packets
    //now let's iterate over the decodingList
    if (!m_degraded) {
      for (uint16_t i=0; i< len-L; i++) {
        m_lpMatrix.SetValue(index,i+L,0);
        //line below should change...
        numVar= (int) m_decodingBuf.at(i)->m_coefsList.size();
        MapType::iterator it;
        std::map<std::string, NCAttribute >::iterator itr;
        for (it=m_decodingBuf.at(i)->m_coefsList.begin (); it!=m_decodingBuf.at(i)->m_coefsList.end (); it++) {
          itr = m_varList.find (it->first);
          //itr2 = std::find(m_decodedList.begin(),m_decodedList.end(),(*it).second.GetAttribute ());
          assert(itr!=m_varList.end());
          //If you find the variable in the variable list
          std::string tmpStr = itr->first;
          if (!listIterator->second.smallNeighborDecodedFilter->contains(tmpStr)) {
            if (!listIterator->second.bigNeighborDecodedFilter->contains(tmpStr)) {
              if (listIterator->second.neighborDecodingFilter->contains(tmpStr)) {
                F.at(L+i)=F.at(L+i) + (double)variableWeight/numVar;
                //X.at(I+L)=1;
                //add the constraints here
                //constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
                //X.at(I+L)=0;
                A.at(L+i)=numVar;
              } else {
                //X.at(I+L)=1;
                //add the constraints here
                //constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
                //X.at(I+L)=0;
                F.at(L+i)=F.at(L+i) + (double)unreceivedWeight/numVar;
                val=m_lpMatrix.GetValue(index,i+L)+1;
                m_lpMatrix.SetValue(index,i+L,val);
                A.at(L+i)=numVar;
              }
              if (((itr->second)).m_destId==listIterator->second.neighborId) {
                F.at(i)=F.at(i) + (double)neighborWeight/numVar;
              }
            }
          }
        }// for loop over decodingList}
      }
    }
    index++;
  }//end of iteration over the neighborhood
  if(index>0) {
    //here we define our problem
    glp_prob *myLpProblem;
    myLpProblem = glp_create_prob();
    glp_set_prob_name(myLpProblem, "myLpProblem");
    glp_set_obj_dir (myLpProblem, GLP_MAX);//maximize
    glp_add_rows (myLpProblem, m_neighborhood.size()+1);
    glp_add_cols (myLpProblem, len);
    int nConstraintMatrixElements =0;
    //let us define the bounds of the cols here (0<=p_i<=1)
    for (int i=0; i<len;i++) {
      glp_set_col_bnds (myLpProblem, i+1, GLP_DB, 0.0, 1.0);
      glp_set_obj_coef(myLpProblem, i+1, F.at(i));
    }
    //let us now set the rows bounds (0<= summation of probabilities <= capacity of corresponding neighbor)
    //before to loading the constraint matrix we should prepare the required args (ia[], ja[], and ar[])
    int ia[1+5000], ja[1+5000];
    double ar[1+5000];
    int M= (int)m_neighborhood.size();
    int i=0;
    for (listIterator=m_neighborhood.begin();listIterator!=m_neighborhood.end();listIterator++) {
      for (int j=0; j<len;j++) {
        ia[i*len+j+1]=i+1;
        ja[i*len+j+1]=j+1;
        ar[i*len+j+1]=m_lpMatrix.GetValue(i,j);
        nConstraintMatrixElements++;
      }
      if (listIterator->second.neighborRemainingCapacity>0) {
        glp_set_row_bnds(myLpProblem, i+1, GLP_DB, 0.0, (int)(listIterator->second.neighborRemainingCapacity)/(m_neighborhood.size()));
      } else {
        glp_set_row_bnds(myLpProblem, i+1, GLP_FX, 0.0, 0.0);
      }
      i++;
    }
    for (int j=0; j<len;j++) {
      ia[i*len+j+1]=m_neighborhood.size()+1;
      ja[i*len+j+1]=j+1;
      ar[i*len+j+1]=A.at(j);
      nConstraintMatrixElements++;
    }
    glp_set_row_bnds(myLpProblem, M+1, GLP_DB, 0.0,MaxNumberOfCoeff);
    //let us load the constraint matrix below
    glp_load_matrix(myLpProblem,nConstraintMatrixElements, ia, ja, ar);
    glp_term_out(GLP_OFF);
    glp_simplex(myLpProblem, NULL);
    double objective= glp_get_obj_val(myLpProblem);
    if (objective>0.0) {
      std::vector<double> probabilities(len);
      for(int i=0; i<len;i++) {
        probabilities.at(i)=glp_get_col_prim(myLpProblem, i+1);
      }
      bool first=true;
      int inserted=0;
      Time now = Simulator::Now ();

      //we build F on map order so the order should be map order there !
      std::vector<Ptr<DecodedPacketStorage> >::iterator itr2;
      int i=0, varNum=0;
      for (itr2=m_decodedBuf.begin();itr2!=m_decodedBuf.end();itr2++) {
//      for (it=indices.begin();it!=indices.end();it++) {
//        itr2=m_decodedBuf.begin()+(*it);
        if ((probabilities.at(i)==1) || ((probabilities.at(i)!=0) &&
        (uniVar->GetValue(0.0,1.0) < probabilities.at(i)))) {
          varNum++;
          coef = uniVar->GetInteger (1,255);
          g=*((*itr2)->ncDatagram);
          g.Product(coef, m_nodeGaloisField );
          inserted++;
          if (first) {
            nc->operator=(g);
            first=false;
          } else {
            nc->Sum (g, m_nodeGaloisField);
          }
          //We have to update neighborhood (WL is to be updated Only in Forward)
        }
        i++;
      }//for over decodedBuf
      if (!m_degraded){
        for (uint16_t i=0;i<m_decodingBuf.size();i++) {
          if (uniVar->GetValue (0.0,1.0) < probabilities.at(i+L)){
            numVar=numVar+m_decodingBuf[i]->m_coefsList.size();
            coef = uniVar->GetInteger (1,255);
            g=(*m_decodingBuf.at(i));
            g.Product(coef, m_nodeGaloisField );
            inserted=inserted+g.m_coefsList.size();
            if (first) {
              nc->operator=(g);
              first=false;
            } else {
              nc->Sum (g, m_nodeGaloisField);
            }
          }
        }//for over decodingBuf
      }
      glp_delete_prob(myLpProblem);//we have to delete the problem at the end of the encode function
      return nc;
    } else { //Blocking situation
      //Two case can happen
      //case 1: no capacity
      std::vector<double>::iterator maxElement = std::max_element(F.begin(),F.end());
      index=maxElement-F.begin();
      if (F[index]>0)
        return m_decodedBuf[index]->ncDatagram;
    }
    //case 2: nothing new to send
    NS_LOG_UNCOND("Blocked situation for "<<m_myNodeId);
  } //if(index>0)
  return NULL;
}



void MyNCApp::Reduce (NetworkCodedDatagram& g) {
  MapType::iterator it;
  std::map<std::string, Ptr<DecodedPacketStorage> >::iterator itr;
	std::vector<Ptr<NetworkCodedDatagram> >::iterator bufItr;
  Ptr<NetworkCodedDatagram> nc;
	if (!m_decodedBuf.empty ()) {
    for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end ();) {
      itr = m_decodedList.find (it->first);
      if (itr!=m_decodedList.end()) {// we should reduce the packet
        nc= CreateObject<NetworkCodedDatagram> (*(itr->second->ncDatagram));
        nc->Product(it->second.m_coef,m_nodeGaloisField);
        it++;
        g.Minus(*nc, m_nodeGaloisField);
        if (g.m_coefsList.size()==0)  {// as their will be a coef that will be remove we should take care
          break;
        }
      } else
        it++;
    }
  }
}

bool MyNCApp::CheckCapacity(NetworkCodedDatagram& g) {
	MapType::iterator it;
	std::map<std::string, NCAttribute >::iterator itr;
  std::map<std::string, Ptr<DecodedPacketStorage> >::iterator itr2;

  if (g.m_coefsList.size ()<2){
    return true;
  }
	int newVar=0;
	for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end (); it++) {
		itr = m_varList.find (it->first);
		itr2 = m_decodedList.find(it->first);
		if (itr==m_varList.end() && itr2==m_decodedList.end()) {
			newVar++;
		}
	}

	if ((newVar>0)&&(m_varList.size()+newVar>MAX_VARLIST_SIZE )) {
		return false;
	}
	return true;
}

void
MyNCApp::UpdateVarList (NetworkCodedDatagram& g)
{
//          assert(CheckVarList());

  MapType::iterator it;
  std::map<std::string, NCAttribute >::iterator itr;
	std::map<std::string, Ptr<DecodedPacketStorage> >::iterator itr2;
  for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end (); it++) {
    itr = m_varList.find (it->first);
    itr2 = m_decodedList.find(it->first);
    if (itr==m_varList.end() && itr2==m_decodedList.end()) {
      Ptr<NCAttribute> att=CreateObject<NCAttribute>(it->second.m_nodeId,it->second.m_index,
        it->second.m_destId, it->second.m_genTime);
      m_varList[it->first]=*(att);
      assert(Simulator::Now ().GetMilliSeconds()-att->GetGenTime()<VAR_LIFETIME*1000+2000);
      m_variableList.push_back(att);
    }
//          assert(CheckVarList());
  }
}

void MyNCApp::GenerateMatrix ()
{
  Time now = Simulator::Now();
	m_matrix.A.clear ();
  std::vector<Ptr<NetworkCodedDatagram> >::iterator bufItr;
	MapType::iterator coefsLstItr;
  std::vector<Ptr<NCAttribute> >::iterator it;
  std::map<std::string, NCAttribute >::iterator varLstItr;
	Ptr<NetworkCodedDatagram> g;
	g =  CreateObject<NetworkCodedDatagram> ();
  NetworkCodedDatagram nc;
	// Number of variables
	int N = m_varList.size();
  // Number of equations
	int M = m_decodingBuf.size();
  assert(M<=N);
	m_matrix.SetDimensions (M, N);
  int j;
  bool found;
  for (int i=0;i<M;i++){
    g=m_decodingBuf[i];
    nc=*g;
    for (coefsLstItr=g->m_coefsList.begin (); coefsLstItr!=g->m_coefsList.end (); coefsLstItr++) {
      found= false;
      for (j=0; j<N;j++) {
        if (m_variableList[j]->Key() ==coefsLstItr->first) {
          found=true;
          break;
        }
      }
      assert(found);
      m_matrix.SetValue (i,j, (*coefsLstItr).second.GetCoef ());
    }
	}
}

bool MyNCApp::CheckVarList(){
  if (m_variableList.size()!=m_varList.size()){
    return false;
  }
  for (unsigned int i=0;i<m_variableList.size();i++){
    if (m_varList.find(m_variableList[i]->Key())==m_varList.end()){
      return false;
    }
    if (Simulator::Now ().GetMilliSeconds()-m_variableList[i]->GetGenTime()>VAR_LIFETIME*1000)
      return false;
  }
  // std::vector < Ptr<NetworkCodedDatagram> >::iterator it;
  // MapType::iterator it1;
  // NetworkCodedDatagram nc;
  // int i=0;
  // for(it=m_decodingBuf.begin();it!=m_decodingBuf.end();it++){
  //   nc=*(*it);
  //   int j=0;
  //   for(it1=(*it)->m_coefsList.begin();it1!=(*it)->m_coefsList.end();it1++){
  //     if (m_varList.find((it1->first))==m_varList.end())
  //       return false;
  //     j++;
  //   }
  //   i++;
  // }
  return true;
}

void MyNCApp::GausElim (int M, int N)
{
  NetworkCodedDatagram g,h;
  int k,i,j,n;
  int pivot;
  m_rank=0;
  // Main Loop : # of iteration = # of lines
  for(k = 0; k < M ; k++) {
    // if pivot is zero, we need to swap
    h=*(m_decodingBuf[k]);
    bool swp = false;
    while(!swp) {
      // first check if we can exchange with a column larger than k
      if(m_matrix.GetValue(k, k) == 0) {
        // if the pivot is zero we should exchange line or column order !
        for(n = k+1 ; n < N ; n++){
          if(m_matrix.GetValue(k, n) != 0) {
            // we have found a column for exchange. Let's swap.
            // Caution: when swapping column we have to take care of m_varList!
            PermuteCol(k, n, M);
            swp = true;
            break;
          }
        }
        if (!swp) {
          // We have a full zero line = an equation is linearly dependent !
          // we have to remove it !
          // we search for a line to exchange with it. Begin with the last line
          // and reduce matrix size
          if (k==(M-1)) {
            // we have reached the last line swapping is useless
            swp=true;
            // This line should be removed
            RemoveLine(M-1);
            M=M-1;
            break;
          } else {
            for (int L=k;L<M-1;L++)
              PermuteLine(L,L+1, N);
          }
          RemoveLine(M-1);
          M=M-1;
        }
      }
      pivot = m_matrix.GetValue(k, k);
      if ((pivot!=0)&&(k<M)) {
        // the pivot is not zero. Let's do the operation
        swp=true;
        m_rank++;
        if (pivot != 1) {
          // we have to rescale the line by the pivot
          m_decodingBuf[k]->Product(m_nodeGaloisField->div(1,pivot), m_nodeGaloisField);
          for(i= k; i < N ; i++) {
            m_matrix.SetValue(k, i , m_nodeGaloisField->div(m_matrix.GetValue(k, i), pivot));
          }
        }
        // make the value under the pivot equal zero
        for(i = k+1; i < M ; i++) {
          // Line index
          if (m_matrix.GetValue(i, k)!=0) {
            int p=m_matrix.GetValue(i, k);
            for(j = k; j < N ; j++) {
              //Column index
              m_matrix.SetValue(i, j, m_nodeGaloisField->sub(m_matrix.GetValue(i, j), m_nodeGaloisField->mul(p, m_matrix.GetValue(k, j))));
            }
            g = *m_decodingBuf[k];
            g.Product(p, m_nodeGaloisField);
            m_decodingBuf[i]->Minus(g, m_nodeGaloisField);
          }
        }
      }
    }
    h=*(m_decodingBuf[k]);
  }
}

void MyNCApp::RemoveLine(int row){
  m_decodingBuf.erase(m_decodingBuf.begin()+row);
  m_matrix.RemoveRow(row);
}

void MyNCApp::RemoveCol(int col){
//            assert(CheckVarList());
  std::string str=m_variableList[col]->Key();
  m_variableList.erase(m_variableList.begin()+col);
  m_varList.erase(str);
  m_matrix.RemoveCol(col);
//            assert(CheckVarList());
}


void
MyNCApp::PermuteCol(int col1, int col2, int L)
{
  // Method swapping column of the coefficient matrix taking care of var_list.
  int tmp;
  if (col1!=col2)
  {
    // swap column in var_list
    // L : # of rows
    Ptr<NCAttribute> ptr ;

    ptr = m_variableList[col1];
    m_variableList[col1] = m_variableList[col2];
    m_variableList[col2] = ptr;

    // swap column in coefficient matrix
    for(int l = 0; l < L; l++)
    {
      tmp = m_matrix.GetValue(l,col1);
      m_matrix.SetValue(l, col1, m_matrix.GetValue(l,col2));
      m_matrix.SetValue(l, col2, tmp);
    }
  }
}

void
MyNCApp::PermuteLine(int lin1, int lin2, int L)
{
  // Method swapping line of the coefficient matrix taking care of decodingBuf.
  int tmp;
  if (lin1 != lin2)
  {
    // swap line in coefficient matrix
    for(int l = 0; l < L; l++)
    {
      tmp = m_matrix.GetValue(lin1,l);
      m_matrix.SetValue(lin1, l, m_matrix.GetValue(lin2,l));
      m_matrix.SetValue(lin2, l, tmp);
    }
    // Swap element in decodingBuf
    Ptr<NetworkCodedDatagram> tmpPointer;
    tmpPointer = CreateObject<NetworkCodedDatagram> ();
    tmpPointer = m_decodingBuf[lin1];
    m_decodingBuf[lin1] = m_decodingBuf[lin2];
    m_decodingBuf[lin2] = tmpPointer;
  }
}

void MyNCApp::ExtractSolved (uint32_t M, uint32_t N)
{

  uint32_t i,j,k,l;
  uint8_t upPivot, alpha;
  MapType::iterator it,it2, it4;
  //std::vector<NCAttribute>::iterator it3;
  CoefElt coef;
  bool solved=true;
  NetworkCodedDatagram h;
  Ptr<NetworkCodedDatagram> g= CreateObject<NetworkCodedDatagram>();
  if (M < m_decodingBuf.size()){
    m_decodingBuf.erase(m_decodingBuf.begin()+M, m_decodingBuf.end());
  }
  //Check if one variable have been determined
  Ptr<NCAttribute> ptr;
  for (i=M;i>=1;i--) {
//                assert(CheckVarList());
    solved=true;
  for (j=i;j<N;j++) {
      if (m_matrix.GetValue(i-1, j)!=0){
        solved=false;
        break;
      }
    }
    if (solved){
      // a variable has been solved
      // First propagate this info in higher lines (equations)
      NetworkCodedDatagram nc;
      for (k=i-1;k>=1;k--){
        // make the value up the pivot equal zero
        upPivot = m_matrix.GetValue(k-1, i-1);
        if (upPivot!=0) {
          nc=*(m_decodingBuf[k-1]);
          alpha=m_nodeGaloisField->div(1,upPivot);
          m_decodingBuf[k-1]->Product(alpha, m_nodeGaloisField);
          m_decodingBuf[k-1]->Minus(*(m_decodingBuf[i-1]),m_nodeGaloisField);
          nc=*(m_decodingBuf[k-1]);
          m_decodingBuf[k-1]->Product(upPivot,m_nodeGaloisField);
          nc=*(m_decodingBuf[k-1]);
          for(l = k; l < N ; l++) {
            //Column index
            m_matrix.SetValue(k-1, l, m_nodeGaloisField->mul(upPivot, m_nodeGaloisField->sub(m_matrix.GetValue(i-1, l),
                                      m_nodeGaloisField->mul(alpha, m_matrix.GetValue(k-1,l)))));
          }
        }
      }
      g = m_decodingBuf[i-1]; //solved variable is in g
      it = g -> m_coefsList.begin();
      insertIntoDecodedBuf(g);
      //update matrix m_matrix
      //swap lines
      PermuteLine (M-1,i-1,N);
      //swap column
      PermuteCol (N-1,i-1,M);
      //remove the variable
      vector<Ptr<NCAttribute> >::iterator it2;
      std::string str;
      for (it2=m_variableList.begin();it2!=m_variableList.end();){
        str=(*it2)->Key();
    	  if ((*it2)->Key()==it->first){
    		  it2=m_variableList.erase(it2);
    		  break;
    	  } else
          it2++;
      }
      m_varList.erase(m_varList.find( it->first));
//      assert(CheckVarList());
      M--;
      N--;
      m_decodingBuf.erase (m_decodingBuf.begin () + M);
    }
  }
  assert(M<= N);
  if (M==0){
    m_varList.clear();
    m_variableList.clear();
  }
  assert(m_varList.size()!=1);
  assert(m_decodedBuf.size()==m_decodedList.size());
}

void MyNCApp::Decode (Ptr<NetworkCodedDatagram> g) {
  m_decodingBuf.insert(m_decodingBuf.begin(),g);
  UpdateVarList(*g);
  // M: number of equations
  int M = m_decodingBuf.size();
  // N: number of variables
  int N = m_varList.size();
  assert(M<=N);
  GenerateMatrix ();
  GausElim(M, N);
  M = m_decodingBuf.size();
  N = m_varList.size();
  ExtractSolved (M,N);
  M = m_decodingBuf.size();
  N = m_varList.size();
  assert(M<=N);
  assert(N<=MAX_VARLIST_SIZE);
  NS_LOG_UNCOND("t="<<Simulator::Now ().GetSeconds()<<" NID: "<<m_myNodeId<<" STATUS: "<<m_decodingBuf.size()<<" "<<m_varList.size());
}

void MyNCApp::insertIntoDecodedBuf(Ptr<NetworkCodedDatagram> g){
  assert(g->m_coefsList.size()==1);
  Time now = Simulator::Now ();
  MapType::iterator it = g -> m_coefsList.begin();
  assert(it->second.m_coef==1);
  // Transfer the decoded packet to decodedbuffer;
  if (m_myNodeId != it -> second.GetNodeId()) {//we are not the source of the packet !
    if (m_decodedList.find(it->first)!=m_decodedList.end()) { //The packet is already received
      return;
    }
    if (!m_changed) {
      NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" Node " <<m_myNodeId<<"changed=true");
      m_changed=true;
      Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
    }
    Ptr<DecodedPacketStorage> dnc=CreateObject<DecodedPacketStorage>();
    dnc->attribute.SetNodeId(it->second.GetNodeId());
    dnc->attribute.SetIndex(it->second.GetIndex());
    dnc->attribute.SetDestination(it->second.GetDestination());
    dnc->attribute.SetGenTime(it->second.GetGenTime());
    if (m_myNodeId == it -> second.GetDestination()) { // We have received a packet at destination !!!!!
      if (UpdateDeliveredList(it->first)){
        dnc->attribute.m_destReceived=true;
        packetDelay += (now.GetMilliSeconds () - it->second.GetGenTime());
        nReceivedPackets++;
        NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" key "<< it -> first<<
        " received in "<<m_myNodeId<<" with delivery delay : "<<
        (now.GetMilliSeconds () - it->second.GetGenTime()));
      } else {
        dnc->attribute.m_destReceived=false;
        NS_LOG_UNCOND("Duplicate Receive!!!");
      }
    }
    dnc->ncDatagram=g;
    if (m_decodedBuf.size() >= DECODED_BUFF_SIZE) {
      //BufferManagement (we remove the oldest packet from decoded lists)
      RemoveOldest();
    }
    m_decodedList[it->first]=dnc;
    m_decodedBuf.push_back(dnc);
    NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" key "<< it -> first<<" decoded in "<<m_myNodeId);
  }
}


void
//MyNCAppSource::GeneratePacket (Ptr<Node> node, const Ipv4InterfaceContainer &destInterfaces, ExponentialRandomVariable &expVar, UniformRandomVariable &uniVar)
MyNCApp::GeneratePacket ()
{
	std::vector<Ptr<NetworkCodedDatagram> >::iterator bufItr;
	MapType::iterator it2;
	uint8_t destId;
	if (m_buffer.size () < BUFFER_SIZE) {
    while (true){
		  destId = uniVar->GetInteger(0,nNode-1);
      if (destId != m_myNodeId){
        break;
      }
    }
		Time now = Simulator::Now ();
		CoefElt coef;
		Ptr<NetworkCodedDatagram> nc;
		nc = CreateObject<NetworkCodedDatagram>();
		coef.SetCoef (1);
		coef.SetIndex (m_nGeneratedPackets);
		coef.SetNodeId (m_myNodeId);
		coef.SetDestination (destId);
		coef.SetGenTime(0);
//			nc->m_coefsList.insert(MapType::value_type(coef.Key (),coef));
    nc->m_coefsList[coef.Key()]=coef;
		m_buffer.push_front (nc);
		m_nGeneratedPackets++;
		NS_LOG_UNCOND ("t = "<<now.GetSeconds ()<<" source "<<m_myNodeId<<" have generated "
    <<coef.Key ()<<" to destination "<<(int)destId);
	}
  while ((m_waitingList.size() < WAITING_LIST_SIZE)&& (!m_buffer.empty())) {
      PacketInjector ();
  }
	//  Simulator::Schedule (Seconds (expVar->GetValue ()), &MyNCAppSource::GeneratePacket, this, node, destInterfaces, expVar, uniVar);
	Simulator::Schedule (Seconds (expVar->GetValue()*15), &MyNCApp::GeneratePacket, this);
}



void MyNCApp::CheckWaitingList (uint8_t neighborId)
{
  //Implement the back Pressure algorithm
  Time now = Simulator::Now ();
  std::stringstream ss;
  ss << neighborId;
  std::string strNodeId= ss.str();
  std::map<uint8_t, Neighbor>::iterator neighborIterator=m_neighborhood.find(neighborId);
  assert(neighborIterator!=m_neighborhood.end());
  for (WaitingList::iterator it=m_waitingList.begin (); it!=m_waitingList.end ();) {
    if ((neighborIterator->second.neighborReceivedFilter!=NULL) &&
         (neighborIterator->second.neighborReceivedFilter->contains(it->pktId))) {
      it=m_waitingList.erase(it);
    } //else {
      //if ((std::find (it->nodeForwardedTo.begin(), it->nodeForwardedTo.end(),strNodeId)==it->nodeForwardedTo.end())) {
          //Packet not forwarded to this neighbor
      //  if ((neighborIterator->second.smallNeighborDecodedFilter->contains(it->pktId)) ||
      //    (neighborIterator->second.bigNeighborDecodedFilter->contains(it->pktId))) {
      //      it = m_waitingList.erase(it);
      //  }  else {
      //    it++;
      //  }
          //else if (neighborIterator->second.neighborDecodingFilter->contains(it->pktId)) {
            //int numVar=MAX_VARLIST_SIZE-(float)neighborIterator->second.neighborRemainingCapacity;
            //if (numVar>0){
            //  it->score+=(float)neighborIterator->second.neighborDecodingBufSize/NumVar;
            //  it->
            //}
            //else
            //  it->score=1;
          //else
          //  it++;
      //}
      else {
        it++;
//      }
    }
  }
  while (m_waitingList.size() < WAITING_LIST_SIZE) {
    if (!m_buffer.empty()) {
      PacketInjector ();
    } else
      break;
  }
}

void MyNCApp::UpdateWaitingList (std::string pktId)
{
	// implement back pressure
	//we assume that all neighbors have received a packet forwarded !!!!!
  Time now = Simulator::Now ();

  std::string strNodeId;
  std::map<uint8_t, Neighbor>::iterator neighborItr;
	for (WaitingList::iterator it=m_waitingList.begin(); it!=m_waitingList.end();it++) {
		if (it->pktId==pktId){
			for (neighborItr=m_neighborhood.begin(); neighborItr!=m_neighborhood.end(); neighborItr++) {
				std::stringstream ss;
				ss << (int)neighborItr->second.neighborId;
				strNodeId= ss.str();
        if (find(it->nodeForwardedTo.begin(),it->nodeForwardedTo.end(), strNodeId)==it->nodeForwardedTo.end())
				    it->nodeForwardedTo.push_back(strNodeId);
			}
		}
	}
}

bool MyNCApp::UpdateDeliveredList (std::string deliveredStr)
{
   std::vector<string>::iterator tempItr = find(m_deliveredList.begin(), m_deliveredList.end(), deliveredStr);
   if (tempItr == m_deliveredList.end()){
    if (m_deliveredList.size() == MAX_DELIVERED_LIST_SIZE){
      m_deliveredList.erase(m_deliveredList.begin());
      m_deliveredList.push_back (deliveredStr);
    } else {
      m_deliveredList.push_back (deliveredStr);
    }
    return true;
  }
  return false;
}

void MyNCApp::RemoveOldVariables () {
  Time now = Simulator::Now ();
  std::map<std::string, NCAttribute>::iterator it;
  std::set<std::string> toErase;
  for (it=m_varList.begin();it!=m_varList.end();it++) {
    std::string str=it->first;
		if (now.GetMilliSeconds() - it->second.GetGenTime() > VAR_LIFETIME*1000){
       toErase.insert(str);
    }
  }
  std::map<std::string, Ptr<DecodedPacketStorage> >::iterator itr;
  for (WaitingList::iterator it=m_waitingList.begin(); it!=m_waitingList.end();) {
    if (now.GetMilliSeconds()-it->entranceTime>VAR_LIFETIME*1000){
        it=m_waitingList.erase(it);
        m_oldwaitingList++;
    } else
      it++;
  }
  while (m_waitingList.size() < WAITING_LIST_SIZE) {
    if (!m_buffer.empty()) {
      PacketInjector ();
    } else
      break;
  }
  if (toErase.size()>0)
    RemoveVariable(toErase);
//  assert(CheckVarList());
}

void MyNCApp::RemoveDeliveredPackets (uint8_t neighborId)
{
  /*next step
  std::vector<Ptr<NetworkCodedDatagram> > m_decodingBuf;
  std::map<std::string, NCAttribute> m_varList;
  std::vector<Ptr<NCAttribute> > m_variableList;*/
//  assert(m_decodedBuf.size()==m_decodedList.size());
  std::vector<Ptr<DecodedPacketStorage> >::iterator it1;
  std::map<uint8_t,Neighbor>::iterator itr=m_neighborhood.find(neighborId);
  assert(itr!=m_neighborhood.end());
  Ptr<MyBloom_filter> eBf=itr->second.neighborReceivedFilter;
  //std::vector<std::string> toEraseKeys;
  //toEraseKeys.clear();
  //int num;
  std::string str;
  for (it1=m_decodedBuf.begin(); it1!= m_decodedBuf.end(); it1++){
    if (!(*it1)->attribute.m_destReceived){
      str=(*it1)->attribute.Key();
      if (eBf->contains (str)) {
        if (!(*it1)->attribute.m_destReceived) {
          UpdateDeliveredList(str);
          (*it1)->attribute.m_destReceived = true;
        }
      }
    }
  }
  assert(m_decodedBuf.size()==m_decodedList.size());
  std::set<std::string> toErase;
  //WHAT ABOUT in varLIST !!!!
  std::map<std::string, NCAttribute>::iterator it;
  for (it=m_varList.begin();it!=m_varList.end();it++) {
    std::string str=it->first;
    if (eBf->contains (str)){
      UpdateDeliveredList(str);
      toErase.insert(str);
    }
  }
  if (toErase.size()>0)
    RemoveVariable(toErase);
}

void MyNCApp::RemoveVariable(std::set<std::string> toErase){
  std::vector<Ptr<NCAttribute> >::iterator it1;
  unsigned int M, N, ind;
  std::string str2,str3,str4;
  Time now = Simulator::Now ();
  std::set<std::string>::iterator itr;
  M=m_decodingBuf.size();
  N=m_varList.size();
  unsigned int j=0, i=0;
  uint8_t pivot, alpha;
  std::string st;
  for (itr=toErase.begin();itr!=toErase.end();itr++){
//    assert(CheckVarList());
    bool found=false;

    for (it1=m_variableList.begin(); it1!=m_variableList.end();it1++){
      if ((*itr)==(*it1)->Key()){
        found=true;
        ind=it1-m_variableList.begin();
        break;
      }
    }
    assert(found);
    PermuteCol(j,ind,M);
    st=m_variableList[j]->Key();
    found=false;
    for (unsigned int k=i;k<M;k++){
      if (!found) {
        if (m_matrix.GetValue(k,j)!=0){
//          nc1=*(m_decodingBuf[i]);
          if (i!=k){
            PermuteLine(i,k,N);
          }
//          nc1=*(m_decodingBuf[i]);
          found=true;
          pivot=m_matrix.GetValue(i,j);
          if (pivot !=1) {
            alpha=m_nodeGaloisField->div(1,pivot);
            m_decodingBuf[i]->Product(alpha,m_nodeGaloisField);
            for (unsigned int l=j ; l < N ; l++) {
              m_matrix.SetValue(i, l , m_nodeGaloisField->mul(alpha, m_matrix.GetValue(i, l)));
            }
          }
        }
//        nc1=*(m_decodingBuf[i]);
      } else {
//        nc=*(m_decodingBuf[k]);
        if (m_matrix.GetValue(k,j)!=0){
          alpha=m_nodeGaloisField->div(1,m_matrix.GetValue(k,j));
          m_decodingBuf[k]->Product(alpha, m_nodeGaloisField);
//          nc=*(m_decodingBuf[k]);
          m_decodingBuf[k]->Minus(*(m_decodingBuf[i]),m_nodeGaloisField);
//          nc=*(m_decodingBuf[k]);
          for ( unsigned int l = j; l < N ; l++) {
            m_matrix.SetValue(k,l,m_nodeGaloisField->mul(alpha,m_matrix.GetValue(k, l)));
            m_matrix.SetValue(k,l,m_nodeGaloisField->sub(m_matrix.GetValue(k, l),m_matrix.GetValue(i, l)));
          }
        }
//        nc=*(m_decodingBuf[k]);
      }
    }
    if ((found) && (i<M))
      i++;
    j++;
  }
  m_decodingBuf.erase(m_decodingBuf.begin(),m_decodingBuf.begin()+i);
  std::string str;
  for (unsigned int l=0;l<j; l++){
    str=m_variableList[0]->Key();
    NS_LOG_UNCOND("t = "<<now.GetSeconds()<<" undecoded pktId : "<< str <<" "<<m_variableList[0]->GetGenTime()*1.0/1000<< " erased at "<<m_myNodeId);
    m_variableList.erase(m_variableList.begin());
    m_varList.erase(str);
  }
  GenerateMatrix();
  if (m_varList.size()==m_decodingBuf.size()) {
    GausElim(m_decodingBuf.size(),m_varList.size());
    ExtractSolved(m_decodingBuf.size(),m_varList.size());
  }
  if (m_decodingBuf.empty()){
    m_varList.clear();
    m_variableList.clear();

  }
  assert(m_decodingBuf.size()<= m_varList.size());
//  assert(m_variableList.size()==m_varList.size());
//  assert(CheckVarList());
}

void MyNCApp::RemoveOldest () {
  std::vector<Ptr<DecodedPacketStorage> >::iterator it,it1;
  uint32_t oldestReceivedTime=Simulator::Now().GetMilliSeconds();
  uint32_t oldestUnreceivedTime=oldestReceivedTime;
  int i=0;
  int maxReceived, maxUnreceived;
  bool foundReceived=false;
//  if (Simulator::Now().GetSeconds()>506.448)
//    NS_LOG_UNCOND("Problem");
  for (it= m_decodedBuf.begin(); it!=m_decodedBuf.end();it++){
    if ((*it)->attribute.m_destReceived) {
      foundReceived=true;
      if ((*it)->attribute.GetGenTime()< oldestReceivedTime) {
        oldestReceivedTime=(*it)->attribute.GetGenTime();
        maxReceived=i;
      }
    } else if (!foundReceived) {
      if ((*it)->attribute.GetGenTime()< oldestUnreceivedTime) {
        oldestUnreceivedTime=(*it)->attribute.GetGenTime();
        maxUnreceived=i;
      }
    }
    i++;
  }

  if (foundReceived){
    assert(maxReceived<m_decodedBuf.size());
    NS_LOG_UNCOND("Oldest Received Packet in : "<< maxReceived);
    std::string str=m_decodedBuf[maxReceived]->attribute.Key();
    m_decodedBuf.erase(m_decodedBuf.begin()+maxReceived);
    m_decodedList.erase(str);
  } else {
    assert(maxUnreceived<m_decodedBuf.size());
    NS_LOG_UNCOND("Oldest unReceived Packet in : "<< maxUnreceived);
    if (!m_decodedBuf[maxUnreceived]->attribute.m_destReceived){
      std::string str=m_decodedBuf[maxUnreceived]->attribute.Key();
      m_decodedBuf.erase(m_decodedBuf.begin()+maxUnreceived);
      m_decodedList.erase(str);
//      for (WaitingList::iterator it=m_waitingList.begin (); it!=m_waitingList.end ();it++) {
//        if (it->pktId==str) {
//          it=m_waitingList.erase(it);
//          break;
//        }
//      }
    }
  }
  oldestDiscardedNum++;
}

void MyNCApp::PacketInjector ()
{
  Time now=Simulator::Now();
  if (m_decodedBuf.size()==DECODED_BUFF_SIZE) {
    RemoveOldest();
  }
  if (!m_changed) {
    NS_LOG_UNCOND ("t = "<< Simulator::Now().GetSeconds ()<<" Node " <<m_myNodeId<<"changed=true");
    m_changed=true;
    Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
  }
  if (m_waitingList.size()<WAITING_LIST_SIZE){
    if (m_buffer.size()>0) {
      Ptr<NetworkCodedDatagram> p;
      p=m_buffer.back();
      Ptr<DecodedPacketStorage> q = CreateObject<DecodedPacketStorage>();
      q->ncDatagram=p;
      Ptr<NCAttribute> attribute= CreateObject<NCAttribute>();
      p->m_coefsList.begin()->second.SetGenTime(now.GetMilliSeconds ());
      q->attribute.m_nodeId=p->m_coefsList.begin()->second.GetNodeId();
      q->attribute.m_index=p->m_coefsList.begin()->second.GetIndex();
      q->attribute.m_destId=p->m_coefsList.begin()->second.GetDestination();
      q->attribute.m_genTime=p->m_coefsList.begin()->second.GetGenTime();
      m_decodedBuf.push_back(q);
      assert(m_decodedList.find(q->attribute.Key())==m_decodedList.end());
      m_decodedList[q->attribute.Key()]=q;
      m_buffer.pop_back();
      WaitingListMember waitElm;
      MapType::iterator it;
      it = p -> m_coefsList.begin();
      waitElm.pktId = (*it).first ;
      waitElm.entranceTime = Simulator::Now ().GetMilliSeconds();
      m_waitingList.push_back (waitElm);
      m_nInjectedPackets++;
      NS_LOG_UNCOND ("t = "<< Simulator::Now().GetSeconds ()<<" "<<m_myNodeId<<" injects "<<it->first<<" m_nInjectedPackets "<<m_nInjectedPackets);
    }
  }
}

Experiment::Experiment()
{}

Experiment::Experiment(std::string name):
 	s_output (name),
  s_simulationTime (SIMULATION_TIME),
  s_verbose (false),
  s_logComponent (false),
  s_nSource (NUMBER_OF_NODES),
  s_nRelay (0),
  s_packetSize (1000),
  s_beaconInterval (BEACON_PERIOD),
  s_packetInterval(FORWARD_PERIOD),
  s_nGeneratedBeacons (0),
  s_nReceivedBeacons (0),
  s_totalInjectedPackets (0),
  s_totalGeneratedPackets (0),
  s_nSrcForwardedPackets (0),
  s_nReceivedPackets (0),
  s_nForwardedPackets (0),
  s_nDuplicateRec (0),
  s_nDroppedPackets (0),
  s_oldestDiscardedNum (0),
  s_bytesTotal (0),
  s_packetDelay(0.0),
  s_oldwaitingList(0)
{
   m_output.SetStyle (Gnuplot2dDataset::LINES);
}
/*
void
Experiment::CheckThroughput ()
{
  for (uint8_t i=0; i < m_nDestination; i++)
    {
      m_bytesTotal += destination[i].nReceivedBytes;
      destination[i].nReceivedBytes = 0;
    }
  double kbs = ((m_bytesTotal * 8.0) /1000);
  m_bytesTotal = 0;
  m_output.Add ((Simulator::Now ()).GetSeconds (), kbs);
  //check throughput every 1 of a second
  Simulator::Schedule (Seconds (1.0), &Experiment::CheckThroughput, this);
}
*/
void
Experiment::CheckThroughput ()
{
  uint16_t temp=0;
  Ptr<MyNCApp> ptr;
  for (uint8_t i=0; i < s_nSource+s_nRelay; i++)
  {
  	temp +=(allApps.Get(i)->GetObject<MyNCApp>())->nReceivedPackets;
  }
    //Adding points for packet reception
  m_output.Add ((Simulator::Now ()).GetSeconds (), (double)temp);
  Simulator::Schedule (Seconds (1), &Experiment::CheckThroughput, this);
}

bool
Experiment::CommandSetup (int argc, char **argv)
{
  // for commandline input
  CommandLine cmd;
  cmd.AddValue ("m_nSource", "Number of Source nodes", s_nSource);
  cmd.AddValue ("m_nRelay", "Number of Relay nodes", s_nRelay);
  cmd.AddValue ("m_verbose", "Increasing number of packets to destinations", s_verbose);
  cmd.AddValue ("m_packetSize", "size of Packet", s_packetSize);
//  cmd.AddValue ("m_interval", "interval (seconds) between packets", m_interval);
  cmd.AddValue ("m_beaconInterval", "beacon interval (seconds) ", s_beaconInterval);
  cmd.AddValue ("m_simulationTime", "Simulation Time", s_simulationTime);
  cmd.AddValue ("m_logComponent", "turn on all WifiNetDevice log components", s_logComponent);

  cmd.Parse (argc, argv);
  return true;
}

Gnuplot2dDataset
Experiment::ApplicationSetup (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy, const NqosWifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel)
{
	//KAVE : Have changed to account for MyNCApp being now a Node
	// Create a container to manage the nodes of the DTN network.
	myNodes.Create(s_nSource+s_nRelay);

	///////////////////////////////////////////////////////////////////////////
	//                                                                       //
	// Construct the DTN Network                                             //
	//                                                                       //
	///////////////////////////////////////////////////////////////////////////

	YansWifiPhyHelper phy = wifiPhy;
	phy.SetChannel (wifiChannel.Create ());

	NqosWifiMacHelper mac = wifiMac;
	NetDeviceContainer nodesDevices;
	nodesDevices = wifi.Install (phy, mac, myNodes);

	InternetStackHelper internet;
	internet.Install (myNodes);
	// Assign IPv4 addresses to the device drivers (actually to the associated
	// IPv4 interfaces) we just created.
	Ipv4AddressHelper address;
	address.SetBase ("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer nodesInterfaces;
	nodesInterfaces = address.Assign (nodesDevices);

	// Turn on all Wifi logging
	if (s_logComponent) {
		wifi.EnableLogComponents ();
	}


	Ptr<MyNCApp> newApp;
	for (uint8_t i = 0; i < s_nSource ; i++) {
		newApp=CreateObject<MyNCApp> ();
		myNodes.Get(i)->AddApplication(newApp);
		newApp->SetNode(myNodes.Get(i));
		sourceApps.Add(newApp);
		newApp->m_pktSize=s_packetSize;
		newApp->nNode=s_nRelay+s_nSource;
		newApp->m_packetInterval=s_packetInterval;
		newApp->m_beaconInterval=s_beaconInterval;
    newApp->m_degraded=true;
		newApp->MakeSource();
		newApp->Start();
	}

	for (uint8_t i = 0; i < s_nRelay ; i++) {
		newApp=CreateObject<MyNCApp> ();
		myNodes.Get(i+s_nSource)->AddApplication(newApp);
		newApp->SetNode(myNodes.Get(i+s_nSource));
		rlyApps.Add(newApp);
		newApp->m_pktSize=s_packetSize;
		newApp->nNode=s_nRelay+s_nSource;
		newApp->m_packetInterval=s_packetInterval;
		newApp->m_beaconInterval=s_beaconInterval;
    newApp->m_degraded=true;
		newApp->Start();
	}

	allApps.Add(sourceApps);
	allApps.Add(rlyApps);

	// The DTN network nodes need a mobility model
	MobilityHelper mobility;
	ObjectFactory pos;
	pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
	pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
	pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
	Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
	mobility.SetPositionAllocator (taPositionAlloc);
	mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
		"Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=5.0]"),
		"Pause", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=0.0]"),
		"PositionAllocator", PointerValue (taPositionAlloc));
	mobility.Install (NodeContainer::GetGlobal ());

		///////////////////////////////////////////////////////////////////////////
		//                                                                       //
		// Application configuration                                             //
		//                                                                       //
		///////////////////////////////////////////////////////////////////////////



	CheckThroughput ();

	Simulator::Stop (Seconds (s_simulationTime));
	Simulator::Run ();

	uint32_t nSourcesGeneratedBeacons = 0;
	uint32_t nRelaysGeneratedBeacons = 0;
	uint32_t nSourcesReceivedBeacons = 0;
	uint32_t nRelaysReceivedBeacons = 0;
	uint32_t nSourcesForwardedPackets = 0;
	uint32_t nRelaysForwardedPackets = 0;
	uint32_t nSourcesDuplicateRec = 0;
	uint32_t nRelaysDuplicateRec = 0;
	uint32_t nSourcesDroppedPackets = 0;
	uint32_t nSourcesOldestDiscardedNum = 0;
	uint32_t nRelaysDroppedPackets = 0;
	uint32_t nRelaysOldestDiscardedNum = 0;
	uint32_t nReceivedLinearCombinations = 0;
	uint32_t bufferOccupation = 0;
	uint32_t sourceBuffOccupation = 0;
  uint32_t deliveredListSize=0;

	NS_LOG_UNCOND ("----------------------------------------------------------------------------------------------------------------");
	Ptr<MyNCApp> ptrsrcApp;
	for (uint8_t i = 0; i < s_nSource; i++)
	{
		ptrsrcApp=sourceApps.Get(i)->GetObject<MyNCApp>();
		nReceivedLinearCombinations+=ptrsrcApp->m_rcvCounter;
		nSourcesGeneratedBeacons += ptrsrcApp->nGeneratedBeacons;
		nSourcesReceivedBeacons += ptrsrcApp->nReceivedBeacons;
		s_totalGeneratedPackets += ptrsrcApp->m_nGeneratedPackets;
		s_totalInjectedPackets += ptrsrcApp->m_nInjectedPackets;
		s_nSrcForwardedPackets += ptrsrcApp->nSrcForwardedPackets;
		s_nReceivedPackets += ptrsrcApp-> nReceivedPackets;
		nSourcesForwardedPackets += ptrsrcApp->nForwardedPackets;
		nSourcesDuplicateRec += ptrsrcApp->nDuplicateRec;
		nSourcesDroppedPackets += ptrsrcApp->nDroppedPackets;
		nSourcesOldestDiscardedNum += ptrsrcApp->oldestDiscardedNum;
		bufferOccupation += (ptrsrcApp->m_decodedBuf).size()+(ptrsrcApp->m_decodingBuf).size();
		sourceBuffOccupation += (ptrsrcApp->m_buffer).size();
		s_packetDelay+= ptrsrcApp->packetDelay;
    deliveredListSize+=ptrsrcApp->m_deliveredList.size();
    s_oldwaitingList+=ptrsrcApp->m_oldwaitingList;
		NS_LOG_UNCOND ("srcBuff size of source "<<(int)i<<" is "<< (ptrsrcApp->m_buffer).size());
		NS_LOG_UNCOND ("m_decodedBuf size of source "<<(int)i<<" is "<< (ptrsrcApp->m_decodedBuf).size());
		NS_LOG_UNCOND ("m_decodingBuf size of source "<<(int)i<<" is "<< (ptrsrcApp->m_decodingBuf).size());
    NS_LOG_UNCOND ("m_varList size of source "<<(int)i<<" is "<< (ptrsrcApp->m_varList).size());
    NS_LOG_UNCOND("m_oldwaitingList of relay "<<(int)i<< " is "<<(int)(ptrsrcApp->m_oldwaitingList));
    std::map<std::string, NCAttribute>::iterator it;
    uint32_t oldestReceivedTime=Simulator::Now().GetMilliSeconds();
    std::string str;
    for (it=(ptrsrcApp->m_varList).begin();it!=(ptrsrcApp->m_varList).end();it++){
      if (it->second.GetGenTime()< oldestReceivedTime) {
        oldestReceivedTime=it->second.GetGenTime();
        str=it->first;
      }
    }
    NS_LOG_UNCOND("Oldest packet in m_varList is "<<str<<" generated at " <<oldestReceivedTime*1.0/1000.0);
    NS_LOG_UNCOND("m_deliveredList size of source " <<(int)i<<" is "<< (ptrsrcApp->m_deliveredList).size());
	}

	NS_LOG_UNCOND ("----------------------------------------------------------------------------------------------------------------");
	Ptr<MyNCApp> ptrRlyApp;
	for (uint8_t i=0; i < s_nRelay; i++)
	{
		ptrRlyApp=(Ptr<MyNCApp>)rlyApps.Get(i)->GetObject<MyNCApp>();
		nReceivedLinearCombinations+= ptrRlyApp->m_rcvCounter;
		nRelaysGeneratedBeacons += ptrRlyApp->nGeneratedBeacons;
		nRelaysReceivedBeacons += ptrRlyApp->nReceivedBeacons;
		nRelaysForwardedPackets += ptrRlyApp->nForwardedPackets;
		nRelaysDuplicateRec += ptrRlyApp->nDuplicateRec;
		nRelaysDroppedPackets += ptrRlyApp->nDroppedPackets;
		nRelaysOldestDiscardedNum += ptrRlyApp-> oldestDiscardedNum;
		bufferOccupation += (ptrRlyApp->m_decodedBuf).size()+(ptrRlyApp->m_decodingBuf).size();
		s_nReceivedPackets += ptrRlyApp-> nReceivedPackets;
		s_packetDelay+= ptrRlyApp->packetDelay;
    s_oldwaitingList+=ptrRlyApp->m_oldwaitingList;
    deliveredListSize+=ptrRlyApp->m_deliveredList.size();
		NS_LOG_UNCOND ("s_decodedBuf size of relay "<<(int)i<<" is "<< (ptrRlyApp->m_decodedBuf).size());
		NS_LOG_UNCOND ("s_decodingBuf size of relay "<<(int)i<<" is "<< (ptrRlyApp->m_decodingBuf).size());
    NS_LOG_UNCOND("m_deliveredList size of relay " <<(int)i<<" is "<< (ptrRlyApp->m_deliveredList).size());
    NS_LOG_UNCOND("m_oldwaitingList of relay "<<(int)i<< " is "<<(int)(ptrRlyApp->m_oldwaitingList));

	}

	NS_LOG_UNCOND ("----------------------------------------------------------------------------------------------------------------");
	s_nGeneratedBeacons = nSourcesGeneratedBeacons + nRelaysGeneratedBeacons;
	s_nReceivedBeacons = nSourcesReceivedBeacons + nRelaysReceivedBeacons;
	s_nForwardedPackets = nSourcesForwardedPackets + nRelaysForwardedPackets;
	s_nDuplicateRec = nSourcesDuplicateRec + nRelaysDuplicateRec;
	s_nDroppedPackets = nSourcesDroppedPackets + nRelaysDroppedPackets;
	s_oldestDiscardedNum = nSourcesOldestDiscardedNum + nRelaysOldestDiscardedNum;

	NS_LOG_UNCOND ("----------------------------------------------------------------------------------------------------------------");
	NS_LOG_UNCOND ("Total Number of generated datagram in sources is : "<<s_totalGeneratedPackets);
	NS_LOG_UNCOND ("Total Number of injected datagram from sources is : "<<s_totalInjectedPackets);
	NS_LOG_UNCOND ("Total Number of datagram received in destinations is : "<<s_nReceivedPackets);
  NS_LOG_UNCOND("Total Number of old in WaitingList in sources is "<<s_oldwaitingList);
  NS_LOG_UNCOND ("Delivery ratio is : "<<((double)s_nReceivedPackets/s_totalInjectedPackets)*100<<" %");
	NS_LOG_UNCOND ("Total Number of redundant datagram received in nodes is : "<<s_nDuplicateRec);
	NS_LOG_UNCOND ("Total Number of dropped datagram in nodes is : "<<s_nDroppedPackets);
	NS_LOG_UNCOND ("Total Number of Oldest Datagrams Discarded is : "<<s_oldestDiscardedNum);
	NS_LOG_UNCOND ("Total Number of received datagram in all nodes = "<<nReceivedLinearCombinations);
		    //NS_LOG_UNCOND ("Total Number of datagram forwarded from their source is : "<<m_nSrcForwardedPackets);
	NS_LOG_UNCOND ("Total Number of datagram forwarded from nodes is : "<<s_nForwardedPackets);
		    //NS_LOG_UNCOND ("Total Number of beacons received in nodes is : "<<m_nReceivedBeacons);
		    //NS_LOG_UNCOND ("Total Number of generated beacons is : "<<m_nGeneratedBeacons);
	NS_LOG_UNCOND ("Average Delay is : "<<s_packetDelay/s_nReceivedPackets/1000<<" seconds");
	NS_LOG_UNCOND ("Average node Buffer occuapancy is : "<<bufferOccupation/ (s_nSource + s_nRelay));
	NS_LOG_UNCOND ("Average Source's m_Buffer occupancy is : "<< sourceBuffOccupation / s_nSource);
  NS_LOG_UNCOND("Average receivedListSize is : " <<deliveredListSize/(s_nSource+s_nRelay));


	Simulator::Destroy ();

	return s_output;
}


int main (int argc, char* argv[]) {
	std::string phyMode ("DsssRate1Mbps");
	Experiment experiment;
	experiment = Experiment ("Low Density NetworkCoding with ⋋ = 0.1");
	Gnuplot gnuplot = Gnuplot ("LDNC01.png");

		  //for commandline input
	experiment.CommandSetup(argc, argv);

		  // Enable RTS/CTS
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
	Config::SetDefault ("ns3::Ipv4L3Protocol::DefaultTtl", UintegerValue (5));
	ns3::PacketMetadata::Enable ();

	gnuplot.SetTitle ("Throughput vs Time");
	gnuplot.SetLegend ("Time ", "Throughput [kbps]");
	std::ofstream outfile ("LDNC01");

		  // Create the adhoc wifi net devices and install them into the nodes in our container
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (RADIO_RANGE));
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	WifiHelper wifi;
	wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
		"DataMode",StringValue (phyMode),
		"ControlMode",StringValue (phyMode));
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
		  // Set it to adhoc mode
	wifiMac.SetType ("ns3::AdhocWifiMac");

	Gnuplot2dDataset dataset;
	dataset = experiment.ApplicationSetup (wifi, wifiPhy, wifiMac, wifiChannel);
	gnuplot.AddDataset (dataset);
	gnuplot.GenerateOutput (outfile);

	return 0;
}
