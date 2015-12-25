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
#include "MyNCApp.h"

//////////////////////////////////////////////


////////////////////////////////////////////////



////////////////////////////////////////////////

using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("NetworkCoding");

static const std::size_t BUFFER_SIZE = 30;
static const std::size_t WAITING_LIST_SIZE = 10;
static const std::size_t MAX_VARLIST_SIZE = 50;
static const std::size_t MaxNumberOfCoeff=5;
static const std::size_t DECODING_BUFF_SIZE = MAX_VARLIST_SIZE;
static const std::size_t DECODED_BUFF_SIZE = 200;
/*static const double BEACON_PERIOD = 1;
static const double FORWARD_PERIOD = 0.08;
static const double SIMULATION_TIME = 3600;
static const double NUMBER_OF_NODES = 40;
static const double RADIO_RANGE = 50;
static const double MEAN = 1;*/
static const std::size_t PEC = 100;
static const double DFPP = 0.02;
static const int variableWeight=1;
static const int unreceivedWeight=10;
// Aging constants parameters declaration:
static const float K0 = 25.0;
static const float K1 = 10.0;
static const float K2 = 0.25;
static const float MAX_TTL = 255;


/////////////////////////////////////////////////////////

Neighbor::Neighbor ()
{
  firstReceptionTime = 0;
  lastReceptionTime = 0;
  neighborReceivedPackets = 0;
  neighborReceivedBeacons = 0;
  neighborForwardedPackets = 0;
  neighborId = 0;
  neighborhoodSize = 0;
  neighborDecodingBufSize = 0;
  neighborRemainingCapacity = 0;
  sourceForwarded.clear();
  neighborNotSolvedVars.clear();
  neighborSolvedVars.clear();
  neighborDecodedListUtilities.clear();
  neighborVarListUtilities.clear();
}

Neighbor::~Neighbor ()
{}

MyHeader::MyHeader ()
{
  m_packetType = 0;
  m_time = 0;
  m_nodeId = 0;
  m_linCombSize = 0;
  m_neighborhoodSize = 0;
  m_decodingBufSize = 0;
  m_linComb. clear ();
  m_decodedTableSize=m_decodingTableSize = 0;
  m_decodedInsertedElementCount=m_decodingInsertedElementCount = 0;
  m_decodedBitTable=m_decodingBitTable = 0;
  m_remainingCapacity = 0;
}

MyHeader::~MyHeader ()
{}

TypeId
MyHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MyHeader")
  .SetParent<Header> ()
  .AddConstructor<MyHeader> ()
  ;
  return tid;
}

TypeId
MyHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void
MyHeader::Print (std::ostream &os) const
{
  os<<"Source="<< m_nodeId <<" Destination=" << m_destId
  <<" nodeId="<< (int)m_nodeId
  <<" m_linCombSize = "<<m_linCombSize;
	for (uint8_t k=0; k<m_linCombSize; k++)
	  {
          std::cout <<" m_linComb["<<k<<"].pktId = "<<m_linComb[k].pktId<<" m_linComb["<<k<<"].coeff = "<<(int)m_linComb[k].coeff;
	  }
}

uint32_t
MyHeader::GetSerializedSize (void) const
{
  return 15 + 8*m_linCombSize + (m_decodedTableSize + m_decodingTableSize) / BITS_PER_CHAR;
}

void
MyHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  //i.WriteHtonU32 (m_source.Get());
  i.WriteU8 (m_nodeId);
  i.WriteU8 (m_destId);
  i.WriteU8 (m_packetType);
 // i.WriteU32 (m_time);
  i.WriteU8 (m_neighborhoodSize);
  i.WriteU8 (m_remainingCapacity);
  i.WriteU8 (m_linCombSize);
  i.WriteU8 (m_decodingBufSize);
  i.WriteU16 (m_decodedTableSize);
  i.WriteU16 (m_decodedInsertedElementCount);
  i.WriteU16 (m_decodingTableSize);
  i.WriteU16 (m_decodingInsertedElementCount);
  for (std::size_t j=0; j < (m_decodedTableSize / BITS_PER_CHAR); j++)
    {
      i.WriteU8 (m_decodedBitTable[j]);
    }
  for (std::size_t j=0; j < (m_decodingTableSize / BITS_PER_CHAR); j++)
    {
      i.WriteU8 (m_decodingBitTable[j]);
    }
  std::string tmpStr;
  std::string colon = ":";
  size_t found;
  uint16_t nodeId, index;
	nodeId=index=0;
  for (uint8_t k1=0; k1!=m_linCombSize; k1++)
    {
      found = m_linComb[k1].pktId.find (colon);
      tmpStr = m_linComb[k1].pktId.substr (0, found);
      std::stringstream (tmpStr) >> nodeId;
      found++;
      tmpStr = m_linComb[k1].pktId.substr (found, m_linComb[k1].pktId.size () - found);
      std::stringstream (tmpStr) >> index;
      i.WriteU8 (nodeId);
      i.WriteU8 (index);
      i.WriteU8 (m_linComb[k1].coeff);
      i.WriteU8 (m_linComb[k1].dstId);
      i.WriteU32 (m_linComb[k1].genTime);
    }
}

uint32_t
MyHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  m_nodeId = i.ReadU8 ();
  m_destId =i.ReadU8 ();
  m_packetType = i.ReadU8 ();
 // m_time=i.ReadU32 ();
  m_neighborhoodSize = i.ReadU8 ();
  m_remainingCapacity = i.ReadU8 ();
  m_linCombSize = i.ReadU8 ();
  m_decodingBufSize = i.ReadU8 ();
  m_decodedTableSize = i.ReadU16 ();
  m_decodedInsertedElementCount = i.ReadU16 ();
  m_decodingTableSize = i.ReadU16 ();
  m_decodingInsertedElementCount = i.ReadU16 ();
  if (m_decodedBitTable)
    {
      delete[] m_decodedBitTable;
    }
  m_decodedBitTable = new unsigned char[m_decodedTableSize /BITS_PER_CHAR];
  for (std::size_t j=0; j < (m_decodedTableSize / BITS_PER_CHAR); j++)
    {
      m_decodedBitTable[j] = i.ReadU8 ();
    }
  if (m_decodingBitTable)
    {
      delete[] m_decodingBitTable;
    }
  m_decodingBitTable = new unsigned char[m_decodingTableSize /BITS_PER_CHAR];
  for (std::size_t j=0; j < (m_decodingTableSize / BITS_PER_CHAR); j++)
    {
      m_decodingBitTable[j] = i.ReadU8 ();
    }
  m_linComb.clear ();
  LinearCombination lc;
  uint16_t nodeId,index;
  nodeId=index=0;
  for (uint8_t k1=0; k1!=m_linCombSize; k1++)
    {
      nodeId = i.ReadU8 ();
      index = i.ReadU8 ();
      lc. pktId =  StringConcat (nodeId, index);
      lc. coeff = i.ReadU8 ();
      lc. dstId= i.ReadU8 ();
      lc.genTime= i.ReadU32 ();
      m_linComb. push_back (lc);
    }

// we return the number of bytes effectively read.
  return GetSerializedSize ();
}

void
MyHeader::SetDestination (uint8_t destination)
{
  m_destId = destination;
}

void
MyHeader::SetPacketType (uint8_t type)
{
  m_packetType = type;
}

void
MyHeader::SetNodeId (uint8_t id)
{
  m_nodeId = id;
}

void
MyHeader::SetTime (uint32_t time)
{
    m_time=time;
}

void
MyHeader::SetNeighborhoodSize (uint8_t size)
{
    m_neighborhoodSize = size;
}

void
MyHeader::SetNeighborDecodingBufSize (uint8_t size)
{
    m_decodingBufSize = size;
}

void
MyHeader::SetRemainingCapacity (uint8_t remainingCapacity)
{
    m_remainingCapacity = remainingCapacity;
}

void
MyHeader::SetLinearCombinationSize (uint8_t size)
{
	m_linCombSize = size;
}

uint8_t
MyHeader::GetDestination (void) const
{
  return m_destId;
}

uint8_t
MyHeader::GetPacketType (void) const
{
  return m_packetType;
}

uint8_t
MyHeader::GetNodeId (void) const
{
  return m_nodeId;
}

uint32_t
MyHeader::GetTime (void) const
{
    return m_time;
}

uint8_t
MyHeader::GetNeighborhoodSize (void) const
{
  return m_neighborhoodSize;
}

uint8_t
MyHeader::GetNeighborDecodingBufSize (void) const
{
  return m_decodingBufSize;
}

uint8_t
MyHeader::GetRemainingCapacity (void) const
{
    return m_remainingCapacity;
}

uint8_t
MyHeader::GetLinearCombinationSize (void) const
{
	return m_linCombSize;
}

void
MyHeader::PutDecodedBloomFilter (Ptr<MyBloom_filter> nodeFilterPointer)
{
    m_decodedTableSize = nodeFilterPointer->size();
    m_decodedBitTable = new unsigned char[m_decodedTableSize / BITS_PER_CHAR];
    std::copy (nodeFilterPointer->table(), nodeFilterPointer->table() + (m_decodedTableSize / BITS_PER_CHAR), m_decodedBitTable);
    m_decodedInsertedElementCount = nodeFilterPointer->element_count();
}

void
MyHeader::PutDecodingBloomFilter (Ptr<MyBloom_filter> nodeFilterPointer)
{
    m_decodingTableSize = nodeFilterPointer->size();
    m_decodingBitTable = new unsigned char[m_decodingTableSize / BITS_PER_CHAR];
    std::copy (nodeFilterPointer->table(), nodeFilterPointer->table() + (m_decodingTableSize / BITS_PER_CHAR), m_decodingBitTable);
    m_decodingInsertedElementCount = nodeFilterPointer->element_count();
}

Ptr<MyBloom_filter> MyHeader::GetDecodedBloomFilter (const std::size_t predictedElementCount, const double falsePositiveProbability) const
{
    Ptr<MyBloom_filter> filterPointer;
    filterPointer = CreateObject<MyBloom_filter> (predictedElementCount, falsePositiveProbability, m_nodeId);
    filterPointer->bit_table_ = new unsigned char[m_decodedTableSize / BITS_PER_CHAR];
    std::copy(m_decodedBitTable, m_decodedBitTable + (m_decodedTableSize / BITS_PER_CHAR), filterPointer->bit_table_);
    filterPointer->inserted_element_count_ = m_decodedInsertedElementCount;
    return filterPointer;
}

Ptr<MyBloom_filter>  MyHeader::GetDecodingBloomFilter (const std::size_t predictedElementCount, const double falsePositiveProbability) const
{
    Ptr<MyBloom_filter> filterPointer;
    filterPointer = CreateObject<MyBloom_filter> (predictedElementCount, falsePositiveProbability, m_nodeId);
    filterPointer->bit_table_ = new unsigned char[m_decodingTableSize / BITS_PER_CHAR];
    std::copy(m_decodingBitTable, m_decodingBitTable + (m_decodingTableSize / BITS_PER_CHAR), filterPointer->bit_table_);
    filterPointer->inserted_element_count_ = m_decodingInsertedElementCount;
    return filterPointer;
}



MyNCApp::MyNCApp(): Application()
{
  m_amSource=false;
  m_idle=true;
  m_port=5000;
  m_pktSize=1000;
  MCLU=10;
  predictedElementCount=PEC;
  falsePositiveProbability=DFPP;
  m_running=false;
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
	m_buffer. clear ();
	m_decodedBuf. clear ();
	m_decodingBuf. clear ();
	m_decodedList. clear ();
	m_varList. clear ();
	m_packetInterval=0.008;
	m_beaconInterval=1.0;
	double mean = 1;
	expVar= CreateObject<ExponentialRandomVariable> ();
	expVar->SetAttribute ("Mean", DoubleValue (mean));
	double l = 0;
	double s = 1;
	uniVar=CreateObject<UniformRandomVariable> ();
	uniVar->SetAttribute ("Min", DoubleValue (l));
	uniVar->SetAttribute ("Max", DoubleValue (s));

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
    m_myNCAppIp = GetNode()->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
	InetSocketAddress local = InetSocketAddress (m_myNCAppIp, m_port);
    sinkSock->Bind (local);
    sinkSock->SetRecvCallback (MakeCallback (&MyNCApp::Receive, this));
	InetSocketAddress remote = InetSocketAddress (Ipv4Address ("10.1.1.255"), m_port);
	sourceSock->SetAllowBroadcast (true);
	sourceSock->Connect (remote);
}

void
MyNCApp::SetupBeacon ()
{
  nGeneratedBeacons=0;
  nReceivedBeacons=0;
  nDuplicateRec=0;
  double expireTime = 0.5*expVar->GetValue();
  Simulator::Schedule (Seconds (expireTime), &MyNCApp::GenerateBeacon, this);
}

void
MyNCApp::GenerateBeacon ()
{
	std::list<Neighbor>::iterator it;
	Time now = Simulator::Now ();
	// First check if there is inactive neighbors
	for (it=m_neighborhood.begin(); it!=m_neighborhood.end();it++) {
		if (now.GetSeconds() - it->lastReceptionTime > NEIGHBOR_TIMER) {
			it = m_neighborhood.erase(it);
			NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" neighbor "<<(int)it->neighborId<<" has been deleted in "<<m_myNodeId<<"'s neighborhood !");
		}
		m_myNeighborhoodSize=m_neighborhood.size();
	}
	if (m_neighborhood.empty()){//no more neighbors
		m_idle=true;
	}
	NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" Beacon broadcast from : "<<m_myNodeId);;
	Ptr<Packet> beaconPacket = Create<Packet> ();
    MyHeader beaconHeader;
    beaconHeader.SetDestination (255);// means broadcast
    beaconHeader.SetPacketType (0);
    beaconHeader.SetNodeId (m_myNodeId);
	Ptr<MyBloom_filter> tempFilter1 =CreateObject<MyBloom_filter> (PEC, DFPP , m_myNodeId);
	Ptr<MyBloom_filter> tempFilter2 = CreateObject<MyBloom_filter> (PEC, DFPP , m_myNodeId);
	for (uint16_t i=0; i< m_varList.size(); i++)
	{
		tempFilter1->insert(m_varList.at(i));
	}
	for (uint16_t i=0; i< m_decodedList.size(); i++)
	{
		tempFilter2->insert(m_decodedList.at(i));
	}
	beaconHeader.PutDecodingBloomFilter (tempFilter1);
	beaconHeader.PutDecodedBloomFilter (tempFilter2);
	beaconHeader.SetNeighborDecodingBufSize ((uint8_t)m_decodingBuf.size());
	beaconHeader.SetRemainingCapacity((uint8_t) MAX_VARLIST_SIZE - m_varList.size());
	beaconHeader.SetNeighborhoodSize((uint16_t) m_neighborhood.size());
	beaconPacket-> AddHeader (beaconHeader);
//	delete tempFilter1;
//	delete tempFilter2;
	sourceSock-> Send (beaconPacket);
	nGeneratedBeacons++;
	Simulator::Schedule (Seconds (m_beaconInterval), &MyNCApp::GenerateBeacon, this);
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



void MyNCApp::UpdateNeighborList(MyHeader header, Ipv4Address senderIp) {
	bool newNeighbor = true;
	Time now = Simulator::Now ();
	std::list<Neighbor>::iterator listIterator;
	std::string tmpStr;
	if (!m_neighborhood.empty()) {
		for (listIterator=m_neighborhood.begin(); listIterator!=m_neighborhood.end(); listIterator++) {
			if(listIterator->neighborId == header.GetNodeId ()) {
				listIterator->lastReceptionTime = now.GetSeconds();
				listIterator->neighborRemainingCapacity = header.GetRemainingCapacity();
				listIterator->neighborDecodingFilter = header.GetDecodingBloomFilter (PEC, DFPP);
				listIterator->neighborDecodedFilter = header.GetDecodedBloomFilter (PEC, DFPP);
				listIterator->neighborhoodSize = header.GetNeighborhoodSize();
				listIterator->neighborDecodingBufSize = header.GetNeighborDecodingBufSize();
				(header.GetPacketType () == 1) ? listIterator->neighborReceivedPackets++ : listIterator->neighborReceivedBeacons++;
				newNeighbor = false;
				break;
			}
		}
		if (newNeighbor) {
			Neighbor neighbor;
			neighbor.firstReceptionTime = neighbor.lastReceptionTime = now.GetSeconds ();
			neighbor.neighborRemainingCapacity = header.GetRemainingCapacity();
			neighbor.neighborId = header.GetNodeId ();
			neighbor.neighborIp = senderIp;
			neighbor.neighborDecodingFilter = header.GetDecodingBloomFilter (PEC, DFPP);
			neighbor.neighborDecodedFilter = header.GetDecodedBloomFilter (PEC, DFPP);
			neighbor.neighborhoodSize = header.GetNeighborhoodSize ();
			neighbor.neighborDecodingBufSize = header.GetNeighborDecodingBufSize();
			NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" nodeId "<<(int)(neighbor.neighborId)<<" Became neighbor of nodeId "<<m_myNodeId);
			m_neighborhood.push_back (neighbor);
			(header.GetPacketType () == 1) ? neighbor.neighborReceivedPackets++ : neighbor.neighborReceivedBeacons++;
		}
	} else {
		Neighbor neighbor;
		neighbor.firstReceptionTime = neighbor.lastReceptionTime = now.GetSeconds ();
		neighbor.neighborRemainingCapacity = header.GetRemainingCapacity();
		neighbor.neighborId = header.GetNodeId ();
		neighbor.neighborIp = senderIp;
		neighbor.neighborDecodingFilter = header.GetDecodingBloomFilter (PEC, DFPP);
		neighbor.neighborDecodedFilter = header.GetDecodedBloomFilter (PEC, DFPP);
		neighbor.neighborhoodSize = header.GetNeighborhoodSize ();
		neighbor.neighborDecodingBufSize = header.GetNeighborDecodingBufSize();
		NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" nodeId "<<(int)(neighbor.neighborId)<<" Became neighbor of nodeId "<<m_myNodeId);
		m_neighborhood.push_back (neighbor);
		(header.GetPacketType () == 1) ? neighbor.neighborReceivedPackets++ : neighbor.neighborReceivedBeacons++;
		if (m_idle) {
			m_idle=false;//awake the node to send data
			Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
		}
	}
	m_myNeighborhoodSize=m_neighborhood.size();
	if (m_amSource){
		if (newNeighbor) {
			CheckWaitingList(--m_neighborhood.end());
		} else
			CheckWaitingList(listIterator);
	}
}

void MyNCApp::UpdateNeighorhoodEst(std::string pktId){
	std::list<Neighbor>::iterator listIterator;
	//we assume any forward packet is entering into all neighbors varList and therefore appears in neighborDecodingFilter
	for (listIterator=m_neighborhood.begin(); listIterator!=m_neighborhood.end(); listIterator++) {
		listIterator->neighborDecodingFilter->insert(pktId);
	}
}



void MyNCApp::Receive (Ptr<Socket> socket)
{
	Time now = Simulator::Now();
	Address from;
	Ptr<Packet> packetIn = socket -> RecvFrom (from);
	Ipv4Address senderIp = InetSocketAddress::ConvertFrom (from).GetIpv4 ();
	MyHeader header;
//	packetIn-> PeekHeader(header);
	packetIn-> RemoveHeader(header);
	UpdateNeighborList(header, senderIp);
	if (header.GetPacketType () == 0) {
		nReceivedBeacons++;
		NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" Received one beacon in a node "<<m_myNodeId<<" from : "<<header.GetNodeId());
	} else {
		NS_LOG_UNCOND("t = "<< now.GetSeconds ()<< " Received one datagram in a node "<<m_myNodeId<<" from : "<<header.GetNodeId());
		if (m_decodingBuf.size() < DECODING_BUFF_SIZE)
		{
			NetworkCodedDatagram* nc= new NetworkCodedDatagram();
			CoefElt coef;
			std::string tmpStr;
			std::string colon = ":";
			int value;
			for (size_t j=0 ;j < header.GetLinearCombinationSize (); j++) {
				coef. SetCoef (header. m_linComb[j].coeff);
				coef. SetDestination (header. m_linComb[j].dstId);
				coef.SetGenTime(header. m_linComb[j].genTime);
				std::string strPktId = header.m_linComb[j].pktId;
				size_t found = strPktId.find (colon);
				tmpStr = strPktId.substr (0, found);
				std::stringstream (tmpStr) >> value;
				coef. m_nodeId=value;
				found++;
				tmpStr = strPktId.substr (found, strPktId.size () - found);
				std::stringstream (tmpStr) >> coef. m_index;
				nc ->m_coefsList.insert(MapType::value_type(strPktId, coef));
			}
			//should change: each var should have its own genTime
			//nc-> m_genTime=header.GetTime();
			Decode (nc, packetIn);
		}
	}
}

void MyNCApp::Forward ()
{
	Time now = Simulator::Now();
	if (!m_idle) {
		std::string tmpStr;
		if (!m_decodedBuf.empty () || !m_decodingBuf.empty()) {
			NetworkCodedDatagram* tmpEncDatagram;
			tmpEncDatagram = new NetworkCodedDatagram();
			tmpEncDatagram = Encode();
			if (tmpEncDatagram!= NULL && !tmpEncDatagram->IsNull())
			{
				Ptr<Packet> lcPacket = Create<Packet> (m_pktSize);
				MyHeader lcHeader;
				lcHeader.SetNodeId (m_myNodeId);
				lcHeader.SetPacketType (1);
				//should change: each var should have its own genTime
				lcHeader.SetTime(tmpEncDatagram->m_genTime);//Write the datagram's generation time in header
				MapType::iterator it;
				std::stringstream ss;
				ss << m_myNodeId;
				std::string strNodeId = ss.str();
				std::string colon = ":";
				LinearCombination lc;
				for (it=tmpEncDatagram->m_coefsList.begin (); it!=tmpEncDatagram->m_coefsList.end (); it++)
				{
					lc.pktId = it->first;
					UpdateNeighorhoodEst(lc.pktId);
					if (it->second.GetNodeId()==m_myNodeId) {// we are the source of the packet so we have to update waitingList to do backpressure
						UpdateWaitingList(lc.pktId);
					}
					lc.coeff = (*it).second.GetCoef();
					lc.dstId = (*it).second.GetDestination();
					lcHeader.m_linComb.push_back (lc);
				}
				lcHeader.SetLinearCombinationSize (lcHeader.m_linComb.size ());
				Ptr<MyBloom_filter> tempFilter1 = CreateObject<MyBloom_filter> (predictedElementCount, falsePositiveProbability , m_myNodeId);
				Ptr<MyBloom_filter> tempFilter2 = CreateObject<MyBloom_filter> (predictedElementCount, falsePositiveProbability , m_myNodeId);
				std::string tmpStr;
				for (int i=(int)m_varList.size ()-1; i>=0; i--) {
					tmpStr = StringConcat(m_varList.at(i).m_nodeId, m_varList.at(i).m_index);
					tempFilter1->insert (tmpStr);
				}
				for (int i=(int)m_decodedList.size ()-1; i>=0; i--) {
					tmpStr = StringConcat(m_decodedList.at(i).m_nodeId, m_decodedList.at(i).m_index);
					tempFilter2->insert (tmpStr);
				}
				lcHeader.PutDecodingBloomFilter (tempFilter1);
				lcHeader.PutDecodedBloomFilter (tempFilter2);
				lcHeader.SetNeighborhoodSize ((uint16_t) m_neighborhood.size());
				lcHeader.SetNeighborDecodingBufSize ((uint8_t)m_decodingBuf.size());
				lcHeader.SetRemainingCapacity ((uint8_t) MAX_VARLIST_SIZE - m_varList.size());
				lcPacket->AddHeader (lcHeader);
				lcPacket->RemoveAllPacketTags ();
				lcPacket->RemoveAllByteTags ();
				sourceSock->Send (lcPacket);
				NS_LOG_UNCOND("t = "<< now.GetSeconds ()<<" "<<"node "<<m_myNodeId<<" is broadcasting.");
			}
		}
		Simulator::Schedule (Seconds (m_packetInterval), &MyNCApp::Forward, this);
	}
}

NetworkCodedDatagram*
 MyNCApp::Encode ()
{
	int L,l;
	int coef;//choice, order;
	NetworkCodedDatagram g;
	NetworkCodedDatagram *nc;

	nc = new NetworkCodedDatagram();
	L = m_decodedBuf.size();
	l = m_decodedBuf.size() + m_varList.size();
	int len = m_decodingBuf.size() + m_decodedBuf.size();
	std::list<Neighbor>::iterator listIterator;
	// For encoding we should have received packets
	if (l==0)
	{
		return NULL;
	}
	if (len ==0)
	{
		NS_LOG_UNCOND ("ERROR : LEN ==0 while L!=0");
	}

	int numVar=0;
	int index=0;
	std::vector<double> var,F, X, B;
	m_lpMatrix.A.clear();
	m_lpMatrix.SetDimentions((int) m_neighborhood.size(), l);
	B.clear();
	B.resize(m_neighborhood.size());
	F.resize(len);
	X.resize(len);

	for (listIterator=m_neighborhood.begin(); listIterator!=m_neighborhood.end(); listIterator++)
	{
		B.at(index)=listIterator->neighborRemainingCapacity;
		//let's first iterate over the decodedList
		for (int i=0; i<L; i++)
		{
			F.at(i)=0;
			std::string tmpStr = StringConcat(m_decodedList.at(i).m_nodeId, m_decodedList.at(i).m_index);
			if (!listIterator->neighborDecodedFilter->contains(tmpStr))
			{
				if (listIterator->neighborDecodingFilter->contains(tmpStr))
				{
					F.at(i)=F.at(i) + variableWeight;
						//X.at(i)=1;
						//add the constraints here
						//constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
						//X.at(i)=0;
				} else {
					F.at(i)=F.at(i) + unreceivedWeight;
						//X.at(i)=1;
						//add the constraints here
						//constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
						//X.at(i)=0;
					m_lpMatrix.SetValue(index,i,1);
				}
			}
		}//for loop over decodedList
			//now let's iterate over the decodingList
		for (uint16_t i=0; i< len-L; i++)
		{
			numVar= (int) m_decodingBuf.at(i)->m_coefsList.size();
			MapType::iterator it;
			std::vector<PacketId>::iterator itr;//,itr2;
			for (it=m_decodingBuf.at(i)->m_coefsList.begin (); it!=m_decodingBuf.at(i)->m_coefsList.end (); it++)
			{
				itr = std::find (m_varList.begin(), m_varList.end(), (*it).second.GetPktId ());
				//itr2 = std::find(m_decodedList.begin(),m_decodedList.end(),(*it).second.GetPktId ());
				if (itr!=m_varList.end())//If you find the variable in the variable list
				{
//					uint16_t I = (uint16_t)(itr - m_varList.begin());//find the index of this variable in the varList!!!
					std::string tmpStr = StringConcat(itr->m_nodeId, itr->m_index);
					if (!listIterator->neighborDecodedFilter->contains(tmpStr))
					{
						if (listIterator->neighborDecodingFilter->contains(tmpStr))
						{
							F.at(L+i)=F.at(L+i) + (double)variableWeight/numVar;
							//X.at(I+L)=1;
							//add the constraints here
							//constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
							//X.at(I+L)=0;
						}
						else
						{
							//X.at(I+L)=1;
							//add the constraints here
							//constraints.add(new LinearConstraint(X,Relationship.LEQ,1));
							//X.at(I+L)=0;
							F.at(L+i)=F.at(L+i) + (double)unreceivedWeight/numVar;
							m_lpMatrix.SetValue(index,i+L,1);
						}
					}
				} else {
					NS_LOG_UNCOND("Wrong not found in varlist!!!!");
				}
			}
		}// for loop over decodingList

			//okay! now add another constraint
			//constraints.add(new LinearConstraint(AA[index],Relationship.LEQ,B[index]));
		index++;
	}//end of iteration over the neighborhood

	if(index>0)
	{
		//here we define our problem
		glp_prob *myLpProblem;
		myLpProblem = glp_create_prob();
		glp_set_prob_name(myLpProblem, "myLpProblem");
		glp_set_obj_dir (myLpProblem, GLP_MAX);//maximize
		glp_add_rows (myLpProblem, m_neighborhood.size()+1);
		glp_add_cols (myLpProblem, l);
		int nConstraintMatrixElements =0;
			//let us define the bounds of the cols here (0<=p_i<=1)
		for (int i=0; i<len;i++)
		{
			glp_set_col_bnds (myLpProblem, i+1, GLP_DB, 0.0, 0.9999);
			glp_set_obj_coef(myLpProblem, i+1, F.at(i));
		}
			//let us now set the rows bounds (0<= summation of probabilities <= capacity of corresponding neighbor)
			//before to loading the constraint matrix we should prepare the required args (ia[], ja[], and ar[])
		int ia[1+1000], ja[1+1000];
		double ar[1+1000];
		int M= (int)m_neighborhood.size();
		int i=0;
		for (listIterator=m_neighborhood.begin();listIterator!=m_neighborhood.end();listIterator++)
		{
			for (int j=0; j<l;j++)
			{
				ia[i*l+j+1]=i+1;
				ja[i*l+j+1]=j+1;
				ar[i*l+j+1]=m_lpMatrix.GetValue(i,j);
				nConstraintMatrixElements++;
			}
			if (listIterator->neighborRemainingCapacity>0) {
				glp_set_row_bnds(myLpProblem, i+1, GLP_DB, 0.0, (double)(listIterator->neighborRemainingCapacity)/(listIterator->neighborhoodSize));
			} else {
				glp_set_row_bnds(myLpProblem, i+1, GLP_FX, 0.0, 0.0);
			}
			i++;
		}
		for (int j=0; j<l;j++)
		{
			ia[M*l+j+1]=M+1;
			ja[i*l+j+1]=j+1;
			ar[i*l+j+1]=1;
			nConstraintMatrixElements++;
		}
		glp_set_row_bnds(myLpProblem, M+1, GLP_DB, 0.0,MaxNumberOfCoeff);
			//let us load the constraint matrix below
		glp_load_matrix(myLpProblem,nConstraintMatrixElements, ia, ja, ar);
		glp_simplex(myLpProblem, NULL);
		double objective= glp_get_obj_val(myLpProblem);
		if (objective>0.0) {
			std::vector<double> probabilities(l);
			std::vector<double>::iterator it;
			for(int i=0; i<l;i++)
			{
				probabilities.at(i)=glp_get_col_prim(myLpProblem, i+1);
			}
			bool first=true;
			int inserted;
			Time now = Simulator::Now ();

			for (int i=0;i<L;i++)
			{
				if (uniVar->GetValue(0.0,1.0) < probabilities.at(i))
				{
					coef = uniVar->GetInteger (1,255);
					//NS_LOG_UNCOND ("t = "<<Simulator::Now ().GetSeconds()<<" In Encode of nodeId="<<m_myNodeId<<"		"<<"L="<<L<<",len = "<<len<<",l = "<<l<<"	i = "<<(int)i<<" choice="<<choice<<" and random coef for this choice is " <<coef);
					g=(*m_decodedBuf.at(i));
					g.Product(coef, m_nodeGaloisField );
					inserted++;
					if (first)
					{
						nc->operator=(g);
						first=false;
					}
					else
					{
						nc->Sum (g, m_nodeGaloisField);
					}
					//We have to update waitingList and neighborhood
				}
			}//for over decodedBuf

			for (uint16_t i=0;i<m_decodingBuf.size();i++)
			{
				if (uniVar->GetValue (0.0,1.0) < probabilities.at(i))
				{
					coef = uniVar->GetInteger (1,255);
					g=(*m_decodingBuf.at(i));
					g.Product(coef, m_nodeGaloisField );
					inserted=inserted+g.m_coefsList.size();
					if (first)
					{
						nc->operator=(g);
						first=false;
					}
					else
					{
						nc->Sum (g, m_nodeGaloisField);
					}
				}
			}//for over decodingBuf

			glp_delete_prob(myLpProblem);//we have to delete the problem at the end of the encode function
			return nc;
		} else { //Blocking situation
			NS_LOG_UNCOND("Blocked situation for "<<m_myNodeId);
		}

	}//if(index>0)
	return NULL;
}

void
MyNCApp::Reduce (NetworkCodedDatagram& g)
{
	MapType::iterator it;
    std::vector<PacketId>::iterator itr;
	std::vector<NetworkCodedDatagram*>::iterator bufItr;
	if (!m_decodedList.empty ())
	  {
        while (it!=g.m_coefsList.end ())
          {
            for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end ();it++)
              {
                itr = std::find (m_decodedList.begin(), m_decodedList.end(), (*it).second.GetPktId ());
                if (itr!=m_decodedList.end())
                  {
                    g.m_coefsList.erase (it);
                    break;
                 }
              }
          }
	  }
}

int
MyNCApp::CheckCapacity(NetworkCodedDatagram& g)
{
	MapType::iterator it;
	std::vector<PacketId>::iterator itr,itr2;
	int newVar=0;
	for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end (); it++) {
		itr = std::find (m_varList.begin(), m_varList.end(), (*it).second.GetPktId ());
		itr2 = std::find(m_decodedList.begin(),m_decodedList.end(),(*it).second.GetPktId ());
		if (itr==m_varList.end() && itr2==m_decodedList.end()) {
			newVar++;
		}
	}
	if (m_varList.size()+newVar<MAX_VARLIST_SIZE ) {
		return 1;
	}
	return 0;
}

void
MyNCApp::UpdateVarList (NetworkCodedDatagram& g)
{
  MapType::iterator it;
  std::vector<PacketId>::iterator itr,itr2;
  for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end (); it++) {
      itr = std::find (m_varList.begin(), m_varList.end(), (*it).second.GetPktId ());
      itr2 = std::find(m_decodedList.begin(),m_decodedList.end(),(*it).second.GetPktId ());
      if (itr==m_varList.end() && itr2==m_decodedList.end()) {
          m_varList.push_back ((*it).second.GetPktId ());
      }
  }
}

void
MyNCApp::GenerateMatrix ()
{
	m_matrix. A. clear ();
    std::vector<NetworkCodedDatagram*>::iterator bufItr;
	MapType::iterator coefsLstItr, it;
    std::vector<PacketId>::iterator varLstItr;
	NetworkCodedDatagram* g;
	g = new NetworkCodedDatagram();

	// Number of variables
	int N = m_varList.size();
  // Number of equations
	int M = m_decodingBuf.size();
	int pos=0;
  NS_LOG_UNCOND("IN GENERATEMATRIX	"<<"nodeId" <<m_myNodeId<<" M = m_decodingBuf.size() is : "<< m_decodingBuf.size()<< " and N = m_varList.size() is : "<<m_varList.size());
  if (M>N)
	  {
      NS_LOG_UNCOND ("# of equations > # of variables !");
	  }
	m_matrix.SetDimentions (M, N);
	int i=0;
	for (bufItr=m_decodingBuf.begin (); bufItr!=m_decodingBuf.end (); bufItr++)
	  {
      g=*bufItr;
		  /*for (it=g-> m_coefsList. begin(); it!=g-> m_coefsList.end(); it++)
		    {
          NS_LOG_UNCOND ("in GM for "<<" nodeId= "<<m_myNodeId<<"		"<<it->first<<" coef="<<(int)it->second. GetCoef()<<" destIp= "<<it->second. GetDestination());
		    }*/
      for (coefsLstItr=g->m_coefsList.begin (); coefsLstItr!=g->m_coefsList.end (); coefsLstItr++)
        {
          varLstItr = std::find (m_varList.begin(), m_varList.end(), (*coefsLstItr).second.GetPktId ());
          pos = varLstItr - m_varList.begin ();
          if (varLstItr==m_varList.end())
            {
              NS_LOG_UNCOND ("ERROR in GenerateMatrix");
            }
          m_matrix.SetValue (i,pos, (*coefsLstItr).second.GetCoef ());
        }
		  i++;
	  }
    //m_matrix. PrintMatrix (M,N, m_myNodeId);
}

int
MyNCApp::GausElim (int M, int N, Ptr<Packet> packetIn)
{
  NetworkCodedDatagram g;
  int k,i,j,n;
  int pivot;
  m_rank=0;
  // Main Loop : # of iteration = # of lines
  for(k = 0; k < M ; k++)
    {
      // if pivot is zero, we need to swap
      bool swp = false;
      // first check if we can exchange with a column larger than k
      while(!swp)
        {
          if(m_matrix.GetValue(k, k) == 0)
            {
              // if the pivot is zero we should exchange line or column order !
              for(n = k+1 ; n < N ; n++)
                {
                  if(m_matrix.GetValue(k, n) != 0)
                    {
                      // we have found a column for exchange. Let's swap.
                      // Caution: when swapping column we have to take care of m_varList!
                      PermuteCol(k, n, M);
                      swp = true;
                      break;
                    }
                }
              if (!swp)
                {
                  // We have a full zero line = an equation is linearly dependent !
                  // we have to remove it !
                  nDuplicateRec++;
                  // we search for a line to exchange with it. Begin with the last line
                  // and reduce matrix size
                  if (k==(M-1))
                    {
                      // we have reached the last line swapping is useless
                      swp=true;
                      // This line should be removed
                      M=M-1;
                      break;
                    }
                  else
                    {
                      M=M-1;
                      PermuteLine(M,k, N);
                    }
                }
            }
          if (k<M)
            {
              // the pivot is not zero. Let's do the operation
              swp=true;
              m_rank++;
              pivot = m_matrix.GetValue(k, k);
              if (pivot != 1)
                {
                  // we have to rescale the line by the pivot
                  for(i= k; i < N ; i++)
                    {
                      m_matrix.SetValue(k, i , m_nodeGaloisField->div(m_matrix.GetValue(k, i), pivot));
                    }
                  //NS_LOG_UNCOND("After Changing PIVOT to 1, printed MATRIX of nodeId="<<m_myNodeId<<" is :");
                  //m_matrix.PrintMatrix(M,N, m_myNodeId);
                  m_decodingBuf[k]->Product(m_nodeGaloisField->div(1,pivot), m_nodeGaloisField);
                }
              // make the value under the pivot equal zero
              for(i = k+1; i < M ; i++)
                {
                  // Line index
                  if (m_matrix.GetValue(i, k)!=0)
                    {
                      int p=m_matrix.GetValue(i, k);
                      for(j = k; j < N ; j++)
                        {
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
    }
  ExtractSolved (M,N,packetIn);
  return M;
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
      PacketId id;
      id = m_varList[col1];
      m_varList[col1] = m_varList[col2];
      m_varList[col2] = id;
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
      NetworkCodedDatagram* tmpPointer;
      tmpPointer = new NetworkCodedDatagram ();
      tmpPointer = m_decodingBuf[lin1];
      m_decodingBuf[lin1] = m_decodingBuf[lin2];
      m_decodingBuf[lin2] = tmpPointer;
    }
}

void
MyNCApp::ExtractSolved (uint32_t M, uint32_t N, Ptr<Packet> packetIn)
{
  uint32_t i,j,k,l,upPivot;
  MapType::iterator it,it2, it4;
  std::vector<PacketId>::iterator it3;
  std::vector<NetworkCodedDatagram*>::iterator bufItr;
  CoefElt coef;
  PacketId id;
  bool solved=true;
  NetworkCodedDatagram* g;
  g= new NetworkCodedDatagram();
  if (M < m_decodingBuf.size())
    {
      for (i=M;i<m_decodingBuf.size();i++)
        {
          m_decodingBuf.erase(m_decodingBuf.begin()+i);
        }
    }
  //Check if one variable have been determined
  for (i=M;i>=1;i--)
    {
      solved=true;
      // Prepare the NCdatagram
      m_decodingBuf[i-1]->m_coefsList.clear();
      coef.SetCoef(m_matrix.GetValue(i-1, i-1));
      coef.SetIndex(m_varList[i-1]. GetIndex());
      coef.SetNodeId(m_varList[i-1]. GetNodeId ());
      coef.SetDestination (m_varList[i-1]. GetDestination());
      coef.SetGenTime(m_varList[i-1].GetGenTime());
      m_decodingBuf[i-1]->m_coefsList.insert(MapType::value_type(coef.Key (),coef));
      for (j=i;j<N;j++)
        {
          if (m_matrix.GetValue(i-1, j)!=0)
            {
              solved=false;
              coef.SetCoef(m_matrix.GetValue(i-1, j));
              coef.SetIndex(m_varList[j].GetIndex());
              coef.SetNodeId(m_varList[j].GetNodeId ());
              coef.SetDestination (m_varList[j]. GetDestination());
              m_decodingBuf[i-1]->m_coefsList.insert(MapType::value_type(coef.Key (),coef));
            }
        }
      if (solved)
        {
          // a variable has been solved
          // First propagate this info in higher lines (equations)
          for (k=i-1;k>=1;k--)
            {
              // make the value up the pivot equal zero
              //NS_LOG_UNCOND("k is : "<<k<<" i is :"<<i<<"M="<<M<<"  "<<"N="<<N);
              upPivot = m_matrix.GetValue(k-1, i-1);
              if (upPivot!=0)
                {
                  for(l = k; l < N ; l++)
                    {
                      //Column index
                      m_matrix.SetValue(k-1, l, m_nodeGaloisField->sub(m_matrix.GetValue(k-1, l), m_nodeGaloisField->mul(upPivot, m_matrix.GetValue(i-1,l))));
                    }
                }
            }
          //NS_LOG_UNCOND ("After ExtractSolved");
          //m_matrix.PrintMatrix(M,N,m_myNodeId);

          // Transfer the decoded packet to decodedbuffer;
          if (m_decodedBuf.size() < DECODED_BUFF_SIZE)
            {
              g = m_decodingBuf[i-1];
              Time now = Simulator::Now ();
              it = g -> m_coefsList.begin();
              if (m_myNCAppIp != it -> second.GetSource())
                {
                  for (bufItr= m_decodedBuf.begin() ; bufItr!=m_decodedBuf.end() ; bufItr++)
                    {
                      it4 = (*bufItr)-> m_coefsList.begin();
                      if (it4->first == it -> first )
                        {
                          break;
                        }
                    }
                  if (bufItr == m_decodedBuf.end())
                    {
                      if (m_myNodeId == it -> second.GetDestination()) // We have received a packet at destination !!!!!
                        {
                          //should change...
                          packetDelay += (now.GetNanoSeconds () - g->m_genTime);
                          nReceivedPackets++;
                          MyHeader removeHeader;
                          packetIn->RemoveHeader(removeHeader);
                          nReceivedBytes += packetIn->GetSize ();
                          NS_LOG_UNCOND("packetIn->GetSize () = "<<packetIn->GetSize ()<<" nReceivedBytes = "<<nReceivedBytes<<"  nReceivedPacket = "<<nReceivedPackets<<" m_myNodeId = "<<m_myNodeId);
                          NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" "<<" the key "<<it -> first<<" have received in "<<m_myNodeId<<" destination !");
                          //should change and merge with above line...
                          NS_LOG_UNCOND ("and delivery delay for this packet is : "<<(now.GetNanoSeconds () - g->m_genTime));
                        }
                      m_decodedBuf.push_back(g);
                      if (g->m_coefsList.size() !=1)
                        {
                          NS_LOG_UNCOND("Error in decoded Packet !");
                        }
                      m_decodedList.push_back((*it).second.GetPktId());
                    }
                }
            }
          else
            {
            	//BufferManagement (we remove the oldest packet from decoded lists)
              int oldestIndex=0;
              nDroppedPackets++;
            }
          //update matrix m_matrix
          //swap lines
          PermuteLine (M-1,i-1,N);
          //swap column
          PermuteCol (N-1,i-1,M);
          //remove the variable
          M--;
          N--;
          m_varList.erase (m_varList.begin () + N);
          m_decodingBuf.erase (m_decodingBuf.begin () + M);
        }
    }
  if (M > N)
    {
      NS_LOG_UNCOND ("Fatal Error !");
    }
  NS_LOG_UNCOND("t = "<<Simulator::Now ().GetSeconds()<<"	nodeId = "<<m_myNodeId<<" End Of ES");
}

void
MyNCApp::Decode (NetworkCodedDatagram* g, Ptr<Packet> packetIn)
{
	std::vector<NetworkCodedDatagram*>::iterator bufItr;
	MapType::iterator ii2;
	// received packet validity check
  //int tmpNumCoef=g->m_coefsList.size();
  m_rcvCounter++;
  NS_LOG_UNCOND ("t = "<<Simulator::Now ().GetSeconds()<<"	nodeId = "<<m_myNodeId<<" m_rcvCounter "<<m_rcvCounter);
  Reduce(*g);
  if(g->m_coefsList.size() == 0)
    {
      NS_LOG_UNCOND("t = " << Simulator::Now().GetSeconds() << " NODE ID: "<< m_myNodeId<<" ACT: Useless packet!!!");
      	return;
    }
    if (CheckCapacity(*g))
    {
        m_decodingBuf.push_back(g);
        UpdateVarList(*g);
    }
  else
    {
		nDroppedPackets++;
		NS_LOG_UNCOND("Dropped packet at NODE ID:" <<m_myNodeId);
      	return;
    }
  // M: number of equations
  int M = m_decodingBuf.size();
  // N: number of variables
  int N = m_varList.size();
  if (M>N)
    {
      NS_LOG_UNCOND("Fatal error !");
    }
  GenerateMatrix ();
  // L: column length (Number of variables+size of payload)
  //   L = N + MDU;
  NS_LOG_UNCOND("t = "<<Simulator::Now ().GetSeconds()<<" NID: "<<m_myNodeId<<" TIME: "<<" STATUS: B "<<M<<" "<<N);
  M = GausElim(M, N,packetIn);
  if (M>N)
    {
      NS_LOG_UNCOND ("Problem: # of equations > # of variables");
    }
  NS_LOG_UNCOND(Simulator::Now ().GetSeconds()<<" NID: "<<m_myNodeId<<" TIME: "<<" STATUS: A "<<m_decodingBuf.size()<<" "<<m_varList.size());
}



void
//MyNCAppSource::GeneratePacket (Ptr<Node> node, const Ipv4InterfaceContainer &destInterfaces, ExponentialRandomVariable &expVar, UniformRandomVariable &uniVar)
MyNCApp::GeneratePacket ()
{
	std::vector<NetworkCodedDatagram*>::iterator bufItr;
	MapType::iterator it2;
	uint8_t destId;
	if (m_buffer.size () < BUFFER_SIZE)
	{
      while (true)
        {
		  destId = uniVar->GetInteger(0,nNode-1);
          if (destId != m_myNodeId)
            {
              break;
            }
        }
		Time now = Simulator::Now ();
		if (m_buffer.size ()!=BUFFER_SIZE) {
			NS_LOG_UNCOND ("t = "<<now.GetSeconds ()<<" for source "<<m_myNodeId<<" to destination "<<destId);
			Time now = Simulator::Now();
			CoefElt coef;
			NetworkCodedDatagram* nc;
			nc = new NetworkCodedDatagram();
			coef.SetCoef (1);
			coef.SetIndex (m_nGeneratedPackets);
			coef.SetNodeId (m_myNodeId);
			coef.SetSource (m_myNCAppIp);
			coef.SetDestination (destId);
			coef.SetGenTime(now.GetNanoSeconds ());
			nc->m_coefsList.insert(MapType::value_type(coef.Key (),coef));
			m_buffer.push_front (nc);
			m_nGeneratedPackets++;
			MapType::iterator it = nc-> m_coefsList.begin ();
			NS_LOG_UNCOND ("t = "<<now.GetSeconds ()<<" source "<<m_myNodeId<<" have generated "<<coef.Key ()<<" coef is "<<int(it-> second. GetCoef()));
		}
	}
	if (m_waitingList.size()< WAITING_LIST_SIZE) {
		if (m_decodedBuf.size() < DECODED_BUFF_SIZE) {
			PacketInjector();
		}
		else
		{
				//ManageBuffer();
		}
	}
	//  Simulator::Schedule (Seconds (expVar->GetValue ()), &MyNCAppSource::GeneratePacket, this, node, destInterfaces, expVar, uniVar);
	Simulator::Schedule (Seconds (expVar->GetValue()), &MyNCApp::GeneratePacket, this);
}

void
MyNCApp::CheckWaitingList (std::list<Neighbor>::iterator listIterator)
{
  //Implement the back Pressure algorithm
  Time now = Simulator::Now ();
  NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" Checking waitinglist of source "<<m_myNodeId<<" ...");
  std::stringstream ss;
	ss << listIterator->neighborId;
	std::string strNodeId= ss.str();
  for (WaitingList::iterator it=m_waitingList.begin (); it!=m_waitingList.end ();it++) {
  	if(!(std::find (it->nodeForwardedTo.begin(), it->nodeForwardedTo.end(),strNodeId)!=it->nodeForwardedTo.end())) {
  	//Packet not forwarded to this neighbor
  		if ((listIterator->neighborDecodingFilter->contains(it->pktId)) || (listIterator->neighborDecodedFilter->contains(it->pktId))) {
  			it = m_waitingList.erase(it);
  		}
  	}
	  while (m_waitingList.size() < WAITING_LIST_SIZE) {
	  	if (!m_buffer.empty()) {
      	PacketInjector ();
	  	} else
	  		break;
    }
  }
}

void MyNCApp::UpdateWaitingList (std::string pktId)
{
	// implement back pressure
	//we assume that all neighbors have received a packet forwarded !!!!!
  Time now = Simulator::Now ();

  std::string strNodeId;
	for (WaitingList::iterator it=m_waitingList.begin(); it!=m_waitingList.end();it++) {
		if (it->pktId==pktId){
			for (std::list<Neighbor>::iterator neighborItr=m_neighborhood.begin(); neighborItr!=m_neighborhood.begin(); neighborItr++) {
				std::stringstream ss;
				ss << neighborItr->neighborId;
				strNodeId= ss.str();
				it->nodeForwardedTo.push_back(strNodeId);
			}
		}
	}
}

void MyNCApp::PacketInjector ()
{
  if (m_decodedBuf.size()==DECODED_BUFF_SIZE) {
 // 	manageBuffer();
  } else {
		NetworkCodedDatagram *p;
		p= new NetworkCodedDatagram();
		p=m_buffer.back();
		m_decodedBuf.push_back(p);
		m_buffer.pop_back();
		WaitingListMember waitElm;
		Time now = Simulator::Now ();
  	MapType::iterator it;
  	it = p -> m_coefsList.begin();
  	waitElm.pktId = (*it).first ;
  	waitElm.entranceTime = now.GetNanoSeconds();
  	m_waitingList.push_back (waitElm);
  	m_decodedList.push_back (it->second.GetPktId());
  	m_nInjectedPackets++;
  	NS_LOG_UNCOND ("t = "<< now.GetSeconds ()<<" Source "<<m_myNodeId<<" injects "<<it->first<<" from m_buffer to m_decodedBuf and m_nInjectedPackets is "<<m_nInjectedPackets);
  }
}




Experiment::Experiment()
{}

Experiment::Experiment(std::string name):
 	s_output (name),
  s_simulationTime (525),
  s_verbose (false),
  s_logComponent (false),
  s_nSource (3),
  s_nRelay (9),
  s_packetSize (1000),
  s_beaconInterval (1),
  s_packetInterval(0.008),
  s_nGeneratedBeacons (0),
  s_nReceivedBeacons (0),
  s_totalInjectedPackets (0),
  s_totalGeneratedPackets (0),
  s_nSrcForwardedPackets (0),
  s_nReceivedPackets (0),
  s_nForwardedPackets (0),
  s_nDuplicateRec (0),
  s_nDroppedPackets (0),
  s_bytesTotal (0),
  s_packetDelay(0.0)
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
		newApp->MakeSource();
		newApp->Start();
	}

	for (uint8_t i = 0; i < s_nRelay ; i++) {
		newApp=CreateObject<MyNCApp> ();
		myNodes.Get(i)->AddApplication(newApp);
		newApp->SetNode(myNodes.Get(i));
		rlyApps.Add(newApp);
		newApp->m_pktSize=s_packetSize;
		newApp->nNode=s_nRelay+s_nSource;
		newApp->m_packetInterval=s_packetInterval;
		newApp->m_beaconInterval=s_beaconInterval;
		newApp->Start();
	}

	allApps.Add(sourceApps);
	allApps.Add(rlyApps);

	// The DTN network nodes need a mobility model
	MobilityHelper mobility;
	ObjectFactory pos;
	pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
	pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
	pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
	Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
	mobility.SetPositionAllocator (taPositionAlloc);
	mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
		"Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=12.0]"),
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
	uint32_t nRelaysDroppedPackets = 0;
	uint32_t nReceivedLinearCombinations = 0;
	uint32_t bufferOccupation = 0;
	uint32_t sourceBuffOccupation = 0;
	double aBillion = 0x3B9ACA00;

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
		bufferOccupation += (ptrsrcApp->m_decodedBuf).size()+(ptrsrcApp->m_decodingBuf).size();
		sourceBuffOccupation += (ptrsrcApp->m_buffer).size();
		s_packetDelay+= ptrsrcApp->packetDelay;
		NS_LOG_UNCOND ("srcBuff size of source "<<(int)i<<" is "<< (ptrsrcApp->m_buffer).size());
		NS_LOG_UNCOND ("m_decodedBuf size of source "<<(int)i<<" is "<< (ptrsrcApp->m_decodedBuf).size());
		NS_LOG_UNCOND ("m_decodingBuf size of source "<<(int)i<<" is "<< (ptrsrcApp->m_decodingBuf).size());
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
		bufferOccupation += (ptrRlyApp->m_decodedBuf).size()+(ptrRlyApp->m_decodingBuf).size();
		s_nReceivedPackets += ptrRlyApp-> nReceivedPackets;
		s_packetDelay+= ptrsrcApp->packetDelay;
		NS_LOG_UNCOND ("s_decodedBuf size of relay "<<(int)i<<" is "<< (ptrRlyApp->m_decodedBuf).size());
		NS_LOG_UNCOND ("s_decodingBuf size of relay "<<(int)i<<" is "<< (ptrRlyApp->m_decodingBuf).size());
	}

	NS_LOG_UNCOND ("----------------------------------------------------------------------------------------------------------------");
	s_nGeneratedBeacons = nSourcesGeneratedBeacons + nRelaysGeneratedBeacons;
	s_nReceivedBeacons = nSourcesReceivedBeacons + nRelaysReceivedBeacons;
	s_nForwardedPackets = nSourcesForwardedPackets + nRelaysForwardedPackets;
	s_nDuplicateRec = nSourcesDuplicateRec + nRelaysDuplicateRec;
	s_nDroppedPackets = nSourcesDroppedPackets + nRelaysDroppedPackets;

	NS_LOG_UNCOND ("----------------------------------------------------------------------------------------------------------------");
	NS_LOG_UNCOND ("Total Number of generated datagram in sources is : "<<s_totalGeneratedPackets);
	NS_LOG_UNCOND ("Total Number of injected datagram from sources is : "<<s_totalInjectedPackets);
	NS_LOG_UNCOND ("Total Number of datagram received in destinations is : "<<s_nReceivedPackets);
    NS_LOG_UNCOND ("Delivery ratio is : "<<((double)s_nReceivedPackets/s_totalInjectedPackets)*100<<" %");
	NS_LOG_UNCOND ("Total Number of redundant datagram received in nodes is : "<<s_nDuplicateRec);
	NS_LOG_UNCOND ("Total Number of dropped datagram in nodes is : "<<s_nDroppedPackets);
	NS_LOG_UNCOND ("Total Number of received datagram in all nodes = "<<nReceivedLinearCombinations);
		    //NS_LOG_UNCOND ("Total Number of datagram forwarded from their source is : "<<m_nSrcForwardedPackets);
	NS_LOG_UNCOND ("Total Number of datagram forwarded from nodes is : "<<s_nForwardedPackets);
		    //NS_LOG_UNCOND ("Total Number of beacons received in nodes is : "<<m_nReceivedBeacons);
		    //NS_LOG_UNCOND ("Total Number of generated beacons is : "<<m_nGeneratedBeacons);
	NS_LOG_UNCOND ("Average Delay is : "<<s_packetDelay/s_nReceivedPackets/aBillion<<" seconds");
	NS_LOG_UNCOND ("Average m_Buffer occuapancy is : "<<bufferOccupation/ (s_nSource + s_nRelay));
	NS_LOG_UNCOND ("Average Source's m_srcBuff occupancy is : "<< sourceBuffOccupation / s_nSource);

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
	wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (25));
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




