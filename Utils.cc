
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
#include <stdint.h>
#include "Utils.h"


std::string StringConcat (uint16_t nodeId, uint16_t packetUniq)
{
	std::stringstream ss1,ss2;
	ss1 << nodeId;
	ss2 << packetUniq;
	return ss1.str()+ ":" + ss2.str();
}

//// LinearCombination definition  ///
LinearCombination::LinearCombination()
{
  coeff = 0;
  genTime=0;
}

LinearCombination::~LinearCombination()
{}


//// Matrix definition  ///
Matrix::Matrix ()
{}

Matrix::Matrix(int d11, int d22)
{
	d1=d11;
	d2=d22;
  for (int i=0; i<d1; i++)
	  {
      for (int j=0; j<d2; j++)
        {
          A[i][j]=0;
        }
	  }
	A.resize (d1, std::vector<int>(d2,0));
}

Matrix::~Matrix()
{}

void
Matrix::SetDimentions (int m, int n)
{
	d1=m;
	d2=n;
    A.resize (d1, std::vector<int>(d2,0));
	for (int i=0; i<m; i++)
	  {
      for (int j=0; j<n; j++)
        {
          A[i][j]=0;
        }
	  }
}

void
Matrix::SetValue(int i, int j, int value)
{
	A[i][j] = value;
}

int
Matrix::GetValue (int i, int j) const
{
	return A[i][j];
}

void
Matrix::PrintMatrix (int m, int n, int nodeId) const
{
    std::cout<<"t = "<<Simulator::Now(). GetSeconds()<<" In the printed matrix of nodeId = "<<nodeId<<" below, M = "<<m<<" and N = "<<n<<std::endl;
	for (int i=0; i<m; i++)
	  {
		  std::cout<<"ROW="<<i<<"	";
      for (int j=0; j<n; j++)
        {
            std::cout << GetValue(i,j)<<"	";
        }
          std::cout << std::endl;
	  }
}



LPMatrix::LPMatrix ()
{}

LPMatrix::LPMatrix(int d11, int d22)
{
	d1=d11;
	d2=d22;
  for (int i=0; i<d1; i++)
	  {
      for (int j=0; j<d2; j++)
        {
          A[i][j]=0.0;
        }
	  }
	A.resize (d1, std::vector<double>(d2,0));
}

LPMatrix::~LPMatrix()
{}

void
LPMatrix::SetDimentions (int m, int n)
{
	d1=m;
	d2=n;
    A.resize (d1, std::vector<double>(d2,0));
	for (int i=0; i<m; i++)
	  {
      for (int j=0; j<n; j++)
        {
          A[i][j]=0.0;
        }
	  }
}

void
LPMatrix::SetValue(int i, int j, double value)
{
	A[i][j] = value;
}

double
LPMatrix::GetValue (int i, int j) const
{
	return A[i][j];
}

void
LPMatrix::PrintMatrix (int m, int n, int nodeId) const
{
    std::cout<<"t = "<<Simulator::Now(). GetSeconds()<<" In the printed matrix of nodeId = "<<nodeId<<" below, M = "<<m<<" and N = "<<n<<std::endl;
	for (int i=0; i<m; i++)
	  {
		  std::cout<<"ROW="<<i<<"	";
      for (int j=0; j<n; j++)
        {
            std::cout << GetValue(i,j)<<"	";
        }
          std::cout << std::endl;
	  }
}//end of definition of class LPMatrix


Attribute::Attribute (const uint8_t nodeId, const int index, const uint8_t Id, const uint32_t genTime)
{
	m_nodeId=nodeId;
	m_index=index;
	m_destId=Id;
	m_genTime=genTime;
}

Attribute::~Attribute ()
{}

// implementation of Attribute methods
int
Attribute::GetIndex () const
{
	return m_index;
}

void
Attribute::SetIndex (int index)
{
	m_index=index;
}

uint32_t
Attribute::GetGenTime () const
{
    return m_genTime;
}
void
Attribute::SetGenTime (uint32_t genTime)
{
  m_genTime=genTime;
}

uint8_t
Attribute::GetNodeId () const
{
	return m_nodeId;
}

void
Attribute::SetNodeId (uint8_t nodeId)
{
	m_nodeId=nodeId;
}

bool
Attribute::operator==(const Attribute& p) const
{
	return m_nodeId==p.m_nodeId && m_index==p.m_index && m_destId==p.m_destId;
}

void
Attribute::SetDestination (uint8_t nodeId)
{
	m_destId=nodeId;
}

uint8_t
Attribute::GetDestination () const
{
	return m_destId;
}







// CoefElt Empty Constructor
CoefElt::CoefElt ()
{}

// CoefElt Constructor
CoefElt::CoefElt (uint8_t coef, int index, uint8_t nodeId, uint32_t genTime)
{
	m_coef=coef ;
	m_index=index;
	m_nodeId=nodeId;
	m_genTime=genTime;
}

// CoefElt Destructor
CoefElt::~CoefElt ()
{}

// Accessor Methods
uint8_t
CoefElt::GetCoef() const
{
	return m_coef;
}

void
CoefElt::SetCoef (uint8_t coef)
{
	m_coef=coef;
}

int
CoefElt::GetIndex() const
{
	return m_index;
}

void
CoefElt::SetIndex(int index)
{
  m_index=index;
}

uint8_t
CoefElt::GetNodeId ()
{
	return m_nodeId;
}

void
CoefElt::SetNodeId (uint8_t nodeId)
{
	m_nodeId = nodeId;
}

uint32_t
CoefElt::GetGenTime () const
{
  return m_genTime;
}

void
CoefElt::SetGenTime (uint32_t genTime)
{
  m_genTime=genTime;
}

void
CoefElt::SetSource (const Ipv4Address& ip)
{
	m_srcIp = ip;
}

Ipv4Address
CoefElt::GetSource () const
{
	return m_srcIp;
}

void
CoefElt::SetDestination (const uint8_t id)
{
	m_destId = id;
}

uint8_t
CoefElt::GetDestination () const
{
	return m_destId;
}

std::string
CoefElt::Key()
{
	std::string str = StringConcat (m_nodeId, m_index);
	return str;
}

Attribute
CoefElt::GetPktId () const
{
	Attribute pktId (m_nodeId, m_index, m_destId);
	return pktId;
}

bool
CoefElt::operator== (const CoefElt& coef) const
{
	if ((m_nodeId==coef.m_nodeId)&& (m_index==coef.m_index))
    {
      return true;
    }
	else
    {
      return false;
    }
}

CoefElt&
CoefElt::operator= (const CoefElt& coef)
{
	m_nodeId=coef.m_nodeId;
	m_index=coef.m_index;
	m_length=coef.m_length;
	m_destId= coef.m_destId;
	return *this;
}






// NetworkCodedDatagram Constructors
NetworkCodedDatagram::NetworkCodedDatagram ()
{
	m_dataLength=1000;
	/*
	 p(x) = 1x^8+1x^7+0x^6+0x^5+0x^4+0x^3+1x^2+1x^1+1x^0
	 1    1    0    0    0    0    1    1    1
	 */
    //unsigned int poly[9] = {1,1,1,0,0,0,0,1,1};
	//galois::GaloisField gf (pwr, poly);
	//m_galoisField = new galois::GaloisField (gf);
	//try
  //{
  //  m_galoisField = new galois::GaloisField (8, poly);
  //}
	//catch (bad_alloc&)
  //{
  //  cout << "Error allocating memory to NetworkCodedDatagram galios field." << endl;
  //}
	m_coefsList. clear ();
}
/*
NetworkCodedDatagram::NetworkCodedDatagram (int index)
{
	m_dataLength=1000;
	m_index=index;

	 p(x) = 1x^8+1x^7+0x^6+0x^5+0x^4+0x^3+1x^2+1x^1+1x^0
	 1    1    0    0    0    0    1    1    1
	 */
	//unsigned int poly[9] = {1,1,1,0,0,0,0,1,1};
	//galois::GaloisField gf (pwr, poly);
	//m_galoisField = new galois::GaloisField (gf);
	/*try
	  {
		  m_galoisField = new galois::GaloisField (8, poly);
	  }
	catch (bad_alloc&)
	  {
		  cout << "Error allocating memory to NetworkCodedDatagram galios field." << endl;
	  }
	m_coefsList. clear ();
	m_genTime = 0;
}
*/
NetworkCodedDatagram&
NetworkCodedDatagram::operator= (const NetworkCodedDatagram& nc)
{
  m_dataLength = nc.m_dataLength;
  m_index = nc.m_index;
  m_genTime = nc.m_genTime;
  MapType::iterator it;
  MapType tmpMap;
  tmpMap = nc.m_coefsList;
  m_coefsList.clear ();
  for (it=tmpMap.begin (); it!=tmpMap.end (); it++)
    {
      //m_coefsList[(*it).first] = (*it).second ;
      m_coefsList.insert (MapType::value_type((*it).first, (*it).second));
    }
  return *this;
}

bool
NetworkCodedDatagram::operator== (const NetworkCodedDatagram& nc) const
{
  MapType::iterator it, it2;
  MapType tmpMap, myMap;
  tmpMap = nc.m_coefsList;
  myMap = m_coefsList;
  for (it= myMap.begin(); it!= myMap.end() ; it++)
    {
      if ((*it).first != (*it2).first)
        {
          return false;
        }
      it2++;
    }
  return true;
}

// NetworkCodedDatagram Destructor
NetworkCodedDatagram::~NetworkCodedDatagram ()
{
    //delete m_galoisField;
}

// NetworkCodedDatagram Methods
int
NetworkCodedDatagram::GetIndex () const
{
	return m_index;
}

void
NetworkCodedDatagram::SetIndex (int index)
{
  m_index=index;
}

bool
NetworkCodedDatagram::IsNull()
{
	if (m_coefsList.size()==0)
	  {
      return true;
	  }
	else
	  {
      return false;
	  }
}

int
NetworkCodedDatagram::GetLength() const
{
	return m_dataLength;
}

void
NetworkCodedDatagram::SetLength (int length)
{
	m_dataLength=length;
}

void
NetworkCodedDatagram::SetDecoded()
{
	m_decoded=true;
}

void
NetworkCodedDatagram::ResetDecoded()
{
	m_decoded=false;
}

bool
NetworkCodedDatagram::IsDecoded() const
{
	return m_decoded;
}

// Validate and LineValidate must be implemented here :
// Product implementations :
void
NetworkCodedDatagram::Product(int coef, galois::GaloisField *galois)
{
  MapType::iterator it;

	// handling the coefsList
	for (it=m_coefsList.begin (); it!=m_coefsList.end (); it++)
	  {
		  ((*it).second). SetCoef (galois->mul ((*it).second. GetCoef (), coef));
	  }
}

// Sum implementation :
void
NetworkCodedDatagram::Sum (NetworkCodedDatagram& g, galois::GaloisField *galois)
{
	MapType::iterator it, itr;
	for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end (); it++)
	  {
		  itr = m_coefsList.find (it-> first);
      if (itr!=m_coefsList.end ())
		    {
          (*itr).second. SetCoef (galois->add ((*itr).second.GetCoef(),(*it).second.GetCoef()));
          if ((*itr).second. GetCoef ()==0)
            {
              m_coefsList.erase (itr);
            }
        }
		  else
		    {
          m_coefsList.insert(MapType::value_type(it-> first,(*it).second));
		    }
	  }
}

// Minus Implementation :
void
NetworkCodedDatagram::Minus (NetworkCodedDatagram& g, galois::GaloisField *galois)
{
	MapType::iterator it, itr;
	for (it=g.m_coefsList.begin (); it!=g.m_coefsList.end (); it++)
    {
      itr = m_coefsList.find (it->first);
      if (itr!=m_coefsList.end ())
        {
          (*itr).second. SetCoef (galois->sub ((*itr).second.GetCoef(),(*it).second.GetCoef()));
          if ((*itr).second. GetCoef()==0)
            {
              m_coefsList.erase (itr);
            }
        }
      else
        {
          m_coefsList.insert(MapType::value_type((*it).first,(*it).second));
        }
    }
}
