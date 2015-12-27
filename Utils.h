#ifndef INCLUDE_UTILS_H
#define INCLUDE_UTILS_H
#include "GaloisField.h"

using namespace ns3;

static const std::size_t BITS_PER_CHAR = 0x08;    // 8 bits in 1 char (unsigned)
static const float NEIGHBOR_R = 1.5;

std::string StringConcat (uint16_t nodeId, uint16_t packetUniq);

class LinearCombination
{
public:
	LinearCombination();
  ~LinearCombination();
  std::string pktId;
	uint8_t coeff;
	uint8_t dstId;
	uint32_t genTime;
};

class Matrix //this class is for coefficient of packets in NC
{
public:
	int d1, d2; //dimensions
    std::vector<std::vector<int> > A; // All values of matrix
	Matrix ();
	Matrix(int, int); //Constructor will only create the space here LLL
	~Matrix();
	void SetDimensions (int, int);
	void SetValue(int, int, int); // first two args are element position specifiers.
	int	GetValue (int i, int j) const;
	void PrintMatrix (int m, int n, int nodeId) const;
};

class NCAttribute : public ns3::Object {
	// data members
public:
	uint8_t m_nodeId;
	int8_t m_index;
	uint8_t m_destId;
	uint32_t m_genTime;
	uint32_t m_receptionNum;
	uint32_t m_sendingNum;
	uint32_t m_length;
	NCAttribute();
	NCAttribute(const NCAttribute& );
	NCAttribute (const uint8_t nodeId,const int8_t index,const uint8_t id, const uint32_t genTime);
	~NCAttribute ();
	//member functions
	void SetNodeId (uint8_t nodeId);
	uint8_t GetNodeId () const;
	void SetIndex (int index);
	uint8_t GetIndex () const;
	void SetGenTime (uint32_t genTime);
	uint32_t GetGenTime () const;
	void SetDestination (const uint8_t ip);
	uint8_t GetDestination () const;
	std::string Key();

	bool operator==(const NCAttribute& p) const;
	NCAttribute& operator= (const NCAttribute& p);
};

// class CoefElt
class CoefElt
{
public:
	// Data Members
	uint8_t m_coef;
	int m_index;
	uint8_t m_nodeId;
	uint8_t m_destId;
	uint8_t m_length;
	uint32_t m_genTime;
	// Constructors and a Destructor
	CoefElt ();
	CoefElt (uint8_t coef, int index, uint8_t nodeId, uint32_t genTime);
	~CoefElt ();
	// Methods
	uint8_t GetCoef () const;
	void SetCoef (uint8_t);
	int GetIndex() const;
	void SetIndex (int index);
	uint8_t GetNodeId ();
	void SetNodeId (uint8_t nodeId);
	uint32_t GetGenTime () const;
	void SetGenTime (uint32_t genTime);
	void SetDestination (uint8_t destId);
	uint8_t GetDestination () const;
	std::string Key ();
	NCAttribute GetAttribute () const;
	bool operator== (const CoefElt& coef) const;
	CoefElt& operator= (const CoefElt& coef);
};

typedef std::map<std::string, CoefElt> MapType;


// class NetworkCodedDatagram
class NetworkCodedDatagram : public ns3::Object {
public:
	// Fields
	//galois::GaloisField *m_galoisField;
	MapType m_coefsList;
	int m_dataLength;
	int m_index;
	bool m_decoded;
	//char * payload;

	// Constructor and Destructor
	NetworkCodedDatagram ();
	NetworkCodedDatagram (NetworkCodedDatagram& );
	~NetworkCodedDatagram ();
    //NetworkCodedDatagram (int index);
	NetworkCodedDatagram& operator= (const NetworkCodedDatagram& nc);
	bool operator== (const NetworkCodedDatagram& nc) const;
	// Methods
	// void InitializeGaloisField ();
	int GetIndex () const;
	void SetIndex (int index);
	bool IsNull ();
	int GetLength() const;
	void SetLength (int length);
	void SetDecoded();
	void ResetDecoded ();
	bool IsDecoded() const;
	void Product(int coef, galois::GaloisField *galois);
  void Sum (NetworkCodedDatagram& g, galois::GaloisField *galois);
	void Minus (NetworkCodedDatagram& g, galois::GaloisField *galois);
};


class LPMatrix // we instantiate from this class Only for constraint matrices in Encode
{
public:
	int d1, d2; //dimensions
    std::vector<std::vector<double> > A; // All values of matrix
	LPMatrix ();
	LPMatrix(int, int); //Constructor will only create the space here LLL
	~LPMatrix();
	void SetDimensions (int, int);
	void SetValue(int, int, double); // first two args are element position specifiers.
	double	GetValue (int i, int j) const;
	void PrintMatrix (int m, int n, int nodeId) const;
};



#endif
