#ifndef MERSENNETWISTER_H
#define MERSENNETWISTER_H


#include <iostream>
#include <limits.h>
#include <cstdio>
#include <ctime>
#include <cmath>
using namespace std;

class MTRand 
{
// Data
public:
	typedef unsigned long uint32;  // unsigned integer type, at least 32 bits
	
	enum { N = 624 };       // length of state vector
	enum { SAVE = N + 1 };  // length of array for save()

protected:
	enum { M = 397 };  // period parameter
	
	uint32 state[N];   // internal state
	uint32 *pNext;     // next value to get from state
	int left;          // number of values left before reload needed


//Methods
public:
	MTRand( const uint32& oneSeed );  // initialize with a simple uint32
	MTRand( uint32 *const bigSeed, uint32 const seedLength = N );  // or an array
	MTRand();  // auto-initialize with /dev/urandom or time() and clock()
		
	double rand();                          // real number in [0,1]
	double rand( const double& n );         // real number in [0,n]
	double randExc();                       // real number in [0,1)
	double randExc( const double& n );      // real number in [0,n)
	double randDblExc();                    // real number in (0,1)
	double randDblExc( const double& n );   // real number in (0,n)
	uint32 randInt();                       // integer in [0,2^32-1]
	uint32 randInt( const uint32& n );      // integer in [0,n] for n < 2^32
	double operator()() { return rand(); }  // same as rand()
	
	double rand53();  // real number in [0,1)
	
	double randNorm( const double& mean = 0.0, const double& variance = 0.0 );
	
	void seed( const uint32 oneSeed );
	void seed( uint32 *const bigSeed, const uint32 seedLength = N );
	void seed();
	
	void save( uint32* saveArray ) const;  // to array of size SAVE
	void load( uint32 *const loadArray );  // from such array
	friend std::ostream& operator<<( std::ostream& os, const MTRand& mtrand );
	friend std::istream& operator>>( std::istream& is, MTRand& mtrand );

protected:
	void initialize( const uint32 oneSeed );
	void reload();
	uint32 hiBit( const uint32& u ) const { return u & 0x80000000UL; }
	uint32 loBit( const uint32& u ) const { return u & 0x00000001UL; }
	uint32 loBits( const uint32& u ) const { return u & 0x7fffffffUL; }
	uint32 mixBits( const uint32& u, const uint32& v ) const
		{ return hiBit(u) | loBits(v); }
	uint32 twist( const uint32& m, const uint32& s0, const uint32& s1 ) const
		{ return m ^ (mixBits(s0,s1)>>1) ^ (-loBit(s1) & 0x9908b0dfUL); }
	static uint32 hash( time_t t, clock_t c );
};

#endif  // MERSENNETWISTER_H
