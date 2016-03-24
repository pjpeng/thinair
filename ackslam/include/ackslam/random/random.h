
#include "MTRand.h"

#ifndef MSL_RANDOM_H
#define MSL_RANDOM_H

class MSLRandomSource {

public:
	long table[31];
	long *ptr0, *ptr1, *ptr_end;
	long buffer;
	long pat;
	int prec;
	bool bit_mode;
	int low,diff;

	void set_seed(int x);
	unsigned long get_rand31();
	void init_table();

public:

	typedef unsigned long uint32;  // unsigned integer type, at least 32 bits
								 //put in for use with MTRand class
	MSLRandomSource();

	double getUniform();
	double getGaussian(double mean, double variance);
	int getRangedInt(int low, int high);
	double getRangedDouble(double low, double high);
	float getRangedFloat(float low, float high);

	MSLRandomSource& operator>>(double& x);

private:
	MTRand *mtr;
};

#endif
