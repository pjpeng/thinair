
#include "random.h"

void MSLRandomSource::init_table()
{ 
  table[ 0] = -1708027847;
  table[ 1] =   853131300;
  table[ 2] = -1687801470;
  table[ 3] =  1570894658;
  table[ 4] =  -566525472;
  table[ 5] =  -552964171;
  table[ 6] =  -251413502;
  table[ 7] =  1223901435;
  table[ 8] =  1950999915;
  table[ 9] = -1095640144;
  table[10] = -1420011240;
  table[11] = -1805298435;
  table[12] = -1943115761;
  table[13] =  -348292705;
  table[14] = -1323376457;
  table[15] =   759393158;
  table[16] =  -630772182;
  table[17] =   361286280;
  table[18] =  -479628451;
  table[19] = -1873857033;
  table[20] =  -686452778;
  table[21] =  1873211473;
  table[22] =  1634626454;
  table[23] = -1399525412;
  table[24] =   910245779;
  table[25] =  -970800488;
  table[26] =  -173790536;
  table[27] = -1970743429;
  table[28] =  -173171442;
  table[29] = -1986452981;
  table[30] =   670779321;

  ptr0    = table;
  ptr1    = table + 3;
  ptr_end = table + 31;
}


void MSLRandomSource::set_seed(int x)
{ 
	table[0] = (unsigned)x;
	int i;
	for(i=1; i<31; i++)
		table[i] = 1103515245 * table[i-1] + 12345;
	ptr0 = table;
	ptr1 = table + 3;
	for(i=0; i<310; i++)
		get_rand31();
}


#define RANDMAX 0x7FFFFFFF

unsigned long MSLRandomSource::get_rand31()
{ 
	*ptr1 += *ptr0;
	long i = (*ptr1 >> 1) & 0x7fffffff;
	if (++ptr0 >= ptr_end) 
		ptr0 = table;
	if (++ptr1 >= ptr_end) 
		ptr1 = table;
	return (unsigned long)i;
}

int randomcount = 0;

MSLRandomSource::MSLRandomSource()
{
	init_table();
	time_t seed;
	time(&seed);
	randomcount++;
	set_seed(int(seed)*randomcount);
	pat = 0xFFFFFFFF; 
	prec = 31;
	bit_mode = true;
	low = diff = 0;
	buffer = 0x00000001;
 }

MSLRandomSource& MSLRandomSource::operator>>(double& x)
{ 
	x = double(get_rand31())/RANDMAX; 
	return *this;
}


