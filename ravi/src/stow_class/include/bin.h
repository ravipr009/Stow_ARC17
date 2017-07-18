#ifndef __ARC_2017_BIN_H_
#define __ARC_2017_BIN_H_

#include <iostream>

class Bin_ARC_2017
{
public:
Bin_ARC_2017(float , float, float , float);

float rack_length ;
float rack_width ;

float row_offset;
float column_offset;

int bin_number;

int bin_locations[2];

double bin_1_position[4][3];
double bin_2_position[4][3];


double* set_bin_number_and_bin_location(int, int);


private:


};




#endif //__ARC_2017_BIN_H_
