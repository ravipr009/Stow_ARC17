#include "bin.h"

Bin_ARC_2017::Bin_ARC_2017(float rack_length,
                           float rack_width,
                           float row_offset,
                           float column_offset)

{
    this->rack_length = rack_length;
    this->rack_width = rack_width;
    this->row_offset = row_offset;
    this->column_offset = column_offset;


    float rack_length_1 = this->rack_length/2;

    int rows_1 = 2;
    int columns_1 = 2;

    int rows_2 = 2;
    int columns_2 = 2;

    for(int i = 0; i< rows_1; i++)
    {
        row_offset = 0;
        if(i == 0)
            row_offset = this->row_offset;
        if(i == rows_1 - 1)
            row_offset = row_offset - this->row_offset;

        for(int k = 0; k< columns_1; k++)
        {
            column_offset = 0;
            if(k == 0)
                column_offset = this->column_offset;
            if(k == columns_1 - 1)
                column_offset = column_offset - this->column_offset;

            for(int j = 0; j< 3; j++)
            {
                if(j == 1)
                    this->bin_1_position[i*columns_1 + k][j] = row_offset + rack_width/(2*rows_1) + i*rack_width/(rows_1);
                if(j == 0)
                    this->bin_1_position[i*columns_1 + k][j] = column_offset + rack_length_1/(2*columns_1) + k*rack_length_1/(columns_1);
                if(j == 2)
                    this->bin_1_position[i*columns_1 + k][j] = 0.20;
            }

            std::cout << this->bin_1_position[i*columns_1 + k][0] <<", "<< this->bin_1_position[i*columns_1 + k][1] <<", "<< this->bin_1_position[i*columns_1 + k][2] <<"\n ";

        }
    }

    float rack_length_2 = this->rack_length/2;


    for(int i = 0; i< rows_2; i++)
    {
        row_offset = 0;
        if(i == 0)
            row_offset = this->row_offset;
        if(i == rows_2 - 1)
            row_offset = row_offset - this->row_offset;

        for(int k = 0; k< columns_2; k++)
        {
            column_offset = 0;
            if(k == 0)
                column_offset = this->column_offset;
            if(k == columns_2 - 1)
                column_offset = column_offset - this->column_offset;

            for(int j = 0; j< 3; j++)
            {
                if(j == 1)
                    this->bin_2_position[i*columns_2 + k][j] = row_offset + rack_width/(2*rows_2) + i*rack_width/(rows_2);
                if(j == 0)
                    this->bin_2_position[i*columns_2 + k][j] = column_offset + rack_length_1 + rack_length_2/(2*columns_2) + k*rack_length_2/(columns_2);
                if(j == 2)
                    this->bin_2_position[i*columns_2 + k][j] = 0.20;
            }

            std::cout << this->bin_2_position[i*columns_2 + k][0] <<", "<< this->bin_2_position[i*columns_2 + k][1] <<", "<< this->bin_2_position[i*columns_2 + k][2] <<"\n ";

        }
    }

}


double* Bin_ARC_2017::set_bin_number_and_bin_location(int bin_number, int bin_position)
{
    switch(bin_number)
    {
    case  1:
        return this->bin_1_position[bin_position];

    case 2:
        return this->bin_2_position[bin_position];
    }
}
