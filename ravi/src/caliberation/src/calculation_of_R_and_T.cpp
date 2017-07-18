#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;


MatrixXf find_rotation_matrix(MatrixXf tip_point_wrt_wrist_2_link, MatrixXf points_wrt_kinect)

{
    int size = tip_point_wrt_wrist_2_link.rows() - 1;

    MatrixXf difference_kinect_data_matrix(size, 3), difference_wrist_3_data_matrix(size, 3), M(3,3), final_r_mat(3,3);

    for(int i=0;i<size;i++)
    {
        difference_wrist_3_data_matrix.row(i) = tip_point_wrt_wrist_2_link.row(i)-tip_point_wrt_wrist_2_link.row(i+1);
        difference_kinect_data_matrix.row(i) = points_wrt_kinect.row(i)-points_wrt_kinect.row(i+1);
    }

    M = difference_wrist_3_data_matrix.transpose()*difference_kinect_data_matrix;

    JacobiSVD<MatrixXf> svd( M, ComputeFullV | ComputeFullU );

    final_r_mat = svd.matrixU()*(svd.matrixV().transpose());

    return final_r_mat;
}

MatrixXf find_translation_matrix(MatrixXf tip_point_wrt_wrist_2_link, MatrixXf points_wrt_kinect, MatrixXf final_r_mat)
{
    MatrixXf final_t_mat(3,1);
    final_t_mat = tip_point_wrt_wrist_2_link - (final_r_mat*points_wrt_kinect);
    return final_t_mat;
}


int main()
{
    fstream myfile;

    char c;
    float a;


    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates_wrt_kinect.txt", std::ios::in);

    int num_kinect_points = 0;

    while (myfile.get(c))
    {
        if(c == '\n')
            num_kinect_points = num_kinect_points + 1;
    }

    myfile.close();


    MatrixXf points_wrt_kinect(num_kinect_points, 3);


    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates_wrt_kinect.txt", std::ios::in);

    int row = 0;
    int column = 0;

    do
    {

        myfile >> a;
        points_wrt_kinect(row, column) = a;

        column = column + 1;
        column = column % 3;
        if(column == 0)
            row = row + 1;

        if(row == num_kinect_points)
            break;

    }while (myfile.get(c));

    myfile.close();




    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates_wrt_ee_link.txt", std::ios::in);

    int num_wrist2_points = 0;

    while (myfile.get(c))
    {
        if(c == '\n')
            num_wrist2_points = num_wrist2_points + 1;
    }

    myfile.close();


    MatrixXf points_wrt_wrist(num_wrist2_points, 3);


    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/marker_coordinates_wrt_ee_link.txt", std::ios::in);

    row = 0;
    column = 0;

    do
    {

        myfile >> a;
        points_wrt_wrist(row, column) = a;

        column = column + 1;
        column = column % 3;
        if(column == 0)
            row = row + 1;

        if(row == num_wrist2_points)
            break;

    }while (myfile.get(c));

    myfile.close();




    MatrixXf  final_r_mat(3,3), final_t_mat(3,1), final_matrix(4,4);



    final_r_mat = find_rotation_matrix(points_wrt_wrist, points_wrt_kinect);

    final_t_mat = find_translation_matrix(points_wrt_wrist.row(0).transpose(), points_wrt_kinect.row(0).transpose(), final_r_mat);

    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/final_R_and_T.txt", std::ios::out);

    std::cout << "calculated _r_mat----->\n" << final_r_mat
              << "\n\n\ncalculated _t_mat----->\n" << final_t_mat << "\n";


    final_matrix.block(0, 0, 3, 3) << final_r_mat;
    final_matrix.block(0, 3, 3, 1) << final_t_mat;
    final_matrix.block(3, 0, 1, 4) << 0, 0, 0, 1;

    myfile << final_matrix;

    myfile.close();



    return 0;
}

