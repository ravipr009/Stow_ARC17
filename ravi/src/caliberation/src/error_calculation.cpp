#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Eigen;



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




    MatrixXf transformation_matrix(4, 4);


    myfile.open ("/home/ravi/Desktop/APC_stow/stow_ws/src/caliberation/data_files/final_R_and_T.txt", std::ios::in);

    row = 0;
    column = 0;

    do
    {

        myfile >> a;
        transformation_matrix(row, column) = a;

        column = column + 1;
        column = column % 4;
        if(column == 0)
            row = row + 1;

        if(row == 4)
            break;

    }while (myfile.get(c));

    myfile.close();




    MatrixXf r_mat(3, 3);
    r_mat << transformation_matrix.block(0, 0, 3, 3);

    MatrixXf t_mat(3, 1);
    t_mat << transformation_matrix.block(0, 3, 3, 1);

    float calib_error = 0;

    MatrixXf A(num_kinect_points, 3);

    for(int i = 0; i<num_kinect_points; i++)
    {
        A.row(i) = (r_mat * points_wrt_kinect.row(i).transpose() + t_mat).transpose();
        calib_error = calib_error + (A.row(i) - points_wrt_wrist.row(i)).norm();
    }

    std::cout <<"\nmarkers wrt wrist2 ------------------- markers wrt kinect ------------------ ransformed markers from kinect frame to world\n";

    for(int i = 0; i<num_kinect_points; i++)
    {

        //std::cout << std::setw(10)
        std::cout << points_wrt_wrist.row(i) << "\t\t\t\t"
                  << points_wrt_kinect.row(i) << "\t\t\t\t"
                  << A.row(i) << "\n";
    }

    std::cout <<"\n\n\nAvg error = " << calib_error/ num_kinect_points<< "\n";














    return 0;
}


