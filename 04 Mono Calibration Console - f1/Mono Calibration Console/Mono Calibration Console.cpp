// Mono Calibration Console.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/src/SVD/JacobiSVD.h>

using namespace std;
using namespace Eigen;

void Compute_Projection_Matrix(int point_count, float* world_points, float* image_points, float* A);

int main()
{
    std::cout << "Hello World!\n"; 
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

void Compute_Projection_Matrix(int point_count, float * world_points, float * image_points, float * A)
{
	MatrixXf D(point_count * 2, 11);
	VectorXf R(point_count * 2);
	float X, Y, Z, x, y;
	// D ve R matrislerinin doldurulması
	for (int i = 0; i < point_count; i++) {
		X = world_points[i * 3];
		Y = world_points[i * 3 + 1];
		Z = world_points[i * 3 + 2];
		x = image_points[i * 2];
		y = image_points[i * 2 + 1];

		// R'nin doldurulması
		R(i * 2) = x;
		R(i * 2 + 1) = y;

		float current_point_data[22] = { X,Y,Z,1,0,0,0,0,-X * x,-Y * x,-Z * x,
										 0,0,0,0,X,Y,Z,1,-X * y,-Y * y,-Z * y };
		
		//D'nin doldurulması
		for (int j = 0; j < 11; j++) {
			D(i * 2, j) = current_point_data[j];
			D(i * 2 + 1, j) = current_point_data[11 + j];
		}
	}
}
