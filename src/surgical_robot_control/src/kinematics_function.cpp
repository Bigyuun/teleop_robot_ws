#include "kinematics_function.hpp"

const double PI = acos(-1);


int main()
{
	auto start = std::chrono::high_resolution_clock::now();

	// Make wire
	double deg = PI / 180;
	double rad = 180 / PI;
	double mm = 0.001;

	int NumJ = 5; // The number of joints

	// Joint angle -> Joystic mapping / NumJ

	double pAngle = 5 * deg;
	double tAngle = -5 * deg;

	double r = 7 * mm;
	double w = 4.5 * mm;
	double disWire = 1.8 * mm;

	double alpha = asin(disWire / r);

	double wrLengthWest, wrLengthEast, wrLengthSouth, wrLengthNorth;

	wrLengthWest = 2 * r * NumJ * (cos(alpha) - cos(alpha - pAngle / 2) + 1 - cos(tAngle / 2));
	wrLengthEast = 2 * r * NumJ * (cos(alpha) - cos(alpha + pAngle / 2) + 1 - cos(tAngle / 2));
	wrLengthSouth = 2 * r * NumJ * (cos(alpha) - cos(alpha - tAngle / 2) + 1 - cos(pAngle / 2));
	wrLengthNorth = 2 * r * NumJ * (cos(alpha) - cos(alpha + tAngle / 2) + 1 - cos(pAngle / 2));

	cout << "South wire length: " << wrLengthSouth / mm << endl;
	cout << "North wire length: " << wrLengthNorth / mm << endl;
	cout << "East wire length: " << wrLengthEast / mm << endl;
	cout << "West wire length: " << wrLengthWest / mm << endl;

	// Wire length -> Motor position transform

	// Monitoring

	auto end = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double> duration = end - start;
	std::cout << "���� �ð�: " << duration.count() << " ��" << std::endl;

	return 0;
}