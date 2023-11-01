// # include "holeType.h"
#include <iostream>
#include <cmath>
#include <chrono>

using namespace std;


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

	double pAngle =  18.0 * deg;
	double tAngle =  0.0 * deg;

	double r = 6.92 * mm;
	double w = 3.0 * mm;
	double disWire = 1.05 * mm;
	double slotLength = 1.05 * mm;
	double slotWidth = 0.3 * mm;

	double alphaP = asin((disWire + 0.5*slotWidth) / r);
	double alphaN = asin((disWire - 0.5*slotWidth) / r);
	double beta = asin(slotLength * 0.5 / r);

	double wrLengthWest, wrLengthEast, wrLengthSouth, wrLengthNorth;

	double checkWire = 2*5*r*(1-cos(alphaP));

	cout << checkWire / mm << endl;

	if (pAngle < 0) {
		if (tAngle < 0) {
			wrLengthWest  = 2 * r * NumJ * (cos(alphaP) - cos(alphaP - pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthEast  = 2 * r * NumJ * (cos(alphaN) - cos(alphaN + pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthSouth = 2 * r * NumJ * (cos(alphaP) - cos(alphaP - tAngle / 2) + 1 - cos(pAngle / 2));
			wrLengthNorth = 2 * r * NumJ * (cos(alphaN) - cos(alphaN + tAngle / 2) + 1 - cos(pAngle / 2));
		}
		else {
			wrLengthWest  = 2 * r * NumJ * (cos(alphaP) - cos(alphaP - pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthEast  = 2 * r * NumJ * (cos(alphaN) - cos(alphaN + pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthSouth = 2 * r * NumJ * (cos(alphaN) - cos(alphaN - tAngle / 2) + 1 - cos(pAngle / 2));
			wrLengthNorth = 2 * r * NumJ * (cos(alphaP) - cos(alphaP + tAngle / 2) + 1 - cos(pAngle / 2));
		}
	}
	else {
		if (tAngle < 0) {
			wrLengthWest  = 2 * r * NumJ * (cos(alphaN) - cos(alphaN - pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthEast  = 2 * r * NumJ * (cos(alphaP) - cos(alphaP + pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthSouth = 2 * r * NumJ * (cos(alphaP) - cos(alphaP - tAngle / 2) + 1 - cos(pAngle / 2));
			wrLengthNorth = 2 * r * NumJ * (cos(alphaN) - cos(alphaN + tAngle / 2) + 1 - cos(pAngle / 2));
		}
		else {
			wrLengthWest  = 2 * r * NumJ * (cos(alphaN) - cos(alphaN - pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthEast  = 2 * r * NumJ * (cos(alphaP) - cos(alphaP + pAngle / 2) + 1 - cos(tAngle / 2));
			wrLengthSouth = 2 * r * NumJ * (cos(alphaN) - cos(alphaN - tAngle / 2) + 1 - cos(pAngle / 2));
			wrLengthNorth = 2 * r * NumJ * (cos(alphaP) - cos(alphaP + tAngle / 2) + 1 - cos(pAngle / 2));
		}
	}

	double posLimitWire = 0.107313 * mm;
	double negLimitWire = -0.0759263 * mm;

	if (pAngle < 1 * deg && pAngle >= 0 * deg && tAngle < 1 * deg && tAngle >= 0 * deg) {
		wrLengthWest = negLimitWire / deg * pAngle;
		wrLengthEast = posLimitWire / deg * pAngle;
		wrLengthSouth = negLimitWire / deg * tAngle;
		wrLengthNorth = posLimitWire / deg * tAngle;
	}

	if (pAngle < 0 * deg && pAngle >= -1 * deg && tAngle < 1 * deg && tAngle >= 0 * deg) {
		wrLengthWest = - posLimitWire / deg * pAngle;
		wrLengthEast = - negLimitWire / deg * pAngle;
		wrLengthSouth = negLimitWire / deg * tAngle;
		wrLengthNorth = posLimitWire / deg * tAngle;
	}

	if (pAngle < 0 * deg && pAngle >= -1 * deg && tAngle < 0 * deg && tAngle >= -1 * deg) {
		wrLengthWest = - posLimitWire / deg * pAngle;
		wrLengthEast = - negLimitWire / deg * pAngle;
		wrLengthSouth = -posLimitWire / deg * tAngle;
		wrLengthNorth = -negLimitWire / deg * tAngle;
	}

	if (pAngle < 1 * deg && pAngle >= 0 * deg && tAngle < 0 * deg && tAngle >= -1 * deg) {
		wrLengthWest = negLimitWire / deg * pAngle;
		wrLengthEast = posLimitWire / deg * pAngle;
		wrLengthSouth = -posLimitWire / deg * tAngle;
		wrLengthNorth = -negLimitWire / deg * tAngle;
	}


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