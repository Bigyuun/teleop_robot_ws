# include "holeType.h"

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

	double pAngle =  18* deg;
	double tAngle = 0 * deg;

	double r = 6.92 * mm;
	double w = 3.0 * mm;
	double disWire = 1.05 * mm;

	double alpha = asin(disWire / r);

	double wrLengthWest, wrLengthEast, wrLengthSouth, wrLengthNorth;

	float scale_factor_kw  = 1.0;
	float scale_factor_ke  = 1.0;
	float scale_factor_ks  = 1.0;
	float scale_factor_kn  = 1.0;
	float pullWire = 1.0;
	float pushWire = 0.9;

	float scale_factor_p1 = 1.0; 
	float scale_factor_p2 = 1.0; 
	float scale_factor_p3 = 1.0; 

	if (pAngle >= 0 && tAngle >= 0){
			scale_factor_kw  = pullWire;
			scale_factor_ke  = pushWire;
			scale_factor_ks  = pullWire;
			scale_factor_kn  = pushWire;
	}
	else if (pAngle < 0 && tAngle >= 0)
	{	
			scale_factor_kw  = pushWire;
			scale_factor_ke  = pullWire;
			scale_factor_ks  = pullWire;
			scale_factor_kn  = pushWire;
	}
		else if (pAngle >= 0 && tAngle < 0)
	{
			scale_factor_kw  = pullWire;
			scale_factor_ke  = pushWire;
			scale_factor_ks  = pushWire;
			scale_factor_kn  = pullWire;
	}
	else{
			scale_factor_kw  = pushWire;
			scale_factor_ke  = pullWire;
			scale_factor_ks  = pushWire;
			scale_factor_kn  = pullWire;
	}
	

	wrLengthWest  = scale_factor_kw * 2 * r * NumJ * (cos(alpha) - cos(alpha - pAngle / 2) + scale_factor_p3 * (1 - cos(tAngle / 2)));
	wrLengthEast  = scale_factor_ke * 2 * r * NumJ * (cos(alpha) - cos(alpha + pAngle / 2) + scale_factor_p3 * (1 - cos(tAngle / 2)));
	wrLengthSouth = scale_factor_ks * 2 * r * NumJ * (cos(alpha) - cos(alpha - tAngle / 2) + scale_factor_p3 * (1 - cos(pAngle / 2)));
	wrLengthNorth = scale_factor_kn * 2 * r * NumJ * (cos(alpha) - cos(alpha + tAngle / 2) + scale_factor_p3 * (1 - cos(pAngle / 2)));

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