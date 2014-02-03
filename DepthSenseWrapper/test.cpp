#include "DepthSense.h"
using namespace DepthSense;

#include <iostream>
using namespace std;

int main() {
	DepthSenseCapture capture;

	while (true) {
		capture.grab();
		DepthSenseImage i = capture.retrieve();
		cout << i.width << "-" << i.height << "\n";
	}

	return 0;
}