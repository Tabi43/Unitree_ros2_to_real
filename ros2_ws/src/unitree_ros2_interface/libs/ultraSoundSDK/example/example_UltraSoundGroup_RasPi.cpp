#include "UltraSound_UART.h"
#include <iostream>
#include <unistd.h>

int main(){
	std::vector<uint8_t> ids;
	//ids.push_back(0);
	ids.push_back(1);
	ids.push_back(2);
	//ids.push_back(3);

	UltraSoundGroup ug(ids);

	std::vector<double> distance;

	long long time;

	while(true){
		time = getSystemTime();
		distance = ug.getDistance();
		std::cout << "cost time: " << getSystemTime() - time << std::endl;

		for(int i(0); i<distance.size(); ++i){
			std::cout << distance[i] << "   ";
		}
		std::cout << std::endl;
		usleep(200000);
	}
}
