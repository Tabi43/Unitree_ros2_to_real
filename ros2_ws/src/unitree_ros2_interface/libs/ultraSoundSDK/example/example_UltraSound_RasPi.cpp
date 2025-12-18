#include "UltraSound_UART.h"
#include <iostream>
#include <unistd.h>

int main(){
	//UltraSound ul0(0);
	UltraSound ul1(1);
	UltraSound ul2(2);
	//UltraSound ul3(3);
	double dis;

	while(true){
		std::cout << "loop runs" << std::endl;
		/*if(ul0.measureDistance(dis)){
			std::cout << "ul0 Distance: " << dis << std::endl;
		}*/
		if(ul1.measureDistance(dis)){
			std::cout << "ul1 Distance: " << dis << std::endl;
		}
		if(ul2.measureDistance(dis)){
			std::cout << "ul2 Distance: " << dis << std::endl;
		}
		/*if(ul3.measureDistance(dis)){
			std::cout << "ul3 Distance: " << dis << std::endl;
		}*/
		usleep(200000);
	    }
}
