#include "../../mobileman_navigation/include/mobileman_navigation/system_log.hpp"

#include "../include/common/serialDevice.h"
struct speed
{
	float x;
	float y;
	float z;
};
int main(int argc, char* argv[])
{
	std::string device_port ("");
	std::string message ("");
	if (argc <=2 )
	{
		std::cout << "Usage: /common_loger device_port message" <<std::endl;
		std::cout << "/dev/SBF is used for default device (sllf lifting platform)" << std::endl;
		std::cout << "AS100 is used for message" <<std::endl;

		device_port = "/dev/SBF";
		message = "AS100";
	
	}
	else 
	{
		device_port.append(std::string(argv[1]));
		message.append(std::string(argv[2]));

	}




	mobileman::SerialDevice selfLiftPlatform (device_port);

	std::cout<<"Response is:: " << selfLiftPlatform.sendData(message) <<std::endl;
	
	return 0;
}
