#include "common.h"
#include "chat.hpp"
#include "kinect_sensor.hpp"
#include "gesture.hpp"

using namespace std;

int main(int argc, char* argv[])
{
	try
	{
		/*if (argc != 3)
		{
			std::cerr << "Usage: chat_client <host> <port>\n";
			return 1;
		}*/

		asio::io_service io_service;

		tcp::resolver resolver(io_service);
		const char * host = "10.0.0.10", *port = "8888";
		// auto endpoint_iterator = resolver.resolve({ argv[1], argv[2] });
		auto endpoint_iterator = resolver.resolve({ host, port });
		chat_client c(io_service, endpoint_iterator);

		std::thread t([&io_service](){ io_service.run(); });

		char line[chat_message::max_body_length + 1];
		// io_service.run();
		kinectSensor(c);
		/*while (std::cin.getline(line, chat_message::max_body_length + 1))
		{
			chat_message msg;
			msg.body_length(std::strlen(line));
			std::memcpy(msg.body(), line, msg.body_length());
			msg.encode_header();
			c.write(msg);
		}*/

		c.close();
		t.join();
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}
