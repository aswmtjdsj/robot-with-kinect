#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <deque>
#include <thread>
#include "asio.hpp"
#include "chat_message.hpp"

using namespace std;

using asio::ip::tcp;

typedef std::deque<chat_message> chat_message_queue;

class Robot {

	public:
		Robot(ros::NodeHandle &nh) {
			nh_ = nh;
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);//"/base_controller/command", 1);
		}

		bool drive(const string & cmd) {
			cout << "[accepted] " << cmd << endl;
			geometry_msgs::Twist base_cmd;
			if(cmd.substr(0, 7) == "forward") {
				base_cmd.linear.x = 0.25;
				cout << "forward" << endl;
			} else if(cmd.substr(0, 4) == "left") {
				base_cmd.linear.x = 0.25;
				base_cmd.angular.z = 0.75;
				cout << "turn left" << endl;
			} else if(cmd.substr(0, 5) == "right") {
				base_cmd.linear.x = 0.25;
				base_cmd.angular.z = -0.75;
				cout << "turn right" << endl;
			} else if(cmd.substr(0, 4) == "stop") {
				cout << "stop" << endl;
			} else {
				cout << "unknown command" << endl;
				return false;
			}
			cmd_vel_pub_.publish(base_cmd);
			return true;
		}

		bool driveKeyboard(int argc, char ** argv) {
			cout << "'+' for forward, 'l' for left, 'r' for right, '.' for stop" << endl;
			geometry_msgs::Twist base_cmd;
			char cmd[50];
			while(nh_.ok()) {
				cin.getline(cmd, 50);
				if(cmd[0] != '+' && cmd[0] != 'l' && cmd[0] != 'r' && cmd[0] != '.') {
					cout << "unknown command" << endl;
					continue;
				}
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

				if(cmd[0] == '+') {
					base_cmd.linear.x = 0.25;
					cout << "forward" << endl;
				} else if(cmd[0] == 'l') {
					base_cmd.linear.x = 0.25;
					base_cmd.angular.z = 0.75;
					cout << "turn left" << endl;
				} else if(cmd[0] == 'r') {
					base_cmd.linear.x = 0.25;
					base_cmd.angular.z = -0.75;
					cout << "turn right" << endl;
				} else if(cmd[0] == '.') {
					cout << "stop" << endl;
					break;
				}
				cmd_vel_pub_.publish(base_cmd);
			}
			return true;
		}

	private:
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;

};


class chat_client
{
	public:
		chat_client(asio::io_service& io_service,
				tcp::resolver::iterator endpoint_iterator,
				Robot & robot)
			: io_service_(io_service),
			socket_(io_service),
			robot_(robot),
			start(0)
	{
		do_connect(endpoint_iterator);
	}

		void write(const chat_message& msg)
		{
			io_service_.post(
					[this, msg]()
					{
					bool write_in_progress = !write_msgs_.empty();
					write_msgs_.push_back(msg);
					if (!write_in_progress)
					{
					do_write();
					}
					});
		}

		void close()
		{
			io_service_.post([this]() { socket_.close(); });
		}

	private:
		void do_connect(tcp::resolver::iterator endpoint_iterator)
		{
			asio::async_connect(socket_, endpoint_iterator,
					[this](std::error_code ec, tcp::resolver::iterator)
					{
					if (!ec)
					{
					do_read_header();
					}
					});
		}

		void do_read_header()
		{
			asio::async_read(socket_,
					asio::buffer(read_msg_.data(), chat_message::header_length),
					[this](std::error_code ec, std::size_t /*length*/)
					{
					if (!ec && read_msg_.decode_header())
					{
					do_read_body();
					}
					else
					{
					socket_.close();
					}
					});
		}

		void do_read_body()
		{
			asio::async_read(socket_,
					asio::buffer(read_msg_.body(), read_msg_.body_length()),
					[this](std::error_code ec, std::size_t /*length*/)
					{
					if (!ec)
					{
					if(strncmp(read_msg_.body(), "[kinect]", strlen("[kinect]")) == 0) {
						string msg_s = read_msg_.body();
						msg_s = msg_s.substr(strlen("[kinect] "));
						// std::cout.write(read_msg_.body(), read_msg_.body_length());
						// std::cout << "\n";
						std::cout << "[real command] " << msg_s << endl;
						if(msg_s.substr(0, 6) == "button") {
							cout << "button pushed" << endl;
							if(start == 0) { // initialize 
								cout << "Init button pushed!" << endl;
								start = 1;
							} else if(start == 1) { // nothing done already
								cout << "Just initialized! Won't do init again!" << endl;
								// LOL
							} else { // start == 2, something has done
								cout << "Something has been done! Can rest now!" << endl;
								start = 0;
							}
						}
						else if(start != 0) {
							cout << "command accepted" << endl;
							start = 2; // something done!
							robot_.drive(msg_s);
						}
						else { // start == 0
							cout << "need button first!" << endl;
						}
						chat_message msg;
						char msg_to_send_[chat_message::max_body_length] = "[kabuki] kinect message received";
						/*msg.body_length(std::strlen(msg_to_send_));
						std::memcpy(msg.body(), msg_to_send_, msg.body_length());
						msg.encode_header();*/
						// write(msg);
					} else if(strncmp(read_msg_.body(), "[kabuki]", strlen("[kabuki]")) == 0) {
						std::cout << "[success] kabuki message sent";
						std::cout << "\n";
					} else {
						std::cout << "[error] message " << "\"";
						// std::cout.write(read_msg_.body(), read_msg_.body_length());
						std::cout << "\" not start with [kinect], discard";
						std::cout << "\n";
					}
					do_read_header();
					}
					else
					{
						socket_.close();
					}
					});
		}

		void do_write()
		{
			asio::async_write(socket_,
					asio::buffer(write_msgs_.front().data(),
						write_msgs_.front().length()),
					[this](std::error_code ec, std::size_t /*length*/)
					{
					//socket_.close();
					// return ;
					if (!ec)
					{
					write_msgs_.pop_front();
					if (!write_msgs_.empty())
					{
					do_write();
					}
					}
					else
					{
					socket_.close();
					}
					});
		}

	private:
		asio::io_service& io_service_;
		tcp::socket socket_;
		chat_message read_msg_;
		chat_message_queue write_msgs_;
	public:

		Robot robot_;
		int start;
};

int main(int argc, char ** argv) {
	try
	{
		/*if (argc != 3)
		  {
		  std::cerr << "Usage: chat_client <host> <port>\n";
		  return 1;
		  }*/
		ros::init(argc, argv, "robot");
		ros::NodeHandle nh;
		Robot driver(nh);
	//	driver.driveKeyboard(argc, argv);
		std::cout << "kobuki controller!" << endl;
		const char * host = "localhost", * port = "8888";

		asio::io_service io_service;

		tcp::resolver resolver(io_service);
		// host = argv[1], port = argv[2];
		auto endpoint_iterator = resolver.resolve({ host, port });
		chat_client c(io_service, endpoint_iterator, driver);

		std::thread t([&io_service](){ io_service.run(); });

		/*  char line[chat_message::max_body_length + 1];
		  while (std::cin.getline(line, chat_message::max_body_length + 1))
		  {
		  chat_message msg;
		  msg.body_length(std::strlen(line));
		  std::memcpy(msg.body(), line, msg.body_length());
		  msg.encode_header();
		  c.write(msg);
		  }*/
		io_service.run();

		c.close();
		t.join();
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}
