#ifndef CHAT_HPP
#define CHAT_HPP

#include "common.h"
#include "message.hpp"

using namespace std;
using asio::ip::tcp;

typedef std::deque<chat_message> chat_message_queue;

class chat_client
{
public:
	chat_client(asio::io_service& io_service,
		tcp::resolver::iterator endpoint_iterator)
		: io_service_(io_service),
		socket_(io_service)
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
				if (strncmp(read_msg_.body(), "[kabuki]", strlen("[kabuki]")) == 0) {
					std::cout << "[success] kabuki message received";
					std::cout << "\n";
				}
				else if (strncmp(read_msg_.body(), "[kinect]", strlen("[kinect]")) == 0) {
					std::cout << "[success] kinect message sent";
					std::cout << "\n";
				}
				else {
					std::cout << "[error] message " << "\"";
					std::cout.write(read_msg_.body(), read_msg_.body_length());
					std::cout << "\" not start with [kabuki], discard";
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
			// cout << "to_send" << endl;
			if (!ec)
			{
				write_msgs_.pop_front();
				if (!write_msgs_.empty())
				{
					// cout << "sent" << endl;
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
};

#endif
