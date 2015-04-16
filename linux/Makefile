CC=g++
CFLAGS=--std=c++0x -DASIO_STANDALONE -pthread
INC=-I/home/parlinyu/repo/asio-1.10.6/include
EXEC=chat_server chat_client

chat_client:chat_client.cpp
	$(CC) $(CFLAGS) $(INC) $^ -o $@

chat_server:chat_server.cpp
	$(CC) $(CFLAGS) $(INC) $^ -o $@

all: $(EXEC)

clean:
	-rm $(EXEC) 
