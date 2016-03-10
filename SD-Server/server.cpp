#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <utils/traci/TraCIAPI.h>
#include "json.hpp"

// SUMO
class TraCIClient : public TraCIAPI {
	public:
		TraCIClient();
};

TraCIClient::TraCIClient() {
	try {
		connect("localhost", 2001);
	} catch(tcpip::SocketException &e) {
		printf("tcpip: %s\n", e.what());
	}
}

// for convenience
using json = nlohmann::json;

#define PORT 2000
#define BUFFER_SIZE 1024

void sigHandler(int);

int serverSocket;

int main(int argc , char *argv[]) {
	TraCIClient traciclnt;
	signal(SIGINT, sigHandler);

	int serverSocket, c, clientSocket;
	char buffer[BUFFER_SIZE];
	struct sockaddr_in serverAddr, clientAddr;

	serverSocket = socket(PF_INET, SOCK_STREAM, 0);

	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(PORT);
	serverAddr.sin_addr.s_addr = inet_addr("0.0.0.0");
	memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);

	if(bind(serverSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) == 0) {
		printf("Bound to port %d \n", ntohs(serverAddr.sin_port));
	} else {
		printf("Bind failed!\n");
		return 1;
	}

	if(listen(serverSocket, 5) == 0)
		printf("Listening\n");
	else
		printf("Error\n");

	c = sizeof(struct sockaddr_in);

	clientSocket = accept(serverSocket, (struct sockaddr *) &clientAddr, (socklen_t *) &c);
	puts("Connection accepted");


	while( recv(clientSocket, buffer, BUFFER_SIZE, 0) ) {
		printf("recv: %s\n", buffer);
		json data = json::parse(buffer);
		for (json::iterator it = data.begin(); it != data.end(); ++it) {
			std::cout << it.key() << " : " << it.value() << "\n";
		}
    traciclnt.vehicle.moveTo("veh0", "route0", data["veh0"]["pos"]);
		memset(buffer, 0, sizeof(buffer));
	}

	return 0;
}

void sigHandler(int signum) {
	printf("Caught signal %d...\n", signum);
	close(serverSocket);
	exit(1);
}
