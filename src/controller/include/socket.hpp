// socket header
 
#ifndef SOCKET_HPP
#define SOCKET_HPP

#include <string>
#include <netinet/in.h> // sockaddr_in
#include <sys/socket.h> // socket, bind, recvfrom
#include <arpa/inet.h>  // inet_addr
#include <unistd.h>     // close

#define SYSTEM_STATE_MAX_LENGTH 1024

 struct SocketConfig {
 	int domain = AF_INET;
 	int type = SOCK_DGRAM;
 	int protocol = IPPROTO_UDP;
 	const std::string dest_address = "192.168.10.254";
 	const std::string local_address = "0.0.0.0";
 	int local_port = 0;
 	int dest_port = 51218;
 	int timeout_sec = 2;
 	int timeout_usec = 0;
};

class Socket
{
	public:

		Socket(SocketConfig config_);//constructor : Automatically called when an object is created
		void bind();
		void connect();
		ssize_t send(const std::string& msg); //ssize_t permet de renvoyer la taille du message ou -1 pour savoir si il y a une erreur
		ssize_t receive(std::string& msg, size_t max_length = SYSTEM_STATE_MAX_LENGTH,int timeout_sec=2); //ssize_t permet de renvoyer la taille du message ou -1 pour savoir si il y a une erreur
		void close();

	private :

	int socket_fd_; //File Descriptor du socket (un entier retrouné par ::socket dans les autres fonctions)
	sockaddr_in remote_addr_; //Représentation système réutilisé dans les autres fonctions. Evite d'avoir à repasser par SocketConfig.
	SocketConfig config_;
	bool is_connected_ = false;
	bool is_bound_= false;
	bool is_sent_=false;
	bool is_received_=false;
};
#endif