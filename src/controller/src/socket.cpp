// Socket functions
// About errors : errno is a global variable used in system functions as bind(), connect()..etc. When these functions fails, they return -1 and return a errno with a certain value
// strerror(errno) gives the attached string to the code errno given. It describes the issue.

#include "socket.hpp"
#include <stdexcept>
#include <arpa/inet.h>  // inet_pton
#include <unistd.h> // close()
#include <iostream>
#include <cstring>      // strerror
#include <sys/socket.h>  // ::shutdown()
#include <vector>

Socket::Socket(SocketConfig config_)
{
	socket_fd_ = ::socket(config_.domain, config_.type, config_.protocol);
	if(socket_fd_ < 0)
	{
		throw std::runtime_error("Socket creation failed : " + std::string(strerror(errno)));
	}

	//partie pour configuration du timeout pour la réception (Socket::receive)
	// timeval est un struct système, tv_sec correspond à l'interval de temps en secondes et tv_usec en microsecondes.
	struct timeval timeout;
	timeout.tv_sec = config_.timeout_sec;
	timeout.tv_usec = 0;

    //fonction setsockopt est une fonction système qui permet de définir différentes options au niveau d'une socket créée, ici on attribue l'option S0_RCVTIMEO
    // qui définir le déalis d'attentes pour le blocage des appels de réception
    if(setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout))<0)
    {
    	throw std::runtime_error("Failed to set socket timeout : " + std::string(strerror(errno)));
    }
}



void Socket::bind()
{
	// This function is used to link my machine's local ip & port to my socket
	// inet_pton : used to convert IP format adress into binary
	// ::bind to differantiate with the system bind function and the one I m creating for my class Socket
	// reinterpret_cast : used for conversion as the bind function requires the system struc sockaddr but we want to give it an other struct, sockadrr_in
	// htons : used to convert the port in network format
	// std : stoi() : used to convert string in int

	// 1 : convert input ip & port into struct sockaddr_in and verify the IP adress
	sockaddr_in local_addr{};
	local_addr.sin_family = config_.domain;
	local_addr.sin_port = htons(config_.local_port); //SocketConfig.local_port est un int)

    if(inet_pton(config_.domain, config_.local_address.c_str(), &local_addr.sin_addr) <=0)
    {
    	throw std::invalid_argument("invalid local IP adress : " + config_.local_address);
    }
    

	// 2 : call the bind fonction on the contructor from the class and Throw an error if exception occurs
	if(::bind(socket_fd_,reinterpret_cast<sockaddr*>(&local_addr),sizeof(local_addr)) <0)
	{
		throw std::runtime_error("Socket bind failed: " + std::string(strerror(errno)));
	}

	is_bound_ = true;

}


void Socket::connect()
{
	// This function is not completely necessary for an UDP connection as there is no need for "handshake" in UDP. However, doing this steps will allow us to
	// use directly native send() and recv() native functions and not sendto() and receivfrom() that requires each time to pass destinations arguments
	sockaddr_in dest_addr{};
    dest_addr.sin_family = config_.domain;
    dest_addr.sin_port = htons(config_.dest_port);

    if(inet_pton(config_.domain, config_.dest_address.c_str(), &dest_addr.sin_addr) <=0)
    {
    	throw std::invalid_argument("invalid destination IP adress : " + config_.dest_address);
    }

    if(::connect(socket_fd_,reinterpret_cast<sockaddr*>(&dest_addr),sizeof(dest_addr)) <0)
	{
		throw std::runtime_error("UDP socket connection failed : " + std::string(strerror(errno)));
	}

	is_connected_ = true;

}


ssize_t Socket::send(const std::string& msg) //ssize_t permet de renvoyer la taille du message ou -1 pour savoir si il y a une erreur
{

    const ssize_t result = ::send(socket_fd_,msg.c_str(),msg.length(),0);

    if( result <0)
    {
        throw std::runtime_error("Failed to send message : " + std::string(strerror(errno)));
    }

    is_sent_ = true;
    is_received_ = false;
    return result;
}

ssize_t Socket::receive(std::string& msg, size_t max_length, int timeout_sec) //ssize_t permet de renvoyer la taille du message ou -1 pour savoir si il y a une erreur
{
	std::vector<char> buffer(max_length,0);  // temporary buffer for message reception
    // The recv() function shall return the length of the message written to the buffer pointed to by the buffer argument.
	ssize_t result= ::recv(socket_fd_,buffer.data(),buffer.size(),0);
	
    if(result<0)
    {
    	if(errno == EWOULDBLOCK || errno == EAGAIN)
    	{
    		// [EAGAIN] or [EWOULDBLOCK] : The socket is marked nonblocking and the receive operation would block, or a receive timeout had been set and the
        	// timeout expired before data was received.  POSIX.1 allows either error to be returned for this case, and does not
        	// require these constants to have the same value, so a portable application should check for both possibilities.
    		throw std::runtime_error("Received timeout");
    	} 
    	else 
    	{
    		throw std::runtime_error("Receive failed: " + std::string(strerror(errno)));
    	}
    }
    
	msg.assign(buffer.data(),result);// convert the buffer into a std::string
	is_sent_ = false;
	is_received_ = true;

	return result;
}

void Socket::close()
{
	// if a socket is still valid
	if(socket_fd_ != -1)
	{
		if (::close(socket_fd_) < 0)
		{
			// here we don't use throw as before because it's not a critical error
			// note : throw is for critical errors that prevent the rest of the program to work. It will interrupt the programm
			// std::cerr is used for informatives errors as a way to print/log an alert to the user but that will not block the next steps of the programm
			// Here, if the close doesn't work it is not so important as we will close the program anyway after that
			std::cerr << "Error: failed to close socket: " << strerror(errno) << std::endl;
		}
		socket_fd_ = -1;
	}
	// internal states reset
	is_connected_ = false;
	is_bound_ = false;
	is_sent_ = false;
	is_received_ = false;
}

