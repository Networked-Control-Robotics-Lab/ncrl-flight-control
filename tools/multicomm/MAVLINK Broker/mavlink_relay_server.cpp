/* 
 * When number of GCS and vehicle is N, M respectively,
 * this server relay packet to eachother.
 */
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>
#include <algorithm>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <common/mavlink.h>

#define DEFAULT_TCP_PORT 20250
#define DEFAULT_UDP_PORT 20200

void error_handling(const char *message);
uint8_t extract_target_id(uint8_t* buf, ssize_t buf_size);
void vehicle2gcs(int sock, uint8_t* buf, ssize_t buf_size);
void gcs2vehicle(int sock, uint8_t* buf, ssize_t buf_size);

int udp_sock;
sockaddr_in udp_clnt_adr;
socklen_t udp_clnt_adr_sz;

uint16_t tcp_port;	/* connect with GCSs */
uint16_t udp_port;	/* connect with vehicles */

/* save mavlink's system id of vehicle with udp socket address */
std::map<uint8_t, sockaddr_in> uav_info;

/* 
 * List of GCS connected with this server.
 * Packet of vehicles are routed to these GCSs.
 */
std::vector<int> gcs_socket_fd_list;

int main(int argc, char *argv[])
{
	if(argc > 3){
		printf("usage : \n");
		printf("mavlink_relay_server {TCP_PORT=20250} {UDP_PORT=20200}\n");
		return 0;
	}
	else if(argc == 3){
		tcp_port = atoi(argv[1]);
		udp_port = atoi(argv[2]);
	}
	else if(argc == 2){
		tcp_port = atoi(argv[1]);
		udp_port = DEFAULT_UDP_PORT;
	}
	else{
		tcp_port = DEFAULT_TCP_PORT;
		udp_port = DEFAULT_UDP_PORT;
	}

	printf(" TCP port (for GCS): %u\n", tcp_port);
	printf(" UDP port (for vehicle) : %u\n", udp_port);

	int fd_max = 0;

	/* TCP socket wating packets from GCSs. */
	int tcp_sock;
	sockaddr_in tcp_adr;
	tcp_sock = socket(PF_INET, SOCK_STREAM, 0);
	if(tcp_sock == -1)
		error_handling("TCP socket creation error");
	memset(&tcp_adr, 0, sizeof(tcp_adr));
    tcp_adr.sin_family=AF_INET;
    tcp_adr.sin_addr.s_addr=htonl(INADDR_ANY);
    tcp_adr.sin_port=htons(tcp_port);
    if(bind(tcp_sock, (struct sockaddr*) &tcp_adr, sizeof(tcp_adr))==-1)
        error_handling("bind() error");
    if(listen(tcp_sock, 3)==-1)
        error_handling("listen() error");

	fd_set reads;
	FD_ZERO(&reads);
    FD_SET(tcp_sock, &reads);
	fd_max = tcp_sock;
	
	/* UDP socket wating packets from vehicles */
	sockaddr_in udp_adr;
	udp_sock = socket(PF_INET, SOCK_DGRAM, 0);
	if(udp_sock==-1)
		error_handling("UDP socket creation error");
	
	memset(&udp_adr, 0, sizeof(udp_adr));
	udp_adr.sin_family=AF_INET;
	udp_adr.sin_addr.s_addr=htonl(INADDR_ANY);
	udp_adr.sin_port=htons(udp_port);
	if(bind(udp_sock, (struct sockaddr*)&udp_adr, sizeof(udp_adr))==-1)
		error_handling("bind() error");
	
    FD_SET(udp_sock, &reads);
    fd_max = udp_sock;

	
	/* TCP and UDP client initialize */
	int tcp_clnt_sock;
	sockaddr_in tcp_clnt_adr;
	socklen_t tcp_clnt_adr_sz = sizeof(tcp_clnt_adr);

	int udp_clnt_sock;
	udp_clnt_adr_sz = sizeof(udp_clnt_adr);

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* variable for iteration */
	fd_set cpy_reads;
	timeval timeout;
	int fd_num;

	while(true) 
	{
		cpy_reads = reads;
		timeout.tv_sec = 0;
		timeout.tv_usec = 0;
		if((fd_num=select(fd_max+1, &cpy_reads, 0, 0, &timeout))==-1)
			break;
		
		if(fd_num==0)
			continue;
		
		/* scan socket connected to this server */
		for(int i=0; i<fd_max+1; i++)
		{
			/* when socket is updated */
			if(FD_ISSET(i, &cpy_reads))
			{
				/* Client(GCS) request server to connect */
				if(i == tcp_sock)
				{
					tcp_clnt_sock=
						accept(tcp_sock, (struct sockaddr*)&tcp_clnt_adr, &tcp_clnt_adr_sz);
					if(tcp_clnt_sock == -1){
						fputs("making connection fail\n", stderr);
						continue;
					}
					gcs_socket_fd_list.push_back(tcp_clnt_sock);
					FD_SET(tcp_clnt_sock, &reads);
					if(fd_max < tcp_clnt_sock)
						fd_max = tcp_clnt_sock;
					printf("GCS is connected : %d \n", tcp_clnt_sock);
				}
				/* packet from vehicle (UDP socket) */
				else if(i == udp_sock)
				{
					//udp recvfrom
					ssize_t n = recvfrom(udp_sock, buf, sizeof(buf), 0, 
											(struct sockaddr*)&udp_clnt_adr, &udp_clnt_adr_sz);
					if(n == -1){
						fputs("udp recvfrom fail\n", stderr);
						continue;
					}
					vehicle2gcs(i, buf, n);
				}
				/* packet from GCS (TCP socket) */
				else
				{
					//tcp read
					ssize_t n = read(i, buf, sizeof(buf));
					/* GCS request server to terminate connection */
					if(n == 0){
						std::vector<int>::iterator iter;
						iter = std::find(gcs_socket_fd_list.begin(), gcs_socket_fd_list.end(), i);
						gcs_socket_fd_list.erase(iter);
						FD_CLR(i, &reads);
						close(i);
						printf("GCS is terminated : %d \n", i);
					}
					/* routing packet to vehicle */
					else{
						gcs2vehicle(i, buf, n);
					}
				}
			}
		}
	}	
	close(tcp_sock);
	close(udp_sock);
	return 0;
}

/*
 * print error status to user.
 * \param	message		error name
 */
void error_handling(const char *message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

/*
 * extract target id from buffer received through socket.
 * \param	buf			buffer received through socket
 * \param	buf_size 	size of buffer
 * \return				target id of this packet
 */
uint8_t extract_target_id(uint8_t* buf, ssize_t buf_size)
{
	mavlink_message_t msg;
	mavlink_status_t status;
	for (int i = 0; i < buf_size; i++) {
		if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
			// handle_new_message(&msg);
			break;
		}
	}
	uint8_t* ptr = (uint8_t*)&msg.payload64;
	const mavlink_msg_entry_t* payload_ofs;
	payload_ofs = mavlink_get_msg_entry(msg.msgid);
	if(payload_ofs->flags != MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)
	{
		return 0;
	}
	// printf("payload offset : %d\n", payload_ofs->target_system_ofs);
	uint8_t target_id = *(ptr+payload_ofs->target_system_ofs);
	// printf("target id : %d\n", target_id);
	return target_id;
}


/*
 * \param	sock		fd of socket
 * \param	buf			buffer received from vehicle
 * \param	buf_size	size of buffer
 */
void vehicle2gcs(int sock, uint8_t* buf, ssize_t buf_size)
{
	mavlink_message_t msg;
	mavlink_status_t status;
	/* parse buf to mavlink message */
	for(int i = 0; i < buf_size; i++) {
		if(mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
			// handle_new_message(&msg);
			break;
		}
	}
	
	/* 
	 * save socket address of this vehicle 
	 * (this data are used for routing GCS's packet to vehicle) 
	 */
	uav_info[msg.sysid] = udp_clnt_adr;

	/* 
	 * Route vehicle's packet to all of GCS connected this server. 
	 */
	for(auto iter = gcs_socket_fd_list.begin(); iter < gcs_socket_fd_list.end(); iter++){
		if(write(*iter, buf, buf_size) == -1){
			fputc(*iter, stderr);
			fputs(" : tcp write fail\n", stderr);
			continue;
		}
	}
}

/*
 * \param 	sock		fd of socket
 * \param	buf			buffer received from GCS
 * \param	buf_size	size of buffer
 */
void gcs2vehicle(int sock, uint8_t* buf, ssize_t buf_size)
{
	socklen_t udp_clnt_adr_sz = sizeof(sockaddr_in);

	/* extract target id from buffer */
	uint8_t target_id = extract_target_id(buf, buf_size);

	/* brodcast the packet when target id is 0 */
	if(target_id == 0){
		for(auto uav : uav_info)
		{
			if(sendto(udp_sock, buf, buf_size, 0, 
						(struct sockaddr*)&uav.second, udp_clnt_adr_sz) == -1){
				fputs("udp sendto fail\n", stderr);
				return;
			}
		}
	}
	/* unicasting packet */
	else{
		if(sendto(udp_sock, buf, buf_size, 0, 
					(struct sockaddr*)&uav_info[target_id], udp_clnt_adr_sz) == -1){
			fputs("udp sendto fail\n", stderr);
			return;
		}
		// printf("%d %d, %d\n", target_id, uav_info[target_id].sin_addr.s_addr, uav_info[target_id].sin_port);
	}
}