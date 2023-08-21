#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#pragma comment(lib, "ws2_32")
//#include <WinSock2.h>
#include <WS2tcpip.h>

#include <windows.h>
#include <math.h>
#include <string.h>
#include <string>
#include <time.h>
#include <chrono>
#include <thread>

// #define IP_ADDRESS "127.0.0.1"
#define IP_ADDRESS "172.16.1.0"
#define PORT_NUMBER 7777

#define TCP_BUFFER_SIZE 512
#define QUEUE_FREQUENCY 1000    // Hz
#define QUEUE_TIME      1       // ms
#define TEST_RECV       1
#define TEST_SEND       1

#define NUM_OF_MOTORS 5

#define ACTIVE_UR_ROBOT_PROTOCOL 0

void ErrorHandling(const char* _Message);

int main(int argc, char* argv[])
{
    /***
     * @brief TCP client open part...
     * ===========================================================================================
    */
    WSADATA wsaData;
    SOCKET hSocket;
    SOCKADDR_IN serverAddress;

    /* setting IP, Port - start */
    std::string ip;
    std::string s_port;
    uint32_t i_port = PORT_NUMBER;
    int strlen;
    char message[] = "";

    while(true){
        std::cout << "[Command] Enter IP address. Enter is using dafault (default = 127.0.0.1) : ";
        std::getline(std::cin, ip);
        static uint8_t cnt = 0;
        
        // set default
        if(ip == "") {ip = IP_ADDRESS; std::cout << ip << std::endl; break;}
        // set manual
        for(int i=0; i<ip.length(); i++) { if(ip[i] == '.') cnt++; }
        if(cnt != 3) std::cout << "[ERROR] Check your ip address (3 .)" << std::endl;
        else {break;}
    }
    while(true){
        std::cout << "[Command] Enter Port. Enter is using dafault (default = " << PORT_NUMBER << ") : ";
        std::getline(std::cin, s_port);
        uint8_t tf = 0;

        // set default
        if(s_port=="") {std::cout << i_port << std::endl; break;}
        // set manual
        for(int i=0; i<s_port.length(); i++)
        {
            tf = isdigit(s_port[i]);
            if(tf==0)
            {
                std::cout << "[ERROR] Check your Port Number (int)" << std::endl;
                break;
            }
        }
        if(tf) 
        {   
            i_port = std::stoi(s_port);
            break;
        }
    }
    std::cout << "[INFO] Set is done -> " << "IP = " << ip << " port = " << i_port << std::endl;
    std::cout << ip.length() << std::endl;
    /* setting IP, Port - finish */

    // if(ip.size!=)
    // initial WinSock.
    if(WSAStartup(MAKEWORD(2,2), &wsaData) != 0) std::cout << "[ERROR] WSAStartup() error" << std::endl;

    hSocket = socket(PF_INET, SOCK_STREAM, 0);
    if(hSocket == INVALID_SOCKET) std::cout << "[ERROR] hSocket() error" << std::endl;

    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = inet_addr(ip.c_str());
    //inet_pton(AF_INET, ip.c_str(), &serverAddress.sin_addr.s_addr);
    serverAddress.sin_port = htons(i_port);
    std::cout << "[INFO] connecting to server...";
    if(connect(hSocket, (SOCKADDR*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) std::cout << "[ERROR] connect() error" << std::endl;
    else std::cout << "done" << std::endl;
    
    // TCP client open part end
    // ===========================================================================================


    static char recvMsg[TCP_BUFFER_SIZE] = {0};
    long recv_val[TCP_BUFFER_SIZE];
    long send_val[TCP_BUFFER_SIZE];
    char sendMsg[TCP_BUFFER_SIZE];
    static double joints_vel[7];
    static uint64_t counter = 0;
    while(true)
    {   
        #if TEST_RECV
        strlen = recv(hSocket, recvMsg, TCP_BUFFER_SIZE, 0);
        if(strlen == -1) {
            std::cout << "TCP/IP disconnected... read() error" << std::endl;
            break;
        }

        // Little-Endian
        memcpy(recv_val, recvMsg, TCP_BUFFER_SIZE);
        system("cls");
        for(int n=0; n<NUM_OF_MOTORS; n++){
            std::cout << n << " pos : " << recv_val[0+n*2] << " / vel : " << recv_val[1+n*2] << " - " << counter << std::endl;
        }
        #endif

        #if TEST_SEND

        for(int i=0; i<NUM_OF_MOTORS; i++){
            send_val[i] = 10*sin(counter*0.01);
            memcpy(sendMsg + i*sizeof(long), &send_val[i], sizeof(send_val[i]));
        }
        uint32_t size_ofsendMsg = send(hSocket, sendMsg, TCP_BUFFER_SIZE, 0);

        /*
         * if use one-by-one telegram
        */
        // std::cout << "[Command] Enter the Send message : ";
        // std::getline(std::cin, sendMsg);
        // uint32_t size_of_sendMsg = send(hSocket, sendMsg.c_str(), TCP_BUFFER_SIZE, 0);

        #endif

        #if ACTIVE_UR_ROBOT_PROTOCOL
        sendMsg = "speedj";
        sendMsg += "([";

        for(int i=0; i<7; i++)
        {
            joints_vel[i] = 100*sin(counter * 0.001);
            sendMsg += std::to_string(joints_vel[i]);
            sendMsg += ",";
        }
        sendMsg.pop_back();
        sendMsg += "])";
        sendMsg = send(hSocket, sendMsg.c_str(), TCP_BUFFER_SIZE, 0);
        #endif

        counter++;
        // std::this_thread::sleep_for(std::chrono::milliseconds(QUEUE_TIME));
    }

    send(hSocket, message, sizeof(message), 0);
    closesocket(hSocket);
    WSACleanup();
    return 0;
}

void ErrorHandling(const char* _Message)
{
    fputs(_Message, stderr);
    fputc('\n', stderr);
    exit(1);
}
