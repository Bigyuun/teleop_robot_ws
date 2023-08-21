#include <stdio.h>
#include <stdlib.h>
#include <winsock2.h>
#include <iostream>
#include <string>

void ErrorHandling( char* message );
 
int main( int argc, char* argv[] )
{
    WSADATA     wsaData;
    SOCKET      hServSock, hClntSock;
    SOCKADDR_IN servAddr, clntAddr;
 
    short   port = 5001;
    int     szClntAddr;
    char    message[] = "Hello World!";
 
    // 윈속 초기화, 소켓사용하기전 초기화 해야함
    if( WSAStartup( MAKEWORD( 2, 2 ), &wsaData ) != 0 )
        ErrorHandling( "WSAStartup() error!" );
 
    // TCP 소켓생성
    hServSock = socket( PF_INET, SOCK_STREAM, 0 );
 
    if( hServSock == INVALID_SOCKET )
        ErrorHandling( "socket() error!" );
 
    // 소켓의 주소정보
    memset( &servAddr, 0, sizeof( servAddr ) );
 
    servAddr.sin_family         = AF_INET;
    servAddr.sin_addr.s_addr    = htonl( INADDR_ANY );  // 현재 PC IP주소 사용
    servAddr.sin_port           = htons( port );        // 포트번호
 
    // 주소 지정
    if( bind( hServSock, (SOCKADDR*)&servAddr, sizeof( servAddr ) ) == SOCKET_ERROR )
        ErrorHandling( "bind() error!" );
 
    // 접속 받을수 있게
    if( listen( hServSock, 5 ) == SOCKET_ERROR )    // 접속대기 5
        ErrorHandling( "listen() error!" );
 
    // 연결 수락
    szClntAddr  = sizeof( clntAddr );
    hClntSock   = accept( hServSock, (SOCKADDR*)&clntAddr, &szClntAddr );   // 클라이언트와 통신 할
 
    if( hClntSock == INVALID_SOCKET )
        ErrorHandling( "accept() error!" );
 
    send( hClntSock, message, sizeof( message ), 0 );
 
    closesocket( hClntSock );
    closesocket( hServSock );
    WSACleanup();
 
    return 0;
}
 
void ErrorHandling( char* message )
{
    fputs( message, stderr );
    fputc( '\n', stderr );
    exit( 1 );
}