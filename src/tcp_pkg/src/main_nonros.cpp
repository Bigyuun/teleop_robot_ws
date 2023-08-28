#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <signal.h>

#include "tcp_node_nonros.hpp"

void signal_callback_handler (int signum) {
  signal(signum, SIG_IGN);
  printf("Ctrl+C break.\n");  
  exit(1);
}

int main(int argc, char * argv[])
{
  TCPClientNode tcp_PCtoMasterMACS;
  signal(SIGINT, signal_callback_handler);
  while(true)
  {
    /* spin */
  }
  return 0;
}
