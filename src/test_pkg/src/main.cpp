#include <iostream>
#include <stdio.h>
#define DEFAULT_TCP_BUFFER_SIZE 10 * 2 * static_cast<int>(sizeof(uint32_t))

int main()
{
  std::cout << "Hi, test DY" << std::endl;
  std::cout << sizeof(uint32_t)  << std::endl;
  std::cout << DEFAULT_TCP_BUFFER_SIZE  << std::endl;
  return 0; 
}