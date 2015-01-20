// Compile with:
// g++ test_infrared_lerner.cpp -o test_infrared_lerner -lmraa
// Repeatedly reads pin A2 and prints the result.

#include <csignal>
#include <iostream>

#include "mraa.hpp"

int running = 1;

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    printf("closing spi nicely\n");
    running = 0;
  }
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  mraa::Aio aio = mraa::Aio(2);

  while (running) {
    int val = aio.read();
    std::cout << "Read: " << val << std::endl;
    sleep(1);
  }
}