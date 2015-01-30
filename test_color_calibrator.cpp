// Compile with:
// g++ test_color_calibrator.cpp -o test_color_calibrator -lmraa
// calibrate color sensor for red, green colors

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

  mraa::Aio aio = mraa::Aio(0);
  while (running) {
    int val = aio.read();
    std::cout << "Read: " << val << std::endl;
    usleep(1000*500);
    if (280 < val <= 680){
      printf("Red Block Found\n");
    }
    else if (val <= 280){
      printf("Green Block Found\n");
    }
    else if (val > 750){
      printf("No Block Found\n");
    }
  }
}