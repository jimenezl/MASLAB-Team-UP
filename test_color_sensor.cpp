// Compile with:
// g++ test_color_sensor.cpp -o test_color_sensor -lmraa
// Repeatedly reads pin A0 and prints the result.

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
    

    /*if (val > 315){
      printf("Green Light");
    }

    else { 
      printf("Red Light");
    }
    */
    sleep(1);
  }
}