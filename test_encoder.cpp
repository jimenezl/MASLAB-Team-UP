// Compile with:
// g++ test_encoder.cpp -o test_encoder -lmraa
//
// Counts encoder ticks for a quadrature encoder. This is a *rough*
// implementation, in that it does not do any sort of locking or thread
// consistency on the assumption that encoder ticks will be coming in slowly
// enough to be serial. This is most likely a good assumption, but keep
// an eye out for those "weird phase transition" prints.
//
// Runs with channel A (yellow) on pin 5 and channel B (white) on pin 7.

#include <cassert>
#include <csignal>
#include <iostream>

#include "mraa.hpp"

int encoderAPin = 1;
int encoderBPin = 2;

int running = 1;

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    printf("closing spi nicely\n");
    running = 0;
  }
}

// Variables are volatile to ensure memory consistency between different
// edge callbacks.
volatile int aState;
volatile int bState;
volatile int encoderCount = 0;

int getPhase(int a, int b) {
  assert(a == 0 || a == 1);
  assert(b == 0 || b == 1);
  if (a == 0 && b == 0) {
    return 0;
  }
  else if (a == 1 && b == 0) {
    return 1;
  }
  else if (a == 1 && b == 1) {
    return 2;
  }
  else if (a == 0 && b == 1) {
    return 3;
  }
  // Unreachable
  assert(false);
}

void updateTick(int prevPhase, int curPhase) {
  // Tick forward (possibly wrapping)
  if (curPhase - prevPhase == 1 ||
      curPhase - prevPhase == -3) {
    encoderCount++;
  }
  // Tick backward (possibly wrapping)
  else if (curPhase - prevPhase == -1 ||
           curPhase - prevPhase == 3) {
    encoderCount--;
  }
  else {
    std::cerr << "Weird phase change: "
              << prevPhase << " to " << curPhase << std::endl;
  }
}

void aHandler(void* args) {
  // Get the gpio handle from the args
  mraa::Gpio *encA = (mraa::Gpio*)args;
  int a = aState;
  int b = bState;
  int newA = encA->read();
  aState = newA;
  int prevPhase = getPhase(a, b);
  int curPhase = getPhase(newA, b);
  updateTick(prevPhase, curPhase);
}

void bHandler(void* args) {
  // Get the gpio handle from the args
  mraa::Gpio *encB = (mraa::Gpio*)args;
  int a = aState;
  int b = bState;
  int newB = encB->read();
  bState = newB;
  int prevPhase = getPhase(a, b);
  int curPhase = getPhase(a, newB);
  updateTick(prevPhase, curPhase);
}

int main() {
  // Handle Ctrl-C quit
  signal(SIGINT, sig_handler);

  mraa::Gpio *encA = new mraa::Gpio(encoderAPin);
  assert(encA != NULL);
  encA->dir(mraa::DIR_IN);
  encA->isr(mraa::EDGE_BOTH, aHandler, encA);
  mraa::Gpio *encB = new mraa::Gpio(encoderBPin);
  assert(encB != NULL);
  encB->dir(mraa::DIR_IN);
  encB->isr(mraa::EDGE_BOTH, bHandler, encB);

  while (running) {
    std::cout << "encoderCount(+): " << encoderCount << std::endl;
    usleep(100000.0);
  }
}
