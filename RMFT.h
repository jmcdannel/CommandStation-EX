#ifndef RMFT_H
#define RMFT_H

#if __has_include ( "myAutomation.h")
#include "RMFT2.h"

class RMFT {
  public:
   static void inline begin() {RMFT2::begin();}
   static void inline loop() {RMFT2::loop();}
};

  #include "myAutomation.h"
  #define RMFT_ACTIVE
#endif

#endif
