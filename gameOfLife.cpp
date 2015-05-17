#include "HT1632.h"

#define DATA 2
#define WR 3
#define CS 4

HT1632LEDMatrix displayMatrix = HT1632LEDMatrix(DATA, WR, CS);
HT1632LEDMatrix memoryMatrix = HT1632LEDMatrix(DATA, WR, CS);

uint8_t W = displayMatrix.width();
uint8_t H = displayMatrix.height();

#define START_DELAY 100
#define GENERATION_DELAY 100
#define RESET_DELAY 1000

#define PREV_ALIVES 10
uint16_t prevAlive[PREV_ALIVES];

void clearPrevAlives() {
  for (uint8_t i=0; i<PREV_ALIVES; i++) {
    prevAlive[i] = 0;
  }
}

boolean samePrevAlives(uint16_t alive) {
  for (uint8_t i=0; i<PREV_ALIVES; i++) {
    if (prevAlive[i] != alive) return false;
  }
  return true;
}

void shiftPrevAlives(uint16_t alive) {
  for (uint8_t i=PREV_ALIVES-1; i>0; i--) {
    prevAlive[i] = prevAlive[i-1];
  }
  prevAlive[0] = alive;
}

void randomize() {
  for (uint8_t y=0; y<H; y++) {
    for (uint8_t x=0; x<W; x++) {
      if (random(0,2)==0) {
        displayMatrix.setPixel(x, y);
        memoryMatrix.setPixel(x, y);
      } else {
        displayMatrix.clrPixel(x, y);
        memoryMatrix.clrPixel(x, y);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
  displayMatrix.begin(HT1632_COMMON_16NMOS);
  displayMatrix.setBrightness(1);
  delay(START_DELAY);
  randomize();
  clearPrevAlives();
  displayMatrix.writeScreen();
  delay(GENERATION_DELAY);
}

void loop() {
  uint8_t x, y;
  uint8_t xn, xp, yn, yp;
  uint8_t nn, _n, pn;
  uint8_t n_, __,  p_;
  uint8_t np, _p, pp;
  uint8_t neighbors;
  uint16_t alive;
  
  alive = 0;
  for (y=0; y<H; y++) {
    for (x=0; x<W; x++) {
      xn = (x + W - 1) % W;
      xp = (x + 1) % W;
      yn = (y + H - 1) % H;
      yp = (y + 1) % H;
      
      nn = memoryMatrix.getPixel(xn, yn);
      _n = memoryMatrix.getPixel(x , yn);
      pn = memoryMatrix.getPixel(xp, yn);
      
      n_ = memoryMatrix.getPixel(xn, y );
      __ = memoryMatrix.getPixel(x , y );
      p_ = memoryMatrix.getPixel(xp, y );
      
      np = memoryMatrix.getPixel(xn, yp);
      _p = memoryMatrix.getPixel(x , yp);
      pp = memoryMatrix.getPixel(xp, yp);
      
      neighbors = nn + _n + pn
                + n_      + p_
                + np + _p + pp;
      
      if ((__ && neighbors == 2) || neighbors == 3) {
        displayMatrix.setPixel(x, y);
        alive++;
      } else {
        displayMatrix.clrPixel(x, y);
      }
    }
  }
  
  for (y=0; y<H; y++) {
    for (x=0; x<W; x++) {
      if (displayMatrix.getPixel(x, y)) {
        memoryMatrix.setPixel(x, y);
      } else {
        memoryMatrix.clrPixel(x, y);
      }
    }
  }
  
  displayMatrix.writeScreen();
  delay(GENERATION_DELAY);
  
  if (samePrevAlives(alive)) {
    delay(RESET_DELAY);
    randomize();
    clearPrevAlives();
    displayMatrix.writeScreen();
    delay(GENERATION_DELAY);
  } else {
    shiftPrevAlives(alive);
  }
}