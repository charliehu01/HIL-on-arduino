#include <SPI.h>
#include "tdc_contract.h"
#include "spi_controller.h"
#include "decode.h"
#include <cmath>

const uint8_t CS_A = 10, CS_B = 9, INT_A = 2, INT_B = 3;
const float PEAK_NS       = -20.0f;   // expected tA - tB (= -delay). Update for the delay sweep.
const float COINC_HALF_NS = 1.8f;     // coincidence window is +/- this around the peak
const unsigned long TIMEOUT_MS = 30;
const bool DEBUG_RAW = false;         // true for the golden packet test

volatile bool gotA = false, gotB = false;
void isrA() { gotA = true; }
void isrB() { gotB = true; }

uint32_t seq = 0, nMeas = 0, nBoth = 0, nCoinc = 0, nTimeout = 0;

// Provided: prints the raw registers and decoded times (use for the golden packet test).
void printRaw(uint8_t cs, uint8_t status, const uint32_t r[N_RESULT_REGS],
              const float *ns, int n) {
  Serial.print(F("R,")); Serial.print(cs == CS_A ? 'A' : 'B');
  Serial.print(F(",status=0x")); Serial.print(status, HEX);
  Serial.print(F(",T1=")); Serial.print(r[0]);
  Serial.print(F(",T2=")); Serial.print(r[2]);
  Serial.print(F(",CAL1=")); Serial.print(r[11]);
  Serial.print(F(",CAL2=")); Serial.print(r[12]);
  for (int i = 0; i < n; i++) {
    Serial.print(F(",TOF")); Serial.print(i + 1); Serial.print('=');
    Serial.print(ns[i], 2);
  }
  Serial.println();
}

// TODO 1: collect the stop times from one plant.
//   - if !got, the plant never signalled INTB: return -1
//   - read INT_STATUS first (this also releases INTB), then readResult()
//   - decode TIME1, TIME2, ... (r[0], r[2], r[4], r[6], r[8]) into ns[] with decodeNs()
//     using CAL1 = r[11], CAL2 = r[12], CAL2_PERIODS, TDC_CLK_NS
//   - a TIME of 0 means "no stop here": stop collecting
//   - if DEBUG_RAW, call printRaw(...)
//   - return the number of stops stored in ns[] (max 5)
int readArm(uint8_t cs, bool got, float *ns) {
  if (!got) {
    return -1;
  }

  uint8_t status = readReg(cs, REG_INT_STATUS);
  
  uint32_t r[N_RESULT_REGS];
  readResult(cs, r);

  int stops = 0;

  for (int i = 0; i < 5; i++) {
    if (r[i * 2] == 0) {
      break;
    }

    ns[i] = decodeNs(r[i * 2], r[11], r[12], CAL2_PERIODS, TDC_CLK_NS);
    stops++;
  }

  if (DEBUG_RAW) {
    printRaw(cs, status, r, ns, stops);
  }

  return stops;
}

// TODO 2: count coincidences between arm A and arm B in this measurement.
//   A coincidence is a pair (a[i], b[j]) with |(a[i] - b[j]) - PEAK_NS| <= COINC_HALF_NS.
int countCoincidences(const float *a, int nA, const float *b, int nB) {
  int coinc = 0;

  for (int i = 0; i < nA; i++) {
    for (int j = 0; j < nB; j++) {
      if (std::abs(a[i] - b[j] - PEAK_NS) <= COINC_HALF_NS) {
        coinc++;
      }
    }
  }
  
  return coinc;
}

void setup() {
  Serial.begin(115200);
  pinMode(INT_A, INPUT_PULLUP);
  pinMode(INT_B, INPUT_PULLUP);
  spiControllerBegin(CS_A, CS_B);
  attachInterrupt(digitalPinToInterrupt(INT_A), isrA, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT_B), isrB, FALLING);
  delay(200);
  writeReg(CS_A, REG_CONFIG2, CFG2_DEFAULT);
  writeReg(CS_B, REG_CONFIG2, CFG2_DEFAULT);
  Serial.print(F("Selftest decode (expect 247.061): "));
  Serial.println(decodeNs(4175, 2110, 21121, 10, 125.0f), 3);
}

void loop() {
  seq++;
  uint8_t s[4] = { (uint8_t)(seq >> 24), (uint8_t)(seq >> 16),
                   (uint8_t)(seq >> 8),  (uint8_t)seq };
  writeBurst(CS_A, REG_SEQ, s, 4);     // both plants need the same measurement number
  writeBurst(CS_B, REG_SEQ, s, 4);

  gotA = gotB = false;
  writeReg(CS_A, REG_CONFIG1, CFG1_FORCE_CAL | CFG1_START_MEAS);
  writeReg(CS_B, REG_CONFIG1, CFG1_FORCE_CAL | CFG1_START_MEAS);

  unsigned long t0 = millis();
  while ((!gotA || !gotB) && millis() - t0 < TIMEOUT_MS) { }

  float a[5], b[5];
  int nA = readArm(CS_A, gotA, a);
  int nB = readArm(CS_B, gotB, b);
  nMeas++;
  if (nA < 0 || nB < 0) nTimeout++;

  if (nA > 0 && nB > 0) {
    nBoth++;
    int coinc = countCoincidences(a, nA, b, nB);
    nCoinc += coinc;
    Serial.print(F("M,")); Serial.print(seq);
    Serial.print(F(",A:"));
    for (int i = 0; i < nA; i++) { if (i) Serial.print('/'); Serial.print(a[i], 2); }
    Serial.print(F(",B:"));
    for (int j = 0; j < nB; j++) { if (j) Serial.print('/'); Serial.print(b[j], 2); }
    Serial.print(F(",coinc=")); Serial.println(coinc);
  }

  if (nMeas % 1000 == 0) {
    Serial.print(F("S,")); Serial.print(nMeas);
    Serial.print(F(",both=")); Serial.print(nBoth);
    Serial.print(F(",coinc=")); Serial.print(nCoinc);
    Serial.print(F(",timeouts=")); Serial.println(nTimeout);
  }
}