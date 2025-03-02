#include "util/PID.hpp"
#include "math.h"

PID pidInit(float fKP, float fKI, float fKD, float fEpsilonInner,
            float fEpsilonOuter, float dInner, float dOuter) {
  PID pid;

  pid.kP = fKP;
  pid.kI = fKI;
  pid.kD = fKD;
  pid.EpsilonInner = fEpsilonInner;
  pid.EpsilonOuter = fEpsilonOuter;
  pid.sigma = 0;
  pid.lastValue = 0;
  pid.dInner = dInner;
  pid.dOuter = dOuter;

  return pid;
}

float pidCalculate(PID pid, float fSetPoint, float fProcessVariable, float dt) {
  if (fSetPoint != pid.lastTarget) {
    pid.sigma = 0;
    pid.lastTarget = fSetPoint;
  }

  float fDeltaTime = dt / 1000.0;

  float fDeltaPV = 0;
  if (fDeltaTime > 0) {
    fDeltaPV = (fProcessVariable - pid.lastValue) / fDeltaTime;
  }

  pid.lastValue = fProcessVariable;

  float fError = fSetPoint - fProcessVariable;

  if (fabs(fError) > pid.EpsilonInner && fabs(fError) < pid.EpsilonOuter)
    pid.sigma += fError * fDeltaTime;

  if (fabs(fError) > pid.EpsilonOuter)
    pid.sigma = 0;

  float fOutput =
      fError * pid.kP + pid.sigma * pid.kI -
      (((fabs(fSetPoint) - fabs(fError) > 0 && fabs(fError) < pid.dInner) ||
        (fabs(fSetPoint) - fabs(fError) < 0 && fabs(fError) < pid.dOuter))
           ? (fDeltaPV * pid.kD)
           : 0);

  fOutput = fabs(fOutput) > 255 ? 255 * fOutput / fabs(fOutput) : fOutput;
  return fOutput;
}