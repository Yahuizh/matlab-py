#include "flyball.h"
#include "rtwtypes.h"

void flyball::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t { rtsiGetT(si) };

  time_T tnew { rtsiGetSolverStopTime(si) };

  time_T h { rtsiGetStepSize(si) };

  real_T *x { rtsiGetContStates(si) };

  ODE4_IntgData *id { static_cast<ODE4_IntgData *>(rtsiGetSolverData(si)) };

  real_T *y { id->y };

  real_T *f0 { id->f[0] };

  real_T *f1 { id->f[1] };

  real_T *f2 { id->f[2] };

  real_T *f3 { id->f[3] };

  real_T temp;
  int_T i;
  int_T nXc { 2 };

  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));
  rtsiSetdX(si, f0);
  flyball_derivatives();
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  flyball_derivatives();
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  flyball_derivatives();
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  flyball_derivatives();
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

void flyball::step()
{
  real_T rtb_Clock1;
  real_T rtb_Gain;
  real_T rtb_Square1;
  real_T rtb_Square2;
  boolean_T OR;
  boolean_T tmp;
  if ((&flyball_M)->isMajorTimeStep()) {
    rtsiSetSolverStopTime(&(&flyball_M)->solverInfo,(((&flyball_M)
      ->Timing.clockTick0+1)*(&flyball_M)->Timing.stepSize0));
  }

  if ((&flyball_M)->isMinorTimeStep()) {
    (&flyball_M)->Timing.t[0] = rtsiGetT(&(&flyball_M)->solverInfo);
  }

  rtb_Clock1 = (&flyball_M)->Timing.t[0];
  OR = ((rtb_Clock1 >= 10.0) || (flyball_X.y_CSTATE <= 0.0));
  tmp = ((&flyball_M)->isMajorTimeStep());
  if (tmp && OR) {
    (&flyball_M)->setStopRequested(1);
  }

  flyball_B.y = flyball_X.y_CSTATE;
  flyball_B.vy_m = flyball_X.vy_CSTATE;
  flyball_B.vy = flyball_B.vy_m;
  if (OR) {
    rtb_Gain = flyball_B.vy - flyball_P.target_vy;
    rtb_Square2 = rtb_Gain * rtb_Gain;
    rtb_Gain = (flyball_B.y - flyball_P.target_y) * 0.33333333333333331;
    rtb_Square1 = rtb_Gain * rtb_Gain;
    rtb_Gain = (rtb_Clock1 - flyball_P.target_t) * 10.0;
    flyball_B.reward = ((rtb_Gain * rtb_Gain + rtb_Square1) + rtb_Square2) *
      -0.01 + 5.0;
  } else {
    flyball_B.reward = -0.001;
  }

  if (tmp) {
    flyball_B.ay_c = flyball_P.control - 9.8;
    flyball_B.ay = flyball_B.ay_c;
  }

  if ((&flyball_M)->isMajorTimeStep()) {
    rt_ertODEUpdateContinuousStates(&(&flyball_M)->solverInfo);
    ++(&flyball_M)->Timing.clockTick0;
    (&flyball_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&flyball_M)->solverInfo);

    {
      (&flyball_M)->Timing.clockTick1++;
    }
  }
}

void flyball::flyball_derivatives()
{
  flyball::XDot_flyball_T *_rtXdot;
  _rtXdot = ((XDot_flyball_T *) (&flyball_M)->derivs);
  _rtXdot->y_CSTATE = flyball_B.vy_m;
  _rtXdot->vy_CSTATE = flyball_B.ay_c;
}

void flyball::initialize()
{
  {
    rtsiSetSimTimeStepPtr(&(&flyball_M)->solverInfo, &(&flyball_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&flyball_M)->solverInfo, (&flyball_M)->getTPtrPtr());
    rtsiSetStepSizePtr(&(&flyball_M)->solverInfo, &(&flyball_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&flyball_M)->solverInfo, &(&flyball_M)->derivs);
    rtsiSetContStatesPtr(&(&flyball_M)->solverInfo, (real_T **) &(&flyball_M)
                         ->contStates);
    rtsiSetNumContStatesPtr(&(&flyball_M)->solverInfo, &(&flyball_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&flyball_M)->solverInfo, &(&flyball_M)
      ->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&flyball_M)->solverInfo, &(&flyball_M
      )->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&flyball_M)->solverInfo, &(&flyball_M)
      ->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&flyball_M)->solverInfo, (boolean_T**)
      &(&flyball_M)->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&flyball_M)->solverInfo, (&flyball_M)
                          ->getErrorStatusPtr());
    rtsiSetRTModelPtr(&(&flyball_M)->solverInfo, (&flyball_M));
  }

  rtsiSetSimTimeStep(&(&flyball_M)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&flyball_M)->solverInfo, false);
  rtsiSetIsContModeFrozen(&(&flyball_M)->solverInfo, false);
  (&flyball_M)->intgData.y = (&flyball_M)->odeY;
  (&flyball_M)->intgData.f[0] = (&flyball_M)->odeF[0];
  (&flyball_M)->intgData.f[1] = (&flyball_M)->odeF[1];
  (&flyball_M)->intgData.f[2] = (&flyball_M)->odeF[2];
  (&flyball_M)->intgData.f[3] = (&flyball_M)->odeF[3];
  (&flyball_M)->contStates = ((X_flyball_T *) &flyball_X);
  (&flyball_M)->contStateDisabled = ((XDis_flyball_T *) &flyball_XDis);
  (&flyball_M)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&flyball_M)->solverInfo, static_cast<void *>(&(&flyball_M
    )->intgData));
  rtsiSetSolverName(&(&flyball_M)->solverInfo,"ode4");
  (&flyball_M)->setTPtr(&(&flyball_M)->Timing.tArray[0]);
  (&flyball_M)->Timing.stepSize0 = 0.1;
  flyball_X.y_CSTATE = 100.0;
  flyball_X.vy_CSTATE = 0.0;
}

void flyball::terminate()
{
}

time_T** flyball::RT_MODEL_flyball_T::getTPtrPtr()
{
  return &(Timing.t);
}

boolean_T flyball::RT_MODEL_flyball_T::getStopRequested() const
{
  return (Timing.stopRequestedFlag);
}

void flyball::RT_MODEL_flyball_T::setStopRequested(boolean_T aStopRequested)
{
  (Timing.stopRequestedFlag = aStopRequested);
}

const char_T* flyball::RT_MODEL_flyball_T::getErrorStatus() const
{
  return (errorStatus);
}

void flyball::RT_MODEL_flyball_T::setErrorStatus(const char_T* const
  aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

time_T* flyball::RT_MODEL_flyball_T::getTPtr() const
{
  return (Timing.t);
}

void flyball::RT_MODEL_flyball_T::setTPtr(time_T* aTPtr)
{
  (Timing.t = aTPtr);
}

boolean_T* flyball::RT_MODEL_flyball_T::getStopRequestedPtr()
{
  return (&(Timing.stopRequestedFlag));
}

const char_T** flyball::RT_MODEL_flyball_T::getErrorStatusPtr()
{
  return &errorStatus;
}

boolean_T flyball::RT_MODEL_flyball_T::isMajorTimeStep() const
{
  return ((Timing.simTimeStep) == MAJOR_TIME_STEP);
}

boolean_T flyball::RT_MODEL_flyball_T::isMinorTimeStep() const
{
  return ((Timing.simTimeStep) == MINOR_TIME_STEP);
}

time_T flyball::RT_MODEL_flyball_T::getTStart() const
{
  return (Timing.tStart);
}

flyball::flyball() :
  flyball_B(),
  flyball_X(),
  flyball_XDis(),
  flyball_M()
{
}

flyball::~flyball() = default;
flyball::RT_MODEL_flyball_T * flyball::getRTM()
{
  return (&flyball_M);
}
