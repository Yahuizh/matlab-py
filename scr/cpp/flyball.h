#ifndef flyball_h_
#define flyball_h_
#include <cmath>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "flyball_types.h"
#include <cstring>
#ifndef ODE4_INTG
#define ODE4_INTG

struct ODE4_IntgData {
  real_T *y;
  real_T *f[4];
};

#endif

class flyball final
{
 public:
  struct B_flyball_T {
    real_T y;
    real_T vy_m;
    real_T vy;
    real_T reward;
    real_T ay_c;
    real_T ay;
  };

  struct X_flyball_T {
    real_T y_CSTATE;
    real_T vy_CSTATE;
  };

  struct XDot_flyball_T {
    real_T y_CSTATE;
    real_T vy_CSTATE;
  };

  struct XDis_flyball_T {
    boolean_T y_CSTATE;
    boolean_T vy_CSTATE;
  };

  struct P_flyball_T {
    real_T control;
    real_T target_t;
    real_T target_vy;
    real_T target_y;
  };

  using odeFSubArray = real_T[2];
  struct RT_MODEL_flyball_T {
    const char_T *errorStatus;
    RTWSolverInfo solverInfo;
    X_flyball_T *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    XDis_flyball_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T CTOutputIncnstWithState;
    real_T odeY[2];
    real_T odeF[4][2];
    ODE4_IntgData intgData;
    struct {
      int_T numContStates;
      int_T numPeriodicContStates;
      int_T numSampTimes;
    } Sizes;

    struct {
      uint32_T clockTick0;
      time_T stepSize0;
      uint32_T clockTick1;
      time_T tStart;
      SimTimeStep simTimeStep;
      boolean_T stopRequestedFlag;
      time_T *t;
      time_T tArray[2];
    } Timing;

    time_T** getTPtrPtr();
    boolean_T getStopRequested() const;
    void setStopRequested(boolean_T aStopRequested);
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const aErrorStatus);
    time_T* getTPtr() const;
    void setTPtr(time_T* aTPtr);
    boolean_T* getStopRequestedPtr();
    const char_T** getErrorStatusPtr();
    boolean_T isMajorTimeStep() const;
    boolean_T isMinorTimeStep() const;
    time_T getTStart() const;
  };

  flyball(flyball const&) = delete;
  flyball& operator= (flyball const&) & = delete;
  flyball(flyball &&) = delete;
  flyball& operator= (flyball &&) = delete;
  flyball::RT_MODEL_flyball_T * getRTM();
  void initialize();
  void step();
  static void terminate();
  flyball();
  ~flyball();
 private:
  B_flyball_T flyball_B;
  static P_flyball_T flyball_P;
  X_flyball_T flyball_X;
  XDis_flyball_T flyball_XDis;
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );
  void flyball_derivatives();
  RT_MODEL_flyball_T flyball_M;
};

#endif

