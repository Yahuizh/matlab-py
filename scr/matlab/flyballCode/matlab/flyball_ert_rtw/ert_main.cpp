#include <stdio.h>
#include "flyball.h"

static flyball flyball_Obj;
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag{ false };

  if (OverrunFlag) {
    flyball_Obj.getRTM()->setErrorStatus("Overrun");
    return;
  }

  OverrunFlag = true;
  flyball_Obj.step();
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  flyball_Obj.initialize();
  while (flyball_Obj.getRTM()->getErrorStatus() == (nullptr)&&
         !flyball_Obj.getRTM()->getStopRequested()) {
    rt_OneStep();
  }

  flyball_Obj.terminate();
  return 0;
}
