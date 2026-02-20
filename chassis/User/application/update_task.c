#include "update_task.h"
#include "cmsis_os.h"
#include "wheel_legged_chassis.h"


void chassis_sys_calc(void);

void update_task(void const *argument)
{

  while (1)
  {
    chassis_sys_calc();
    osDelay(3);
  }
}
