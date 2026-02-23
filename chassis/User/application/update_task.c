/**
 * @file update_task.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "update_task.h"
#include "cmsis_os.h"

void update_task(void const *argument)
{

  while (1)
  {
    osDelay(3);
  }
}
