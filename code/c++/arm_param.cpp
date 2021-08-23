#include "arm_param.h"

//убиваем группировку динамикселей. в 1.4 это не надо
/*
uint8_t dxl_groups[DXL_GROUPS][DXL_IN_GROUP] = {
  {1, 4},
  {5, 6},
  {2, 3}
};
*/

uint8_t dxl_id[DXL_CNT] = {1, 2, 3, 4, 5, 6};

float ik_angular_offsets[DXL_CNT] = { 0,  PI / 2.,  PI / 2.,  PI / 2., 0, 0};
