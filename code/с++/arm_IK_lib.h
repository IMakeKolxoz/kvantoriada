#ifndef ARM_IK_LIB_H
#define ARM_IK_LIB_H

#include "arm_param.h"
#include <DynamixelWorkbench.h>

#define DEVICE_NAME "3"   // Динамиксели подключены к Serial13(USART3)  <-OpenCM Expansion Board
#define BAUDRATE  1000000 // Скорость общения движков между собой

#define PACKET_DELAY 500 // Временная задержка в мс для отправки пакетов

/* Группировка будет следующая: 1) MX28; 2) MX64; 3) AX
   Соответственно, двигатели будут расположены так: {1,4}, {2,3}, {5,6}
   Важно: SyncWrite будет считать, соответственно, что данные из массива будут отправляться следующим образом:
   [ pos1, pos2, pos3, pos4, pos5, pos6]
   ID:  1     4     2     3     5     6
*/


// Иницализируем класс DXL_Workbrench
extern DynamixelWorkbench dxl_wb;

// Массив для синхронной отправки удержания позиции
extern int32_t torque_on[DXL_CNT];


// Массив для синхронного отключения удержания позиции
extern int32_t torque_off[DXL_CNT];

// Массив для синхронной отправки позиции в 0
extern int32_t goal_position_null[DXL_CNT];

// Массив для синхронной отправки требуемой позиции
extern int32_t goal_position[DXL_CNT];

// Массив для синхронной отправки требуемой скорости
extern int32_t goal_speed[DXL_CNT];

// Массив адапативных скоростей
extern int32_t adapting_speed[DXL_CNT];

// Нужна для расчета адаптивной скорости
extern int32_t dxl_delta_move[DXL_CNT];

#define button D10

// Расчет и применение скорости движения каждой сервы
void adapt_vel(float pos_x, float pos_z, float angle, float next_pos_x, float next_pos_z, float next_angle);

// Перевод из единицы позиции в обороты
float pos_to_rotations(int pos, char type = 'M');

// Пингование двигателей и перевод их в режим joint
void dxls_init();

/* Эта функция переводит требуемой позиции, соответствующей возрастающему ID двигателя,  в номер согласно очереди пингования.
   Данная функция нужна для использования в массивах, отсылаемых через SyncWrute
*/
int sync_order(int i);

//Расчет угловых смещений для расчета обратной книматики
void calc_ik_angular_offsets();

//Фукнция расчета обратной задачи
void inverse_kinematics(float x, float z, float angle);

// Перевод углов (Радианы), в позицию двигателей. Второй аргумент определяет тип двигателя и, соответственно, его разрядность.
int angle_to_pos(float angle, char type = 'M');

//Функция проверки факта перемещения манипулятора: если какой-либо двигатель двигается, то манипулятор все еще выполняет задачу
int is_moving();

// Функция вывода позиции в виду пакета M:is_moving:pos:pos2:...:posN#
void print_packet(int32_t* data_array);
String str_packet(int32_t* data_array);

// Функции получения информации типа table_pos со всех двигателей
void get_data_pos(int32_t* data_array, char* table_pos);
void get_data_temp(int32_t* data_array, char* table_pos);
void get_data_load(int32_t* data_array, char* table_pos);

#endif
