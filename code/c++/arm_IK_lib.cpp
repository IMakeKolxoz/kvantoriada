#include "arm_IK_lib.h"

// Иницализируем класс DXL_Workbrench
DynamixelWorkbench dxl_wb;

const uint8_t handler_index_pos = 0;
const uint8_t handler_index_torque = 1;
const uint8_t handler_index_speed = 2;

// Массив для синхронной отправки удержания позиции
int32_t torque_on[DXL_CNT] = {1, 1, 1, 1, 1, 1};
//ID                          1  4  2  3  5  6

// Массив для синхронного отключения удержания позиции
int32_t torque_off[DXL_CNT] = {0, 0, 0, 0, 0, 0};
//ID                           1  4  2  3  5  6

// Массив для синхронной отправки требуемой позиции
int32_t goal_position[DXL_CNT] = {2048, 2048, 2048, 2048, 512, END_EFFECTOR_OPEN};
//ID                               1      4     2     3    5        6

// Массив для синхронной всех приводов в 0
int32_t goal_position_null[DXL_CNT] = {0, 0, 0, 0, 0, 0};


// Массив для синхронной отправки требуемой скорости
int32_t goal_speed[DXL_CNT] = {16, 12, 12, 16, 60, 100};
//ID                           1   2   3  4   5   6

// Массив адапативных скоростей
int32_t adapting_speed[DXL_CNT] = {0, 0, 0, 0, 0, 0};

// Нужна для расчета адаптивной скорости
int32_t dxl_delta_move[DXL_CNT] = {0, 0, 0, 0, 0, 0};


// Расчет и применение адаптивной скорости
void adapt_vel(float pos_x, float pos_z, float angle, float next_pos_x, float next_pos_z, float next_angle)
{
  //Для логирования и контроля ошибок
const char *log;
bool result = false;


  float  dxl_delta_move_med = 0.;
  int32_t data_packet[DXL_CNT];

  //Иницализируем начальную скорость
  for (int i = 0; i < DXL_CNT; i++)
  {
    adapting_speed[i] = goal_speed[i];
  }

  // Получаем информацию о нынешних позициях всех двигателей
  get_data_pos(data_packet, "Present_Position");

  //Считаем следующее смещение
  inverse_kinematics(next_pos_x, next_pos_z, next_angle);

  //Считаем разницу в угле поворота для каждого движка и находим суммарное смещение
  int fist_moving_dxl = 0;


  if (abs(angle - next_angle) == 0.) //Если первый движок не будет двигаться
  {
    fist_moving_dxl = 1;  //То пропускаем его при расчете суммарного смещения
  }
  // dxl_delta_move[0] = abs(data_packet[0] - goal_position[sync_order(0)]); //Но все-равно считаем для него само смещение
  //  Serial1.println(dxl_delta_move[0]);
  // Если требуется позиционирование (кроме вращения 1-го двигателя), то только тогда считаем адаптированную скорость
  if ((abs(pos_x - next_pos_x) > 0) || ((abs(pos_z - next_pos_z) > 0)))
  {
    // не учитываем 1 двигатель
    for (int i = 1; i < DXL_CNT - 2; i++) //Без 2-х последних движков
    {
      dxl_delta_move[i] = abs(data_packet[i] - goal_position[sync_order(i)]);
      dxl_delta_move_med += dxl_delta_move[i];
    }

    //Ищем среднее смещение
    dxl_delta_move_med /= (double)(DXL_CNT - 2 - 1);

    //Рассчитываем итоговую адаптированную скорость. Здесь уже адаптируем первую скорость
    for (int i = 1; i < DXL_CNT - 2; i++)
    {
      adapting_speed[sync_order(i)] = (int) ((dxl_delta_move_med / dxl_delta_move[i]) * goal_speed[sync_order(i)]);
      Serial1.println((int)((dxl_delta_move_med / dxl_delta_move[i] ) * goal_speed[sync_order(i)]));
    }
  }

  //dxl_wb.syncWrite("Moving_Speed", adapting_speed);   // Сразу ограничим скорость движения элементов руки
    result = dxl_wb.syncWrite(handler_index_speed, adapting_speed, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to sync adapting_speed");
  }

}



float pos_to_rotations(int pos, char type)
{
  double dxl_center = 2048.;
  if (dxl_center == 'A')
    dxl_center = 1024.;
  // Здесь идет учето того, что реально центр двигателя это угол в PI радиан, но обратная задача считалась для случая при котором центр это PI/2.
  return pos / dxl_center ;
}


// Пингование двигателей и перевод их в режим joint
void dxls_init()
{
  //Для логирования и контроля ошибок
const char *log;
bool result = false;
uint16_t model_number = 0;

    for (int cnt = 0; cnt < 6; cnt++)
  {
    result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);
    if (result == false)
    {
      Serial1.println(log);
      Serial1.println("#Failed to ping");
    }
    else
    {
      Serial1.println("#Succeeded to ping");
      Serial1.print("id : ");
      Serial1.print(dxl_id[cnt]);
      Serial1.print(" model_number : ");
      Serial1.println(model_number);
    }
    delay(100);
    result = dxl_wb.jointMode(dxl_id[cnt], 0, 0, &log);
    if (result == false)
    {
      Serial1.println(log);
      Serial1.println("#Failed to change joint mode");
    }
    else
    {
      Serial1.println("#Succeed to change joint mode");
    }
    delay(100);
  }
Serial1.println("#End init");
/*
Serial1.println("Set all position in 0");
  result = dxl_wb.syncWrite(handler_index_pos, goal_position_null, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("Failed to sync goal_position");
  }
  delay(3000);
  Serial1.println("End set all position in 0");
  */
}


/* Эта функция переводит требуемой позиции, соответствующей возрастающему ID двигателя,  в номер согласно очереди пингования.
   Данная функция нужна для использования в массивах, отсылаемых через SyncWrute
*/
int sync_order(int i)
{
  
  switch (i)
  {
    // Так как мы говорим про позиции в массиве, то фактически позиция = id - 1
    case (0):
      return 0; // первый двигатель пинговался первым
    case (1):
      return 2; // второй двигатель пинговался третьим
    case (2):
      return 3; // третий двигатель пинговался четвертым
    case (3):
      return 1; // четвертый двигатель пинговался второым
    default:
      return i; // 5 и 6 двигатели без изменений
  }
}


//Расчет угловых смещений для расчета обратной книматики
void calc_ik_angular_offsets()
{
  float alpha = atan((double)L1B / L1A);
  float beta = alpha;
  ik_angular_offsets[1] += alpha; //Второй двигатель
  ik_angular_offsets[2] = ik_angular_offsets[2] - beta + PI;  //Третий двигатель
}


/* Функция inverse_kinematics считает обратную задачу. Аргументы:
    1) расстояние от центра (мм) ;
    2) высота середины схвата над низом базы(мм);
    3) угол поворота базы (радианы).
   Сам схват смотрим всегда вертикально вниз.
   Так как мы хотим сейчас вывести руку в серединное положение (П образное), то расстояние от центра базы равно длине звена 2: L2,
   А вот высота, с учтом того, что она считается с учтом высоты самой базы и длины звена 3 (длина от двигателя 4 до середины схвата),
   то высота должна быть равна длине первого звена + высота базы - длина звена 3: L1 + BASE_HEIGHT- L3.
   Ну и поворот относительно базы пусть будет равным PI.
*/
//Фукнция расчета обратной задачи
void inverse_kinematics(float x, float z, float angle)
{
  float q[4]; // Массив углов.

  z = z - BASE_HEIGHT + L3; // На самом деле эта задача считается относительно позиции первого двигателя и до позиции кисти.
  q[0] = angle; // Двигатель 1 - поворот базы
  q[2] = -acos( (pow(x, 2) + pow(z, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * (double)L1 * L2) );  // Сначала нужно расчитать позицию 3 двигателя
  q[1] = atan(z / (double)x) - atan( ((double)L2 * sin(q[2])) / ((double)L1 + L2 * cos(q[2])) );  // Через нее считаем позицию 2 двигателя
  q[3] = asin(((double)L1 * sin(q[1]) - (double)z) / (double)L2); // Обеспечиваем вертикальность схвата перерасчитывая углы для 4 двигателя
  for (int i = 0; i < DXL_POSITIONING_END_EFFECTOR; i++)
  {
    q[i] += (double)ik_angular_offsets[i];  // Учитываем угловые смещения.
    goal_position[sync_order(i)] = angle_to_pos(q[i]);  // Сохраняем значения позиций с учетом очередности пингования двигателей.
  }
  goal_position[sync_order(3)] += POSITION_OFFSET_DXL4;
}


// Перевод углов (Радианы), в позицию двигателей. Второй аргумент определяет тип двигателя и, соответственно, его разрядность.
int angle_to_pos(float angle, char type)
{
  double dxl_center = 2048.;
  double pos = dxl_center * (double)angle / PI;
  // Здесь идет учето того, что реально центр двигателя это угол в PI радиан, но обратная задача считалась для случая при котором центр это PI/2.
  if (type == 'A')
    return (map (pos , 0., 4095., 0., 1023. + 1024. / 5.) - 1024. / 10.);

  return pos ;
}


//Функция проверки факта перемещения манипулятора: если какой-либо двигатель двигается, то манипулятор все еще выполняет задачу
int is_moving()
{
 
  int32_t moving = 0;
  if(digitalRead(button) != 1){
  for (int cnt = 0; cnt < 6 ; cnt++)
  {
    dxl_wb.itemRead(dxl_id[cnt], "Moving", &moving );
    delay(30); //nujno li7
    if (moving)
      return 1;
  }
  return 0;
      }
  else return 2;
}


// Функция вывода позиции в виду пакета M:is_moving:pos:pos2:...:posN#
void print_packet(int32_t* data_array)
{
  String message = "";
  // message += String(is_moving());
  for (int dxl = 0; dxl < DXL_CNT - 1; dxl++)
    message += String(data_array[dxl]) + ":";
  message += String(data_array[DXL_CNT - 1]);
  // message += "#";
  Serial1.println(message);
}

String str_packet(int32_t* data_array)
{
  String message = "";
  // message += String(is_moving());
  for (int dxl = 0; dxl < DXL_CNT - 1; dxl++)
    message += String(data_array[dxl]) + ":";
  message += String(data_array[DXL_CNT - 1]);
  // message += "#";
  return message;
}


// Функция получения информации о позициях
void get_data_pos(int32_t* data_array, char* table_pos)
{
  for (int cnt = 0; cnt < 6; cnt++){
  int32_t get_present_position = 0;
    dxl_wb.itemRead(dxl_id[cnt], "Present_Position", &get_present_position);  //Для протокола 1.0 SyncRead не доступна.
    data_array[cnt] = get_present_position;
  }
}

// Функция получения информации о нагрузках
void get_data_load(int32_t* data_array, char* table_pos)
{
  for (int cnt = 0; cnt < 6; cnt++){
  int32_t get_present_load = 0;
    dxl_wb.itemRead(dxl_id[cnt], "Present_Load", &get_present_load);  //Для протокола 1.0 SyncRead не доступна.
    data_array[cnt] = get_present_load;
  }
}

// Функция получения информации о температуре
void get_data_temp(int32_t* data_array, char* table_pos)
{
  for (int cnt = 0; cnt < 6; cnt++){
  int32_t get_present_temp = 0;
    dxl_wb.itemRead(dxl_id[cnt], "Present_Temperature", &get_present_temp);  //Для протокола 1.0 SyncRead не доступна.
    data_array[cnt] = get_present_temp;
  }
}
