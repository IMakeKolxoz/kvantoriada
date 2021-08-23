//Правка под версию WorkBench 1.4
//данная версия системы управления под обычный угловой с обычным схватом
//Для управления по Ethernet

/* Ищем dynamixel_item.cpp
   Внутри описаны все названия из таблицы протокола, которые сейчас поддерживаются в WorkBench. Так, P_gain, I_gain, D_gain, а также Present_Temperature не поддерживаются.
   TODO: добвить эти таблицы в dynamixel_item.h оффициально в git репозиторий. Что сделано сейчас: добавлено локально в файл dynamixel_item необходимые куски таблицы для использования в Workbench
   Для этого заходим в dynamixel_item.cpp, ищем нужный тип движка (setMXItem (MX28), setExtMXItem(MX64), setAXItem(AX18/12)) и в самый конец таблицы для OPENCM добавляем новый
   итем с нужными характеристиками. Если посмотреть в dynamixel_tool.cpp метод DynamixelTool::getControlItem, то увидем сравнение строк по названию => более нам ничего добавлять не нужно.
   Не забываем увеличить размерность массива the_number_of_item.
   TODO: почему-то с MX28 считать температуру все-равно не выходит. С других движков норм.
   Если поставить для MX28 строку температуры после "present_Load" (40, далее 43 как раз), то все работает.
   Изучить этот момент более детально. Мб мы теряем Goal_Acceleration? А поч на других движках работает?
*/
/* В DW добавили кастомно параметры P,I,D, Present_Tempreture. Важно чтобы они в массиве шли последовательно согласно битовой позиции!!! */
#include "arm_IK_lib.h"


#define MOVE_STEP_TIME 15
#define MOVE_STEP_LEN 0.5
#define MAX_FI 150.
#define MIN_FI 0.
int32_t data_packet[DXL_CNT] = {0, 0, 0, 0, 0, 0};

// Это переменные нужны при анализе пакета позиционирования и ориентации, приходящего с сервера: p:rotation:distance:DOWN/UP:OPEN/CLOSE
float rotation = 0;
float distance = 0;
float z;
float fi = PI;

// Эта переменная нужна для засекания времени при отправке пакетов
long packet_timer = 0;

// Эта переменная нужна для выбора фазы отсылки данных: 0,2,4 - позиция; 1,3 - нагрузка; 5 - температура.
int packet_phase = 0;

// Определяет будет ли отправляться данные  робота
int SendMode = 0;

// Эта переменная нужна для планирования траектории
float ex_pos_x = 0;
float ex_pos_z = 0;
float ex_angle = 0;

// Эта переменная нужна для разрешения планирования траектории
int allow_trajectory = 0;
int steps = 0;
float delta_x ;
float delta_z ;
float delta_angle = 0;
float dxl_speed_med = 0.;
int wait_time = 70;
float step_len = 1.;
float planed_x, planed_z, planed_angle;
long move_timer = 0;

int first_move = 1;

int moving = 0;

const uint8_t handler_index_pos = 0;
const uint8_t handler_index_torque = 1;
const uint8_t handler_index_speed = 2;

//Переменная для кнопки
#define button D10
uint8_t button_state = 0;
uint8_t button_last_state = 0;

void setup()
{
  Serial1.begin(115200); // Скорость отправки и приема данных через последовательный порт.
  while (!Serial1); // TODO: если вам не нужен запуск руки после открытия последовательного порта, то эту строчку нужно убрать: она стопорит программу до этого момента.
  //инициализируем кнопку
pinMode(button, INPUT_PULLUP);
 
 //Для логирования и контроля ошибок
const char *log;
bool result = false;
  //выполняем инициализацию DXL шины на заданной скорости
  Serial1.println("#System Boot");
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to init");
  }
  else
  {
    Serial1.print("#Succeeded to init : ");
    Serial1.println(BAUDRATE);  
  }
//инициализируем динамиксели и устанавливаем им режим работы в Joint
  Serial1.println("#Setting DXLs");
  dxls_init();

  //Инициализируем синхронную запись
  Serial1.println("#Setting SyncWrite Goal_Position");
 // dxl_wb.addSyncWrite("Goal_Position");
 // dxl_wb.addSyncWrite("Torque_Enable");
 // dxl_wb.addSyncWrite("Moving_Speed");
 
 //Настройка синхронной записи Goal_Position - handler_index_pos = 0
  result = dxl_wb.addSyncWriteHandler(1, "Goal_Position", &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to add sync write handler");
  }
    Serial1.println("#Setting SyncWrite Torque_Enable");
//Настройка синхронной записи Torque_Enable - handler_index_torque = 1
 result = dxl_wb.addSyncWriteHandler(1, "Torque_Enable", &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to add sync write handler");
  }
  
  Serial1.println("#Setting SyncWrite Moving_Speed");
//Настройка синхронной записи Moving_Speed handler_index_speed = 2
   result = dxl_wb.addSyncWriteHandler(1, "Moving_Speed", &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to add sync write handler");
  }
  
  /*
    dxl_wb.itemWrite(2, "P_gain", (uint32_t)58); //Регулирвоание коэффициента П для двигателя. Приводит к застреванию программы
    dxl_wb.itemWrite(3, "P_gain", (uint32_t)58); //Регулирвоание коэффициента П для двигателя. Приводит к застреванию программы
    dxl_wb.itemWrite(4, "P_gain", (uint32_t)58); //Регулирвоание коэффициента П для двигателя. Приводит к застреванию программы

    dxl_wb.itemWrite(2, "I_gain", (uint32_t)46); //Регулирвоание коэффициента И для двигателя. Приводит к застреванию программы
    dxl_wb.itemWrite(3, "I_gain", (uint32_t)46); //Регулирвоание коэффициента И для двигателя. Приводит к застреванию программы
    dxl_wb.itemWrite(4, "I_gain", (uint32_t)46); //Регулирвоание коэффициента И для двигателя. Приводит к застреванию программы*/

 // dxl_wb.syncWrite("Moving_Speed", goal_speed);   
// Сразу ограничим скорость движения элементов руки
  result = dxl_wb.syncWrite(handler_index_speed, goal_speed, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to sync goal_speed");
  }


  calc_ik_angular_offsets();  // Считаем угловые смещения всех двигателей

  Serial1.println("#Ready");
  RedrawMenu();

  allow_trajectory = 0; //Изначально планирования траектории нет
  distance = 0.;
  rotation = 0.;
  z = 0.;
  planed_x = 0.;
  planed_z = 0.;
  planed_angle = 0.;
  packet_timer = millis();
}


void loop()
{

  button_state = digitalRead(button);
  if (button_state!= button_last_state){
  //  Serial.println("diff state");
  if(digitalRead(button) == 1){
   // Serial.println("button 1");
  distance = L2;
  z = L1 + BASE_HEIGHT - L3;
  rotation = PI;
       //останавливаем манипулятор в той же позиции
    for (int cnt = 0; cnt < 5; cnt++){
    int32_t get_present_position = 0;
    dxl_wb.itemRead(dxl_id[cnt], "Present_Position", &get_present_position);  
 //   goal_position[cnt] = get_present_position;
    dxl_wb.itemWrite(dxl_id[cnt], "Goal_Position", get_present_position);
  }
 //   dxl_wb.syncWrite(handler_index_pos, goal_position);
    }
    
   }
     
  button_last_state = button_state;
  
//Для логирования и контроля ошибок
const char *log;
bool result = false;
  
  //Блок отсылки данных по фазам
  if ((millis() - packet_timer >= PACKET_DELAY) && SendMode)
  {
    packet_timer = millis();
 //   Serial1.print(String(is_moving()) + ":");
    switch (packet_phase % 6)
    {
      case 1: case 3:
        get_data_load(data_packet, "Present_Load");
       Serial1.println("L:56:"+ String(is_moving()) + ":" + str_packet(data_packet) + "#");
        break;
      case 5:
        get_data_temp(data_packet, "Present_Temperature");
        Serial1.println(("T:56:")+ String(is_moving()) + ":" + str_packet(data_packet) + "#");
        break;
      default:
        get_data_pos(data_packet, "Present_Position");
        Serial1.println(("M:56:")+ String(is_moving()) + ":" + str_packet(data_packet) + "#");
    }
    //  Serial1.println(dxl_wb.itemRead(dxl_id[0], "Present_Temperature"));
 //   print_packet(data_packet);
    packet_phase++;
  }

  /***************НАЧАЛО БЛОКА ЧТЕНИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА*******************************/
  if (Serial1.available())
  {
    float buf_distance = 0.;
    float buf_rotation = 0.;
    float buf_z = 0.;
    float buf_fi = 0.;
    delay(20);
    char c = Serial1.read();

    //Начинаем парсинг пакета
    if (digitalRead(button) != 1){
    switch (c)
    {
      case '1':
        MoveHome();
        Serial1.println("#\nMoving to start position");
        break;

      case '2':
        MoveCenter();
        Serial1.println("#\nMoving to center position");
        break;

      case '3':
        RelaxServos();
        Serial1.println("#\nMoving to safe position and relaxing");
        break;

      case 'h':
        RedrawMenu();
        break;

      case '\n':    //Игнорим символ новой строки
        break;

      case 'r':
        SendMode = 1;
        Serial1.println("#\nSending activated.");
        RedrawMenu();
        break;

      case 's':
        SendMode = 0;
        Serial1.println("#\nSending stoped.");
        RedrawMenu();
        break;

      case 'g':
      delay(20);
        /***************НАЧАЛО ПАРСЕРА*******************************/

        if ( (c = Serial1.read()) == ':' )
        {
          while ((c = Serial1.read()) != ':')
          {
            buf_rotation *= 10;
            buf_rotation += (int)(c - '0');
            if (buf_rotation > 1000)
              break;
          }
          if ((buf_rotation > MAX_ROT) || (buf_rotation < MIN_ROT))
          {
            Serial1.println("\nRotation out of bounds. Max: " + String(MAX_ROT) + " Min: " + String(MIN_ROT));
            // Очищаем буфер
            while (Serial1.available() > 0)
              Serial1.read();
          }
          else
          {
            delay(20);
            while ((c = Serial1.read()) != ':' )
            {
              buf_distance *= 10;
              buf_distance += (int)(c - '0');
              if ( buf_distance > 1000)
                break;
            }


            if ((buf_distance > MAX_DIST) || (buf_distance < MIN_DIST))
            {
              Serial1.println("\nDistance out of bounds. Max: " + String(MAX_DIST) + " Min: " + String(MIN_DIST));
              while (Serial1.available() > 0)
                Serial1.read();
            }
            else
            {
              int sign = 1;
              delay(20);
              if ((c = Serial1.read()) == '-')
                sign = -1;
              else
              {
                buf_fi *= 10;
                buf_fi += (int)(c - '0');
              }
              delay(20);
              while ((c = Serial1.read()) != ':' )
              {
                buf_fi *= 10;
                buf_fi += (int)(c - '0');
                if ( buf_fi > 1000)
                  break;
              }
              buf_fi = sign * buf_fi;
              if ((fabs(buf_fi) > MAX_FI) || (fabs(buf_fi) < MIN_FI))
              {
                Serial1.println("#\nEnd-effector rotation out of bounds. Max module: " + String(MAX_FI) + " Min: " + String(MIN_FI));
                while (Serial1.available() > 0)
                  Serial1.read();
              }
              else
              {
                delay(20);
                if ((c = Serial1.read()) == '1')
                  buf_z = CARGO_POS_Z;
                else if ( c == '0')
                {
                  buf_z = ABOVE_CARGO_Z;
                }
                else if ( c == '2')
                {
                  buf_rotation = 140;
                  buf_distance = 180;
                  buf_z = ABOVE_CARGO_Z;
                }
                else if ( c == '3')
                {
                  buf_rotation = 180;
                  buf_z = (CARGO_POS_Z + ABOVE_CARGO_Z) / 2 ;
                  buf_distance = 180.;
                }
                delay(20);
                if ((c = Serial1.read()) == ':')
                {
                  delay(20);
                  if ((c = Serial1.read()) == '0')
                    goal_position[sync_order(GRIPPER)] = END_EFFECTOR_OPEN;
                  else if (c == '1')
                    goal_position[sync_order(GRIPPER)] = END_EFFECTOR_CLOSE;
                    delay(20);
                  if ((c = Serial1.read()) == '#')
                  {
                    // Движение возможно только после полностью корректного пакета
                    Serial1.println("#\nExecuting");
                    fi = (buf_fi + 180.) / 180. * PI;
                    goal_position[4] = angle_to_pos(fi, 'A');
                    //  Serial1.println(String(dxl_id[GRIPPER - 1]) + ": " + String(goal_position[sync_order(GRIPPER - 1)]));
                    dxl_wb.itemWrite(5, "Goal_Position",  goal_position[4]);
                    dxl_wb.itemWrite(6, "Goal_Position", goal_position[sync_order(GRIPPER)]);

                    allow_trajectory = 1; //Если нам нужно разрешить планирование траектории, то измените это значение на 1.
                    buf_rotation = buf_rotation / 180. * PI;

                    rotation = buf_rotation;
                    distance = buf_distance;
                    z = buf_z;
                  }
                  else
                  {
                    Serial1.println("#\nIncorrect packet: No # symbol");
                    while (Serial1.available() > 0)
                      Serial1.read();
                  }
                  break;
                }
                else
                {
                  Serial1.println("#\nIncorrect packet: No gripper command");
                  while (Serial1.available() > 0)
                    Serial1.read();
                }
              }
            }
          }
        }
        else
        {
          Serial1.println("#\nIncorrect packet");
          while (Serial1.available() > 0)
            Serial1.read();
        }
        break;
      /***************КОНЕЦ ПАРСЕРА*******************************/

      default:
        Serial1.println("#\nIncorrect packet");
        while (Serial1.available() > 0)
          Serial1.read();
    }
    }
  }
  /***************КОНЕЦ БЛОКА ЧТЕНИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА*******************************/
  //Serial1.println(goal_position[4]);
  if ((distance != 0) && (z != 0) && (rotation != 0))
    if ( millis() - move_timer > MOVE_STEP_TIME)
    {
      move_timer = millis();
      if(digitalRead(button) != 1){
      TrajectoryPlanning(distance, z, rotation);
    }
    }
}



void TrajectoryPlanning(float x, float z, float angle)
{

    //если кнопка не нажата
 if (digitalRead(button) != 1){
  // Serial1.println(String(steps) + ": " + String(ex_pos_x) + " : " + String(x) + " : " + String(ex_angle));
 // Serial1.println(dxl_wb.itemRead(1, "Moving"));

 //Для логирования и контроля ошибок
const char *log;
bool result = false;

  if (first_move)
  {

    inverse_kinematics(x, z, angle);
 //   dxl_wb.syncWrite("Goal_Position", goal_position);
  result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to sync goal_position");
  }

 
    delay(100);
    // moving = 0;
    if (!is_moving())
    {

      ex_pos_x = x;
      planed_x = x;
      ex_pos_z = z;
      planed_z = z;
      ex_angle = angle;
      planed_angle = angle;
      first_move = 0;
      moving = 0;
      Serial1.println("#first_move_ended");
    }
  }
  else if ((allow_trajectory) && ((x != ex_pos_x) || (z != ex_pos_z) || angle != ex_angle))
  {
    moving = 1;
    if ((planed_x != x) || (planed_z != z) || (planed_angle != angle))
    {
      planed_x = x;
      planed_z = z;
      planed_angle = angle;
      delta_x = fabs(x - ex_pos_x);
      delta_z = fabs(z - ex_pos_z);
      delta_angle = fabs(angle - ex_angle);

      if (delta_x > delta_z)
        steps = delta_x / MOVE_STEP_LEN;
      else if (delta_x < delta_z)
        steps = delta_z / MOVE_STEP_LEN;
      else if (delta_x == 0 && delta_z == 0)
        steps = delta_angle / MOVE_STEP_LEN;
      // Расчет дельт шага
      delta_x = (x - ex_pos_x) / (double)steps;
      delta_z = (z - ex_pos_z) / (double)steps;
      delta_angle = (angle - ex_angle) / (double)steps;

      //adapt_vel(ex_pos_x, ex_pos_z, ex_angle, ex_pos_x + delta_x, ex_pos_z + delta_z, ex_angle + delta_angle);

    }
   /* if ( millis() - move_timer > MOVE_STEP_TIME)
    {
      move_timer = millis();*/
      steps--;
      //перемещение дискретное
      ex_pos_x += delta_x;
      ex_pos_z += delta_z;
      ex_angle += delta_angle;
      //   Serial1.println(String(steps) + ": " + String(ex_pos_x) + " : " + String(x) + " : " + String(ex_angle));

      inverse_kinematics(ex_pos_x, ex_pos_z, ex_angle);
//      dxl_wb.syncWrite("Goal_Position", goal_position);
  result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to sync goal_position");
  }


      if (steps <= 1) //На всякий случай дополнительный довод
      {
        ex_pos_x = x;
        planed_x = x;
        ex_pos_z = z;
        planed_z = z;
        ex_angle = angle;
        planed_angle = angle;
        inverse_kinematics(x, z, angle);
//        dxl_wb.syncWrite("Goal_Position", goal_position);
   result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to sync goal_position");
  }

        moving = 0;
      //}
    }
  }
  else
    moving = 0;
}
   //если кнопка нажата
  else{
    //устанавливаем статус в 2 для облака
    moving = 2;
   //  
     //dxl_wb.syncWrite(handler_index_torque, torque_on);
  }

}


void MoveHome()
{
  Serial1.println("#\nMoving to center position");
  allow_trajectory = 1;
  goal_position[5] = END_EFFECTOR_CLOSE;
  distance = L2;
  z = L1 + BASE_HEIGHT - L3;
  rotation = PI;
}


void MoveCenter()
{
  Serial1.println("#\nMoving to center position");
  allow_trajectory = 1;
  goal_position[5] = END_EFFECTOR_CLOSE;
  rotation = 230 / 180. * PI;
  distance = 220.;
  z = (CARGO_POS_Z + ABOVE_CARGO_Z) / 2;
}


void RelaxServos()
{
//Для логирования и контроля ошибок
const char *log;
bool result = false;
  
  allow_trajectory = 0;
  first_move = 1;
  distance = 0.;
  rotation = 0.;
  z = 0.;
  
//  dxl_wb.syncWrite("Torque_Enable", torque_off);
  result = dxl_wb.syncWrite(handler_index_torque, torque_off, &log);
  if (result == false)
  {
    Serial1.println(log);
    Serial1.println("#Failed to sync torque_off");
  }

  
  
}


void RedrawMenu()
{
  Serial1.println("\n################# g:240:220:0:0:1###################");
  Serial1.println("# Please enter option 1-7 to run");
  Serial1.println("# (h) Redraw Menu");
  Serial1.println("# (r) Run Sending Data");
  Serial1.println("# (s) Stop Sending Data");
  Serial1.println("# (p) Move RoboArm to position p:angle:distance:state:take");
  Serial1.println("# (1) Move to Start Position");
  Serial1.println("# (2) Move to Center");
  Serial1.println("# (3) Relax servos");
  Serial1.println("###################################");
}
