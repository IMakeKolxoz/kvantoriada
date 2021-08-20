#include <DynamixelWorkbench.h>
DynamixelWorkbench dxl_wb;

#if defined(__OPENCM904__)
	#define DEVICE_NAME "4"
#elif defined(__OPENCR__)
	#define DEVICE_NAME ""
#endif
#define PI 3.14
#define BAUDRATE 1000000
#define DXL_ID_6 6
#define DXL_ID_5 5
#define DXL_ID_4 4
#define DXL_ID_3 3
#define DXL_ID_2 2
#define DXL_ID_1 1



//Переменная для кнопки
#define button D10
uint8_t button_state = 0;
uint8_t button_last_state = 0;

uint8_t dxl_id[6] ={DXL_ID_1,DXL_ID_2,DXL_ID_3,DXL_ID_4,DXL_ID_5,DXL_ID_6};
uint8_t dxl_cnt = 6;

void setup()
{
Serial.begin(115200);
dxl_wb.begin(DEVICE_NAME, BAUDRATE);
dxl_wb.ping(dxl_id[0]);
dxl_wb.ping(dxl_id[3]);
dxl_wb.ping(dxl_id[1]);
dxl_wb.ping(dxl_id[2]);
dxl_wb.ping(dxl_id[4]);
dxl_wb.ping(dxl_id[5]);
dxl_wb.jointMode(dxl_id[0],20,100);
dxl_wb.jointMode(dxl_id[1],20,100);
dxl_wb.jointMode(dxl_id[2],20,100);
dxl_wb.jointMode(dxl_id[3],30,100);
dxl_wb.jointMode(dxl_id[4],60,100);
dxl_wb.jointMode(dxl_id[5],50,100);
dxl_wb.goalSpeed(DXL_ID_1, 300);
dxl_wb.goalSpeed(DXL_ID_2, 300);
dxl_wb.goalSpeed(DXL_ID_3, 300);
dxl_wb.goalSpeed(DXL_ID_4, 300);
dxl_wb.goalSpeed(DXL_ID_5, 300);
dxl_wb.goalSpeed(DXL_ID_6, 300);


}
void loop()
{
 
//Для логирования и контроля ошибок
const char *log;
bool result = false;
  
  //Блок отсылки данных по фазам
  if ((millis() - packet_timer >= PACKET_DELAY) && SendMode)
  {
    packet_timer = millis();
    Serial.print(String(is_moving()) + ":");
    switch (packet_phase % 6)
    {
      case 1: case 3:
        get_data_load(data_packet, "Present_Load");
        Serial.print("L:");
        break;
      case 5:
        get_data_temp(data_packet, "Present_Temperature");
        Serial.print("T:");
        break;
      default:
        get_data_pos(data_packet, "Present_Position");
        Serial.print("M:");
    }
    //  Serial.println(dxl_wb.itemRead(dxl_id[0], "Present_Temperature"));
    print_packet(data_packet);
    packet_phase++;
  }

  /***************НАЧАЛО БЛОКА ЧТЕНИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА*******************************/
  if (Serial.available())
  {
    float buf_distance = 0.;
    float buf_rotation = 0.;
    float buf_z = 0.;
    float buf_fi = 0.;
    char c = Serial.read();

    //Начинаем парсинг пакета
    switch (c)
    {
      case '1':
        MoveHome();
        Serial.println("\nMoving to start position");
        break;

      case '2':
        MoveCenter();
        Serial.println("\nMoving to center position");
        break;

      case '3':
        RelaxServos();
        Serial.println("\nMoving to safe position and relaxing");
        break;

      case 'h':
        RedrawMenu();
        break;

      case '\n':    //Игнорим символ новой строки
        break;

      case 'r':
        SendMode = 1;
        Serial.println("\nSending activated.");
        RedrawMenu();
        break;

      case 's':
        SendMode = 0;
        Serial.println("\nSending stoped.");
        RedrawMenu();
        break;

      case 'g':
        /***************НАЧАЛО ПАРСЕРА*******************************/

        if ( (c = Serial.read()) == ':' )
        {
          while ((c = Serial.read()) != ':')
          {
            buf_rotation *= 10;
            buf_rotation += (int)(c - '0');
            if (buf_rotation > 1000)
              break;
          }
          if ((buf_rotation > MAX_ROT) || (buf_rotation < MIN_ROT))
          {
            Serial.println("\nRotation out of bounds. Max: " + String(MAX_ROT) + " Min: " + String(MIN_ROT));
            // Очищаем буфер
            while (Serial.available() > 0)
              Serial.read();
          }
          else
          {
            while ((c = Serial.read()) != ':' )
            {
              buf_distance *= 10;
              buf_distance += (int)(c - '0');
              if ( buf_distance > 1000)
                break;
            }


            if ((buf_distance > MAX_DIST) || (buf_distance < MIN_DIST))
            {
              Serial.println("\nDistance out of bounds. Max: " + String(MAX_DIST) + " Min: " + String(MIN_DIST));
              while (Serial.available() > 0)
                Serial.read();
            }
            else
            {
              int sign = 1;
              if ((c = Serial.read()) == '-')
                sign = -1;
              else
              {
                buf_fi *= 10;
                buf_fi += (int)(c - '0');
              }
              while ((c = Serial.read()) != ':' )
              {
                buf_fi *= 10;
                buf_fi += (int)(c - '0');
                if ( buf_fi > 1000)
                  break;
              }
              buf_fi = sign * buf_fi;
              if ((fabs(buf_fi) > MAX_FI) || (fabs(buf_fi) < MIN_FI))
              {
                Serial.println("\nEnd-effector rotation out of bounds. Max module: " + String(MAX_FI) + " Min: " + String(MIN_FI));
                while (Serial.available() > 0)
                  Serial.read();
              }
              else
              {
                if ((c = Serial.read()) == '1')
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
                if ((c = Serial.read()) == ':')
                {
                  if ((c = Serial.read()) == '0')
                    goal_position[sync_order(GRIPPER)] = END_EFFECTOR_OPEN;
                  else if (c == '1')
                    goal_position[sync_order(GRIPPER)] = END_EFFECTOR_CLOSE;
                  if ((c = Serial.read()) == '#')
                  {
                    // Движение возможно только после полностью корректного пакета
                    Serial.println("\nExecuting");
                    fi = (buf_fi + 180.) / 180. * PI;
                    goal_position[4] = angle_to_pos(fi, 'A');
                    //  Serial.println(String(dxl_id[GRIPPER - 1]) + ": " + String(goal_position[sync_order(GRIPPER - 1)]));
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
                    Serial.println("\nIncorrect packet: No # symbol");
                    while (Serial.available() > 0)
                      Serial.read();
                  }
                  break;
                }
                else
                {
                  Serial.println("\nIncorrect packet: No gripper command");
                  while (Serial.available() > 0)
                    Serial.read();
                }
              }
            }
          }
        }
        else
        {
          Serial.println("\nIncorrect packet");
          while (Serial.available() > 0)
            Serial.read();
        }
        break;
      /***************КОНЕЦ ПАРСЕРА*******************************/

      default:
        Serial.println("\nIncorrect packet");
        while (Serial.available() > 0)
          Serial.read();
    }
  }
  /***************КОНЕЦ БЛОКА ЧТЕНИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА*******************************/
  //Serial.println(goal_position[4]);
  if ((distance != 0) && (z != 0) && (rotation != 0))
    if ( millis() - move_timer > MOVE_STEP_TIME)
    {
      move_timer = millis();
      TrajectoryPlanning(distance, z, rotation);
    }
}
void set_pos(float p1,float p2,float p3,float p4,float p5,float p6, int del)
{
dxl_wb.goalPosition(dxl_id[0],p1);
dxl_wb.goalPosition(dxl_id[1],p2);
dxl_wb.goalPosition(dxl_id[2],p3);
dxl_wb.goalPosition(dxl_id[3],p4);
dxl_wb.goalPosition(dxl_id[4],p5);
dxl_wb.goalPosition(dxl_id[5],p6);
delay(del);
//while 
}

void start_pos()
{
set_pos(0,0,0,0,0,0.8,5000);
set_pos(PI/2, -PI/6, 0, 0, 0, 0.8, 1000);
}


void center_pos()
{
set_pos(0,0,0,0,0,0.8,5000);
set_pos(PI, -PI/6, 0, 0, 0, 0.8, 1000);
}
