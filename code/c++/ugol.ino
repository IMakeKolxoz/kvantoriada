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
 /***************НАЧАЛО БЛОКА ЧТЕНИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА*******************************/
  
  if (Serial.available())
  {
    char c = Serial.read();

    //Начинаем парсинг пакета
    if (digitalRead(button) != 1)
    {
    switch (c)
    {
      case '1':
        start_pos();
        Serial.println("#\nMoving to start position");
        break;

      case '2':
        center_pos();
        Serial.println("#\nMoving to center position");
        break;

    }
    }
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
