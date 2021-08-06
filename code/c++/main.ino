#include <math.h>                                    // для функции acos
// настройки манипулятор
const int shoulder_1 = 210;  // в миллиметраx
const int shoulder_2 = 135;  // в миллиметрах
const int base = 155;  // расстояние от дна до сервы в миллиметрах
//объявляем все переменные
///координаты
float x;  
float y;
float z;
///стороны треугольника №1
float side_1;  
float side_2;
float side_3;
///стороны треугольника №2
float shoulder_1, shoulder_2;
//косинус и углы в радианах
float cos_a;  
float cos_b;
float cos_c;
float radian_a;  
float radian_b;
float radian_c;
float a;  
float b;
float c;
//
void setup() {
  x = 150;
  y = 250;
  z = 300;
  Serial.begin(9600);
}

void loop() {
  // находим расстояние от основания манипулятора до заданной точки
  side_1 = sqrtf((x * x) + (y * y));
  side_2 = fabs(z - base);
  side_3 = sqrtf((side_1 * side_1) + (side_2 * side_2);
  // проверка на существование треугольника
  if ((shoulder_1 + shoulder_2 > side_3) && (shoulder_2 + side_3 > shoulder_1) && (shoulder_1 + side_3 > shoulder_2))
  {
    a = shoulder_1;
    b = shoulder_2;
    c = side_3;
    //находим косинус углов
    cos_a = (b * b + c * c - a * a) / (2 * b * c);
    cos_b = (a * a + c * c - b * b) / (2 * c * a);
    cos_c = (b * b + a * a - c * c) / (2 * a * b);
    //переводим косинус в радианы и округляем до сотых
    radian_a = math.acos(cos_a);
    radian_b = math.acos(cos_b);
    radian_c = math.acos(cos_c);
    Serial.println("radian_a= " radian_a);
    Serial.println("radian_b= " radian_b);
    Serial.println("radian_c= " radian_c);
  }
  else:
  {
    Serial.println("Ошибка");
}
}
