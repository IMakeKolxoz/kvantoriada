# устанавливаем библиотеку
import math

# настройки манипулятор
shoulder_1 = 210  # в миллиметрах
shoulder_2 = 135  # в миллиметрах
base = 155  # расстояние от дна до сервы в миллиметрах
# получаем координаты
x = float(input('Введите X: '))
y = float(input('Введите Y: '))
z = float(input('Введите Z: '))
# модуль х и у чтобы был адекватный расчет(наверное)(можно было использовать готовую функцию, но я решил поиграться)
x = x if x > 0 else -x
y = y if y > 0 else -y
# находим расстояние от основания манипулятора до заданной точки
side_1 = ((x * x) + (y * y)) ** (0.5)
side_2 = z - base
side_2 = side_2 if side_2 > 0 else -side_2  # пока так, если z будет меньше 155 то значение стороны будет с минусом(не знаю надо ли это)
side_3 = ((side_1 * side_1) + (side_2 * side_2)) ** (0.5)
# проверка на существование треугольника
if (shoulder_1 + shoulder_2 > side_3) and (shoulder_2 + side_3 > shoulder_1) and (
        shoulder_1 + side_3 > shoulder_2):  # сумма двух сторон треугольника должны быть больше 3 стороны
    a = shoulder_1
    b = shoulder_2
    c = side_3
    # находим косинус углов
    cos_a = (b * b + c * c - a * a) / (2 * b * c)
    cos_b = (a * a + c * c - b * b) / (2 * c * a)
    cos_c = (b * b + a * a - c * c) / (2 * a * b)
    # переводим косинус в радианы и округляем до сотых
    radian_a = round(math.acos(cos_a), 2)
    radian_b = round(math.acos(cos_b), 2)
    radian_c = round(math.acos(cos_c), 2)
    # выводим значения
    print('radian_a = ', + radian_a)
    print('radian_b = ', + radian_b)
    print('radian_c = ', + radian_c)

else:
    print('какая-то ошибка')
