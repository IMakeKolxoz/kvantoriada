import serial
import time
import requests
from bs4 import BeautifulSoup

a = 1
old_result = 0
ser = serial.Serial('COM8', 9600)
print(ser.name)
time.sleep(2)
response = requests.get("")  # тут указываем адрес сайт(мы вводили адрес локального сайта)
soup = BeautifulSoup(response.content, 'html.parser')
result = soup.body.string
while a != 999:
    if result == old_result:
        print('Ошибка')
    else:
        ser.write(result)
        result = old_result
ser.close()
