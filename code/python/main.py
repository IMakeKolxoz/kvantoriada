import serial
import time
import requests
from bs4 import BeautifulSoup

url = 'https://quotes.toscrape.com/'
response = requests.get(url)
soup = BeautifulSoup(response.text, 'lxml')
quotes = soup.find_all('span', class_='text')

for quote in quotes:
    print(quote.text)

a = 1

ser = serial.Serial('COM8', 9600)
print(ser.name)
time.sleep(2)
while a != 999:
    a = input()
    b = ser.read()
    print(b)
ser.close
