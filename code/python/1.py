import requests
from bs4 import BeautifulSoup
response = requests.get("https://imakekolxoz.github.io/test/")
soup = BeautifulSoup(response.content, 'html.parser')
print(soup.body.string)