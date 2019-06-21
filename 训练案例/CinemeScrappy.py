import requests

res = requests.get('http://m.maoyan.com/imeituan/').text 
print(res)