from requests_html import HTMLSession
import requests


# 保存图片到bg/目录
def save_image(url, title):
    img_response = requests.get(url)
    with open('./bg/'+title+'.jpg', 'wb') as file:
        file.write(img_response.content)

# 背景图片地址，这里选择1920*1080的背景图片
url = "http://www.win4000.com/wallpaper_2358_0_10_1.html"

session = HTMLSession()
r = session.get(url)

# 查找页面中背景图，找到链接，访问查看大图，并获取大图地址
items_img = r.html.find('ul.clearfix > li > a')
for img in items_img:
    img_url = img.attrs['href']
    if "/wallpaper_detail" in img_url:
        r = session.get(img_url)
        item_img = r.html.find('img.pic-large', first=True)
        url = item_img.attrs['src']
        title = item_img.attrs['title']
        print(url+title)
        save_image(url, title)