import requests
import json
import sqlite3

conn = sqlite3.connect('f:/cinemas.db')
c = conn.cursor()
c.execute('create table if not exists cinemas(id int, name text, mark int, address text, sellPrice float, regionId int, regionName text)')
print(u'数据库已打开')

class Cinema: 
    def __init__(self, name, id, mark, address, sellPrice):
        self.name = name
        self.id = id 
        self.mark = mark
        self.address = address
        self.sellPrice = sellPrice

class RegionCinema:
    def __init__(self, regionId, regionName, regionPinyin):
        self.id = regionId
        self.name = regionName
        self.pinyin = regionPinyin
        self.cinemas = []
    def append(self, cinema):
        self.cinemas.append(cinema)

def main():
    list = get_city_list()
    fill_cinemas_info(list)
    
def get_city_list():
    jsondata = requests.get("https://maoyan.com/ajax/cities")
    j = json.loads(jsondata.content)

    domesticCinemas = []
    for key in j.keys():
        if ('letterMap' == key):
            letterMapInfo = j.get(key)
            if isinstance(letterMapInfo,dict):
                for key2 in letterMapInfo.keys():  #获取各字母开头
                    letterVal = letterMapInfo.get(key2)
                    if isinstance(letterVal, list): #获取各cities
                        for info in letterVal:
                            print(info["id"], info["nm"], info["py"] )
                            sRegionCinema = RegionCinema(info["id"], info["nm"], info["py"])
                            domesticCinemas.append(sRegionCinema)
    return domesticCinemas
                            
def fill_cinemas_info(list):
    for sItem in list: 
        tUrl = "https://m.maoyan.com/ajax/cinemaList?day=2018-09-21&offset=0&limit=2000&cityId=" + str(sItem.id)
        print('目前尝试爬取城市：'+ sItem.name)
        fill_single_city_info(tUrl, sItem)
        print('爬取完毕：' + sItem.name)
    return
        
def fill_single_city_info(url, sItem):
    jsondata = requests.get(url)
    jsonInfo = jsondata.json()
    for key in jsonInfo.keys():
        if ("cinemas" == key):
            cinemasInfo = jsonInfo.get(key)
            for item2 in cinemasInfo:
                sellPricep = 0
                if item2.__contains__('sellPrice'):
                    sellPricep = item2['sellPrice']
                sCinema = Cinema(item2["nm"], item2["id"], item2["mark"], item2["addr"], sellPricep)
                sItem.append(sCinema)
                # 写入数据库
                sellPricesql = 0.0
                if (sCinema.sellPrice != ''):
                    sellPricesql = float(sCinema.sellPrice)
                singleSqlData = [int(sCinema.id), sCinema.name, int(sCinema.mark), sCinema.address, sellPricesql, int(sItem.id), sItem.name]
                c.execute('insert into cinemas VALUES(?,?,?,?,?,?,?)', singleSqlData)
    return
                                          
if __name__ == "__main__":
    main()

conn.close()