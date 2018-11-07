# -*- coding: UTF-8 -*-
import datetime
if __name__ == '__main__':
    #输出当天日期
    print(datetime.date.today().strftime('%d%m%Y'))
    #创建日期对象
    miyazakiBirthDate = datetime.date(1941,1,5)
    print (miyazakiBirthDate.strftime('%d%m%Y'))
    #日期计算算数
    miyazakiBirthNextDay = miyazakiBirthDate + datetime.timedelta(days = 10)
    print(miyazakiBirthNextDay.strftime('%d%m%Y'))
    #日期替换
    miyazakiFirstBirthday = miyazakiBirthDate.replace(year=miyazakiBirthDate.year+1)
    print (miyazakiFirstBirthday.strftime('%d%m%Y'))
