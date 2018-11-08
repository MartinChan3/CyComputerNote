#!/usr/bin/python
# -*- coding: UTF-8 -*-

def power(x):
    if x ** 2 >= 50:
        print('{}的平方为:{},不小于50，继续'.format(x, x ** 2))
    else:
        print('{}的平方为:{},小于50，退出'.format(x, x ** 2))
        quit()
while True:
    x = int(input('输入数字:'))
    power(x)
