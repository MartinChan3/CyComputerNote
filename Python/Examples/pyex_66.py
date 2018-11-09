#!/usr/bin/python
# -*- coding: UTF-8 -*-

if __name__=='__main__':
    l=[]
    for i in range(3):
        x=raw_input('输入一个数字:')
        l.append(x)
    for i in range(3):
        print(max(l))
        l.remove(max(l))  #利用 remove（）函数依次输出最大值
