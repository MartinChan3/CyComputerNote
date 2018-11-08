#!/usr/bin/python
# -*- coding: UTF-8 -*-
 
Tn = 0
Sn = []
n = int(raw_input('n = '))
a = int(raw_input('a = '))
for count in range(n):
    Tn = Tn + a
    a = a * 10
    Sn.append(Tn)
    print Tn

#lambda表达式采取匿名函数
#reduce表示累积运算，第一个参数为需要进行累积的函数，第二参数为待选值
Sn = reduce(lambda x,y : x + y,Sn)
print "计算和为：",Sn

