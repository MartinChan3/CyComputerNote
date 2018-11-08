#!/usr/bin/python
# -*- coding: UTF-8 -*-

a = int(raw_input('input a num '))
x = str(a)
flag = True
for i in range(len(x)/2):
    if x[i] != x[ -i - 1]:
    #python允许负数索引，来进行索引的倒序指定,-1为右侧第一个索引
        flag = False
        break
    
print flag

