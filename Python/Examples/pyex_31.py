#!/usr/bin/python
# -*- coding: UTF-8 -*-

import re

def judge(first,list):
    li=[]
    first = first.upper()
    for a in list:
        if re.match(first,a):
            li.append(a)
    if len(li) == 1:
        print li[0]
    else:
        second = raw_input('请输入第二个字母')
        for b in li:
            if re.match(first + second,b):
                print b

list=['Monday','Tuesday','Wednesday','Thursday','Friday','Saturday','Sunday']
first=raw_input('请输入第一个字母：')
judge(first,list)
