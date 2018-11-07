#!/usr/bin/python
# -*- coding: UTF-8 -*-

score = int(raw_input('输入分数：\n'))
if score >= 90:
    grade = 'A'
elif score >= 60:
    grade = 'B'
else:
    grade = 'C'

print '%d属于%s' % (score,grade)
        
