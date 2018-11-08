#!/usr/bin/python
# -*- coding: UTF-8 -*-

from sys import stdout
for i in range(1,8):
    spaceNum = abs(i -4)
    starNum = 7 - 2 *spaceNum
    for j in range(spaceNum):
        stdout.write(' ')
    for j in range(starNum):
        stdout.write('*')
    for j in range(spaceNum):
        stdout.write(' ')
    stdout.write('\n')
    
