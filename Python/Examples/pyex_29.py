#!/usr/bin/python
# -*- coding: UTF-8 -*-

inputNum = int(raw_input('Enter in a num '))
rev = []
while 1:
    remainder = inputNum % 10
    rev.append(remainder)
    inputNum /= 10
    if inputNum == 0:
        break;

print 'The size is %d and the number is' % len(rev)
print rev
