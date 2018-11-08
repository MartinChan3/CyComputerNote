#!/usr/bin/python
# -*- coding: UTF-8 -*-

def fact(j):
    sum1 = 0
    if j == 0:
        sum1 = 1
    else:
        sum1 = j * fact(j - 1)
    return sum1

print fact(5)
