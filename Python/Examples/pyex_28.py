#!/usr/bin/python
# -*- coding: UTF-8 -*-

def age(n):
    if n == 1: c =10
    else: c = age(n - 1) + 2
    return c
print age(5)
