#!/usr/bin/python
# -*- coding: UTF-8 -*-

for i in range(2,10000):
    if 0  not in [i%n for n in range(2,i)]:
        print i,
