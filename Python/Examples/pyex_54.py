#!/usr/bin/python
# -*- coding: UTF-8 -*-
 
if __name__ == '__main__':
    a = int(raw_input('input a number:\n'))
    b = a >> 4
    c = ~(~0 << 4)
    d = b
    print '%o\t%o' %(a,d)
