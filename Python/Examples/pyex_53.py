#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 异或运算符为^

if __name__ == '__main__':
    a = 077
    b = a ^ 3
    print 'The a ^ 3 = %d' % b
    b ^= 7
    print 'The a ^ b = %d' % b
