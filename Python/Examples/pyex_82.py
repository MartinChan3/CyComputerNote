#!/usr/bin/python
# -*- coding: UTF-8 -*-
 
if __name__ == '__main__':
    n = 0
    p = raw_input('input a octal number:\n')
    # This place is very effective way to write
    for i in range(len(p)):
        n = n * 8 + ord(p[i]) - ord('0')
    print n
