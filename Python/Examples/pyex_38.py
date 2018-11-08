#!/usr/bin/python
# -*- coding: UTF-8 -*-

if __name__ == '__main__':
    a = []
    sum2 = 0.0
    for i in range(3):
        a.append([])
        for j in range(3):
            a[i].append(float(raw_input('input num:\n')))

    for i in range(3):
        sum2 += a[i][i]

    print sum2
