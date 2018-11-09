#!/usr/bin/python
# -*- coding: UTF-8 -*-

if __name__ == '__main__':
    n = int (raw_input('整数n为:\n'))
    m = int (raw_input('往后移动的数字为:\n'))

    def move(array,n,m):
        array_end = array[n-1]
        for i in range(n-1,-1,-1):
            array[i] = array[i-1]
        array[0] = array_end
        m -= 1
        if m > 0: move(array,n,m)

    number = []
    for i in range(n):
        number.append(int(raw_input('输入一个数字：\n')))
    print '原始列表：', number

    move(number, n, m)
    print '移动之后：', number

# 思路扩展：Python中可以直接用暴力拆解法，截取前一半和后一半，然后再倒序拼接=。=太强了
