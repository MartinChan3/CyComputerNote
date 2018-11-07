# -*- coding: UTF-8 -*-
def reduceNum(n):
    print '{}='.format(n),
    if not isinstance(n,int) or n <= 0:
        print '请输入正确的数字！'
        exit(0)
    elif n in [1]:
        print '{}'.format(n)
    while n not in [1]: # 循环保证递归
        for index in xrange(2,n + 1):
            if n % index == 0:
                n /= index # n等于n/index
                if n == 1:
                    print index
                else:
                    print '{} * '.format(index),
                break

reduceNum(90)
reduceNum(100)
