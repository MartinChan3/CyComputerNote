#!/usr/bin/python
# -*- coding: UTF-8 -*-

def sumFraction(size):
    rSum = 0.0
    numerator = 2
    denominator = 1
    if (size <= 0):
        print 'the size is wrong'
        return rSum
    rSum += numerator / denominator
    if (size == 1):
        return rSum
    
    for i in range(2 ,size + 1):
        nNumerator = numerator + denominator
        nDenominator = numerator
        numerator = nNumerator
        denominator = nDenominator
        rSum += float(nNumerator) / float(nDenominator)
    return rSum

print sumFraction(20)
