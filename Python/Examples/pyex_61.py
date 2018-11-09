#! /usr/bin/env python
# coding:utf-8

n = 30
def lst(i,j):
    if i==j or j==1:
        return 1
    else:
        return lst(i-1,j-1) + lst(i-1,j)
for i in range(1,n+1):
    for j in range(1,i+1):
        print lst(i,j),
    print
