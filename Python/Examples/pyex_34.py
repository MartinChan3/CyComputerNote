#!/usr/bin/python
# -*- coding: UTF-8 -*-

def hello_world():
    print 'hello world'

def three_hellos():
    for i in range(3):
        hello_world()
# 这里__name__是用py编译器代指本身模块的宏
# 这里区分使用为单个模块调用，还是自身执行
# 因为python很多模块都是能够自身调用的，这点和C区别非常大
if __name__ == '__main__':
    three_hellos()
