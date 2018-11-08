#!/usr/bin/python
#  -*- coding:UTF-8 -*-

def varfunc():
    var = 0
    print 'var = %d' % var
    var += 1

if __name__ == '__main__':
    for i in range(3):
        varfunc()

class Static:
    StaticVar = 5
    def varfunc(self):
        self.StaticVar += 1
        print self.StaticVar

print Static.StaticVar
a = Static()
for i in range(3):
    a.varfunc()

# 讲道理Python的类初始化机制有些混乱：如果不是自身生成的初始化类型，那么会和整个类一起
# 共用（直到第一次有效操作后脱离？？？），这会产生非常大的问题
