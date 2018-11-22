#!/usr/bin/python
# -*- coding: UTF-8 -*-

if __name__=='__main__':
    list1=[]
    str1=raw_input('请输入第一个字符串：')
    str2=raw_input('请输入第二个字符串：')
    str3=raw_input('请输入第三个字符串：')
    list1.extend([str1,str2,str3])
    list2=sorted(list1)
    print '排序后的字符串为：'
    for item in list2:
        print item