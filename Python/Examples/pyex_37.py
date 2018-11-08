#!usr/bin/python
# -*- coding: UTF-8 -*-

print 'Please enter into 10 numbers'
a = []
for n in range(10):
    a.append(int(raw_input('Please enter the %d number: ' % n)))
a.sort(reverse = False)
print a
