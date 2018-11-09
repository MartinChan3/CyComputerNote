#coding:utf-8

import numpy as np
a = []
for i in range(5):
    a.append(int(raw_input("Please input a number：")))
print a
a = np.array(a)
max_index = np.argmax(a)
min_index = np.argmin(a)
a[0], a[max_index] = a[max_index], a[0]
a[-1], a[min_index] = a[min_index], a[-1]
print a
