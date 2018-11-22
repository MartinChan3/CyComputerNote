#coding:utf-8

if __name__ == '__main__':
    N = 3
    arr = []
    for i in range(N):
        arr.append(raw_input('Please input a number:\n'))

    for i in arr[::-1]:
        print i
