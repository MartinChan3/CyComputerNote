#coding:utf-8

if __name__ == '__main__':
    a = [1,3,2]
    b = [3,4,5]
    a.sort()
    print a
    
    print a + b

    a.extend(b)
    print a