#coding:utf-8

def fodd(oddNum):
    total = 0.0
    for i in range(1,oddNum+1,2):
        total += 1.0 / float(i)
    return total

def feven(evenNum):
    total = 0.0
    for i in range(2,evenNum+1,2):
        total += 1.0 / float(i)
    return total

if __name__ == '__main__':
    inputNum = int(raw_input('Please input a positive integer\n'))
    if (inputNum % 2):
        print fodd(inputNum)
    else:
        print feven(inputNum)
