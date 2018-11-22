#!/usr/bin/python
# -*- coding: UTF-8 -*-
 
if __name__ == '__main__':
    oddNumber = int(raw_input('Input an odd number\n'))

    nineCount = 0
    flagExactDivision = False
    while flagExactDivision == False:
        nineCount = nineCount * 10 + 9
        if (0 == (nineCount % oddNumber)):
            flagExactDivision = True
        if (nineCount >= 10E20):
            print 'Its too much'
            break

    print nineCount

