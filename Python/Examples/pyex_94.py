#encoding=utf-8
import time
import random
start=time.time()
while True:
    play=int(raw_input('play the game(1/0)?'))
    if play==1:
        number=random.randint(0,1000)
        guess=int(raw_input('guess a number: '))
        while True:
            if number>guess:
                guess=int(raw_input("guess a bigger number: "))
            elif number<guess:
                guess=int(raw_input("guess a smaller number: "))
            else:
                end=time.time()
                print "bingo! "
                print u"%0.2fs猜中"%(end-start)
                break
    else:
        break