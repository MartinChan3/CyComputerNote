#include <stdio.h>
#include <vector>
#include <iostream>

//02: replace space
void replaceSpace(char *str, int length){
    int spaceCount = 0;
    for (int i = 0; i < length; i++)
    {
        if (str[i] == ' ')
            spaceCount++;
    }

    if (!spaceCount) return;

    char *newStr = new char[length + 2 * spaceCount];
    int j = 0;
    for (int i = 0; i < length; i++)
    {
        char t = str[i];
        if (t == ' ')
        {
            newStr[j++] = '%';
            newStr[j++] = '2';
            newStr[j++] = '0';
        }
        else
        {
            newStr[j++] = t;
        }
    }

    str = newStr;
    return;
}

//核心：倒序追赶
void replaceSpace2(char *str, int length){
    int blankNumber = 0;
    int oldstringLen;
    for (oldstringLen = 0; str[oldstringLen] != '\0'; oldstringLen++){

        if (str[oldstringLen] == ' ')
            blankNumber++;
    }

    int newstringLen = oldstringLen + blankNumber * 2;
    if (newstringLen > length)
        return;
    str[newstringLen] = '\0';

    //设置两个指针point1和point2分别指向原字符串和新字符串的末尾位置
    int point1 = oldstringLen - 1, point2 = newstringLen - 1;

    while (point1 >= 0 && point2 > point1){
        if (str[point1] == ' '){
            str[point2--] = '0';
            str[point2--] = '2';
            str[point2--] = '%';
        }
        else
            str[point2--] = str[point1];

        point1--;
    }
}

void main()
{
    char c[] = "Time flys quickly";
    std::cout << c << std::endl;
    replaceSpace2(&(c[0]), 1000);
    std::cout << c << std::endl;
}
