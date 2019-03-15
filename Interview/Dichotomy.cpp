#include <stdio.h>
#include <vector>
#include <iostream>

//Dicho find a number in one-dimension array
int FindDicho(std::vector<int> arr, int target, int si, int ei)
{
    if (si > ei || target < arr[si] || target > arr[ei])
        return -1;
    if (si == ei && arr[si] != target)
        return -1;
    int middle = (si + ei) / 2;
    if (arr[middle] == target)
        return middle;
    if (arr[middle] < target)
        return FindDicho(arr, target, middle + 1, ei);
    else
        return FindDicho(arr, target, si, middle - 1);
}

//No recrusion way
int FindDicho(std::vector<int> arr, int target)
{
    int size = arr.size();
    if (!size)
        return -1;
    if (target < arr[0] || target > arr[size - 1])
        return -1;
    int si = 0; int ei = size - 1;
    while (si <= ei)
    {
        if (si == ei)
        {
            if (arr[si] == target)
                return si;
            else
                return -1;
        }

        int middle = (si + ei) / 2;
        if (arr[middle] == target)
            return middle;
        else if (arr[middle] < target)
            si = middle + 1;
        else
            ei = middle - 1;
    }
    return -1;
}

//QuickSort:gurad walk-in rule
void QuickSort(std::vector<int> &arr, int left, int right)
{
    if (left >= right)
        return;
    int gs, ge;
    gs = left;
    ge = right;
    int stand = arr[gs];
    while (gs < ge)
    {
        while (gs < ge && arr[ge] >= stand) //First due to exchange?
            ge--;
        while (gs < ge && arr[gs] <= stand)
            gs++;
        if (gs < ge)
            std::swap(arr[gs], arr[ge]);
    }

    //Put stand into the current pos(like change two nums)
    arr[left] = arr[gs];
    arr[gs] = stand;

    //Recurse
    QuickSort(arr, left, gs - 1);
    QuickSort(arr, gs + 1, right);
}


int main()
{
    std::vector<int> arr = {1,2,3,4,5,6,7,8};
    for (int i = 1; i <= 9; i++)
        //std::cout << FindDicho(arr, i, 0, arr.size()-1) << std::endl;
        std::cout << FindDicho(arr, i) << std::endl;


    std::cout << "QuickSort" << std::endl;
    std::vector<int> arrMess = {8, 12, 1, 51, 21};
    QuickSort(arrMess, 0, arrMess.size() - 1);
    for (auto i : arrMess)
    {
        std::cout << i << std::endl;
    }
    return 0;
}
