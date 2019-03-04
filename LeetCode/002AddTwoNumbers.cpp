#include <vector>
#include <unordered_map>
#include <iostream>

using namespace std;

//给出两个 非空 的链表用来表示两个非负的整数。其中，它们各自的位数是按照逆序的方式存储的，并且它们的每个节点只能存储一位数字。
//如果，我们将这两个数相加起来，则会返回一个新的链表来表示它们的和。
//您可以假设除了数字 0 之外，这两个数都不会以 0 开头。
//示例：
//输入：(2 -> 4 -> 3) + (5 -> 6 -> 4)
//输出：7 -> 0 -> 8
//原因：342 + 465 = 807



//常见的ListNode结点，用next指针代指下一位
 struct ListNode {
     int val;
     ListNode *next;
     ListNode(int x) : val(x), next(NULL) {}
 };


class Solution {
public:
	ListNode* addTwoNumbers(ListNode* l1, ListNode* l2){
		ListNode *dummy = new ListNode(-1), *cur = dummy; //这里使用了一个小技巧，它引出了一个-1位作为头部，这样方便接下来循环的书写
		int carry = 0; //进位
		while (l1 || l2){
			int val1 = l1 ? l1->val : 0;
			int val2 = l2 ? l2->val : 0;
			int sum = val1 + val2 + carry;
			carry = sum / 10;
			cur->next = new ListNode(sum % 10);
			cur = cur->next;
			if (l1) l1 = l1->next;
			if (l2) l2 = l2->next;
		}
		if (carry) cur->next = new ListNode(1);
		return dummy->next;
	}
};

void main()
{
	ListNode a1(2), a2(4), a3(3);
	ListNode b1(5), b2(6), b3(4);
	a1.next = &a2;
	a2.next = &a3;
	b1.next = &b2;
	b2.next = &b3;
	Solution solution;
	ListNode *result = solution.addTwoNumbers(&a1, &b1);
	while (result)
	{
		cout << result->val;
		result = result->next;
	}
}