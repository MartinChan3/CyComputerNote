#include <vector>
#include <unordered_map>
#include <iostream>

using namespace std;

//�������� �ǿ� ������������ʾ�����Ǹ������������У����Ǹ��Ե�λ���ǰ�������ķ�ʽ�洢�ģ��������ǵ�ÿ���ڵ�ֻ�ܴ洢һλ���֡�
//��������ǽ��������������������᷵��һ���µ���������ʾ���ǵĺ͡�
//�����Լ���������� 0 ֮�⣬���������������� 0 ��ͷ��
//ʾ����
//���룺(2 -> 4 -> 3) + (5 -> 6 -> 4)
//�����7 -> 0 -> 8
//ԭ��342 + 465 = 807



//������ListNode��㣬��nextָ���ָ��һλ
 struct ListNode {
     int val;
     ListNode *next;
     ListNode(int x) : val(x), next(NULL) {}
 };


class Solution {
public:
	ListNode* addTwoNumbers(ListNode* l1, ListNode* l2){
		ListNode *dummy = new ListNode(-1), *cur = dummy; //����ʹ����һ��С���ɣ���������һ��-1λ��Ϊͷ�����������������ѭ������д
		int carry = 0; //��λ
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