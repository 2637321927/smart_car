#include "LQ_List_drv.h"

/********************************************************************************
 * @brief   初始化链表
 * @param   list 链表指针
 * @return  无返回值
 * @date    2025/11/19
 ********************************************************************************/
void lq_list_init(LQ_List *list)
{
    if (unlikely(list == NULL)) {   // unlikely宏用于判断条件是否为假，如果是假的，则执行后面的代码
        pr_warn("list_init is list=NULL\n");
        return;
    }
    list->head = NULL;
}

/********************************************************************************
 * @brief   判断某个数据是否在链表中
 * @param   list 链表指针
 * @param   data 需要判断的数据
 * @return  如果数据在链表中，返回true；否则返回false
 * @date    2025/11/19
 ********************************************************************************/
bool lq_list_contains(LQ_List *list, uint8_t data)
{
    LQ_Node *curr_node;
    if (unlikely(list == NULL) || list->head == NULL)
        return false;
    
    curr_node = list->head;
    while (curr_node != NULL) {
        if (curr_node->data == data) {
            return true;
        }
        curr_node = curr_node->next;
    }
    return false;
}

/********************************************************************************
 * @brief   向链表中添加数据
 * @param   list 链表指针
 * @param   data 需要添加的数据
 * @return  如果添加成功，返回1；如果数据已存在，返回0；如果内存分配失败，返回-2；如果参数不合法，返回-1
 * @date    2025/11/19
 ********************************************************************************/
int lq_list_add(LQ_List *list, uint8_t data)
{
    LQ_Node *new_node;
    // 参数合法性校验
    if (unlikely(list == NULL))
        return -1;
    // 去重检查
    if (lq_list_contains(list, data))
        return 0;
    // 分配内存
    new_node = kmalloc(sizeof(LQ_Node), GFP_KERNEL);
    if (unlikely(new_node == NULL))
        return -2;
    
    // 初始化新节点
    new_node->data = data;
    new_node->next = NULL;
    // 若链表为空，则新节点即为头结点
    if (list->head == NULL) {
        list->head = new_node;
    } else {
        // 遍历到链表末尾，将新节点添加到末尾
        LQ_Node *cur_node = list->head;
        while (cur_node->next != NULL) {
            cur_node = cur_node->next;
        }
        cur_node->next = new_node;
    }
    return 1;
}

/********************************************************************************
 * @brief   从链表中删除数据
 * @param   list 链表指针
 * @param   data 需要删除的数据
 * @return  如果删除成功，返回1；如果数据不存在，返回0；如果参数不合法，返回-1
 * @date    2025/11/19
 ********************************************************************************/
int lq_list_remove(LQ_List *list, uint8_t data)
{
    LQ_Node *curr_node, *prev_node;

    if (unlikely(list == NULL) || list->head == NULL)
        return -1;
    
    curr_node = list->head;
    prev_node = NULL;

    // 查找要删除的节点
    while (curr_node != NULL && curr_node->data != data) {
        prev_node = curr_node;
        curr_node = curr_node->next;
    }
    // 未找到数据
    if (curr_node == NULL)
        return 0;
    // 删除节点
    if (prev_node == NULL) {
        list->head = curr_node->next;
    } else {
        prev_node->next = curr_node->next;
    }
    kfree(curr_node);
    return 1;
}

/********************************************************************************
 * @brief   获取链表中的数据个数
 * @param   list 链表指针
 * @return  链表中的数据个数；如果参数不合法，返回0
 * @date    2025/11/19
 ********************************************************************************/
int lq_list_count(LQ_List *list)
{
    LQ_Node *curr_node;
    int count = 0;

    if (unlikely(list == NULL))
        return 0;
    
    curr_node = list->head;
    while (curr_node != NULL) {
        count++;
        curr_node = curr_node->next;
    }
    return count;
}

/********************************************************************************
 * @brief   清空链表
 * @param   list 链表指针
 * @return  无返回值
 * @date    2025/11/19
 ********************************************************************************/
void lq_list_clear(LQ_List *list)
{
    LQ_Node *curr_node, *next_node;

    if (unlikely(list == NULL || list->head == NULL))
        return;

    curr_node = list->head;
    while (curr_node != NULL) {
        next_node = curr_node->next;
        kfree(curr_node);
        curr_node = next_node;
    }

    list->head = NULL;
}

/********************************************************************************
 * @brief   比较两个链表是否相等
 * @param   list1 链表1指针
 * @param   list2 链表2指针
 * @return  如果两个链表相等，返回1；否则返回0
 * @date    2025/11/20
 ********************************************************************************/
uint8_t lq_list_compare(LQ_List *list1, LQ_List *list2)
{
    int len1, len2;
    LQ_Node *curr_node;
    // 两个均为NULL，视为相同;一个为NULL，另一个不为NULL，视为不同
    if (unlikely(list1 == NULL && list2 == NULL)) return 1;
    if (unlikely(list1 == NULL || list2 == NULL)) return 0;
    // 两个均为空链表，视为相同
    if (unlikely(list1->head == NULL && list2->head == NULL))
        return 1;
    // 一个为空链表，另一个不为空，视为不同
    if (unlikely(list1->head == NULL || list2->head == NULL))
        return 0;
    // 比较长度，长度不同，视为不同
    len1 = lq_list_count(list1);
    len2 = lq_list_count(list2);
    if (len1 != len2)
        return 0;
    // 比较每个节点的数据
    curr_node = list1->head;
    while (curr_node != NULL)
    {
        if (lq_list_contains(list2, curr_node->data) == false)
            return 0;
        curr_node = curr_node->next;
    }
    return 1;
}

/********************************************************************************
 * @brief   交换两个链表
 * @param   list1 链表1指针
 * @param   list2 链表2指针
 * @return  无返回值
 * @note    调试使用
 * @date    2025/11/20
 ********************************************************************************/
int lq_list_swap(LQ_List *list1, LQ_List *list2)
{
    LQ_Node *temp;
    // 参数校验(任一为NULL，则不交换，返回-1)
    if (unlikely(list1 == NULL || list2 == NULL)) {
        pr_warn("list_swap: list1 or list2 is NULL\n");
        return -1;
    }
    // 交换头节点
    temp = list1->head;
    list1->head = list2->head;
    list2->head = temp;
    return 0;
}

/********************************************************************************
 * @brief   求两个链表的差集
 * @param   list1 链表1指针
 * @param   list2 链表2指针
 * @return  一个新的链表，包含所有在list2中但不在list1中的元素
 *          如果内存分配失败或list2为NULL，返回NULL
 * @note    新链表中的元素是list2中独有的，并已去重
 *          调用者需要在使用完毕后释放新链表占用的内存资源
 * @date    2025/11/21
 ********************************************************************************/
LQ_List *lq_list_difference(LQ_List *list1, LQ_List *list2)
{
    LQ_List *result_list;
    LQ_Node *curr_node;
    // 参数合法性校验
    if (unlikely(list2 == NULL))
    {
        pr_warn("lq_list_difference: list2 is NULL\n");
        return NULL;
    }
    // 初始化新的链表来存储结果
    result_list = kmalloc(sizeof(LQ_List), GFP_KERNEL);
    if (unlikely(result_list == NULL))
    {
        pr_warn("lq_list_difference: kmalloc failed\n");
        return NULL;
    }
    lq_list_init(result_list);
    // 遍历list2的每一个节点
    curr_node = list2->head;
    while (curr_node != NULL)
    {
        // 如果list1中不包含当前节点，则将其加入结果链表
        if (!lq_list_contains(list1, curr_node->data))
        {
            if (lq_list_add(result_list, curr_node->data) == -2)
            {
                pr_err("lq_list_difference: Failed to add node to result list\n");
                // 添加失败，清理已分配的内存并返回NULL
                lq_list_clear(result_list);
                kfree(result_list);
                return NULL;
            }
        }
        curr_node = curr_node->next;
    }
    // 返回结果链表
    return result_list;
}

/********************************************************************************
 * @brief   打印链表中的数据
 * @param   list 链表指针
 * @return  无返回值
 * @note    调试使用
 * @date    2025/11/19
 ********************************************************************************/
void lq_list_print(LQ_List *list)
{
    LQ_Node *curr_node;

    if (unlikely(list == NULL || list->head == NULL))
        return;

    curr_node = list->head;
    printk(KERN_INFO "List: ");
    while (curr_node != NULL) {
        printk(KERN_INFO "0x%X ", curr_node->data);
        curr_node = curr_node->next;
    }
    printk(KERN_INFO "\n");
}
