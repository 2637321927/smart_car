#ifndef __LQ_LIST_DRV_H__
#define __LQ_LIST_DRV_H__

#include "LQ_drv_common.h"

/* 链表节点结构体, 请勿修改 */
typedef struct LQ_Node {
    uint8_t         data;   // 数据域
    struct LQ_Node *next;   // 指针域，指向下一个节点
} LQ_Node;

/* 链表管理结构体, 请勿修改 */
typedef struct LQ_List {
    LQ_Node *head;          // 头节点指针
} LQ_List;

void     lq_list_init       (LQ_List *list);                    /* 初始化链表 */
bool     lq_list_contains   (LQ_List *list, uint8_t data);      /* 判断某个数据是否在链表中 */
int      lq_list_add        (LQ_List *list, uint8_t data);      /* 向链表中添加数据 */
int      lq_list_remove     (LQ_List *list, uint8_t data);      /* 从链表中删除数据 */
int      lq_list_count      (LQ_List *list);                    /* 获取链表中的数据个数 */
void     lq_list_clear      (LQ_List *list);                    /* 清空链表 */
uint8_t  lq_list_compare    (LQ_List *list1, LQ_List *list2);   /* 比较两个链表是否相等 */
int      lq_list_swap       (LQ_List *list1, LQ_List *list2);   /* 交换两个链表 */
LQ_List *lq_list_difference (LQ_List *list1, LQ_List *list2);   /* 求两个链表的差集 */
void     lq_list_print      (LQ_List *list);                    /* 打印链表中的数据 */

#endif
