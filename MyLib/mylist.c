#include "mylist.h"

// 创建链表
MyList_t* ListCreate(int element_size)
{
    if (element_size <= 0)
        return NULL;

    MyList_t* list = (MyList_t*)malloc(sizeof(MyList_t));
    if (!list)
        return NULL;

    list->data = NULL;
    list->length = 0;
    list->element_size = element_size;
    return list;
}

// 删除整个链表
int ListRemove(MyList_t *list)
{
    if (!list)
        return -1;

    Load_t* node = list->data;
    while (node)
    {
        Load_t* tmp = node->next;
        free(node);
        node = tmp;
    }

    list->data = NULL;
    list->length = 0;
    return 0;
}

// 添加元素
int ListAddElement(MyList_t *list, void* data)
{
    if (!list || !data)
        return -1;

    Load_t* node = (Load_t*)malloc(sizeof(Load_t) + list->element_size);
    if (!node)
        return -1;

    node->next = NULL;
    memcpy(node->data, data, list->element_size);

    if (!list->data)
    {
        list->data = node;
    }
    else
    {
        Load_t* tail = list->data;
        while (tail->next)
            tail = tail->next;
        tail->next = node;
    }

    list->length++;
    return 0;
}

// 删除指定索引的元素
int ListDeleteElement(MyList_t *list, int index)
{
    if (!list || index < 0 || index >= list->length)
        return -1;

    Load_t* node = list->data;
    if (index == 0)
    {
        list->data = node->next;
        free(node);
    }
    else
    {
        for (int i = 0; i < index - 1; i++)
        {
            node = node->next;
        }
        Load_t* to_delete = node->next;
        node->next = to_delete->next;
        free(to_delete);
    }

    list->length--;
    return 0;
}

// 查找元素
void* ListFind(MyList_t *list, void* user, ListIsMatch_Cb_t match_cb)
{
    if (!list || !match_cb)
        return NULL;

    Load_t* node = list->data;
    while (node)
    {
        if (match_cb(user, node->data))
            return node->data;
        node = node->next;
    }
    return NULL;
}

// 获取元素索引
int ListGetIndex(MyList_t *list, void* src, ListIsMatch_Cb_t match_cb)
{
    if (!list || !match_cb)
        return -1;

    Load_t* node = list->data;
    int index = 0;
    while (node)
    {
        if (match_cb(src, node->data))
            return index;
        node = node->next;
        index++;
    }
    return -1;
}

// 根据索引获取数据
void* ListGetDataByIndex(MyList_t *list, int index)
{
    if (!list || index < 0 || index >= list->length)
        return NULL;

    Load_t* node = list->data;
    for (int i = 0; i < index; i++)
        node = node->next;

    return node->data;
}

// 迭代器初始化
void InitListIterator(ListIterator_t *iterater, MyList_t *list)
{
    if (!iterater)
        return;
    iterater->list = list;
    iterater->current = list ? list->data : NULL;
}

// 重置迭代器
void ResetListIterator(ListIterator_t *iterater)
{
    if (!iterater || !iterater->list)
        return;
    iterater->current = iterater->list->data;
}

// 获取迭代器当前元素
void* IteraterGet(ListIterator_t *iterater)
{
    if (!iterater || !iterater->current)
        return NULL;
    return iterater->current->data;
}

// 移动迭代器到下一个元素
void IteraterNext(ListIterator_t *iterater)
{
    if (!iterater || !iterater->current)
        return;
    iterater->current = iterater->current->next;
}