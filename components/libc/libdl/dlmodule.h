/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/08/11     Bernard      the first version
 */

#ifndef RT_DL_MODULE_H__
#define RT_DL_MODULE_H__

#include <rtthread.h>

#define RT_DLMODULE_STAT_INIT       0x00//模块状态
#define RT_DLMODULE_STAT_RUNNING    0x01
#define RT_DLMODULE_STAT_CLOSING    0x02
#define RT_DLMODULE_STAT_CLOSED     0x03

struct rt_dlmodule;
typedef void* rt_addr_t;

typedef void (*rt_dlmodule_init_func_t)(struct rt_dlmodule *module);//模块初始化
typedef void (*rt_dlmodule_cleanup_func_t)(struct rt_dlmodule *module);//模块反初始化
typedef int  (*rt_dlmodule_entry_func_t)(int argc, char** argv);//模块入口函数类型

struct rt_dlmodule
{
    struct rt_object parent;//模块的内核对象成员
    rt_list_t object_list;  /* objects inside this module */

    rt_uint8_t stat;        /* status of module */

    /* main thread of this module */
    rt_uint16_t priority;
    rt_uint32_t stack_size;
    struct rt_thread *main_thread;
    /* the return code */
    int ret_code;

    /* VMA base address for the first LOAD segment */
    rt_uint32_t vstart_addr;

    /* module entry, RT_NULL for dynamic library */
    rt_dlmodule_entry_func_t  entry_addr;
    char *cmd_line;         /* command line */

    rt_addr_t   mem_space;  /* memory space */
    rt_uint32_t mem_size;   /* sizeof memory space */

    /* init and clean function */
    rt_dlmodule_init_func_t     init_func;
    rt_dlmodule_cleanup_func_t  cleanup_func;

    rt_uint16_t nref;       /* reference count */

    rt_uint16_t nsym;       /* number of symbols in the module *///模块中的函数符号数量
    struct rt_module_symtab *symtab;    /* module symbol table *///函数符号链表，函数名指针，函数地址指针。
};

struct rt_dlmodule *dlmodule_create(void);
rt_err_t dlmodule_destroy(struct rt_dlmodule* module);

struct rt_dlmodule *dlmodule_self(void);

struct rt_dlmodule *dlmodule_load(const char* pgname);
struct rt_dlmodule *dlmodule_exec(const char* pgname, const char* cmd, int cmd_size);
void dlmodule_exit(int ret_code);

struct rt_dlmodule *dlmodule_find(const char *name);

rt_uint32_t dlmodule_symbol_find(const char *sym_str);

#endif
