//用来自测试一些功能和效率xqy

#if 0 /*xqy 2019-12-23*/
#include <rthw.h>
#include <rtthread.h>

/*测试FAT32文件系统读写速度*/
u8 rw_buff[4096];

void fat32_read_test(u32 len)
{
    int fd = -1;
    u8 *file_name = "/test/rw_file.bin";
    rt_memset(rw_buff,0,4096);
    fd = open()
    
}
#endif
