#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Project Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 100
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDEL_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 1024
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512
#define RT_DEBUG
#define RT_DEBUG_INIT_CONFIG
#define RT_DEBUG_INIT 1
#define RT_DEBUG_MEM_CONFIG
#define RT_DEBUG_MEM 1
#define RT_DEBUG_SLAB_CONFIG
#define RT_DEBUG_SLAB 1
#define RT_DEBUG_MEMHEAP_CONFIG
#define RT_DEBUG_MEMHEAP 1
#define RT_DEBUG_MODULE_CONFIG
#define RT_DEBUG_MODULE 1

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_SMALL_MEM
#define RT_USING_MEMTRACE
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_INTERRUPT_INFO
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x40000
#define ARCH_ARM
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M4

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */


/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_ARG_MAX 10

/* Device virtual file system */

#define RT_USING_DFS
#define DFS_FILESYSTEMS_MAX 2
#define DFS_FILESYSTEM_TYPES_MAX 2
#define DFS_FD_MAX 16
#define RT_USING_DFS_ELMFAT

/* elm-chan's FatFs, Generic FAT Filesystem Module */

#define RT_DFS_ELM_CODE_PAGE 936
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_3
#define RT_DFS_ELM_USE_LFN 3
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 2
#define RT_DFS_ELM_MAX_SECTOR_SIZE 512
#define RT_DFS_ELM_REENTRANT
#define RT_USING_DFS_DEVFS

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
#define RT_USING_PIN
#define RT_USING_SPI
#define RT_USING_SPI_MSD
#define RT_USING_W25QXX
#define RT_USING_WDT
#define RT_USING_AUDIO

/* Using WiFi */


/* Using USB */


/* POSIX layer and C standard library */

#define RT_USING_LIBC
#define RT_USING_POSIX

/* Network */

/* Socket abstraction layer */

#define RT_USING_SAL

/* protocol stack implement */

#define SAL_USING_AT
#define SAL_USING_POSIX
#define SAL_PROTO_FAMILIES_NUM 4

/* light weight TCP/IP stack */


/* Modbus master and slave stack */


/* AT commands */

#define RT_USING_AT
#define AT_DEBUG
#define AT_USING_CLIENT
#define AT_CLIENT_NUM_MAX 1
#define AT_USING_SOCKET
#define AT_USING_CLI
#define AT_SW_VERSION_NUM 0x10200

/* VBUS(Virtual Software BUS) */


/* Utilities */


/* ARM CMSIS */


/* RT-Thread online packages */

/* IoT - internet of things */

#define PKG_USING_PAHOMQTT
#define PAHOMQTT_PIPE_MODE
#define RT_PKG_MQTT_THREAD_STACK_SIZE 4096
#define PKG_PAHOMQTT_SUBSCRIBE_HANDLERS 1
#define MQTT_DEBUG
#define PKG_USING_PAHOMQTT_LATEST
#define PKG_USING_CJSON
#define PKG_USING_CJSON_V102

/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */

#define PKG_USING_AT_DEVICE
#define PKG_AT_INIT_BY_THREAD
#define AT_DEVICE_ESP8266
#define AT_DEVICE_SOCKETS_NUM 5
#define AT_DEVICE_NAME "uart2"
#define AT_DEVICE_RECV_BUFF_LEN 512
#define AT_DEVICE_WIFI_SSID "rtthread"
#define AT_DEVICE_WIFI_PASSWORD "12345678"
#define PKG_USING_AT_DEVICE_V140
#define PKG_AT_DEVICE_VER_NUM 0x10400

/* IoT Cloud */


/* security packages */


/* language packages */


/* multimedia packages */


/* tools packages */

#define PKG_USING_CMBACKTRACE
#define PKG_CMBACKTRACE_PLATFORM_M4
#define PKG_CMBACKTRACE_DUMP_STACK
#define PKG_CMBACKTRACE_PRINT_ENGLISH
#define PKG_USING_CMBACKTRACE_LATEST_VERSION
#define PKG_USING_QRCODE
#define PKG_USING_QRCODE_LATEST_VERSION

/* system packages */

#define PKG_USING_GUIENGINE
#define PKG_USING_GUIENGINE_LATEST_VERSION
#define PKG_USING_RGB888_PIXEL_BITS_24
#define PKG_USING_RGB888_PIXEL_BITS 24
#define GUIENGINE_NAME_MAX 16
#define GUIENG_USING_FNT_FILE
#define GUIENGINE_USING_FONT16
#define GUIENGINE_USING_PNG
#define GUIENGINE_IMAGE_LODEPNG
#define GUIENGINE_IMAGE_CONTAINER
#define GUIENGINE_USING_DEMO

/* peripheral libraries and drivers */


/* miscellaneous packages */

#define PKG_USING_FASTLZ
#define PKG_USING_FASTLZ_LATEST_VERSION

/* samples: kernel and components samples */

#define SOC_STM32F4
#define RT_USING_UART1

#endif
