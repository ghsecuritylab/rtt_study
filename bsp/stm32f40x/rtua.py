
def GetCPPPATH(BSP_ROOT, RTT_ROOT):
	CPPPATH=[
		RTT_ROOT + "/bsp/stm32f40x",
		RTT_ROOT + "/bsp/stm32f40x/Libraries/CMSIS/Include",
		RTT_ROOT + "/bsp/stm32f40x/Libraries/CMSIS/ST/STM32F4xx/Include",
		RTT_ROOT + "/bsp/stm32f40x/Libraries/STM32F4xx_StdPeriph_Driver/inc",
		RTT_ROOT + "/bsp/stm32f40x/applications",
		RTT_ROOT + "/bsp/stm32f40x/drivers",
		RTT_ROOT + "/bsp/stm32f40x/packages/at_device-v1.4.0",
		RTT_ROOT + "/bsp/stm32f40x/packages/cJSON-v1.0.2",
		RTT_ROOT + "/bsp/stm32f40x/packages/fastlz-latest",
		RTT_ROOT + "/bsp/stm32f40x/packages/qrcode-latest/inc",
		RTT_ROOT + "/components/dfs/filesystems/devfs",
		RTT_ROOT + "/components/dfs/filesystems/elmfat",
		RTT_ROOT + "/components/dfs/include",
		RTT_ROOT + "/components/drivers/audio",
		RTT_ROOT + "/components/drivers/include",
		RTT_ROOT + "/components/drivers/include/drivers",
		RTT_ROOT + "/components/drivers/spi",
		RTT_ROOT + "/components/finsh",
		RTT_ROOT + "/components/libc/compilers/common",
		RTT_ROOT + "/components/libc/compilers/newlib",
		RTT_ROOT + "/components/net/at/at_socket",
		RTT_ROOT + "/components/net/at/include",
		RTT_ROOT + "/components/net/sal_socket/impl",
		RTT_ROOT + "/components/net/sal_socket/include",
		RTT_ROOT + "/components/net/sal_socket/include/dfs_net",
		RTT_ROOT + "/components/net/sal_socket/include/socket",
		RTT_ROOT + "/components/net/sal_socket/include/socket/sys_socket",
		RTT_ROOT + "/include",
		RTT_ROOT + "/libcpu/arm/common",
		RTT_ROOT + "/libcpu/arm/cortex-m4",
	]

	return CPPPATH

def GetCPPDEFINES():
	CPPDEFINES=['RT_USING_NEWLIB', 'USE_STDPERIPH_DRIVER']
	return CPPDEFINES

