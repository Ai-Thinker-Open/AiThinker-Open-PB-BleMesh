multi role project 工程说明：
		因为 multi role link layer 等数据结构变化，不能与原SDK components 的
	头文件共用，因此将新的头文件添加到了 Trunk\lib\multi_include 目录
	
1、multi proj 目录更改记录
	(1)、components->ble->include  
		(该目录有多个文件，但仅有部分文件不同，因此将整个目录更改)
	(2)、components->ble->controller->include
	