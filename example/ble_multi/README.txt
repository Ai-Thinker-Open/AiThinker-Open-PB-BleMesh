multiRole 做 master 时，连接 slave MAC Addr:
	addr[5] = 0x11
	addr[4] = 0x22
	addr[3] = 0x33
	addr[2] = 0x44
	addr[1] = 0x60
	addr[0] = 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07
	
master 通过 MultiRole characteristic 0xFFF5 控制 Slave LED Status
master --> MultiRole --> Slave control LED ( Characteristic OxFFF5 )


流程图查看：
“multiRole.eddx” 可使用网页“http://www.edrawsoft.cn/viewer/app/” 查看