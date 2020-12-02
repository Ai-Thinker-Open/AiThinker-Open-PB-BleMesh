import os
import sys
from struct import *
#hex to bin
def hexf(hexfile):
	try:
		fin = open(hexfile)
	except:
		print('No file opened', hexfile)
		return None
	table =[]
	result = b''
	addr = 0
	addr_flg = 0
	for hexstr in fin.readlines():
		#print(hexstr)
		hexstr = hexstr.strip()
		#print(hexstr)
		size = int(hexstr[1:3],16)
		if int(hexstr[7:9],16) == 4:
			if(len(result)):
				#print(hex(addr))
				#print(addr,result)
				#input()
				table.append([addr,result])
			addr = 	int(hexstr[9:13],16) <<16
			addr_flg = 0
			result = b''
			continue
		if int(hexstr[7:9],16) == 5 or  int(hexstr[7:9],16) == 1:
			#print(hex(addr))
			#print(addr,result)
			table.append([addr,result])
			break
		
		if(addr_flg == 0):
			addr_flg = 1
			addr = addr | (int(hexstr[3:7],16))
		#end if	
		for h in range( 0, size):
			b = int(hexstr[9+h*2:9+h*2+2],16)
			#print(type(b),b,result)
			result += pack('B',b)
		#end if
		#fout.write(result)		
		#result=b''
		#input()
	#end for
	#print(table)
	fin.close()
	return table

	
def create_cfile(fsize):
	try:
		cfile = open('otaboot_hex.c', 'w')
	except:
		print('create file: otaboot_hex.c failed, exiting')
		exit()
	cfile.write('\n#define USE_FCT 0\n')
	cfile.write('\n#define CFG_OTA_BANK_MODE OTA_SINGLE_BANK\n')
	if(fsize == 0):
		cfile.write('\n#define CFG_FLASH 512\n')
	else:
		cfile.write('\n#define CFG_FLASH 128\n')
	
	cfile.write('const unsigned char ota_apptable[] = {\n')
	return cfile
	
	
def build_1st_boot(c_file, addr_list):
	s = 'const unsigned int ota_boottable[] = {%d,'%len(addr_list)
	if(addr_list[0][2] != 0x20008000):
		return
	
	for itm in addr_list:
		#flashaddress, size, runaddr
		s = s+'0x%x,0x%x,0x%x,'%(itm[0],itm[1],itm[2])
	
	s = s + '};\n'
	c_file.write('\n//boot table\n')
	
	c_file.write(s)
	c_file.close()
	
def build_ota_bootapp(c_file, part):
	plen = len(part[1]) + 7
	plen = plen - (plen%4)
	cnt = 0
	c_file.write('\n//partition 0x%.8x\n'%part[0])
	for val in part[1]:
		c_file.write('0x%.2x,'%val)
		cnt = cnt +1
		if((cnt%16) == 0):
			c_file.write('\n')
			
	
	for i in range(plen-len(part[1])):
		c_file.write('0,')
	

	return plen
	
	

def main(argv):
	if(len(argv)!= 2):
		print('convert hex|hexe file to C array')
		print('usage: otaboot_hex xxx.hex|hexe')
		return
	print('Merge hex|hexe file to C Array')
	print('Input flash size[0:512KB | 1:128KB]:')
	fsize = input()
	if(fsize == ''):
		fsize = 0
	else:
		try:
			fsize = int(fsize)
		except:
			fsize = 0
		if(fsize != 1 and fsize != 0):
			fsize = 0
	if(fsize == 0):
		print('512KB Flash')
	else:
		print('128KB Flash')
	
	hexname = argv[1]
	hex_data = hexf(hexname)
	
	
	c_file = None
	#flashaddr, size,runaddr
	addr_base = 0xa000
	addr_list = [[0, 0, 0x20008000]]
	for part in hex_data:
		print(hex(part[0]))
		addr_itm = [addr_base,len(part[1]),part[0]]
		if(c_file is None):
			c_file = create_cfile(fsize)
		plen = build_ota_bootapp(c_file, part)
		addr_base = addr_base + plen
		if(addr_itm[2] == 0x20008000):
			addr_list[0] = addr_itm
		else:
			addr_list.append(addr_itm)
	c_file.write('\n};\n\n')
	
		
	if(c_file is not None):
		#build 1st boot table
		build_1st_boot(c_file, addr_list)
		print("\nCompleted!")
		return
	print("Failed!")
	
	
if __name__ == '__main__':
	main(sys.argv)	
	
'''

a = hex('ota.hex')

print(a)

fp = open('ota.ota','wb')
strline = '['+ str(a[0][0]) + ',' +str(len(a[0][1])) +']\n'
fp.write(strline.encode())
strline = str(list(a[0][1])) + '\n'
fp.write(strline.encode())


print('a len is %d'%(len(a)))
print('%x,%x'%(a[0][0],len(a[0][1])))
print('%x,%x'%(a[1][0],len(a[1][1])))
print('%x,%x'%(a[2][0],len(a[2][1])))

	
if len(sys.argv) != 3 or (sys.argv[1] != '-h' and sys.argv[1] != '-b'):
	print( 'usage:')
	print( 'convert binary format to hexadecimal format: ')
	print( ' hexbin.py -h binfile hexfile')
	print( 'convert hexadecimal format to binary format:')
	print( ' hexbin.py -b hexfile binfile')
	exit(0)

	
if sys.argv[1] == '-h':
	print('None')
else:
	print(hex_bin(sys.argv[2]))


'''
	