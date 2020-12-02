import sys
from intelhex import IntelHex
from struct import *

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


def crc16(a,x):
	#print(x)
	if(x is None):
		return a
	#a = 0#xFFFF
	b = 0xa001#0x1021#0xA001
	for byte in x:
		#print(byte, type(a), type(byte)	)
		a ^= byte
		for i in range(8):
			last = a % 2
			a >>= 1
			if last == 1:
				a ^= b
	#s = hex(a).upper()
	#print(s)
	return a
'''	
word      | desc:
0         | flag: "OTAF"
1         | partition number
i*2 + 2   | run address
i*2 + 3   | size
N*2 +2    | data area
'''
# dfl_packer.py flashaddr mode pattern0 pattern1 ...
def main(argv):
	if(len(argv) != 2 and len(argv) != 3 ):
		print('Usage: otam_hex.py fw.hex [resource.hex]')
		return
	fname = argv[1]
	fname_res = None
	if(len(argv) == 3):
		fname_res = argv[2]
	
	try:
		ih = IntelHex(fname)
		ih = None
		ih_res = None

		if(fname_res is not None):
			ih_res = IntelHex(fname)
			ih_res = None
	except:
		print('Open hex file failed:', fname, 'or resouce file error:', fname_res)
		print('Usage: otam_hex.py fw.hex [resource.hex]')
		return
		
	ih = hexf(fname)
	ih_res = []
	if(fname_res is not None):
		ih_res = hexf(fname_res)
		
	is_res = True
	for ihp in ih:
		if(ihp[0] == 0x1fff4000 or ihp[0] == 0x1fff4800):
			is_res = False
			break
	
	fname_bin = fname + '.bin'
	fp = open(fname_bin, 'wb')
	try:
		
		tag = 'OTAF'.encode()
		if(is_res):
			tag = 'OTAR'.encode()
		fp.write(tag)
		lval = [0,0,0,0]
		lval[0] = (len(ih) + len(ih_res)) & 0xff
		fp.write(bytes(lval))
		
		for ihp in ih:
			#run address
			param = ihp[0]
			lval = [0,0,0,0]
			lval[0] = param & 0xff
			lval[1] = (param>>8) & 0xff
			lval[2] = (param>>16) & 0xff
			lval[3] = (param>>24) & 0xff
			fp.write(bytes(lval))

			#partition size
			param = len(ihp[1])
			lval = [0,0,0,0]
			lval[0] = param & 0xff
			lval[1] = (param>>8) & 0xff
			lval[2] = (param>>16) & 0xff
			lval[3] = (param>>24) & 0xff
			fp.write(bytes(lval))
		for ihp in ih_res:
			#run address
			param = ihp[0]
			lval = [0,0,0,0]
			lval[0] = param & 0xff
			lval[1] = (param>>8) & 0xff
			lval[2] = (param>>16) & 0xff
			lval[3] = (param>>24) & 0xff
			fp.write(bytes(lval))

			#partition size
			param = len(ihp[1])
			lval = [0,0,0,0]
			lval[0] = param & 0xff
			lval[1] = (param>>8) & 0xff
			lval[2] = (param>>16) & 0xff
			lval[3] = (param>>24) & 0xff
			fp.write(bytes(lval))
			
		for ihp in ih:
			fp.write(bytes(ihp[1]))

		for ihp in ih_res:
			fp.write(bytes(ihp[1]))

		fp.close()
	except:
		print('Open hex file failed:', fname)
		print('Usage: otam_hex.py fw.hex')
	
	
	
if __name__ == '__main__':
	main(sys.argv)
