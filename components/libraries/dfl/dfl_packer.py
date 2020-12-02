import sys
from intelhex import IntelHex

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
	
	
# dfl_packer.py flashaddr mode pattern0 pattern1 ...
def main(argv):
	
	fp = open('flashmap.bin', 'wb')
	flist = argv[3:]
	try:
		flashaddr = eval(argv[1])
		tag = 'DFL:'.encode()
		fp.write(tag)
		lval = [0,0,0,0]
		lval[0] = len(flist) & 0xff
		fp.write(bytes(lval))
		lval = [1,0,0,0]
		fp.write(bytes(lval))
		lval = [0x00,0,0,0]
		fp.write(bytes(lval))
		lval = [0xff,0xff,0xff,0xff]
		fp.write(bytes(lval))
		
		for i in range(11):
			print(i)
			lval = [0x1f,0xff,0xff,0xff]
			fp.write(bytes(lval))
	except:
		print('usage:\n	dfl_packer.py flashaddr pattern0 pattern1 ...')
	
	flashaddr = 144*4
	packinfo = []
	datatable = []
	
	emptytable = []
	for i in range(512):
		emptytable = emptytable + [0xff]
	
	print(flist)
	for fname in flist:
		#try:
		ih = IntelHex(fname)
		dih = ih.todict()
		ldih = list(dih)
		ldihv = list(dih.values())
		ldihv = ldihv[:-1]
		datasize = len(ldihv)+3
		ldihv = ldihv + [0,0,0,0,0,0,0,0]
		datasize = (datasize - (datasize%4))
		ldihv = ldihv[:datasize]
		runaddr = ldih[0]
		lflashaddr = [flashaddr&0xff, (flashaddr>>8)&0xff,(flashaddr>>16)&0xff,(flashaddr>>24)&0xff]
		lrunaddr = [runaddr&0xff, (runaddr>>8)&0xff,(runaddr>>16)&0xff,(runaddr>>24)&0xff]
		lrunaddr = [runaddr&0xff, (runaddr>>8)&0xff,(runaddr>>16)&0xff,(runaddr>>24)&0xff]
		ldatasize = [datasize&0xff, (datasize>>8)&0xff,(datasize>>16)&0xff,(datasize>>24)&0xff]
		#print(ldihv)
		crc = crc16(0, ldihv)
		lcrc = [crc&0xff, (crc>>8)&0xff,(crc>>16)&0xff,(crc>>24)&0xff]
		packinfo = packinfo + lflashaddr + lrunaddr + ldatasize + lcrc
		#print('pack info')
		#print(packinfo)

		datatable += ldihv	
		flashaddr += datasize
		#except:
		#	print('Create symble version error!')
		#	print('usage:\n	dfl_packer.py flashaddr pattern0 pattern1 ...')
	packinfo = packinfo + emptytable
	packinfo = packinfo[:32*4*4]
	fp.write(bytes(packinfo))
	fp.write(bytes(datatable))
	
	
if __name__ == '__main__':
	main(sys.argv)
