#<SYMDEFS># ARM Linker, 5060422: Last Updated: Mon Mar 11 11:20:20 2019
import sys

dict_month = {
'Jan':0,
'Feb':1,
'Mar':2,
'Apr':3,
'May':4,
'Jun':5,
'Jul':6,
'Aug':7,
'Sep':8,
'Oct':9,
'Nov':10,
'Dec':11
}

def main(argv):
	sym_fname = argv[1]
	try:
		fp = open(sym_fname, 'r')
		ln = fp.readline()
		offset = ln.find(':')
		ln = ln[offset+1:]
		offset = ln.find(':')
		ln = ln[offset+2:]
		lln = ln.split(' ')
		year = int(lln[4])%100
		month = dict_month[lln[1]]
		day = int(int(lln[2]))
		ltm = lln[3].split(':')
		hour = int(ltm[0])
		minute = int(ltm[1])
		second = int(ltm[2])
		print(year, month, day, hour, minute, second)
		fp.close()
		fp = open('dfl_ver.h', 'w')
		fp.write('\n#ifndef __DFL_VER_H\n#define __DFL_VER_H\n\n')
		fp.write('#define DFL_YEAR    '+str(year) + '\n')
		fp.write('#define DFL_MONTH   '+str(month) + '\n')
		fp.write('#define DFL_DAY     '+str(day) + '\n')
		fp.write('#define DFL_HOUR    '+str(hour) + '\n')
		fp.write('#define DFL_MINUTE  '+str(minute) + '\n')
		fp.write('#define DFL_SECOND  '+str(second) + '\n')

		fp.write('#define DFL_FIELD_0  (DFL_SECOND | (DFL_MINUTE<<8) | (DFL_HOUR<<16) | (DFL_DAY<<24))\n')
		fp.write('#define DFL_FIELD_1  (DFL_MONTH | (DFL_YEAR<<8))\n')

		fp.write('#endif\n\n')
		
	except:
		print('Create symble version error!')
		
if __name__ == '__main__':
	main(sys.argv)
