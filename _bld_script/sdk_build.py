#! /usr/bin/env python3
import threading
import os
import glob
import sys
import time
import random
import msvcrt
import traceback
import yaml
from datetime import datetime

'''
curpath = os.path.dirname(os.path.realpath(__file__))
result=os.path.exists("newsdk_build.yml")
if result:
    yamlpath = os.path.join(curpath, "newsdk_build.yml")  #3 is people-edited file
else:
    yamlpath = os.path.join(curpath, "sdk_build.yml")  #2 is default file
f = open(yamlpath, 'r', encoding='utf-8')
c = f.read()
d = yaml.load(c,Loader=yaml.FullLoader)

dict_build_list = d
'''
dict_yml_name = 'sdk_build'
dict_build_list = {}
'''
dict_build_list={'rfslave' : ['lib\\generate_lib\\rf.uvprojx',['Objects\\rf.lib', '..\\rf.lib']],
'ble_host' : ['lib\\generate_ble_host_lib\\ble_host.uvprojx',['Objects\\ble_host.lib', '..\\ble_host.lib']],
'aox_lib' : ['lib\\generate_aoxEst_lib\\aoxEst.uvprojx',['Objects\\aoxEst.lib', '..\\aoxEst.lib']],
'phy_tx'  : ['example\\PhyPlusPhy\\aoa_test\\PhyPlusPhy.uvprojx',{'PHY_MODE_DEF':'0'},['Objects\\PhyPlusPhy.hex','bin\\ppp_tx.hex']],
'phy_rx'  : ['example\\PhyPlusPhy\\aoa_test\\PhyPlusPhy.uvprojx',{'PHY_MODE_DEF':'1'},['Objects\\PhyPlusPhy.hex','bin\\ppp_rx.hex']],
'sbp' : ['example\\ble_peripheral\\simpleBlePeripheral\\simpleBlePeripheral.uvprojx'],
'sbc' : ['example\\ble_central\\simpleBleCentral\\simpleBleCentral.uvprojx'],
'app0' : ['example\\peripheral\\app0_tasktst\\app0_m0\\app0_m0.uvprojx'],
}
'''
def checkTimeCost(strIn,lastTick,prtFlg=0,logFile=None):
	tCur    = time.perf_counter()
	tCost   = (tCur-lastTick)/(1.0/1e6)
	if(logFile):
		logFile.write( 'Time Cost------ %20s -------- %9.3f ms\n'%(strIn,tCost/1000 ))

	if(prtFlg==1):
		print( 'Time Cost------ %20s -------- %9.3f ms'%(strIn,tCost/1000 ))
	else:
		pass

	return tCur,tCost

class build:
	def __init__(self, path, keil_path = 'C:\\Keil_v5\\UV4\\UV4.exe'):
		global dict_yml_name
		self.yml_name = dict_yml_name
		self.m_path = path
		
		self.m_keil_path = keil_path
		self.m_logfile = None

		path = self.m_path
		st = 0
		while(True):
			st1 = path.find('\\',st+1)
			if(st1<0):
				break
			st = st1
		path = path[:st+1]
		self.m_fold = path
		
	def log(self, para0, para1 = None, para2 = None, para3 = None):
		if(self.m_logfile is None):
			print(para0, para1, para2, para3)
			return
		if(para1 is None and para2 is None and para3 is None):
			self.m_logfile.write(str(para0)+'\n')
		
		if(type(para1) is list):
			self.m_logfile.write('    '+str(para0)+'\n')
			for itm in para1:
				self.m_logfile.write('    '+str(itm))
	
		if(type(para0) is int):
			self.m_logfile.write('    Error %d, Warning %d\n'%(para0, para1))
			for itm in para2:
				self.m_logfile.write('    '+str(itm))
		
	def new_config_line(self, proj_line, build_param):
		#apply new config
		new_line = proj_line[:proj_line.find('<Define>')+8]
		start_space = ''
		proj_cfg = proj_line[proj_line.find('<Define>')+8:proj_line.find('</Define>')]
		#print(proj_cfg)
		proj_cfg = proj_cfg.split(' ')
		for i in range(len(proj_cfg)):
			proj_cfg[i] = proj_cfg[i].split('=')
			value = proj_cfg[i][0]
			if(value in build_param):
				if(len(proj_cfg[i]) == 2):
					proj_cfg[i][1] = build_param[value]
					print('BLD_CFG : %s --> %s'%(proj_cfg[i][0],proj_cfg[i][1]))
				elif(len(proj_cfg[i]) == 1):
					proj_cfg[i][0] = build_param[value]
					print("BLD_CFG : %s",proj_cfg[i][0])
		for cfg in proj_cfg:
			if(len(cfg) == 1):
				new_line = new_line +start_space + cfg[0]
			else:
				new_line = new_line +start_space + cfg[0]+'='+cfg[1]
			start_space = ' '
		new_line = new_line + '</Define>\r'
		
		return new_line
				

	def config_proj(self, build_param):
		#backup project file
		c_flg = False
		proj_backup = ''
		proj_items = []
		fsize = os.path.getsize(self.m_path)
		fprj = open(self.m_path, 'r')
		project_backup = fprj.read(fsize)
		fprj.seek(0)
		while(True):
			proj_line = fprj.readline()
			if(proj_line.find('<Cads>')>0):
				#print('<Cads>')
				c_flg = True
			if(proj_line.find('<Aads>')>0):
				#print('<Aads>')
				c_flg = False
			if(c_flg):
				if(proj_line.find('<Define>')>=0):
					proj_line = self.new_config_line(proj_line, build_param)
			if(len(proj_line) == 0):
				break
			proj_line = proj_line[:-1]
			proj_items.append(proj_line)
		fprj.close()
		
		#new project file
		fprj = open(self.m_path, 'wb')
		for proj_line in proj_items:
			fprj.write(proj_line.encode())
			fprj.write(b'\n')
		fprj.close()
		#print('pause')
		#input()
		return project_backup
	
	def new_hex(self, output):
		path = self.m_fold
		os.system('copy /Y '+ path + output[0] + ' ' + path + output[1])
	
	def gen_asm(self, output):
		if(len(output)<3):
			print("Check Output[3] no asm config!!!")
			return False
		fromElfPath='C:\\Keil_v5\\ARM\\ARMCC\\bin\\fromelf.exe'
		path = self.m_fold
		if(len(output)==4):
			axfPath = path + output[3]
		else:
			axfPath = path + output[0].split(".hex")[0]+'.axf'
		if(os.path.exists(axfPath) != True):
			print("Can Not Find AXF : %s \n",axfPath)
			return False
		asmPath = path + '\\Listings\\'+ output[2]
		os.system(fromElfPath +' -c -a -d -e -v -o '+ asmPath + ' ' + axfPath)
		return True


	def setlogfile(self, logf):
		self.m_logfile = logf

	#return True, False
	def build_check(self, timeout = 100):
		i = 0
		for i in range(timeout):
			if(os.path.exists(self.m_fold + '_bld.txt')):
				time.sleep(1)
				break
			time.sleep(0.1)
		if(i == timeout):
			print('Wait build result timeout')
			self.log('Wait build result timeout', [])
			return False
		
		log_list = []
		flog = open(self.m_fold + '_bld.txt', 'r')
		compile_flg = False
		while(True):
			logstr = flog.readline()
			if(len(logstr) == 0):
				break #read conpleted
			log_list.append(logstr)
			if(logstr.find('Error(s)')>0 and logstr.find('Warning(s)')>0 ):
				compile_flg = True
				errnum = int(logstr[logstr.find('-')+1:logstr.find('Error(s)')])
				warnum = int(logstr[logstr.find('Error(s)')+9: logstr.find('Warning(s)')])
				if(errnum + warnum):
					print('Error %d, Warning %d'%(errnum, warnum))
					self.log(errnum, warnum, log_list)
					if(errnum):
						return False
		if(compile_flg is False):
			print('Compile Failed')
			self.log('Compile Failed', log_list)
			return False
		return True
		#if():#check warning
		
		
	
	def __call__(self, build_param = None, output = None):
		#print('build', build_param, output)
		lastTick=time.perf_counter()
		tcLog = self.m_logfile
		tcPrtFlg = 1
		project_backup = ''
		if(os.path.exists(self.m_path) != True):
			print('\n\nProject file is not exist:',self.m_path)
			self.log('\n\nProject file is not exist:',self.m_path)
			return False
		if(os.path.exists(self.m_keil_path) != True):
			print('Can\'t find Keil_V5 IDE :',self.m_keil_path)
			return False
		
		#delete log file
		#os.system('del '+self.m_fold + '_bld.txt 2>1>nul')
		os.system('del /f /q  '+ self.m_fold + '_bld.txt')

		print('\n\nBuilding...\n'+self.m_path)
		self.log('\n\nBuilding...\n'+self.m_path)
		[lastTick,tCost]=checkTimeCost('T0',lastTick,tcPrtFlg,tcLog)
		
		if(build_param is not None):
			project_backup = self.config_proj(build_param)
		
		[lastTick,tCost]=checkTimeCost('Config Proj',lastTick,tcPrtFlg,tcLog)
		#run build
		cmd = self.m_keil_path + ' -r -j0 ' + self.m_path + ' -o _bld.txt' 
		os.system(cmd)
		
		[lastTick,tCost]=checkTimeCost('CMD Build',lastTick,tcPrtFlg,tcLog)
		#check and save result
		ret = self.build_check(200)
		
		[lastTick,tCost]=checkTimeCost('Build Check',lastTick,tcPrtFlg,tcLog)
		#restore project
		if(build_param is not None):
			fprj = open(self.m_path, 'wb')
			fprj.write(project_backup.encode())
			fprj.close()
		
		[lastTick,tCost]=checkTimeCost('Restore',lastTick,tcPrtFlg,tcLog)
			
		if(ret is False):
			return False

		if(output is not None):
			if(len(output)>=3):
				self.gen_asm(output)
				[lastTick,tCost]=checkTimeCost('GEN ASM',lastTick,tcPrtFlg,tcLog)
		

		if(output is not None):
			self.new_hex(output)
			[lastTick,tCost]=checkTimeCost('Copy Hex',lastTick,tcPrtFlg,tcLog)
		
		return True


##########################ultility##############################

def make_version_file(path, major, minor, revision, test_build = ''):
	list_sdk_version_h = [
	'/**************************************************************************************************',
	' ',
	'  Phyplus Microelectronics Limited confidential and proprietary. ',
	'  All rights reserved.',
	' ',
	'  IMPORTANT: All rights of this software belong to Phyplus Microelectronics ',
	'  Limited ("Phyplus"). Your use of this Software is limited to those ',
	'  specific rights granted under  the terms of the business contract, the ',
	'  confidential agreement, the non-disclosure agreement and any other forms ',
	'  of agreements as a customer or a partner of Phyplus. You may not use this ',
	'  Software unless you agree to abide by the terms of these agreements. ',
	'  You acknowledge that the Software may not be modified, copied, ',
	'  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy ',
	'  (BLE) integrated circuit, either as a product or is integrated into your ',
	'  products.  Other than for the aforementioned purposes, you may not use, ',
	'  reproduce, copy, prepare derivative works of, modify, distribute, perform, ',
	'  display or sell this Software and/or its documentation for any purposes.',
	' ',
	'  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE',
	'  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,',
	'  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,',
	'  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL',
	'  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,',
	'  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER',
	'  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES',
	'  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE',
	'  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT',
	'  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES',
	'  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.',
	'  ',
	'**************************************************************************************************/',
	' ',
	'/*******************************************************************************',
	'* @file		sdk_version.h',
	'* @brief	',
	'* @author	',
	'* ',
	'*',
	'*******************************************************************************/',
	'#ifndef __SDK_VER_H__',
	'#define __SDK_VER_H__',
	' ',]
	
	if(path == ''):
		#path = 'Trunk'
		path = '..'
	if(path[-1] == '\\'):
		path = path[:-1]
		
	fp = open(path+'\\components\\inc\\version.h', 'w')
	for ln in list_sdk_version_h:
		fp.writelines(ln+'\n')
	
	fp.writelines('#define __DEF_CHIP_QFN32__  '+' '*16+'(0x0001) \n')
	fp.writelines('#define __DEF_CHIP_TSOP16__  '+' '*16+'(0x0002) \n')
	
	fp.writelines('#define SDK_VER_MAJOR      '+' '*16+str(major) + '\n')
	fp.writelines('#define SDK_VER_MINOR      '+' '*16+str(minor) + '\n')
	fp.writelines('#define SDK_VER_REVISION   '+' '*16+str(revision) + '\n')
	fp.writelines('#define SDK_VER_RELEASE_ID '+' '*16+'((SDK_VER_MAJOR<<16)|(SDK_VER_MINOR<<8)|(SDK_VER_REVISION))'+  '\n')
	
	if(major==3):
		fp.writelines('#define SDK_VER_CHIP      '+' '*16+'__DEF_CHIP_QFN32__'+  '\n')
	elif(major==5):
		fp.writelines('#define SDK_VER_CHIP      '+' '*16+'__DEF_CHIP_TSOP16__'+  '\n')
	
	
	if(test_build != ''):
		fp.writelines('#define SDK_VER_TEST_BUILD \''+ test_build + '\'\n')
	else:
		fp.writelines('//#define SDK_VER_TEST_BUILD "'+ test_build + '"\n')
	
	fp.writelines('#endif\n\n')
	fp.close()
	
def get_param(param):
	dict_param = {}
	#print(param)
	phase_opt = None
	phase_param = []
	for s in param:

		if(s[0]=='-'):
			if(phase_param != [] and phase_opt is not None):
				dict_param[phase_opt] = phase_param
			phase_param = []
		else:
			phase_param.append(s)
			continue
		
		if(s == '-help' or s == '-h'): #help
			dict_param['help'] = None
			phase_opt = 'help'
			phase_param = []
			continue

		if (s == '-lcfg' or s == '-l'):  # list config
			dict_param['listconfig'] = None
			phase_opt = 'listconfig'
			phase_param = []
			continue
			
		if s == '-list': #list project
			dict_param['list'] = None
			phase_opt = 'list'
			phase_param = []
			continue
			
		if s == '-path': #set project path
			dict_param['path'] = None
			phase_opt = 'path'
			phase_param = []
			continue
			
		if s == '-ver' or s == '-version' : #set version
			dict_param['version'] = None
			phase_opt = 'version'
			phase_param = []
			continue
		
		if s == '-b':
			dict_param['build'] = None
			phase_opt = 'build'
			phase_param = []
			continue
			
		if s == '-c' or s == '-clear':
			dict_param['clear'] = None
			phase_opt = 'clear'
			phase_param = []
			continue
	if(phase_param != [] and phase_opt is not None):
		dict_param[phase_opt] = phase_param
	
	return dict_param

def help(prj = None):
	print('sdk_build.py: Build PhyPlus BLE SDK')
	print('useage:')
	print('	sdk_build.py [-help [projectname]] [-clear] [-ver 1.1.1.b] [-path sdk_path][-list] [-b [projectname]|[all]]')
	
def files(curr_dir, ext):
	for i in glob.glob(os.path.join(curr_dir, ext)):
		yield i

def remove_files(rootdir, ext, show = False):
	for i in files(rootdir, ext):
		if show:
			print(i)
		os.remove(i)	
def clear_log():
	remove_files('.', 'buildlog_*.txt', True)

def list_config(param):

	global dict_build_list
	global dict_yml_name
	curpath = os.path.dirname(os.path.realpath(__file__))
	result = os.path.exists(param[0]+'.yml')
	if result:
		yamlpath = os.path.join(curpath, param[0]+'.yml')  # 3 is people-edited file
		dict_yml_name = param[0]
	else:
		yamlpath = os.path.join(curpath, dict_yml_name+'.yml')  # sdk_build.yml is default file
	f = open(yamlpath, 'r', encoding='utf-8')
	c = f.read()
	d = yaml.load(c, Loader=yaml.FullLoader)
	dict_build_list = d

def list_prj():
	global dict_build_list
	if len(dict_build_list) == 0:
		print('dict_build_list is blank.-lcfg first')
		return
	print('Project:')
	cnt = 0
	for prjname in dict_build_list:
		ss1 = dict_build_list[prjname][0]
		print('  %03d  %-20s%-40s'%(cnt,prjname,ss1))
		try:
			ss2=dict_build_list[prjname][1]
			if(type(ss2) is dict):
				print(' '*27 +':%s'%(ss2))
		except:
			pass
		cnt = cnt +1

def build_single(path, blditm, logfile= None):
	bld = build(path+'\\'+blditm[0])
	bld.setlogfile(logfile)
	#cfg = {'CFG_OTA_BANK_MODE':'OTA_SINGLE_BANK','USE_FCT':'0'}
	cfg = None
	output = None
	for i in range(len(blditm)):
		if i == 0:
			continue
		if(type(blditm[i]) is dict):
			cfg = blditm[i]
		elif(type(blditm[i]) is list):
			output = blditm[i]
	#bld.config_proj(cfg)
	#bld.new_hex(['bin\\ota.hex','bin\\ota1.hex'])
	ret =bld(cfg, output)
	return ret

def log_err_check(flog):
	flog.seek(0,0)
	errnum,warnum,failnum=0,0,0
	while(True):
		logstr = flog.readline()
		if(len(logstr) == 0):
			break #read completed
		if(logstr.find('Error(s)')>0 and logstr.find('Warning(s)')>0 ):
			errnum =errnum+ int(logstr[logstr.find('-')+1:logstr.find('Error(s)')])
			warnum = warnum+int(logstr[logstr.find('Error(s)')+9: logstr.find('Warning(s)')])
		if(logstr.find('prj build fail check _bld.txt')>0):
			failnum = failnum+1

	return errnum,warnum,failnum

		
def build_prj(param, path):
	global dict_build_list
	global dict_yml_name
	if len(dict_build_list) == 0:
		print('dict_build_list is blank.-lcfg first')
		return
	lprj = list(dict_build_list)
	if(path == ''):
		#path = 'Trunk'
		path = '..'
	if(path[-1] == '\\'):
		path = path[:-1]
	id = -1
	if(param is None):
		list_prj()
		print('input id:')
		id = int(input())
		if(id < 0 or id >= len(lprj)):
			print('input out of range, exiting')
			exit()
	elif(param[0] in lprj):
		id = lprj.index(param[0])
	elif(param[0] == 'all'):
		id = -1
	else:
		print('Error build parameter, exiting')
		exit()
		
	bldT0 = time.perf_counter()	
	if(id == -1):#build all
		logfile = open('buildlog_'+dict_yml_name+'_'+datetime.strftime(datetime.now(),'%Y%m%d%H%M%S')+'.txt', 'w+')
		for prjname in lprj:
			prjitm = dict_build_list[prjname]
			#print(prjitm)
			logfile.write('-'*88+'\n'+'ProjName: '+prjname)
			ret=build_single(path, prjitm, logfile)
			if(ret==False):
				logfile.write('\n\n*****prj build fail check _bld.txt****\n\n')
		[e,w,f]=log_err_check(logfile)
		logfile.write('\n\n'+'-'*88+'\n'+'Total Err %d Warning %d Fail %d\n'%(e,w,f))
		print('\n\n'+'-'*88+'\n'+'Total Err %d Warning %d Fail %d\n'%(e,w,f))
		
	else:
		logfile = open('buildlog_'+datetime.strftime(datetime.now(),'%Y%m%d%H%M%S')+'.txt', 'w')
		prjname = lprj[id]
		prjitm = dict_build_list[prjname]
		logfile.write('-'*88+'\n'+'ProjName: '+prjname)
		#logfile.write('------------------------------------------------\n'+prjname)
		build_single(path, prjitm, logfile)


	checkTimeCost('All Build Finished',bldT0,1,logfile)
	return
		
def main(argv):
	dict_param = get_param(argv[1:])
	print(dict_param)
	if(dict_param == {}):
		help()
		return
		
	if('help' in dict_param):
		help(dict_param['help'])
		return
	
	if('clear' in dict_param):
		clear_log()
		return

	if('listconfig' in dict_param):
		if(dict_param['listconfig'] is None):
			dict_param['listconfig'] = ['sdk_build.yml']
			#print('configure file needed to choose')
			#exit()
		list_config(dict_param['listconfig'])
		#return
	else:
		dict_param['listconfig'] = ['sdk_build.yml']
		list_config(dict_param['listconfig'])
		
	if('list' in dict_param):
		list_prj()
		return
	
	path = ''
	if('path' in dict_param):
		if(dict_param['path'] is None):
			print('Path need input!')
			exit()
		if(os.path.exists(dict_param['path'][0]) == False):
			print('Path is not exist:', dict_param['path'][0])
			exit()
		path = dict_param['path'][0]
	
	if('version' in dict_param):
		try:
			list_ver = dict_param['version'][0].split('.')
			major = int(list_ver[0])
			minor = int(list_ver[1])
			revision = int(list_ver[2])
			test_build = ''
			if(len(list_ver) == 4):
				test_build = list_ver[3]
			make_version_file(path,major,minor,revision,test_build)
		except:
			traceback.print_exc()
			print('-version parameter is not correct, please check:',  dict_param['version'])
			return
	if('build' in dict_param):
		build_prj(dict_param['build'], path)
		return
	#bld = build(argv[1])
	#bld.build_check()
	#cfg = {'CFG_OTA_BANK_MODE':'OTA_SINGLE_BANK','USE_FCT':'0'}
	#bld.config_proj(cfg)
	#bld.new_hex(['bin\\ota.hex','bin\\ota1.hex'])
	#bld(cfg, ['bin\\ota.hex','bin\\ota2.hex'])
		
if __name__ == '__main__':
	#sys.argv = ['.\\sdk_build.py', '-lcfg', 'newname','-list']   #ok  config file not exist
	#sys.argv = ['.\\sdk_build.py', '-lcfg', '-list']  #no config file name  #ok
	#sys.argv = ['.\\sdk_build.py', '-lcfg', 'sdk_build', '-list']  # ok #config file exist
	main(sys.argv)
