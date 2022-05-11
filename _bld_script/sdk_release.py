#! /usr/bin/env python
import os
import sys
import getopt
import traceback
import getpass
import platform
import git
import yaml
from datetime import datetime

giturl = 'git@192.168.1.11:gecko/bbb_sdk.git'
gitreleaseurl = 'git@192.168.1.11:phyplusdev/release_bbb_sdk.git'
#for test
#giturl = 'git@git.phyplusinc.com:yu.zheng/bbb_sdk.git'
#gitreleaseurl = 'git@git.phyplusinc.com:yu.zheng/hello_w.git'
#locallocation = '../git_python/clone3'

locallocation = os.path.dirname(os.path.realpath(__file__))
locallocation = os.path.dirname(locallocation)

# curpath = os.path.dirname(os.path.realpath(__file__))
# result=os.path.exists("newsdkrelease.yml")
# if result:
#     yamlpath = os.path.join(curpath, "newsdkrelease.yml")  #3 is people-edited file
# else:
#     yamlpath = os.path.join(curpath, "sdkelease.yml")  #2 is default file
# f = open(yamlpath, 'r', encoding='utf-8')
# c = f.read()
# d = yaml.load(c,Loader=yaml.FullLoader)

# unexport_list = d
#unexport_list = [[0, 'components\\arch'],
#                 [1,'example\\ble_peripheral\\simpleBlePeripheral\\simpleBlePeripheral.uvprojx']]

usage = 'Usage: sdk_release.py [--help] [--branch] [--tag] [--trunk] [--merge] [--release=......]\n' + \
        '[--mkbranch=......] [--mktag=......]\n'
optioninfo = '--help: help\n' + \
             '--branch: change branch,export a remote branch to local\n' + \
             '--trunk: download from git\n' + \
             '--merge: fetch + merge , make the local updated\n' + \
             '--release=\'commit test README14\': upload to git and add a commit message\n' + \
             '--mkbranch=\'branch name\': create a branch in remote, and upload files to this new-created branch\n' + \
             '--mkbranch (format --mkbranch \'branch name\')\n' +\
             '--mktag=\'tag name\': create a tag (format --mktag tagname SHA \'commit message\')\n' + \
             '--2releaseR= \'TAG name\' \'build name\' \'cfg or None\' \'branch name\n' + \
             '--tag=\'tag name\' \'new branch name\': export a tag to local\n' +\
             '--rls=\'release config yaml\': export a tag to local push release remote repo\n'
helpstr = usage + optioninfo
#dest = 0

def pyver():
    vstr = platform.python_version()
    vlist = vstr.split('.')
    if (vlist[0] == '3'):
        return 3
    return 2


def getcmd():
    if pyver() == 3:
        return input()
    return raw_input()


def validcmd(cmd):
    return os.system(cmd + ' 1>nul 2>nul')


def branch_list():
    new_repo = git.Repo(locallocation)
    print('all branches are listed below')
    branch = new_repo.git.branch('-a')
    branch = branch.split('\n')

    branchlist = []
    for s in branch:
        if ('remotes' in s and 'HEAD' not in s):  # head point to default branch
            ssub = s.split('/')
            ssub = ssub[len(ssub) - 1]
            branchlist.append(ssub)
        if ('*' in s):
            print('local branch is ', s.split(' ')[1])

    print('branch list is', branchlist)
    return branchlist


def branch_chose():
    global giturl
    ret = 0
    if (not os.path.exists(locallocation)):
        print('no such folder')
        return

    branchlist = branch_list()
    print('Chose Branch: 0 -', len(branchlist) - 1)
    try:
        print('input branch number')
        branchNum = int(getcmd())            #number -1is the last one ,-2 is last but one .also can out of range
        #branchNum = int(2)
        ret = branchlist[branchNum]
    except:
        print('Input out of range')
        exit()
    return ret  #ret will be a string


def branch_checkout(arg):
    new_repo = git.Repo(locallocation)
    new_repo.git.checkout(arg)  # subfiles all be updated when change to another branch

    # check the local branch now
    branch = new_repo.git.branch('-a')  # in clone file
    branch = branch.split('*')
    print('local branch now is', branch[1].split('\n')[0])
    return


def sha_finder():
    new_repo = git.Repo(locallocation)

    log_message = new_repo.git.log()
    log_SHA = log_message.split(' ')[1]
    log_SHA = log_SHA.split('\n')[0]

    return log_SHA

def rls_config(fname):
    d=[]
    curpath = os.path.dirname(os.path.realpath(__file__))
    #result=os.path.exists(fname+'.yml')
    #if result:
    
    yamlpath = os.path.join(curpath, fname+'.yml')  
    if(os.path.exists(yamlpath)):
        pass 
    else:
        print('check file input--> rls_confgi')
        return d
    f = open(yamlpath, 'r', encoding='utf-8')
    c = f.read()
    d = yaml.load(c,Loader=yaml.FullLoader)
    return d
def sdk_rls(args):
    # global dest
    print(args)
    cfg=rls_config(args[0])

    if(len(cfg)==0):
        print('check release config file !!!')
        return
    else:
        print(' Tag : %s \n Release Branch:%s \n BuildCofing:%s \n'%(cfg['ReleaseTagName'],cfg['ReleaseBranchName'],cfg['BuildConfig']))
        
    # releasefolder = '\\release_' + datetime.strftime(datetime.now(), '%Y%m%d%H%M%S')
    # dest = locallocation + releasefolder
    # repo = git.Repo.init(path=releasefolder)
    # new_repo = git.Repo.clone_from(url=giturl, to_path=dest)
    print(locallocation)
    new_repo = git.Repo(locallocation)
    print('tag is ', cfg['ReleaseTagName'])

    log_SHA = sha_finder()
    tag_args = [log_SHA, 'release tag']
    ret=tag_create(cfg['ReleaseTagName'][0], tag_args)
    if(ret==False):
        return
    print('remove protect file')
    protectfile(cfg['ProtectFile'])
    print('build', cfg['BuildConfig'])
    # choose sdk_build.yaml
    #os.system('python .\sdk_build.py ' + '-l ' + cfg['BuildConfig'][0] + ' -b ' + cfg['BuildConfig'][1])   # with modified sdk_build.yaml
    os.system('python .\sdk_build.py ' + '-l ' + cfg['BuildConfig'][0] + ' -b all')   # with modified sdk_build.yaml

    # find latest log file
    base_dir = locallocation+'\\_bld_script\\'
    filelist = os.listdir(base_dir)
    filelist.sort(key=lambda fn: os.path.getmtime(base_dir + fn) if not os.path.isdir(base_dir + fn) else 0)

    logfile = filelist[-1]
    print('latest file is', logfile)

    [err,warn,fail] = log_err_check(logfile)    
    if((err+warn)>0 or fail>4):
        print('!!! release build check fail !!! Err %d Warning %d Fail %d\n'%(err,warn,fail))
        return
    remote = new_repo.git.remote('--v')
    if gitreleaseurl in remote:
        for i in range(len(remote.split('\n'))):
            print('i = ', i)
            print(remote.split('\n')[i])
            print(remote.split('\n')[i].split('\t')[1].split(' ')[0])
            if (remote.split('\n')[i].split('\t')[1].split(' ')[0] == gitreleaseurl):
                new_origin = remote.split('\n')[i].split('\t')[0]  # remote url name already exist, take it
                print('the release reposity name is ', new_origin)
                break
    else:
        new_origin = 'release'
        new_repo.git.remote('add', new_origin, gitreleaseurl)   # origin2 is the name of release reposity

    #branch_name = args[3]+'_'+ args[0]+ '_'+ datetime.strftime(datetime.now(), '%Y%m%d%H%M%S')
    branch_name = cfg['ReleaseBranchName'][0]
    new_repo.git.checkout('-b', branch_name) # create local branch.will be pushed to release roposity#branch name with time
    #new_repo.git.pull('origin', 'master')
    # new_repo.git.fetch(new_origin)
    os.system('git branch --set-upstream-to='+ new_origin+'/'+branch_name)
    #new_repo.git('branch', '--set-upstream-to', new_origin +'/master')
    new_repo.git.add('.')
    #new_repo.git.add('-f', 'buildlog_*.txt')
    new_repo.git.commit('-m', 'sdk release repo push')
    new_repo.git.push(new_origin, branch_name)
    '''
    # for add orgin2 tag
    new_repo.git.remote('rm', 'origin')  #remove remote giturl origin
    #new_repo.git.tag('|', 'xargs', '-I', '{}', 'git', 'tag', '-d', '{}')#git tag | xargs -I {} git tag -d {} #delete all local tag
    tag_delete = new_repo.git.tag()
    tag_delete = tag_delete.split('\n')
    for i in range(len(tag_delete)):
        new_repo.git.tag('-d', tag_delete[i])  #delete all local tag

    new_repo.git.fetch('--tags')  #only gitreleaseurl exist
    new_repo.git.remote('add', 'origin', gitreleaseurl)  #change origin's url for calling of tag_create()

    print('tag is ', args[0])

    log_SHA = sha_finder()
    tag_args = [log_SHA, 'release tag']
    tag_create(args[0], tag_args)
    print('now the origin url is release reposity')
    '''
    return


def sdk_2release(args):
    # global dest
    print(args)
    # releasefolder = '\\release_' + datetime.strftime(datetime.now(), '%Y%m%d%H%M%S')
    # dest = locallocation + releasefolder
    # repo = git.Repo.init(path=releasefolder)
    # new_repo = git.Repo.clone_from(url=giturl, to_path=dest)
    new_repo = git.Repo(locallocation)
    print('tag is ', args[0])

    log_SHA = sha_finder()
    tag_args = [log_SHA, 'release tag']
    tag_create(args[0], tag_args)

    print('remove protect file')
    #protectfile()
    print('build', args[1])

    # choose sdk_build.yaml
    if (args[2] == 'None'):
        os.system('python .\sdk_build.py -build .\\ -b '+args[1])    # sdk_build
    else:
        os.system('python .\sdk_build.py ' + '-lcfg ' + args[2] + ' -b ' + args[1])   # with modified sdk_build.yaml

    # find latest log file
    base_dir = locallocation+'\\'
    filelist = os.listdir(base_dir)
    filelist.sort(key=lambda fn: os.path.getmtime(base_dir + fn) if not os.path.isdir(base_dir + fn) else 0)

    logfile = filelist[-1]
    print('latest file is', logfile)

    build_error = build_result(logfile)
    if(build_error):
        print('error occurs during building')
        return
    remote = new_repo.git.remote('--v')
    if gitreleaseurl in remote:
        for i in range(len(remote.split('\n'))):
            print('i = ', i)
            print(remote.split('\n')[i])
            print(remote.split('\n')[i].split('\t')[1].split(' ')[0])
            if (remote.split('\n')[i].split('\t')[1].split(' ')[0] == gitreleaseurl):
                new_origin = remote.split('\n')[i].split('\t')[0]  # remote url name already exist, take it
                print('the release reposity name is ', new_origin)
                break
            else:
                new_origin = 'origin2'
                if i == (len(remote.split('\n')) - 1):
                    new_repo.git.remote('add', new_origin, gitreleaseurl)   # origin2 is the name of release reposity

    branch_name = args[3]+'_'+ args[0]+ '_'+ datetime.strftime(datetime.now(), '%Y%m%d%H%M%S')
    new_repo.git.checkout('-b', branch_name) # create local branch.will be pushed to release roposity#branch name with time
    new_repo.git.pull('origin', 'master')
    # new_repo.git.fetch(new_origin)
    os.system('git branch --set-upstream-to '+ new_origin +'/master')
    #new_repo.git('branch', '--set-upstream-to', new_origin +'/master')
    new_repo.git.add('.')
    new_repo.git.add('-f', 'buildlog_*.txt')
    new_repo.git.commit('-m', 'commit new-added file')
    new_repo.git.push(new_origin, branch_name)
    '''
    # for add orgin2 tag
    new_repo.git.remote('rm', 'origin')  #remove remote giturl origin
    #new_repo.git.tag('|', 'xargs', '-I', '{}', 'git', 'tag', '-d', '{}')#git tag | xargs -I {} git tag -d {} #delete all local tag
    tag_delete = new_repo.git.tag()
    tag_delete = tag_delete.split('\n')
    for i in range(len(tag_delete)):
        new_repo.git.tag('-d', tag_delete[i])  #delete all local tag

    new_repo.git.fetch('--tags')  #only gitreleaseurl exist
    new_repo.git.remote('add', 'origin', gitreleaseurl)  #change origin's url for calling of tag_create()

    print('tag is ', args[0])

    log_SHA = sha_finder()
    tag_args = [log_SHA, 'release tag']
    tag_create(args[0], tag_args)
    print('now the origin url is release reposity')
    '''
    return


def build_result(logfile):
    f = open(logfile, "r")
    builderror = 0
    while (True):
        logstr = f.readline()
        if (len(logstr) == 0):
            break  # read conpleted
        if (logstr.find('Error(s)') > 0 and logstr.find('Warning(s)') > 0):
            errnum = int(logstr[logstr.find('-') + 1:logstr.find('Error(s)')])
            warnum = int(logstr[logstr.find('Error(s)') + 9: logstr.find('Warning(s)')])
            logstr = ''.join(logstr)
            prjname = logstr.split('\\')[2].split('.')[0]
            if (errnum + warnum):
                print('Error %d, Warning %d' % (errnum, warnum))
                print('project name', prjname)
                if (errnum):
                    print('error in ', prjname)
                    builderror = builderror + errnum
    return builderror
def log_err_check(fname):
    flog=open(fname,"r")
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
    flog.close()
    return errnum,warnum,failnum

def export_sdk():
    if (os.path.exists(locallocation)):
        if os.path.getsize(locallocation):
            print(os.path.getsize(locallocation))
            print('git file exist, no need to clone')
        else:
            repo = git.Repo.init(path='.')
            new_repo = git.Repo.clone_from(url=giturl, to_path=locallocation)
    else:  # no folder
        os.mkdir(locallocation)  # create a folder
        repo = git.Repo.init(path='.')
        new_repo = git.Repo.clone_from(url=giturl, to_path=locallocation)


    print('protect code')
    #protectfile()#can repeat delete
    return 0


def sdk_merge():
    new_repo = git.Repo(locallocation)
    new_repo.git.pull()
    return 0

def get_bld_path(bld):
    bldList=[]
    for p in bld:
        bldList.append(os.path.split(bld[p][0])[0])
    return bldList

def get_dir_path(path,depth=None):
    a=[]
    rD = path.count('\\')
    for root,dirs,files in os.walk(path):
        for name in dirs:
            if(depth):
                if(root.count('\\')-rD==depth-1):    
                    a.append(os.path.join(root,name))
            else:
                a.append(os.path.join(root,name))
    return a

def find_del_fold(bld,cur):
    delPath=[]
    
    for a in cur:
        flg=1
        for c in bld:
            if(a.find(c)>0):
                if(a.split('\\')[-1]==c.split('\\')[-1]):
                    flg=0
                break
        if(flg):
            delPath.append(a)
    return delPath

def protectfile(cfg):
    print('protect code')
    print(cfg)
    bld = get_bld_path(rls_config(cfg[0]['rls_example'][0]))
    print(bld)
    cur=get_dir_path(os.path.join(locallocation,'example'),2)
    remove_example=find_del_fold(bld,cur)
    
    for del_path in remove_example:
        # print(del_path)
        f_path = 'aliGenie_bleMesh'
        if f_path in del_path :
            print(del_path)
        else:
            cmd = 'rd /s /q ' + del_path
            print(cmd)
            validcmd(cmd)

    for del_path in cfg[2]['remove_folder']:
        #libsrcpath = 'clonefile'+ '\\' + del_path[1]  #clone file cannot have a space inside
        libsrcpath = locallocation.split('/')[-1] + '\\' + del_path  # clone file cannot have a space inside
        cmd = 'rd /s /q ' + libsrcpath  # rd means delete, s means project and subproject, q means no notification(quiet) then is the address
        print(cmd)
        validcmd(cmd)   #os.system(cmd)
    
    for del_path in cfg[1]['remove_file']:
        libsrcpath = locallocation.split('/')[-1] + '\\' + del_path  # clone file cannot have a space inside
        cmd = 'del /f /q ' + libsrcpath  # part of files will be deleted
        print(cmd)
        validcmd(cmd)   #os.system(cmd)
    
    return 0


def branch_create(arg):
    new_repo = git.Repo(locallocation)

    branch = new_repo.git.branch('-a')  # in clone file
    if arg not in branch:
        new_repo.git.branch(arg)  # subfiles all be updated when change to another branch
    branch_checkout(arg)

    # upload files to this new branch
    new_repo.git.add('.')  # add all
    #new_repo.git.commit(m='created a new branch')
    new_repo.git.push('--set-upstream','origin',arg)
    print('push successfully')
    return


def tag_create(arg, args):
    new_repo = git.Repo(locallocation)
    tagname = arg
    print(args)
    if len(args) == 0:
        print('no arguments: SHA and commit message')
        print('no SHA value, go to git and find it')
        return False

    elif len(args) == 1:
        print('no commit message')
        return False

    SHA = args[0]
    msg = args[1]

    #if (msg == 'release tag'):
        #new_repo = git.Repo(dest)

#    remote = new_repo.git.remote('--v')
#    if gitreleaseurl in remote:
#        new_repo.git.fetch(gitreleaseurl)
#        new_repo.git.fetch(giturl)
#    else:
#        new_repo.git.fetch(giturl)

    new_repo.git.fetch('origin')
    alltag = new_repo.git.tag()
    if tagname in alltag:
        #tag_detail = new_repo.git.show(tagname)
        #tag_detail = new_repo.git.tag('-n')
        print('the same tag name already exist,change a name，use --mktag')
        return False
    if (msg == 'release tag'):
        new_repo.git.tag('-a', tagname, SHA, '-m', 'create a release tag')
    else:
        new_repo.git.tag('-a', tagname, SHA, '-m', msg)  #git tag -a TAG4 4a9fc2769f17475c62768bb0a6bbc2fd81ecd593 -m 'a test for creating tag'
    new_repo.git.push('origin', 'tag', tagname)    #origin will change before calling this func
    alltag = new_repo.git.tag()
    if tagname in alltag:
        print('tag created and pushed to git successful')
        return True
    else:
        print('tag created failed')
        return False


def tag_checkout(arg, args):
    tag_name = arg
    branch_name = args[0]
    new_repo = git.Repo(locallocation)
    alltag = new_repo.git.tag()
    if tag_name not in alltag:
        print('tag name chose fail')
        return
    new_repo.git.checkout('-b', branch_name, tag_name)
    new_repo.git.push('--set-upstream', 'origin', branch_name) #push new branch to remote
    new_repo.git.branch('--set-upstream-to=origin/'+branch_name, branch_name) #associate remote branch with local branch
    print('tag checkout successfully and you can make changes on new branch')
    print('tag checkout successfully & a new branch has been created starting from tag')

    return


def main(argv):
    global helpstr
    print(argv)

    if (validcmd('git help') == 1):     #no git package
        print('please install command line client for GIT')
        return
    opts = []
    if (len(argv) == 1):   #just run the python file
        print(helpstr)
        return
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'h?', ['branch', 'merge', 'release=', '2releaseR', 'rls','mkbranch=', 'mktag=', 'tag=','trunk', 'help', 'dummy'])
        #if (len(opts) + len(args) != len(argv) - 1):
            #print('args after =')
            #print(helpstr)
    except:
        print("getopt.getopt error")
        print(helpstr)
        return

    for opt, arg in opts:
        if opt in ('h', '--help', 'help'):
            print(helpstr) #if there is 'help' in opts, print out help and return back
            return

#not use
    if (len(args) != 1):
        dest = '.\\release'  #release folder will be added into the path
    else:
        dest = '.\\' + args[0]   #add a new folder created by your argument


    for opt, arg in opts:
        print('opt and arg in opts are ')
        print(opt, arg)
        if opt[0] in ('h', '--help', 'help'):
            print(helpstr)
            return

        if opt == '--branch':
            arg = branch_chose()
            if (arg == None):
                print('no such folder,please export to local using --trunk first')
                break
            branch_checkout(arg)
            break

        elif opt == '--trunk':
            export_sdk()
            print('exported')
            break

        elif opt == '--merge':
            sdk_merge()
            print('merged')
            break

        elif opt == '--2releaseR':
            if (len(args) != 4):
                print('4 arguments:\n1.TAG name\n2.which project to build or all')
                print('3.configuration file(sdk_build) or default(None)\n4.branch created in destination reposity\n')
                break
            sdk_2release(args)
            print('uploaded')
            break
        elif opt == '--rls':
            if(len(args)!=1):
                print('input release yaml file')
                break
            sdk_rls(args)
            print('uploaded')
            break
        elif opt == '--mkbranch':
            if (arg == ''):
                print('you need to add a new branch name(format --mkbranch \'branch name\')')
                break
            branch_create(arg)
            print('new branch has been created and local repository uploaded to new branch')
            break

        elif opt == '--mktag':
            if (arg == '' or args == ''):
                print('you need to add a new tag name(format --mktag tag name SHA \'commit message\')')
                break
            tag_create(arg, args)

            break

        elif opt == '--tag':
            new_repo = git.Repo(locallocation)
            print('all tags are listed below')
            tag = new_repo.git.tag()
            print(tag)

            if (arg == '' or len(args) == 0):
                print('you need to add a tag name & a branch name(branch will be created later)')
                print('format --tag \'tag name\' branch_name')
                break
            print('branch name ', args[0])
            #print(new_repo.git.branch())   #just local branch

            branchlist = branch_list()

            if args[0] in branchlist:
                print('branch exist. new one')
                #print('branch has been created,tag has been checkouted out')
                #print('now associate local branch with remote branch ')
                #new_repo.git.push('--set-upstream', 'origin', args[0]) #push new branch to remote
                #new_repo.git.branch('--set-upstream-to=origin', args[0])  # associate remote branch with local branch
                break
            sdk_merge()#pull down the new created tag from git
            tag_checkout(arg, args)

            break

        elif opt == '--release':
            if (arg == ''):
                #print('Error!: \"--release=version\", \"version\" could not be empty (version is tag name)')
                print('you need to add commit message(format --release \'commit message\')')
                return
            print('remove protect file(second time)')
            #protectfile() #double delete ensure no protected files
            new_repo = git.Repo(locallocation)
            new_repo.git.add('.') #add all
            if (len(new_repo.git.status('-s')) != 0):  # new change
                #new_repo.git.fetch()
                #if (new_repo.git.diff('newbranch','origin/newbranch')):
                sdk_merge()

                commit_step = new_repo.git.commit(m=arg)
                print(commit_step)  # need to judge if is something new-added)

                new_repo.git.push()

                # check if its  commit
                commitSHA = commit_step.split(' ')[1].split(']')[0]
                log_message = new_repo.git.log()
                log_SHA = log_message.split(' ')[1][:7]
                if (commitSHA == log_SHA):
                    print('commit successful')
                else:
                    print('commit message not equal')
                # push successful
                branch = new_repo.git.branch('-a')  # in clone file
                branch = branch.split('*')
                print('local branch now is', branch[1].split('\n')[0])
                localbranch = branch[1].split('\n')[0]
                localbranch = localbranch.replace(' ', '')
                remotebranch = 'origin/'+localbranch
                remotebranch = remotebranch.replace(' ', '')
                if (new_repo.git.diff(localbranch, remotebranch)):
                    print('failed to push')

                else:
                    print('push successfully')

            else:
                print('nothing new to be added')
        break


if __name__ == '__main__':
    main(sys.argv)



