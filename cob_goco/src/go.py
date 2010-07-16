##########################################################################
#                                                                        #
#  GO - Eg(G)secution Framework for Robotics Applicati(O)ns 	         #
#                                                                        #
#       Release: 040707                                                  #
#                                                                        #
#  (C) Copyright 2005 Fraunhofer Institute                               #
#                     Manufacturing Engineering                          #
#         --- /       and Automation                                     #
#       /     \                                                          #
#      =========                                                         #
#      |       |      Winfried Baum        <wmb(at)ipa.fraunhofer.de>    #
#      =========<<    Christopher Parlitz  <cip(at)ipa.fraunhofer.de>    #
#       \     /                                                          #
#         ---                                                            #
#                                                                        #
#  GO is free software; you can redistribute it and/or modify            #
#  it under the terms of the GNU General Public License as published by  #
#  the Free Software Foundation; either version 2 of the License, or     #
#  (at your option) any later version.                                   #
#                                                                        #
#  GO is distributed in the hope that it will be useful,                 #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#  GNU General Public License for more details.                          #
#                                                                        #
##########################################################################


from time import sleep, time
import thread, os, sys
from subprocess import Popen, PIPE

debug = 1

def dprint(s):
  if debug:
    print s

class ActivityError(StandardError):
  def __init__(self, errno=0, msg=None):
    self.errno = errno
    if msg:
      self.msg = msg
    else:
      self.msg = self.messages[errno]

  def __repr__(self):
    return self.msg

  def __str__(self):
    return self.msg

  messages = []

def addError(msg = "error"):
  ActivityError.messages.append(msg)
  return len(ActivityError.messages) - 1

errUnknown = addError("unknown error")
errTimeout = addError("activity has timed out")
errStopped = addError("activity has been stopped")

AS_RUNNING=0
AS_STOPPING=1
AS_ENDED=2
AS_STOPPED=3
AS_TIMEOUT=4
AS_FAILED = 5

def altWait(*activities):
  "waits until at least one of the activities has ended"
  dprint("waiting for following activities")
  waitLock = thread.allocate_lock()
  for a in activities:
    dprint("  %s" % a.name)
    a.runLock.addWaitLock(waitLock)
  waitLock.acquire()

def altWaitCycle(*activities):
  "waits until the current cycle of at least one of the activities has ended"
  dprint("waiting for following activities")
  waitLock = thread.allocate_lock()
  for a in activities:
    dprint("  %s" % a.name)
    a.cycleLock.addWaitLock(waitLock)
  waitLock.acquire()

def check():
  "checks, if there is a request to stop the current activity"
  a = Async.activityTable.get(thread.get_ident())
  if a != None:
    if a.state == AS_STOPPING:
      raise ActivityError(errStopped)
    a.checkTimeout()

def csleep(duration):
  endTime = time() + duration
  while duration > 0:
    sleep(min(duration, 0.1))
    check()
    duration = endTime - time()

class _MultiLock:
  def __init__(self):
    self.intLock = thread.allocate_lock()
    self.waitLocks = []
    self.locked = 1

  def addWaitLock(self, waitLock):
    self.intLock.acquire()
    if self.locked:
      self.waitLocks.append(waitLock)
      if not waitLock.locked():
        waitLock.acquire()
    else:
      if waitLock.locked():
        waitLock.release()
    self.intLock.release()

  def unlock(self):
    self.intLock.acquire()
    self.locked = 0
    for lock in self.waitLocks:
      try:
        lock.release()
      except thread.error:
        pass
    self.waitLocks = []
    self.intLock.release()

  def unlockLock(self):
    self.intLock.acquire()
    for lock in self.waitLocks:
      lock.release()
    self.waitLocks = []
    self.intLock.release()

  def wait(self):
    waitLock = thread.allocate_lock()
    self.addWaitLock(waitLock)
    # TODO to be changed later
    while waitLock.locked():
      check()
      sleep(0.1)
    waitLock.acquire()

  def locked(self):
    return self.locked


class Async:
  "one shot activity"
  activityTable = {}

  def __init__(self, func, args, timeout=None, name=None):
    """
    func:    function to be executed
    args:    tuple of the arguments to be passed to the function
    timeout: time, after which the function is automatically stopped
    name:    name of the activity (for debug)"""
    self.func = func

    if timeout:
      self.endTime = time() + timeout
    else:
      self.endTime = None

    if name:
      self.name = name
    else:
      self.name = "%s%s" % (func.__name__, args)

    self.runLock = _MultiLock()
    self.returnVal = None
    self.state = AS_RUNNING

    dprint("starting activity %s" % self.name)
    self.ident = thread.start_new_thread(self.run, (args,))
    Async.activityTable[self.ident] = self

  def __repr__(self):
    return self.name

  def run(self, args):
    try:
      self.returnVal = apply(self.func, args)
      dprint("activity %s ended" % self.name)
      self.state = AS_ENDED
    except ActivityError, e:
      if e.errno == errStopped:
        dprint("activity %s stopped" % self.name)
        self.state = AS_STOPPED
      elif e.errno == errTimeout:
        dprint("activity %s timed out" % self.name)
        self.state = AS_TIMEOUT
      else:
        dprint("activity %s failed: %s" % (self.name, e.msg))
        self.state = AS_FAILED

    self.runLock.unlock()
    
    del Async.activityTable[self.ident]

  def getState(self):
    "returns the current execution state of the activity"
    return self.state

  def getReturnVal(self):
    """
    returns the return value of the function, after this has successfully
    ended"""
    return self.returnVal

  def stop(self):
    "advise the activity to stop itself"
    dprint("stopping activity %s ..." % self.name)
    self.state = AS_STOPPING
    ### stop also all Asyncs on which we wait

  def wait(self):
    """
    wait, until the activity has ended
    returns the returnvalue of the executed function"""
    dprint("waiting for activity %s ..." % self.name)
    self.runLock.wait()
    if self.state == AS_STOPPED:
      raise ActivityError(errStopped)
    elif self.state == AS_TIMEOUT:
      raise ActivityError(errTimeout)
    return self.returnVal

  def checkTimeout(self):
    if self.endTime:
      if time() > self.endTime:
        raise ActivityError(errTimeout)

class AsyncCyclic(Async):
  "cyclic activity"
  def __init__(self, func, args, timeout=None, cycleTime=0, name=None):
    """
    func:    function to be executed cyclically
    args:    tuple of the arguments to be passed to the function
    timeout: time, after which the function is automatically stopped
    cycleTime: cycle time, in which the function is executed
    name:    name of the activity (for debug)"""
    if not name:
      name = "cyclic %s%s" % (func.__name__, args)
    self.cyclicFunc = func
    self.cycleTime = cycleTime
    self.cycleLock = _MultiLock()
    self.cycleLock.lock()
    self.lastReturnVal = None
    Async.__init__(self, self.runCyclic, (args,), timeout, name)

  def runCyclic(self, args):
    while 1:
      self.lastReturnVal = apply(self.cyclicFunc, args)
      self.cycleLock.unlockLock()
      check()
      if self.cycleTime:
        sleep(self.cycleTime)

  def waitCycle(self):
    """
    wait, until the current cycle has ended
    returns the return value of the function"""
    self.cycleLock.wait()
    return self.lastReturnVal

  def waitCond(self, cond=lambda x: x):
    """
    wait, until the return value of the function satisfies a condition
    cond: boolean function to be applied on the return value of the
          cyclic function
    returns the last return value (which satisfies the condition)"""
    while not cond(self.waitCycle()):
      pass
    return self.lastReturnVal

  def getLastReturnVal(self):
    "returns the return value of the last successful cycle"
    return self.lastReturnVal


class AsyncProc(Async):
  
  if sys.platform == 'win32' : #if os is broken
    if os.system("taskkill -? > nul")==0:
      myKill = lambda self: os.system("taskkill /F /T /PID " + str(self.pid) + " >nul")
    else:
      # Install a kill function
      print 'Please install "taskkill"'
      myKill = lambda self: None
  else: #Linux
    myKill = lambda self: os.kill(self.pid)
  
  def __init__(self, command, procName=None, input=None, timeout=None, name=None):
    self.procName = procName
    if name == None:
      name = command
    Async.__init__(self, self.execute, (command, input), timeout, name)
    if timeout:
      thread.start_new_thread(self.killThread, (timeout,))
  
  def execute(self, command, input):
    dprint("execute", command, input)
 #   fout, fin, ferr = popen2.popen3(command)
    pp = Popen(command.split(),stdin=PIPE, stdout=PIPE, stderr=PIPE)
    self.pid = pp.pid
    dprint("Popen successful")
   
    if input:
      pp.stdin.write(input)
      pp.stdin.close()

    out = pp.stdout.read()
    err =  pp.stderr.read()
    
    if self.state == AS_STOPPING:
      raise ActivityError(errStopped)
    elif self.state == AS_TIMEOUT:
      raise ActivityError(errTimeout)
    return out, err

  def stop(self):
    "kill process"
    Async.stop(self)
    self.myKill()

  def killThread(self, timeout):
    endTime = time() + timeout
    while time() < endTime:
      sleep(0.5)
      if self.state != AS_RUNNING:
        break
    else:
      self.state = AS_TIMEOUT
      #os.system(AsyncProc.killApp + " " + self.procName)
      self.myKill()
