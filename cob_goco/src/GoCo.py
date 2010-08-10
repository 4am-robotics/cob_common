# module GoCo 

import  socket, struct, cPickle, sys
import SocketServer
import thread
import linecache
import inspect
import fcntl

# server classes ##########################################

class GoCoRequestHandler(SocketServer.StreamRequestHandler):

  def handle(self):
    # set FD_CLOEXEC on socket stream to avoid inheritance to a sub-process,
    # if the called function forks
    flags = fcntl.fcntl(self.wfile, fcntl.F_GETFD)
    fcntl.fcntl(self.wfile, fcntl.F_SETFD, flags | fcntl.FD_CLOEXEC)

    n = struct.unpack("<i", self.rfile.read(4))[0]
    method, params = cPickle.loads(self.rfile.read(n))
    try:
      func = self.server.instance
      for id in method.split('.'):
        #The following 8 commands require special handling on servers side
        if id == "dir":
          if params:
            if params != ('',):
              params = params[0].rstrip(".")
              for param in params.split('.'):
                func = getattr(func, param)
          response = dir(func) 
        elif id == "type":
          if params:
            if params != ('',):
              params = params[0].rstrip(".")
              for param in params.split('.'):
                func = getattr(func, param)
          response = str(type(func))
        elif id == "_class":
            _class = func.__class__
            classname = _class.__name__
            response = classname
        elif id == "getargspec":
            import inspect
            if params:
                if params != ('',):
                  params = params[0].rstrip(".")
                  for param in params.split('.'):
                    func = getattr(func, param)
            response = inspect.getargspec(func) 
        elif id == "modulename":
            module = func.__module__
            response = module
        elif id == "filename":
            filenam = func.__file__
            response = filenam
        elif id == "getdir":
            dirpref = sys.prefix
            direxpref = sys.exec_prefix
            dirs = dirpref + "," + direxpref
            response = dirs
        else:
          func = getattr(func, id)
      if id=="dir":
        pass
      elif id=="type":
        pass
      elif id=="_class":
        pass
      elif id=="modulename":
        pass
      elif id == "getargspec":
        pass
      elif id == "filename":
        pass
      elif id == "getdir":
          pass
      else:
        response = apply(func, params)
        
    except:
      response = sys.exc_value
    try:
      data = cPickle.dumps(response, 1)
    except TypeError:
      data = cPickle.dumps(sys.exc_value, 1)
    self.wfile.write(data)

###########################################################

class Server(SocketServer.TCPServer):

  def __init__(self, server_address, instance):
    SocketServer.TCPServer.__init__(self, server_address, GoCoRequestHandler)
    self.instance = instance
    thread.start_new_thread(self.serve_forever, ())
  
  def server_bind(self):
    self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    SocketServer.TCPServer.server_bind(self)


# client classes ##########################################

class _Method:
  """ documentation _Method"""
  def __init__(self, request, name):
    self.request = request
    self.name = name

  def __getattr__(self, name):
    return _Method(self.request, "%s.%s" % (self.name, name))
    
  def __call__(self, *args):
    return self.request(self.name, args)

  def copyDict(self):
    for key in self.request("%s.__dict__.keys" % self.name, ()):
      try:
        val = self.request("%s.__dict__.get" % self.name, (key,))
        setattr(self, key, val)
      except:# (TypeError, ImportError):
        pass


    

###########################################################

class Proxy:

  def __init__(self, server_address):
    self._server_address = server_address

  def __getattr__(self, name):
    return _Method(self._request, name)

  def _request(self, methodname, params):
    data = cPickle.dumps((methodname, params), 1)
    n = struct.pack("<i", len(data))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(self._server_address)
    s.send(n + data)
    try:
      data = s.makefile("rb").read()
    except Exception, msg:
      print "Exception '%s' when receiving from" % msg, self._server_address
      raise
    s.close()
    response = cPickle.loads(data)
    if isinstance(response, Exception):
      raise response
    return response
