def __bootstrap__():
   global __bootstrap__, __file__, __loader__
   import sys, os, pkg_resources, imp
   __file__ = pkg_resources.resource_filename(__name__,'build/lib.macosx-10.15-x86_64-3.7/pose_estimation.cpython-37m-darwin.so')
   del __bootstrap__
   if '__loader__' in globals():
       del __loader__

   old_dir = os.getcwd()
   try:
     os.chdir(os.path.dirname(__file__))

     imp.load_dynamic(__name__,__file__)
   finally:

     os.chdir(old_dir)
__bootstrap__()
