import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion
from distutils import log


if_dl = lambda s: s if False else ''


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)
            self.write_stub(os.curdir, ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)
        
    def write_stub(self, output_dir, ext, compile=False):
        log.info("writing stub loader for %s to %s", ext._full_name,
                 output_dir)
        stub_file = (os.path.join(output_dir, *ext._full_name.split('.')) +
                     '.py')
        if compile and os.path.exists(stub_file):
            raise DistutilsError(stub_file + " already exists! Please delete.")
        if not self.dry_run:
            f = open(stub_file, 'w')
            f.write(
                '\n'.join([
                    "def __bootstrap__():",
                    "   global __bootstrap__, __file__, __loader__",
                    "   import sys, os, pkg_resources, imp" + if_dl(", dl"),
                    "   __file__ = pkg_resources.resource_filename"
                    "(__name__,%r)"
                    % self.get_ext_fullpath(ext.name),
                    "   del __bootstrap__",
                    "   if '__loader__' in globals():",
                    "       del __loader__",
                    if_dl("   old_flags = sys.getdlopenflags()"),
                    "   old_dir = os.getcwd()",
                    "   try:",
                    "     os.chdir(os.path.dirname(__file__))",
                    if_dl("     sys.setdlopenflags(dl.RTLD_NOW)"),
                    "     imp.load_dynamic(__name__,__file__)",
                    "   finally:",
                    if_dl("     sys.setdlopenflags(old_flags)"),
                    "     os.chdir(old_dir)",
                    "__bootstrap__()",
                    ""  # terminal \n
                ])
            )
            f.close()
        if compile:
            from distutils.util import byte_compile

            byte_compile([stub_file], optimize=0,
                         force=True, dry_run=self.dry_run)
            optimize = self.get_finalized_command('install_lib').optimize
            if optimize > 0:
                byte_compile([stub_file], optimize=optimize,
                             force=True, dry_run=self.dry_run)
            if os.path.exists(stub_file) and not self.dry_run:
                os.unlink(stub_file)

setup(
    name='cmake_example',
    version='0.0.1',
    author='Guocong Song',
    author_email='songgc@gmail.co,',
    description='A test project using pybind11 and CMake',
    long_description='',
    ext_modules=[CMakeExtension('curve_fit')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)