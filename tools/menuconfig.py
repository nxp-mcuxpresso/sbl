#
# File      : menuconfig.py
# This file is part of RT-Thread RTOS
# COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Change Logs:
# Date           Author       Notes
# 2017-12-29     Bernard      The first version
# 2018-07-31     weety        Support pyconfig
# 2019-07-13     armink       Support guiconfig

import os
import re
import sys
import platform
import shutil

# make sblconfig.h from .config

def is_pkg_special_config(config_str):
    ''' judge if it's CONFIG_PKG_XX_PATH or CONFIG_PKG_XX_VER'''

    if type(config_str) == type('a'):
        if config_str.startswith("PKG_") and (config_str.endswith('_PATH') or config_str.endswith('_VER')):
            return True
    return False


def mk_sblconfig(filename):
    try:
        config = open(filename, 'r')
    except:
        print('open config:%s failed' % filename)
        return

    sblconfig = open('sblconfig.h', 'w')
    sblconfig.write('/*\n')
    sblconfig.write('* Copyright 2021 NXP\n')
    sblconfig.write('* All rights reserved.\n')
    sblconfig.write('*\n')
    sblconfig.write('* SPDX-License-Identifier: BSD-3-Clause\n')
    sblconfig.write('*/\n\n')

    empty_line = 1

    sblconfig.write('#ifndef SBL_CONFIG_H__\n')
    sblconfig.write('#define SBL_CONFIG_H__\n\n')

    empty_line = 1

    for line in config:
        line = line.lstrip(' ').replace('\n', '').replace('\r', '')

        if len(line) == 0:
            continue

        if line[0] == '#':
            if len(line) == 1:
                if empty_line:
                    continue

                sblconfig.write('\n')
                empty_line = 1
                continue

            if line.startswith('# CONFIG_'):
                line = ' ' + line[9:]
            else:
                line = line[1:]
                sblconfig.write('/*%s */\n' % line)

            empty_line = 0
        else:
            empty_line = 0
            setting = line.split('=')
            if len(setting) >= 2:
                if setting[0].startswith('CONFIG_'):
                    setting[0] = setting[0][7:]

                # remove CONFIG_PKG_XX_PATH or CONFIG_PKG_XX_VER
                if is_pkg_special_config(setting[0]):
                    continue

                if setting[1] == 'y':
                    sblconfig.write('#define %s\n' % setting[0])
                else:
                    sblconfig.write('#define %s %s\n' % (setting[0], re.findall(r"^.*?=(.*)$",line)[0]))

    if os.path.isfile('sblconfig_project.h'):
        sblconfig.write('#include "sblconfig_project.h"\n')

    sblconfig.write('\n')
    sblconfig.write('#endif\n')
    sblconfig.close()

def config():
    mk_sblconfig('.config')

def get_env_dir():
    if os.environ.get('ENV_ROOT'):
        return os.environ.get('ENV_ROOT')

    if sys.platform == 'win32':
        home_dir = os.environ['USERPROFILE']
        env_dir  = os.path.join(home_dir, '.sbl')
    else:
        home_dir = os.environ['HOME']
        env_dir  = os.path.join(home_dir, '.sbl')

    if not os.path.exists(env_dir):
        return None

    return env_dir

def help_info():
    print("**********************************************************************************\n"
          "* Help infomation:\n"
          "**********************************************************************************\n")

def touch_env(SBL_ROOT):
    if sys.platform != 'win32':
        home_dir = os.environ['HOME']
    else:
        home_dir = os.environ['USERPROFILE']

    env_dir  = os.path.join(home_dir, '.sbl')
    if not os.path.exists(env_dir):
        os.mkdir(env_dir)
        os.mkdir(os.path.join(env_dir, 'tools'))
        os.mkdir(os.path.join(env_dir, 'tools', 'scripts'))
        from distutils.dir_util import copy_tree
        copy_tree(os.path.join(SBL_ROOT, 'tools', 'env', 'scripts'), os.path.join(env_dir, 'tools', 'scripts'))

    if sys.platform != 'win32':
        env_sh = open(os.path.join(env_dir, 'env.sh'), 'w')
        env_sh.write('export PATH=~/.sbl/tools/scripts:$PATH')
    else:
        if os.path.exists(os.path.join(env_dir, 'tools', 'scripts')):
            os.environ["PATH"] = os.path.join(env_dir, 'tools', 'scripts') + ';' + os.environ["PATH"]

def menuconfig(SBL_ROOT):
    os_version = platform.platform(True)[11:15]
    if sys.platform != 'win32':
        kconfig_dir = os.path.join(SBL_ROOT, 'tools', 'kconfig-frontends')
        os.system('scons -C ' + kconfig_dir)

    touch_env(SBL_ROOT)
    env_dir = get_env_dir()

    fn = '.config'

    if os.path.isfile(fn):
        mtime = os.path.getmtime(fn)
    else:
        mtime = -1

    if sys.platform != 'win32':
        kconfig_cmd = os.path.join(SBL_ROOT, 'tools', 'kconfig-frontends', 'kconfig-mconf')
        os.system(kconfig_cmd + ' Kconfig')
    else:
        if float(os_version) >= 10.0:
            os.system('kconfig-mconf Kconfig')
        else:
            os.system('kconfig-mconf_win7 Kconfig')

    if os.path.isfile(fn):
        mtime2 = os.path.getmtime(fn)
    else:
        mtime2 = -1

    # make sblconfig.h
    if mtime != mtime2:
        mk_sblconfig(fn)
