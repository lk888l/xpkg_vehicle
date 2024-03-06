# -*- coding: utf-8 -*-
pkg_name = "xpkg_vehicle" # 本包名称  

import sys  # 检查是不是 python3
if sys.version_info.major < 3:
   print("Use python3 to run this file!")
   exit()

class Colors:
   BLACK = "\033[0;30m"
   RED = "\033[0;31m"
   GREEN = "\033[0;32m"
   BROWN = "\033[0;33m"
   BLUE = "\033[0;34m"
   PURPLE = "\033[0;35m"
   CYAN = "\033[0;36m"
   LIGHT_GRAY = "\033[0;37m"
   DARK_GRAY = "\033[1;30m"
   LIGHT_RED = "\033[1;31m"
   LIGHT_GREEN = "\033[1;32m"
   YELLOW = "\033[1;33m"
   LIGHT_BLUE = "\033[1;34m"
   LIGHT_PURPLE = "\033[1;35m"
   LIGHT_CYAN = "\033[1;36m"
   LIGHT_WHITE = "\033[1;37m"
   BOLD = "\033[1m"
   FAINT = "\033[2m"
   ITALIC = "\033[3m"
   UNDERLINE = "\033[4m"
   BLINK = "\033[5m"
   NEGATIVE = "\033[7m"
   CROSSED = "\033[9m"
   END = "\033[0m"

import os
import subprocess


def runCommand(command:list, internal:bool = False, requireConfirm:bool = False)->bool:
   """Run Commands

   Args:
       command (list): Args passed for shell running
       internal (bool, optional): True indicate this is an internal command, if set to true, will directly run command without bothering user. Defaults to False.
         This confict with requireConfirm
       requireConfirm (bool, optional): Require user to input y/n for confirming. Defaults to False.

   Returns:
       bool: If requireConfirm is true, will return True if user decide to keep running script, else false 
   """
   if requireConfirm:
      user_said = False
      user_option = False
      while (not user_said):
         print(Colors.YELLOW + " Confirm running following command, answer y or n:")
         print(command)
         print("answer y or n: " + Colors.END)
         answer = input().lower()
         if answer == "y":
            user_said = True
            user_option = True
         elif answer == "n":
            user_said = True
            print(Colors.RED + "You have refused running the command\n", end="")
            
      if user_option:
         print(Colors.GREEN + "Running ", end="")
         print(command)
         print(Colors.END)
         subprocess.call(command)
      return user_option
   
   if not internal:
      print(Colors.GREEN + "Running ", end="")
      print(command)
      print(Colors.END)
   subprocess.call(command)
   return True


import argparse
parser = argparse.ArgumentParser(
                    prog='Installer for ' + pkg_name,
                    description='Installed needed deps for you to use this pkg',
                    epilog='')

parser.add_argument('--get-pkg-name', action='store_true')        # 此参数代表包应该打印本包名称, 并立刻退出程序
parser.add_argument('--get-install-deps', action='store_true')    # 此参数代表安装依赖, 即应该打印需要在本包之前运行的包名称(如果你的配置一定需要其他包先行配置完成的话)(未启用此功能)
parser.add_argument('--pre-exit', action='store_true')            # 此参数代表工作空间配置已经结束, 如果需要收尾可以在此进行(大多数包不需要此功能, 直接退出即可)
parser.add_argument('--config', action='store_true')              # 此参数代表包应该立刻开始配置包, 例如取出正确的 package.xml, 安装udev等
parser.add_argument('--pre-running', action='store_true')         # 此参数代表工作空间配置即将开始, 如果需要收尾可以在此进行(大多数包不需要此功能, 直接退出即可)

args = parser.parse_args()

# 此参数代表包应该打印本包名称, 并立刻退出程序
if (args.get_pkg_name):
   print(pkg_name)
   exit()

# 此参数代表安装依赖, 即应该打印需要在本包之前运行的包名称(如果你的配置一定需要其他包先行配置完成的话)(未启用此功能)
if (args.get_install_deps):
   print()
   exit()

# 此参数代表工作空间配置已经结束, 如果需要收尾可以在此进行(大多数包不需要此功能, 直接退出即可)
if (args.pre_exit):
   exit()

# 此参数代表工作空间配置即将开始, 如果需要收尾可以在此进行(大多数包不需要此功能, 直接退出即可)
if (args.pre_running):
   exit()

# 此参数代表包应该立刻开始配置包, 例如取出正确的 package.xml, 安装udev等
if (not args.config):
   exit()

# 开始本包的安装配置
# 得到ROS版本
ROS_VERSION = os.getenv('ROS_VERSION')
if (ROS_VERSION is None):
   print(Colors.RED + Colors.BLINK + Colors.BOLD + "Unknown ROS Verison! Please install the ros_environment package and source the /opt/ros/YOUR-ROS-VERSION/setup.bash" + Colors.END)
   print(ROS_VERSION)
   raise RuntimeError
ROS_VERSION = int(ROS_VERSION)
# 得到本python文件的路径
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

'''
这里放ROS1与ROS2都需要的安装步骤(apt装包, udev等)
'''
print(Colors.GREEN + "Install " + pkg_name + Colors.END)
if ROS_VERSION == 1:
   # 开始执行ROS1的安装需要
   pass
elif ROS_VERSION == 2:
   # 开始执行ROS2的安装需要
   pass
else:
   print(Colors.BLINK + Colors.BOLD + Colors.NEGATIVE + Colors.RED + "Unknown ROS Verison! Please install the ros_environment package and source the /opt/ros/YOUR-ROS-VERSION/setup.bash" + Colors.END)
   print(ROS_VERSION)
   raise RuntimeError
print(Colors.GREEN + "Install " + pkg_name + " finish\n" + Colors.END)
