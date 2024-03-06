import sys
if sys.version_info.major < 3:
   print("Use python3 to run this file!")
   exit()

import subprocess

subprocess.call(["rm", ".gitlab-ci.yml"])
subprocess.call(["rm", ".gitignore"])
