import time
import os


os.system("echo servod 0-100% > /dev/servoblaster")
time.sleep(2)
print "finished"
