from user_control import UserControl
import sys
import time

# python3 delete_user.py username

user = sys.argv[1]
uc = UserControl()
with open('/etc/passwd', 'r') as f:
    for line in f:
        line = line.split(':')
        if line[0] == user:
            info = line[4].split('/')
            mail = info[0]
uc.delete_user_core(user)
uc.delete_user_spot(user)
uc.estop_pub.publish("press stop")
time.sleep(5)
ipfs_hash = uc.pin_to_ipfs(f"/home/spot/{user}/")