from user_control import UserControl
import sys

# python3 create_user.py ssh-pubkey

key = sys.argv[1]
uc = UserControl()
user, passw = uc.create_user_pass()
user1, passw1 = uc.create_user_pass()
uc.add_user_core(user, passw1, key)
uc.add_user_spot(user, passw)
uc.estop_pub.publish("press allow")