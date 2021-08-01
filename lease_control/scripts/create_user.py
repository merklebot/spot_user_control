from user_control import UserControl
import sys

# python3 create_user.py email ssh-pubkey

email = sys.argv[1]
key = sys.argv[2]
uc = UserControl()
user, passw = uc.create_user_pass()
user1, passw1 = uc.create_user_pass()
uc.add_user_core(user, passw1, email, key)
uc.add_user_spot(user, passw)
uc.estop_pub.publish("press allow")