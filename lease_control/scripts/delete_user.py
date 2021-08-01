from user_control import UserControl
import sys

# python3 delete_user.py username

user = sys.argv[1]
uc = UserControl()
uc.delete_user_core(user)
uc.delete_user_spot(user)
uc.estop_pub.publish("press stop")