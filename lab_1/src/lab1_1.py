from pymycobot.mycobot320 import MyCobot320
import time

mc = MyCobot320("dev/ttyAMA0", "115200")

mc.power_on()

mc.send_angles([0, 0, 0, 0, 0, 0], 30)
time.sleep(4)

mc.send_angles([30, 30, 30, 30, 30, 30], 20)

print(mc.get_coords())