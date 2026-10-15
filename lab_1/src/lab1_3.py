from pymycobot.mycobot320 import MyCobot320
import time, math

mc = MyCobot320("deb/ttyAMA0", 115200)

mc.power_on()

for _ in range(1, 10):
    free_angle = 160 * ((-1) ** _)
    angle = (130 - (130 / 10 * _)) * ((-1) ** _)

    mc.send_angles([free_angle, angle, -angle, 0, free_angle, 0], 20)
    mc.set_color(255 * (_ % 2), 255 * (1 ^ (_ % 2)), 32 * (_ % 2))
