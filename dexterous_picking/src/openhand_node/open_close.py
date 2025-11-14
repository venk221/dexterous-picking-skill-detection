# sudo chmod 666 /dev/ttyUSB0

from hands import *

T=Model_O('/dev/ttyUSB0', 3, 4, 1, 2, "XM")

# T.open()
# time.sleep(0.2)
# T.pinch_close()
# # time.sleep(0.3)
# # T.pinch_object_move()
# time.sleep(3)
# T.open()
# time.sleep(0.4)


# Motor 1 - 0.5, Motor 2 - 0.3, Motor 3 - 0.45
# 1. Simple pick
# 2. Push-to-horizontal
# 3. Slide-to-edge
# 4. Push-to-vertical
# 5. Flip
print("Enter an action to perform (1-5):")
action = int(input())

if action == 1:
    T.open()
    time.sleep(0.3)
    T.adduct(1)
    time.sleep(2)
    T.power_close(0.5)
    time.sleep(5) #hold object time
    T.adduct(0)
    T.open()
    time.sleep(1.5)
elif action == 2:
    T.open()
    time.sleep(0.5)
    T.adduct(1)
    time.sleep(2)
    T.moveMotor(3, 0.3)
    T.moveMotor(1, 0.3)
    T.moveMotor(2, 0.4)
    time.sleep(10) #hold object time
    T.power_close(0.5)
    time.sleep(5) #hold object time
    T.open()
    T.adduct(0)
    time.sleep(1.5)
elif action == 3:
    T.open()
    time.sleep(0.3)
    T.adduct(1)
    time.sleep(3)
    T.moveMotor(1, 0.3)
    T.moveMotor(2, 0.4)
    time.sleep(5) #hold object time
    T.power_close(0.5)
    time.sleep(5) #hold object time
    T.open()
    T.adduct(0)
    time.sleep(1.5)
elif action == 4:
    T.open()
    time.sleep(0.3)
    T.adduct(1)
    time.sleep(3)
    T.moveMotor(1, 0.55)
    T.moveMotor(2, 0.55)
    time.sleep(5) #hold object time
    T.power_close()
    time.sleep(5) #hold object time
    T.open()
    T.adduct(0)
    time.sleep(1.5)
elif action == 5:
    T.open()
    time.sleep(0.3)
    T.adduct(1)
    time.sleep(3)
    T.moveMotor(1, 0.45)
    T.moveMotor(2, 0.55)
    T.moveMotor(3, 0.55)
    time.sleep(5) #hold object time
    T.moveMotor(3, 0.7)
    time.sleep(5) #hold object time
    T.open()
    T.adduct(0)
    time.sleep(1.5)
else:
    print("Invalid action")
