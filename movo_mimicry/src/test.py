import multiprocessing
import time
import rospy

# rospy.init_node("test")

def infinite_loop():
    while True:
        # print("True")
        pass

# while True:
#     infinite_loop()
while True:
    p = multiprocessing.Process(target=infinite_loop)
    p.start()
    print(p.is_alive())
    p.terminate()
    time.sleep(1)
    print("After" + str(p.is_alive()))
