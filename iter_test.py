import threading
import time

def test1():
    while True:
        print('test1')
        time.sleep(0.5)

def test2():
    test1_thread = threading.Thread(target=test1)
    test1_thread.setDaemon(True)
    test1_thread.start()
    print('hello')

    while True:
        print('test2')
        time.sleep(0.5)

test2_thread = threading.Thread(target=test2)
test2_thread.setDaemon(True)
test2_thread.start()