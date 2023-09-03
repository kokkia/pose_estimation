import threading
import concurrent.futures
import time

cnt = 0
# while True:
#     print("a")

def func(lock):
    global cnt
    while True:
        print("lock")
        try:
            with lock:
                cnt += 1
            print("func1", cnt)
        except Exception as e:
            print(e)
        time.sleep(0.001)

def func2(lock):
    global cnt
    while True:
        print("func2",cnt)
        time.sleep(0.001)

if __name__ == "__main__":
    executor = concurrent.futures.ThreadPoolExecutor(2) # 複数のスレッドを立ち上げる

    lock = threading.Lock()  # threading.Lockオブジェクトのインスタンスを1つ生成する

    # 複数スレッドで同時に同じ処理を行う
    executor.submit(func, lock)
    executor.submit(func2, lock)