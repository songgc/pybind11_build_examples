import cppimport
import numpy as np
import time
import os
import psutil

code = cppimport.imp("code")


def memory_percent():
    pid = os.getpid()
    p = psutil.Process(pid)
    return p.memory_percent()


if __name__ == '__main__':
    xs = np.arange(12).reshape(3,4).astype('float')
    print(xs)
    print("np :", xs.sum())
    # print("cpp:", code.sum(xs))

    print()
    code.twice(xs)
    print(xs)
    for _ in range(10):
        print("cpp:", memory_percent(),  code.sum(xs), memory_percent())
        time.sleep(5)

