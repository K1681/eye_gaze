from functions_3 import *

def test_raw():
    while True:
        ch = input("Press enter to get few raw samples. e to exit.")
        if ch == "e": break
        for _ in range(10):
            direction = get_direction_uncallibrated()
            print(round(direction[0][0], 4), round(direction[0][1], 4))

def test_callibration():
    while True:
        ch = input("Press enter to get few callibrated samples. e to exit.")
        if ch == "e": break
        for _ in range(1):
            direction = get_direction(1)
            print(round(direction[0][0], 4), round(direction[0][1], 4))

if __name__ == "__main__":
    callibrate()
    test_raw()
    test_callibration()
