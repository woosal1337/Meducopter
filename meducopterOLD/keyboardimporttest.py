import keyboard
import urllib
import urllib.request

while True:  # making a loop # used try so that if user pressed other than the given key error will not be shown
            if keyboard.is_pressed('a'):  # if key 'q' is pressed
                print('left')
            elif keyboard.is_pressed('s'):  # if key 'a' is pressed
                print('backward')
            elif keyboard.is_pressed('d'):  # if key 'a' is pressed
                print('right')
            elif keyboard.is_pressed('w'):  # if key 'a' is pressed
                print('forward')
            elif keyboard.is_pressed('space'):  # if key 'a' is pressed
                print('go down')
            elif keyboard.is_pressed('v'):  # if key 'a' is pressed
                print('go up')
            else:
                print('stabilize')