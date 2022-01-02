from pynput.mouse import Button, Controller
import tkinter as tk

DIST_BETWEEN_WHEELS = 20
ROBOT_MOMENT_INERTIA = 100

HALF_DIST_BETWEEN_WHEELS = DIST_BETWEEN_WHEELS*0.5

root = tk.Tk()
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
mouse = Controller()

while(True):
    x = 2 * ((mouse.position[0]/screen_width) - 0.5)
    y = -2 * ((mouse.position[1]/screen_height) - 0.5)


    f1 = y-x
    f2 = y+x

    try:
        
        turningCenter = (HALF_DIST_BETWEEN_WHEELS)*((f1+f2)/(f1-f2))
        
        torqueSum = f1 * (turningCenter + HALF_DIST_BETWEEN_WHEELS) + f2 * (turningCenter - HALF_DIST_BETWEEN_WHEELS)

        angAccel = torqueSum / ROBOT_MOMENT_INERTIA

        print(f1, f2, angAccel)

    except KeyboardInterrupt:
        break
    except Exception as e:
        print(e)