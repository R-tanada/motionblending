from subprocess import Popen, PIPE, STDOUT
import signal
import tkinter as tk

cmd = ('python ExManager.py').split()

def killProcess():
    if 'proc' in globals():
        proc.terminate()
        proc.wait()
        output, err = proc.communicate()
        output = output.decode('shift_jis')
        print(output)
    root.destroy()

def handler(signum, root):
    print("handler")
    killProcess()

def ctrlC(event):
    print("ctrlC")
    killProcess()

def stop():
    print("stop")
    killProcess()

def start():
    global proc
    print("start")
    proc = Popen(cmd, stdout=PIPE, stderr=STDOUT)

def check(): 
    root.after(500, check)  #  time in ms.
#
signal.signal(signal.SIGINT, handler) # ctrl-c from console

root = tk.Tk()
root.after(500, check)  #  time in ms.

tk.Button(root, text="Start subprocess", command=start).pack()
tk.Button(root, text="Stop subprocess", command=stop).pack()
root.bind('<Control-c>', ctrlC) 
root.protocol("WM_DELETE_WINDOW", stop)

root.mainloop()
print("exited")
