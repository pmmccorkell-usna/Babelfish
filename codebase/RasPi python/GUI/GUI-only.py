from tkinter import *
import time
import threading

def global_values():
        global heading
        heading=111
        global roll
        roll=122
        global pitch
        pitch=133
        global depth
        depth=200
        global port_pw
        port_pw=1.1
        global starboard_pw
        starboard_pw=1.2
        global fore_pw
        fore_pw=1.6
        global aft_pw
        aft_pw=1.7
        global status
        status=0xffff0000

        global quit_window
        quit_window=threading.Event()


class Application(Frame):

    def __init__(self, master):
        Frame.__init__(self, master)
        self.pack()
        self.widgets()
        self.update_values()
        
    def widgets(self):
        
        self.heading_gui_data=IntVar()
        self.roll_gui_data=IntVar()
        self.pitch_gui_data=IntVar()
        self.depth_gui_data=IntVar()
        self.port_gui_data=IntVar()
        self.star_gui_data=IntVar()
        self.fore_gui_data=IntVar()
        self.aft_gui_data=IntVar()
        
        self.heading_text = Label(self,text="Heading:")
        self.heading_text.grid(column=1,row=1)
        self.heading_gui = Label(self,textvariable=self.heading_gui_data)
        self.heading_gui.grid(column=2,row=1)

        self.roll_text = Label(self,text="Roll:")
        self.roll_text.grid(column=1,row=2)
        self.roll_gui = Label(self,textvariable=self.roll_gui_data)
        self.roll_gui.grid(column=2,row=2)

        self.pitch_text = Label(self,text="Pitch:")
        self.pitch_text.grid(column=1,row=3)
        self.pitch_gui = Label(self,textvariable=self.pitch_gui_data)
        self.pitch_gui.grid(column=2,row=3)

        self.depth_text = Label(self,text="Depth:")
        self.depth_text.grid(column=1,row=4)
        self.depth_gui = Label(self,textvariable=self.depth_gui_data)
        self.depth_gui.grid(column=2,row=4)

        self.port_pw_text=Label(self,text="Port pw:")
        self.port_pw_text.grid(column=1,row=5)
        self.port_gui = Label(self,textvariable=self.port_gui_data)
        self.port_gui.grid(column=2,row=5)

        self.star_pw_text=Label(self,text="Stbd pw:")
        self.star_pw_text.grid(column=1,row=6)
        self.star_gui=Label(self,textvariable=self.star_gui_data)
        self.star_gui.grid(column=2,row=6)

        self.fore_pw_text=Label(self,text="Fore pw:")
        self.fore_pw_text.grid(column=1,row=7)
        self.fore_gui=Label(self,textvariable=self.fore_gui_data)
        self.fore_gui.grid(column=2,row=7)

        self.aft_pw_text=Label(self,text="Aft pw:")
        self.aft_pw_text.grid(column=1,row=8)
        self.aft_gui=Label(self,textvariable=self.aft_gui_data)
        self.aft_gui.grid(column=2,row=8)

        self.heading_field_text=Label(self,text="Enter heading:")
        self.heading_field_text.grid(column=3,row=1)
        self.heading_field=Entry(self)
        self.heading_field.grid(column=4,row=1)
        self.heading_field_button = Button(self,text="Go to heading", command=lambda: check_gui_heading(heading_field.get()))
        self.heading_field_button.grid(column=5,row=1)
        self.heading_field_stop = Button(self,text="Stop heading", command=lambda: stop_persistent_h.set())
        self.heading_field_stop.grid(column=6,row=1)

        self.depth_field_text=Label(self,text="Enter depth:")
        self.depth_field_text.grid(column=3,row=4)
        self.depth_field=Entry(self)
        self.depth_field.grid(column=4,row=4)
        self.depth_set_indicator=Label(self,text="Depth setup",bg="white")
        self.depth_set_indicator.grid(column=5,row=3)
        self.depth_field_button = Button(self,text="Go to depth",command=lambda: check_gui_depth(depth_field.get()))
        self.depth_field_button.grid(column=5,row=4)
        self.depth_zero_button = Button(self,text="Depth zero",command=lambda: depth_zero())
        self.depth_zero_button.grid(column=5,row=5)
        self.depth_set_button = Button(self,text="Depth set",command=lambda: depth_set())
        self.depth_set_button.grid(column=5,row=6)

        self.Horizon_button = Button(self,text="EMERGENCY STOP",command=lambda: horizon())
        self.Horizon_button.grid(column=2,row=0)

        self.command_indicator = Label(self,text="Command Function",bg="white")
        self.command_indicator.grid(row=0,column=6)
        self.call_indicator = Label(self,text="Call Function",bg="white")
        self.call_indicator.grid(row=0,column=7)

        self.Test1_button = Button(self,text="Test fwd/back",command=lambda: direction_command(1))
        self.Test1_button.grid(column=8,row=1)
        self.Test2_button = Button(self,text="Test left/right",command=lambda: direction_command(2))
        self.Test2_button.grid(column=8,row=2)
        self.Test3_button = Button(self,text="Test up/down",command=lambda: direction_command(3))
        self.Test3_button.grid(column=8,row=3)
        self.Test4_button = Button(self,text="Test pitch",command=lambda: direction_command(4))
        self.Test4_button.grid(column=8,row=4)

        self.Quit_button=Button(self,text="Leave GUI",command=lambda: self.quit_method())
        #self.Quit_button["command"] =  self.quit
        self.Quit_button.grid(column=8,row=0)
        
        self.reset_button=Button(self,text="Reset Mbed",command=lambda: reset_mbed())
        self.reset_button.grid(column=0,row=0)

    def quit_method(self):
        quit_window.set()
        self.quit
   
    def heading(self,h):
        self.heading_gui_data.set(h)
    def roll(self,r):
        self.roll_gui_data.set(r)
    def pitch(self,p):
        self.pitch_gui_data.set(p)
    def depth(self,d):
        self.depth_gui_data.set(d)
    def port(self,pw):
        self.port_gui_data.set(pw)
    def star(self,pw):
        self.star_gui_data.set(pw)
    def fore(self,pw):
        self.fore_gui_data.set(pw)
    def aft(self,pw):
        self.aft_gui_data.set(pw)
    def change_green(self,word):
        if word=='depth':
            self.depth_set_indicator.config(bg="green2")
        if word=='command':
            self.command_indicator.config(bg="green2")
        if word=='call':
            self.call_indicator.config(bg="green2")
    def change_white(self,word):
        if word=='depth':
            self.depth_set_indicator.config(bg="white")
        if word=='command':
            self.command_indicator.config(bg="white")
        if word=='call':
            self.call_indicator.config(bg="white")
    def update_values(self):
        self.heading(heading)
        self.roll(roll)
        self.pitch(pitch)
        self.depth(depth)
        self.port(port_pw)
        self.star(starboard_pw)
        self.fore(fore_pw)
        self.aft(aft_pw)
        if ((status&0x0008)==0x0008):
            self.change_green('depth')
        else:
            self.change_white('depth')
        if ((status&0x0100)==0x0100):
            self.change_green('command')
        else:
            self.change_white('command')
        if ((status&0x0200)==0x0200):
            self.change_green('call')
        else:
            self.change_white('call')       
        self.update()
        self.update_idletasks()

def windowsetup():
        global_values()
        root = Tk()
        root.title("UAV Controls")
        root.geometry('1100x400')
        app = Application(master=root)
        #app.mainloop()
        i=0
        while not quit_window.is_set():
        #while(1):
        #print('while loop')
                i+=1
                global heading
                heading+=1
                app.update_values()
                time.sleep(0.1)
        root.destroy()

def main():
        windowsetup()
        
main()

