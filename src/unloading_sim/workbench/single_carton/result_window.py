"""Independent, resizable, hide-only result displays on the Tk main thread."""
from collections import deque
import time
import tkinter as tk
from tkinter import ttk
from PIL import Image,ImageTk
from ..stable_ui import StableStringVar,fixed_text


class ResultWindow:
    def __init__(self,root,stage):
        self.stage=stage;self.window=tk.Toplevel(root)
        self.window.title('检测结果' if stage=='detection' else '分割结果')
        self.window.geometry('700x590');self.window.minsize(420,320)
        self.window.protocol('WM_DELETE_WINDOW',self.window.withdraw)
        self.status=StableStringVar(value='等待相机和模型')
        fixed_text(self.window,self.status,height=3)
        self.canvas=tk.Canvas(self.window,width=1,height=1,bg='#101924',highlightthickness=0)
        self.canvas.pack(fill='both',expand=True)
        self.item=self.canvas.create_image(0,0,anchor='center')
        self.canvas.bind('<Configure>',lambda _:self.draw())
        self.packet=None;self.photo=None;self.shown=None;self.refresh_times=deque(maxlen=20)
    def open(self):self.window.deiconify();self.window.lift();self.draw()
    def update(self,packet,status,error):
        changed=packet is not self.packet;self.packet=packet
        if changed:self.draw()
        if packet is None:
            self.status.set(f'{status}\n{error or "等待本会话首条新结果"}');return
        f=packet.frame;now=time.monotonic()
        times=[x for x in self.refresh_times if now-x<10]
        fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.
        self.status.set(f'RGB #{f.rgb_number} / Depth #{f.depth_number} · 结果年龄 {f.stamp.age_s():7.2f} s · 刷新 {fps:5.2f} Hz\n'
                        f'{status} · {len(packet.result.detections)} 个目标 · {f.observation_id}\n{error}')
    def draw(self):
        if not self.window.winfo_viewable():return
        if self.packet is None:
            self.canvas.itemconfigure(self.item,image='');self.photo=None;self.shown=None;return
        width=max(1,self.canvas.winfo_width());height=max(1,self.canvas.winfo_height())
        im=Image.fromarray(self.packet.pixels);im.thumbnail((width,height))
        self.photo=ImageTk.PhotoImage(im);self.canvas.itemconfigure(self.item,image=self.photo)
        self.canvas.coords(self.item,width/2,height/2)
        if self.shown is not self.packet:
            self.refresh_times.append(time.monotonic());self.shown=self.packet
