"""Stable Tk surfaces: text cannot negotiate geometry on a data refresh."""
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText


class StableStringVar(tk.StringVar):
    def set(self, value):
        value = str(value)
        if self.get() != value:
            super().set(value)


def fixed_text(parent, variable, height=3, **pack):
    text = ScrolledText(parent, height=height, width=1, wrap='word',
                        bg='#152435', fg='#e2edf5', state='disabled',
                        font=('DejaVu Sans Mono', 10), relief='flat')
    text.pack(fill='x', **pack)
    def update(*_):
        value = variable.get()
        if getattr(text, '_content', None) == value:
            return
        text._content = value
        pos = text.yview()[0]
        text.configure(state='normal')
        text.delete('1.0', 'end'); text.insert('end', value)
        text.configure(state='disabled'); text.yview_moveto(pos)
    variable.trace_add('write', update)
    update()
    return text


def constrain_labels(parent):
    """A variable label is clipped inside its allocated slot, never autosized.

    Long diagnostic text belongs in fixed_text; these short slots retain the
    caller's row count. Image labels also have stable requested dimensions.
    """
    for child in parent.winfo_children():
        if isinstance(child, ttk.Label) and child.cget('textvariable') and not hasattr(child,'_stable_slot'):
            # ttk.Label has no height option. A nonpropagating parent also
            # prevents embedded newlines from increasing the row height.
            info=child.pack_info() if child.winfo_manager()=='pack' else None
            if info:
                siblings=parent.pack_slaves();index=siblings.index(child)
                following=siblings[index+1] if index+1<len(siblings) else None
                info.pop('in',None)
                slot=ttk.Frame(parent,width=280 if info.get('side')=='right' else 1,height=24)
                if following:info['before']=following
                child.pack_forget();slot.pack(**info);slot.pack_propagate(False)
                child.configure(width=1,anchor='w')
                child.pack(in_=slot,fill='both',expand=True);child.lift();child._stable_slot=slot
        constrain_labels(child)


def append_log(widget, rows, previous):
    rows = tuple(rows)
    if rows == previous:
        return rows
    # The backend is a bounded deque. Find its retained suffix, including
    # identical adjacent lines, and append only genuinely new records.
    overlap = min(len(previous), len(rows))
    while overlap and previous[-overlap:] != rows[:overlap]:
        overlap -= 1
    at_bottom = widget.yview()[1] >= .999
    widget.mark_set('_log_anchor', widget.index('@0,0'))
    widget.configure(state='normal')
    removed = len(previous)-overlap
    if removed:
        widget.delete('1.0', f'{removed+1}.0')
    for row in rows[overlap:]:
        widget.insert('end', row+'\n')
    widget.configure(state='disabled')
    if at_bottom:
        widget.see('end')
    else:
        widget.yview('_log_anchor')
    return rows
