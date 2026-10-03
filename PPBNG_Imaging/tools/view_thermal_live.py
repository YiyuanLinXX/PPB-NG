"""Read-only latest-saved A6701 frame viewer. No SDK, ROS, camera or serial control."""
import argparse
from pathlib import Path
import struct
import time
import numpy as np
from PIL import Image, ImageTk
from export_snapshot_images import crc32, thermal_false_color

PAYLOAD = 640 * 513 * 2
RECORD = PAYLOAD + 96

def latest_frame(dataset):
    for path in sorted((dataset/'segments').glob('thermal_*.ppbseg'), reverse=True):
        with path.open('rb') as stream:
            header=stream.read(16)
            if len(header)<16: continue
            if header[:8]!=b'PPBNGSG1' or struct.unpack_from('<I',header,12)[0]!=16:
                raise ValueError('Unsupported segment header')
            count=max(0,(path.stat().st_size-16)//RECORD)
            if not count: continue
            stream.seek(16+(count-1)*RECORD)
            h=stream.read(72); pixels=stream.read(PAYLOAD); trailer=stream.read(24)
            if len(trailer)!=24: continue  # Writer has not committed the live tail yet.
            if h[:4]!=b'FRM1' or struct.unpack_from('<Q',h,48)[0]!=PAYLOAD or struct.unpack_from('<Q',h,60)[0]!=RECORD:
                raise ValueError('Expected fixed 640x513 Mono16 A6701 transport')
            if crc32(h[:68])!=struct.unpack_from('<I',h,68)[0]: raise ValueError('Header CRC failed')
            if trailer[:4]!=b'CMIT': continue
            checksum=struct.unpack_from('<I',h,56)[0]
            if (struct.unpack_from('<Q',trailer,8)[0]!=RECORD or
                struct.unpack_from('<I',trailer,16)[0]!=checksum or
                crc32(trailer[:20])!=struct.unpack_from('<I',trailer,20)[0] or crc32(pixels)!=checksum):
                raise ValueError('Frame CRC/commit validation failed')
            sample=struct.unpack_from('<Q',h,8)[0]
            return (path.name,sample),np.frombuffer(pixels,dtype='<u2').reshape(513,640)[1:,:].copy()
    return None

def main():
    import tkinter as tk
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path,help='Exact active or completed dataset directory')
    args=parser.parse_args()
    if not (args.dataset/'segments').is_dir(): parser.error('Dataset segments directory not found')
    root=tk.Tk(); root.title('A6701 focus - latest SAVED frame (not temperature)')
    label=tk.Label(root); label.pack()
    status=tk.Label(root,text='Waiting for a committed thermal frame...',wraplength=720); status.pack()
    last=None; changed=time.monotonic()
    def refresh():
        nonlocal last,changed
        try:
            result=latest_frame(args.dataset)
            if result:
                key,frame=result
                if key!=last:
                    last=key; changed=time.monotonic()
                    photo=ImageTk.PhotoImage(Image.fromarray(thermal_false_color(frame)))
                    label.configure(image=photo); label.image=photo
                age=time.monotonic()-changed
                status.configure(text=f'{key[0]} | sample {key[1]} | no new saved frame for {age:.1f}s\n'
                    '1-99% counts stretch; NOT temperature. Closing this viewer does NOT stop recording.',
                    fg='red' if age>3 else 'black')
            else: status.configure(text='Waiting for first committed frame; recording must be started separately.')
        except (OSError,ValueError) as error:
            status.configure(text=f'VIEWER WARNING: {error}',fg='red')
        root.after(500,refresh)
    refresh(); root.mainloop()

if __name__=='__main__': main()
