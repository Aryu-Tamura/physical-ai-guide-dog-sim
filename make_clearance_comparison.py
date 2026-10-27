from pathlib import Path
import imageio.v2 as imageio
import numpy as np
from PIL import Image,ImageDraw

out=Path(__file__).resolve().parent/'outputs'
paths=[out/'clearance_demo_robot_only_seed0.mp4',out/'clearance_demo_person_aware_seed0.mp4']
readers=[imageio.get_reader(str(p)) for p in paths]
counts=[r.count_frames() for r in readers]
last=[None,None]
try:
    with imageio.get_writer(str(out/'clearance_comparison.mp4'),fps=25) as writer:
        for i in range(max(counts)):
            frames=[]
            for k,r in enumerate(readers):
                if i<counts[k]:last[k]=r.get_data(i)
                frame=last[k].copy()
                if i>=counts[k]:
                    im=Image.fromarray(frame);draw=ImageDraw.Draw(im)
                    draw.rectangle((0,447,640,480),fill=(20,25,30))
                    draw.text((12,458),'Playback held: test stopped at detected person collision',fill='white')
                    frame=np.asarray(im)
                frames.append(frame)
            combined=np.concatenate(frames,axis=1)
            writer.append_data(combined)
            if i==400:imageio.imwrite(out/'clearance_comparison.png',combined)
finally:
    for r in readers:r.close()
print('Comparison saved; frames:',max(counts),'fps:25. Left stops after collision; held frame explicitly labelled.')
