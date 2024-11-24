# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 11:20:15 2024

@author: Houch
"""

import pickle
import numpy as np
import tqdm
import matplotlib.pyplot as plt
demo_dir = r'D:/output/pen/data_1.pkl'
with open(demo_dir, 'rb') as f:
        demo = pickle.load(f)
demo_length = len(demo['image'])
image = []
for step_idx in tqdm.tqdm(range(demo_length)):
    obs_point_cloud = demo["image"][step_idx]
    image.append(obs_point_cloud)


#import visualizer
#visualizer.visualize_pointcloud(image[10])
#import matplotlib.pyplot as plt
for i in range(len(image)):
    plt.imshow(image[i])
    plt.show()
    
 
""" 
import os
import imageio
from PIL import Image
import numpy as np

filename = 'output.mp4'
filepath = os.path.join(r'D:/output/picture_2', filename)

images = []
#for file_name in sorted(os.listdir(r'D:/output/picture_1')):
for i in range(20):
    file_name = "Figure 2024-05-07 132856(" +str(i) +").png"
    if file_name.endswith('.png'):
        print(file_name)
        images.append(Image.open(r'D:/output/picture_2/'+ file_name))

fps = 2
with imageio.get_writer(filepath, fps=fps) as video:
    for image in images:
        frame = image.convert('RGB')
        #print(frame)
        frame = np.array(frame)
        video.append_data(frame)
"""