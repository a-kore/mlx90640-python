# mlx90640-python
A python implementation of the MLX90640 Thermal Camera.

It utilizes most of the same functions as the pimoroni/mlx90640-library re-written for Python 3.X

## How to use

import the library and create an instance of the class:
```
from mlx90640 import MLX90640 as MLX90640
tcam = MLX90640.MLX90640()
```


GetFrameData() reads a frame, converts it to object temperatures and returns each subpage as an array:
GetVideoStream() produces a video stream of the incoming frames, returns an average of the subpages and an 8 bit heatmap with post-processing(resizing,blurring).
```
frame = tcam.GetFrameData()
image, image_modified = tcam.GetVideoStream()
```
