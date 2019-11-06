from MLX90640 import MLX90640 as MLX90640

tcam = MLX90640.MLX90640()

image, image_modified = tcam.GetVideoStream()