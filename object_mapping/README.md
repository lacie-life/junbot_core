# Object Mapping

```
git clone -b v6.0 https://github.com/ultralytics/yolov5.git
# create conda envs and install requierments.txt for running gen_wts.py
# stupid scripts

# convert .pt to .wts using gen_wts.py file
python gen_wts.py -w yolov5s.pt
# a file 'yolov5s.wts' will be generated.

./object_mapping -s [.wts] [.engine] [n/s/m/l/x/n6/s6/m6/l6/x6 or c/c6 gd gw]  // serialize model to plan file

# For example yolov5s
./object_mapping -s yolov5s.wts yolov5s.engine s
# a file 'yolov5s.engine' will be generated.

# For example Custom model with depth_multiple=0.17, width_multiple=0.25 in yolov5.yaml
./object_mapping -s yolov5_custom.wts yolov5_custom.engine c 0.17 0.25
# a file 'yolov5_custom.engine' will be generated.

./object_mapping -d [.engine] [zed camera id / optional svo filepath]  // deserialize and run inference

# For example yolov5s
./object_mapping -d yolov5s.engine 0      # 0  for zed camera id 0

# With an SVO file
./object_mapping -d yolov5.engine ./foo.svo
```

# TODO

- [x] Add ObjectDatabase
- [x] Convert Object to Map
- [x] Publish Polygon
- [ ] Test with ZED 
