## Point
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 2.0, y: 1.0, z: 0.0}]]"
```
## Line
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 1.0, y: 1.0, z: 0.0}, {x: 2.0, y: 1.0, z: 0.0}]]"
```
## Circle
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 1.0, y: 1.0, z: 1.0}]]"
```
## Polygon
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: 1.0, y: 1.0, z: 0.0}, {x: 2.0, y: 1.0, z: 0.0}, {x: 1.0, y: 2.0, z: 0.0}, {x: 2.0, y: 2.0, z: 0.0}]]"
```
## Multiple
```
rostopic pub /virtual_costamp_layer/obsctacles custom_msgs/Obstacles "list: [form: [{x: -1.0, y: 1.0, z: 1.0}], form: [{x: 1.0, y: 1.0, z: 0.0}, {x: 2.0, y: 1.0, z: 0.0}, {x: 1.0, y: 2.0, z: 0.0}, {x: 2.0, y: 2.0, z: 0.0}]]"
```
