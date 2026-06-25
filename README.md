# Group 11 assignment 1
## Authors
Erba Lorenzo\
Fusari Nicolò\
Piccoli Leonardo Arduino

## Content
* src/
  - april_detection_node.cpp
  - table_detection_node.cpp
  - brain_node.cpp
* srv/
  - FindApril.srv
  - FindTables.srv
* launch/
  - assignment.launch.py
  - camera.launch.py
- CMakeLists.txt
- package.xml
- report1.pdf

### Launch package
After building and sourcing the package, it can be launched with:\
`ros2 launch group11_assignament1 assignment.launch.py` \
or, to have in terminal only the output log of the nodes in this package:\
`ros2 launch group11_assignament_1 assignment.launch.py 2>&1 | grep -E "brain_node|april_detection|table_detection"`
