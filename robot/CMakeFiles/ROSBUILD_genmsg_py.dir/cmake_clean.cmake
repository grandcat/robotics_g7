FILE(REMOVE_RECURSE
  "msg_gen"
  "src/robot/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/robot/msg/__init__.py"
  "src/robot/msg/_Rotate.py"
  "src/robot/msg/_EKF.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
