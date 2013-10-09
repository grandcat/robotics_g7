FILE(REMOVE_RECURSE
  "msg_gen"
  "src/camera/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/camera/msg/__init__.py"
  "src/camera/msg/_Position.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
