FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/fmActuators/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/fmActuators/srv/__init__.py"
  "../src/fmActuators/srv/_roboteq.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
