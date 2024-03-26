file(REMOVE_RECURSE
  "librelative_pose.a"
  "librelative_pose.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/relative_pose.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
