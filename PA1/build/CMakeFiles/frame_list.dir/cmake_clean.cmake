file(REMOVE_RECURSE
  "libframe_list.a"
  "libframe_list.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/frame_list.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
