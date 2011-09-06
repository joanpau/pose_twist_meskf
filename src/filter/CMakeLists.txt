rosbuild_add_library(pose_twist_meskf 
                       pose_twist_meskf.cpp
                       extendedkalmanfilter_resetcapable.cpp
                       analyticconditionalgaussian_posetwisterrorstate.cpp
                       nominal_state_vector.cpp
                       error_state_vector.cpp
                       input_vector.cpp)
