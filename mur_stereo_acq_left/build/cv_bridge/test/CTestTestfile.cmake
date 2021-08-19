# CMake generated Testfile for 
# Source directory: /workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test
# Build directory: /workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_cv_bridge_gtest_cv_bridge-utest "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/gtest-cv_bridge-utest.xml" "--return-code" "/workspace/both_cam_acq/mur_stereo_acq_left/devel/.private/cv_bridge/lib/cv_bridge/cv_bridge-utest --gtest_output=xml:/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/gtest-cv_bridge-utest.xml")
set_tests_properties(_ctest_cv_bridge_gtest_cv_bridge-utest PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;6;catkin_add_gtest;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;0;")
add_test(_ctest_cv_bridge_nosetests_enumerants.py "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/nosetests-enumerants.py.xml" "--return-code" "\"/opt/cmake-3.18.2-Linux-x86_64/bin/cmake\" -E make_directory /workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge" "/usr/bin/nosetests-2.7 -P --process-timeout=60 /workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/enumerants.py --with-xunit --xunit-file=/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/nosetests-enumerants.py.xml")
set_tests_properties(_ctest_cv_bridge_nosetests_enumerants.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;13;catkin_add_nosetests;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;0;")
add_test(_ctest_cv_bridge_nosetests_conversions.py "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/nosetests-conversions.py.xml" "--return-code" "\"/opt/cmake-3.18.2-Linux-x86_64/bin/cmake\" -E make_directory /workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge" "/usr/bin/nosetests-2.7 -P --process-timeout=60 /workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/conversions.py --with-xunit --xunit-file=/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/nosetests-conversions.py.xml")
set_tests_properties(_ctest_cv_bridge_nosetests_conversions.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;14;catkin_add_nosetests;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;0;")
add_test(_ctest_cv_bridge_nosetests_python_bindings.py "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/nosetests-python_bindings.py.xml" "--return-code" "\"/opt/cmake-3.18.2-Linux-x86_64/bin/cmake\" -E make_directory /workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge" "/usr/bin/nosetests-2.7 -P --process-timeout=60 /workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/python_bindings.py --with-xunit --xunit-file=/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/test_results/cv_bridge/nosetests-python_bindings.py.xml")
set_tests_properties(_ctest_cv_bridge_nosetests_python_bindings.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;15;catkin_add_nosetests;/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/test/CMakeLists.txt;0;")
