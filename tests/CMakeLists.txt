


add_executable (Levenberg-Marquardt Levenberg-Marquardt-eigen3.cpp)
add_executable (Levenberg-Marquardt-fit Levenberg-Marquardt-fit-eigen3.cpp)
add_executable (nonlinear-least-sqares nonlinear-least-sqares-eigen.cpp)
add_executable (Levenberg-Marquardt-fit-sin-eigen3 Levenberg-Marquardt-fit-sin-eigen3.cpp)


target_link_libraries (Levenberg-Marquardt)
target_link_libraries (Levenberg-Marquardt-fit)
target_link_libraries (nonlinear-least-sqares)
target_link_libraries (Levenberg-Marquardt-fit-sin-eigen3)

configure_file(measured_data.txt ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COPYONLY)
configure_file(measured_data_no_noise.txt ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COPYONLY)
