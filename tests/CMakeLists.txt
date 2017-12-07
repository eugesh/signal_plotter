


add_executable (Levenberg-Marquardt Levenberg-Marquardt-eigen3.cpp)
add_executable (nonlinear-least-sqares nonlinear-least-sqares-eigen.cpp)


target_link_libraries (Levenberg-Marquardt)
target_link_libraries (nonlinear-least-sqares)
