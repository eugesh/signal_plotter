

add_executable (Levenberg-Marquardt Levenberg-Marquardt-eigen3.cpp)
add_executable (Levenberg-Marquardt-fit Levenberg-Marquardt-fit-eigen3.cpp)
add_executable (nonlinear-least-sqares nonlinear-least-sqares-eigen.cpp)
add_executable (Levenberg-Marquardt-fit-sin-eigen3 Levenberg-Marquardt-fit-sin-eigen3.cpp)
add_executable (test_median1d test_median1d.cpp)
add_executable (lsm_test lsm_test.cpp)
add_executable (test_Gauss test_Gauss.cpp)
add_executable (test_fft_eigen fft_eigen_test.cpp)


target_link_libraries (test_median1d)
target_link_libraries (lsm_test)
target_link_libraries (test_Gauss)

if(CMAKE_VERSION VERSION_LESS "3.0")
	target_link_libraries (Levenberg-Marquardt)# Eigen3::Eigen)
	target_link_libraries (Levenberg-Marquardt-fit)# Eigen3::Eigen)
	target_link_libraries (nonlinear-least-sqares)# Eigen3::Eigen)
	target_link_libraries (Levenberg-Marquardt-fit-sin-eigen3)# Eigen3::Eigen)
	target_link_libraries (test_fft_eigen -lfftw3f)# Eigen3::Eigen)
else()
	message("USE Eigen3::Eigen")
	target_link_libraries (Levenberg-Marquardt Eigen3::Eigen)
	target_link_libraries (Levenberg-Marquardt-fit Eigen3::Eigen)
	target_link_libraries (nonlinear-least-sqares Eigen3::Eigen)
	target_link_libraries (Levenberg-Marquardt-fit-sin-eigen3 Eigen3::Eigen)
	target_link_libraries (test_fft_eigen -lfftw3f Eigen3::Eigen)    
endif()


configure_file(measured_data.txt ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COPYONLY)
configure_file(measured_data_no_noise.txt ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COPYONLY)
