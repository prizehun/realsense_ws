#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV
#include <iostream>


int main(int argc, char * argv[]) try
{
	using namespace cv;
	using namespace rs2;
	

	// Define colorizer and align processing-blocks
	colorizer colorize;
	align align_to(RS2_STREAM_COLOR);

	// Start the camera
	pipeline pipe;
	pipe.start();

	// Skips some frames to allow for auto-exposure stabilization
	for (int i = 0; i < 10; i++) pipe.wait_for_frames();

	while (1){

		frameset data = pipe.wait_for_frames(); // data read
		frameset aligned_set = align_to.process(data);
		frame depth = aligned_set.get_depth_frame();

		auto color_mat = frame_to_mat(aligned_set.get_color_frame());
		colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);
				
		frame bw_depth = depth.apply_filter(colorize);
		auto depth_mat = frame_to_mat(bw_depth);
		auto depth_meter = depth_frame_to_meters(depth);

		resize(color_mat, color_mat, Size(640, 360));
		resize(depth_mat, depth_mat, Size(640, 360));
		resize(depth_meter, depth_meter, Size(640, 360));

		imshow("color", color_mat);
		imshow("depth", depth_mat);
		imshow("depth_meter", depth_meter);

		if (waitKey(5) == 'q')
			break;
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
