#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;

	float volume()
	{
		return cube_length * cube_width * cube_height;
	}
};
struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;

	float volume()
	{
		return (x_max - x_min) * (y_max - y_min) * (z_max - z_min);
	}
};
#endif