#include "cvKeypoint.h"

sofacv::cvKeypoint::cvKeypoint(const cv::KeyPoint &kp)
    : cv::KeyPoint(kp.pt, kp.size, kp.angle, kp.response, kp.octave,
                   kp.class_id)
{
}

sofacv::cvKeypoint::cvKeypoint(cv::Point2f _pt, float _size, float _angle, float _response, int _octave, int _class_id)
    : cv::KeyPoint(_pt, _size, _angle, _response, _octave, _class_id)
{
}

sofacv::cvKeypoint::cvKeypoint(float x, float y, float _size, float _angle, float _response, int _octave, int _class_id)
    : cv::KeyPoint(x, y, _size, _angle, _response, _octave, _class_id)
{
}
