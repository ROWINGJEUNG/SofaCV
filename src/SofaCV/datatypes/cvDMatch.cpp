#include "cvDMatch.h"

sofacv::cvDMatch::cvDMatch()
{
    distance = -1.0f;
    imgIdx = -1;
    queryIdx = -1;
    trainIdx = -1;
}

sofacv::cvDMatch::cvDMatch(const cv::DMatch &o)
{
    distance = o.distance;
    imgIdx = o.imgIdx;
    queryIdx = o.queryIdx;
    trainIdx = o.trainIdx;
}

sofacv::cvDMatch::cvDMatch(int _queryIdx, int _trainIdx, float _distance)
    : cv::DMatch(_queryIdx, _trainIdx, _distance)
{
}

sofacv::cvDMatch::cvDMatch(int _queryIdx, int _trainIdx, int _imgIdx, float _distance)
    : cv::DMatch(_queryIdx, _trainIdx, _imgIdx, _distance)
{
}
