#include <opencv2/core.hpp>

#include <kdtree.hpp>
#include <ply.hpp>

template<typename T>
KDTree<T>::KDTree(int k, int depth, std::vector<T> &points, int start, int end)
{
    int axis = depth % k;
    std::sort(points.begin() + start, points.begin() + end, [](T a, T b)
    {
        return getValueForAxis(a) - getValueForAxis(b);
    });
    int medianIndex = (end - start) / 2;
    location = &points[medianIndex];
    leftChild = KDTree<T>(k, depth + 1, points, start, medianIndex);
    rightChild = KDTree<T>(k, depth + 1, points, medianIndex + 1, end);
}

template<typename T>
double KDTree<T>::getValueForAxis(T point, int axis)
{
    return 0;
}

template<>
double KDTree<cv::Point2d>::getValueForAxis(cv::Point2d point, int axis)
{
    switch (axis)
    {
        case 0:
            return point.x;
        default:
            return point.y;
    }
}

template<>
double KDTree<Vertex>::getValueForAxis(Vertex point, int axis)
{
    switch (axis)
    {
        case 0:
            return point.x;
        case 1:
            return point.y;
        default:
            return point.z;
    }
    return 0;
}
