#ifndef _KDTREE_H
#define _KDTREE_H
template<typename T>
struct KDTree
{
    T *location;
    KDTree<T> *leftChild;
    KDTree<T> *rightChild;
    KDTree(int k, int depth, std::vector<T> &points, int start, int end);
private:
    double getValueForAxis(T point, int axis);
};
#endif
