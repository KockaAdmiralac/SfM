#include <icp.hpp>

int icp(VertexList ourCloud, VertexList theirCloud)
{
    int result = 0;
    for (Vertex v : ourCloud)
    {
        int minDist = INT32_MAX;
        for (Vertex v2 : theirCloud)
        {
            int xDist = v.x - v2.x,
                yDist = v.y - v2.y,
                zDist = v.z - v2.z,
                dist = xDist * xDist + yDist * yDist + zDist * zDist;
            if (dist < minDist)
            {
                minDist = dist;
            }
        }
        result += minDist;
    }
    return result;
}
