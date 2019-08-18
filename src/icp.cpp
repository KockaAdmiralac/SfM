#include <iostream>
#include <fstream>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#ifndef SEQUENCE_NUMBER
#define SEQUENCE_NUMBER 1
#endif

int main (int argc, char** argv)
{
    if (SEQUENCE_NUMBER != 0)
    {
        return 0;
    }
    pcl::PLYReader plyReader;
    for (int i = 0; i < 4540; ++i)
    {
        char ourCloudFilename[PATH_MAX];
        char velodyneCloudFilename[PATH_MAX];
        sprintf(ourCloudFilename, "TEMP/TRIANGULATION/00/%02d.ply", i);
        sprintf(velodyneCloudFilename, "TEMP/VELODYNE/%04d.ply", i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ourCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr velodyneCloud(new pcl::PointCloud<pcl::PointXYZ>);
        plyReader.read(ourCloudFilename, *ourCloud);
        plyReader.read(velodyneCloudFilename, *velodyneCloud);

        // Execute ICP.
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(velodyneCloud);
        icp.setInputTarget(ourCloud);
        pcl::PointCloud<pcl::PointXYZ> finalCloud;
        icp.align(finalCloud);

        printf("%f\n", icp.getFitnessScore());
    }
    return 0;
}
