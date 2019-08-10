#include <cstdio>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <opencv2/imgcodecs.hpp>

#include <kitti.hpp>

cv::Mat readMatrix(std::string line) {
    std::istringstream lineStream(line);
    std::string element;
    int index = 0;
    cv::Mat result(3, 4, CV_64F);
    do {
        std::string element;
        lineStream >> element;
        if (element.size() == 0) {
            break;
        }
        result.at<double>(index / 4, index % 4) = stod(element);
        index++;
    } while(lineStream);
    return result;
}

Sequence::Sequence(int number) : number(number) {
    char calibFilename[PATH_MAX];
    char posesFilename[PATH_MAX];
    char timesFilename[PATH_MAX];
    char imagesPath[PATH_MAX];
    sprintf(calibFilename, "../dataset/sequences/%02d/calib.txt", number);
    sprintf(posesFilename, "../dataset/poses/%02d.txt", number);
    sprintf(timesFilename, "../dataset/sequences/%02d/times.txt", number);
    sprintf(imagesPath, "../dataset/sequences/%02d/image_0", number);
    std::ifstream calibFile(calibFilename),
                  posesFile(posesFilename),
                  timesFile(timesFilename);
    std::string line;
    while (std::getline(calibFile, line)) {
        if (line[0] == 'P') {
            calib.push_back(readMatrix(line.substr(4)));
        } else if (line[0] == 'T') {
            tr = readMatrix(line.substr(4));
        }
    }
    while (std::getline(posesFile, line)) {
        poses.push_back(readMatrix(line));
    }
    while (std::getline(timesFile, line)) {
        times.push_back(stod(line));
    }
    calibFile.close();
    posesFile.close();
    timesFile.close();
    fileNumber = std::distance(
        std::filesystem::directory_iterator(imagesPath),
        std::filesystem::directory_iterator{}
    );
    std::cout << fileNumber << std::endl;
}

cv::Mat Sequence::image(int image, int sequence) {
    char imageFilename[PATH_MAX];
    sprintf(imageFilename, "../dataset/sequences/%02d/image_%d/%06d.png", number, image, sequence);
    return cv::imread(imageFilename, cv::IMREAD_GRAYSCALE);
}
