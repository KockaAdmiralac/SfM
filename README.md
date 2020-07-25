# Project Title

Summer project in [Petnica Science Center](http://petnica.rs), Serbia.

Goal of project was comparing combinations of algorithms used to solve [Structure from Motion](https://en-wp.org/wiki/Structure_from_Motion).

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

- [OpenCV](https://opencv.org/)
- [KITTI dataset](http://www.cvlibs.net/datasets/kitti/), you need odometry data
tbw.
### Installing
A step by step series of examples that tell you how to get a development env running

Clone the repo using

```
git clone git@github.com:KockaAdmiralac/SfM.git
```

Edit the CMakeLists.txt in order to get it running on your system, open an issue if you need assistance with this step.
tbw.
## Running the tests
You can open the code in any editor of your choice, in order to compile it you can use these commands:

Run it on one sequence, one combination of algorithms
```
./metrics.sh (method) ("opencv"|"our") (RANSAC parameter 1) (RANSAC parameter 2) (RANSAC parameter 3) (matcher) (match threshold) (KITTI sequence)
```

Run it on one sequence, multiple combinations of algorithms
```
./all-metrics.sh
```
tbw.
## Built With

* [OpenCV](https://opencv.org/) - OpenCV

tbw.

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

- **Petar Marković** - *Initial work* - [petarpetarpetar](https://github.com/petarpetarpetar)
- **Luka Simić** - *Initial work* - [KockaAdmiralac](https://github.com/KockaAdmiralac)

Because it was a part of Applied Physics and Electonics programme in [Science Center Petnica](http://petnica.rs/), we had two mentors:

* **Damjan Dakić** - *First Mentor*
* **Nikola Milenić** - *Second Mentor*

See also the list of [contributors](https://github.com/KockaAdmiralac/SfM/contributors) who participated in this project.


## Acknowledgments

- Hat tip to anyone whose code was used
tbw.
