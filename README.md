Libcbdetect
---
- Unofficial implemention of [libcbdetect](http://www.cvlibs.net/software/libcbdetect/) in C++.  
- Deltille detector.  

Libdetect is a fully automatic sub-pixel checkerboard / chessboard / deltille pattern detection. The algorithm autmatically extracts corners to sub-pixel accuracy and combines them to (rectangular) checkerboards / chessboard-like / deltille patterns.  
  
My calibration tool: [Libcalib](https://github.com/ftdlyc/libcalib)
  
#### Require
- C++ 14  
- Opencv >= 3.0  
  
#### Example
> using namespace cbdetect;  
> cv::Mat img = cv::imread("image.bmp", cv::IMREAD_COLOR);  
> Params params;  
> find_corners(img, corners, params);  
> plot_corners(img, corners);  
> boards_from_corners(corners, boards);  
> plot_boards(img, corners, boards);  
  
![image](https://github.com/ftdlyc/libcbdetect/blob/master/example_data/e2_result.png)  
![image](https://github.com/ftdlyc/libcbdetect/blob/master/example_data/e6_result.png)  
![image](https://github.com/ftdlyc/libcbdetect/blob/master/example_data/e7_result.png)  
  
#### Reference Papers
[1] Geiger, A., Moosmann, F., Car, Ö., & Schuster, B. (2012, May). Automatic camera and range sensor calibration using a single shot. In Robotics and Automation (ICRA), 2012 IEEE International Conference on (pp. 3936-3943). IEEE.  
[2] Schönbein, M., Strauß, T., & Geiger, A. (2014, May). Calibrating and centering quasi-central catadioptric cameras. In Robotics and Automation (ICRA), 2014 IEEE International Conference on (pp. 4443-4450). IEEE.  
[3] Placht, S., Fürsattel, P., Mengue, E. A., Hofmann, H., Schaller, C., Balda, M., & Angelopoulou, E. (2014, September). Rochade: Robust checkerboard advanced detection for camera calibration. In European Conference on Computer Vision (pp. 766-779). Springer, Cham.  
[4] Ha, H., Perdoch, M., Alismail, H., Kweon, I. S., & Sheikh, Y. (2017, October). Deltille Grids for Geometric Camera Calibration. In 2017 IEEE International Conference on Computer Vision (ICCV) (pp. 5354-5362). IEEE.  
[5] Duda, A., & Frese, U. (2018, September). Accurate Detection and Localization of Checkerboard Corners for Calibration. In British Machine Vision Conference (BMCV), 2018.  
