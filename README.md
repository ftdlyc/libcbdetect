Libdetect
---
Unofficial implemention of [libcbdetect](http://www.cvlibs.net/software/libcbdetect/) in C++.  
  
Libdetect is a fully automatic sub-pixel checkerboard / chessboard pattern detection. The algorithm autmatically extracts corners to sub-pixel accuracy and combines them to (rectangular) checkerboards / chessboard-like patterns.  
  
#### Require
- C++ 14  
- Opencv >= 3.0  
  
#### Example
> using namespace cbdetect;  
> cv::Mat img = cv::imread("image.bmp", cv::IMREAD_GRAYSCALE);  
> Params params { true, true, 0.01 };  
> find_corners(img, corners, params);  
> plot_corners(img, corners);  
> chessboards_from_corners(corners, chessboards);  
> plot_chessboards(img, corners, chessboards);

![image](https://github.com/ftdlyc/libcbdetect/blob/master/example_data/e2_result.png)  
    
#### Reference Papers
[1]  Andreas Geiger and Frank Moosmann and Oemer Car and Bernhard Schuster, Automatic Calibration of Range and Camera Sensors using a single Shot, ICRA, 2012  
[2] Miriam Schoenbein and Tobias Strauss and Andreas Geiger, Calibrating and Centering Quasi-Central Catadioptric Cameras, ICRA, 2014  