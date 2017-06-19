This file is a additional remark for the [src](https://github.com/fangchun007/CarND-Unscented-Kalman-Filter-Project) folder.

1.  tools.h, tools.cpp

In the tools, we contained two helper functions. They are used to calculate RMSE and to convert from polar to Cartesian coordinates, respectively.

2. ukf.h, ukf.cpp

The following is the processing flow.

[//]: # (Image References)
[image1]: ./UFK_processingFlow.jpeg
[image2]: ./Result_pic.png

![alt text][image1]

Here, we assume the standar deviation for longitudinal acceleration is 0.8 and assume the standard deviation for yaw acceleration is 0.5.

3. Results

The final RMSE is as follows.

     X: 0.0646
     Y: 0.0851
    VX: 0.2787
    VY: 0.2074

![alt_text][image2]
