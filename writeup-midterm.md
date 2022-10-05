# Writeup: Mid-Term Project: 3D Object Detection

## Observation from point-cloud and coresponding range image
1. The bottom part of vehicles
![pcl_2.png](img/pcl_2.png)

    As shown below, we could observed the points in blue with higher density shows us that should be the bottom of vehicles. Because the more blue the point is the distance between point and the ground is much closer.
And the blue points are surrounding to a object on the road, those might be the bottom part of vehicles.

    Also, we could confirm our guess on the range image show below which is captured at the same moment.
    ![range_2](img/range_2.png)

2. A shape of a truck.
![pcl_3.png](img/pcl_3.png)

    Refer to the red circle of above image, obviously, we guess it would be a truck-like vehicle. And it's clearly to see the truck on the range image.
    ![range_3.png](img/range_3.png)

3. The window and the tires.
![pcl_4.png](img/pcl_4.png)

    Refer to the above image, the red circle shows the tires of a vehicle and the green circle shows the window of it.
    We could observe the tires by the shape of blue points. and we know the window is made by glass, so it reflects less laser from the glass to the receiver, we could see there is fewer point on the region. So it make sense to guess they are windows.
    Also check our guess is correct or not, the following range image shows that we guess right.
    ![range_4.png](img/range_4.png)