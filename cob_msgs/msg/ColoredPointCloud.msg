Header header
sensor_msgs/Image colorImage
sensor_msgs/Image greyImage
sensor_msgs/Image pointCloud

# Reprojection matrix
# Reprojects 3-D coordinates to image coordinates
float64[9] reprojectionMatrix # 3x3 row-major matrix
