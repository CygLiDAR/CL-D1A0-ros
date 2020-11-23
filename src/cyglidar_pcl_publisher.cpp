#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <boost/asio.hpp>
#include <std_msgs/UInt16.h>
#include <cyglidar_pcl.h>
#include <cmath>
#include <thread>

#define PAYLOAD_SIZE            6

#define DATABUFFER_SIZE_2D      249
#define DATASET_SIZE_2D         121

#define DATABUFFER_SIZE_3D      14407
#define DATASET_SIZE_3D         9600

#define COLOR_MIN               0
#define COLOR_MAX               255

#define MATH_PI                 3.14159265

const float BASE_DEPTH_3D = 3000.0;
const float BASE_DEPTH_2D = 16000.0;
const float BASE_ANGLE_2D = 120.0;
const float HALF_ANGLE = 180.0;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointXYZRGBA;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_2D;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_3D;

ros::Publisher pub;
int version_num;
bool version_changed;

uint8_t *cloudBuffer = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];
float *cloudSetBuffer = new float[DATASET_SIZE_3D];
uint8_t *cloudBuffer_2D = new uint8_t[DATABUFFER_SIZE_2D - PAYLOAD_SIZE];
float *cloudSetBuffer_2D = new float[DATASET_SIZE_2D];

uint8_t* bufferPtr_2D;
uint8_t* bufferPtr01;
uint8_t* bufferPtr02;

int order, dataCnt;
uint8_t FIRST, SECOND, THIRD;
uint16_t LSB, MSB;

float centerX, centerY, focalLength, width, height;
float xAxis = 0.0, yAxis = 0.0, zAsix = 0.0;
float actualX = 0.0, actualY = 0.0;

int distanceIndex;
float data1, data2, actualDistance, length;
uint8_t currentBuffer;

static std::vector<uint32_t> colorArray;
void colorBuffer()
{
    int colors = 0;
    uint8_t r = 255;
    uint8_t g = 0;
    uint8_t b = 0;

    for (int colors = 0; colors < 2; colors++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int colorCnt = 0; colorCnt < COLOR_MAX + 1; colorCnt++)
            {
                switch (colors)
                {
                    // RED -> YELLOW
                    case 0:
                        g++;
                        break;
                    // YELLOW -> BLUE
                    case 1:
                        r--;
                        g--;
                        b++;
                        break;
                    // BLUE -> RED
                    case 2:
                        r++;
                        b--;
                        break;
                }

                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                colorArray.push_back(rgb);

                if (colorCnt == COLOR_MAX)
                {
                    colors++;
                }
            }
        }
    }

    ROS_INFO("COLORS STORED (%d)", colorArray.size());
}

bool drawing = false;
uint8_t cloudScatter_2D()
{
    if (!drawing)
    {
        drawing = true;
        memcpy(cloudBuffer_2D, &bufferPtr_2D[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_2D);

        int distanceCnt = 0;
        float angleStep = 0;

        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_2D - PAYLOAD_SIZE) - 1; dataLength+=2)
        {
            LSB = cloudBuffer_2D[dataLength];
            MSB = cloudBuffer_2D[dataLength + 1];

            data1 = (float)((MSB << 8) | LSB);

            cloudSetBuffer_2D[distanceCnt++] = data1;
        }

        float point_angle_var = 0.0;
        angleStep = (BASE_ANGLE_2D / DATASET_SIZE_2D);

        for (int idx = 0; idx < DATASET_SIZE_2D; idx++)
        {
            float point_angle = (float)(((BASE_ANGLE_2D / 2 * -1) + point_angle_var) * MATH_PI / HALF_ANGLE);
            point_angle_var += angleStep;

            actualDistance = (cloudSetBuffer_2D[idx] * 0.1);
            actualX = (sin(point_angle) * actualDistance);
            actualY = (cos(point_angle) * actualDistance); // depth

            scan_2D.get()->points[idx].x = actualX;
            scan_2D.get()->points[idx].y = actualY;
            scan_2D.get()->points[idx].z = 0.0;
            
            if (cloudSetBuffer_2D[idx] < (float)BASE_DEPTH_2D)
            {
                scan_2D.get()->points[idx].r = 255;
                scan_2D.get()->points[idx].g = 255;
                scan_2D.get()->points[idx].b = 0;
                scan_2D.get()->points[idx].a = 255;
            }
            else
            {
                scan_2D.get()->points[idx].a = 0;
            }
        }

        drawing = false;
    }
}

uint8_t cloudScatter_3D()
{
    if (!drawing)
    {
        drawing = true;

        switch (currentBuffer)
        {
            case 0x00:
                memcpy(cloudBuffer, &bufferPtr02[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_3D);
                break;
            case 0x01:
                memcpy(cloudBuffer, &bufferPtr01[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_3D);
                break;
        }
        
        int distanceCnt = 0;
        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_3D - PAYLOAD_SIZE) - 1; dataLength+=3)
        {
            FIRST = cloudBuffer[dataLength];
            SECOND = cloudBuffer[dataLength + 1];
            THIRD = cloudBuffer[dataLength + 2];

            data1 = (float)((FIRST << 4) | (SECOND >> 4));
            data2 = (float)(((SECOND & 0xf) << 8) | THIRD);

            cloudSetBuffer[distanceCnt++] = data1;
            cloudSetBuffer[distanceCnt++] = data2;
        }

        int index = 0;
        for (int yy = 0; yy < height; yy++)
        {
            for (int xx = 0; xx < width; xx++)
            {
                index = (xx + (yy * width));

                float h = sqrt(pow((centerX - xx), 2) + pow((centerY - yy), 2));
                float fh = sqrt(pow(h, 2) + pow(focalLength, 2));
                float dRatio = cloudSetBuffer[index] / fh;

                actualDistance = focalLength * dRatio;
                actualX = (xx - (width / 2)) * actualDistance / focalLength;
                actualY = ((height - yy) - (height / 2)) * actualDistance / focalLength;

                if (cloudSetBuffer[index] < (float)BASE_DEPTH_3D)
                {
                    scan_3D.get()->points[index].x = actualX;
                    scan_3D.get()->points[index].y = actualDistance;
                    scan_3D.get()->points[index].z = actualY;
                    
                    uint32_t rgb = colorArray[(int)actualDistance % colorArray.size()];
                    scan_3D.get()->points[index].rgb = *reinterpret_cast<float*>(&rgb);
                    scan_3D.get()->points[index].a = 255;
                }
                else
                {
                    scan_3D.get()->points[index].x = 0.0;
                    scan_3D.get()->points[index].y = 0.0;
                    scan_3D.get()->points[index].z = 0.0;
                    scan_3D.get()->points[index].a = 0;
                }
            }
        }

        drawing = false;
    }
}

std::string getVersion()
{
    std::string input;
    std::cout << ">>>>> PLEASE ENTER THE TYPE OF DATA YOU WANT TO RECEIVE FROM CYGLIDAR: ";
    std::getline(std::cin, input);

    return input;
}

void running()
{
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string port;
    int baud_rate;
    std::string frame_id;

    priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
    priv_nh.param("baud_rate", baud_rate, 3000000); 
    priv_nh.param("frame_id", frame_id, std::string("laser_link"));

    ROS_INFO("READY");

    colorBuffer();

    pub = nh.advertise<sensor_msgs::PointCloud2>("scan", 1);
    scan_2D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scan_3D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    // PointCloud2 for 2D
    scan_2D.get()->is_dense = false;
    scan_2D.get()->header.frame_id = frame_id;

    scan_2D.get()->points.resize(DATABUFFER_SIZE_2D);
    for (int point = 0; point < DATABUFFER_SIZE_2D; point++)
    {
        scan_2D.get()->points[point].a = 0;
    }

    // PointCloud2 for 3D
    scan_3D.get()->width = 160;
    scan_3D.get()->height = 60;
    scan_3D.get()->is_dense = false;
    scan_3D.get()->header.frame_id = frame_id;

    width = (float)(scan_3D.get()->width);
    height = (float)(scan_3D.get()->height);
    centerX = width / 2.0;
    centerY = height / 2.0;
    focalLength = 40.5;

    scan_3D.get()->points.resize(DATABUFFER_SIZE_3D);
    for (int point = 0; point < DATABUFFER_SIZE_3D; point++)
    {
        scan_3D.get()->points[point].a = 0;
    }

    dataCnt = 0;
    distanceIndex = 0;
    actualDistance = 0.0;

    length = 0.0;
    order = 0;

    drawing = false;
    currentBuffer = 0x00;

    version_changed = false;
    boost::asio::io_service io;
    try
    {
        cyglidar_pcl_driver::cyglidar_pcl laser(port, baud_rate, io);

        version_num = std::stoi(getVersion());
        laser.packet(version_num);

        while (ros::ok())
        {
            uint8_t* result = laser.poll(version_num);
            if (result != false)
            {/*
                ROS_INFO(">>>>>>> TEST: %x, %x, %x, %x, %x, %x",
                result[0], result[1], result[2], result[3], result[4], result[5]);*/

                switch (result[5])
                {
                    case 0x01: // 2D
                        bufferPtr_2D = &result[0];
                        cloudScatter_2D();

                        pcl_conversions::toPCL(ros::Time::now(), scan_2D->header.stamp);
                        pub.publish(scan_2D);
                        break;
                    case 0x08: // 3D
                        switch (currentBuffer)
                        {
                            case 0x00:
                                bufferPtr01 = &result[0];
                                currentBuffer = 0x01;
                                break;
                            case 0x01:
                                bufferPtr02 = &result[0];
                                currentBuffer = 0x00;
                                break;
                        }
                        cloudScatter_3D();

                        pcl_conversions::toPCL(ros::Time::now(), scan_3D->header.stamp);
                        pub.publish(scan_3D);
                        break;
                }
            }
        }
        laser.close();
    }
    catch (boost::system::system_error ex)
    {
        ROS_ERROR("[Error] instantiating laser object. \
        Are you sure you have the correct port and baud rate? \
        Error was %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "line_laser");

    //std::thread first(getVersion);
    std::thread second(running);

    //first.join();
    second.join();

    return 0;
}
