///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/************************************************************************************
 ** This sample demonstrates how to use PCL (Point Cloud Library) with the ZED SDK **
 ************************************************************************************/

// ZED includes
#include <sl_zed/Camera.hpp>

// PCL includes
// Undef on Win32 min/max for PCL
#ifdef _WIN32
#undef max
#undef min
#endif
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
// OpenCV includes
#include <opencv2/opencv.hpp>
// Sample includes
#include <thread>
#include <mutex>

// Namespace
using namespace sl;
using namespace std;

// Global instance (ZED, Mat, callback)
Camera zed;
Mat data_cloud;
std::thread zed_callback;
std::mutex mutex_input;
bool stop_signal;
bool has_data;

// Sample functions
void startZED();
void run();
void closeZED();
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
inline float convertColor(float colorIn);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* p_pcl_point_cloud_);
cv::Mat slMat2cvMat(Mat& input);
// Main process

int main(int argc, char **argv) {

    if (argc > 2) {
        cout << "Only the path of a SVO can be passed in arg" << endl;
        return -1;
    }

    // Set configuration parameters
    InitParameters init_params;
    if (argc == 2)
        init_params.svo_input_filename = argv[1];
    else {
        init_params.camera_resolution = RESOLUTION_HD720;
        init_params.camera_fps = 30;
    }
    init_params.coordinate_units = UNIT_METER;
    init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.depth_mode = DEPTH_MODE_MEDIUM;


    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        return 1;
    }

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getResolution();
    int new_width = image_size.width ;
    int new_height = image_size.height;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed(new_width, new_height, MAT_TYPE_8U_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);

    cv::Point_<u_int32_t> left_up(420,180);
    cv::Point_<u_int32_t> right_up(860,180);
    cv::Point_<u_int32_t> right_down(860,540);
    cv::Point_<u_int32_t> left_down(420,540);

    // Allocate PCL point cloud at the resolution,organized clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->width = right_up.x-left_up.x;
    p_pcl_point_cloud->height = left_down.y-left_up.y;
    p_pcl_point_cloud->points.resize(p_pcl_point_cloud->width * p_pcl_point_cloud->height);
//    std::cout<<"points:"<<p_pcl_point_cloud->points.size()<<std::endl;
//    std::cout<<"width*height:"<<p_pcl_point_cloud->width * p_pcl_point_cloud->height<<std::endl;


    // Create the PCL point cloud visualizer
    shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&p_pcl_point_cloud);
    // Start ZED callback
    startZED();

        // Print camera information
    printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

    // Create a Mat to store images
    Mat zed_image;

    // Capture new images until 'q' is pressed
    char key_cv = ' ';
    while (key_cv != 'q' && !viewer->wasStopped()) {
        // Check that grab() is successful
        if (zed.grab(runtime_parameters) == SUCCESS) {
            // Retrieve left image
            zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, new_width, new_height);
            cv::line(image_ocv,left_up,right_up,cv::Scalar_<int>(0,0,255));
            cv::line(image_ocv,right_up,right_down,cv::Scalar_<int>(0,0,255));
            cv::line(image_ocv,right_down,left_down,cv::Scalar_<int>(0,0,255));
            cv::line(image_ocv,left_down,left_up,cv::Scalar_<int>(0,0,255));
            // Display image with OpenCV
            cv::imshow("VIEW", image_ocv);
            key_cv = cv::waitKey(10);
        }

        if (mutex_input.try_lock()) {
            float *p_data_cloud = data_cloud.getPtr<float>();
            for(int i = 0;i<p_pcl_point_cloud->height;i++)
            {
                for(int j = 0; j < p_pcl_point_cloud->width; j++)
                {
                    int index_zed = 4 * zed.getResolution().width * (left_up.y + i) + 4 * (left_up.x + j);
                    int index_pcl = i*p_pcl_point_cloud->width+j;
                    float X = p_data_cloud[index_zed];
                    if(!isValidMeasure(X))
                        p_pcl_point_cloud->points[index_pcl].x=p_pcl_point_cloud->points[index_pcl].x=p_pcl_point_cloud->points[index_pcl].x=p_pcl_point_cloud->points[index_pcl].rgb=0;
                    else{
                        p_pcl_point_cloud->points[index_pcl].x = p_data_cloud[index_zed];
                        p_pcl_point_cloud->points[index_pcl].y = p_data_cloud[index_zed+1];
                        p_pcl_point_cloud->points[index_pcl].z = p_data_cloud[index_zed+2];
                        p_pcl_point_cloud->points[index_pcl].rgb = convertColor(p_data_cloud[index_zed + 3]);
                    }

                }
            }
//            // Check and adjust points for PCL format
//            for (auto &it : p_pcl_point_cloud->points) {
//                float X = p_data_cloud[index];
//                if (!isValidMeasure(X)) // Checking if it's a valid point
//                    it.x = it.y = it.z = it.rgb = 0;
//                else {
//                    it.x = X;
//                    it.y = p_data_cloud[index + 1];
//                    it.z = p_data_cloud[index + 2];
//                    it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
//                }
//                index += 4;
//            }

            // Unlock data and update Point cloud
            mutex_input.unlock();
            viewer->updatePointCloud(p_pcl_point_cloud);
            viewer->spinOnce(10);
        }
//        else
//            sleep_ms(1);



    }


    // Close the viewer
    viewer->close();

    // Close the zed
    closeZED();

    return 0;
}

/**
 *  This functions start the ZED's thread that grab images and data.
 **/
void startZED() {
    // Start the thread for grabbing ZED data
    stop_signal = false;
    has_data = false;
    zed_callback = std::thread(run);

    //Wait for data to be grabbed
    while (!has_data)
        sleep_ms(1);
}

/**
 *  This function loops to get the point cloud from the ZED. It can be considered as a callback.
 **/
void run() {
    while (!stop_signal) {
        if (zed.grab(SENSING_MODE_STANDARD) == SUCCESS) {
            mutex_input.lock(); // To prevent from data corruption
            zed.retrieveMeasure(data_cloud, MEASURE_XYZRGBA);
            mutex_input.unlock();
            has_data = true;
        } else
            sleep_ms(1);
    }
}

/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void closeZED() {
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
}

/**
 *  This function creates a PCL visualizer
 **/
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

/**
 *  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* p_pcl_point_cloud_void)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud_ = *static_cast<pcl::PointCloud<pcl::PointXYZRGB>::Ptr *> (p_pcl_point_cloud_void);
    if (event.getKeySym () == "m" && event.keyDown ())
    {

        pcl::io::savePCDFileASCII ("test_pcd720_medium.pcd", *p_pcl_point_cloud_);
        std::cerr << "Saved " << (*p_pcl_point_cloud_).points.size () << " data points to test_pcd.pcd." << std::endl;
    }
}
/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}