// created by:  zhiyu zhou 2025/3/2
#ifndef IMG_H
#define IMG_H
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <opencv2/ximgproc.hpp>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Mapper/VoxelMapper.h"
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "Common.h"
#include <mutex>
#include <pcl/features/boundary.h>

struct Matchlinelist
    {
      double k;
      pcl::PointXYZI p3d;
      std::vector<int> p2d;
      std::vector<int> linestart;
      std::vector<int> lineend;
      double a,b,c;
    };
class imgProcesser
    {
    public:
    struct Plane
    {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZRGB p_center;
    Eigen::Vector3d normal;
    int index;
    };
    
    cv::Mat visualimg(cv::Mat img,std::vector<Matchlinelist>& matchlinelist);
    void buildmatchlinelist(std::vector<Matchlinelist>& matchlinelist, pcl::PointCloud<pcl::PointXYZI>::Ptr& _sparselinecloud, std::vector<std::vector<int>> segments_list);
    cv::Mat projectPinhole(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, cv::Mat scanlineIdMap, bool isinter);
    cv::Mat fillHolesFast(const cv::Mat& input);
    void cannyEdgeDetection(const cv::Mat& src, cv::Mat& dst);
    void extractIntensityEdgesOptimized(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, 
        pcl::PointCloud<pcl::PointXYZI>::Ptr& intensity_edge);
    cv::Mat densifyLidarPointCloud(const cv::Mat& inputImage, const cv::Mat& templateImage);
    cv::Mat interpolateLidarImage(const cv::Mat& sparse_image);
    void removeLines(cv::Mat& img);
    void filterBrightness(cv::Mat& img);
    cv::Mat projectToXZ(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,cv::Mat scanlineIdMap,bool isinter) ;
    cv::Mat overlayRedOnGrayscale(const cv::Mat& gray1, const cv::Mat& gray2);
    cv::Mat stackImagesVertical(const cv::Mat& gray1, const cv::Mat& gray2) ;
    cv::Mat draw_matches(cv::Mat &ref_points, cv::Mat &dst_points, cv::Mat &img1, cv::Mat &img2);
    Eigen::Vector2d projectTosingle(pcl::PointXYZI p);
    void calcLineLin(std::vector<Plane>& plane_list, std::vector<pcl::PointCloud<pcl::PointXYZI>>& line_cloud_list);
    void initVoxel(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        const float voxel_size,
        std::unordered_map<VOXEL_LOC, Voxel*>& voxel_map);
    void extractLineFeatures(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr& _sparselinecloud
        );
    void LiDAREdgeExtraction(std::unordered_map<VOXEL_LOC, Voxel*>& voxel_map, const float ransac_dis_thre, const int plane_size_threshold,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_line_cloud_3d);
        void FitPlaneSVD(Plane& single_plane)//????????????????棬????????????????
        {
            // 1??????????	// 2???????
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_buf(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(single_plane.cloud, *cloud_buf);
            Eigen::MatrixXf cloudMat = cloud_buf->getMatrixXfMap(3, 4, 0);
            Eigen::VectorXf centroid(cloudMat.rows());
            for (int i = 0; i < cloudMat.rows(); i++)
            {
                centroid(i) = cloudMat.row(i).mean();
                cloudMat.row(i) = cloudMat.row(i) - (Eigen::VectorXf::Ones(cloudMat.cols()).transpose() * centroid(i));
            }
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(cloudMat, Eigen::ComputeFullU);
            Eigen::MatrixXf U = svd.matrixU();
            Eigen::VectorXf S = svd.singularValues();
        
            // std::cout<<std::endl<<U.col(2)<<std::endl;//model.coefficients.topRows(3).dot(centroid);
            // std::cout<<U.col(2).dot(centroid)<<std::endl;
            // std::cout<<single_plane.p_center<<std::endl;
            // std::cout<<centroid<<std::endl;
        
            single_plane.normal[0] = U.col(2)[0];
            single_plane.normal[1] = U.col(2)[1];
            single_plane.normal[2] = U.col(2)[2];
            single_plane.p_center.getVector3fMap() = centroid;
        }

        void mergePlanes(std::vector<Plane>& PlaneList)
        {
            int CurrentLength = PlaneList.size();
            int BaseIndex = 0;
        
            while (1)
            {
                CurrentLength = PlaneList.size();
                if (BaseIndex >= CurrentLength)
                {
                    break;
                }
                Eigen::Vector3d normal = PlaneList[BaseIndex].normal;
                pcl::PointXYZRGB center_PCL = PlaneList[BaseIndex].p_center;
                Eigen::Vector3d center(center_PCL.x, center_PCL.y, center_PCL.z);
                PlaneList[BaseIndex].index = BaseIndex;
        
                int key = 0;
                while (1)
                {
                    if (key + BaseIndex + 1 >= CurrentLength)
                        break;
                    Eigen::Vector3d normal_t = PlaneList[key + BaseIndex + 1].normal;
                    pcl::PointXYZRGB center_tPCL = PlaneList[key + BaseIndex + 1].p_center;
                    Eigen::Vector3d center_t(center_tPCL.x, center_tPCL.y, center_tPCL.z);
        
                    float D_t = -(normal_t.dot(center_t));//second plane ABCD
                    float cosAngle = abs(normal.dot(normal_t));
                    float Distance = abs(normal_t.dot(center) + D_t);
        
                    if ((cosAngle > 0.7) && (Distance < 0.05))//0.1
                    {
                        pcl::copyPointCloud(PlaneList[BaseIndex].cloud + PlaneList[key + BaseIndex + 1].cloud, PlaneList[BaseIndex].cloud);
                        FitPlaneSVD(PlaneList[BaseIndex]);
                        PlaneList.erase(PlaneList.begin() + key + BaseIndex + 1);
                        // cout<<"fit"<<BaseIndex<<" "<<key + BaseIndex + 1;
                        CurrentLength = PlaneList.size();
                        // PlaneList[index]
                    }
                    else
                    {
                        // cout<<"pass"<<BaseIndex<<" "<<key + BaseIndex + 1;
                        key++;
                    }
                }
                PlaneList[BaseIndex].index = BaseIndex;
                BaseIndex++;
            }
        }
        
        
        void savePlanes(std::vector<Plane> plane_list, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)
        {
            for (int i = 0; i < plane_list.size(); i++)
            {
                std::vector<unsigned int> colors;
                colors.push_back(static_cast<unsigned int>(rand() % 256));
                colors.push_back(static_cast<unsigned int>(rand() % 256));
                colors.push_back(static_cast<unsigned int>(rand() % 256));
                for (size_t j = 0; j < plane_list[i].cloud.points.size(); j++)
                {
                    //plane_list[i].cloud.points[j].g
                    pcl::PointXYZRGB p;
                    p.getArray3fMap() = plane_list[i].cloud.points[j].getArray3fMap();
                    p.r = colors[0];
                    p.g = colors[1];
                    p.b = colors[2];
                    pc->push_back(p);
                }
            }
        }
    private:
        int size_x;
        int size_y;
    };

     #endif