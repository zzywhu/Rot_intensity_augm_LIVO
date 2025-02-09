//
// Created by w on 2022/9/26.
//
#include "System.h"
#include "CloudInfo.h"

float System::calcDist(const PointType &p1, const PointType &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

void System::updateDenseMap(PointCloudXYZI::Ptr cloudPtr)
{
    _dispMutex.lock();
    PointCloudXYZI::Ptr cloudAlignedPtr(new PointCloudXYZI);
    transCloud(cloudPtr, cloudAlignedPtr, _rotAlign, Eigen::Vector3d::Zero());
    *_denseCloudMap += *cloudAlignedPtr;
    _dispMutex.unlock();
}

void System::pointBodyToWorld(state_ikfom& stateCur, PointType const *const pi, PointType *const po)
{
    V3D pl(pi->x, pi->y, pi->z); // pl
    V3D pw(stateCur.rot * (stateCur.offset_R_L_I * pl + stateCur.offset_T_L_I) + stateCur.pos); // pw=Twi*(Til*pl)

    po->x = pw(0);
    po->y = pw(1);
    po->z = pw(2);
    po->intensity = pi->intensity;
}

void System::pointBodyToWorld(StatesGroup& stateCur, PointType const *const pi, PointType *const po)
{
    V3D pl(pi->x, pi->y, pi->z); // pl
    V3D pw(stateCur.rot_end * (stateCur.offset_R_L_I * pl + stateCur.offset_T_L_I) + stateCur.pos_end); // pw=Twi*(Til*pl)

    po->x = pw(0);
    po->y = pw(1);
    po->z = pw(2);
    po->intensity = pi->intensity;
}

void System::transCloud(const PointCloudXYZI::Ptr &cloudIn, PointCloudXYZI::Ptr &cloudOut,
                        const Eigen::Matrix3d &Rol, const Eigen::Vector3d &tol)
{
    const int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    for (int i = 0; i < cloudSize; i++)
    {
        const auto &pb = cloudIn->points[i];
        V3D pl(pb.x, pb.y, pb.z); // pl
        V3D po(Rol * pl + tol);   // po=(Tol*pl)

        cloudOut->points[i].x = po[0];
        cloudOut->points[i].y = po[1];
        cloudOut->points[i].z = po[2];
        cloudOut->points[i].intensity = pb.intensity;
        cloudOut->points[i].curvature = pb.curvature;
        cloudOut->points[i].normal_x = pb.normal_x;
        cloudOut->points[i].normal_y = pb.normal_y;
        cloudOut->points[i].normal_z = pb.normal_z;
    }
}

void System::transCloudInMotorAxis(const PointCloudXYZI::Ptr &cloudIn, PointCloudXYZI::Ptr &cloudOut,
                                   const double &angle, const Eigen::Matrix3d &Rol, const Eigen::Vector3d &tol)
{
    const int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    for (int i = 0; i < cloudSize; i++)
    {
        const auto &pb = cloudIn->points[i];
        V3D pl(pb.x, pb.y, pb.z); // pl
        V3D po(Rol * pl + tol);   // po=(Tol*pl)

        double rot[3] = {0, 0, angle};
        double pworld[3];
        ceres::AngleAxisRotatePoint(rot, po.data(), pworld);
        cloudOut->points[i].x = pworld[0];
        cloudOut->points[i].y = pworld[1];
        cloudOut->points[i].z = pworld[2];
        cloudOut->points[i].intensity = pb.intensity;
        cloudOut->points[i].curvature = pb.curvature;
        cloudOut->points[i].normal_x = pb.normal_x;
        cloudOut->points[i].normal_y = pb.normal_y;
        cloudOut->points[i].normal_z = pb.normal_z;
    }
}

void System::printState(const state_ikfom& state, const std::string& prefix)
{
//    static std::ofstream fExtrinsicOfs =std::ofstream(RESULT_FILE_DIR("Extrinsic.txt"), std::ios::trunc | std::ios::in);
//    fExtrinsicOfs.setf(ios::fixed);
//    fExtrinsicOfs << setprecision(32);
//    Eigen::Vector3d euler=RotMtoEuler(_stateIkfom.offset_R_L_I.matrix()).transpose() * 57.3;
//    Eigen::Vector3d translation=_stateIkfom.offset_T_L_I.transpose();
//    fExtrinsicOfs<<euler[0]<<","<<euler[1]<<","<<euler[2]<<","<<translation[0]<<","<<translation[1]<<","<<translation[2]<<std::endl;

//    cout << "Rotation LiDAR to IMU    = " << state.offset_R_L_I.matrix()<< endl;
//    cout << "Translation LiDAR to IMU = " << state.offset_T_L_I.matrix()<< endl;

    cout.setf(ios::fixed);
    //cout << setprecision(32)<< endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Rotation LiDAR to IMU    = " << RotMtoEuler(state.offset_R_L_I.matrix()).transpose() * 57.3<< " deg" << endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Translation LiDAR to IMU = " << state.offset_T_L_I.matrix().transpose() << " m" << endl;
//    printf(BOLDGREEN "[Final Result] " RESET);
//    printf("Time Lag IMU to LiDAR    = %.8lf s \n", _timediffImuWrtLidar + _config._timeLagIMUWtrLidar);
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Velocity in World Frame   = " << state.vel.matrix().transpose() << " m/s" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Bias of Gyroscope        = " << state.bg.matrix().transpose() << " rad/s" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Bias of Accelerometer    = " << state.ba.matrix().transpose() << " m/s^2" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Gravity in World Frame   = " << state.grav.get_vect().transpose() << " m/s^2" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Pos in World Frame   = " << state.pos.matrix().transpose() << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Rot in World Frame   = " << RotMtoEuler(state.rot.matrix()).transpose() * 57.3 << endl;
    // cout << "P  = " << _kf.get_P()<< endl;
}

void System::printState(const std::string& prefix)
{
//    static std::ofstream fExtrinsicOfs =std::ofstream(RESULT_FILE_DIR("Extrinsic.txt"), std::ios::trunc | std::ios::in);
//    fExtrinsicOfs.setf(ios::fixed);
//    fExtrinsicOfs << setprecision(6)<< endl;
//    Eigen::Vector3d euler=RotMtoEuler(_stateCur.offset_R_L_I).transpose() * 57.3;
//    Eigen::Vector3d translation=_stateCur.offset_T_L_I.transpose();
//    fExtrinsicOfs<<euler[0]<<","<<euler[1]<<","<<euler[2]<<","<<translation[0]<<","<<translation[1]<<","<<translation[2]<<std::endl;

    cout.setf(ios::fixed);
    //cout << setprecision(32)<< endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Rotation LiDAR to IMU    = " << RotMtoEuler(_stateCur.offset_R_L_I).transpose() * 57.3<< " deg" << endl;
    printf(BOLDGREEN "[Final Result] " RESET);
    cout << "Translation LiDAR to IMU = " << _stateCur.offset_T_L_I.matrix().transpose() << " m" << endl;
//    printf(BOLDGREEN "[Final Result] " RESET);
//    printf("Time Lag IMU to LiDAR    = %.8lf s \n", _timediffImuWrtLidar + _config._timeLagIMUWtrLidar);
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Velocity in World Frame   = " << _stateCur.vel_end.matrix().transpose() << " m/s" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Bias of Gyroscope        = " << _stateCur.bias_g.matrix().transpose() << " rad/s" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Bias of Accelerometer    = " << _stateCur.bias_a.matrix().transpose() << " m/s^2" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Gravity in World Frame   = " << _stateCur.gravity.transpose() << " m/s^2" << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Pos in World Frame   = " << _stateCur.pos_end.matrix().transpose() << endl;
    printf(BOLDGREEN "[%s] " RESET, prefix.c_str());
    cout << "Rot in World Frame   = " << RotMtoEuler(_stateCur.rot_end).transpose() * 57.3 << endl;
    // cout << "P  = " << _kf.get_P()<< endl;
}

void System::printProgress(double percentage)
{
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\033[1A\r");
    printf(BOLDMAGENTA "[Refinement] ");
    if (percentage < 1)
    {
        printf(BOLDYELLOW "Online Refinement: ");
        printf(YELLOW "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    }
    else
    {
        printf(BOLDGREEN " Online Refinement ");
        printf(GREEN "%3d%% [%.*s%*s]\n", val, lpad, PBSTR, rpad, "");
        cout << RESET;
    }
    fflush(stdout);
}

void System::fileoutCalibResult(ofstream &foutResult, const StatesGroup &state, const double &timeLagIMUWrtLidar)
{
    foutResult.setf(ios::fixed);
    foutResult << setprecision(6)
               << "Rotation LiDAR to IMU (degree)     = " << RotMtoEuler(state.offset_R_L_I).transpose() * 57.3 << endl;
    foutResult << "Translation LiDAR to IMU (meter)   = " << state.offset_T_L_I.transpose() << endl;
    foutResult << "Time Lag IMU to LiDAR (second)     = " << timeLagIMUWrtLidar << endl;
    foutResult << "Bias of Gyroscope  (rad/s)         = " << state.bias_g.transpose() << endl;
    foutResult << "Bias of Accelerometer (meters/s^2) = " << state.bias_a.transpose() << endl;
    foutResult << "Gravity in World Frame(meters/s^2) = " << state.gravity.transpose() << endl
               << endl;

    MD(4, 4) Transform;
    Transform.setIdentity();
    Transform.block<3, 3>(0, 0) = state.offset_R_L_I;
    Transform.block<3, 1>(0, 3) = state.offset_T_L_I;
    foutResult << "Homogeneous Transformation Matrix from LiDAR to IMU: " << endl;
    foutResult << Transform << endl
               << endl
               << endl;
}


void System::correctLoop()
{
    if(!config()._isLoopEn)
        return;
    if(LoopCloser::config()._isCorrectRealTime)
        return;

    _loopCloser->getProcessLock().lock();
    auto& pgoEdges = _loopCloser->getPgoEdges();
    //loopCloser->addSubMapBlock(loopCloser->getCurSubmap());
    //const auto& _subMapBlocks = _loopCloser->getSubMapBlocks();
    int loopCount=0;
   // int i=1;
    for(auto& edge:pgoEdges)
    {
        if(edge.con_type!=REGISTRATION)
            continue;
        //edge.information_matrix=(i++)*100.*edge.information_matrix;
        edge.information_matrix=100.*edge.information_matrix;
        //  edge.information_matrix=Eigen::Matrix<double, 6, 6>::Identity();
        loopCount++;
    }
    std::cout<<"Detected loop count:"<<loopCount<<std::endl;
    if(loopCount>0)
    {
        if(_loopCloser->config()._isFramewisePGO)
        {
            if(_loopCloser->config()._isPgoIncremental)
                _loopCloser->frameGraphOptimize();
            else//while gnss data
                _loopCloser->correctFrameLoop();
        }
        else
        {
            if(_loopCloser->config()._isPgoIncremental)
                _loopCloser->submapGraphOptimize();
            else//while gnss data
                _loopCloser->correctSubmapLoop();
        }

        updateLoopStatus(_stateIkfom);
        if(!_loopCloser->config()._isFramewisePGO)
        {
            if(!_loopCloser->config()._isPgoIncremental)
            {
                std::cout<<"Correct frame blocks size:"<<_loopCloser->getFrameBlocks().size()<<std::endl;

                const Matrix4ds& relPoses=_loopCloser->getRelPoses();
                CloudBlockPtrs submaps=_loopCloser->getSubMapBlocks();
//    PointCloudXYZI::Ptr pgoCloudPtr(new PointCloudXYZI());
//    PointCloudXYZI::Ptr trajCloud(new PointCloudXYZI());
                for(auto& submap:submaps)
                {
                    const int& firstIdx=submap->_uniqueId;
                    const int& lastIdx=submap->_lastFrameId;

                    for(int i=firstIdx;i<=lastIdx;i++)
                    {
                        CloudBlockPtr frameBlock=_loopCloser->getFrameBlock(i);

                        std::vector<Eigen::Matrix4d> relIMUPoses;
                        for(auto& imuData:frameBlock->_imuDataList)
                        {
                            Eigen::Matrix4d imuPose=Eigen::Matrix4d::Identity();
                            imuPose.block<3,3>(0,0)=imuData._orientation.toRotationMatrix();
                            imuPose.block<3,1>(0,3)=imuData._position;
                            auto relPose=frameBlock->_poseLo.inverse()*imuPose; //Tli=Tlw*Twi
                            relIMUPoses.emplace_back(relPose);
                        }

                        if(i==firstIdx)
                            frameBlock->_poseLo=submap->_poseLo;
                        else
                            frameBlock->_poseLo=_loopCloser->getFrameBlock(i-1)->_poseLo*relPoses[i-1];

                        frameBlock->freeAll();
                        //correct imu poses by relposes
                        int imuPoseSize=frameBlock->_imuDataList.size();
                        for(int i=0;i<imuPoseSize;i++)
                        {
                            Eigen::Matrix4d imuPose=frameBlock->_poseLo*relIMUPoses[i];
                            frameBlock->_imuDataList[i]._orientation = imuPose.block<3,3>(0,0);
                            frameBlock->_imuDataList[i]._position = imuPose.block<3,1>(0,3);
                        }
//            PointCloudXYZI::Ptr frameCloud(new PointCloudXYZI());
//            pcl::io::loadPCDFile(frameBlock->_pcdFilePath, *frameBlock->_pcRaw);
//            pcl::transformPointCloud(*frameBlock->_pcRaw, *frameCloud, frameBlock->_poseLo);//pw=Tw1*p1
//            *pgoCloudPtr+=*frameCloud;

//            PointType trajPt;
//            trajPt.x = frameBlock->_poseLo(0, 3);
//            trajPt.y = frameBlock->_poseLo(1, 3);
//            trajPt.z = frameBlock->_poseLo(2, 3);
//            trajCloud->push_back(trajPt) ;
                    }
//        pcl::PCDWriter pcdWriter;
//        pcdWriter.writeBinary(string(ROOT_DIR) + "PCD/pgoFrameCloud"+std::to_string(submap->_uniqueId)+".pcd", *pgoCloudPtr);
//        pgoCloudPtr->clear();
                }

                auto curSubmap=_loopCloser->getCurSubmap();
                const int& firstIdx=curSubmap->_uniqueId;
                const int& lastIdx=curSubmap->_lastFrameId;
                for(int i=firstIdx;i<=lastIdx;i++)
                {
                    CloudBlockPtr frameBlock=_loopCloser->getFrameBlock(i);
                    frameBlock->_poseLo=_loopCloser->getFrameBlock(i-1)->_poseLo*relPoses[i-1];
                }
            }
        }

        // pcl::PCDWriter pcdWriter;
        //pcdWriter.writeBinary(string(ROOT_DIR) + "PCD/frameTrajCloud.pcd", *trajCloud);
        //pcdWriter.writeBinary(string(ROOT_DIR) + "PCD/pgoFrameCloud.pcd", *pgoCloudPtr);
        std::cout<<"Loop correct done!"<<std::endl;
    }

    _loopCloser->getProcessLock().unlock();
}

void System::saveMap(bool isForced)
{
    static pcl::PCDWriter pcdWriter;
//    static std::ofstream evoTrajOfs = std::ofstream(string(ROOT_DIR) + "/result/evoTraj", std::ios::trunc | std::ios::in);
//    evoTrajOfs<< std::fixed << std::setprecision(6)<<_measures.lidar_end_time<<" "<<_stateIkfom.pos.x() << " " << _stateIkfom.pos.y() << " " << _stateIkfom.pos.z()<<std::endl;

    V3D twlAligned=_rotAlign*_twl;
    PointType curPos;
    curPos.x = twlAligned.x();
    curPos.y = twlAligned.y();
    curPos.z = twlAligned.z();
    _trajCloud->push_back(curPos);

    if (isForced)
    {
        if(!_denseCloudMap->empty()&&_config._pcdSaveInterval>0)
        {
            string mapSavPath(string(ROOT_DIR) + "PCD/map_" + to_string(_pcdIndex) + string(".pcd"));
            pcdWriter.writeBinary(mapSavPath, *_denseCloudMap);
            string trajSavPath(string(ROOT_DIR) + "PCD/traj_" + to_string(_pcdIndex) + string(".pcd"));
            pcdWriter.writeBinary(trajSavPath, *_trajCloud);
            _denseCloudMap->clear();
            _trajCloud->clear();
            _pcdIndex++;
        }
    }
    else if (_config._pcdSaveInterval <= 0)
    {
        if (_frameId % 100 == 0)
        {
            string mapSavPath(string(ROOT_DIR) + "PCD/all.pcd");
            pcdWriter.writeBinary(mapSavPath, *_denseCloudMap);
            string trajSavPath(string(ROOT_DIR) + "PCD/traj.pcd");
            pcdWriter.writeBinary(trajSavPath, *_trajCloud);
            cout << "Current map saved to " << mapSavPath << endl;
        }
    }
    else if (_frameId % _config._pcdSaveInterval == 0)
    {
        string mapSavPath(string(ROOT_DIR) + "PCD/map_" + to_string(_pcdIndex) + string(".pcd"));
        pcdWriter.writeBinary(mapSavPath, *_denseCloudMap);

        string trajSavPath(string(ROOT_DIR) + "PCD/traj_" + to_string(_pcdIndex) + string(".pcd"));
        pcdWriter.writeBinary(trajSavPath, *_trajCloud);

//        string pcdPath(string(ROOT_DIR) + "PCD/frame" + to_string(_pcdIndex) + string(".pcd"));
//        transCloud(_localCloudPtr, _localCloudPtr, _stateIkfom.offset_R_L_I.matrix(), _stateIkfom.offset_T_L_I.matrix());
//        pcdWriter.writeBinary(pcdPath, *_localCloudPtr);
//
//        Eigen::Matrix4d curPose = Eigen::Matrix4d::Identity();
//        curPose.block<3, 3>(0, 0) = _stateIkfom.rot.matrix();
//        curPose.block<3, 1>(0, 3) = _stateIkfom.pos.matrix();
//        static std::ofstream fPose =std::ofstream(string(ROOT_DIR)+"PCD/pose.json", std::ios::trunc | std::ios::in);
//        fPose << setprecision(6) << std::fixed;
//        Eigen::Quaterniond quat=Eigen::Quaterniond (_stateIkfom.rot.matrix());
//        fPose<<_stateIkfom.pos.x()<<" "<<_stateIkfom.pos.y()<<" "<<_stateIkfom.pos.z()<<" "<<quat.w()<<" "<<quat.x()<<" "<<quat.y()<<" "<<quat.z()<<std::endl;
//        fPose<<curPose(0,0)<<","<<curPose(0,1)<<","<<curPose(0,2)<<","<<curPose(0,3)<<std::endl;
//        fPose<<curPose(1,0)<<","<<curPose(1,1)<<","<<curPose(1,2)<<","<<curPose(1,3)<<std::endl;
//        fPose<<curPose(2,0)<<","<<curPose(2,1)<<","<<curPose(2,2)<<","<<curPose(2,3)<<std::endl;
//        fPose<<0<<","<<0<<","<<0<<","<<_measures.lidar_end_time<<std::endl;

//        static std::ofstream gravOfs = std::ofstream(string(ROOT_DIR) + "/PCD/grav", std::ios::trunc | std::ios::in);
//        gravOfs<< std::fixed << std::setprecision(6)<<_stateIkfom.grav<<std::endl;

        _denseCloudMap->clear();
        _trajCloud->clear();
        _pcdIndex++;
        cout << "Current map saved to " << mapSavPath << endl;
    }
}


void System::voxelFilter(pcl::PointCloud<PointType>::Ptr &cloudIn, pcl::PointCloud<PointType>::Ptr &cloudOut, float gridSize)
{
    pcl::VoxelGrid<PointType> voxGrid;
    voxGrid.setLeafSize(gridSize, gridSize, gridSize);
    voxGrid.setInputCloud(cloudIn);
    voxGrid.filter(*cloudOut);
}

