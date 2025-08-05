/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ctime>
#include<sstream>
#include<map>
#include <sys/stat.h>  // For mkdir


#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

bool LoadCameraIntrinsics(const string &strIntrinsicsPath, map<long long, std::vector<float>> &mapCamIntrinsics);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_inertial_dmt path_to_vocabulary path_to_settings path_to_dmt_scan_folder_1 ... path_to_dmt_scan_folder_N output_folder" << endl;
        return 1;
    }

    const int num_seq = argc - 4;
    cout << "num_seq = " << num_seq << endl;

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector< map<long long, std::vector<float>> > vMapCamIntrinsics; // CUSTOM
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    vMapCamIntrinsics.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    bool bUseIntrinsicsFile = true;
    bool bUseIMU = false;

    string output_folder = argv[argc - 1];
    cout << "Output folder: " << output_folder << endl;
    if (mkdir(output_folder.c_str(), 0777) != 0) {
        if (errno != EEXIST) {  // Ignore error if directory already exists
            cerr << "Failed to create output directory: " << output_folder << endl;
            return 1;
        }
    }

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        string folder_name = string(argv[seq + 3]);
        cout << "DMT scan folder: " << folder_name << endl;

        // Load camera intrinsics if provided
        if (bUseIntrinsicsFile) {
            string intrinsicsFile = folder_name + "/CameraIntrinsics.csv";
            if (!LoadCameraIntrinsics(intrinsicsFile, vMapCamIntrinsics[seq])) {
                cerr << "Failed to load camera intrinsics from file: " << intrinsicsFile << endl;
                return 1;
            }
            cout << "Loaded " << vMapCamIntrinsics[seq].size() << " camera intrinsic parameters" << endl;
        }
        else {
            cout << "No frame-by-frame camera intrinsics file. Using values from settings YAML." << endl;
        }

        cout << "Loading images for sequence " << seq << "..." << endl;

        string pathSeq(argv[seq + 3]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";
        string pathTimeStamps = pathSeq + "/Timestamps.txt";

        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "..." << endl;
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);

        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence " << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool bUseViewer = true;
    ORB_SLAM3::System SLAM(
        argv[1],argv[2],
        bUseIMU ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR,
        bUseViewer, 0, "", output_folder);

    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Main loop
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            //cout<<"SLAM.TrackMonocular(im,tframe,vImuMeas)  "<<nImages[seq]<<endl;
            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);

            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #endif
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
            {
                // cout << "t_cam " << tframe << endl;

                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #endif

            // Pass the image to the SLAM system            
            if (bUseIntrinsicsFile) {
                long long frametimeNs = static_cast<long long>(tframe * 1e9);
                auto frameIntrinsics = vMapCamIntrinsics[seq][frametimeNs];
                SLAM.TrackMonocular(
                    im, tframe,
                    bUseIMU ? vImuMeas : vector<ORB_SLAM3::IMU::Point>(),
                    "", frameIntrinsics
                );
            }
            else {
                SLAM.TrackMonocular(
                    im, tframe,
                    bUseIMU ? vImuMeas : vector<ORB_SLAM3::IMU::Point>(),
                    "", {}
                );
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];
            if(ttrack<T)
                //usleep((T-ttrack)*1e6); // 1e6
                usleep(5000);
        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    const string kf_file = output_folder + "/KeyFrameTrajectory.txt";
    const string f_file = output_folder + "/CameraTrajectory.txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);

    if (!fTimes.is_open()) {
        cerr << "Failed to open timestamps file: " << strPathTimes << endl;
        return;
    }

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    cout << "Loading IMU path:" << strImuPath << endl;
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    if (!fImu.is_open()) {
        cerr << "Failed to open IMU file: " << strImuPath << endl;
        return;
    }

    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}

bool LoadCameraIntrinsics(const string &strIntrinsicsPath, map<long long, std::vector<float>> &mapCamIntrinsics)
{
    cout << "Loading Camera Intrinsics path:" << strIntrinsicsPath << endl;
    ifstream fIntrinsics;
    fIntrinsics.open(strIntrinsicsPath.c_str());
    
    if (!fIntrinsics.is_open()) {
        cerr << "Failed to open camera intrinsics file: " << strIntrinsicsPath << endl;
        return false;
    }
    
    string header;
    getline(fIntrinsics, header);
    
    while(!fIntrinsics.eof())
    {
        string s;
        getline(fIntrinsics, s);
        if (s.empty() || s[0] == '#')
            continue;
            
        stringstream ss(s);
        string item;
        
        // Format expected: timestamp,fx,fy,cx,cy
        double timestamp;
        float fx, fy, cx, cy;
        
        // Parse timestamp
        getline(ss, item, ',');
        timestamp = stod(item); // Already in seconds
        long long timestampNs = static_cast<long long>(timestamp * 1e9);
        
        // Parse fx
        getline(ss, item, ',');
        fx = stof(item);
        
        // Parse fy
        getline(ss, item, ',');
        fy = stof(item);
        
        // Parse cx
        getline(ss, item, ',');
        cx = stof(item);
        
        // Parse cy
        getline(ss, item, ',');
        cy = stof(item);
        
        // Store in map
        mapCamIntrinsics[timestampNs] = {fx, fy, cx, cy};
    }
    
    return !mapCamIntrinsics.empty();
}

