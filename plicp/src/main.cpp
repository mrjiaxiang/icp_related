//
// Created by melody on 2022/9/13.
//

#include <iostream>
#include <fstream>
#include "plicp_debug.h"
#include "Solve.h"


using namespace std;

void SplitString(const string &ori_str,vector<string> &v_str,const string &flag){
    string::size_type pos1,pos2;
    pos2=ori_str.find(flag);
    pos1=0;
    while (string::npos!=pos2){
        v_str.push_back(ori_str.substr(pos1,pos2-pos1));
        pos1=pos2+flag.size();
        pos2=ori_str.find(flag,pos1);
    }
    if (pos1!=ori_str.length()){
        v_str.push_back(ori_str.substr(pos1));
    }
}

void ReadData(const string &path,std::vector<LidarDataInfo> &scan){
    std::ifstream ifs(path,std::ios::in);
    if (ifs.fail()){
        std::cout<<"open file fail"<<std::endl;
        return;
    }
    string line_str;
    LidarDataInfo tmp_laser_data;
    while (getline(ifs,line_str)){
        if(line_str==""){
            continue;
        }
        std::vector<string> tmp_data;
        SplitString(line_str,tmp_data,",");
        tmp_laser_data.timestamp=std::strtoull(tmp_data[0].c_str(), nullptr,0);
        tmp_laser_data.odo_pose.x=std::strtof(tmp_data[1].c_str(), nullptr);
        tmp_laser_data.odo_pose.y=std::strtof(tmp_data[2].c_str(), nullptr);
        tmp_laser_data.odo_pose.SetPhi(std::strtof(tmp_data[3].c_str(), nullptr));
        for (int i=4;i<tmp_data.size();){
            LaserLine laser_line;
            laser_line.start_x=std::strtof(tmp_data[i].c_str(), nullptr);
            laser_line.start_y=std::strtof(tmp_data[i+1].c_str(), nullptr);
            laser_line.end_x=std::strtof(tmp_data[i+2].c_str(), nullptr);
            laser_line.end_y=std::strtof(tmp_data[i+3].c_str(), nullptr);
            laser_line.end_edg=std::strtof(tmp_data[i+4].c_str(), nullptr);
            laser_line.end_dis=std::strtof(tmp_data[i+5].c_str(), nullptr);
            tmp_laser_data.laser_lines.push_back(laser_line);
            i+=6;
        }
        scan.push_back(tmp_laser_data);
        tmp_laser_data.laser_lines.clear();
    }

}

int main(){
    string data_path="/home/melody/plicp/lidar_data.txt";
    std::vector<LidarDataInfo> scan;
    ReadData(data_path,scan);
    vector<Pose2D> scan_odom;
    PLICPDebug plicpDebug;
    for(int i=0;i<scan.size();i++){
        plicpDebug.LaserScanCallback(scan[i]);
    }
    Solver solver;
    double data[3]={0,0,1};
    solver.Calib(plicpDebug.odom_and_scanOdom,data);

    return 0;
}