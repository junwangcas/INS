//
// Created by junwangcas on 28/09/17.
//
#include "struct.h"
#include "optimizer.h"
#include <string>
#include <Eigen/Dense>
#include <fstream>
#include <iterator>
#include <iostream>

using namespace std;
using namespace Eigen;
string nsFileNode = "../../../data/node_init.txt";
string nsFileEdge = "../../../data/edge.txt";
string nfileNodeWrite = "../../../data/node_opti.txt";


void ReadNode(string nfileNode,vector< Vector8d>& variable_list){
    ifstream nfilehand(nfileNode.c_str());
    if (!nfilehand){
        std::cout<<"read node file open failed\n";
        exit(1);
    }
    string line;
    while (getline(nfilehand,line)){
        // line to vector double;
        istringstream nstrline(line);
        vector<double> variable_data;
        std::copy(std::istream_iterator<double>(nstrline),
                  std::istream_iterator<double>(),
                  std::back_inserter(variable_data));
        // read variables;
        if (variable_data[1] == 1){// imu variable
            Vector8d variable;
            variable << variable_data[0],variable_data[1],
                    variable_data[2],variable_data[3],variable_data[4],variable_data[5],variable_data[6],variable_data[7];
            variable_list.push_back(variable);
        }else if(variable_data[1] == 2) {// R variable
            Vector8d variable;
            variable << variable_data[0], variable_data[1], variable_data[2], variable_data[3], variable_data[4], -99, -99, -99;
            variable_list.push_back(variable);
        }else if (variable_data[1] == 3){// T variable
            Vector8d variable;
            variable << variable_data[0],variable_data[1],variable_data[2],variable_data[3],variable_data[4],-99,-99,-99;
            variable_list.push_back(variable);
        }else if (variable_data[1] == 4){// Velocity variable
            Vector8d variable;
            variable << variable_data[0],variable_data[1],variable_data[2],variable_data[3],variable_data[4],-99,-99,-99;
            variable_list.push_back(variable);
        }else{
            std::cout<<"readingdata: some data reading error\n";
        }
    }
}

void ReadEdge(string nfileEdge,vector< Vector9d>& nEdgeLst){
    ifstream nfilehand(nfileEdge.c_str());
    if (!nfilehand){
        std::cout<<"read edge file open failed\n";
        exit(1);
    }
    string line;
    while (getline(nfilehand,line)){
        // line to vector double;
        istringstream nstrline(line);
        vector<double> ndata;
        std::copy(std::istream_iterator<double>(nstrline),
                  std::istream_iterator<double>(),
                  std::back_inserter(ndata));
        if (ndata[1] == 1){// gps observe
            Vector9d edge;
            edge << ndata[0],ndata[1],ndata[2],ndata[3],ndata[4],ndata[5],-99,-99,-99;
            nEdgeLst.push_back(edge);
        }else if (ndata[1] == 2){//car velocity
            Vector9d edge;
            edge << ndata[0],ndata[1],ndata[2],ndata[3],ndata[4],ndata[5],ndata[6],-99,-99;
            nEdgeLst.push_back(edge);
        }else if (ndata[1] == 3){//acc
            Vector9d edge;
            edge << ndata[0],ndata[1],ndata[2],ndata[3],ndata[4],ndata[5],ndata[6],ndata[7],ndata[8];
            nEdgeLst.push_back(edge);
        }else if (ndata[1] == 4) {//gyro
            Vector9d edge;
            edge << ndata[0], ndata[1], ndata[2], ndata[3], ndata[4], ndata[5], ndata[6], -99, -99;
            nEdgeLst.push_back(edge);
        }else if (ndata[1] == 5){//motion model
            Vector9d edge;
            edge << ndata[0],ndata[1],ndata[2],ndata[3],ndata[4],-99,-99,-99,-99;
            nEdgeLst.push_back(edge);
        }else{
            std::cout<<"readingdata: some edges data reading error\n";
        }
    }
}

void WriteNode(vector< Vector8d> variable_list){
    ofstream nfilehand(nfileNodeWrite.c_str());
    if (!nfilehand){
        std::cout<<"write node file open error \n";
        exit(0);
    }
    for (int i=0;i<variable_list.size();i++){
        Vector8d variable_val = variable_list[i];
        // imu bias variable;
        if (variable_val[0] == 1){
            Vector6d imu_variable = variable_val.block<6,1>(2,0);
            nfilehand << 1 <<" "<<imu_variable.transpose()<<"\n";
        }else if (variable_val[1] == 2){
            // R variables;
            Vector3d R_variable = variable_val.block<3,1>(2,0);
            nfilehand << 2 <<" "<< R_variable.transpose()<<"\n";
        }else if (variable_val[1] == 3){
            // T variables;
            Vector3d T_variable = variable_val.block<3,1>(2,0);
            nfilehand << 3 <<" "<<T_variable.transpose()<<"\n";
        }else if (variable_val[1] == 4){
            // velocity variables;
            Vector3d Velocity_variable = variable_val.block<3,1>(2,0);
            nfilehand<<4<<" "<<Velocity_variable.transpose()<<"\n";
        }
    }
}
int main(){
    // 1：首先读取node文件，两种node，pose的node保存在vector《Vector8d》中。
    // map point 保存在vector《Vector4D》中。
    vector< Vector8d> variable_list;
    ReadNode(nsFileNode,variable_list);

    // 2： 然后读取edge的文件，只有一种edge，保存在vector《Vector13D》 Vector13D这种结构需要自己定义一下。
    vector< Vector9d> nEdgeLst;
    ReadEdge(nsFileEdge,nEdgeLst);
    // 3: 将这三个参数传递到优化函数中去，返回优化之后的pose的数据，以及点的数据。然后写到文件中。
    //Optimizer nOptimizer;
    //nOptimizer.DoOptimize(nPoseLst,nPtsLst,nEdgeLst);
    optimizer my_optimizer;
    my_optimizer.run_optimize(variable_list,nEdgeLst);

    // 将结果保存起来
    WriteNode(variable_list);
    return 1;
}