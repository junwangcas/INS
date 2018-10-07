//
// Created by junwangcas on 28/09/17.
//
#include "struct.h"
#include <string>
#include <Eigen/Dense>
#include <fstream>
#include <iterator>
#include <iostream>

using namespace std;
using namespace Eigen;
string nsFileNode = "../data/node_init.txt";
string nsFileEdge = "../data/edge.txt";
string nfileNodeWrite = "../data/node_opti.txt";


void ReadNode(string nfileNode,vector< Vector8d>& variable_list){
    ifstream nfilehand(nfileNode.c_str());
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

void WriteNode(vector< Vector8d>& nPoseLst,vector< Vector4d>& nPtsLst){
    ofstream nfilehand(nfileNodeWrite.c_str());
    //先写这个pose；
    for (size_t i=0;i<nPoseLst.size();i++){
        Vector8d nPose = nPoseLst[i];
        for (size_t j=0;j<8;j++){
            if (j==0){
                nfilehand<<int(nPose(0))<<" ";
                //fprintf(nfilehand,"%d ",int(nPose(0)));
            }else{
                //fprintf(nfilehand,"%f ",int(nPose(j)));
                nfilehand<<nPose(j)<<" ";
            }
        }
        nfilehand<<"\n";
        //sprintf(nfilehand,"\n");
    }
    //再写map point部分；
    for (size_t i=0;i<nPtsLst.size();i++){
        Vector4d nPts = nPtsLst[i];
        for (size_t j=0;j<4;j++){
            if (j==0){
                nfilehand<<int(nPts(0))<<" ";
            }else{
                nfilehand<<nPts(j)<<" ";
            }
        }
        nfilehand<<"\n";
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

    // 将结果保存起来
    //WriteNode(nPoseLst,nPtsLst);
}