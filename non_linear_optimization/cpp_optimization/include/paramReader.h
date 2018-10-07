//
// Created by junwangcas on 15/07/17.
//

#ifndef PLEXTRACT_PARAMREADER_H
#define PLEXTRACT_PARAMREADER_H

#include <iostream>
#include <fstream>
#include <string>
#include <map>

using namespace std;
namespace LOCALMAP {
    class paramReader {
    public:
        paramReader( string filename= "parameters.txt");
        string getData(string key);
    public:
        map <string, string> data;
        //string mfilename = "/home/junwangcas/code/ORB_SLAM2_V2/parameters.txt";
    };
}


#endif //PLEXTRACT_PARAMREADER_H
