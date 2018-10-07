//
// Created by junwangcas on 15/07/17.
//

#include "paramReader.h"
namespace LOCALMAP {
// 参数读取类
    paramReader::paramReader( string  filename)
        {
            //filename = mfilename;
            ifstream fin(filename.c_str());
            if (!fin) {
                cerr << "parameter file does not exist." << endl;
                return;
            }
            while (!fin.eof()) {
                string str;
                getline(fin, str);
                if (str[0] == '#') {
                    // 以‘＃’开头的是注释
                    continue;
                }

                int pos = str.find("=");
                if (pos == -1)
                    continue;
                string key = str.substr(0, pos);
                string value = str.substr(pos + 1, str.length());
                data[key] = value;

                if (!fin.good())
                    break;
            }
        }

    string paramReader::getData(string key) {
            map<string, string>::iterator iter = data.find(key);
            if (iter == data.end()) {
                cerr << "Parameter name " << key << " not found!" << endl;
                return string("NOT_FOUND");
            }
            return iter->second;
        }
}