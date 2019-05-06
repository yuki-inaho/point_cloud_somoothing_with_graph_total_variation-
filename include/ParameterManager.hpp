#pragma once
#include "toml.hpp"
#include <iostream>

class ParameterManager{

private:

    toml::table _data;

public:

    /// Constructor
    ParameterManager(std::string filename){

        ReadParameterFile(filename);
    };

    /// Destructor
    ~ParameterManager(){

    };

    ///　パラメータファイル読み込み
    void ReadParameterFile(std::string filename);

    /// 違いは型だけなので後で修正
    int ReadIntData(std::string table, std::string key);
    bool ReadBoolData(std::string table, std::string key);
    float ReadFloatData(std::string table, std::string key);
    double ReadDoubleData(std::string table, std::string key);
    std::string ReadStringData(std::string table, std::string key);

};