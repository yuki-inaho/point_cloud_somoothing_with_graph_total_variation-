#include "ParameterManager.hpp"


void ParameterManager::ReadParameterFile(std::string filename) {

    _data  = toml::parse(filename);

}

int ParameterManager::ReadIntData(std::string table ,std::string key){

    const auto tab = toml::get<toml::Table>(_data.at(table));
    int read_data = toml::get<int>(tab.at(key));

    return read_data;
}

bool ParameterManager::ReadBoolData(std::string table, std::string key){

    const auto tab = toml::get<toml::Table>(_data.at(table));
    bool read_data = toml::get<bool>(tab.at(key));

    return read_data;
}

float ParameterManager::ReadFloatData(std::string table, std::string key){

    const auto tab = toml::get<toml::Table>(_data.at(table));
    float read_data = toml::get<float>(tab.at(key));

    return read_data;
}

double ParameterManager::ReadDoubleData(std::string table, std::string key){

    const auto tab = toml::get<toml::Table>(_data.at(table));
    double read_data = toml::get<double>(tab.at(key));

    return read_data;
}

std::string ParameterManager::ReadStringData(std::string table, std::string key){

    const auto tab = toml::get<toml::Table>(_data.at(table));
    std::string read_data = toml::get<std::string>(tab.at(key));

    return read_data;
}