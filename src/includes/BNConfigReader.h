#include "common.h"
#include <fstream>
#ifndef BN_CONFIG_READER_H
#define BN_CONFIG_READER_H


class BNConfigReader
{
public:
    BNConfigReader(std::string filePath);
    std::unordered_map<std::string,std::string> GetConfig();
private:
    std::string m_configFileName;
    void LoadConfig();
    std::unordered_map<std::string,std::string> m_configurations;
};

#endif