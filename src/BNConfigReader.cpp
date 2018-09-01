#include "includes/BNConfigReader.h"

BNConfigReader::BNConfigReader(std::string fileName):
m_configFileName(fileName)
{
    LoadConfig();
}

void BNConfigReader::LoadConfig()
{
    if (m_configFileName=="")
    {
        BNUtils::BNLogger(BNUtils::logERROR) << "Configuration file not found";
        throw "Configuration file path not set";
    }

    std::ifstream configFile(m_configFileName);

    if(!configFile)
    {
        throw "Error opening configuration file";
    }
    std::string line;
    std::cout << "This is called" << std::endl;
    while(std::getline(configFile,line))
    {
        std::istringstream iss(line);
        std::string key, value;
        if(!(iss >> key >> value))
        {
            break;
        }

        m_configurations.insert({key,value});
    }

}

std::unordered_map<std::string,std::string> BNConfigReader::GetConfig()
{
    return m_configurations;
}