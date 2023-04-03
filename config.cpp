#include "config.h"
using namespace tinyxml2;

Config::Config(){
    precision = 0.0000001;
    agent_size = 0.353553;
    timelimit = 60;
    connectedness = 3;
    k_robustness = 3;
}

void Config::get_config(const char* filename){
     std::stringstream stream;
    XMLDocument doc;
    if (doc.LoadFile(filename) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening Config XML file!" << std::endl;
        return;
    }

    XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No 'root' element found in XML file."<<std::endl;
        return;
    }

    XMLElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if(!algorithm)
    {
        std::cout << "No 'algorithm' element found in XML file."<<std::endl;
        return;
    }

    XMLElement *element = algorithm->FirstChildElement("precision");
    if (!element)
    {
        std::cout << "Error! No 'precision' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_PRECISION<<"'."<<std::endl;
        precision = CN_PRECISION;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>precision;
        if(precision > 1.0 || precision <= 0)
        {
            std::cout << "Error! Wrong 'precision' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_PRECISION<<"'."<<std::endl;
            precision = CN_PRECISION;
        }
        stream.clear();
        stream.str("");
    }    

    element = algorithm->FirstChildElement("connectedness");
    if (!element)
    {
        std::cout << "Error! No 'connectedness' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_CONNECTEDNESS<<"'."<<std::endl;
        connectedness = CN_CONNECTEDNESS;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>connectedness;
        if(connectedness > 5 || connectedness < 2)
        {
            std::cout << "Error! Wrong 'connectedness' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_CONNECTEDNESS<<"'."<<std::endl;
            connectedness = CN_CONNECTEDNESS;
        }
        stream.clear();
        stream.str("");
    }
    
    element = algorithm->FirstChildElement("agent_size");
    if (!element)
    {
        std::cout << "Error! No 'agent_size' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_AGENT_SIZE<<"'."<<std::endl;
        agent_size = CN_AGENT_SIZE;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>agent_size;
        if(agent_size < 0 || agent_size > 0.5)
        {
            std::cout << "Error! Wrong 'agent_size' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_AGENT_SIZE<<"'."<<std::endl;
            agent_size = CN_AGENT_SIZE;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement("k_robustness"); 
    if (!element)
    {
        std::cout << "Error! No 'k_robustness' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_ROBUSTNESS<<"'."<<std::endl;
        k_robustness = CN_ROBUSTNESS;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>k_robustness;
        if(k_robustness < 0 || k_robustness > 7)
        {
            std::cout << "Error! Wrong 'k_robustness' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_ROBUSTNESS<<"'."<<std::endl;
            k_robustness = CN_ROBUSTNESS;
        }
        stream.clear();
        stream.str("");
    }   

    element = algorithm->FirstChildElement("timelimit");
    if (!element)
    {
        std::cout << "Error! No 'timelimit' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_TIMELIMIT<<"'."<<std::endl;
        timelimit = CN_TIMELIMIT;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>timelimit;
        if(timelimit <= 0)
            timelimit = CN_INFINITY;
        stream.clear();
        stream.str("");
    }
    return;
}

void Config::print() const{
    std::cout<<"precision: "<<precision<<std::endl;
    std::cout<<"agent_size: "<<agent_size<<std::endl;
    std::cout<<"timelimit: "<<timelimit<<std::endl;
    std::cout<<"connectedness: "<<connectedness<<std::endl;
    std::cout<<"k_robustness: "<<k_robustness<<std::endl;
}
