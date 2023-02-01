#include "map.h"

//gets the map using the tinyxml2 library
bool Map::get_map(const char* FileName){

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement* root = nullptr;
    if(doc.LoadFile(FileName) != tinyxml2::XML_SUCCESS){
        std::cout << "Error loading file" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    return get_grid(FileName);
}

bool Map::get_grid(const char* FileName){

    tinyxml2::XMLElement *root = nullptr, *map = nullptr, *element = nullptr, *mapnode = nullptr;
    tinyxml2::XMLDocument doc;

    std::string value;
    std::stringstream stream;
    bool hasGridMem(false), hasGrid(false), hasHeight(false), hasWidth(false);

    if(doc.LoadFile(FileName) != tinyxml2::XML_SUCCESS){
        std::cout << "Error loading file" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if(!root){
        std::cout << "Error, no " << CNS_TAG_ROOT << " tag found" << std::endl;
        return false;
    }
    map = root->FirstChildElement(CNS_TAG_MAP);
    if(!map){
        std::cout << "Error, no " << CNS_TAG_MAP << " tag found" << std::endl;
        return false;
    }
    for(mapnode = map->FirstChildElement(); mapnode; mapnode = mapnode->NextSiblingElement()){
        element = mapnode->ToElement();
        value = element->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        stream.str("");
        stream.clear();
        stream << element->GetText();    

        if(!hasGridMem and hasHeight and hasWidth){
            grid.resize(height);
            for(int i = 0; i < height; i++){
                grid[i].resize(width);
            }
            hasGridMem = true;
        }

        if(value == CNS_TAG_HEIGHT){
            if(hasHeight){
                std::cout << "Error, " << CNS_TAG_HEIGHT << " tag found more than once, using first value" << std::endl;
            }
            else{
                if(!((stream >> height) && (height > 0))){
                    std::cout << "Error, " << CNS_TAG_HEIGHT << " tag has invalid value" << std::endl;
                    std::cout << "Continuing to read file and hoping a correct value is found" << std::endl;
                }
                else{
                    hasHeight = true;
                }
            }
        }
        else if(value == CNS_TAG_WIDTH){
            if(hasWidth){
                std::cout << "Error, " << CNS_TAG_WIDTH << " tag found more than once, using first value" << std::endl;
            }
            else{
                if(!((stream >> width) && (width > 0))){
                    std::cout << "Error, " << CNS_TAG_WIDTH << " tag has invalid value" << std::endl;
                    std::cout << "Continuing to read file and hoping a correct value is found" << std::endl;
                }
                else{
                    hasWidth = true;
                }
            }
        }
        else if(value == CNS_TAG_GRID){
            int grid_i(0), grid_j(0);
            hasGrid = true;
            if(!(hasHeight and hasWidth)){
                std::cout << "Error, " << CNS_TAG_GRID << " tag found before " << CNS_TAG_HEIGHT << " or " << CNS_TAG_WIDTH << " tag" << std::endl;
                return false;
            }
            element = mapnode->FirstChildElement();
            while(grid_i < height){
                if(!element){
                    std::cout << "Error, " << CNS_TAG_GRID << " tag has too few values" << std::endl;
                    return false;
                }
                std::string str = element->GetText();
                std::vector<std::string> elems;
                std::stringstream ss(str);
                std::string item;
                while (std::getline(ss, item, ' '))
                    elems.push_back(item);
                grid_j = 0;
                int val;
                if (elems.size() > 0)
                    for (grid_j = 0; grid_j < width; ++grid_j)
                    {
                        if (grid_j == int(elems.size()))
                            break;
                        stream.str("");
                        stream.clear();
                        stream << elems[grid_j];
                        stream >> val;
                        grid[grid_i][grid_j] = val;
                    }

                if (grid_j != width)
                {
                    std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << grid_i + 1 << " " << CNS_TAG_ROW
                                << std::endl;
                    return false;
                }
                ++grid_i;
                element = element->NextSiblingElement();
            }
        }
    }
    if(!hasGrid){
        std::cout << "Error, no " << CNS_TAG_GRID << " tag found" << std::endl;
        return false;
    }
    size = height * width;
    std::vector<Step> moves;
    valid_moves.resize(size);
    //moves are generated for a k=3 connected neighborhood
    moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1}};
    //iterate through every move in moves, check if they're valid and if they are, create a new Node object with the move data
    //and add it to the valid_moves vector
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            for(int k = 0; k < moves.size(); k++){
                int new_i = i + moves[k].i;
                int new_j = j + moves[k].j;
                if(new_i >= 0 && new_i < height && new_j >= 0 && new_j < width){
                    if(!cell_is_obstacle(new_i, new_j)){
                        valid_moves[i*width+j].push_back(Node((new_i)*width + new_j, 0, 0, new_i, new_j));
                    }
                }
            }
        }
    }
    return true;    
}

bool Map::cell_is_obstacle(int i, int j) const
{
    return (grid[i][j] == CN_OBSTL);
}

void Map::print() const{
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            std::cout << grid[i][j] << " ";
        }
        std::cout << std::endl;
    }
    //print all the valid moves for each cell
    // for(int i = 0; i < height; i++){
    //     for(int j = 0; j < width; j++){
    //         std::cout << "Cell " << i << " " << j << " has " << valid_moves[i*width+j].size() << " valid moves" << std::endl;
    //         for(int k = 0; k < valid_moves[i*width+j].size(); k++){
    //             std::cout << "Move " << k << " is to " << valid_moves[i*width+j][k].i << " " << valid_moves[i*width+j][k].j << std::endl;
    //         }
    //     }
    // }
}
