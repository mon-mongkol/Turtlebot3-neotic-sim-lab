#include <string>
#include <vector>
#include <sstream>
#include <iostream>
/*
std::string split implementation by using delimeter as a character.
*/
// std::vector<std::string> split(std::string strToSplit, char delimeter)
// {
//     std::stringstream ss(strToSplit);
//     std::string item;
//     std::vector<std::string> splittedStrings;
//     while (std::getline(ss, item, delimeter))
//     {
//         splittedStrings.push_back(item);
//     }
//     return splittedStrings;
// }
/*
std::string split implementation by using delimeter as an another string
*/
std::vector<std::string> split(std::string stringToBeSplitted, std::string delimeter)
{
    std::vector<std::string> splittedString;
    int startIndex = 0;
    int  endIndex = 0;
    while( (endIndex = stringToBeSplitted.find(delimeter, startIndex)) < stringToBeSplitted.size() )
    {
        std::string val = stringToBeSplitted.substr(startIndex, endIndex - startIndex);
        splittedString.push_back(val);
        startIndex = endIndex + delimeter.size();
    }
    if(startIndex < stringToBeSplitted.size())
    {
        std::string val = stringToBeSplitted.substr(startIndex);
        splittedString.push_back(val);
    }
    return splittedString;
}
int main()
{
    std::string str = "ROHM020020220004A";
    // std::string data;
    // Spliting the string by ''
    std::vector<std::string> splittedStrings = split(str, "ROHM020020220");
    for(int i = 0; i < splittedStrings.size() ; i++)
        splittedStrings[i];
        // std::cout<<splittedStrings[i]<<std::endl;
        std::string splittedStrings_data = splittedStrings[1];
        // std::cout<<data<<std::endl;
    std::vector<std::string> splittedStrings_2 = split(splittedStrings_data, "A");
    for(int i = 0; i < splittedStrings_2.size() ; i++)
        std::cout<<splittedStrings_2[i]<<std::endl; 

    return 0;
}