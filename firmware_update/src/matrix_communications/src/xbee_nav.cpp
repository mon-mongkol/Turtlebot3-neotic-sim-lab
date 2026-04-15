#include <iostream>
#include <string>
#include <sstream>
 
 // for convert String 8 byte (Hex 4 byte 0x00, 0x00, 0x00, 0x00)
 float HexString_float(std::string value, float divider)
 {
     unsigned int i;
    std::istringstream iss(value);
    iss >> std::hex >> i;
    auto j = static_cast<int>(i);
    float b = j;
    b = b/divider;
    std::cout << b << std::endl;    // 1000
    return b;
 }
 
 // get string between 2 delimer 
 std::string split_string(std::string string_cmd, std::string begin_pos, std::string end_pos)
 {
    size_t pos = 0;
    std::string token;
    while ((pos = string_cmd.find(begin_pos)) != std::string::npos) {
        string_cmd.erase(0, pos + begin_pos.length());
        string_cmd.erase(string_cmd.find(end_pos));
    }
    // std::cout << string_cmd << std::endl;
    return string_cmd.c_str();
 }
 
 
int main()
{
    std::string string_cmd = "$R0001C0002PXFFFFD8BCPYFFFFEF65OR000082#";
   
    std::string stringPosX = "PX";
    std::string stringPosY = "PY";
    std::string stringPosOR = "OR";
    std::string stringEnd = "#";
    
    std::string px = split_string(string_cmd, stringPosX, stringPosY);
    std::string py = split_string(string_cmd, stringPosY, stringPosOR);
    std::string theta = split_string(string_cmd, stringPosOR, stringEnd);
    
    float x_ = HexString_float(px, float(100.00));
    float y_ = HexString_float(py, float(100.00));
    float theata_ = HexString_float(theta, float(1.00));
 
    return 0;
}