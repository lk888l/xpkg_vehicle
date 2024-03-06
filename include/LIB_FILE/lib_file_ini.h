#ifndef FUNC_FILE_INI_H
#define FUNC_FILE_INI_H

#include <cstdio>
#include <string>
#include <map>


using namespace std;

typedef map<std::string,std::string> KEYMAP;//子键索引 子键值
typedef map<std::string,KEYMAP> MAINKEYMAP;//主键索引 主键值

class LibFileIni
{
public:
    LibFileIni();
    static LibFileIni& GetLibFileIni();
    bool OpenFile(const char* pathName);
    string GetKeyValue(const char* section, const char* key);
    int GetKeyInt(const char* section, const char* key);
    double GetKeyDouble(const char* section, const char* key);
    string ToHexStr(int num);
private:
    FILE *m_fp;
    char  m_key_value[256];
    MAINKEYMAP m_key_map_main;
};

#endif // FUNC_FILE_INI_H
