#include <string>

class ScanConfig{
public:
    ScanConfig();
    ~ScanConfig();

    void LoadConfig();
    void SaveConfig();

private:
    ScanConfig(const ScanConfig& copy);
    ScanConfig& operator=(const ScanConfig& rhs);
    const std::string filename;
};
