
#include "utils.hxx"

#include <filesystem>
#include <map>

std::string out_file(const char* _flnm)
{
  static std::map<std::string, int> name_map;
  namespace fs = std::filesystem;
  if (!fs::is_directory(OUTDIR) || !fs::exists(OUTDIR))
    fs::create_directory(OUTDIR); // create _dir folder
  auto& num = name_map[std::string(_flnm)];
  auto str_num = std::to_string(num++);
  if (str_num.size() < 4)
    str_num.insert(0, 4 - str_num.size(), '0');
  return std::string(OUTDIR"/") + str_num + '_' + _flnm;
}
