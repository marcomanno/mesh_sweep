
#include "utils.hxx"

#include <filesystem>

std::string out_file(const char* _flnm)
{
  namespace fs = std::filesystem;
  if (!fs::is_directory(OUTDIR) || !fs::exists(OUTDIR))
    fs::create_directory(OUTDIR); // create _dir folder
  return std::string(OUTDIR"/") + _flnm;
}
