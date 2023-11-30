///|/ Copyright (c) Prusa Research 2022 David Kocík @kocikdav
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_AppUpdate_hpp_
#define slic3r_AppUpdate_hpp_

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include "libslic3r/Utils.hpp"
#include "wx/event.h"

//class boost::filesystem::path;

namespace Slic3r {

#ifdef __APPLE__
// implmented at MacUtils.mm
std::string get_downloads_path_mac();
#endif //__APPLE__

class AppUpdater
{
};

} //namespace Slic3r 
#endif
