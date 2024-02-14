///|/ Copyright (c) PR 2022 - 2023 Oleksandra Iushchenko @YuSanka, David Kocík @kocikdav, Vojtěch Bubník @bubnikv, Enrico Turri @enricoturri1966, Lukáš Matěna @lukasmatena
///|/ Copyright (c) 2022 KARBOWSKI Piotr
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "AppUpdater.hpp"

#include <atomic>
#include <thread>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/convert.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <curl/curl.h>

#include "slic3r/GUI/format.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/Utils/Http.hpp"

#include "libslic3r/Utils.hpp"

#ifdef _WIN32
#include <shellapi.h>
#include <Shlobj_core.h>
#include <windows.h>
#include <KnownFolders.h>
#include <shlobj.h>
#endif // _WIN32


namespace Slic3r {

namespace {
	
}

} //namespace Slic3r 
