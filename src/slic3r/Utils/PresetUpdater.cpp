///|/ Copyright (c) Prusa Research 2018 - 2023 Vojtěch Bubník @bubnikv, David Kocík @kocikdav, Oleksandra Iushchenko @YuSanka, Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966, Vojtěch Král @vojtechkral
///|/ Copyright (c) 2020 Ondřej Nový @onovy
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PresetUpdater.hpp"

#include <algorithm>
#include <thread>
#include <unordered_map>
#include <ostream>
#include <utility>
#include <stdexcept>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>

#include <wx/app.h>
#include <wx/msgdlg.h>
#include <wx/progdlg.h>

#include "libslic3r/libslic3r.h"
#include "libslic3r/format.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/UpdateDialogs.hpp"
#include "slic3r/GUI/ConfigWizard.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/format.hpp"
#include "slic3r/GUI/NotificationManager.hpp"
#include "slic3r/Config/Version.hpp"
#include "slic3r/Config/Snapshot.hpp"

namespace fs = boost::filesystem;
using Slic3r::GUI::Config::Index;
using Slic3r::GUI::Config::Version;
using Slic3r::GUI::Config::Snapshot;
using Slic3r::GUI::Config::SnapshotDB;



// FIXME: Incompat bundle resolution doesn't deal with inherited user presets


namespace Slic3r {


//static const char *INDEX_FILENAME = "index.idx";
static const char *TMP_EXTENSION = ".download";

namespace {
void copy_file_fix(const fs::path &source, const fs::path &target)
{
	BOOST_LOG_TRIVIAL(debug) << format("PresetUpdater: Copying %1% -> %2%", source, target);
	std::string error_message;
	CopyFileResult cfr = copy_file(source.string(), target.string(), error_message, false);
	if (cfr != CopyFileResult::SUCCESS) {
		BOOST_LOG_TRIVIAL(error) << "Copying failed(" << cfr << "): " << error_message;
		throw Slic3r::CriticalException(GUI::format(
				_L("Copying of file %1% to %2% failed: %3%"),
				source, target, error_message));
	}
	// Permissions should be copied from the source file by copy_file(). We are not sure about the source
	// permissions, let's rewrite them with 644.
	static constexpr const auto perms = fs::owner_read | fs::owner_write | fs::group_read | fs::others_read;
	fs::permissions(target, perms);
}
std::string escape_string_url(const std::string& unescaped)
{
    auto ISUNRESERVED = [](char c){
        bool ISURLPUNTCS =
            c == '-' ||
            c == '.' ||
            c == '_' ||
            c == '~';
        return std::isalpha(c) || ISURLPUNTCS;
    };

    std::string res;
    for (auto& c: unescaped) {
        if (ISUNRESERVED(c)) {
            res += c;
        } else {
            // encode it
            const char hex[] = "0123456789ABCDEF";
            res += '%';
            res += hex[c>>4];
            res += hex[c & 0xf];
        }
    }
    return res;
}
}

wxDEFINE_EVENT(EVT_CONFIG_UPDATER_SYNC_DONE, wxCommandEvent);

struct Update
{
	fs::path source;
	fs::path target;

	Version version;
	std::string vendor;
	std::string changelog_url;

	bool forced_update;
	std::vector<std::string> new_printers;

	Update() {}
	Update(fs::path &&source, fs::path &&target, const Version &version, std::string vendor, std::string changelog_url, bool forced = false, std::vector<std::string> new_printers = {})
		: source(std::move(source))
		, target(std::move(target))
		, version(version)
		, vendor(std::move(vendor))
		, changelog_url(std::move(changelog_url))
		, forced_update(forced)
		, new_printers(std::move(new_printers))
	{}

	void install() const
	{
		copy_file_fix(source, target);
	}

	friend std::ostream& operator<<(std::ostream& os, const Update &self)
	{
		os << "Update(" << self.source.string() << " -> " << self.target.string() << ')';
		return os;
	}
};

struct Incompat
{
	fs::path bundle;
	Version version;
	std::string vendor;

	Incompat(fs::path &&bundle, const Version &version, std::string vendor)
		: bundle(std::move(bundle))
		, version(version)
		, vendor(std::move(vendor))
	{}

	void remove() {
		// Remove the bundle file
		fs::remove(bundle);

		// Look for an installed index and remove it too if any
		const fs::path installed_idx = bundle.replace_extension("idx");
		if (fs::exists(installed_idx)) {
			fs::remove(installed_idx);
		}
	}

	friend std::ostream& operator<<(std::ostream& os , const Incompat &self) {
		os << "Incompat(" << self.bundle.string() << ')';
		return os;
	}
};

struct Updates
{
	std::vector<Incompat> incompats;
	std::vector<Update> updates;
};

struct PresetUpdater::priv
{
	std::vector<Index> index_db;

	bool enabled_version_check;
	bool enabled_config_update;
	std::string version_check_url;

	fs::path cache_path;
	fs::path cache_vendor_path;
	fs::path rsrc_path;
	fs::path vendor_path;

	bool cancel;
	std::thread thread;

	bool has_waiting_updates { false };
	Updates waiting_updates;

	priv();

	void set_download_prefs(const AppConfig *app_config);
	bool get_file(const std::string &url, const fs::path &target_path) const;
	void prune_tmps() const;
	void sync_config(const VendorMap vendors, const std::string& index_archive_url);

	void check_install_indices() const;
	Updates get_config_updates(const Semver& old_slic3r_version) const;
	bool perform_updates(Updates &&updates, bool snapshot = true) const;
	void set_waiting_updates(Updates u);
	// checks existence and downloads resource to cache
	void get_missing_resource(const std::string& vendor, const std::string& filename, const std::string& url) const; 
	// checks existence and downloads resource to vendor or copy from cache to vendor
	void get_or_copy_missing_resource(const std::string& vendor, const std::string& filename, const std::string& url) const;
	void update_index_db();
};

PresetUpdater::priv::priv()
	: cache_path(fs::path(Slic3r::data_dir()) / "cache")
	, cache_vendor_path(cache_path / "vendor")
	, rsrc_path(fs::path(resources_dir()) / "profiles")
	, vendor_path(fs::path(Slic3r::data_dir()) / "vendor")
	, cancel(false)
{
	set_download_prefs(GUI::wxGetApp().app_config);
	// Install indicies from resources. Only installs those that are either missing or older than in resources.
	check_install_indices();
	// Load indices from the cache directory.
	index_db = Index::load_db();
}

void PresetUpdater::priv::update_index_db()
{
	index_db = Index::load_db();
}

// Pull relevant preferences from AppConfig
void PresetUpdater::priv::set_download_prefs(const AppConfig *app_config)
{
	enabled_version_check = app_config->get("notify_release") != "none";
	version_check_url = app_config->version_check_url();
	enabled_config_update = app_config->get_bool("preset_update") && !app_config->legacy_datadir();
}

// Downloads a file (http get operation). Cancels if the Updater is being destroyed.
bool PresetUpdater::priv::get_file(const std::string &url, const fs::path &target_path) const
{
	return false;
}

// Remove leftover paritally downloaded files, if any.
void PresetUpdater::priv::prune_tmps() const
{
    for (auto &dir_entry : boost::filesystem::directory_iterator(cache_path))
		if (is_plain_file(dir_entry) && dir_entry.path().extension() == TMP_EXTENSION) {
			BOOST_LOG_TRIVIAL(debug) << "Cache prune: " << dir_entry.path().string();
			fs::remove(dir_entry.path());
		}
}

void PresetUpdater::priv::get_missing_resource(const std::string& vendor, const std::string& filename, const std::string& url) const
{
}

void PresetUpdater::priv::get_or_copy_missing_resource(const std::string& vendor, const std::string& filename, const std::string& url) const
{
	if (filename.empty() || vendor.empty())
		return;

	std::string escaped_filename = escape_string_url(filename);
	const fs::path file_in_vendor(vendor_path / (vendor + "/" + filename));
	const fs::path file_in_rsrc(rsrc_path / (vendor + "/" + filename));
	const fs::path file_in_cache(cache_path / (vendor + "/" + filename));

	if (fs::exists(file_in_vendor)) { // Already in vendor. No need to do anything.
		BOOST_LOG_TRIVIAL(info) << "Resource " << vendor << " / " << filename << " found in vendor folder. No need to download.";
		return;
	}
	if (fs::exists(file_in_rsrc)) { // In resources dir since installation. No need to do anything.
		BOOST_LOG_TRIVIAL(info) << "Resource " << vendor << " / " << filename << " found in resources folder. No need to download.";
		return;
	}
	if (!fs::exists(file_in_cache)) { // No file to copy. Download it to straight to the vendor dir.
		if (!boost::starts_with(url, "http://files.prusa3d.com/wp-content/uploads/repository/") &&
			!boost::starts_with(url, "https://files.prusa3d.com/wp-content/uploads/repository/"))
		{
			throw Slic3r::CriticalException(GUI::format("URL outside prusa3d.com network: %1%", url));
		}
		BOOST_LOG_TRIVIAL(info) << "Downloading resources missing in cache directory: " << vendor << " / " << filename;

		const auto resource_url = format("%1%%2%%3%", url, url.back() == '/' ? "" : "/", escaped_filename); // vendor should already be in url 

		if (!fs::exists(file_in_vendor.parent_path()))
			fs::create_directory(file_in_vendor.parent_path());

		get_file(resource_url, file_in_vendor);
		return;
	}

	if (!fs::exists(file_in_vendor.parent_path())) // create vendor_name dir in vendor 
		fs::create_directory(file_in_vendor.parent_path());

	BOOST_LOG_TRIVIAL(debug) << "Copiing: " << file_in_cache << " to " << file_in_vendor;
	copy_file_fix(file_in_cache, file_in_vendor);
}

void PresetUpdater::priv::sync_config(const VendorMap vendors, const std::string& index_archive_url)
{
}

// Install indicies from resources. Only installs those that are either missing or older than in resources.
void PresetUpdater::priv::check_install_indices() const
{
	BOOST_LOG_TRIVIAL(info) << "Checking if indices need to be installed from resources...";

    for (auto &dir_entry : boost::filesystem::directory_iterator(rsrc_path))
		if (is_idx_file(dir_entry)) {
			const auto &path = dir_entry.path();
			const auto path_in_cache = cache_path / path.filename();

			if (! fs::exists(path_in_cache)) {
				BOOST_LOG_TRIVIAL(info) << "Install index from resources: " << path.filename();
				copy_file_fix(path, path_in_cache);
			} else {
				Index idx_rsrc, idx_cache;
				idx_rsrc.load(path);
				idx_cache.load(path_in_cache);

				if (idx_cache.version() < idx_rsrc.version()) {
					BOOST_LOG_TRIVIAL(info) << "Update index from resources: " << path.filename();
					copy_file_fix(path, path_in_cache);
				}
			}
		}
}

// Generates a list of bundle updates that are to be performed.
// Version of slic3r that was running the last time and which was read out from PrusaSlicer.ini is provided
// as a parameter.
Updates PresetUpdater::priv::get_config_updates(const Semver &old_slic3r_version) const
{
	Updates updates;

	BOOST_LOG_TRIVIAL(info) << "Checking for cached configuration updates...";

	// Over all indices from the cache directory:
    for (const Index& idx : index_db) {
		auto bundle_path = vendor_path / (idx.vendor() + ".ini");
		auto bundle_path_idx = vendor_path / idx.path().filename();

		if (! fs::exists(bundle_path)) {
			BOOST_LOG_TRIVIAL(info) << format("Confing bundle not installed for vendor %1%, skipping: ", idx.vendor());
			continue;
		}

		// Perform a basic load and check the version of the installed preset bundle.
		VendorProfile vp;
		try {
			vp = VendorProfile::from_ini(bundle_path, false);
		}
		catch (const std::exception& e) {
			BOOST_LOG_TRIVIAL(error) << format("Corrupted profile file for vendor %1% at %2%, message: %3%", idx.vendor(), bundle_path, e.what());
			continue;
		}
		// Getting a recommended version from the latest index, wich may have been downloaded
		// from the internet, or installed / updated from the installation resources.
		auto recommended = idx.recommended();
		if (recommended == idx.end()) {
			BOOST_LOG_TRIVIAL(error) << format("No recommended version for vendor: %1%, invalid index? Giving up.", idx.vendor());
			// XXX: what should be done here?
			continue;
		}

		const auto ver_current = idx.find(vp.config_version);
		const bool ver_current_found = ver_current != idx.end();

		BOOST_LOG_TRIVIAL(debug) << format("Vendor: %1%, version installed: %2%%3%, version cached: %4%",
			vp.name,
			vp.config_version.to_string(),
			(ver_current_found ? "" : " (not found in index!)"),
			recommended->config_version.to_string());

		if (! ver_current_found) {
			// Any published config shall be always found in the latest config index.
			auto message = format("Preset bundle `%1%` version not found in index: %2%", idx.vendor(), vp.config_version.to_string());
			BOOST_LOG_TRIVIAL(error) << message;
			GUI::show_error(nullptr, message);
			continue;
		}

		bool current_not_supported = false; //if slcr is incompatible but situation is not downgrade, we do forced updated and this bool is information to do it 

		if (ver_current_found && !ver_current->is_current_slic3r_supported()){
			if (ver_current->is_current_slic3r_downgrade()) {
				// "Reconfigure" situation.
				BOOST_LOG_TRIVIAL(warning) << "Current Slic3r incompatible with installed bundle: " << bundle_path.string();
				updates.incompats.emplace_back(std::move(bundle_path), *ver_current, vp.name);
				continue;
			}
		current_not_supported = true;
		}

		if (recommended->config_version < vp.config_version) {
			BOOST_LOG_TRIVIAL(warning) << format("Recommended config version for the currently running PrusaSlicer is older than the currently installed config for vendor %1%. This should not happen.", idx.vendor());
			continue;
		}

		if (recommended->config_version == vp.config_version) {
			// The recommended config bundle is already installed.
			continue;
		}

		// Config bundle update situation. The recommended config bundle version for this PrusaSlicer version from the index from the cache is newer
		// than the version of the currently installed config bundle.

		// The config index inside the cache directory (given by idx.path()) is one of the following:
		// 1) The last config index downloaded by any previously running PrusaSlicer instance
		// 2) The last config index installed by any previously running PrusaSlicer instance (older or newer) from its resources.
		// 3) The last config index installed by the currently running PrusaSlicer instance from its resources.
		// The config index is always the newest one (given by its newest config bundle referenced), and older config indices shall fully contain
		// the content of the older config indices.

		// Config bundle inside the cache directory.
		fs::path path_in_cache 		= cache_path / (idx.vendor() + ".ini");
		// Config bundle inside the resources directory.
		fs::path path_in_rsrc 		= rsrc_path  / (idx.vendor() + ".ini");
		// Config index inside the resources directory.
		fs::path path_idx_in_rsrc 	= rsrc_path  / (idx.vendor() + ".idx");

		// Search for a valid config bundle in the cache directory.
		bool 		found = false;
		Update    	new_update;
		fs::path 	bundle_path_idx_to_install;
		if (fs::exists(path_in_cache)) {
			try {
				VendorProfile new_vp = VendorProfile::from_ini(path_in_cache, true);
				VendorProfile old_vp = VendorProfile::from_ini(bundle_path, true);
				if (new_vp.config_version == recommended->config_version) {
					// The config bundle from the cache directory matches the recommended version of the index from the cache directory.
					// This is the newest known recommended config. Use it.
					if (!PresetUtils::vendor_profile_has_all_resources(new_vp)) {
						BOOST_LOG_TRIVIAL(warning) << "Some resources are missing for update of vedor " << new_vp.id;
					}
					std::vector<std::string> new_printers;
					PresetUtils::compare_vendor_profile_printers(old_vp, new_vp, new_printers);
					new_update = Update(std::move(path_in_cache), std::move(bundle_path), *recommended, vp.name, vp.changelog_url, current_not_supported, std::move(new_printers));
					// and install the config index from the cache into vendor's directory.
					bundle_path_idx_to_install = idx.path();
					found = true;
				}
			} catch (const std::exception &ex) {
				BOOST_LOG_TRIVIAL(info) << format("Failed to load the config bundle `%1%`: %2%", path_in_cache.string(), ex.what());
			}
		}

		// Keep the rsrc_idx outside of the next block, as we will reference the "recommended" version by an iterator.
		Index rsrc_idx;
		if (! found && fs::exists(path_in_rsrc) && fs::exists(path_idx_in_rsrc)) {
			// Trying the config bundle from resources (from the installation).
			// In that case, the recommended version number has to be compared against the recommended version reported by the config index from resources as well, 
			// as the config index in the cache directory may already be newer, recommending a newer config bundle than available in cache or resources.
			VendorProfile rsrc_vp;
			try {
				rsrc_vp = VendorProfile::from_ini(path_in_rsrc, false);
			} catch (const std::exception &ex) {
				BOOST_LOG_TRIVIAL(info) << format("Cannot load the config bundle at `%1%`: %2%", path_in_rsrc.string(), ex.what());
			}
			if (rsrc_vp.valid()) {
				try {
					rsrc_idx.load(path_idx_in_rsrc);
				} catch (const std::exception &ex) {
					BOOST_LOG_TRIVIAL(info) << format("Cannot load the config index at `%1%`: %2%", path_idx_in_rsrc.string(), ex.what());
				}
				recommended = rsrc_idx.recommended();
				if (recommended != rsrc_idx.end() && recommended->config_version == rsrc_vp.config_version && recommended->config_version > vp.config_version) {
					new_update = Update(std::move(path_in_rsrc), std::move(bundle_path), *recommended, vp.name, vp.changelog_url, current_not_supported);
					bundle_path_idx_to_install = path_idx_in_rsrc;
					found = true;
				} else {
					BOOST_LOG_TRIVIAL(warning) << format("The recommended config version for vendor `%1%` in resources does not match the recommended\n"
			                                             " config version for this version of PrusaSlicer. Corrupted installation?", idx.vendor());
				}
			}
		}

		if (found) {
			// Load 'installed' idx, if any.
			// 'Installed' indices are kept alongside the bundle in the `vendor` subdir
			// for bookkeeping to remember a cancelled update and not offer it again.
			if (fs::exists(bundle_path_idx)) {
				Index existing_idx;
				try {
					existing_idx.load(bundle_path_idx);
					// Find a recommended config bundle version for the slic3r version last executed. This makes sure that a config bundle update will not be missed
					// when upgrading an application. On the other side, the user will be bugged every time he will switch between slic3r versions.
                    /*const auto existing_recommended = existing_idx.recommended(old_slic3r_version);
                    if (existing_recommended != existing_idx.end() && recommended->config_version == existing_recommended->config_version) {
						// The user has already seen (and presumably rejected) this update
						BOOST_LOG_TRIVIAL(info) << format("Downloaded index for `%1%` is the same as installed one, not offering an update.",idx.vendor());
						continue;
					}*/
				} catch (const std::exception &err) {
					BOOST_LOG_TRIVIAL(error) << format("Cannot load the installed index at `%1%`: %2%", bundle_path_idx, err.what());
				}
			}
#if 0
			// Check if the update is already present in a snapshot
			if(!current_not_supported)
			{
				const auto recommended_snap = SnapshotDB::singleton().snapshot_with_vendor_preset(vp.name, recommended->config_version);
				if (recommended_snap != SnapshotDB::singleton().end()) {
					BOOST_LOG_TRIVIAL(info) << format("Bundle update %1% %2% already found in snapshot %3%, skipping...",
						vp.name,
						recommended->config_version.to_string(),
						recommended_snap->id);
					continue;
				}
			}
#endif // 0
			updates.updates.emplace_back(std::move(new_update));
			// 'Install' the index in the vendor directory. This is used to memoize
			// offered updates and to not offer the same update again if it was cancelled by the user.
			copy_file_fix(bundle_path_idx_to_install, bundle_path_idx);
		} else {
			BOOST_LOG_TRIVIAL(warning) << format("Index for vendor %1% indicates update (%2%) but the new bundle was found neither in cache nor resources",
				idx.vendor(),
				recommended->config_version.to_string());
		}
	}

	return updates;
}

bool PresetUpdater::priv::perform_updates(Updates &&updates, bool snapshot) const
{
	if (updates.incompats.size() > 0) {
		if (snapshot) {
			BOOST_LOG_TRIVIAL(info) << "Taking a snapshot...";
			if (! GUI::Config::take_config_snapshot_cancel_on_error(*GUI::wxGetApp().app_config, Snapshot::SNAPSHOT_DOWNGRADE, "",
				_u8L("Continue and install configuration updates?")))
				return false;
		}
		
		BOOST_LOG_TRIVIAL(info) << format("Deleting %1% incompatible bundles", updates.incompats.size());

		for (auto &incompat : updates.incompats) {
			BOOST_LOG_TRIVIAL(info) << '\t' << incompat;
			incompat.remove();
		}

		
	} else if (updates.updates.size() > 0) {
		
		if (snapshot) {
			BOOST_LOG_TRIVIAL(info) << "Taking a snapshot...";
			if (! GUI::Config::take_config_snapshot_cancel_on_error(*GUI::wxGetApp().app_config, Snapshot::SNAPSHOT_UPGRADE, "",
				_u8L("Continue and install configuration updates?")))
				return false;
		}

		BOOST_LOG_TRIVIAL(info) << format("Performing %1% updates", updates.updates.size());

		wxProgressDialog progress_dialog(_L("Installing profiles"), _L("Installing profiles") , 100, nullptr, wxPD_AUTO_HIDE);
		progress_dialog.Pulse();
		
		for (const auto &update : updates.updates) {
			BOOST_LOG_TRIVIAL(info) << '\t' << update;

			update.install();

			PresetBundle bundle;
			// Throw when parsing invalid configuration. Only valid configuration is supposed to be provided over the air.
			bundle.load_configbundle(update.source.string(), PresetBundle::LoadConfigBundleAttribute::LoadSystem, ForwardCompatibilitySubstitutionRule::Disable);

			BOOST_LOG_TRIVIAL(info) << format("Deleting %1% conflicting presets", bundle.prints.size() + bundle.filaments.size() + bundle.printers.size());

			auto preset_remover = [](const Preset &preset) {
				BOOST_LOG_TRIVIAL(info) << '\t' << preset.file;
				fs::remove(preset.file);
			};

			for (const auto &preset : bundle.prints)    { preset_remover(preset); }
			for (const auto &preset : bundle.filaments) { preset_remover(preset); }
			for (const auto &preset : bundle.printers)  { preset_remover(preset); }

			// Also apply the `obsolete_presets` property, removing obsolete ini files

			BOOST_LOG_TRIVIAL(info) << format("Deleting %1% obsolete presets",
				bundle.obsolete_presets.prints.size() + bundle.obsolete_presets.filaments.size() + bundle.obsolete_presets.printers.size());

			auto obsolete_remover = [](const char *subdir, const std::string &preset) {
				auto path = fs::path(Slic3r::data_dir()) / subdir / preset;
				path += ".ini";
				BOOST_LOG_TRIVIAL(info) << '\t' << path.string();
				fs::remove(path);
			};

			for (const auto &name : bundle.obsolete_presets.prints)    { obsolete_remover("print", name); }
			for (const auto &name : bundle.obsolete_presets.filaments) { obsolete_remover("filament", name); }
			for (const auto &name : bundle.obsolete_presets.sla_prints) { obsolete_remover("sla_print", name); } 
			for (const auto &name : bundle.obsolete_presets.sla_materials/*filaments*/) { obsolete_remover("sla_material", name); } 
			for (const auto &name : bundle.obsolete_presets.printers)  { obsolete_remover("printer", name); }
			
			// check if any resorces of installed bundle are missing. If so, new ones should be already downloaded at cache/vendor_id/
			VendorProfile vp;
			try {
				vp = VendorProfile::from_ini(update.target, true);
			}
			catch (const std::exception& e) {
				BOOST_LOG_TRIVIAL(error) << format("Corrupted profile file for vendor %1%, message: %2%", update.target, e.what());
				continue;
			}
			progress_dialog.Update(1, GUI::format_wxstr(_L("Downloading resources for %1%."),vp.id));
			progress_dialog.Pulse();
			for (const auto& model : vp.models) {
				for (const std::string& resource : { model.bed_texture, model.bed_model, model.thumbnail }) {
					if (resource.empty())
						continue;
					try
					{
						get_or_copy_missing_resource(vp.id, resource, vp.config_update_url);
					}
					catch (const std::exception& e)
					{
						BOOST_LOG_TRIVIAL(error) << "Failed to prepare " << resource << " for " << vp.id << " " << model.id << ": " << e.what();
					}
				}
			}	
		}

		progress_dialog.Destroy();
	}

	return true;
}
 
void PresetUpdater::priv::set_waiting_updates(Updates u)
{
	waiting_updates = u;
	has_waiting_updates = true;
}

PresetUpdater::PresetUpdater() :
	p(new priv())
{}


// Public

PresetUpdater::~PresetUpdater()
{
	if (p && p->thread.joinable()) {
		// This will stop transfers being done by the thread, if any.
		// Cancelling takes some time, but should complete soon enough.
		p->cancel = true;
		p->thread.join();
	}
}

void PresetUpdater::sync(const PresetBundle *preset_bundle, wxEvtHandler* evt_handler)
{
}

void PresetUpdater::cancel_sync()
{
	if (p && p->thread.joinable()) {
		// This will stop transfers being done by the thread, if any.
		// Cancelling takes some time, but should complete soon enough.
		p->cancel = true;
		p->thread.join();
	}
}

void PresetUpdater::slic3r_update_notify()
{
	if (! p->enabled_version_check)
		return;
	auto* app_config = GUI::wxGetApp().app_config;
	const auto ver_online_str = app_config->get("version_online");
	const auto ver_online = Semver::parse(ver_online_str);
	const auto ver_online_seen = Semver::parse(app_config->get("version_online_seen"));

	if (ver_online) {
		// Only display the notification if the version available online is newer AND if we haven't seen it before
		if (*ver_online > Slic3r::SEMVER && (! ver_online_seen || *ver_online_seen < *ver_online)) {
			GUI::MsgUpdateSlic3r notification(Slic3r::SEMVER, *ver_online);
			notification.ShowModal();
			if (notification.disable_version_check()) {
				app_config->set("notify_release", "none");
				p->enabled_version_check = false;
			}
		}

		app_config->set("version_online_seen", ver_online_str);
	}
}

static bool reload_configs_update_gui()
{
	wxString header = _L("Configuration Update will cause the preset modification to be lost.\n"
						 "So, check unsaved changes and save them if necessary.");
	if (!GUI::wxGetApp().check_and_save_current_preset_changes(_L("Updating"), header, false ))
		return false;

	// Reload global configuration
	auto* app_config = GUI::wxGetApp().app_config;
	// System profiles should not trigger any substitutions, user profiles may trigger substitutions, but these substitutions
	// were already presented to the user on application start up. Just do substitutions now and keep quiet about it.
	// However throw on substitutions in system profiles, those shall never happen with system profiles installed over the air.
	GUI::wxGetApp().preset_bundle->load_presets(*app_config, ForwardCompatibilitySubstitutionRule::EnableSilentDisableSystem);
	GUI::wxGetApp().load_current_presets();
	GUI::wxGetApp().plater()->set_bed_shape();

	return true;
}

PresetUpdater::UpdateResult PresetUpdater::config_update(const Semver& old_slic3r_version, UpdateParams params) const
{
 	if (! p->enabled_config_update) { return R_NOOP; }

	auto updates = p->get_config_updates(old_slic3r_version);
	if (updates.incompats.size() > 0) {
		BOOST_LOG_TRIVIAL(info) << format("%1% bundles incompatible. Asking for action...", updates.incompats.size());

		std::unordered_map<std::string, wxString> incompats_map;
		for (const auto &incompat : updates.incompats) {
			const auto min_slic3r = incompat.version.min_slic3r_version;
			const auto max_slic3r = incompat.version.max_slic3r_version;
			wxString restrictions;
			if (min_slic3r != Semver::zero() && max_slic3r != Semver::inf()) {
                restrictions = GUI::format_wxstr(_L("requires min. %s and max. %s"),
                    min_slic3r.to_string(),
                    max_slic3r.to_string());
			} else if (min_slic3r != Semver::zero()) {
				restrictions = GUI::format_wxstr(_L("requires min. %s"), min_slic3r.to_string());
				BOOST_LOG_TRIVIAL(debug) << "Bundle is not downgrade, user will now have to do whole wizard. This should not happen.";
			} else {
                restrictions = GUI::format_wxstr(_L("requires max. %s"), max_slic3r.to_string());
			}

			incompats_map.emplace(std::make_pair(incompat.vendor, std::move(restrictions)));
		}

		GUI::MsgDataIncompatible dlg(std::move(incompats_map));
		const auto res = dlg.ShowModal();
		if (res == wxID_REPLACE) {
			BOOST_LOG_TRIVIAL(info) << "User wants to re-configure...";

			// This effectively removes the incompatible bundles:
			// (snapshot is taken beforehand)
			if (! p->perform_updates(std::move(updates)) ||
				! GUI::wxGetApp().run_wizard(GUI::ConfigWizard::RR_DATA_INCOMPAT))
				return R_INCOMPAT_EXIT;

			return R_INCOMPAT_CONFIGURED;
		}
		else {
			BOOST_LOG_TRIVIAL(info) << "User wants to exit Slic3r, bye...";
			return R_INCOMPAT_EXIT;
		}

	} else if (updates.updates.size() > 0) {

		bool incompatible_version = false;
		for (const auto& update : updates.updates) {
			incompatible_version = (update.forced_update ? true : incompatible_version);
			//td::cout << update.forced_update << std::endl;
			//BOOST_LOG_TRIVIAL(info) << format("Update requires higher version.");
		}

		//forced update
		if (incompatible_version)
		{
			BOOST_LOG_TRIVIAL(info) << format("Update of %1% bundles available. At least one requires higher version of Slicer.", updates.updates.size());

			std::vector<GUI::MsgUpdateForced::Update> updates_msg;
			for (const auto& update : updates.updates) {
				std::string changelog_url = update.version.config_version.prerelease() == nullptr ? update.changelog_url : std::string();
				std::string printers;
				for (size_t i = 0; i < update.new_printers.size(); i++) {
					if (i > 0)
						printers += ", ";
					printers += update.new_printers[i];
				}
				updates_msg.emplace_back(update.vendor, update.version.config_version, update.version.comment, std::move(changelog_url), std::move(printers));
			}

			GUI::MsgUpdateForced dlg(updates_msg);

			const auto res = dlg.ShowModal();
			if (res == wxID_OK) {
				BOOST_LOG_TRIVIAL(info) << "User wants to update...";
				if (! p->perform_updates(std::move(updates)) ||
					! reload_configs_update_gui())
					return R_INCOMPAT_EXIT;
				return R_UPDATE_INSTALLED;
			}
			else {
				BOOST_LOG_TRIVIAL(info) << "User wants to exit Slic3r, bye...";
				return R_INCOMPAT_EXIT;
			}
		}

		// regular update
		if (params == UpdateParams::SHOW_NOTIFICATION) {
			p->set_waiting_updates(updates);
			bool new_printer = false;
			for (const Update& updt : updates.updates) {
				if(updt.new_printers.size() > 0) {
					new_printer = true;
					break;
				}
			}
			GUI::wxGetApp().plater()->get_notification_manager()->push_notification(new_printer? GUI::NotificationType::PresetUpdateAvailableNewPrinter : GUI::NotificationType::PresetUpdateAvailable);
		}
		else {
			BOOST_LOG_TRIVIAL(info) << format("Update of %1% bundles available. Asking for confirmation ...", p->waiting_updates.updates.size());

			std::vector<GUI::MsgUpdateConfig::Update> updates_msg;
			for (const auto& update : updates.updates) {
				std::string changelog_url = update.version.config_version.prerelease() == nullptr ? update.changelog_url : std::string();
				std::string printers;
				for (size_t i = 0; i < update.new_printers.size(); i++) {
					if (i > 0)
						printers += ", ";
					printers += update.new_printers[i];
				}
				updates_msg.emplace_back(update.vendor, update.version.config_version, update.version.comment, std::move(changelog_url), std::move(printers));
			}

			GUI::MsgUpdateConfig dlg(updates_msg, params == UpdateParams::FORCED_BEFORE_WIZARD);

			const auto res = dlg.ShowModal();
			if (res == wxID_OK) {
				BOOST_LOG_TRIVIAL(debug) << "User agreed to perform the update";
				if (! p->perform_updates(std::move(updates)) ||
					! reload_configs_update_gui())
					return R_ALL_CANCELED;
				return R_UPDATE_INSTALLED;
			}
			else {
				BOOST_LOG_TRIVIAL(info) << "User refused the update";
				if (params == UpdateParams::FORCED_BEFORE_WIZARD && res == wxID_CANCEL)
					return R_ALL_CANCELED;
				return R_UPDATE_REJECT;
			}
		}
		
		// MsgUpdateConfig will show after the notificaation is clicked
	} else {
		BOOST_LOG_TRIVIAL(info) << "No configuration updates available.";
	}

	return R_NOOP;
}

bool PresetUpdater::install_bundles_rsrc_or_cache_vendor(std::vector<std::string> bundles, bool snapshot) const
{
	Updates updates;

	BOOST_LOG_TRIVIAL(info) << format("Installing %1% bundles from resources ...", bundles.size());

	for (const auto &bundle : bundles) {
		auto path_in_rsrc = (p->rsrc_path / bundle).replace_extension(".ini");
		auto path_in_cache_vendor = (p->cache_vendor_path / bundle).replace_extension(".ini");
		auto path_in_vendors = (p->vendor_path / bundle).replace_extension(".ini");

		bool is_in_rsrc = fs::exists(path_in_rsrc);
		bool is_in_cache_vendor = fs::exists(path_in_cache_vendor) && !fs::is_empty(path_in_cache_vendor);

		// Find if in cache vendor is newer version than in resources.
		// But we also need to mind too new versions - have to read index.

		// Fresh index should be in archive_dir, otherwise look for it in cache 
		fs::path idx_path (path_in_cache_vendor);
		idx_path.replace_extension(".idx");
		if (!boost::filesystem::exists(idx_path)) {
			BOOST_LOG_TRIVIAL(error) << GUI::format("Couldn't locate idx file %1% when performing updates.", idx_path.string());
			idx_path = fs::path(p->cache_path / idx_path.filename());
		}
		if (!boost::filesystem::exists(idx_path)) {
			std::string msg = GUI::format(_L("Couldn't locate index file for vendor %1% when performing updates. The profile will not be installed."), bundle);
			BOOST_LOG_TRIVIAL(error) << msg;
			GUI::show_error(nullptr, msg);
			continue;
		}
		Slic3r::GUI::Config::Index index;
		try {
			index.load(idx_path);
		}
		catch (const std::exception& /* err */) {
			std::string msg = GUI::format(_L("Couldn't load index file for vendor %1% when performing updates. The profile will not be installed. Reason: Corrupted index file %2%."), bundle, idx_path.string());
			BOOST_LOG_TRIVIAL(error) << msg;
			GUI::show_error(nullptr, msg);
			continue;
		}
		const auto recommended_it = index.recommended();
		const auto recommended = recommended_it->config_version;

		if (is_in_cache_vendor) {
			Semver version_cache = Semver::zero();
			try {
				auto vp_cache = VendorProfile::from_ini(path_in_cache_vendor, false);
				version_cache = vp_cache.config_version;
			}
			catch (const std::exception& e) {
				BOOST_LOG_TRIVIAL(error) << format("Corrupted profile file for vendor %1%, message: %2%", path_in_cache_vendor, e.what());
				version_cache = Semver::zero();
			}
			if (version_cache > recommended)
				version_cache = Semver::zero();

			Semver version_rsrc = Semver::zero();
			try {
				if (is_in_rsrc) {
					auto vp = VendorProfile::from_ini(path_in_rsrc, false);
					version_rsrc = vp.config_version;
				}
			}
			catch (const std::exception& e) {
				BOOST_LOG_TRIVIAL(error) << format("Corrupted profile file for vendor %1%, message: %2%", path_in_rsrc, e.what());
				//continue;
				version_rsrc = Semver::zero();
			}
			// Should not happen!
			if (version_rsrc > recommended)
				version_rsrc = Semver::zero();

			if (version_cache == Semver::zero() && version_rsrc == Semver::zero()) {
				std::string msg = GUI::format(_L("Couldn't open profile file for vendor %1% when performing updates. The profile will not be installed. This installation might be corrupted."), bundle);
				BOOST_LOG_TRIVIAL(error) << msg;
				GUI::show_error(nullptr, msg);
				continue;
			} else if (version_cache == Semver::zero()) {
				// cache vendor cannot be used, use resources
				updates.updates.emplace_back(std::move(path_in_rsrc), std::move(path_in_vendors), Version(), "", "");
			} else if (version_rsrc == Semver::zero()) {
				// resources cannto be used, use cache vendor
				updates.updates.emplace_back(std::move(path_in_cache_vendor), std::move(path_in_vendors), Version(), "", "");
			} else if (version_cache > version_rsrc) {
				// in case we are installing from cache / vendor. we should also copy index to cache
				// This needs to be done now bcs the current one would be missing this version on the next start 
				auto  path_idx_cache = (p->cache_path / bundle).replace_extension(".idx");
				if (idx_path != path_idx_cache)
					copy_file_fix(idx_path, path_idx_cache);
				updates.updates.emplace_back(std::move(path_in_cache_vendor), std::move(path_in_vendors), Version(), "", "");
			} else {
				updates.updates.emplace_back(std::move(path_in_rsrc), std::move(path_in_vendors), Version(), "", "");
			}
		} else {
			if (! is_in_rsrc) {
				// This should not happen. Instead of an assert, make it crash in Release mode too.
				BOOST_LOG_TRIVIAL(error) << "Internal error in PresetUpdater! Terminating the application.";
				std::terminate();
			}
			updates.updates.emplace_back(std::move(path_in_rsrc), std::move(path_in_vendors), Version(), "", "");
		}
	}

	return p->perform_updates(std::move(updates), snapshot);
}

void PresetUpdater::on_update_notification_confirm()
{
	if (!p->has_waiting_updates)
		return;
	BOOST_LOG_TRIVIAL(info) << format("Update of %1% bundles available. Asking for confirmation ...", p->waiting_updates.updates.size());

	std::vector<GUI::MsgUpdateConfig::Update> updates_msg;
	for (const auto& update : p->waiting_updates.updates) {
		std::string changelog_url = update.version.config_version.prerelease() == nullptr ? update.changelog_url : std::string();
		std::string printers;
		for (size_t i = 0; i < update.new_printers.size(); i++) {
			if (i > 0)
				printers += ", ";
			printers += update.new_printers[i];
		}
		updates_msg.emplace_back(update.vendor, update.version.config_version, update.version.comment, std::move(changelog_url), std::move(printers));
	}

	GUI::MsgUpdateConfig dlg(updates_msg);

	const auto res = dlg.ShowModal();
	if (res == wxID_OK) {
		BOOST_LOG_TRIVIAL(debug) << "User agreed to perform the update";
		if (p->perform_updates(std::move(p->waiting_updates)) &&
			reload_configs_update_gui()) {
			p->has_waiting_updates = false;
		}
	}
	else {
		BOOST_LOG_TRIVIAL(info) << "User refused the update";
	}	
}

bool PresetUpdater::version_check_enabled() const
{
	return p->enabled_version_check;
}

void PresetUpdater::update_index_db()
{
	p->update_index_db();
}

}
