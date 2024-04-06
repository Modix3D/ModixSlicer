///|/ Copyright (c) Prusa Research 2018 - 2023 David Kocík @kocikdav, Oleksandra Iushchenko @YuSanka, Vojtěch Král @vojtechkral, Vojtěch Bubník @bubnikv
///|/
///|/ ported from lib/Slic3r/GUI/ConfigWizard.pm:
///|/ Copyright (c) Prusa Research 2016 - 2018 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2012 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Henrik Brix Andersen @henrikbrixandersen
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_ConfigWizard_hpp_
#define slic3r_ConfigWizard_hpp_

#include <memory>

#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>

#include "GUI_Utils.hpp"

namespace Slic3r {

class PresetBundle;
class PresetUpdater;

namespace GUI {

class ConfigWizard: public DPIDialog
{
public:
    // Why is the Wizard run
    enum RunReason {
        RR_DATA_EMPTY,                  // No or empty datadir
        RR_DATA_LEGACY,                 // Pre-updating datadir
        RR_DATA_INCOMPAT,               // Incompatible datadir - Slic3r downgrade situation
        RR_USER,                        // User requested the Wizard from the menus
    };

    // What page should wizard start on
    enum StartPage {
        SP_WELCOME,
        SP_PRINTERS,
        SP_FILAMENTS,
        SP_MATERIALS,
    };

    ConfigWizard(wxWindow *parent);
    ConfigWizard(ConfigWizard &&) = delete;
    ConfigWizard(const ConfigWizard &) = delete;
    ConfigWizard &operator=(ConfigWizard &&) = delete;
    ConfigWizard &operator=(const ConfigWizard &) = delete;
    ~ConfigWizard();

    // Run the Wizard. Return whether it was completed.
    bool run(RunReason reason, StartPage start_page = SP_WELCOME);

    static const wxString& name(const bool from_menu = false);
protected:
    void on_dpi_changed(const wxRect &suggested_rect) override ;
    void on_sys_color_changed() override;

private:
    struct priv;
    std::unique_ptr<priv> p;

    friend struct ConfigWizardPage;
};



}
}

#endif
