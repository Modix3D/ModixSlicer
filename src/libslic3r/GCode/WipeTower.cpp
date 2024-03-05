///|/ Copyright (c) Prusa Research 2017 - 2023 Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv, Enrico Turri @enricoturri1966
///|/ Copyright (c) SuperSlicer 2023 Remi Durand @supermerill
///|/ Copyright (c) 2020 Paul Arden @ardenpm
///|/ Copyright (c) 2019 Thomas Moore
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "WipeTower.hpp"

#include <cassert>
#include <iostream>
#include <vector>
#include <numeric>
#include <memory>
#include <sstream>
#include <iomanip>

#include "ClipperUtils.hpp"
#include "GCodeProcessor.hpp"
#include "BoundingBox.hpp"
#include "LocalesUtils.hpp"
#include "Geometry.hpp"
#include "Surface.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Flow.hpp"

#include <boost/algorithm/string/predicate.hpp>


namespace Slic3r
{

class WipeTowerWriter
{
public:
	WipeTowerWriter(float layer_height, float line_width, GCodeFlavor flavor, const std::vector<WipeTower::FilamentParameters>& filament_parameters) :
		m_current_pos(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
		m_current_z(0.f),
		m_current_feedrate(0.f),
		m_layer_height(layer_height),
		m_preview_suppressed(false),
		m_elapsed_time(0.f),
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
        m_default_analyzer_line_width(line_width),
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING
        m_gcode_flavor(flavor),
        m_filpar(filament_parameters)
        {
            // adds tag for analyzer:
            std::ostringstream str;
            str << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) << m_layer_height << "\n"; // don't rely on GCodeAnalyzer knowing the layer height - it knows nothing at priming
            str << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role) << gcode_extrusion_role_to_string(GCodeExtrusionRole::WipeTower) << "\n";
            m_gcode += str.str();
            change_analyzer_line_width(line_width);
        }

    WipeTowerWriter& change_analyzer_line_width(float line_width) {
        // adds tag for analyzer:
        std::stringstream str;
        str << ";" << GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width) << line_width << "\n";
        m_gcode += str.str();
        return *this;
    }

#if ENABLE_GCODE_VIEWER_DATA_CHECKING
    WipeTowerWriter& change_analyzer_mm3_per_mm(float len, float e) {
        static const float area = float(M_PI) * 1.75f * 1.75f / 4.f;
        float mm3_per_mm = (len == 0.f ? 0.f : area * e / len);
        // adds tag for processor:
        std::stringstream str;
        str << ";" << GCodeProcessor::Mm3_Per_Mm_Tag << mm3_per_mm << "\n";
        m_gcode += str.str();
        return *this;
    }
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING

	WipeTowerWriter& 			 set_initial_position(const Vec2f &pos, float width = 0.f, float depth = 0.f, float internal_angle = 0.f) {
        m_wipe_tower_width = width;
        m_wipe_tower_depth = depth;
        m_internal_angle = internal_angle;
		m_start_pos = this->rotate(pos);
		m_current_pos = pos;
		return *this;
	}

    WipeTowerWriter& set_position(const Vec2f &pos) { m_current_pos = pos; return *this; }

    WipeTowerWriter& set_initial_tool(size_t tool) { m_current_tool = tool; return *this; }

	WipeTowerWriter& set_z(float z)
		{ m_current_z = z; return *this; }

	WipeTowerWriter& set_extrusion_flow(float flow)
		{ m_extrusion_flow = flow; return *this; }

    float get_extrusion_flow()
        { return m_extrusion_flow; }

	WipeTowerWriter& feedrate(float f)
	{
        if (f != m_current_feedrate) {
			m_gcode += "G1" + set_format_F(f) + "\n";
            m_current_feedrate = f;
        }
		return *this;
	}

	const std::string&   gcode() const { return m_gcode; }
	const std::vector<WipeTower::Extrusion>& extrusions() const { return m_extrusions; }
	float                x()     const { return m_current_pos.x(); }
	float                y()     const { return m_current_pos.y(); }
	const Vec2f& 		 pos()   const { return m_current_pos; }
	const Vec2f	 		 start_pos_rotated() const { return m_start_pos; }
	const Vec2f  		 pos_rotated() const { return this->rotate(m_current_pos); }
	float 				 elapsed_time() const { return m_elapsed_time; }
    float                get_and_reset_used_filament_length() { float temp = m_used_filament_length; m_used_filament_length = 0.f; return temp; }

	// Extrude with an explicitely provided amount of extrusion.
	WipeTowerWriter& extrude_explicit(float x, float y, float e, float f = 0.f, bool record_length = false, bool limit_volumetric_flow = true)
	{
		if (x == m_current_pos.x() && y == m_current_pos.y() && e == 0.f && (f == 0.f || f == m_current_feedrate))
			// Neither extrusion nor a travel move.
			return *this;

		float dx = x - m_current_pos.x();
		float dy = y - m_current_pos.y();
        float len = std::sqrt(dx*dx+dy*dy);
        if (record_length)
            m_used_filament_length += e;

		// Now do the "internal rotation" with respect to the wipe tower center
		Vec2f rotated_current_pos(this->pos_rotated());
		Vec2f rot(this->rotate(Vec2f(x,y)));                               // this is where we want to go

        if (! m_preview_suppressed && e > 0.f && len > 0.f) {
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
            change_analyzer_mm3_per_mm(len, e);
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING
            // Width of a squished extrusion, corrected for the roundings of the squished extrusions.
			// This is left zero if it is a travel move.
            float width = e * m_filpar[0].filament_area / (len * m_layer_height);
			// Correct for the roundings of a squished extrusion.
			width += m_layer_height * float(1. - M_PI / 4.);
			if (m_extrusions.empty() || m_extrusions.back().pos != rotated_current_pos)
				m_extrusions.emplace_back(WipeTower::Extrusion(rotated_current_pos, 0, m_current_tool));
			m_extrusions.emplace_back(WipeTower::Extrusion(rot, width, m_current_tool));
		}

		m_gcode += "G1";
        if (std::abs(rot.x() - rotated_current_pos.x()) > (float)EPSILON)
			m_gcode += set_format_X(rot.x());

        if (std::abs(rot.y() - rotated_current_pos.y()) > (float)EPSILON)
			m_gcode += set_format_Y(rot.y());


		if (e != 0.f)
			m_gcode += set_format_E(e);

        if (f != 0.f && f != m_current_feedrate) {
            if (limit_volumetric_flow) {
                float e_speed = e / (((len == 0.f) ? std::abs(e) : len) / f * 60.f);
                f /= std::max(1.f, e_speed / m_filpar[m_current_tool].max_e_speed);
            }
			m_gcode += set_format_F(f);
        }

        // Append newline if at least one of X,Y,E,F was changed.
        // Otherwise, remove the "G1".
        if (! boost::ends_with(m_gcode, "G1"))
            m_gcode += "\n";
        else
            m_gcode.erase(m_gcode.end()-2, m_gcode.end());

        m_current_pos.x() = x;
        m_current_pos.y() = y;

		// Update the elapsed time with a rough estimate.
        m_elapsed_time += ((len == 0.f) ? std::abs(e) : len) / m_current_feedrate * 60.f;
		return *this;
	}

	WipeTowerWriter& extrude_explicit(const Vec2f &dest, float e, float f = 0.f, bool record_length = false, bool limit_volumetric_flow = true)
		{ return extrude_explicit(dest.x(), dest.y(), e, f, record_length); }

	// Travel to a new XY position. f=0 means use the current value.
	WipeTowerWriter& travel(float x, float y, float f = 0.f)
		{ return extrude_explicit(x, y, 0.f, f); }

	WipeTowerWriter& travel(const Vec2f &dest, float f = 0.f)
		{ return extrude_explicit(dest.x(), dest.y(), 0.f, f); }

	// Extrude a line from current position to x, y with the extrusion amount given by m_extrusion_flow.
	WipeTowerWriter& extrude(float x, float y, float f = 0.f)
	{
		float dx = x - m_current_pos.x();
		float dy = y - m_current_pos.y();
        return extrude_explicit(x, y, std::sqrt(dx*dx+dy*dy) * m_extrusion_flow, f, true);
	}

	WipeTowerWriter& extrude(const Vec2f &dest, const float f = 0.f)
		{ return extrude(dest.x(), dest.y(), f); }

    WipeTowerWriter& rectangle(const Vec2f& ld,float width,float height,const float f = 0.f)
    {
        Vec2f corners[4];
        corners[0] = ld;
        corners[1] = ld + Vec2f(width,0.f);
        corners[2] = ld + Vec2f(width,height);
        corners[3] = ld + Vec2f(0.f,height);
        int index_of_closest = 0;
        if (x()-ld.x() > ld.x()+width-x())    // closer to the right
            index_of_closest = 1;
        if (y()-ld.y() > ld.y()+height-y())   // closer to the top
            index_of_closest = (index_of_closest==0 ? 3 : 2);

        travel(corners[index_of_closest].x(), y());      // travel to the closest corner
        travel(x(),corners[index_of_closest].y());

        int i = index_of_closest;
        do {
            ++i;
            if (i==4) i=0;
            extrude(corners[i], f);
        } while (i != index_of_closest);
        return (*this);
    }

    WipeTowerWriter& rectangle(const WipeTower::box_coordinates& box, const float f = 0.f)
    {
        rectangle(Vec2f(box.ld.x(), box.ld.y()),
                  box.ru.x() - box.lu.x(),
                  box.ru.y() - box.rd.y(), f);
        return (*this);
    }

	WipeTowerWriter& load(float e, float f = 0.f)
	{
		if (e == 0.f && (f == 0.f || f == m_current_feedrate))
			return *this;
		m_gcode += "G1";
		if (e != 0.f)
			m_gcode += set_format_E(e);
		if (f != 0.f && f != m_current_feedrate)
			m_gcode += set_format_F(f);
		m_gcode += "\n";
		return *this;
	}

	WipeTowerWriter& retract(float e, float f = 0.f)
		{ return load(-e, f); }

// Loads filament while also moving towards given points in x-axis (x feedrate is limited by cutting the distance short if necessary)
    WipeTowerWriter& load_move_x_advanced(float farthest_x, float loading_dist, float loading_speed, float max_x_speed = 50.f)
    {
        float time = std::abs(loading_dist / loading_speed); // time that the move must take
        float x_distance = std::abs(farthest_x - x());       // max x-distance that we can travel
        float x_speed = x_distance / time;                   // x-speed to do it in that time

        if (x_speed > max_x_speed) {
            // Necessary x_speed is too high - we must shorten the distance to achieve max_x_speed and still respect the time.
            x_distance = max_x_speed * time;
            x_speed = max_x_speed;
        }

        float end_point = x() + (farthest_x > x() ? 1.f : -1.f) * x_distance;
        return extrude_explicit(end_point, y(), loading_dist, x_speed * 60.f, false, false);
    }

	// Elevate the extruder head above the current print_z position.
	WipeTowerWriter& z_hop(float hop, float f = 0.f)
	{
		m_gcode += std::string("G1") + set_format_Z(m_current_z + hop);
		if (f != 0 && f != m_current_feedrate)
			m_gcode += set_format_F(f);
		m_gcode += "\n";
		return *this;
	}

	// Lower the extruder head back to the current print_z position.
	WipeTowerWriter& z_hop_reset(float f = 0.f)
		{ return z_hop(0, f); }

	// Move to x1, +y_increment,
	// extrude quickly amount e to x2 with feed f.
	WipeTowerWriter& ram(float x1, float x2, float dy, float e0, float e, float f)
	{
		extrude_explicit(x1, m_current_pos.y() + dy, e0, f, true, false);
		extrude_explicit(x2, m_current_pos.y(), e, 0.f, true, false);
		return *this;
	}

	// Let the end of the pulled out filament cool down in the cooling tube
	// by moving up and down and moving the print head left / right
	// at the current Y position to spread the leaking material.
	WipeTowerWriter& cool(float x1, float x2, float e1, float e2, float f)
	{
		extrude_explicit(x1, m_current_pos.y(), e1, f, false, false);
		extrude_explicit(x2, m_current_pos.y(), e2, false, false);
		return *this;
	}

    WipeTowerWriter& set_tool(size_t tool)
	{
		m_current_tool = tool;
		return *this;
	}

	// Set extruder temperature, don't wait by default.
	WipeTowerWriter& set_extruder_temp(int temperature, bool wait = false)
	{
        m_gcode += "M" + std::to_string(wait ? 109 : 104) + " S" + std::to_string(temperature) + "\n";
        return *this;
    }

    // Wait for a period of time (seconds).
	WipeTowerWriter& wait(float time)
	{
        if (time==0.f)
            return *this;
        m_gcode += "G4 S" + Slic3r::float_to_string_decimal_point(time, 3) + "\n";
		return *this;
    }

	// Set speed factor override percentage.
	WipeTowerWriter& speed_override(int speed)
	{
        m_gcode += "M220 S" + std::to_string(speed) + "\n";
		return *this;
    }

	// Let the firmware back up the active speed override value.
	WipeTowerWriter& speed_override_backup()
    {
        // This is only supported by Prusa at this point (https://github.com/prusa3d/PrusaSlicer/issues/3114)
        if (m_gcode_flavor == gcfMarlinLegacy || m_gcode_flavor == gcfMarlinFirmware)
            m_gcode += "M220 B\n";
		return *this;
    }

	// Let the firmware restore the active speed override value.
	WipeTowerWriter& speed_override_restore()
	{
        if (m_gcode_flavor == gcfMarlinLegacy || m_gcode_flavor == gcfMarlinFirmware)
            m_gcode += "M220 R\n";
		return *this;
    }

	// Set digital trimpot motor
	WipeTowerWriter& set_extruder_trimpot(int current)
	{
        if (m_gcode_flavor == gcfKlipper)
            return *this;
        if (m_gcode_flavor == gcfRepRapSprinter || m_gcode_flavor == gcfRepRapFirmware)
            m_gcode += "M906 E";
        else
            m_gcode += "M907 E";
        m_gcode += std::to_string(current) + "\n";
		return *this;
    }

	WipeTowerWriter& flush_planner_queue()
	{
		m_gcode += "G4 S0\n";
		return *this;
	}

	// Reset internal extruder counter.
	WipeTowerWriter& reset_extruder()
	{
		m_gcode += "G92 E0\n";
		return *this;
	}

	WipeTowerWriter& comment_with_value(const char *comment, int value)
    {
        m_gcode += std::string(";") + comment + std::to_string(value) + "\n";
		return *this;
    }


    WipeTowerWriter& set_fan(unsigned speed)
	{
		if (speed == m_last_fan_speed)
			return *this;
		if (speed == 0)
			m_gcode += "M107\n";
        else
            m_gcode += "M106 S" + std::to_string(unsigned(255.0 * speed / 100.0)) + "\n";
		m_last_fan_speed = speed;
		return *this;
	}

	WipeTowerWriter& append(const std::string& text) { m_gcode += text; return *this; }

    const std::vector<Vec2f>& wipe_path() const
    {
        return m_wipe_path;
    }

    WipeTowerWriter& add_wipe_point(const Vec2f& pt)
    {
        m_wipe_path.push_back(rotate(pt));
        return *this;
    }

    WipeTowerWriter& add_wipe_point(float x, float y)
    {
        return add_wipe_point(Vec2f(x, y));
    }

private:
	Vec2f         m_start_pos;
	Vec2f         m_current_pos;
    std::vector<Vec2f>  m_wipe_path;
	float    	  m_current_z;
	float 	  	  m_current_feedrate;
    size_t        m_current_tool;
	float 		  m_layer_height;
    float 	  	  m_extrusion_flow;
	bool		  m_preview_suppressed;
	std::string   m_gcode;
	std::vector<WipeTower::Extrusion> m_extrusions;
	float         m_elapsed_time;
	float   	  m_internal_angle = 0.f;
	float		  m_y_shift = 0.f;
	float 		  m_wipe_tower_width = 0.f;
	float		  m_wipe_tower_depth = 0.f;
    unsigned      m_last_fan_speed = 0;
    int           current_temp = -1;
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
    const float   m_default_analyzer_line_width;
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING
    float         m_used_filament_length = 0.f;
    GCodeFlavor   m_gcode_flavor;
    const std::vector<WipeTower::FilamentParameters>& m_filpar;

	std::string   set_format_X(float x)
    {
        m_current_pos.x() = x;
        return " X" + Slic3r::float_to_string_decimal_point(x, 3);
	}

	std::string   set_format_Y(float y) {
        m_current_pos.y() = y;
        return " Y" + Slic3r::float_to_string_decimal_point(y, 3);
	}

	std::string   set_format_Z(float z) {
        return " Z" + Slic3r::float_to_string_decimal_point(z, 3);
	}

	std::string   set_format_E(float e) {
        return " E" + Slic3r::float_to_string_decimal_point(e, 4);
	}

	std::string   set_format_F(float f) {
        char buf[64];
        sprintf(buf, " F%d", int(floor(f + 0.5f)));
        m_current_feedrate = f;
        return buf;
	}

	WipeTowerWriter& operator=(const WipeTowerWriter &rhs);

	// Rotate the point around center of the wipe tower about given angle (in degrees)
	Vec2f rotate(Vec2f pt) const
	{
		pt.x() -= m_wipe_tower_width / 2.f;
		pt.y() += m_y_shift - m_wipe_tower_depth / 2.f;
	    double angle = m_internal_angle * float(M_PI/180.);
	    double c = cos(angle);
	    double s = sin(angle);
	    return Vec2f(float(pt.x() * c - pt.y() * s) + m_wipe_tower_width / 2.f, float(pt.x() * s + pt.y() * c) + m_wipe_tower_depth / 2.f);
	}

}; // class WipeTowerWriter



WipeTower::ToolChangeResult WipeTower::construct_tcr(WipeTowerWriter& writer,
                                                     bool priming,
                                                     size_t old_tool) const
{
    ToolChangeResult result;
    result.priming      = priming;
    result.initial_tool = int(old_tool);
    result.new_tool     = int(m_current_tool);
    result.print_z      = m_z_pos;
    result.layer_height = m_layer_height;
    result.elapsed_time = writer.elapsed_time();
    result.start_pos    = writer.start_pos_rotated();
    result.end_pos      = priming ? writer.pos() : writer.pos_rotated();
    result.gcode        = std::move(writer.gcode());
    result.extrusions   = std::move(writer.extrusions());
    result.wipe_path    = std::move(writer.wipe_path());
    return result;
}



WipeTower::WipeTower(const PrintConfig& config, const PrintRegionConfig& default_region_config, size_t initial_tool) :
    m_wipe_tower_pos(config.wipe_tower_x, config.wipe_tower_y),
    m_wipe_tower_width(float(config.wipe_tower_width)),
    m_wipe_tower_depth(float(config.wipe_tower_depth)),
    m_wipe_tower_rotation_angle(float(config.wipe_tower_rotation_angle)),
    m_wipe_tower_brim_width(float(config.wipe_tower_brim_width)),
    m_y_shift(0.f),
    m_z_pos(0.f),
    m_gcode_flavor(config.gcode_flavor),
    m_travel_speed(config.travel_speed),
    m_infill_speed(default_region_config.wipe_tower_infill_speed),
    m_perimeter_speed(default_region_config.wipe_tower_perimeter_speed),
    m_current_tool(initial_tool),
	m_extra_perimeters(config.wipe_tower_perimeters-1),
	m_density(float(config.wipe_tower_density)),
	m_brim_layers(float(config.wipe_tower_brim_layers/100.f)),
	m_wipe_tower_extruder(config.wipe_tower_extruder)
{
    // Read absolute value of first layer speed, if given as percentage,
    // it is taken over following default. Speeds from config are not
    // easily accessible here.
    const float default_speed = 60.f;
    m_first_layer_speed = config.get_abs_value("first_layer_speed", default_speed);
    if (m_first_layer_speed == 0.f) // just to make sure autospeed doesn't break it.
        m_first_layer_speed = default_speed / 2.f;

    // Autospeed may be used...
    if (m_infill_speed == 0.f)
        m_infill_speed = 80.f;
    if (m_perimeter_speed == 0.f)
        m_perimeter_speed = 80.f;


    // Calculate where the priming lines should be - very naive test not detecting parallelograms etc.
    const std::vector<Vec2d>& bed_points = config.bed_shape.values;
    BoundingBoxf bb(bed_points);
    m_bed_width = float(bb.size().x());
    m_bed_shape = (bed_points.size() == 4 ? RectangularBed : CircularBed);

    if (m_bed_shape == CircularBed) {
        // this may still be a custom bed, check that the points are roughly on a circle
        double r2 = std::pow(m_bed_width/2., 2.);
        double lim2 = std::pow(m_bed_width/10., 2.);
        Vec2d center = bb.center();
        for (const Vec2d& pt : bed_points)
            if (std::abs(std::pow(pt.x()-center.x(), 2.) + std::pow(pt.y()-center.y(), 2.) - r2) > lim2) {
                m_bed_shape = CustomBed;
                break;
            }
    }

    m_bed_bottom_left = m_bed_shape == RectangularBed
                  ? Vec2f(bed_points.front().x(), bed_points.front().y())
                  : Vec2f::Zero();
}



void WipeTower::set_extruder(size_t idx, const PrintConfig& config, const PrintRegionConfig& default_region_config)
{
    //while (m_filpar.size() < idx+1)   // makes sure the required element is in the vector
    m_filpar.push_back(FilamentParameters());

    m_filpar[idx].material = config.filament_type.get_at(idx);
    m_filpar[idx].temperature = config.temperature.get_at(idx);
    m_filpar[idx].first_layer_temperature = config.first_layer_temperature.get_at(idx);
    m_filpar[idx].filament_area = float((M_PI/4.f) * pow(config.filament_diameter.get_at(idx), 2)); // all extruders are assumed to have the same filament diameter at this point
    float nozzle_diameter = float(config.nozzle_diameter.get_at(idx));
    m_filpar[idx].nozzle_diameter = nozzle_diameter;

    float max_vol_speed = float(config.filament_max_volumetric_speed.get_at(idx));
    if (max_vol_speed!= 0.f) {
        m_filpar[idx].max_e_speed = (max_vol_speed / filament_area());
    }

    m_used_filament_length.resize(std::max(m_used_filament_length.size(), idx + 1)); // makes sure that the vector is big enough so we don't have to check later
}

// Appends a toolchange into m_plan and calculates neccessary depth of the corresponding box
void WipeTower::plan_toolchange(float z_par, float layer_height_par, unsigned int old_tool,
                                unsigned int new_tool, float wipe_volume)
{
	assert(m_plan.empty() || m_plan.back().z <= z_par + WT_EPSILON);	// refuses to add a layer below the last one

	if (m_plan.empty() || m_plan.back().z + WT_EPSILON < z_par) // if we moved to a new layer, we'll add it to m_plan first
		m_plan.push_back(WipeTowerInfo(z_par, layer_height_par));

    if (m_first_layer_idx == size_t(-1) && (old_tool != new_tool || m_plan.size() == 1))
        m_first_layer_idx = m_plan.size() - 1;

    if (old_tool == new_tool)	// new layer without toolchanges - we are done
        return;

	m_plan.back().tool_changes.push_back(WipeTowerInfo::ToolChange(old_tool, new_tool));
}



void WipeTower::plan_tower()
{
    m_wipe_tower_height = m_plan.empty() ? 0.f : m_plan.back().z;
}

// Processes vector m_plan and calls respective functions to generate G-code for the wipe tower
// Resulting ToolChangeResults are appended into vector "result"
void WipeTower::generate(std::vector<std::vector<WipeTower::ToolChangeResult>> &result)
{
	if (m_plan.empty())
        return;

    m_wipe_tower_height = m_plan.back().z;
    m_layer_info = m_plan.begin();

    // we don't know which extruder to start with - we'll set it according to the first toolchange
    for (const auto& layer : m_plan) {
        if (!layer.tool_changes.empty()) {
            m_current_tool = layer.tool_changes.front().old_tool;
            break;
        }
    }

    for (auto& used : m_used_filament_length) {
        // reset used filament stats
        used = 0.f;
    }

    for (const WipeTower::WipeTowerInfo& layer : m_plan)
    {
        set_layer(layer.z, layer.layer_height);

        std::vector<WipeTower::ToolChangeResult> layer_result;
        ToolChangeResult layer_tcr;

        int old_tool = m_current_tool;

        WipeTowerWriter writer(m_layer_height, current_nozzle_diameter(), m_gcode_flavor, m_filpar);
        writer.set_z(m_z_pos);
        writer.set_initial_tool(m_current_tool);

        if (m_wipe_tower_extruder) {
            change_tool(writer, m_wipe_tower_extruder-1);
        } else {
            change_tool(writer, m_current_tool);
        }

        int extrude_speed_perimeter, extrude_speed_infill;
        if (is_first_layer()) {
            extrude_speed_perimeter = extrude_speed_infill = m_first_layer_speed * 60.f;
        } else {
            extrude_speed_perimeter = m_perimeter_speed * 60.f;
            extrude_speed_infill = m_infill_speed * 60.f;
        }

        // add number of layers to the brim
        float reinforce_max_z = m_brim_layers * m_wipe_tower_height;
        {

            // xx. do we have to adjust the number of loops because the nozzle size changed?
            // xx. Hack! assume 2 nozzles
            auto biggest = std::max(m_filpar[0].nozzle_diameter, m_filpar[1].nozzle_diameter);
            float alpha = biggest / m_filpar[m_current_tool].nozzle_diameter;

            // 1. convert brim width to loop number
            int loops = m_wipe_tower_brim_width / biggest;

            // brim (first layer only)
            if (is_first_layer()) {
                wipe_contour_2(writer, loops * alpha, extrude_speed_perimeter);
            } else {
                // 2. adjust number of loops depending on z position
                int loops2 = (m_z_pos / reinforce_max_z) * loops;

                // 3. above the support reinforcement, but keep extra perimeters
                int loops3 = std::max( int(1+m_extra_perimeters) , loops - loops2);

                wipe_contour_2(writer, loops3 * alpha, extrude_speed_perimeter);
            }

        }

        // This is rather crude, is there a situation where the wipe tower
        // would present with more than 1 tool change, on the IDEX machine?

        switch (layer.tool_changes.size())
        {
        case 1:
            // there is one tool change
            change_tool(writer, layer.tool_changes.front().new_tool);
            break;
        case 0:
            break;
        default:
            throw Slic3r::InvalidArgument("WipeTower: invalid parameter");
            break;
        }

        // complete layer
        wipe_lines_1(writer, extrude_speed_infill);

        layer_tcr = construct_tcr(writer, false, old_tool);
        layer_result.emplace_back(std::move(layer_tcr));
		result.emplace_back(std::move(layer_result));

	}//layer
}


void
WipeTower::change_tool(WipeTowerWriter& writer, int new_tool)
{
    writer.append(";----------------------\n");
    writer.append(";--  TOOL CHANGE START \n");
    writer.set_initial_position({0.f,0.f,}, m_wipe_tower_width, m_wipe_tower_depth, 0);

    if (new_tool != -1) {
        writer.comment_with_value(" toolchange #", m_num_tool_changes + 1); // the number is zero-based
        writer.append(std::string("; material : " + (m_current_tool < m_filpar.size() ? m_filpar[m_current_tool].material : "(NONE)") + " -> " + m_filpar[new_tool].material + "\n").c_str());
        writer.comment_with_value(" current tool ", m_current_tool);
        writer.comment_with_value(" new     tool ", new_tool);

        m_num_tool_changes ++;
    }

    writer.flush_planner_queue();
    this->m_current_tool = new_tool;

    writer.speed_override_backup();
    writer.speed_override(100);

    // Ask the writer how much of the filament was consumed:
    if (m_current_tool < m_used_filament_length.size())
        m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

#if 0
    // This is where we want to place the custom gcodes. We will use placeholders for this.
    // These will be substituted by the actual gcodes when the gcode is generated.
    //writer.append("[end_filament_gcode]\n");
    writer.append("[toolchange_gcode_from_wipe_tower_generator]\n");
#else
    // Perform the switch in-place, that way we can be sure there's no issue with the
    // gcode generation getting confused with the tool changes.
    writer.append("T" + std::to_string(new_tool) + "\n");
#endif


    // Travel to where we assume we are. Custom toolchange or some special T code handling (parking extruder etc)
    // gcode could have left the extruder somewhere, we cannot just start extruding. We should also inform the
    // postprocessor that we absolutely want to have this in the gcode, even if it thought it is the same as before.
    Vec2f current_pos = writer.pos_rotated();
    writer.feedrate(m_travel_speed * 60.f) // see https://github.com/prusa3d/PrusaSlicer/issues/5483
            .append(std::string("G1 X") + Slic3r::float_to_string_decimal_point(current_pos.x())
                                +  " Y"  + Slic3r::float_to_string_decimal_point(current_pos.y())
                                + never_skip_tag() + "\n"
    );

    writer.append("[deretraction_from_wipe_tower_generator]\n");

    // The toolchange Tn command will be inserted later, only in case that the user does
    // not provide a custom toolchange gcode.
    writer.set_tool(new_tool); // This outputs nothing, the writer just needs to know the tool has changed.
    writer.flush_planner_queue();
    m_current_tool = new_tool;

    writer.speed_override_restore();
    writer.feedrate(m_travel_speed * 60.f);
    writer.flush_planner_queue();
    writer.reset_extruder();

    writer.set_extrusion_flow( m_layer_height * ( current_nozzle_diameter() - m_layer_height * (1.f-float(M_PI)/4.f)) / filament_area() );
    writer.change_analyzer_line_width( current_nozzle_diameter() );

    writer.append(";--  TOOL CHANGE END   \n");
    writer.append(";----------------------\n");
}

void
WipeTower::wipe_contour_2(WipeTowerWriter& writer, int loops, int extrude_speed)
{
    writer.append("; -- wipe tower's perimeter\n");
    writer.append("; -- extrusion flow: " + std::to_string(writer.get_extrusion_flow()) + "\n");
    writer.append("; -- extrusion speed: " + std::to_string(extrude_speed) + "\n");

    // h-----------------g
    // | d-------------c |
    // | |             | |
    // | |             | |
    // | |             | |
    // | a-------------b |
    // e-----------------f

    Point a = Point::new_scale(0,0);
    Point b = Point::new_scale(m_wipe_tower_width,0);
    Point c = Point::new_scale(m_wipe_tower_width,m_wipe_tower_depth);
    Point d = Point::new_scale(0,m_wipe_tower_depth);

    {   // the first loop (always)
        Polygon poly {a,b,c,d};
        int cp = poly.closest_point_index(Point::new_scale(writer.x(), writer.y()));
        writer.travel(unscale(poly.points[cp]).cast<float>(), m_travel_speed * 60.f);
        for (int i=cp+1; true; ++i ) {
            if (i==int(poly.points.size()))
                i = 0;
            writer.extrude(unscale(poly.points[i]).cast<float>(), extrude_speed);
            if (i == cp)
                break;
        }
    }
    loops --;
    {
        float spacing = current_nozzle_diameter() - m_layer_height * (1. - M_PI_4);

        Polygon poly {a,b,c,d};
        for (int i = 0; i < loops; i++) {
            poly = offset(poly, scale_(spacing)).front();
            int cp = poly.closest_point_index(Point::new_scale(writer.x(), writer.y()));
            writer.travel(unscale(poly.points[cp]).cast<float>(), m_travel_speed * 60.f);
            for (int i=cp+1; true; ++i ) {
                if (i==int(poly.points.size()))
                    i = 0;
                writer.extrude(unscale(poly.points[i]).cast<float>(), extrude_speed);
                if (i == cp)
                    break;
            }
        }
    }
}

void
WipeTower::wipe_lines_1(WipeTowerWriter& writer, int extrude_speed)
{
    writer.append("; -- wipe tower's infill\n");
    writer.append("; -- extrusion flow:  " + std::to_string(writer.get_extrusion_flow()) + "\n");
    writer.append("; -- extrusion speed: " + std::to_string(extrude_speed) + "\n");

    //   d-------------c
    //   |/////////////|
    //   |/////////////|
    //   |/////////////|
    //   a-------------b
    //

    Point a = Point::new_scale(0,0);
    Point b = Point::new_scale(m_wipe_tower_width,0);
    Point c = Point::new_scale(m_wipe_tower_width,m_wipe_tower_depth);
    Point d = Point::new_scale(0,m_wipe_tower_depth);

    std::unique_ptr<Fill> filler(Fill::new_from_type("rectilinear"));
    Slic3r::ExPolygon expolygon({a,b,c,d});
    Polylines polylines;
    Surface surface(stBottom, expolygon);

    // width, height, nozzle_dmr
    auto flow = Slic3r::Flow(current_nozzle_diameter(), m_layer_height, current_nozzle_diameter());

    const float spacing = current_nozzle_diameter() - m_layer_height*float(1.-M_PI_4);
    filler->angle = Geometry::deg2rad(45.f);
    filler->spacing = flow.spacing();
    FillParams params;
    params.density = m_density / 100.f;
    filler->bounding_box = get_extents(expolygon);

    if (is_first_layer()) {
        params.density = 1.f;
    }

    polylines = filler->fill_surface(&surface, params);

    for (const Polyline& line: polylines) {
        writer.travel(unscale(line.points.front()).cast<float>(), m_travel_speed * 60.f);
        for (auto& point: line.points) {
            writer.extrude(unscale(point).cast<float>(), extrude_speed);
        }
    }
}


std::vector<std::pair<float, float>> WipeTower::get_z_and_depth_pairs() const
{
    std::vector<std::pair<float, float>> out = {{0.f, m_wipe_tower_depth}};
    for (const WipeTowerInfo& layer : m_plan) {
        out.emplace_back(layer.z, m_wipe_tower_depth);
    }
    return out;
}

} // namespace Slic3r
