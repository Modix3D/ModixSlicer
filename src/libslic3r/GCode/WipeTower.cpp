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
#include <functional>

#include "ClipperUtils.hpp"
#include "GCodeProcessor.hpp"
#include "BoundingBox.hpp"
#include "LocalesUtils.hpp"
#include "Geometry.hpp"
#include "Surface.hpp"
#include "Fill/FillRectilinear.hpp"
#include "libslic3r/Flow.hpp"

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
		m_extrusion_flow(0.f),
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

    WipeTowerWriter& 			 set_position(const Vec2f &pos) { m_current_pos = pos; return *this; }

    WipeTowerWriter&				 set_initial_tool(size_t tool) { m_current_tool = tool; return *this; }

	WipeTowerWriter&				 set_z(float z) 
		{ m_current_z = z; return *this; }

	WipeTowerWriter& 			 set_extrusion_flow(float flow)
		{ m_extrusion_flow = flow; return *this; }

	WipeTowerWriter&				 set_y_shift(float shift) {
        m_current_pos.y() -= shift-m_y_shift;
        m_y_shift = shift;
        return (*this);
    }

    WipeTowerWriter&            disable_linear_advance() {
        if (m_gcode_flavor == gcfRepRapSprinter || m_gcode_flavor == gcfRepRapFirmware)
            m_gcode += (std::string("M572 D") + std::to_string(m_current_tool) + " S0\n");
        else if (m_gcode_flavor == gcfKlipper)
            m_gcode += "SET_PRESSURE_ADVANCE ADVANCE=0\n";
        else
            m_gcode += "M900 K0\n";
        return *this;
    }

	// Suppress / resume G-code preview in Slic3r. Slic3r will have difficulty to differentiate the various
	// filament loading and cooling moves from normal extrusion moves. Therefore the writer
	// is asked to suppres output of some lines, which look like extrusions.
#if ENABLE_GCODE_VIEWER_DATA_CHECKING
    WipeTowerWriter& suppress_preview() { change_analyzer_line_width(0.f); m_preview_suppressed = true; return *this; }
    WipeTowerWriter& resume_preview() { change_analyzer_line_width(m_default_analyzer_line_width); m_preview_suppressed = false; return *this; }
#else
    WipeTowerWriter& 			 suppress_preview() { m_preview_suppressed = true; return *this; }
	WipeTowerWriter& 			 resume_preview()   { m_preview_suppressed = false; return *this; }
#endif // ENABLE_GCODE_VIEWER_DATA_CHECKING

	WipeTowerWriter& 			 feedrate(float f)
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



WipeTower::WipeTower(const PrintConfig& config, const PrintRegionConfig& default_region_config, const std::vector<std::vector<float>>& wiping_matrix, size_t initial_tool) :
    m_semm(config.single_extruder_multi_material.value),
    m_wipe_tower_pos(config.wipe_tower_x, config.wipe_tower_y),
    m_wipe_tower_width(float(config.wipe_tower_width)),
    m_wipe_tower_rotation_angle(float(config.wipe_tower_rotation_angle)),
    m_wipe_tower_brim_width(float(config.wipe_tower_brim_width)),
    m_wipe_tower_cone_angle(float(0.)),
    m_extra_spacing(float(1.)),
    m_y_shift(0.f),
    m_z_pos(0.f),
    m_bridging(float(config.wipe_tower_bridging)),
    m_no_sparse_layers(config.wipe_tower_no_sparse_layers),
    m_gcode_flavor(config.gcode_flavor),
    m_travel_speed(config.travel_speed),
    m_travel_speed_z(config.travel_speed_z),
    m_infill_speed(default_region_config.infill_speed),
    m_perimeter_speed(default_region_config.perimeter_speed),
    m_current_tool(initial_tool),
    wipe_volumes(wiping_matrix),
	m_extra_perimeters(std::max(0, config.wipe_tower_perimeters-1)),
	m_minimum_depth(float(config.wipe_tower_depth)),
	m_density(float(config.wipe_tower_density)),
    m_perimeter_extruder(config.wipe_tower_extruder)
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


    // If this is a single extruder MM printer, we will use all the SE-specific config values.
    // Otherwise, the defaults will be used to turn off the SE stuff.
    if (m_semm) {
        m_cooling_tube_retraction = float(config.cooling_tube_retraction);
        m_cooling_tube_length     = float(config.cooling_tube_length);
        m_parking_pos_retraction  = float(config.parking_pos_retraction);
        m_extra_loading_move      = float(config.extra_loading_move);
        m_set_extruder_trimpot    = config.high_current_on_filament_swap;
    }

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
    m_filpar[idx].is_soluble = config.wipe_tower_extruder == 0 ? config.filament_soluble.get_at(idx) : (idx != size_t(config.wipe_tower_extruder - 1));
    m_filpar[idx].temperature = config.temperature.get_at(idx);
    m_filpar[idx].first_layer_temperature = config.first_layer_temperature.get_at(idx);
    m_filpar[idx].is_soluble = 0;


    // If this is a single extruder MM printer, we will use all the SE-specific config values.
    // Otherwise, the defaults will be used to turn off the SE stuff.
    if (m_semm) {
        m_filpar[idx].loading_speed           = float(config.filament_loading_speed.get_at(idx));
        m_filpar[idx].loading_speed_start     = float(config.filament_loading_speed_start.get_at(idx));
        m_filpar[idx].unloading_speed         = float(config.filament_unloading_speed.get_at(idx));
        m_filpar[idx].unloading_speed_start   = float(config.filament_unloading_speed_start.get_at(idx));
        m_filpar[idx].delay                   = float(config.filament_toolchange_delay.get_at(idx));
        m_filpar[idx].cooling_moves           = config.filament_cooling_moves.get_at(idx);
        m_filpar[idx].cooling_initial_speed   = float(config.filament_cooling_initial_speed.get_at(idx));
        m_filpar[idx].cooling_final_speed     = float(config.filament_cooling_final_speed.get_at(idx));
    }

    m_filpar[idx].filament_area = float((M_PI/4.f) * pow(config.filament_diameter.get_at(idx), 2)); // all extruders are assumed to have the same filament diameter at this point
    float nozzle_diameter = float(config.nozzle_diameter.get_at(idx));
    m_filpar[idx].nozzle_diameter = nozzle_diameter; // to be used in future with (non-single) multiextruder MM

    float max_vol_speed = float(config.filament_max_volumetric_speed.get_at(idx));
    if (max_vol_speed!= 0.f)
        m_filpar[idx].max_e_speed = (max_vol_speed / filament_area());

	m_perimeter_width = float(default_region_config.wipe_tower_perimeter_extrusion_width);
	m_infill_width = float(default_region_config.wipe_tower_infill_extrusion_width);
	if (m_perimeter_width < WT_EPSILON) {
		m_perimeter_width = nozzle_diameter;
	}
	if (m_infill_width < WT_EPSILON) {
		m_infill_width = nozzle_diameter;
	}

    if (m_semm) {
        std::istringstream stream{config.filament_ramming_parameters.get_at(idx)};
        float speed = 0.f;
        stream >> m_filpar[idx].ramming_line_width_multiplicator >> m_filpar[idx].ramming_step_multiplicator;
        m_filpar[idx].ramming_line_width_multiplicator /= 100;
        m_filpar[idx].ramming_step_multiplicator /= 100;
        while (stream >> speed)
            m_filpar[idx].ramming_speed.push_back(speed);
        // ramming_speed now contains speeds to be used for every 0.25s piece of the ramming line.
        // This allows to have the ramming flow variable. The 0.25s value is how it is saved in config
        // and the same time step has to be used when the ramming is performed.
    } else {
        // We will use the same variables internally, but the correspondence to the configuration options will be different.
        float vol  = config.filament_multitool_ramming_volume.get_at(idx);
        float flow = config.filament_multitool_ramming_flow.get_at(idx);
        m_filpar[idx].multitool_ramming = config.filament_multitool_ramming.get_at(idx);
        m_filpar[idx].ramming_line_width_multiplicator = 2.;
        m_filpar[idx].ramming_step_multiplicator = 1.;

        // Now the ramming speed vector. In this case it contains just one value (flow).
        // The time is calculated and saved separately. This is here so that the MM ramming
        // is not limited by the 0.25s granularity - it is not possible to create a SEMM-style
        // ramming_speed vector that would respect both the volume and flow (because of 
        // rounding issues with small volumes and high flow).
        m_filpar[idx].ramming_speed.push_back(flow);
        m_filpar[idx].multitool_ramming_time = vol/flow;
    }

    m_used_filament_length.resize(std::max(m_used_filament_length.size(), idx + 1)); // makes sure that the vector is big enough so we don't have to check later
}



// Returns gcode to prime the nozzles at the front edge of the print bed.
std::vector<WipeTower::ToolChangeResult> WipeTower::prime(float, const std::vector<unsigned int>&, bool)
{
	return {};
}

// Change the tool, set a speed override for soluble and flex materials.
void WipeTower::toolchange_Change(
	WipeTowerWriter &writer,
    const size_t 	new_tool,
    const std::string&  new_material)
{
    // Ask the writer about how much of the old filament we consumed:
    if (m_current_tool < m_used_filament_length.size())
    	m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

    // This is where we want to place the custom gcodes. We will use placeholders for this.
    // These will be substituted by the actual gcodes when the gcode is generated.
    //writer.append("[end_filament_gcode]\n");
    writer.append("[toolchange_gcode_from_wipe_tower_generator]\n");

    // Travel to where we assume we are. Custom toolchange or some special T code handling (parking extruder etc)
    // gcode could have left the extruder somewhere, we cannot just start extruding. We should also inform the
    // postprocessor that we absolutely want to have this in the gcode, even if it thought it is the same as before.
    Vec2f current_pos = writer.pos_rotated();
    writer.feedrate(m_travel_speed * 60.f) // see https://github.com/prusa3d/PrusaSlicer/issues/5483
          .append(std::string("G1 X") + Slic3r::float_to_string_decimal_point(current_pos.x())
                             +  " Y"  + Slic3r::float_to_string_decimal_point(current_pos.y())
                             + never_skip_tag() + "\n"
    );

    writer.append("[deretraction_from_wipe_tower_generator]");

    // The toolchange Tn command will be inserted later, only in case that the user does
    // not provide a custom toolchange gcode.
	writer.set_tool(new_tool); // This outputs nothing, the writer just needs to know the tool has changed.
    //writer.append("[start_filament_gcode]\n");

	writer.flush_planner_queue();
	m_current_tool = new_tool;
}

void WipeTower::toolchange_Load(
	WipeTowerWriter &writer,
	const box_coordinates  &cleaning_box)
{
    if (m_semm && (m_parking_pos_retraction != 0 || m_extra_loading_move != 0)) {
        float xl = cleaning_box.ld.x() + m_perimeter_width * 0.75f;
        float xr = cleaning_box.rd.x() - m_perimeter_width * 0.75f;
        float oldx = writer.x();	// the nozzle is in place to do the first wiping moves, we will remember the position

        // Load the filament while moving left / right, so the excess material will not create a blob at a single position.
        float turning_point = ( oldx-xl < xr-oldx ? xr : xl );
        float edist = m_parking_pos_retraction+m_extra_loading_move;

        writer.append("; CP TOOLCHANGE LOAD\n")
              .suppress_preview()
              .load(0.2f * edist, 60.f * m_filpar[m_current_tool].loading_speed_start)
              .load_move_x_advanced(turning_point, 0.7f * edist,        m_filpar[m_current_tool].loading_speed)  // Fast phase
              .load_move_x_advanced(oldx,          0.1f * edist, 0.1f * m_filpar[m_current_tool].loading_speed)  // Super slow*/

              .travel(oldx, writer.y()) // in case last move was shortened to limit x feedrate
              .resume_preview();

        // Reset the extruder current to the normal value.
        if (m_set_extruder_trimpot)
            writer.set_extruder_trimpot(550);
    }
}

// Wipe the newly loaded filament until the end of the assigned wipe area.
void WipeTower::toolchange_Wipe(
	WipeTowerWriter &writer,
	const box_coordinates  &cleaning_box,
	float wipe_volume)
{
    // Modix --
    //
    //   This section switches to infill extrusion parameters.
    // Once wiping is done, switch back to perimeters extrusion parameters.
    //

	writer.change_analyzer_line_width(m_infill_width);
	// Increase flow on first layer, slow down print.
    writer.set_extrusion_flow(m_infill_extrusion_flow * (is_first_layer() ? 1.18f : 1.f))
		  .append("; CP TOOLCHANGE WIPE\n");
	const float& xl = cleaning_box.ld.x();
	const float& xr = cleaning_box.rd.x();

    // wipe until the end of the assigned area.

	float dy = (is_first_layer() ? 1.f : 100.f/m_density) * m_infill_width; // Don't use the extra spacing for the first layer.
    // All the calculations in all other places take the spacing into account for all the layers.

    const float target_speed = is_first_layer() ? m_first_layer_speed * 60.f : m_infill_speed * 60.f;
    float wipe_speed = 0.33f * target_speed;

    // if there is less than 2.5*m_infill_width to the edge, advance straightaway (there is likely a blob anyway)
    if ((m_left_to_right ? xr-writer.x() : writer.x()-xl) < 2.5f*m_infill_width) {
        writer.travel((m_left_to_right ? xr-m_infill_width : xl+m_perimeter_width),writer.y()+dy);
        m_left_to_right = !m_left_to_right;
    }
    
    // now the wiping itself:
	for (int i = 0; true; ++i)	{
		if (i!=0) {
            if      (wipe_speed < 0.34f * target_speed) wipe_speed = 0.375f * target_speed;
            else if (wipe_speed < 0.377 * target_speed) wipe_speed = 0.458f * target_speed;
            else if (wipe_speed < 0.46f * target_speed) wipe_speed = 0.875f * target_speed;
            else wipe_speed = std::min(target_speed, wipe_speed + 50.f);
		}

		if (m_left_to_right)
            writer.extrude(xr - (i % 4 == 0 ? 0 : 1.5f*m_infill_width), writer.y(), wipe_speed);
		else
            writer.extrude(xl + (i % 4 == 1 ? 0 : 1.5f*m_infill_width), writer.y(), wipe_speed);

        if (writer.y()+float(EPSILON) > cleaning_box.lu.y()-0.5f*m_infill_width)
            break;		// in case next line would not fit

		// stepping to the next line:
        writer.extrude(writer.x() + (i % 4 == 0 ? -1.f : (i % 4 == 1 ? 1.f : 0.f)) * 1.5f*m_infill_width, writer.y() + dy);
		m_left_to_right = !m_left_to_right;
	}

    // We may be going back to the model - wipe the nozzle. If this is followed
    // by finish_layer, this wipe path will be overwritten.
    writer.add_wipe_point(writer.x(), writer.y())
          .add_wipe_point(writer.x(), writer.y() - dy)
          .add_wipe_point(! m_left_to_right ? m_wipe_tower_width : 0.f, writer.y() - dy);

    if (m_layer_info != m_plan.end() && m_current_tool != m_layer_info->tool_changes.back().new_tool)
        m_left_to_right = !m_left_to_right;

    writer.change_analyzer_line_width(m_perimeter_width);
    writer.set_extrusion_flow(m_extrusion_flow); // Reset the extrusion flow.
}

// Static method to get the radius and x-scaling of the stabilizing cone base.
std::pair<double, double> WipeTower::get_wipe_tower_cone_base(double width, double height, double depth, double angle_deg)
{
    double R = std::tan(Geometry::deg2rad(angle_deg/2.)) * height;
    double fake_width = 0.66 * width;
    double diag = std::hypot(fake_width / 2., depth / 2.);
    double support_scale = 1.;
    if (R > diag) {
        double w = fake_width;
        double sin = 0.5 * depth / diag;
        double tan = depth / w;
        double t = (R - diag) * sin;
        support_scale = (w / 2. + t / tan + t * tan) / (w / 2.);
    }
    return std::make_pair(R, support_scale);
}

// Static method to extract wipe_volumes[from][to] from the configuration.
std::vector<std::vector<float>> WipeTower::extract_wipe_volumes(const PrintConfig& config)
{
    // Get wiping matrix to get number of extruders and convert vector<double> to vector<float>:
    std::vector<float> wiping_matrix(cast<float>(config.wiping_volumes_matrix.values));

    // The values shall only be used when SEMM is enabled. The purging for other printers
    // is determined by filament_minimal_purge_on_wipe_tower.
    if (! config.single_extruder_multi_material.value)
        std::fill(wiping_matrix.begin(), wiping_matrix.end(), 0.f);

    // Extract purging volumes for each extruder pair:
    std::vector<std::vector<float>> wipe_volumes;
    const unsigned int number_of_extruders = (unsigned int)(sqrt(wiping_matrix.size())+EPSILON);
    for (unsigned int i = 0; i<number_of_extruders; ++i)
        wipe_volumes.push_back(std::vector<float>(wiping_matrix.begin()+i*number_of_extruders, wiping_matrix.begin()+(i+1)*number_of_extruders));

    // Also include filament_minimal_purge_on_wipe_tower. This is needed for the preview.
    for (unsigned int i = 0; i<number_of_extruders; ++i)
        for (unsigned int j = 0; j<number_of_extruders; ++j)
            wipe_volumes[i][j] = std::max<float>(wipe_volumes[i][j], config.filament_minimal_purge_on_wipe_tower.get_at(j));

    return wipe_volumes;
}

// Appends a toolchange into m_plan and calculates neccessary depth of the corresponding box
void WipeTower::plan_toolchange(float _z, float _height, unsigned int _old_tool,
                                unsigned int _new_tool, float)
{

    // if (_old_tool == _new_tool)
    //     return;

    // Check if the layer is unreasonably thin, skip it (but update the last entry's target).
    if (_height < 0.05f) {
        printf("thin layer: z=%f h=%f\n", _z, _height);
        // m_plan.back().tool_changes[0].new_tool = _new_tool;
        // return;
    }

    WipeTowerInfo wti (_z, _height);
    m_plan.push_back(wti);

    WipeTowerInfo::ToolChange tc (_old_tool, _new_tool);
    m_plan.back().tool_changes.push_back(tc);




// 	assert(m_plan.empty() || m_plan.back().z <= z_par + WT_EPSILON);	// refuses to add a layer below the last one

//     printf("plan_toolchange z=%f h=%f ot=%u nt=%u\n", z_par, layer_height_par, old_tool, new_tool);

// 	if (m_plan.empty() || m_plan.back().z + WT_EPSILON < z_par) // if we moved to a new layer, we'll add it to m_plan first
// 		m_plan.push_back(WipeTowerInfo(z_par, layer_height_par));

//     if (m_first_layer_idx == size_t(-1) && (! m_no_sparse_layers || old_tool != new_tool || m_plan.size() == 1))
//         m_first_layer_idx = m_plan.size() - 1;

//     if (old_tool == new_tool)	// new layer without toolchanges - we are done
//         return;

// 	// this is an actual toolchange - let's calculate depth to reserve on the wipe tower
//     float depth = 0.f;
// 	float width = m_wipe_tower_width - 3*m_perimeter_width; 
// 	float length_to_extrude = volume_to_length(0.25f * std::accumulate(m_filpar[old_tool].ramming_speed.begin(), m_filpar[old_tool].ramming_speed.end(), 0.f),
// 										m_perimeter_width * m_filpar[old_tool].ramming_line_width_multiplicator,
// 										layer_height_par);
// 	depth = (int(length_to_extrude / width) + 1) * (m_perimeter_width * m_filpar[old_tool].ramming_line_width_multiplicator * m_filpar[old_tool].ramming_step_multiplicator);
//     float ramming_depth = depth;
//     length_to_extrude = width*((length_to_extrude / width)-int(length_to_extrude / width)) - width;
//     float first_wipe_line = -length_to_extrude;
//     length_to_extrude += volume_to_length(wipe_volume, m_perimeter_width, layer_height_par);
//     length_to_extrude = std::max(length_to_extrude,0.f);

// #if 0
// 	depth += (int(length_to_extrude / width) + 1) * m_perimeter_width;
// 	depth *= m_extra_spacing;
// #else
// 	depth  = m_minimum_depth - m_perimeter_width;
// #endif

// 	m_plan.back().tool_changes.push_back(WipeTowerInfo::ToolChange(old_tool, new_tool, depth, ramming_depth, first_wipe_line, wipe_volume));
}

static WipeTower::ToolChangeResult merge_tcr(WipeTower::ToolChangeResult& first,
                                             WipeTower::ToolChangeResult& second)
{
    assert(first.new_tool == second.initial_tool);
    WipeTower::ToolChangeResult out = first;
    if (first.end_pos != second.start_pos)
        out.gcode += "G1 X" + Slic3r::float_to_string_decimal_point(second.start_pos.x(), 3)
                     + " Y" + Slic3r::float_to_string_decimal_point(second.start_pos.y(), 3)
                     + " F7200\n";
    out.gcode += second.gcode;
    out.extrusions.insert(out.extrusions.end(), second.extrusions.begin(), second.extrusions.end());
    out.end_pos = second.end_pos;
    out.wipe_path = second.wipe_path;
    out.initial_tool = first.initial_tool;
    out.new_tool = second.new_tool;
    return out;
}

void WipeTower::wipe_lines_1(WipeTowerWriter& writer)
{
    // XXX switch to infill width....
    // XXX how much filament consumed....
    // XXX respect speed settings....

    //   d-------------c
    //   |             |
    //   |             |
    //   |             |
    //   a-------------b
    //

    Point a = Point::new_scale(0,0);
    Point b = Point::new_scale(m_wipe_tower_width,0);
    Point c = Point::new_scale(m_wipe_tower_width,m_wipe_tower_depth);
    Point d = Point::new_scale(0,m_wipe_tower_depth);

    a.x() += m_perimeter_width;
    a.y() += m_perimeter_width;
    b.x() -= m_perimeter_width;
    b.y() += m_perimeter_width;
    c.x() -= m_perimeter_width;
    c.y() -= m_perimeter_width;
    d.x() += m_perimeter_width;
    d.y() -= m_perimeter_width;

    std::unique_ptr<Fill> filler(Fill::new_from_type("rectilinear"));
    Slic3r::ExPolygon expolygon({a,b,c,d});
    Polylines polylines;
    Surface surface(stBottom, expolygon);

    // width, height, nozzle_dmr
    auto flow = Slic3r::Flow(m_perimeter_width, m_layer_height, m_perimeter_width);

    const float spacing = m_perimeter_width - m_layer_height*float(1.-M_PI_4);
    filler->angle = Geometry::deg2rad(45.f);
    filler->spacing = flow.spacing();
    FillParams params;
    params.density = m_density / 100.f;
    filler->bounding_box = get_extents(expolygon);

    if (is_first_layer())
        params.density = 1.f;

    polylines = filler->fill_surface(&surface, params);

    for (const Polyline& line: polylines) {
        writer.travel(unscale(line.points.front()).cast<float>());
        for (auto& point: line.points) {
            writer.extrude(unscale(point).cast<float>());
        }
    }
}

// Processes vector m_plan and calls respective functions to generate G-code for the wipe tower
// Resulting ToolChangeResults are appended into vector "result"
void WipeTower::generate(std::vector<std::vector<WipeTower::ToolChangeResult>> &result)
{

	if (m_plan.empty())
        return;


    auto change_tool = [this](WipeTowerWriter& writer,
                              std::optional<unsigned int> new_tool) {
        writer.append(";----------------------\n");
        writer.append(";--  TOOL CHANGE START \n");
        writer.set_initial_position({0.f,0.f,}, m_wipe_tower_width, m_wipe_tower_depth, 0);

        if (new_tool.has_value()) {
            writer.comment_with_value(" toolchange #", m_num_tool_changes + 1); // the number is zero-based
            writer.append(std::string("; material : " + (m_current_tool < m_filpar.size() ? m_filpar[m_current_tool].material : "(NONE)") + " -> " + m_filpar[new_tool.value()].material + "\n").c_str());
        }

        writer.flush_planner_queue();
        this->m_current_tool = new_tool.value_or(-1);

        writer.speed_override_backup();
        writer.speed_override(100);

        // Ask the writer how much of the filament was consumed:
        if (m_current_tool < m_used_filament_length.size())
            m_used_filament_length[m_current_tool] += writer.get_and_reset_used_filament_length();

        // This is where we want to place the custom gcodes. We will use placeholders for this.
        // These will be substituted by the actual gcodes when the gcode is generated.
        //writer.append("[end_filament_gcode]\n");
        
        //writer.append("[toolchange_gcode_from_wipe_tower_generator]\n");
        // XXX
        // Can't use the macro directly because it is updated later.
        std::string T = "T" + std::to_string(new_tool.value_or(m_current_tool)) + "\n";
        writer.append( T );

        // Travel to where we assume we are. Custom toolchange or some special T code handling (parking extruder etc)
        // gcode could have left the extruder somewhere, we cannot just start extruding. We should also inform the
        // postprocessor that we absolutely want to have this in the gcode, even if it thought it is the same as before.
        Vec2f current_pos = writer.pos_rotated();
        writer.feedrate(m_travel_speed * 60.f) // see https://github.com/prusa3d/PrusaSlicer/issues/5483
                .append(std::string("G1 X") + Slic3r::float_to_string_decimal_point(current_pos.x())
                                    +  " Y"  + Slic3r::float_to_string_decimal_point(current_pos.y())
                                    + never_skip_tag() + "\n"
        );

        writer.append(";[deretraction_from_wipe_tower_generator]\n");

        // The toolchange Tn command will be inserted later, only in case that the user does
        // not provide a custom toolchange gcode.
        writer.set_tool(new_tool.value()); // This outputs nothing, the writer just needs to know the tool has changed.
        //writer.append("[start_filament_gcode]\n");

        writer.flush_planner_queue();
        m_current_tool = new_tool.value_or(-1);

        writer.speed_override_restore();
        writer.feedrate(m_travel_speed * 60.f);
        writer.flush_planner_queue();
        writer.reset_extruder();
        writer.append(";--  TOOL CHANGE END   \n");
        writer.append(";----------------------\n");
    };

    m_layer_info = m_plan.begin();
    m_current_height = 0.f;

    // we don't know which extruder to start with - we'll set it according to the first toolchange
    for (const auto& layer : m_plan) {
        if (!layer.tool_changes.empty()) {
            m_current_tool = layer.tool_changes.front().old_tool;
            break;
        }
    }

    for (auto& used : m_used_filament_length) // reset used filament stats
        used = 0.f;

    m_old_temperature = -1; // reset last temperature written in the gcode
    m_wipe_tower_depth = m_minimum_depth;

	for (const WipeTower::WipeTowerInfo& wtlayer : m_plan)
	{
        wtlayer.debug();


        std::vector<WipeTower::ToolChangeResult> layer_result;
        set_layer(wtlayer.z, wtlayer.height);

        ToolChangeResult layer_tcr;

        assert(! this->layer_finished());
        m_current_layer_finished = true;

        size_t old_tool = m_current_tool;

        WipeTowerWriter writer(m_layer_height, m_perimeter_width, m_gcode_flavor, m_filpar);
        writer.set_extrusion_flow(m_extrusion_flow)
        .set_z(m_z_pos)
        .set_initial_tool(m_current_tool);

        // Change to tool needed for perimeter; if not, current tool.
        //
        change_tool(writer, 1);


        // Print a brim (always)
        //
        const int brim_loops = 50;
        if (is_first_layer()) {

            Vec2f ld(0.f,0.f);
            Vec2f m(m_perimeter_width,m_perimeter_width);
            for (int i = 0; i < brim_loops; i++) {
                writer.rectangle(ld-i*m, m_wipe_tower_width+2*i*m_perimeter_width, m_wipe_tower_depth+2*i*m_perimeter_width);
            }

        } else {

            // Print perims
            static int loops = 2*(brim_loops-1);
            Vec2f ld(0.f,0.f);
            Vec2f m(m_perimeter_width,m_perimeter_width);
            for (int i = 0; i < loops/2; i++) {
                writer.rectangle(ld-i*m, m_wipe_tower_width+2*i*m_perimeter_width, m_wipe_tower_depth+2*i*m_perimeter_width);
            }
            (loops > 6)? loops -- : 0;

        }

        if (wtlayer.tool_changes.size() > 1) {
            throw Slic3r::InvalidArgument("Error: WipeTower invalid parameter");
        }

        if (wtlayer.tool_changes.size() == 1)
        {
            // Change to final tool
            //
            change_tool(writer, wtlayer.tool_changes.front().new_tool);
            wipe_lines_1(writer);
        } else {
            // If there is no toolswitch, complete layer.
            wipe_lines_1(writer);
        }

        layer_tcr = construct_tcr(writer, false, old_tool);
        layer_result.emplace_back(std::move(layer_tcr));
		result.emplace_back(std::move(layer_result));
	}
}



std::vector<std::pair<float, float>> WipeTower::get_z_and_depth_pairs() const
{
    std::vector<std::pair<float, float>> out = {{0.f, m_wipe_tower_depth}};
    for (const auto& wti: m_plan) {
        out.emplace_back(wti.z, m_wipe_tower_depth);
    }
    return out;
}

} // namespace Slic3r
