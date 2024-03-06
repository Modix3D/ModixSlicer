///|/ Copyright (c) Prusa Research 2017 - 2023 Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_GCode_WipeTower_hpp_
#define slic3r_GCode_WipeTower_hpp_

#include <cmath>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>

#include "libslic3r/Point.hpp"

namespace Slic3r
{

class WipeTowerWriter;
class PrintConfig;
class PrintRegionConfig;
enum GCodeFlavor : unsigned char;



class WipeTower
{
public:
    static const std::string never_skip_tag() { return "_GCODE_WIPE_TOWER_NEVER_SKIP_TAG"; }

    struct Extrusion
    {
		Extrusion(const Vec2f &pos, float width, unsigned int tool) : pos(pos), width(width), tool(tool) {}
		// End position of this extrusion.
		Vec2f				pos;
		// Width of a squished extrusion, corrected for the roundings of the squished extrusions.
		// This is left zero if it is a travel move.
		float 			width;
		// Current extruder index.
		unsigned int    tool;
	};

	struct ToolChangeResult
	{
		// Print heigh of this tool change.
		float					print_z;
		float 					layer_height;
		// G-code section to be directly included into the output G-code.
		std::string				gcode;
		// For path preview.
		std::vector<Extrusion> 	extrusions;
		// Initial position, at which the wipe tower starts its action.
		// At this position the extruder is loaded and there is no Z-hop applied.
		Vec2f						start_pos;
		// Last point, at which the normal G-code generator of Slic3r shall continue.
		// At this position the extruder is loaded and there is no Z-hop applied.
		Vec2f						end_pos;
		// Time elapsed over this tool change.
		// This is useful not only for the print time estimation, but also for the control of layer cooling.
		float  				    elapsed_time;

        // Pass a polyline so that normal G-code generator can do a wipe for us.
        // The wipe cannot be done by the wipe tower because it has to pass back
        // a loaded extruder, so it would have to either do a wipe with no retraction
        // (leading to https://github.com/prusa3d/PrusaSlicer/issues/2834) or do
        // an extra retraction-unretraction pair.
        std::vector<Vec2f> wipe_path;

        // Initial tool
        int initial_tool;

        // New tool
        int new_tool;

		// Sum the total length of the extrusion.
		float total_extrusion_length_in_plane() {
			float e_length = 0.f;
			for (size_t i = 1; i < this->extrusions.size(); ++ i) {
				const Extrusion &e = this->extrusions[i];
				if (e.width > 0) {
					Vec2f v = e.pos - (&e - 1)->pos;
					e_length += v.norm();
				}
			}
			return e_length;
		}

		bool force_travel = false;
	};

    struct box_coordinates
    {
        box_coordinates(float left, float bottom, float width, float height) :
            ld(left        , bottom         ),
            lu(left        , bottom + height),
            rd(left + width, bottom         ),
            ru(left + width, bottom + height) {}
        box_coordinates(const Vec2f &pos, float width, float height) : box_coordinates(pos(0), pos(1), width, height) {}
        void translate(const Vec2f &shift) {
            ld += shift; lu += shift;
            rd += shift; ru += shift;
        }
        void translate(const float dx, const float dy) { translate(Vec2f(dx, dy)); }
        void expand(const float offset) {
            ld += Vec2f(- offset, - offset);
            lu += Vec2f(- offset,   offset);
            rd += Vec2f(  offset, - offset);
            ru += Vec2f(  offset,   offset);
        }
        void expand(const float offset_x, const float offset_y) {
            ld += Vec2f(- offset_x, - offset_y);
            lu += Vec2f(- offset_x,   offset_y);
            rd += Vec2f(  offset_x, - offset_y);
            ru += Vec2f(  offset_x,   offset_y);
        }
        Vec2f ld;  // left down
        Vec2f lu;	// left upper
        Vec2f rd;	// right lower
        Vec2f ru;  // right upper
    };

    // Construct ToolChangeResult from current state of WipeTower and WipeTowerWriter.
    // WipeTowerWriter is moved from !
    ToolChangeResult construct_tcr(WipeTowerWriter& writer,
                                   size_t old_tool) const;

	// x			-- x coordinates of wipe tower in mm ( left bottom corner )
	// y			-- y coordinates of wipe tower in mm ( left bottom corner )
	// width		-- width of wipe tower in mm ( default 60 mm - leave as it is )
	// wipe_area	-- space available for one toolchange in mm
    WipeTower(const PrintConfig& config,
	          const PrintRegionConfig& default_region_config,
			  size_t initial_tool);


	// Set the extruder properties.
    void set_extruder(size_t idx, const PrintConfig& config, const PrintRegionConfig& default_region_config);

	// Appends into internal structure m_plan containing info about the future wipe tower
	// to be used before building begins. The entries must be added ordered in z.
    void plan_toolchange(float z_par, float layer_height_par, unsigned int old_tool, unsigned int new_tool, float wipe_volume = 0.f);

	// Iterates through prepared m_plan, generates ToolChangeResults and appends them to "result"
	void generate(std::vector<std::vector<ToolChangeResult>> &result);
	void change_tool(WipeTowerWriter& writer, int new_tool);
	void wipe_contour_2(WipeTowerWriter& writer, int loops, int extrude_speed);
	void wipe_lines_1(WipeTowerWriter& writer, int extrude_speed);


    float get_depth() const { return m_wipe_tower_depth; }
	std::vector<std::pair<float, float>> get_z_and_depth_pairs() const;
    float get_brim_width() const { return m_wipe_tower_brim_width; }
	float get_wipe_tower_height() const { return m_wipe_tower_height; }





	// Switch to a next layer.
	void set_layer(float z, float layer_height)
	{
		m_z_pos = z;
		m_layer_height = layer_height;

        // Advance m_layer_info iterator, making sure we got it right
		while (!m_plan.empty() && m_layer_info->z < z - WT_EPSILON && m_layer_info+1 != m_plan.end())
			++m_layer_info;

		if (this->is_first_layer()) {
            m_num_layer_changes = 0;
            m_num_tool_changes 	= 0;
        } else
            ++ m_num_layer_changes;
	}

	// Return the wipe tower position.
	const Vec2f& 		 position() const { return m_wipe_tower_pos; }
	// Return the wipe tower width.
	float     		 width()    const { return m_wipe_tower_width; }
	// The wipe tower is finished, there should be no more tool changes or wipe tower prints.
	bool 	  		 finished() const { return m_max_color_changes == 0; }

	// Returns gcode to prime the nozzles at the front edge of the print bed.
	std::vector<ToolChangeResult> prime(
		// print_z of the first layer.
		float 						first_layer_height,
		// Extruder indices, in the order to be primed. The last extruder will later print the wipe tower brim, print brim and the object.
		const std::vector<unsigned int> &tools,
		// If true, the last priming are will be the same as the other priming areas, and the rest of the wipe will be performed inside the wipe tower.
		// If false, the last priming are will be large enough to wipe the last extruder sufficiently.
		bool 						last_wipe_inside_wipe_tower);

	// Returns gcode for a toolchange and a final print head position.
	// On the first layer, extrude a brim around the future wipe tower first.
    ToolChangeResult tool_change(size_t new_tool);

	// Fill the unfilled space with a sparse infill.
	// Call this method only if layer_finished() is false.
	ToolChangeResult finish_layer();

    std::vector<float> get_used_filament() const { return m_used_filament_length; }
    int get_number_of_toolchanges() const { return m_num_tool_changes; }

    struct FilamentParameters {
        std::string 	    material = "PLA";
        int  			    temperature = 0;
        int  			    first_layer_temperature = 0;
        float               max_e_speed = std::numeric_limits<float>::max();
        float               nozzle_diameter;
        float               filament_area;
    };

private:
    const float WT_EPSILON            = 1e-3f;
    float filament_area() const {
        return m_filpar[0].filament_area; // all extruders are assumed to have the same filament diameter at this point
    }


    Vec2f  m_wipe_tower_pos; 			// Left front corner of the wipe tower in mm.
    float  m_wipe_tower_width; 			// Width of the wipe tower.
    float  m_wipe_tower_depth 	= 0.f; 	// Depth of the wipe tower.
    float  m_wipe_tower_height  = 0.f;  // Height of the wipe tower.
    float  m_wipe_tower_brim_width      = 0.f; 	// Width of brim (mm) from config
    float  m_wipe_tower_rotation_angle = 0.f; // Wipe tower rotation angle in degrees (with respect to x axis)
    float  m_internal_rotation  = 0.f;
	float  m_y_shift			= 0.f;  // y shift passed to writer
	float  m_z_pos 				= 0.f;  // Current Z position.
	float  m_layer_height 		= 0.f; 	// Current layer height.
	size_t m_max_color_changes 	= 0; 	// Maximum number of color changes per layer.
    int    m_old_temperature    = -1;   // To keep track of what was the last temp that we set (so we don't issue the command when not neccessary)
    float  m_travel_speed       = 0.f;
    float  m_travel_speed_z     = 0.f;
	float  m_infill_speed       = 0.f;
	float  m_perimeter_speed    = 0.f;
    float  m_first_layer_speed  = 0.f;
    size_t m_first_layer_idx    = size_t(-1);
	size_t m_extra_perimeters   = 0;     // Extra perimeters to increase stability of the wipe tower
	float  m_density            = 0.f;
	float  m_brim_layers        = 0.f;   // Add layers to the brim - (%).
	int    m_wipe_tower_extruder;
    size_t m_current_tool  = 0;

    float current_nozzle_diameter()
    {
        return m_filpar[m_current_tool].nozzle_diameter;
    }

	// G-code generator parameters.
    GCodeFlavor     m_gcode_flavor;

    // Bed properties
    enum {
        RectangularBed,
        CircularBed,
        CustomBed
    } m_bed_shape;
    float m_bed_width; // width of the bed bounding box
    Vec2f m_bed_bottom_left; // bottom-left corner coordinates (for rectangular beds)

	// Extruder specific parameters.
    std::vector<FilamentParameters> m_filpar;

	// State of the wipe tower generator.
	unsigned int m_num_layer_changes = 0; // Layer change counter for the output statistics.
	unsigned int m_num_tool_changes  = 0; // Tool change change counter for the output statistics.

    bool is_first_layer() const { return size_t(m_layer_info - m_plan.begin()) == m_first_layer_idx; }

	// Calculates depth for all layers and propagates them downwards
	void plan_tower();

    // to store information about tool changes for a given layer
	struct WipeTowerInfo{
		struct ToolChange {
            size_t old_tool;
            size_t new_tool;
            ToolChange(size_t old, size_t newtool)
            : old_tool{old}, new_tool{newtool} {}
		};
		float z;		// z position of the layer
		float layer_height;

		std::vector<ToolChange> tool_changes;

		WipeTowerInfo(float _z, float _layer_height)
			: z{_z}, layer_height{_layer_height} {}
	};

	std::vector<WipeTowerInfo> m_plan; 	// Stores information about all layers and toolchanges for the future wipe tower (filled by plan_toolchange(...))
	std::vector<WipeTowerInfo>::iterator m_layer_info = m_plan.end();

    // Stores information about used filament length per extruder:
    std::vector<float> m_used_filament_length;

    // Return index of first toolchange that switches to non-soluble extruder
    // ot -1 if there is no such toolchange.
    int first_toolchange_to_nonsoluble(
            const std::vector<WipeTowerInfo::ToolChange>& tool_changes) const;
};




} // namespace Slic3r

#endif // slic3r_GCode_WipeTower_hpp_
