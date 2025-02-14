#ifndef LANDING_RADIO_H
#define LANDING_RADIO_H

#include "common.h"
#include "bus.h"

// Landing radio connection settings
#define LANDING_RADIO_PORT 5556
#define LANDING_RADIO_HOST "localhost"

// ILS constants
#define GLIDE_SLOPE_ANGLE 3.0    // Standard 3-degree glide slope
#define LOC_WIDTH 5.0            // Localizer beam width in degrees
#define RUNWAY_HEADING 280.0     // Runway heading in degrees

// SFO runway 28L threshold position
static const Position RUNWAY_THRESHOLD = {
    .latitude = 37.6161,
    .longitude = -122.3569,
    .altitude = 13.0    // feet above sea level
};

// ILS Data Structure
typedef struct {
    double localizer;     // Horizontal deviation (-2.5° to +2.5°)
    double glideslope;    // Vertical deviation (typically 3° path)
    double distance;      // Distance to runway threshold (nautical miles)
    bool localizer_valid;
    bool glideslope_valid;
    bool marker_beacon;   // Marker beacon signal
} ILSData;

typedef struct LandingRadio LandingRadio;

// Initialize landing radio receiver
LandingRadio* landing_radio_init(Bus* bus);

// Clean up landing radio receiver
void landing_radio_cleanup(LandingRadio* radio);

// Process one iteration of landing radio data
void landing_radio_process(LandingRadio* radio);

// Main entry point for landing radio receiver process
void landing_radio_main(Bus* bus);

// Convert ILS deviations to position
Position ils_deviations_to_position(const ILSData* ils, const Position* runway_threshold);

#endif // LANDING_RADIO_H
