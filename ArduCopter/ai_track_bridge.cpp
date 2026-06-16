/*
 * Force AP_AI_Track linking
 * This creates a reference that forces the linker to include the library
 */

#include <AP_AI_Track/AP_AI_Track.h>

static AP_AI_Track *__force_ai_track_link = AP_AI_Track::get_singleton();
