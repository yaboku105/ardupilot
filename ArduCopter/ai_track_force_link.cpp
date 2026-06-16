/*
 * Force AP_AI_Track linking - This doesn't modify any existing code
 */

#include <AP_AI_Track/AP_AI_Track.h>

namespace {
    struct ForceLink {
        ForceLink() {
            AP_AI_Track::get_singleton();
        }
    } force_link_instance;
}
