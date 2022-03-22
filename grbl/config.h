/**
    Config loader depending on build type
    for wainlux board the wainlux header will be included for default build
    the default config will be included
*/

#ifdef WAINLUX_JL1
#include "config_wainlux.h"
#else
#include "config_default.h"
#endif
