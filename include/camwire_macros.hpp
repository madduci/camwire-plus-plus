#ifndef CAMWIRE_MACROS_HPP
#define CAMWIRE_MACROS_HPP
/***********************************************************************

    Copyright (c) Industrial Research Limited 2004-2011

    This file is part of Camwire, a generic camera interface.

    Camwire is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; either version 2.1 of the
    License, or (at your option) any later version.

    Camwire is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with Camwire; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
    USA


    Title: Macro header for Camwire modules

    Description:
    This header file provides some macros for printing debugging
    information and for error status returns to make the code more
    readable and maintainable.  This header should only be included in
    implementation-specific files, i.e. into files that contain `1394'
    in the file name.

Camwire++: Michele Adduci <info@micheleadduci.net>
***********************************************************************/

#include <iostream>
#include <dc1394/types.h>

/* Print debugging error message: */
#ifdef CAMWIRE_DEBUG
#define DPRINTF(m) std::err << "In " << __FILE__ << " line "<< __LINE__ << " function " << __func__ << ": " << m << std::endl;
#else
#define DPRINTF(m) std::cout << m << std::endl;
#endif

/* If consistently used, it should be possible to change these return
   codes without breaking anything: */
const int CAMWIRE_SUCCESS = 1;  /* true */
const int CAMWIRE_FAILURE = 0;  /* false */

/* Function error status returns, for readability and
   maintainability: */
#define ERROR_IF_NULL(p) \
    if ((p)==NULL) {DPRINTF("Null pointer."); return(CAMWIRE_FAILURE);}
#define ERROR_IF_ZERO(v) \
    if ((v)==0) {DPRINTF("Bad (zero) value."); return(CAMWIRE_FAILURE);}
#define ERROR_IF_CAMWIRE_FAIL(r) \
    if ((r)!=CAMWIRE_SUCCESS) \
      {DPRINTF("Camwire function call failed."); \
       return(CAMWIRE_FAILURE);}
#define ERROR_IF_DC1394_FAIL(r)	\
    if ((r)!= DC1394_SUCCESS) \
      {DPRINTF("dc1394 function call failed."); \
       return(CAMWIRE_FAILURE);}

#define CONFFILE_EXTENSION		".conf"
#define ENVIRONMENT_VAR_CONF	"CAMWIRE_CONF"

/*
    Since libdc1394 doesn't offer and "Invalid video mode" enum type, here I add it:
*/

static const dc1394video_mode_t DC1394_VIDEO_MODE_INVALID = static_cast<dc1394video_mode_t>(0);


/* This format string must exactly match the arguments in the fprintf()
   call in camwire_write_state_to_file(): */
static const char settings_format[] =
    "    num_frame_buffers %d\n"
    "    gain              %lg\n"
    "    brightness        %lg\n"
    "    white_balance     %lg %lg\n"
    "    gamma             %d\n"
    "    colour_corr       %d\n"
    "    colour_coef       %lg %lg %lg %lg %lg %lg %lg %lg %lg\n"
    "    left              %d\n"
    "    top               %d\n"
    "    width             %d\n"
    "    height            %d\n"
    "    coding            %d\n"
    "    tiling            %d\n"
    "    frame_rate        %lg\n"
    "    shutter           %lg\n"
    "    external_trigger  %d\n"
    "    trigger_polarity  %d\n"
    "    single_shot       %d\n"
    "    running           %d\n"
    "    shadow            %d\n";

/* Precalculated normalized inverse gamma transform: */
static const float gamma_inv[256] = {
    0.000000, 0.000871, 0.001743, 0.002614, 0.003486, 0.004357, 0.005229, 0.006100,
    0.006972, 0.007843, 0.008715, 0.009586, 0.010458, 0.011329, 0.012200, 0.013072,
    0.013943, 0.014815, 0.015686, 0.016558, 0.017429, 0.018246, 0.019135, 0.020046,
    0.020981, 0.021940, 0.022922, 0.023928, 0.024958, 0.026011, 0.027089, 0.028190,
    0.029316, 0.030467, 0.031641, 0.032840, 0.034064, 0.035312, 0.036585, 0.037883,
    0.039206, 0.040554, 0.041927, 0.043325, 0.044749, 0.046197, 0.047672, 0.049171,
    0.050697, 0.052247, 0.053824, 0.055427, 0.057055, 0.058709, 0.060390, 0.062096,
    0.063829, 0.065588, 0.067374, 0.069185, 0.071024, 0.072888, 0.074780, 0.076698,
    0.078643, 0.080614, 0.082613, 0.084638, 0.086691, 0.088770, 0.090877, 0.093011,
    0.095172, 0.097361, 0.099577, 0.101820, 0.104091, 0.106389, 0.108715, 0.111069,
    0.113451, 0.115860, 0.118298, 0.120763, 0.123256, 0.125777, 0.128327, 0.130904,
    0.133510, 0.136144, 0.138806, 0.141497, 0.144216, 0.146964, 0.149740, 0.152545,
    0.155379, 0.158241, 0.161132, 0.164051, 0.167000, 0.169978, 0.172984, 0.176020,
    0.179084, 0.182178, 0.185301, 0.188453, 0.191634, 0.194845, 0.198085, 0.201355,
    0.204654, 0.207982, 0.211340, 0.214728, 0.218145, 0.221592, 0.225068, 0.228575,
    0.232111, 0.235677, 0.239274, 0.242900, 0.246556, 0.250242, 0.253958, 0.257705,
    0.261482, 0.265288, 0.269126, 0.272993, 0.276891, 0.280819, 0.284778, 0.288767,
    0.292787, 0.296838, 0.300919, 0.305030, 0.309173, 0.313346, 0.317550, 0.321784,
    0.326050, 0.330347, 0.334674, 0.339033, 0.343422, 0.347843, 0.352295, 0.356778,
    0.361292, 0.365837, 0.370414, 0.375022, 0.379661, 0.384332, 0.389034, 0.393767,
    0.398532, 0.403329, 0.408157, 0.413017, 0.417908, 0.422832, 0.427787, 0.432773,
    0.437792, 0.442842, 0.447924, 0.453038, 0.458184, 0.463362, 0.468573, 0.473815,
    0.479089, 0.484395, 0.489734, 0.495104, 0.500507, 0.505943, 0.511410, 0.516910,
    0.522442, 0.528007, 0.533604, 0.539234, 0.544896, 0.550590, 0.556317, 0.562077,
    0.567870, 0.573695, 0.579553, 0.585443, 0.591367, 0.597323, 0.603312, 0.609334,
    0.615389, 0.621476, 0.627597, 0.633751, 0.639938, 0.646158, 0.652411, 0.658697,
    0.665016, 0.671369, 0.677754, 0.684173, 0.690626, 0.697111, 0.703630, 0.710183,
    0.716768, 0.723387, 0.730040, 0.736726, 0.743446, 0.750200, 0.756986, 0.763807,
    0.770661, 0.777549, 0.784471, 0.791426, 0.798415, 0.805438, 0.812495, 0.819586,
    0.826711, 0.833869, 0.841062, 0.848288, 0.855549, 0.862843, 0.870172, 0.877535,
    0.884931, 0.892362, 0.899828, 0.907327, 0.914861, 0.922429, 0.930031, 0.937668,
    0.945339, 0.953044, 0.960784, 0.968558, 0.976367, 0.984210, 0.992088, 1.000000};

#endif /* ndef CAMWIREMACROS_H */
