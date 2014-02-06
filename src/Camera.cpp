/******************************************************************************

    Copyright (c) Industrial Research Limited 2004-2012

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


    Title: Camwire main module

    Description:
    This module is about using a single named camera through its
    handle. The handle should be all a user need know about for complete
    access to all camera functions.  Finding cameras and assigning
    handles to them is done in the Camwire bus module.

    This implementation is for IEEE 1394 digital cameras complying with
    the 1394 Trade Association IIDC (DCAM) specification version 1.31.

******************************************************************************/
#include <ctype.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <netinet/in.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

//#include "dc1394/dc1394.h"
#include <dc1394/vendor/avt.h>

//#include <CameraHandle.h>
#include <CameraConfig.h>
#include <Camera.h>
#include <CameraMacros.h>


/* The newer juju IEEE 1394 stack calculates timestamps differently: */
#define USE_JUJU_STACK

/* Other constants: */
#define ERROR_MESSAGE_MAX_CHARS		500
#define ENVIRONMENT_VAR_CONF		"CAMERA_CONF"
#define CONFFILE_PATH_MAX_CHARS		200
#define CONFFILE_EXTENSION		".conf"
#define CONFFILE_EXTENSION_MAX_CHARS	10
#define CONFFILE_NAME_MAX_CHARS		(CONFFILE_PATH_MAX_CHARS + \
                                         1 + CAMERA_ID_MAX_CHARS + \
                                         CONFFILE_EXTENSION_MAX_CHARS)
#define STATEFILE_LINE_MAX_CHARS	500
#define STATEFILE_COMMENT_CHAR		'#'
#define WHITESPACE			" \t\r\n"
#define VERSION_STRING_LENGTH           (3*4 + 2 + 1)

/* Local prototypes: */
static int create(const Camwire_handle c_handle, const Camwire_state *set);
static int connect_cam(const Camwire_handle c_handle, Camwire_conf *cfg,
		       const Camwire_state *set);
static int set_non_dma_registers(const Camwire_handle c_handle,
				 const Camwire_state *set);
static void disconnect_cam(const Camwire_handle c_handle);
static int reconnect_cam(const Camwire_handle c_handle, Camwire_conf *cfg,
			 const Camwire_state *set);
static void free_internals(const Camwire_handle c_handle);
inline static int feature_has_mode(const dc1394feature_info_t *cap,
				   const dc1394feature_mode_t mode);
inline static int feature_is_usable(const dc1394feature_info_t *cap);
static int feature_switch_on(const Camwire_handle c_handle,
			     dc1394feature_info_t *cap);
static int feature_go_manual(const Camwire_handle c_handle,
			     dc1394feature_info_t *cap);
inline static
dc1394feature_info_t * get_feature_capability(const Camwire_handle c_handle,
					      const dc1394feature_t feature);
static int probe_camera_gamma(const Camwire_handle c_handle);
static int probe_camera_colour_correction(const Camwire_handle c_handle);
static Camwire_tiling probe_camera_tiling(const Camwire_handle c_handle);
inline static
dc1394video_frame_t * get_captureframe(const Camwire_handle c_handle);
inline static Camwire_state * get_shadow_state(const Camwire_handle c_handle);
inline static dc1394video_mode_t get_1394_video_mode(const Camwire_handle c_handle);
inline static int fixed_image_size(const dc1394video_mode_t video_mode);
inline static int variable_image_size(const dc1394video_mode_t video_mode);
static int get_current_settings(const Camwire_handle c_handle,
				Camwire_state *set);
static int sleep_frametime(const Camwire_handle c_handle,
			   const double multiple);
static Camwire_pixel convert_colorid2pixelcoding(const dc1394color_coding_t color_id);
static Camwire_tiling convert_filterid2pixeltiling(
    const dc1394color_filter_t filter_id);
static int component_depth(const Camwire_pixel coding);
static Camwire_pixel convert_videomode2pixelcoding(const dc1394video_mode_t video_mode);
static double convert_busspeed2busfreq(const int bus_speed);
static int convert_busspeed2dc1394(const int bus_speed);
static double convert_numpackets2framerate(const Camwire_handle c_handle,
					   const uint32_t num_packets);
static double convert_index2framerate(const dc1394framerate_t frame_rate_index);
static
dc1394framerate_t convert_framerate2index(const double frame_rate,
					  const dc1394framerates_t *framerate_list);
static int get_numpackets(const Camwire_handle c_handle, uint32_t *num_p);
static uint32_t convert_packetsize2numpackets(const Camwire_handle c_handle,
					 const uint32_t packet_size,
					 const int width,
					 const int height,
					 const Camwire_pixel coding);
static uint32_t convert_numpackets2packetsize(const Camwire_handle c_handle,
					 const uint32_t num_packets,
					 const int width,
					 const int height,
					 const Camwire_pixel coding);
static uint32_t convert_framerate2numpackets(const Camwire_handle c_handle,
					const double frame_rate);
static dc1394color_coding_t convert_pixelcoding2colorid(
    const Camwire_pixel coding, const dc1394color_codings_t *coding_list);
inline static dc1394video_mode_t convert_format_mode2dc1394video_mode(const int format,
								       const int mode);
inline static void convert_dc1394video_mode2format_mode(
    const dc1394video_mode_t video_mode, int *format, int *mode);
static void convert_colourcoefs2avtvalues(const double coef[9], int32_t val[9]);
static void convert_avtvalues2colourcoefs(const int32_t val[9], double coef[9]);
static int generate_default_settings(const Camwire_handle c_handle,
				     Camwire_state *set);
inline static int config_cache_exists(const User_handle internal_status);
static int read_conf_file(FILE *conffile, Camwire_conf *cfg);
static FILE * find_conf_file(const Camwire_id *id);
static FILE * open_named_conf_file(const char *path, const char *filename);
static int generate_default_config(const Camwire_handle c_handle,
				   Camwire_conf *cfg);
static char * skip_whitespace(const char *string);
static char * skip_non_whitespace(const char *string);
static int is_in_coding_list(const dc1394color_codings_t *coding_list,
			     const dc1394color_coding_t color_id);


/* Global variables needed in various places: */

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

/* Inverse gamma transform look-up table used by camwire_inv_gamma().
   Assumed to be initialized when Camwire_user_data.gamma_maxval
   is non-zero: */
uint16_t gamma_lut[256];

/* The current version string, to be filled in by camwire_version(): */
static char const version_string[VERSION_STRING_LENGTH];

/* To translate to or from dc1394 mode enums: */
static const int mode_dc1394_offset[] = {
    DC1394_VIDEO_MODE_160x120_YUV444,   /* Format 0.*/
    DC1394_VIDEO_MODE_800x600_YUV422,   /* Format 1.*/
    DC1394_VIDEO_MODE_1280x960_YUV422,  /* Format 2.*/
    0, 0, 0,                            /* Reserved formats.*/
    DC1394_VIDEO_MODE_EXIF,             /* Format 6.*/
    DC1394_VIDEO_MODE_FORMAT7_0         /* Format 7.*/
};



/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* This can be implemented as a constructor of a Camwire C++ class. */

int camwire_create(const Camwire_handle c_handle)
{
    Camwire_state settings;
    
    /* Get factory default start-up settings: */
    ERROR_IF_NULL(c_handle);
    if (camwire_get_state(c_handle, &settings) != CAMERA_SUCCESS)
    {
	DPRINTF("camwire_get_state() failed.");
    return CAMERA_FAILURE;
    }
    /* CAMERA_SUCCESS & CAMERA_FAILURE are defined in camwire.h.*/

    return create(c_handle, &settings);
} /* camwire_create() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* This can be implemented as a constructor of a Camwire class. */

int camwire_create_from_struct(const Camwire_handle c_handle,
			       const Camwire_state *set)
{
    ERROR_IF_NULL(c_handle);
    return create(c_handle, set);
} /* camwire_create_from_struct() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* This can be implemented as the destructor of a Camwire class. */

void camwire_destroy(const Camwire_handle c_handle)
{
    if (c_handle)
    {
	camwire_set_run_stop(c_handle, 0);
	sleep_frametime(c_handle, 1.5);
	/* Reset causes problems with too many cameras, so comment it out: */
	/* dc1394_camera_reset(camwire_handle_get_camera(c_handle)); */
	disconnect_cam(c_handle);
	free_internals(c_handle);
    }
} /* camwire_destroy() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* Camwire_state is defined in camwire.h.*/

int camwire_get_state(const Camwire_handle c_handle, Camwire_state *set)
{
    User_handle internal_status;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    if (!internal_status || !internal_status->camera_connected)
    {  /* Camera does not exit.*/
    ERROR_IF_CAMERA_FAIL(
	    generate_default_settings(c_handle, set));
    }
    else
    {  /* Camera exists.*/
    ERROR_IF_CAMERA_FAIL(
	    get_current_settings(c_handle, set));
    }
    return CAMERA_SUCCESS;
} /* camwire_get_state() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_state(const Camwire_handle c_handle, const Camwire_state *set)
{
    Camwire_state current_settings;
    Camwire_conf config;

    ERROR_IF_NULL(c_handle);
    ERROR_IF_CAMERA_FAIL(
	get_current_settings(c_handle, &current_settings));
    
    if (set->num_frame_buffers != current_settings.num_frame_buffers ||
	set->width != current_settings.width ||
	set->height != current_settings.height ||
	set->coding != current_settings.coding ||
	set->frame_rate != current_settings.frame_rate)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_config(c_handle, &config));
	
	/* Set new state by re-initializing the camera: */
    ERROR_IF_CAMERA_FAIL(
	    reconnect_cam(c_handle, &config, set));
    }
    else
    {
	/* Frame offset is a special case which neither requires
	   reconnect_cam() nor is set in set_non_dma_registers(): */
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_frame_offset(c_handle, set->left, set->top));

	/* Set all the others: */
    ERROR_IF_CAMERA_FAIL(
	    set_non_dma_registers(c_handle, set));
    }
   
    return CAMERA_SUCCESS;
} /* camwire_set_state() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_read_state_from_file(FILE *infile, Camwire_state *set)
{
    int empty, comment, scan_error;
    char linebuffer[STATEFILE_LINE_MAX_CHARS+1];
    char *lineptr;
    char *new_tag, *value;
    char *ref_tag, *format;
    char error_message[ERROR_MESSAGE_MAX_CHARS+1];
    char old_tags[] = "Camwire settings:\n blue_gain %lg\n red_gain %lg\n";
    
    scan_error = 0;
    new_tag = "";
    while(!feof(infile))
    {
	/* Read lines until not empty and not a comment: */
	empty = 1;
	comment = 1;
	while (empty || comment)
	{
	    lineptr = fgets(linebuffer, STATEFILE_LINE_MAX_CHARS, infile);
	    if (lineptr == NULL)  break;
	    empty = (strlen(skip_whitespace(linebuffer)) == 0);
	    comment = (linebuffer[0] == STATEFILE_COMMENT_CHAR);
	}
	if (lineptr == NULL)  break;

	/* Separate input tag string and value string: */
	new_tag = skip_whitespace(linebuffer);
	lineptr = skip_non_whitespace(new_tag);
	if (*(lineptr - 1) == ':')  --lineptr;  /* Backwards compatibility.*/
	*lineptr = '\0';
	lineptr++;
	value = skip_whitespace(lineptr);
	if (strlen(value) >= 2)  /* Including a newline char.*/
	{
	    /* Find corresponding tag and conversion in the format: */
	    ref_tag = strstr(settings_format, new_tag);
	    if (!ref_tag)
		ref_tag = strstr(old_tags, new_tag);  /* Backwards compatibility.*/
	    if (ref_tag)
	    {
		lineptr = skip_non_whitespace(ref_tag);
		format = skip_whitespace(lineptr);
		if (strstr(format, new_tag))
		{
		    DPRINTF("Duplicate or ambiguous tag in settings file.");
            return CAMERA_FAILURE;
		}

	        scan_error = 1;  /* Error if a break is reached below.*/
		if (strstr("Camwire", new_tag))  /* Backwards compatibility.*/
		{
		    if (!strstr(value, "settings"))  break;
		}
		else if (strstr("num_frame_buffers", new_tag))
		{
		    if (sscanf(value, format, &set->num_frame_buffers) != 1)  break;
		}
		else if (strstr("gain", new_tag))
		{
		    if (sscanf(value, format, &set->gain) != 1)  break;
		}
		else if (strstr("brightness", new_tag))
		{
		    if (sscanf(value, format, &set->brightness) != 1)  break;
		}
		else if (strstr("white_balance", new_tag))
		{
		    if (sscanf(value, format, &set->white_balance[0],
				&set->white_balance[1]) != 2)
			break;
		}
		else if (strstr("blue_gain", new_tag))  /* Backwards compatibility.*/
		{
		    if (sscanf(value, format, &set->white_balance[0]) != 1)  break;
		}
		else if (strstr("red_gain", new_tag))  /* Backwards compatibility.*/
		{
		    if (sscanf(value, format, &set->white_balance[1]) != 1)  break;
		}
		else if (strstr("gamma", new_tag))
		{
		    if (sscanf(value, format, &set->gamma) != 1)  break;
		}
		else if (strstr("colour_corr", new_tag))
		{
		    if (sscanf(value, format, &set->colour_corr) != 1)  break;
		}
		else if (strstr("colour_coef", new_tag))
		{
		    if (sscanf(value, format, &set->colour_coef[0],
				&set->colour_coef[1], &set->colour_coef[2],
				&set->colour_coef[3], &set->colour_coef[4],
				&set->colour_coef[5], &set->colour_coef[6],
				&set->colour_coef[7], &set->colour_coef[8]) != 9)
			break;
		}
		else if (strstr("left", new_tag))
		{
		    if (sscanf(value, format, &set->left) != 1)  break;
		}
		else if (strstr("top", new_tag))
		{
		    if (sscanf(value, format, &set->top) != 1)  break;
		}
		else if (strstr("width", new_tag))
		{
		    if (sscanf(value, format, &set->width) != 1)  break;
		}
		else if (strstr("height", new_tag))
		{
		    if (sscanf(value, format, &set->height) != 1)  break;
		}
		else if (strstr("coding", new_tag))
		{
		    if (sscanf(value, format, &set->coding) != 1)  break;
		}
		else if (strstr("tiling", new_tag))
		{
		    if (sscanf(value, format, &set->tiling) != 1)  break;
		}
		else if (strstr("frame_rate", new_tag))
		{
		    if (sscanf(value, format, &set->frame_rate) != 1)  break;
		}
		else if (strstr("shutter", new_tag))
		{
		    if (sscanf(value, format, &set->shutter) != 1)  break;
		}
		else if (strstr("external_trigger", new_tag))
		{
		    if (sscanf(value, format, &set->external_trigger) != 1)  break;
		}
		else if (strstr("trigger_polarity", new_tag))
		{
		    if (sscanf(value, format, &set->trigger_polarity) != 1)  break;
		}
		else if (strstr("single_shot", new_tag))
		{
		    if (sscanf(value, format, &set->single_shot) != 1)  break;
		}
		else if (strstr("running", new_tag))
		{
		    if (sscanf(value, format, &set->running) != 1)  break;
		}
		else if (strstr("shadow", new_tag))
		{
		    if (sscanf(value, format, &set->shadow) != 1)  break;
		}
		else
		{
		    snprintf(error_message, ERROR_MESSAGE_MAX_CHARS,
			     "Internal error scanning tag `%s' in state file.",
			     new_tag);
		    DPRINTF(error_message);
            return CAMERA_FAILURE;
		}
		scan_error = 0;  /* No error if we arrive here.*/
	    }
	    else
	    {
		snprintf(error_message, ERROR_MESSAGE_MAX_CHARS,
			 "Unrecognized tag `%s' in state file.", new_tag);
		DPRINTF(error_message);
        return CAMERA_FAILURE;
	    }
	}
	else
	{
	    snprintf(error_message, ERROR_MESSAGE_MAX_CHARS,
		     "No value for tag `%s' in state file.", new_tag);
	    DPRINTF(error_message);
        return CAMERA_FAILURE;
	}
    }  /* while */

    if (scan_error)
    {
	snprintf(error_message, ERROR_MESSAGE_MAX_CHARS,
		 "Error reading value for tag `%s' in state file.", new_tag);
	DPRINTF(error_message);
    return CAMERA_FAILURE;
    }

    return CAMERA_SUCCESS;
} /* camwire_read_state_from_file() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_write_state_to_file(FILE *outfile, const Camwire_state *set)
{
    int print_result;

    fprintf(outfile, "# Camwire settings:\n");
    print_result = fprintf(outfile, settings_format,
	       set->num_frame_buffers,
	       set->gain,
	       set->brightness,
	       set->white_balance[0],
	       set->white_balance[1],
	       set->gamma,
	       set->colour_corr,
	       set->colour_coef[0],
	       set->colour_coef[1],
	       set->colour_coef[2],
	       set->colour_coef[3],
	       set->colour_coef[4],
	       set->colour_coef[5],
	       set->colour_coef[6],
	       set->colour_coef[7],
	       set->colour_coef[8],
	       set->left,
	       set->top,
	       set->width,
	       set->height,
	       set->coding,
	       set->tiling,
	       set->frame_rate,
	       set->shutter,
	       set->external_trigger,
	       set->trigger_polarity,
	       set->single_shot,
	       set->running,
	       set->shadow);
    /* Any changes to the arguments to fprintf() above must also be done
       to the global format string settings_format. */
    
    if (print_result < 1)
    {
	DPRINTF("fprintf() failed.");
    return CAMERA_FAILURE;
    }
    fflush(outfile);
    return CAMERA_SUCCESS;
} /* camwire_write_state_to_file() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* Camwire_id is defined in camwire.h.*/

int camwire_get_identifier(const Camwire_handle c_handle,
			   Camwire_id *identifier)
{
    Camera_handle camera;

    ERROR_IF_NULL(c_handle);
    camera = camwire_handle_get_camera(c_handle);
    
    strncpy(identifier->vendor, camera->vendor, CAMERA_ID_MAX_CHARS);
    /* CAMERA_ID_MAX_CHARS is defined in camwire.h.*/
    identifier->vendor[CAMERA_ID_MAX_CHARS] = '\0';
    strncpy(identifier->model, camera->model, CAMERA_ID_MAX_CHARS);
    identifier->model[CAMERA_ID_MAX_CHARS] = '\0';
    snprintf(identifier->chip, CAMERA_ID_MAX_CHARS, "%"PRIX64"h", camera->guid);
    identifier->chip[CAMERA_ID_MAX_CHARS] = '\0';
    return CAMERA_SUCCESS;
} /* camwire_get_identifier() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_stateshadow(const Camwire_handle c_handle, int *shadow)
{
    Camwire_state *shadow_state;
    
    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *shadow = shadow_state->shadow;
    return CAMERA_SUCCESS;
} /* camwire_get_stateshadow() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_stateshadow(const Camwire_handle c_handle, const int shadow)
{
    Camwire_state *shadow_state;
    
    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    shadow_state->shadow = shadow;
    return CAMERA_SUCCESS;
} /* camwire_set_stateshadow() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_num_framebuffers(const Camwire_handle c_handle,
				 int *num_frame_buffers)
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    
    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *num_frame_buffers = internal_status->num_dma_buffers;
    shadow_state->num_frame_buffers = *num_frame_buffers;
    return CAMERA_SUCCESS;
} /* camwire_get_num_framebuffers() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_num_framebuffers(const Camwire_handle c_handle,
				 const int num_frame_buffers)
{
    Camwire_state settings;
    Camwire_conf config;
    int temp_num_bufs;

    ERROR_IF_NULL(c_handle);
    ERROR_IF_CAMERA_FAIL(
	get_current_settings(c_handle, &settings));

    /* Ensure that video1394 lower limit is met: */
    if (num_frame_buffers < 2)  temp_num_bufs = 2;
    else                        temp_num_bufs = num_frame_buffers;

    /* Only proceed if number of buffers has changed: */
    if (settings.num_frame_buffers != temp_num_bufs)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_config(c_handle, &config));
	
	/* Set new number of buffers by re-initializing the camera: */
	settings.num_frame_buffers = temp_num_bufs;
    ERROR_IF_CAMERA_FAIL(
	    reconnect_cam(c_handle, &config, &settings));
    }
    
    return CAMERA_SUCCESS;
} /* camwire_set_num_framebuffers() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.  Deprecated.
*/
int camwire_get_framebuffer_lag(const Camwire_handle c_handle, int *buffer_lag)
{
    dc1394video_frame_t *capture_frame;

    ERROR_IF_NULL(c_handle);
    capture_frame = get_captureframe(c_handle);
    if (!capture_frame)
    {
	*buffer_lag = 0;
    }
    else
    {
	*buffer_lag = capture_frame->frames_behind;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_framebuffer_lag() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_flush_framebuffers(const Camwire_handle c_handle,
			       const int num_to_flush,
			       int *num_flushed,
			       int *buffer_lag)
{
    void *buffer;
    int flush_count;

    ERROR_IF_NULL(c_handle);
    
    for (flush_count = 0; flush_count < num_to_flush; ++flush_count)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_point_next_frame_poll(c_handle, &buffer, buffer_lag));
	if (!buffer)  break;
    ERROR_IF_CAMERA_FAIL(
	    camwire_unpoint_frame(c_handle));
    }

    if (num_flushed)  *num_flushed = flush_count;

    /* Best-guess buffer lag if no frames got flushed: */
    if (buffer_lag && num_to_flush < 1)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_framebuffer_lag(c_handle, buffer_lag));
    }
    return CAMERA_SUCCESS;
} /* camwire_flush_framebuffers() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_frame_offset(const Camwire_handle c_handle, int *left,
			     int *top)
{
    Camwire_state *shadow_state;
    dc1394video_mode_t video_mode;
    uint32_t left_val, top_val;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *left = shadow_state->left;
    *top = shadow_state->top;
    if (!shadow_state->shadow)
    {
	video_mode = get_1394_video_mode(c_handle);
	ERROR_IF_ZERO(video_mode);
	if (fixed_image_size(video_mode)) 	/* Format 0, 1 or 2.*/
	{
	    *left = *top = 0;
	}
	else if (variable_image_size(video_mode))  /* Format 7.*/
	{
	    ERROR_IF_DC1394_FAIL(
		dc1394_format7_get_image_position(
		    camwire_handle_get_camera(c_handle),
		    video_mode,
		    &left_val,
		    &top_val));
	    *left = left_val;
	    *top = top_val;
	}
	else
	{
	    DPRINTF("Unsupported camera format.");
        return CAMERA_FAILURE;
	}
	shadow_state->left = *left;
	shadow_state->top = *top;
    }
    
    return CAMERA_SUCCESS;
} /* camwire_get_frame_offset() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_frame_offset(const Camwire_handle c_handle,
			     const int left, const int top)
{
    dc1394video_mode_t video_mode;
    uint32_t max_width, max_height;
    uint32_t hor_pixel_unit, ver_pixel_unit;
    int hor_limit, ver_limit;
    uint32_t width, height;
    int new_left, new_top;
    Camwire_state *shadow_state;

    ERROR_IF_NULL(c_handle);
    video_mode = get_1394_video_mode(c_handle);
    ERROR_IF_ZERO(video_mode);
    if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
    {
	DPRINTF("Attempt to set frame offset in a fixed image size video_mode.");
    return CAMERA_FAILURE;
    }
    else if (variable_image_size(video_mode))  /* Format 7.*/
    {

	/* Get maximum width and height from the camera and adjust input
	   arguments if necessary, taking frame dimensions into
	   account: */
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_max_image_size(camwire_handle_get_camera(c_handle),
					     video_mode,
					     &max_width, &max_height));
	if (max_width == 0 || max_height == 0)
	{
	    DPRINTF("dc1394_format7_get_max_image_size() returned a "
		    "zero size.");
        return CAMERA_FAILURE;
	}
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_image_size(
		camwire_handle_get_camera(c_handle),
		video_mode,
		&width, &height));
	if (width > max_width || height > max_height)
	{
	    DPRINTF("dc1394_format7_get_image_size() returned a size which "
		    "exceeded the maximum.");
        return CAMERA_FAILURE;
	}
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_unit_position(
		camwire_handle_get_camera(c_handle),
		video_mode,
		&hor_pixel_unit,
		&ver_pixel_unit));
	if (hor_pixel_unit == 0 || ver_pixel_unit == 0)
	{
	    DPRINTF("dc1394_format7_get_unit_position() returned a zero "
		    "unit size.");
        return CAMERA_FAILURE;
	}

	new_left = left/hor_pixel_unit;
	new_top  = top /ver_pixel_unit;
	hor_limit = (max_width  - width )/hor_pixel_unit;
	ver_limit = (max_height - height)/ver_pixel_unit;
	if (new_left > hor_limit)  new_left = hor_limit;
	if (new_top  > ver_limit)  new_top  = ver_limit;
	new_left *= hor_pixel_unit;
	new_top  *= ver_pixel_unit;

	/* Write the new offsets to the camera: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_format7_set_image_position(
		camwire_handle_get_camera(c_handle),
		video_mode,
		new_left, new_top));
	shadow_state = get_shadow_state(c_handle);
	ERROR_IF_NULL(shadow_state);
	shadow_state->left = new_left;
	shadow_state->top = new_top;
    }
    else
    {
	DPRINTF("Unsupported camera format.");
    return CAMERA_FAILURE;
    }
    return CAMERA_SUCCESS;
} /* camwire_set_frame_offset() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_frame_size(const Camwire_handle c_handle, int *width,
			   int *height)
{
    Camwire_state *shadow_state;
    dc1394video_mode_t video_mode;
    dc1394video_frame_t *capture_frame;
    uint32_t width_val, height_val;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *width = shadow_state->width;
    *height = shadow_state->height;
    if (!shadow_state->shadow)
    {
	video_mode = get_1394_video_mode(c_handle);
	ERROR_IF_ZERO(video_mode);
	if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
	{
	    capture_frame = get_captureframe(c_handle);
	    ERROR_IF_NULL(capture_frame);
	    if (capture_frame->size[0] == 0 ||
		capture_frame->size[1] == 0)
	    {
		DPRINTF("dc1394video_frame_t contains a zero frame size.");
        return CAMERA_FAILURE;
	    }
	    *width = capture_frame->size[0];
	    *height = capture_frame->size[1];
	}
	else if (variable_image_size(video_mode))  /* Format 7.*/
	{
	    ERROR_IF_DC1394_FAIL(
		dc1394_format7_get_image_size(
		    camwire_handle_get_camera(c_handle),
		    video_mode,
		    &width_val,
		    &height_val));
	    *width = width_val;
	    *height = height_val;
	}
	else
	{
	    DPRINTF("Unsupported camera format.");
        return CAMERA_FAILURE;
	}
	shadow_state->width = *width;
	shadow_state->height = *height;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_frame_size() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* The frame size is updated by disconnecting and reconnecting the
   camera.  There does not seem to be a less brutal way. */

int camwire_set_frame_size(const Camwire_handle c_handle, const int width,
			   const int height)
{
    dc1394video_mode_t video_mode;
    Camwire_state settings;
    uint32_t max_width, max_height;
    uint32_t hor_pixel_unit, ver_pixel_unit;
    int left, top;
    int hor_limit, ver_limit;
    int new_width, new_height;
    Camwire_conf config;
    int min_pixel_units;

    ERROR_IF_NULL(c_handle);
    video_mode = get_1394_video_mode(c_handle);
    ERROR_IF_ZERO(video_mode);
    if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
    {
	DPRINTF("Attempt to change frame size in a fixed image size format.");
    return CAMERA_FAILURE;
    }
    else if (variable_image_size(video_mode))  /* Format 7.*/
    {
    ERROR_IF_CAMERA_FAIL(
	    get_current_settings(c_handle, &settings));

	/* Width and height: */
	if (width == settings.width && height == settings.height)
	{
        return CAMERA_SUCCESS; 	/* Nothing has changed.*/
	}

	/* Get maximum width, maximum height, unit pixel sizes, and
	   offsets from the camera: */
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_max_image_size(
		camwire_handle_get_camera(c_handle),
		video_mode,
		&max_width,
		&max_height));
	if (max_width  == 0 || max_height == 0)
	{
	    DPRINTF("dc1394_format7_get_max_image_size() returned a zero "
		    "maximum size.");
        return CAMERA_FAILURE;
	}
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_unit_size(
		camwire_handle_get_camera(c_handle),
		video_mode,
		&hor_pixel_unit,
		&ver_pixel_unit));
	if (hor_pixel_unit == 0 || ver_pixel_unit == 0)
	{
	    DPRINTF("dc1394_format7_get_unit_size() returned a zero "
		    "unit size.");
        return CAMERA_FAILURE;
	}
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_frame_offset(c_handle, &left, &top));

	/* Adjust input arguments if necessary, taking maximum frame
	   sizes, current offsets and unit pixel sizes into account: */
	if (width < INT_MAX - (int)hor_pixel_unit/2)
	    new_width  = (width  + hor_pixel_unit/2)/hor_pixel_unit;
	else
	    new_width = INT_MAX/hor_pixel_unit;
	if (height < INT_MAX - (int)ver_pixel_unit/2)
	    new_height = (height + ver_pixel_unit/2)/ver_pixel_unit;
	else
	    new_height = INT_MAX/ver_pixel_unit;
	if (new_width  < 1)  new_width = 1;
	if (new_height < 1)  new_height = 1;
	hor_limit = (max_width - left)/hor_pixel_unit;
	ver_limit = (max_height - top)/ver_pixel_unit;
	if (new_width  > hor_limit)  new_width  = hor_limit;
	if (new_height > ver_limit)  new_height = ver_limit;
	
	/* Maintain the minimum number of pixels: */
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_config(c_handle, &config));
	min_pixel_units = config.min_pixels/(hor_pixel_unit*ver_pixel_unit);
	if (new_width*new_height < min_pixel_units)
	{
	    new_width = (min_pixel_units + new_height - 1)/new_height;
	    if (new_width > hor_limit)
	    {
		new_width = hor_limit;
		new_height = (min_pixel_units + new_width - 1)/new_width;
		if (new_height > ver_limit)  new_height = ver_limit;
	    }
	}
	new_width  *= hor_pixel_unit;
	new_height *= ver_pixel_unit;

        /* Only proceed if size has changed after all: */
	if (new_width != settings.width || new_height != settings.height)
	{
	    settings.width  = new_width;
	    settings.height = new_height;

	    /* Set the new dimensions by re-initializing the camera: */
        ERROR_IF_CAMERA_FAIL(
		reconnect_cam(c_handle, &config, &settings));
	}
    }
    else
    {
	DPRINTF("Unsupported camera format.");
    return CAMERA_FAILURE;
    }
    return CAMERA_SUCCESS;
} /* camwire_set_frame_size() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_pixel_coding(const Camwire_handle c_handle, Camwire_pixel *coding)
{
    Camwire_state *shadow_state;
    dc1394video_mode_t video_mode;
    dc1394color_coding_t color_id;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *coding = shadow_state->coding;
    if (!shadow_state->shadow)
    {
	video_mode = get_1394_video_mode(c_handle);
	ERROR_IF_ZERO(video_mode);
	if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
	{
	    *coding = convert_videomode2pixelcoding(video_mode);
	}
	else if (variable_image_size(video_mode))  /* Format 7.*/
	{
	    ERROR_IF_DC1394_FAIL(
		dc1394_format7_get_color_coding(
		    camwire_handle_get_camera(c_handle),
		    video_mode,
		    &color_id));
	    *coding = convert_colorid2pixelcoding(color_id);
	}
	else
	{
	    DPRINTF("Unsupported camera format.");
        return CAMERA_FAILURE;
	}
	shadow_state->coding = *coding;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_pixel_coding() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* The pixel colour coding is updated by disconnecting and reconnecting
   the camera.  I have not been able to do it less brutally.  It seems
   that the video1394 driver does not expect the frame size to change
   even if enough memory has been allocated for larger frames. */

int camwire_set_pixel_coding(const Camwire_handle c_handle,
			     const Camwire_pixel coding)
{
    dc1394video_mode_t video_mode;
    Camwire_pixel old_coding;
    dc1394color_codings_t coding_list;
    dc1394color_coding_t color_id;
    int old_depth, new_depth;
    Camwire_state settings;
    Camwire_state *shadow_state;
    Camwire_conf config;
    
    ERROR_IF_NULL(c_handle);
    video_mode  = get_1394_video_mode(c_handle);
    ERROR_IF_ZERO(video_mode);
    if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
    {
	DPRINTF("Attempt to set pixel coding in a fixed image size format.");
	/* TODO: Set it by changing the mode?*/
    return CAMERA_FAILURE;
    }
    else if (variable_image_size(video_mode))  /* Format 7.*/
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_pixel_coding(c_handle, &old_coding));
	
        /* Only proceed if pixel colour coding has changed: */
	if (coding != old_coding)
	{
	    /* Check if new pixel coding is supported by camera: */
	    ERROR_IF_DC1394_FAIL(
		dc1394_format7_get_color_codings(
		    camwire_handle_get_camera(c_handle),
		    video_mode,
		    &coding_list));
	    if (coding_list.num == 0)
	    {
		DPRINTF("dc1394_format7_get_color_codings() returned an empty list.");
        return CAMERA_FAILURE;
	    }
	    color_id = convert_pixelcoding2colorid(coding, &coding_list);
	    if (color_id == 0)
	    {
		DPRINTF("Pixel colour coding is invalid or not supported  by "
			"the camera.");
        return CAMERA_FAILURE;
	    }

	    /* Set the new coding: */
        ERROR_IF_CAMERA_FAIL(
		camwire_pixel_depth(old_coding, &old_depth));
        ERROR_IF_CAMERA_FAIL(
		camwire_pixel_depth(coding, &new_depth));
	    if (new_depth == old_depth)
	    {
		/* Set the new coding directly: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_format7_set_color_coding(
			camwire_handle_get_camera(c_handle),
			video_mode,
			color_id));
		shadow_state = get_shadow_state(c_handle);
		ERROR_IF_NULL(shadow_state);
		shadow_state->coding = coding;
	    }
	    else
	    {
		/* Re-initialize the camera with the new coding: */
        ERROR_IF_CAMERA_FAIL(
		    camwire_get_config(c_handle, &config));
        ERROR_IF_CAMERA_FAIL(
		    get_current_settings(c_handle, &settings));
		settings.coding = coding;
        ERROR_IF_CAMERA_FAIL(
		    reconnect_cam(c_handle, &config, &settings));
	    }
	}
    }
    else
    {
	DPRINTF("Unsupported camera format.");
    return CAMERA_FAILURE;
    }
    return CAMERA_SUCCESS;
} /* camwire_set_pixel_coding() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_pixel_tiling(const Camwire_handle c_handle, Camwire_tiling *tiling)
{
    Camwire_state *shadow_state;
    dc1394video_mode_t video_mode;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *tiling = shadow_state->tiling;
    if (!shadow_state->shadow)
    {
	video_mode = get_1394_video_mode(c_handle);
	ERROR_IF_ZERO(video_mode);
	if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
	{
        *tiling = CAMERA_TILING_INVALID;  /* No tiling in fixed image sizes.*/
	}
	else if (variable_image_size(video_mode))  /* Format 7.*/
	{
	    *tiling = probe_camera_tiling(c_handle);
	}
	else
	{
	    DPRINTF("Unsupported camera format.");
        return CAMERA_FAILURE;
	}
	shadow_state->tiling = *tiling;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_pixel_tiling() */


/* On most cameras, pixel colour tiling pattern is fixed and cannot be
   set.  That's why there is no camwire_set_pixel_tiling() function .*/

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_pixel_depth(const Camwire_pixel coding, int *depth)
{
    switch (coding)
    {
    case CAMERA_PIXEL_MONO8:
    case CAMERA_PIXEL_RAW8:
	    *depth = 8;
	    break;
    case CAMERA_PIXEL_YUV411:
	    *depth = 12;
	    break;
    case CAMERA_PIXEL_YUV422:
    case CAMERA_PIXEL_MONO16:
    case CAMERA_PIXEL_MONO16S:
    case CAMERA_PIXEL_RAW16:
	    *depth = 16;
	    break;
    case CAMERA_PIXEL_YUV444:
    case CAMERA_PIXEL_RGB8:
	    *depth = 24;
	    break;
    case CAMERA_PIXEL_RGB16:
    case CAMERA_PIXEL_RGB16S:
	    *depth = 48;
	    break;
	default:
	    *depth = 0;
        return CAMERA_FAILURE;  /* Invalid or unknown coding.*/
	    break;
    }
    return CAMERA_SUCCESS;
} /* camwire_pixel_depth() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* In Formats 0, 1 and 2, the frame rate is stored in the camera as an
   index, which we translate into a frame rate in frames per second.

   In format 7, the camera's frame rate index is ignored.  One has to
   calculate the frame rate from the number of packets required to send
   one frame.  For example, since exactly one packet is sent every 125
   microseconds (assuming bus speed of 400 Mb/s), the frame rate is
   1/(num_packets*125us).  The camera calculates num_packets and we read
   it from a register called PACKET_PER_FRAME_INQ.
*/

int camwire_get_framerate(const Camwire_handle c_handle,
			  double *frame_rate)
{
    Camwire_state *shadow_state;
    dc1394video_mode_t video_mode;
    dc1394framerate_t frame_rate_index;
    uint32_t num_packets;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *frame_rate = shadow_state->frame_rate;
    if (!shadow_state->shadow)
    {
	video_mode = get_1394_video_mode(c_handle);
	ERROR_IF_ZERO(video_mode);
	if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
	{
	    ERROR_IF_DC1394_FAIL(
		dc1394_video_get_framerate(camwire_handle_get_camera(c_handle),
					   &frame_rate_index));
	    
	    *frame_rate = convert_index2framerate(frame_rate_index);
	    if (*frame_rate < 0.0)
	    {
		DPRINTF("convert_index2framerate() failed.");
        return CAMERA_FAILURE; 	/* Invalid index.*/
	    }
	}
	else if (variable_image_size(video_mode))  /* Format 7.*/
	{
	    /* It is safe to call get_numpackets() because we are not
	       changing the image_size or color_id: */
        ERROR_IF_CAMERA_FAIL(
		get_numpackets(c_handle, &num_packets));
	    *frame_rate = convert_numpackets2framerate(c_handle, num_packets);
	}
	else
	{
	    DPRINTF("Unsupported camera format.");
        return CAMERA_FAILURE;
	}
	shadow_state->frame_rate = *frame_rate;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_framerate() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* In Formats 0, 1 and 2, the frame rate is written to a camera register
   as an index, which we get by quantizing the frame rate to the nearest
   legal value.

   In Format 7, the frame rate register is ignored.  The frame rate is
   controlled indirectly by setting the number of packets required per
   frame.  This is slightly involved because the number of packets
   cannot be written to the camera as such (although we may be able to
   read its current value).  We instead have to set the packet size,
   which in turn depends on the frame size.

   Even worse, changing the packet size causes the total frame size to
   change because of varying amounts of padding.  I have not been able
   to find a way to change the frame rate in Format 7 without
   re-initializing the camera interface to the video1394 driver.
*/

int camwire_set_framerate(const Camwire_handle c_handle,
			  const double frame_rate)
{
    dc1394video_mode_t video_mode;
    dc1394framerates_t framerate_list;
    dc1394framerate_t frame_rate_index;
    Camwire_state settings;
    int width, height;
    Camwire_pixel coding;
    uint32_t old_num_packets, new_num_packets;
    uint32_t old_packet_size, new_packet_size;
    double actual_frame_rate;
    Camwire_state *shadow_state;
    Camwire_conf config;

    ERROR_IF_NULL(c_handle);
    if (frame_rate < 0.0)
    {
	DPRINTF("frame_rate argument is negative.");
    return CAMERA_FAILURE;
    }

    video_mode = get_1394_video_mode(c_handle);
    ERROR_IF_ZERO(video_mode);
    if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_get_supported_framerates(
		camwire_handle_get_camera(c_handle),
		video_mode,
		&framerate_list));
	if (framerate_list.num == 0)
	{
	    DPRINTF("dc1394_video_get_supported_framerates() returned an empty list.");
        return CAMERA_FAILURE;
	}
	frame_rate_index = convert_framerate2index(frame_rate, &framerate_list);
	ERROR_IF_ZERO(frame_rate_index);
	
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_set_framerate(camwire_handle_get_camera(c_handle),
				       frame_rate_index));
	actual_frame_rate = convert_index2framerate(frame_rate_index);
	shadow_state = get_shadow_state(c_handle);
	ERROR_IF_NULL(shadow_state);
	shadow_state->frame_rate = actual_frame_rate;
    }
    else if (variable_image_size(video_mode))  /* Format 7.*/
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_frame_size(c_handle, &width, &height));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_pixel_coding(c_handle, &coding));
    ERROR_IF_CAMERA_FAIL(
	    get_current_settings(c_handle, &settings));

	/* It's safe to use convert_numpackets2packetsize() here because
	   the color_id and image_size are not changing, therefore
	  dc1394_format7_get_packet_parameters() (or
	  dc1394_format7_get_total_bytes()) will return an up-to-date
	   max_bytes (or total_bytes) value: */
	old_num_packets =
	    convert_framerate2numpackets(c_handle, settings.frame_rate);
	ERROR_IF_ZERO(old_num_packets);
	old_packet_size = convert_numpackets2packetsize(c_handle,
							old_num_packets,
							width, height, coding);
	ERROR_IF_ZERO(old_packet_size);
	new_num_packets = convert_framerate2numpackets(c_handle, frame_rate);
	ERROR_IF_ZERO(new_num_packets);
	new_packet_size = convert_numpackets2packetsize(c_handle,
							new_num_packets,
							width, height, coding);
	ERROR_IF_ZERO(new_packet_size);

	/* Only proceed if frame rate has actually changed: */
	if (old_packet_size != new_packet_size)
	{
	    settings.frame_rate = frame_rate;

	    /* Set the new frame rate by re-initializing the camera: */
        ERROR_IF_CAMERA_FAIL(
		camwire_get_config(c_handle, &config));
        ERROR_IF_CAMERA_FAIL(
		reconnect_cam(c_handle, &config, &settings));
	}
    }
    else
    {
	DPRINTF("Unsupported camera format.");
    return CAMERA_FAILURE;
    }
    return CAMERA_SUCCESS;
} /* camwire_set_framerate() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_shutter(const Camwire_handle c_handle, double *shutter)
{
    Camwire_state *shadow_state;
    uint32_t shutter_reg;
    Camwire_conf config;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *shutter = shadow_state->shutter;

    if (!feature_is_usable(get_feature_capability(c_handle, DC1394_FEATURE_SHUTTER)))
    {
	/* Camera has no usable shutter:*/
    return CAMERA_SUCCESS;  /* Return shadow values.*/
    }
    
    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_get_value(camwire_handle_get_camera(c_handle),
				     DC1394_FEATURE_SHUTTER,
				     &shutter_reg));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_config(c_handle, &config));
	*shutter =
	    config.exposure_offset + shutter_reg*config.exposure_quantum;
	shadow_state->shutter = *shutter;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_shutter() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_shutter(const Camwire_handle c_handle, const double shutter)
{
    Camwire_state *shadow_state;
    dc1394feature_info_t *cap;
    Camwire_conf config;
    int shutter_reg;  /* Allow negative values.*/

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    cap = get_feature_capability(c_handle, DC1394_FEATURE_SHUTTER);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	shadow_state->shutter = shutter;
	DPRINTF("Camera reported no usable shutter.");
    return CAMERA_FAILURE;
    }

    /* Transform to register value: */
    ERROR_IF_CAMERA_FAIL(
	camwire_get_config(c_handle, &config));
    shutter_reg = (int)
	((shutter - config.exposure_offset)/config.exposure_quantum + 0.5);
    
    /* Limit shutter_reg to the allowed range: */
    if (shutter_reg < cap->min)  shutter_reg = cap->min;
    if (shutter_reg > cap->max)  shutter_reg = cap->max;

    ERROR_IF_DC1394_FAIL(
	dc1394_feature_set_value(camwire_handle_get_camera(c_handle),
				 DC1394_FEATURE_SHUTTER,
				 shutter_reg));
    shadow_state->shutter =
	config.exposure_offset + shutter_reg*config.exposure_quantum;

    return CAMERA_SUCCESS;
} /* camwire_set_shutter() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_trigger_source(const Camwire_handle c_handle, int *external)
{
    Camwire_state *shadow_state;
    dc1394switch_t trigger_on;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *external = shadow_state->external_trigger;

    if (!feature_is_usable(get_feature_capability(c_handle, DC1394_FEATURE_TRIGGER)))
    {
	/* Camera has no usable trigger:*/
    return CAMERA_SUCCESS;  /* Return shadow values.*/
    }
    
    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_external_trigger_get_power(camwire_handle_get_camera(c_handle),
					      &trigger_on));
	if (trigger_on == DC1394_OFF)  *external = 0;
	else                           *external = 1;
	shadow_state->external_trigger = *external;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_trigger_source() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_trigger_source(const Camwire_handle c_handle,
			       const int external)
{
    Camwire_state *shadow_state;
    dc1394switch_t on_off;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    if (!feature_is_usable(get_feature_capability(c_handle, DC1394_FEATURE_TRIGGER)))
    {
	shadow_state->external_trigger = external;
	DPRINTF("Camera reported no usable trigger.");
    return CAMERA_FAILURE;
    }
    if (external != 0)  on_off = DC1394_ON;
    else                on_off = DC1394_OFF;
    ERROR_IF_DC1394_FAIL(
	dc1394_external_trigger_set_power(camwire_handle_get_camera(c_handle),
					  on_off));
    shadow_state->external_trigger = external;
    return CAMERA_SUCCESS;
} /* camwire_set_trigger_source() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_trigger_polarity(const Camwire_handle c_handle, int *rising)
{
    Camwire_state *shadow_state;
    dc1394trigger_polarity_t polarity;
    
    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *rising = shadow_state->trigger_polarity;

    if (!feature_is_usable(get_feature_capability(c_handle, DC1394_FEATURE_TRIGGER)))
    {
	/* Camera has no usable trigger:*/
    return CAMERA_SUCCESS;  /* Return shadow values.*/
    }
    
    if (!shadow_state->shadow)
    {
	/* Assume we can read trigger polarity even if it is not settable.*/
	ERROR_IF_DC1394_FAIL(
	    dc1394_external_trigger_get_polarity(camwire_handle_get_camera(c_handle),
						 &polarity));
	if (polarity == DC1394_TRIGGER_ACTIVE_LOW)  *rising = 0;
	else                                        *rising = 1;
	shadow_state->trigger_polarity = *rising;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_trigger_polarity() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_trigger_polarity(const Camwire_handle c_handle,
				 const int rising)
{
    Camwire_state *shadow_state;
    dc1394feature_info_t *cap;
    dc1394trigger_polarity_t polarity;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);

    shadow_state->trigger_polarity = rising;
    cap = get_feature_capability(c_handle, DC1394_FEATURE_TRIGGER);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	DPRINTF("Camera reported no usable trigger.");
    return CAMERA_FAILURE;
    }
    if (cap->polarity_capable != DC1394_TRUE)
    {
	DPRINTF("Camera reported no changeable trigger polarity.");
    return CAMERA_FAILURE;
    }
    if (rising != 0)  polarity = DC1394_TRIGGER_ACTIVE_HIGH;
    else              polarity = DC1394_TRIGGER_ACTIVE_LOW;
    ERROR_IF_DC1394_FAIL(
	dc1394_external_trigger_set_polarity(camwire_handle_get_camera(c_handle),
					     polarity));
    shadow_state->trigger_polarity = rising;
    return CAMERA_SUCCESS;
} /* camwire_set_trigger_polarity() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_gain(const Camwire_handle c_handle, double *gain)
{
    dc1394feature_info_t *cap;
    Camwire_state *shadow_state;
    uint32_t gain_reg;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *gain = shadow_state->gain;

    cap = get_feature_capability(c_handle, DC1394_FEATURE_GAIN);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	/* Camera has no usable gain:*/
    return CAMERA_SUCCESS;  /* Return shadow values.*/
    }
    
    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_get_value(camwire_handle_get_camera(c_handle),
				     DC1394_FEATURE_GAIN,
				     &gain_reg));
	if ((int)gain_reg >= cap->min &&
	    (int)gain_reg <= cap->max)
	{
	    if (cap->max != cap->min)
		*gain = (double)(gain_reg - cap->min)/(cap->max - cap->min);
	    else
		*gain = 0.0;
	}
	else
	{
	    DPRINTF("Invalid gain min and max values.");
        return CAMERA_FAILURE;
	}
	shadow_state->gain = *gain;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_gain() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_gain(const Camwire_handle c_handle,
			   const double gain)
{
    Camwire_state *shadow_state;
    dc1394feature_info_t *cap;
    uint32_t gain_reg;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    cap = get_feature_capability(c_handle, DC1394_FEATURE_GAIN);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	shadow_state->gain = gain;
	DPRINTF("Camera reported no usable gain.");
    return CAMERA_FAILURE;
    }

    /* Check limits: */
    if (gain < 0.0 || gain > 1.0)
    {
	DPRINTF("Gain argument should be in the range [0.0, 1.0].");
    return CAMERA_FAILURE;
    }

    /* Update the camera with new gains: */
    if (cap->max >= cap->min)
	gain_reg = cap->min + gain*(cap->max - cap->min) + 0.5;
    else
	gain_reg = 0;

    ERROR_IF_DC1394_FAIL(
	dc1394_feature_set_value(camwire_handle_get_camera(c_handle),
				 DC1394_FEATURE_GAIN,
				 gain_reg));
    
    if (cap->max > cap->min)
	shadow_state->gain = (double)(gain_reg - cap->min)/(cap->max - cap->min);
    else
	shadow_state->gain = 0.0;
    
    return CAMERA_SUCCESS;
} /* camwire_set_gain() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_brightness(const Camwire_handle c_handle, double *brightness)
{
    dc1394feature_info_t *cap;
    Camwire_state *shadow_state;
    uint32_t brightness_reg;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    *brightness = shadow_state->brightness;

    cap = get_feature_capability(c_handle, DC1394_FEATURE_BRIGHTNESS);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	/* Camera has no usable brightness:*/
    return CAMERA_SUCCESS;  /* Return shadow values.*/
    }
    
    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_get_value(camwire_handle_get_camera(c_handle),
				     DC1394_FEATURE_BRIGHTNESS,
				     &brightness_reg));
	if ((int)brightness_reg >= cap->min &&
	    (int)brightness_reg <= cap->max)
	{
	    if (cap->max > cap->min)
		*brightness = 2.0*(double)(brightness_reg - cap->min) /
		    (cap->max - cap->min) - 1.0;
	    else
		*brightness = 0.0;
	}
	else
	{
	    DPRINTF("Invalid brightness min and max values.");
        return CAMERA_FAILURE;
	}
	shadow_state->brightness = *brightness;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_brightness() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_brightness(const Camwire_handle c_handle,
			   const double brightness)
{
    Camwire_state *shadow_state;
    dc1394feature_info_t *cap;
    uint32_t brightness_reg;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    cap = get_feature_capability(c_handle, DC1394_FEATURE_BRIGHTNESS);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	shadow_state->brightness = brightness;
	DPRINTF("Camera reported no usable brightness.");
    return CAMERA_FAILURE;
    }

    /* Check limits: */
    if (brightness < -1.0 || brightness > 1.0)
    {
	DPRINTF("Brightness argument should be in the range [-1.0, +1.0].");
    return CAMERA_FAILURE;
    }

    /* Update the camera with new brightness: */
    if (cap->max >= cap->min)
	brightness_reg = cap->min + 0.5*(brightness + 1.0)*(cap->max - cap->min) + 0.5;
    else
	brightness_reg = 0;
    
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_set_value(camwire_handle_get_camera(c_handle),
				 DC1394_FEATURE_BRIGHTNESS,
				 brightness_reg));
    
    if (cap->max > cap->min)
	shadow_state->brightness =
	    2.0*(double)(brightness_reg - cap->min)/(cap->max - cap->min) - 1.0;
    else
	shadow_state->brightness = 0.0;
    
    return CAMERA_SUCCESS;
} /* camwire_set_brightness() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_white_balance(const Camwire_handle c_handle, double bal[2])
{
    dc1394feature_info_t *cap;
    Camwire_state *shadow_state;
    uint32_t blue_reg, red_reg;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    bal[0] = shadow_state->white_balance[0];
    bal[1] = shadow_state->white_balance[1];

    cap = get_feature_capability(c_handle, DC1394_FEATURE_WHITE_BALANCE);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	/* Camera has no usable white balance:*/
    return CAMERA_SUCCESS;  /* Return shadow values.*/
    }
    
    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_whitebalance_get_value(camwire_handle_get_camera(c_handle),
						  &blue_reg, &red_reg));
	if ((int)blue_reg >= cap->min && (int)blue_reg <= cap->max &&
	    (int)red_reg >= cap->min && (int)red_reg <= cap->max)
	{
	    if (cap->max != cap->min)
	    {
		bal[0] = (double)(blue_reg - cap->min)/(cap->max - cap->min);
		bal[1] = (double)(red_reg - cap->min)/(cap->max - cap->min);
	    }
	    else
	    {
		bal[0] = bal[1] = 0.0;
	    }
	}
	else
	{
	    DPRINTF("Invalid white balance min and max values.");
        return CAMERA_FAILURE;
	}
	shadow_state->white_balance[0] = bal[0];
	shadow_state->white_balance[1] = bal[1];
    }
    return CAMERA_SUCCESS;
} /* camwire_get_white_balance() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_white_balance(const Camwire_handle c_handle, const double bal[2])
{
    Camwire_state *shadow_state;
    dc1394feature_info_t *cap;
    uint32_t blue_reg, red_reg;

    ERROR_IF_NULL(c_handle);
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    cap = get_feature_capability(c_handle, DC1394_FEATURE_WHITE_BALANCE);
    ERROR_IF_NULL(cap);
    if (!feature_is_usable(cap))
    {
	shadow_state->white_balance[0] = bal[0];
	shadow_state->white_balance[1] = bal[1];
	DPRINTF("Camera reported no usable white balance.");
    return CAMERA_FAILURE;
    }

    /* Check limits: */
    if (bal[0] < 0.0 || bal[0] > 1.0 || bal[1] < 0.0 || bal[1] > 1.0)
    {
	DPRINTF("White balance arguments should be in the range [0.0, 1.0].");
    return CAMERA_FAILURE;
    }

    /* Update the camera with new balance: */
    if (cap->max >= cap->min)
    {
	blue_reg = cap->min + bal[0]*(cap->max - cap->min) + 0.5;
	red_reg  = cap->min + bal[1]*(cap->max - cap->min) + 0.5;
    }
    else
    {
	blue_reg = red_reg = 0;
    }
    
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_whitebalance_set_value(camwire_handle_get_camera(c_handle),
					      blue_reg, red_reg));
    
    if (cap->max > cap->min)
    {
	shadow_state->white_balance[0] =
	    (double)(blue_reg - cap->min)/(cap->max - cap->min);
	shadow_state->white_balance[1]  =
	    (double)(red_reg - cap->min)/(cap->max - cap->min);
    }
    else
    {
	shadow_state->white_balance[0] = shadow_state->white_balance[1] = 0.0;
    }

    return CAMERA_SUCCESS;
} /* camwire_set_white_balance() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
  Only AVT cameras are supported here at the moment.
*/

int camwire_get_gamma(const Camwire_handle c_handle, int *gamma_on)
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    dc1394bool_t lut_set;
    uint32_t lut_num;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);
    *gamma_on = shadow_state->gamma;

    if (!internal_status->extras->gamma_capable)
    {
	/* Camera has no gamma: */
    return CAMERA_SUCCESS;
    }

    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_avt_get_lut(camwire_handle_get_camera(c_handle),
			       &lut_set, &lut_num));
	if (lut_set == DC1394_TRUE)  *gamma_on = 1;
	else                         *gamma_on = 0;
	shadow_state->gamma = *gamma_on;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_gamma() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
  Only AVT cameras are supported here at the moment.
*/
int camwire_set_gamma(const Camwire_handle c_handle, const int gamma_on)
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    dc1394bool_t lut_set;
    uint32_t lut_num;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);

    if (!internal_status->extras->gamma_capable)
    {
	/* Gamma is always switched off if the camera can't do it: */
	shadow_state->gamma = 0;
	DPRINTF("Camera reported no gamma capability.");
    return CAMERA_FAILURE;
    }

    lut_set = (gamma_on ? DC1394_TRUE : DC1394_FALSE);
    lut_num = 0;
    ERROR_IF_DC1394_FAIL(
	dc1394_avt_set_lut(camwire_handle_get_camera(c_handle),
			   lut_set, lut_num));
    shadow_state->gamma = (gamma_on ? 1 : 0);
    
    return CAMERA_SUCCESS;
} /* camwire_set_gamma() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_inv_gamma(const Camwire_handle c_handle, const void *cam_buf,
		      void *lin_buf, const unsigned long max_val)
{
    User_handle internal_status;
    Camwire_pixel coding;
    size_t i, num_components;
    int width, height, depth;
    uint16_t *endp, *outp;
    const uint8_t *inp;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);

    /* Checking: */
    if (max_val == 0 || max_val > UINT16_MAX)
    {
	DPRINTF("3rd argument is zero or exceeds 16-bit range.");
    return CAMERA_FAILURE;
    }
    ERROR_IF_CAMERA_FAIL(
	camwire_get_pixel_coding(c_handle, &coding));
    if (component_depth(coding) != 8)
    {
	DPRINTF("Pixel coding does not have 8-bit components.");
    return CAMERA_FAILURE;
    }
    
    /* Initialize look-up table if necessary: */
    if ((uint16_t) max_val != internal_status->extras->gamma_maxval)
    {
	for (i=0; i<256; ++i)
	    gamma_lut[i] = htons((uint16_t)(max_val*gamma_inv[i] + 0.5));
	internal_status->extras->gamma_maxval = (uint16_t) max_val;
    }

    /* Transform: */
    ERROR_IF_CAMERA_FAIL(
	camwire_get_frame_size(c_handle, &width, &height));
    ERROR_IF_CAMERA_FAIL(
	camwire_pixel_depth(coding, &depth));
    num_components = (size_t) width*height*depth/8;

    inp = (uint8_t *)cam_buf;
    endp = (uint16_t *)lin_buf + num_components;
    for (outp=lin_buf; outp!=endp; ++outp)  *outp = gamma_lut[*inp++];
    
    return CAMERA_SUCCESS;
} /* camwire_inv_gamma() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
  Only AVT cameras are supported here at the moment.
*/

int camwire_get_colour_correction(const Camwire_handle c_handle, int *corr_on)
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    dc1394bool_t on_off;
    int32_t val[9];
    double coef[9];

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);
    *corr_on = shadow_state->colour_corr;

    if (!internal_status->extras->colour_corr_capable)
    {
	/* Camera has no colour correction.  Return the default corr-on of 0: */
    return CAMERA_SUCCESS;
    }

    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_avt_get_color_corr(camwire_handle_get_camera(c_handle), &on_off,
				      &val[0], &val[1], &val[2],
				      &val[3], &val[4], &val[5],
				      &val[6], &val[7], &val[8]));
	*corr_on = (on_off == DC1394_FALSE);
	/* Note 0 means on (see AVT Stingray Tech Manual).  There is a
	   bug in dc1394_avt_get_color_corr() &
	   dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/
	shadow_state->colour_corr = *corr_on;
	convert_avtvalues2colourcoefs(val, coef);	
	memcpy(shadow_state->colour_coef, coef, 9*sizeof(coef[0]));
    }
    return CAMERA_SUCCESS;
} /* camwire_get_colour_correction() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
  Only AVT cameras are supported here at the moment.
*/
int camwire_set_colour_correction(const Camwire_handle c_handle, const int corr_on)
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    double coef[9];
    int32_t val[9];
    dc1394bool_t on_off;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);

    if (!internal_status->extras->colour_corr_capable)
    {
	/* Colour correction is always switched off if the camera can't do it: */
	shadow_state->colour_corr = 0;
	DPRINTF("Camera reported no colour correction capability.");
    return CAMERA_FAILURE;
    }

    on_off = (corr_on ? DC1394_FALSE : DC1394_TRUE);
    /* Note 0 means on (see AVT Stingray Tech Manual).  There is a bug
       in dc1394_avt_get_color_corr() &
       dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/

    ERROR_IF_CAMERA_FAIL(
	camwire_get_colour_coefficients(c_handle, coef));
    convert_colourcoefs2avtvalues(coef, val);

    ERROR_IF_DC1394_FAIL(
	dc1394_avt_set_color_corr(camwire_handle_get_camera(c_handle),
				  on_off,
				  DC1394_FALSE,
				  val[0], val[1], val[2],
				  val[3], val[4], val[5],
				  val[6], val[7], val[8]));
    shadow_state->colour_corr = (corr_on ? 1 : 0);
    
    return CAMERA_SUCCESS;
} /* camwire_set_colour_correction() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
  Only AVT cameras are supported here at the moment.
*/

int camwire_get_colour_coefficients(const Camwire_handle c_handle, double coef[9])
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    dc1394bool_t on_off;
    int32_t val[9];

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);
    memcpy(coef, shadow_state->colour_coef, 9*sizeof(coef[0]));

    if (!internal_status->extras->colour_corr_capable)
    {
	/* Camera cannot change colour correction coefficients: */
    return CAMERA_SUCCESS;
    }

    if (!shadow_state->shadow)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_avt_get_color_corr(camwire_handle_get_camera(c_handle),
				      &on_off,
				      &val[0], &val[1], &val[2],
				      &val[3], &val[4], &val[5],
				      &val[6], &val[7], &val[8]));
	convert_avtvalues2colourcoefs(val, coef);	
	memcpy(shadow_state->colour_coef, coef, 9*sizeof(coef[0]));
	shadow_state->colour_corr = (on_off == DC1394_FALSE);
	/* Note 0 means on (see AVT Stingray Tech Manual).  There is a
	   bug in dc1394_avt_get_color_corr() &
	   dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/
    }
    return CAMERA_SUCCESS;
} /* camwire_get_colour_coefficients() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
  Only AVT cameras are supported here at the moment.
*/
int camwire_set_colour_coefficients(const Camwire_handle c_handle,
				    const double coef[9])
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    int32_t val[9];
    int corr_on;
    dc1394bool_t on_off;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);

    if (!internal_status->extras->colour_corr_capable)
    {
	/* Colour coefficient matrix is fixed if the camera can't set it: */
	DPRINTF("Camera reported no capability to change colour coefficients.");
    return CAMERA_FAILURE;
    }

    convert_colourcoefs2avtvalues(coef, val);

    ERROR_IF_CAMERA_FAIL(
	camwire_get_colour_correction(c_handle, &corr_on));
    on_off = (corr_on ? DC1394_FALSE : DC1394_TRUE);
    /* Note 0 means on (see AVT Stingray Tech Manual).  There is a bug
       in dc1394_avt_get_color_corr() &
       dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/

    ERROR_IF_DC1394_FAIL(
	dc1394_avt_set_color_corr(camwire_handle_get_camera(c_handle),
				  on_off,
				  DC1394_FALSE,
				  val[0], val[1], val[2],
				  val[3], val[4], val[5],
				  val[6], val[7], val[8]));
    convert_avtvalues2colourcoefs(val, shadow_state->colour_coef);
    
    return CAMERA_SUCCESS;
} /* camwire_set_colour_coefficients() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* We have a concept of single-shot/continuous that is orthogonal to
   run/stop.  The closest equivalents in IIDC DCAM are the
   ISO_EN/Continuous_shot and One_shot/Multi_shot registers accessed
   through dc1394_video_get/set_transmission() and
   dc1394_video_get/set_one_shot().

   Our orthogonal concept is implemented with the functions
   camwire_get/set_single_shot() and camwire_get/set_run_stop().
   
   We are forced to maintain our own idea of the single-shot/continuous
   status in shadow_state->single_shot.  libdc1394 only tells us the
   state of the ISO_EN/Continuous_Shot register which is used to
   describe both continuous and run statuses, and the One_Shot register
   which asynchronously auto-clears itself after transmitting a frame.
   We use every opportunity to re-align our internal status with the
   hardware status.  */

int camwire_get_single_shot(const Camwire_handle c_handle, int *single_shot_on)
{
    dc1394switch_t iso_en;
    dc1394bool_t one_shot_set;
    User_handle internal_status;
    Camwire_state *shadow_state;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);
    *single_shot_on = shadow_state->single_shot;

    if (!internal_status->extras->single_shot_capable)
    {
	/* Camera has no single-shot:*/
    return CAMERA_SUCCESS;
    }

    if (!shadow_state->shadow)
    {  /* Don't use shadow: ask the camera: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_get_transmission(camwire_handle_get_camera(c_handle),
					  &iso_en));
	
	if (iso_en == DC1394_ON)
	{  /* Running in continuous mode.*/
	    shadow_state->running = 1;
	    *single_shot_on = 0;
	}
	else
	{  /* Running in single-shot mode or stopped.*/
	    ERROR_IF_DC1394_FAIL(
		dc1394_video_get_one_shot(camwire_handle_get_camera(c_handle),
					  &one_shot_set));
	    if (one_shot_set == DC1394_TRUE)
	    {  /* Camera is running.*/
		shadow_state->running = 1;
		*single_shot_on = 1;
	    }
	    else
	    {  /* Camera is stopped.*/
		shadow_state->running = 0;
		*single_shot_on = shadow_state->single_shot;  /* Remember.*/
	    }
	}
	shadow_state->single_shot = *single_shot_on;
    }
    return CAMERA_SUCCESS;
} /* camwire_get_single_shot() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_single_shot(const Camwire_handle c_handle,
			    const int single_shot_on)
{
    dc1394switch_t iso_en;
    dc1394bool_t one_shot_set;
    User_handle internal_status;
    Camwire_state *shadow_state;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);

    if (!internal_status->extras->single_shot_capable)
    {
	/* Single-shot is always switched off if the camera can't do it: */
	shadow_state->single_shot = 0;
	DPRINTF("Camera reported no single shot capability.");
    return CAMERA_FAILURE;
    }

    if (shadow_state->shadow)
    {
	if (shadow_state->running)
	{  /* We *think* the camera is running.*/
	    if (!shadow_state->single_shot && single_shot_on)
	    { 	/* Camera is running: change to single-shot: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_transmission(
			camwire_handle_get_camera(c_handle),
			DC1394_OFF));
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_one_shot(
			camwire_handle_get_camera(c_handle),
			DC1394_ON));
	    }
	    else if (shadow_state->single_shot && !single_shot_on)
	    { 	/* Don't know if camera is still runnning: let's find out: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_get_one_shot(camwire_handle_get_camera(c_handle),
					      &one_shot_set));
		if (one_shot_set == DC1394_TRUE)
		{  /* Camera is still runnning: change to continuous: */
		    ERROR_IF_DC1394_FAIL(
			dc1394_video_set_one_shot(camwire_handle_get_camera(c_handle),
						  DC1394_OFF));
		    ERROR_IF_DC1394_FAIL(
			dc1394_video_set_transmission(
			    camwire_handle_get_camera(c_handle),
			    DC1394_ON));
		}
		else
		{  /* Camera has finished single shot: update shadow state: */
		    shadow_state->running = 0;
		}
	    }
	}
	/* else change only the internal state.*/
    }
    else
    {  /* Don't use shadow: ask the camera: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_get_transmission(camwire_handle_get_camera(c_handle),
					  &iso_en));
	
	if (iso_en == DC1394_ON && single_shot_on)
	{ 	/* Camera is running: change to single-shot: */
	    ERROR_IF_DC1394_FAIL(
		dc1394_video_set_transmission(camwire_handle_get_camera(c_handle),
					      DC1394_OFF));
	    ERROR_IF_DC1394_FAIL(
		dc1394_video_set_one_shot(camwire_handle_get_camera(c_handle),
					  DC1394_ON));
	    shadow_state->running = 1;
	}
	else if (iso_en == DC1394_OFF && !single_shot_on)
	{
	    ERROR_IF_DC1394_FAIL(
		dc1394_video_get_one_shot(camwire_handle_get_camera(c_handle),
					  &one_shot_set));
	    if (one_shot_set == DC1394_TRUE)
	    { 	/* Camera is still runnning: change to continuous: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_one_shot(camwire_handle_get_camera(c_handle),
					      DC1394_OFF));
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_transmission(
			camwire_handle_get_camera(c_handle),
			DC1394_ON));
		shadow_state->running = 1;
	    }
	    /* else change only the internal state.*/
	}
    }
    shadow_state->single_shot = single_shot_on;

    return CAMERA_SUCCESS;
} /* camwire_set_single_shot() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_run_stop(const Camwire_handle c_handle, int *runsts)
{
    dc1394switch_t iso_en;
    dc1394bool_t one_shot_set;
    User_handle internal_status;
    Camwire_state *shadow_state;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);
    if (shadow_state->shadow)
    {
	*runsts = shadow_state->running;
	
	/* One_Shot register self-clears after transmission: */
	if (shadow_state->running && shadow_state->single_shot)
	{  /* Don't know if camera is still runnning: let's find out: */
	    ERROR_IF_DC1394_FAIL(
		dc1394_video_get_one_shot(camwire_handle_get_camera(c_handle),
					  &one_shot_set));
	    if (one_shot_set == DC1394_FALSE)
	    {  /* Camera has finished single shot: update shadow state: */
		shadow_state->running = *runsts = 0;
	    }
	}
    }
    else
    {  /* Don't use shadow: ask the camera: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_get_transmission(camwire_handle_get_camera(c_handle),
					  &iso_en));
	if (iso_en == DC1394_ON) 
	{ 	/* Camera is running in continuous mode: */
	    shadow_state->single_shot = 0;
	    *runsts = 1;
	}
	else
	{
	    *runsts = 0;
	    if (internal_status->extras->single_shot_capable)
	    {
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_get_one_shot(camwire_handle_get_camera(c_handle),
					      &one_shot_set));
		if (one_shot_set == DC1394_TRUE) 
		{ 	/* Camera is running in single-shot mode: */
		    shadow_state->single_shot = 1;
		    *runsts = 1;
		}
	    }
	}
	shadow_state->running = *runsts;
    }
    
    return CAMERA_SUCCESS;
} /* camwire_get_run_stop() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_set_run_stop(const Camwire_handle c_handle, const int runsts)
{
    dc1394switch_t iso_en;
    dc1394bool_t one_shot_set;
    User_handle internal_status;
    Camwire_state *shadow_state;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);
    if (shadow_state->shadow)
    {
	if (shadow_state->single_shot)
	{  /* Single-shot.*/
	    if (shadow_state->running && !runsts)
	    { 	/* Stop camera (even if it has already stopped): */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_one_shot(camwire_handle_get_camera(c_handle),
					      DC1394_OFF));
	    }
	    else if (runsts)
	    {  /* Run in single-shot mode (even if we think it is already): */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_one_shot(camwire_handle_get_camera(c_handle),
					      DC1394_ON));
	    }
	    /* else do nothing.*/
	}
	else
	{  /* Continuous.*/
	    if (shadow_state->running && !runsts)
	    {  /* Stop camera: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_transmission(
			camwire_handle_get_camera(c_handle),
			DC1394_OFF));
	    }
	    else if (!shadow_state->running && runsts)
	    { 	/* Run camera: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_transmission(
			camwire_handle_get_camera(c_handle),
			DC1394_ON));
	    }
	    /* else do nothing.*/
	}
    }
    else
    {  /* Don't use shadow: ask the camera: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_get_transmission(camwire_handle_get_camera(c_handle),
					  &iso_en));
	if (iso_en == DC1394_ON) 
	{  /* Camera is running in continuous mode: */
	    shadow_state->single_shot = 0;
	    if (!runsts) 
	    {  /* Stop camera: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_transmission(
			camwire_handle_get_camera(c_handle),
			DC1394_OFF));
	    }
	    /* else do nothing.*/
	}
	else
	{
	    if (internal_status->extras->single_shot_capable)
	    {
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_get_one_shot(camwire_handle_get_camera(c_handle),
					      &one_shot_set));
		if (one_shot_set == DC1394_TRUE) 
		{  /* Camera is running in single-shot mode: */
		    shadow_state->single_shot = 1;
		    if (!runsts)
		    { 	/* Stop camera: */
			ERROR_IF_DC1394_FAIL(
			    dc1394_video_set_one_shot(camwire_handle_get_camera(c_handle),
						      DC1394_OFF));
		    }
		    /* else do nothing.*/
		}
		else if (runsts)
		{  /* Camera is stopped.  Have to use shadow to decide: */
		    if (!shadow_state->single_shot) 
		    {  /* Run in continuous mode: */
			ERROR_IF_DC1394_FAIL(
			    dc1394_video_set_transmission(
				camwire_handle_get_camera(c_handle),
				DC1394_ON));
		    }
		    else
		    {  /* Run in single-shot mode: */
			ERROR_IF_DC1394_FAIL(
			    dc1394_video_set_one_shot(camwire_handle_get_camera(c_handle),
						      DC1394_ON));
		    }
		}
	    }
	    else if (runsts)
	    {  /* Camera is stopped.  Run in continuous mode: */
		ERROR_IF_DC1394_FAIL(
		    dc1394_video_set_transmission(
			camwire_handle_get_camera(c_handle),
			DC1394_ON));
	    }
	}
    }
    shadow_state->running = runsts;
    
    return CAMERA_SUCCESS;
} /* camwire_set_run_stop() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_copy_next_frame(const Camwire_handle c_handle, void *buffer,
			    int *buffer_lag)
{
    void *buf_ptr;
    int width, height, depth;
    Camwire_pixel coding;

    ERROR_IF_NULL(c_handle);
    ERROR_IF_CAMERA_FAIL(
	camwire_point_next_frame(c_handle, &buf_ptr, buffer_lag));

    /* Copy the minimum number of bytes to avoid segfaults if the user's
       frame buffer is only just big enough to take one frame.  Note
       that internal_status->frame->total_bytes should not be used
       because it may include padding that the user has not made
       provision for: */
    ERROR_IF_CAMERA_FAIL(
	camwire_get_frame_size(c_handle, &width, &height));
    ERROR_IF_CAMERA_FAIL(
	camwire_get_pixel_coding(c_handle, &coding));
    ERROR_IF_CAMERA_FAIL(
	camwire_pixel_depth(coding, &depth));
    memcpy(buffer, buf_ptr, (size_t) width*height*depth/8);

    camwire_unpoint_frame(c_handle);

    return CAMERA_SUCCESS;
} /* camwire_copy_next_frame() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_point_next_frame(const Camwire_handle c_handle, void **buf_ptr,
			     int *buffer_lag)
{
    User_handle internal_status;
    int dc1394_return;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);

    if (internal_status->frame_lock)
    {
	DPRINTF("Can't point to new frame before unpointing previous frame.");
    return CAMERA_FAILURE;
    }
    dc1394_return = dc1394_capture_dequeue(camwire_handle_get_camera(c_handle),
					   DC1394_CAPTURE_POLICY_WAIT,
					   &internal_status->frame);
    if (dc1394_return != DC1394_SUCCESS)
    {
	internal_status->frame = 0;
	DPRINTF("dc1394_capture_dequeue() failed.");
    return CAMERA_FAILURE;
    }
    ERROR_IF_NULL(internal_status->frame);
    *buf_ptr = (void *)internal_status->frame->image;
    internal_status->frame_lock = 1;

    /* Record buffer timestamp for later use by camwire_get_timestamp(),
       because we don't want to assume that dc1394_capture_enqueue()
       does not mess with its frame arg:*/
    internal_status->dma_timestamp = internal_status->frame->timestamp*1.0e-6;

    /* Increment the frame number if we have a frame: */
    ++internal_status->frame_number;

    if (buffer_lag)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_framebuffer_lag(c_handle, buffer_lag));
    }
    return CAMERA_SUCCESS;
} /* camwire_point_next_frame() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_point_next_frame_poll(const Camwire_handle c_handle,
				  void **buf_ptr, int *buffer_lag)
{
    User_handle internal_status;
    int dc1394_return;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);

    if (internal_status->frame_lock)
    {
	DPRINTF("Can't point to new frame before unpointing previous frame.");
    return CAMERA_FAILURE;
    }
    dc1394_return = dc1394_capture_dequeue(camwire_handle_get_camera(c_handle),
					   DC1394_CAPTURE_POLICY_POLL,
					   &internal_status->frame);
    if (dc1394_return != DC1394_SUCCESS)
    {
	internal_status->frame = 0;
	DPRINTF("dc1394_capture_dequeue() failed.");
    return CAMERA_FAILURE;
    }
    if (internal_status->frame == 0)
    {
	*buf_ptr = 0;  // No frame ready.
    }
    else
    {
	ERROR_IF_NULL(internal_status->frame);
	*buf_ptr = (void *)internal_status->frame->image;
	internal_status->frame_lock = 1;
	
	/* Record buffer timestamp for later use by
	   camwire_get_timestamp(), because we don't want to assume that
	   dc1394_capture_enqueue() does not mess with its frame arg:*/
	internal_status->dma_timestamp = internal_status->frame->timestamp*1.0e-6;
	
	/* Increment the frame number if we have a frame: */
	++internal_status->frame_number;
    }

    if (buffer_lag)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_framebuffer_lag(c_handle, buffer_lag));
    }
    return CAMERA_SUCCESS;
} /* camwire_point_next_frame_poll() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_unpoint_frame(const Camwire_handle c_handle)
{
    User_handle internal_status;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    if (internal_status->frame_lock)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_capture_enqueue(camwire_handle_get_camera(c_handle),
				   internal_status->frame));
	internal_status->frame = 0;
	internal_status->frame_lock = 0;
    }
    return CAMERA_SUCCESS;
} /* camwire_unpoint_frame() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_get_framenumber(const Camwire_handle c_handle, long *framenumber)
{
    User_handle internal_status;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    *framenumber = internal_status->frame_number;
    return CAMERA_SUCCESS;
} /* camwire_get_framenumber() */

/*
  -----------------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* The estimated timestamp lag (the difference between external trigger
   and DMA buffer filled) has a tolerance of slightly more than half the
   bus cycle period (125 microsecond for IEEE 1394 running at 400 Mb/s)
   because start of transmission has to wait for the next bus cycle. */

int camwire_get_timestamp(const Camwire_handle c_handle, struct timespec *timestamp)
{
    double sensor_transfer_time, bus_transmit_time, trans_time;
    double timelag, estimate;
    uint32_t num_packets;
    double frame_rate, shutter;
    int width, height;
    Camwire_conf config;
    User_handle internal_status;

    ERROR_IF_NULL(c_handle);
    
    ERROR_IF_CAMERA_FAIL(
	camwire_get_framerate(c_handle, &frame_rate));
    num_packets = convert_framerate2numpackets(c_handle, frame_rate);
    ERROR_IF_ZERO(num_packets);
    ERROR_IF_CAMERA_FAIL(
	camwire_get_shutter(c_handle, &shutter));
    ERROR_IF_CAMERA_FAIL(
	camwire_get_frame_size(c_handle, &width, &height));
    ERROR_IF_CAMERA_FAIL(
	camwire_get_config(c_handle, &config));

    sensor_transfer_time = config.line_transfer_time*height;
    /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
    bus_transmit_time = num_packets/convert_busspeed2busfreq(config.bus_speed);
    if (config.transmit_overlap)
    {
	trans_time = (sensor_transfer_time > bus_transmit_time ?
	    sensor_transfer_time : bus_transmit_time);
    }
    else
    {
	trans_time = sensor_transfer_time + bus_transmit_time;
    }
#ifdef USE_JUJU_STACK
    trans_time -= bus_transmit_time;  /* juju stack includes this already. */
#endif
    timelag = config.trig_setup_time +
	shutter +
	config.transmit_setup_time +
	trans_time;

    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    
    estimate = internal_status->dma_timestamp - timelag;
    timestamp->tv_sec = estimate;
    timestamp->tv_nsec = (estimate - timestamp->tv_sec)*1.0e9;
    return CAMERA_SUCCESS;
} /* camwire_get_timestamp() */

/*
  ----------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
/* Camwire_conf is defined in camwire.h.*/

int camwire_get_config(const Camwire_handle c_handle, Camwire_conf *cfg)
{
    User_handle internal_status;
    Camwire_id identifier;
    FILE *conffile;

    ERROR_IF_NULL(c_handle);
    internal_status = camwire_bus_get_userdata(c_handle);
    
    /* Use cached config if it is available: */
    if (internal_status && config_cache_exists(internal_status))
    {
	memcpy(cfg, internal_status->config_cache,
	       sizeof(Camwire_conf));
    }
    else
    { 	/* Read a conf file and cache it.*/
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_identifier(c_handle, &identifier));
	conffile = find_conf_file(&identifier);
	if (conffile)
	{
        ERROR_IF_CAMERA_FAIL(
		read_conf_file(conffile, cfg));
	    fclose(conffile);
	    if (internal_status &&
		internal_status->config_cache)
	    { /* A camera has been created (not strictly necessary).*/
		memcpy(internal_status->config_cache, cfg,
		       sizeof(Camwire_conf));
	    }
	}
	else
	{
	    fprintf(stderr,
    "\n"
    "Camwire could not find a hardware configuration file.\n"
    "Generating a default configuration...\n");
        ERROR_IF_CAMERA_FAIL(
		generate_default_config(c_handle, cfg));
	    printf(
    "\n"
    "----------------------------------------------------------------\n"
            );
        ERROR_IF_CAMERA_FAIL(
		camwire_write_config_to_file(stdout, cfg));
	    printf(
    "----------------------------------------------------------------\n"
    "\n"
    "This is a best guess of the camera and its bus's hardware\n"
    "configuration.  See README_conf in the Camwire distribution for\n"
    "details.\n\n"
    "To create a configuration file, copy the text between (but not\n"
    "including) the ----- lines into a file (and edit as needed).\n"
    "The filename must be identical to one of the camera's ID strings\n"
    "(as may be obtained from camwire_get_identifier()) appended by\n"
    "an extension of \".conf\".\n\n"
    "For the current camera suitable filenames are:\n"
    "%s.conf \t(chip)\n"
    "%s.conf \t(model)\n"
    "%s.conf \t(vendor)\n"
    "Camwire checks for filenames like these in this\n"
    "chip-model-vendor order.  It first looks for the three filenames\n"
    "in the current working directory and after that in a directory\n"
    "given by the CAMERA_CONF environment variable.\n\n",
            identifier.chip, identifier.model, identifier.vendor);
	    fflush(stdout);
        return CAMERA_FAILURE;
	}
    }
    
    return CAMERA_SUCCESS;
} /* camwire_get_config() */

/*
  ----------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_write_config_to_file(FILE *outfile, const Camwire_conf *cfg)
{
    int print_result;

    print_result = fprintf(outfile,
			   "Camwire IEEE 1394 IIDC DCAM hardware configuration:\n"
			   "  bus_speed:           %d\n"
			   "  format:              %d\n"
			   "  mode:                %d\n"
			   "  max_packets:         %d\n"
			   "  min_pixels:          %d\n"
			   "  trig_setup_time:     %g\n"
			   "  exposure_quantum:    %g\n"
			   "  exposure_offset:     %g\n"
			   "  line_transfer_time:  %g\n"
			   "  transmit_setup_time: %g\n"
			   "  transmit_overlap:    %d\n"
			   "  drop_frames:         %d\n"
			   "  dma_device_name:     %s\n",
			   cfg->bus_speed,
			   cfg->format,
			   cfg->mode,
			   cfg->max_packets,
			   cfg->min_pixels,
			   cfg->trig_setup_time,
			   cfg->exposure_quantum,
			   cfg->exposure_offset,
			   cfg->line_transfer_time,
			   cfg->transmit_setup_time,
			   cfg->transmit_overlap,
			   cfg->drop_frames,
			   cfg->dma_device_name);
    if (print_result < 1)
    {
    return CAMERA_FAILURE;
    }
    fflush(outfile);
    return CAMERA_SUCCESS;
} /* camwire_write_config_to_file() */

/*
  ----------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_debug_print_status(const Camwire_handle c_handle)
{
    User_handle internal_status;
    Extra_features *extra;
    dc1394video_frame_t *capture_frame;
    Camwire_conf *cfg;
    Camwire_state *set;
    
    fprintf(stderr, "\ninternal_status:");
    internal_status = camwire_bus_get_userdata(c_handle);
    if (!internal_status)
    {
	fprintf(stderr, "  (null)\n");
    return CAMERA_SUCCESS;
    }
    fprintf(stderr, "\n"
	    "  camera_connected:      %d\n"
	    "  frame_lock:            %d\n"
	    "  frame_number:          %"PRId64"\n"
	    "  num_dma_buffers:       %d\n"
	    ,
	    internal_status->camera_connected,
	    internal_status->frame_lock,
	    internal_status->frame_number,
	    internal_status->num_dma_buffers);
    fprintf(stderr, "extras:");
    extra = internal_status->extras;
    if (extra)
    {
	fprintf(stderr, "\n"
	    "    single_shot_capable:   %d\n"
	    "    gamma_capable:         %d\n"
	    "    gamma_maxval:          %d\n"
	    "    colour_corr_capable:   %d\n"
	    "    tiling_value:          %d\n"
	    ,
	    extra->single_shot_capable,
	    extra->gamma_capable,
	    extra->gamma_maxval,
	    extra->colour_corr_capable,
	    extra->tiling_value);
    }
    else
	fprintf(stderr, "               (null)\n");
    fprintf(stderr, "features:");
    if (internal_status->feature_set.feature)
    {
        fprintf(stderr, "\n");
	dc1394_feature_print_all(&(internal_status->feature_set), stderr);
    }
    else
	fprintf(stderr, "               (null)\n");
    fprintf(stderr, "frame:");
    capture_frame = internal_status->frame;
    if (capture_frame)
    {
        fprintf(stderr, "\n"
	   "    image:                  %s\n"
	   "    size [width, height]:   %u, %u\n"
	   "    position [hor, ver]:    %u, %u\n"
	   "    color_coding:           %d\n"
	   "    color_filter:           %d\n"
	   "    yuv_byte_order:         %u\n"
	   "    data_depth:             %u\n"
	   "    data_stride:            %u\n"
	   "    video_mode:             %d\n"
	   "    total_bytes:            %" PRIu64 "\n"
	   "    image_bytes:            %u\n"
	   "    padding_bytes:          %u\n"
	   "    packet_size:            %u\n"
	   "    packets_per_frame:      %u\n"
	   "    timestamp:              %" PRIu64 "\n"
	   "    frames_behind:          %u\n"
	   "    camera:                 %s\n"
	   "    id:                     %u\n"
	   "    allocated_image_bytes:  %" PRIu64 "\n"
	   "    little_endian:          %d\n"
	   "    data_in_padding:        %d\n",
	   capture_frame->image   ? "(available)" : "(null)",
	   capture_frame->size[0], capture_frame->size[1],
	   capture_frame->position[0], capture_frame->position[1],
	   capture_frame->color_coding,
	   capture_frame->color_filter,
	   capture_frame->yuv_byte_order,
	   capture_frame->data_depth,
	   capture_frame->stride,
	   capture_frame->video_mode,
	   capture_frame->total_bytes,
	   capture_frame->image_bytes,
	   capture_frame->padding_bytes,
	   capture_frame->packet_size,
	   capture_frame->packets_per_frame,
	   capture_frame->timestamp,
	   capture_frame->frames_behind,
	   capture_frame->camera ? "(available)" : "(null)",
	   capture_frame->id,
	   capture_frame->allocated_image_bytes,
	   capture_frame->little_endian,
	   capture_frame->data_in_padding);
    }
    else
	fprintf(stderr, "               (null)\n");
    fprintf(stderr, "  config_cache:");
    cfg = internal_status->config_cache;
    if (cfg)
    {
	fprintf(stderr, "\n");
	camwire_write_config_to_file(stderr, cfg);
    }
    else
	fprintf(stderr, "               (null)\n");
    fprintf(stderr, "  current_set:");
    set = internal_status->current_set;
    if (set)
    {
	fprintf(stderr, "\n");
	camwire_write_state_to_file(stderr, set);
    }
    else
	fprintf(stderr, "               (null)\n");
    
    return CAMERA_SUCCESS;
} // camwire_debug_print_status()

/*
  ----------------------------------------------------------------------
  See camwire.h for documentation on this function.
*/
int camwire_version(char const **version_str)
{
    int size;
    
    memset((void *)version_string, 0, VERSION_STRING_LENGTH);
    size = snprintf((char *)version_string, VERSION_STRING_LENGTH, "%d.%d.%d",
		    Camwire_VERSION_MAJOR, Camwire_VERSION_MINOR,
		    Camwire_VERSION_PATCH);
    if (size >= VERSION_STRING_LENGTH)
    {
	*version_str = NULL;
	DPRINTF("Internal error: Camwire version string is too long.");
    return CAMERA_FAILURE;
    }
    *version_str = version_string;
    return CAMERA_SUCCESS;
} // camwire_version()

/*
  ----------------------------------------------------------------------
  Returns true if the configuration cache exists and has been
  initialized.  It is assumed that Camwire_handle.User_handle exists and
  is not 0.
*/
inline static int config_cache_exists(const User_handle internal_status)
{

#ifdef CAMERA_DEBUG
    if (!internal_status)
	DPRINTF(
	    "Internal error: User_handle internal_status pointer is 0.");
#endif
    
    /* FIXME: bus_speed will soon disappear from config_cache: */
    return internal_status->config_cache &&
	   internal_status->config_cache->bus_speed != 0;
} /* config_cache_exists() */

/*
  ----------------------------------------------------------------------
  Reads configuration from the given conf file into the given
  configuration structure.  Returns CAMERA_SUCCESS on success or
  CAMERA_FAILURE on failure.
*/
static int read_conf_file(FILE *conffile, Camwire_conf *cfg)
{
    int scan_result;
    int ch;
    char *name;
    int speed, num_bits_set;
    
    scan_result =
	fscanf(conffile,
	       "Camwire IEEE 1394 IIDC DCAM hardware configuration:\n"
	       "  bus_speed:           %d\n"
	       "  format:              %d\n"
	       "  mode:                %d\n"
	       "  max_packets:         %d\n"
	       "  min_pixels:          %d\n"
	       "  trig_setup_time:     %lf\n"
	       "  exposure_quantum:    %lf\n"
	       "  exposure_offset:     %lf\n"
	       "  line_transfer_time:  %lf\n"
	       "  transmit_setup_time: %lf\n"
	       "  transmit_overlap:    %d\n"
	       "  drop_frames:         %d\n"
	       "  dma_device_name:",
	       /* FIXME: bus_speed will soon disappear from config: */
	       &cfg->bus_speed,
	       &cfg->format,
	       &cfg->mode,
	       &cfg->max_packets,
	       &cfg->min_pixels,
	       &cfg->trig_setup_time,
	       &cfg->exposure_quantum,
	       &cfg->exposure_offset,
	       &cfg->line_transfer_time,
	       &cfg->transmit_setup_time,
	       &cfg->transmit_overlap,
	       &cfg->drop_frames);
    if (scan_result == EOF || scan_result != 12)
    {
	DPRINTF("fscanf() failed reading configuration file.");
    return CAMERA_FAILURE;
    }
    
    /* Read the DMA device name: */
    ch = fgetc(conffile);
    while (ch != EOF && ch != '\n' && isspace(ch))  ch = fgetc(conffile);
    name = cfg->dma_device_name;
    while (ch != EOF && !isspace(ch))  
    {
	*name = ch;
	++name;
    if (name - cfg->dma_device_name >= CAMERA_CONF_DMA_DEVICE_MAX_CHARS)
	    break;
	ch = fgetc(conffile);
    }
    *name = '\0';

    /* FIXME: bus_speed will soon disappear from config; no need to check: */
    /* Ensure that bus_speed is one of 100, 200, 400...: */
    num_bits_set = 0;
    speed = cfg->bus_speed/100;
    while (speed != 0)
    {
	if ((speed & 1) != 0)  ++num_bits_set;
	speed >>= 1;
    }
    
    if (cfg->bus_speed%100 != 0 || num_bits_set != 1)
    {
	DPRINTF("Invalid bus_speed in configuration file read.");
    return CAMERA_FAILURE;
    }

    return CAMERA_SUCCESS;
} /* read_conf_file() */

/*
  ----------------------------------------------------------------------
  Attempts to open a configuration file for reading.  Returns the stream
  pointer on success or 0 on failure.
*/
static FILE * find_conf_file(const Camwire_id *id)
{
    FILE *conffile;
    char *env_directory;

    conffile = open_named_conf_file(0, id->chip);
    if (conffile)  return conffile;

    conffile = open_named_conf_file(0, id->model);
    if (conffile)  return conffile;

    conffile = open_named_conf_file(0, id->vendor);
    if (conffile)  return conffile;

    env_directory = getenv(ENVIRONMENT_VAR_CONF);
    if (env_directory)
    {
	conffile = open_named_conf_file(env_directory, id->chip);
	if (conffile)  return conffile;

	conffile = open_named_conf_file(env_directory, id->model);
	if (conffile)  return conffile;

	conffile = open_named_conf_file(env_directory, id->vendor);
	if (conffile)  return conffile;
    }
    return 0;
} /* find_conf_file() */

/*
  ----------------------------------------------------------------------
  Attempts to open the named configuration file for reading after
  appending the configuration filename extension.  Returns the stream
  pointer on success or 0 on failure.
*/
static FILE * open_named_conf_file(const char *path,
				     const char *filename)
{
    char conffilename[CONFFILE_NAME_MAX_CHARS+1];

    conffilename[0] = '\0';
    if (path)
    {
	strncat(conffilename, path, CONFFILE_PATH_MAX_CHARS);
	if (conffilename[strlen(conffilename)-1] != '/')
	{
	    strncat(conffilename, "/", 1);
	}
    }
    strncat(conffilename, filename, CAMERA_ID_MAX_CHARS);
    strncat(conffilename, CONFFILE_EXTENSION,
	    CONFFILE_EXTENSION_MAX_CHARS);
    return fopen(conffilename, "r");
} /* open_named_conf_file() */

/*
  ----------------------------------------------------------------------
  Queries the camera and attempts to create a sensible default
  configuration.  Returns CAMERA_SUCCESS on success or CAMERA_FAILURE
  on failure.  */
/* The IEEE 1394 IIDC digital camera (DCAM) specification gives camera
   manufacturers a choice of several predefined resolutions called
   "formats":

   Format 0 is for 160x120 up to 640x480 (VGA).
   Format 1 is for 800x600 up to 1024x768 (SVGA).
   Format 2 is for 1280x960 up to 1600x1200 (XVGA).
   Format 6 is "memory channels" or still images.
   Format 7 is for scalable image sizes. */

static int generate_default_config(const Camwire_handle c_handle,
				   Camwire_conf *cfg)
{
    int dc1394_return;
    dc1394video_mode_t video_mode;
    dc1394video_modes_t mode_list;

    /* Initialize the camera to factory settings: */
    ERROR_IF_NULL(c_handle);

    /* dc1394_camera_reset() does not work on all cameras, so we are
       lenient on the test: */
    dc1394_return = dc1394_camera_reset(camwire_handle_get_camera(c_handle));
    if (dc1394_return != DC1394_SUCCESS)
    {
	DPRINTF("Warning: dc1394_camera_reset() failed.  Continuing configuration, "
		"but camera may not be properly initialized.");
	sleep(1);  /* Increase chances that camera may recover.*/
    }

    /* video_mode = get_1394_video_mode(c_handle); */
  
    /* Determine the highest supported format and mode: */
    ERROR_IF_DC1394_FAIL(
	dc1394_video_get_supported_modes(camwire_handle_get_camera(c_handle),
					 &mode_list));
    if (mode_list.num == 0)
    {
	DPRINTF("dc1394_video_get_supported_modes() returned an empty list.");
    return CAMERA_FAILURE;
    }
    video_mode = mode_list.modes[mode_list.num-1];  /* Highest format and mode. */
    convert_dc1394video_mode2format_mode(video_mode, &cfg->format, &cfg->mode);

    /* Some default values (may be overwritten below): */
    cfg->max_packets = 4095;
    cfg->min_pixels = 4096;
    cfg->trig_setup_time = 0.0;
    cfg->exposure_quantum = 20e-6;
    cfg->exposure_offset = 0.0;
    cfg->line_transfer_time = 0.0;
    cfg->transmit_setup_time = 0.0;
    
    /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
    cfg->bus_speed = 400;
    cfg->transmit_overlap = 0;
    cfg->drop_frames = 0;
    cfg->dma_device_name[0] = '\0'; 	/* Use default.*/

    /* FIXME: Some of the defaults, eg. bus_speed, above could be read
       from the camera.*/
    
    return CAMERA_SUCCESS;
} /* generate_default_config() */

/*
  -----------------------------------------------------------------------------
  Returns the bus frequency (cycles per second) corresponding to the
  given bus speed (megabits per second).
*/
static double convert_busspeed2busfreq(const int bus_speed)
{
    return (double)(20*bus_speed);
} /* convert_busspeed2busfreq() */

/*
  -----------------------------------------------------------------------------
  Returns the libdc1394 data speed enumeration (SPEED_100, SPEED_200,
  etc.) corresponding to the given bus speed (megabits per second).
*/
static int convert_busspeed2dc1394(const int bus_speed)
{
    switch (bus_speed)
    {
	case 100:
	    return DC1394_ISO_SPEED_100;
	case 200:
	    return DC1394_ISO_SPEED_200;
	case 400:
	    return DC1394_ISO_SPEED_400;
	case 800:
	    return DC1394_ISO_SPEED_800;
	case 1600:
	    return DC1394_ISO_SPEED_1600;
	case 3200:
	    return DC1394_ISO_SPEED_3200;
	default:
	    DPRINTF("Bus speed is not a legal value.");
	    return DC1394_ISO_SPEED_400;
    }
} /* convert_busspeed2dc1394() */

/*
  -----------------------------------------------------------------------------
  Returns the frame rate corresponding to the given number of packets
  per frame.
*/
static double convert_numpackets2framerate(const Camwire_handle c_handle,
					   const uint32_t num_packets)
{
    Camwire_conf config;
    uint32_t actual;
    
    if (camwire_get_config(c_handle, &config) != CAMERA_SUCCESS)
    {
	DPRINTF("camwire_get_config() failed.");
    }

#ifdef CAMERA_DEBUG
    /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
    if (config.bus_speed == 0)
    {
	DPRINTF("camwire_get_config() returned null format.");
    }
    else if (config.format != 7)
    {
	DPRINTF("Camera is not in Format 7.");
    }
#endif
    
    actual = num_packets;
    if (actual < 1)  actual = 1;
    if (actual > config.max_packets)  actual = config.max_packets;
    /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
    return convert_busspeed2busfreq(config.bus_speed)/actual;
} /* convert_numpackets2framerate() */

/*
  ----------------------------------------------------------------------
  Queries the camera for supported features and attempts to create
  sensible default settings.  Note that the camera itself is initialized
  to factory settings in the process.  Returns CAMERA_SUCCESS on
  success or CAMERA_FAILURE on failure.
*/
static int generate_default_settings(const Camwire_handle c_handle,
				     Camwire_state *set)
{
    int dc1394_return;
    dc1394video_mode_t video_mode;
    uint32_t num_packets;
    dc1394color_coding_t color_id;
    dc1394framerates_t supported_fr;
    double fr;
    int f;
    dc1394feature_info_t capability;
    Camwire_conf config;
    double max_shutter, min_shutter;
    dc1394bool_t on_off; 
    int32_t coef_reg[9];
    
    /* Initialize the camera to factory settings: */
    ERROR_IF_NULL(c_handle);
    
    /* dc1394_camera_reset() does not work on all cameras, so we are
       lenient on the test: */
    dc1394_return = dc1394_camera_reset(camwire_handle_get_camera(c_handle));
    if (dc1394_return != DC1394_SUCCESS)
    {
	DPRINTF("Warning: dc1394_camera_reset() failed.  Continuing configuration, "
		"but camera may not be properly initialized.");
	sleep(1);  /* Increase chances that camera may recover.*/
    }

    video_mode = get_1394_video_mode(c_handle);

    /* Format and mode-specific frame dimensions and pixel coding: */
    if (fixed_image_size(video_mode))  /* Formats 0-2.*/
    {
	set->left = 0; 			/* Pixels.*/
	set->top = 0; 			/* Pixels.*/
	ERROR_IF_DC1394_FAIL(
	    dc1394_get_image_size_from_video_mode(camwire_handle_get_camera(c_handle),
						  video_mode,
						  (uint32_t *)&set->width,
						  (uint32_t *)&set->height));
	set->coding = convert_videomode2pixelcoding(video_mode);
    }
    else if (variable_image_size(video_mode))  /* Format 7.*/
    {
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_image_position(camwire_handle_get_camera(c_handle),
						video_mode,
						(uint32_t *)&set->left,
						(uint32_t *)&set->top));
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_image_size(camwire_handle_get_camera(c_handle),
					    video_mode,
					    (uint32_t *)&set->width,
					    (uint32_t *)&set->height));
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_color_coding(
		camwire_handle_get_camera(c_handle),
		video_mode,
		&color_id));
	set->coding = convert_colorid2pixelcoding(color_id);
    }
    else
    {
	DPRINTF("Camera's format is not supported.");
    return CAMERA_FAILURE;
    }
    
    /* Determine the maximum supported framerate in this mode and
       format: */
    if (fixed_image_size(video_mode))
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_get_supported_framerates(camwire_handle_get_camera(c_handle),
						  video_mode,
						  &supported_fr));
	if (supported_fr.num == 0)
	{
	    DPRINTF("dc1394_video_get_supported_framerates() returned no "
		    "framerates.");
        return CAMERA_FAILURE;
	}
	set->frame_rate = 0;
	for (f=0; f<supported_fr.num; ++f)
	{
	    fr = convert_index2framerate(supported_fr.framerates[f]);
	    if (fr < 0.0)
	    {
		DPRINTF("convert_index2framerate() failed.");
        return CAMERA_FAILURE; 	/* Invalid index.*/
	    }
	    if (fr > set->frame_rate)  set->frame_rate = fr;
	}
    }
    else if (variable_image_size(video_mode))  /* Format 7.*/
    {
	/* PACKET_PER_FRAME_INQ depends on BYTE_PER_PACKET which in turn
	   depends on IMAGE_SIZE and COLOR_CODING_ID.  Since we are not
	   changing these registers, it is safe to use the value
	   returned by get_num_packets(): */
    ERROR_IF_CAMERA_FAIL(
	    get_numpackets(c_handle, &num_packets));
	set->frame_rate = convert_numpackets2framerate(c_handle, num_packets);
    }
    else
    {
	DPRINTF("Camera's format is not supported.");
    return CAMERA_FAILURE;
    }

    /* Get the shutter speed and try to fit it into one frame period: */
    set->shutter = 0.5/set->frame_rate; /* Seconds, default.*/
    capability.id = DC1394_FEATURE_SHUTTER;
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_get(camwire_handle_get_camera(c_handle),
			   &capability));
    if (feature_is_usable(&capability))
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_config(c_handle, &config));
	set->shutter =
	    config.exposure_offset + capability.value*config.exposure_quantum;
	max_shutter = config.exposure_quantum *
	    ((unsigned long int)(1.0/(set->frame_rate*config.exposure_quantum)));
	if (set->shutter > max_shutter)  set->shutter = max_shutter;
	min_shutter =
	    config.exposure_offset + capability.min*config.exposure_quantum;
	if (set->shutter < min_shutter)  set->shutter = min_shutter;
    }
    else
    {
	DPRINTF("Camera reported no usable shutter.");
    }
    
    /* Format and mode-independent settings: */
    set->external_trigger = 0; 		/* Flag.*/
    set->trigger_polarity = 1; 		/* Flag, default.*/
    capability.id = DC1394_FEATURE_TRIGGER;
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_get(camwire_handle_get_camera(c_handle),
			   &capability));
    if (feature_is_usable(&capability))
    {
	if (capability.trigger_polarity == DC1394_TRIGGER_ACTIVE_LOW)
	    set->trigger_polarity = 0;
	else
	    set->trigger_polarity = 1;
    }

    /* Get the factory gain and set our normalized gain accordingly: */
    set->gain = 0.0;			/* Units, default.*/
    capability.id = DC1394_FEATURE_GAIN;
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_get(camwire_handle_get_camera(c_handle),
			   &capability));
    if (feature_is_usable(&capability))
    {
	if (capability.max != capability.min)
	    set->gain = (double)(capability.value - capability.min) /
		(capability.max - capability.min);
    }
    else
    {
	DPRINTF("Camera reported no usable gain.");
    }

    /* Get the factory brightness and set our normalized brightness
       accordingly: */
    set->brightness = 0.0;		/* Units, default.*/
    capability.id = DC1394_FEATURE_BRIGHTNESS;
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_get(camwire_handle_get_camera(c_handle),
			   &capability));
    if (feature_is_usable(&capability))
    {
	set->brightness = 2.0*(double)(capability.value - capability.min) /
		    (capability.max - capability.min) - 1.0;
    }
    else
    {
	DPRINTF("Camera reported no usable brightness.");
    }

    /* Get the factory white balance and set our normalized levels
       accordingly: */
    set->white_balance[0] = set->white_balance[1] = 0.0; 	/* Units, default.*/
    capability.id = DC1394_FEATURE_WHITE_BALANCE;
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_get(camwire_handle_get_camera(c_handle),
			   &capability));
    if (feature_is_usable(&capability))
    {
	if (capability.max != capability.min)
	{
	    set->white_balance[0] = (double)(capability.BU_value - capability.min) /
		(capability.max - capability.min);
	    set->white_balance[1] = (double)(capability.RV_value - capability.min) /
		(capability.max - capability.min);
	}
    }
    else
    {
	DPRINTF("Camera reported no usable white balance.");
    }

    /* Enable colour correction by default if the camera supports it,
       and get the factory colour correction coefficients: */
    set->colour_corr = probe_camera_colour_correction(c_handle);
    if (set->colour_corr) 		/* Flag, on by default.*/
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_avt_get_color_corr(camwire_handle_get_camera(c_handle),
				      &on_off,
				      &coef_reg[0], &coef_reg[1], &coef_reg[2],
				      &coef_reg[3], &coef_reg[4], &coef_reg[5],
				      &coef_reg[6], &coef_reg[7], &coef_reg[8]));
	convert_avtvalues2colourcoefs(coef_reg, set->colour_coef);	
    }
    else
    {
	DPRINTF("Camera reported no usable colour correction.");
    }

    /* Enable gamma if the camera supports it: */
    set->gamma = probe_camera_gamma(c_handle);  /* Flag.*/
    if (!set->gamma)
    {
	DPRINTF("Camera reported no usable gamma correction.");
    }
    
    /* Other defaults: */
    set->tiling = probe_camera_tiling(c_handle);  /* Pattern.*/
    set->num_frame_buffers = 10; 	   /* Frames.*/
    set->single_shot = 0; 		   /* Flag.*/
    set->running = 0; 			   /* Flag.*/
    set->shadow = 1; 			   /* Flag.*/

    return CAMERA_SUCCESS;
} /* generate_default_settings() */

/*
  -----------------------------------------------------------------------------
  Returns the number of bits per component in the given pixel coding.
*/
static int component_depth(const Camwire_pixel coding)
{
    switch (coding)
    {
    case CAMERA_PIXEL_MONO8:
    case CAMERA_PIXEL_YUV411:
    case CAMERA_PIXEL_YUV422:
    case CAMERA_PIXEL_YUV444:
    case CAMERA_PIXEL_RGB8:
    case CAMERA_PIXEL_RAW8:
	    return 8;
    case CAMERA_PIXEL_MONO16:
    case CAMERA_PIXEL_RGB16:
    case CAMERA_PIXEL_MONO16S:
    case CAMERA_PIXEL_RGB16S:
    case CAMERA_PIXEL_RAW16:
	    return 16;
	default:
	    return 0;
    }
} /* component_depth() */

/*
  -----------------------------------------------------------------------------
  Returns the pixel coding given the libdc1394 mode in Formats 0, 1 and 2.
*/
static Camwire_pixel convert_videomode2pixelcoding(const dc1394video_mode_t video_mode)
{
    switch (video_mode)
    {
	case DC1394_VIDEO_MODE_160x120_YUV444:
        return CAMERA_PIXEL_YUV444;  /* 24 bits/pixel.*/
	    break;
	case DC1394_VIDEO_MODE_320x240_YUV422:
	case DC1394_VIDEO_MODE_640x480_YUV422:
	case DC1394_VIDEO_MODE_800x600_YUV422:
	case DC1394_VIDEO_MODE_1024x768_YUV422:
	case DC1394_VIDEO_MODE_1280x960_YUV422:
	case DC1394_VIDEO_MODE_1600x1200_YUV422:
        return CAMERA_PIXEL_YUV422;  /* 16 bits/pixel.*/
	    break;
	case DC1394_VIDEO_MODE_640x480_YUV411:
        return CAMERA_PIXEL_YUV411;  /* 12 bits/pixel.*/
	    break;
	case DC1394_VIDEO_MODE_640x480_RGB8:
	case DC1394_VIDEO_MODE_800x600_RGB8:
	case DC1394_VIDEO_MODE_1024x768_RGB8:
	case DC1394_VIDEO_MODE_1280x960_RGB8:
	case DC1394_VIDEO_MODE_1600x1200_RGB8:
        return CAMERA_PIXEL_RGB8;  /* 24 bits/pixel.*/
	    break;
	case DC1394_VIDEO_MODE_640x480_MONO8:
	case DC1394_VIDEO_MODE_800x600_MONO8:
	case DC1394_VIDEO_MODE_1024x768_MONO8:
	case DC1394_VIDEO_MODE_1280x960_MONO8:
	case DC1394_VIDEO_MODE_1600x1200_MONO8:
        return CAMERA_PIXEL_MONO8;  /* 8 bits/pixel.*/
	    break;
	case DC1394_VIDEO_MODE_640x480_MONO16:
	case DC1394_VIDEO_MODE_800x600_MONO16:
	case DC1394_VIDEO_MODE_1024x768_MONO16:
	case DC1394_VIDEO_MODE_1280x960_MONO16:
	case DC1394_VIDEO_MODE_1600x1200_MONO16:
        return CAMERA_PIXEL_MONO16;  /* 16 bits/pixel.*/
	    break;
	default:
        return CAMERA_PIXEL_INVALID;  /* Unknown.*/
	    break;
    }
} /* convert_videomode2pixelcoding() */

/*
  -----------------------------------------------------------------------------
   Returns the pixel coding given the libdc1394 colour coding ID in
   Format 7.
*/
static Camwire_pixel convert_colorid2pixelcoding(const dc1394color_coding_t color_id)
{
    switch (color_id)
    {
	case DC1394_COLOR_CODING_MONO8:
        return CAMERA_PIXEL_MONO8;  /* 8 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_YUV411:
        return CAMERA_PIXEL_YUV411;  /* 12 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_YUV422:
        return CAMERA_PIXEL_YUV422;  /* 16 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_YUV444:
        return CAMERA_PIXEL_YUV444;  /* 24 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_RGB8:
        return CAMERA_PIXEL_RGB8;  /* 24 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_MONO16:
        return CAMERA_PIXEL_MONO16;  /* 16 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_RGB16:
        return CAMERA_PIXEL_RGB16;  /* 48 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_MONO16S:
        return CAMERA_PIXEL_MONO16S;  /* 16 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_RGB16S:
        return CAMERA_PIXEL_RGB16S;  /* 48 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_RAW8:
        return CAMERA_PIXEL_RAW8;  /* 8 bits/pixel.*/
	    break;
	case DC1394_COLOR_CODING_RAW16:
        return CAMERA_PIXEL_RAW16;  /* 16 bits/pixel.*/
	    break;
	default:
        return CAMERA_PIXEL_INVALID;  /* Not supported.*/
	    break;
    }
} /* convert_colorid2pixelcoding() */

/*
  -----------------------------------------------------------------------------
   Returns the pixel tiling given the libdc1394 colour coding ID in
   Format 7.
*/
static Camwire_tiling convert_filterid2pixeltiling(const dc1394color_filter_t filter_id)
{
    switch (filter_id)
    {
	case DC1394_COLOR_FILTER_RGGB:
        return CAMERA_TILING_RGGB;
	    break;
	case DC1394_COLOR_FILTER_GBRG:
        return CAMERA_TILING_GBRG;
	    break;
	case DC1394_COLOR_FILTER_GRBG:
        return CAMERA_TILING_GRBG;
	    break;
	case DC1394_COLOR_FILTER_BGGR:
        return CAMERA_TILING_BGGR;
	    break;
	default:
        return CAMERA_TILING_INVALID;  /* Not supported.*/
	    break;
    }
} /* convert_filterid2pixeltiling() */

/*
  -----------------------------------------------------------------------------
  Does the actual work of camwire_create() and
  camwire_create_from_struct(), after they have initialized the camera
  to factory settings and sorted out where the start-up settings come
  from.
*/

static int create(const Camwire_handle c_handle, const Camwire_state *set)
{
    User_handle internal_status;
    Camwire_conf config;

    /* Allocate zero-filled space for internal status, and register a
       pointer to it in the camera handle: */
    internal_status = (Camwire_user_data *)calloc(1, sizeof(Camwire_user_data));
    /* Camwire_user_data is defined above.*/
    /* Note that internal_status is zero-filled by calloc().  Other
       functions may use this fact (and the existence of a pointer to
       it) to check if it has been initialized or not. */
    ERROR_IF_NULL(internal_status); 	/* Allocation failure.*/
    if (!camwire_bus_set_userdata(c_handle, internal_status))
    {   /* Already in use.*/
	DPRINTF("camwire_bus_set_userdata() failed.");
	free(internal_status);
    return CAMERA_FAILURE;
    }

    /* Allocate zero-filled space for the extra features: */
    internal_status->extras =
	(Extra_features *) calloc(1, sizeof(Extra_features));
    if (!internal_status->extras)
    { 	/* Allocation failure.*/
	DPRINTF("calloc(Extra_features) failed.");
	free_internals(c_handle);
    return CAMERA_FAILURE;
    }

    /* Allocate zero-filled space for the config cache: */
    internal_status->config_cache =
	(Camwire_conf *) calloc(1, sizeof(Camwire_conf));
    if (!internal_status->config_cache)
    { 	/* Allocation failure.*/
	DPRINTF("calloc(Camwire_conf) failed.");
	free_internals(c_handle);
    return CAMERA_FAILURE;
    }

    /* Allocate space and make a copy of the initial settings: */
    internal_status->current_set =
	(Camwire_state *) malloc(sizeof(Camwire_state));
    if (!internal_status->current_set)
    { 	/* Allocation failure.*/
	DPRINTF("malloc(Camwire_state) failed.");
	free_internals(c_handle);
    return CAMERA_FAILURE;
    }
    memcpy(internal_status->current_set, set, sizeof(Camwire_state));

    /* Get 1394-specific hardware configuration: */
    if (camwire_get_config(c_handle, &config) != CAMERA_SUCCESS)
    {
	DPRINTF("camwire_get_config() failed.");
	free_internals(c_handle);
    return CAMERA_FAILURE;
    }

    /* Make sure the video1394 device is not already listening on this
       channel (due to a provious process being killed without resetting
       the camera).  Hopefully this won't hurt anything: */
    dc1394_capture_stop(camwire_handle_get_camera(c_handle));
    dc1394_iso_release_all(camwire_handle_get_camera(c_handle));

    /* Connect the camera to the bus and initialize it with our
       settings: */
    if (connect_cam(c_handle, &config, set) != CAMERA_SUCCESS)
    {
	DPRINTF("connect_cam() failed.");
	free_internals(c_handle);
    return CAMERA_FAILURE;
    }

    return CAMERA_SUCCESS;
} /* create() */

/*
  -----------------------------------------------------------------------------
  Connects the camera to the bus and sets it to the given configuration
  and initial settings.  Returns CAMERA_SUCCESS on success or
  CAMERA_FAILURE on failure.  The function disconnect_cam() must be
  called when done to free the allocated memory.
*/
static int connect_cam(const Camwire_handle c_handle, Camwire_conf *cfg,
		       const Camwire_state *set)
{
    User_handle internal_status;
    dc1394video_mode_t video_mode;
    dc1394framerate_t frame_rate_index;
    uint32_t num_packets, packet_size;
    /* char *dma_device_file; */
    int depth;
    dc1394framerates_t framerate_list;
    dc1394color_coding_t color_id;
    double actual_frame_rate;
    dc1394color_codings_t coding_list;
    Camwire_pixel actual_coding;
    Camwire_state *shadow_state;
    dc1394camera_t *dc1394_camera;
    
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);

    /*
    if (cfg->dma_device_name[0] == '\0')
    {
	dma_device_file = 0;
    }
    else
    {
	dma_device_file = cfg->dma_device_name;
    }
    */
    
    /* If dc1394_capture_stop() is called without a preceding
       successful call to dc1394_capture_setup(), libdc1394 used to get
       into a tangled state.  That is why we keep track with the
       camera_connected flag, and check it in disconnect_cam(): */
    internal_status->camera_connected = 0;
    
    /* Set 1394B mode if it is available.  Don't check the return
       status, because older cameras won't support it: */
    dc1394_video_set_operation_mode(camwire_handle_get_camera(c_handle),
				    DC1394_OPERATION_MODE_1394B);
    
    video_mode = convert_format_mode2dc1394video_mode(cfg->format, cfg->mode);
    ERROR_IF_ZERO(video_mode);
    if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_get_supported_framerates(
		camwire_handle_get_camera(c_handle),
		video_mode,
		&framerate_list));
	if (framerate_list.num == 0)
	{
	    DPRINTF("dc1394_video_get_supported_framerates() returned an empty list.");
        return CAMERA_FAILURE;
	}
	frame_rate_index = convert_framerate2index(set->frame_rate, &framerate_list);
	ERROR_IF_ZERO(frame_rate_index);
    
	/* FIXME: Use dc1394_video_get_iso_speed() for bus_speed, unless there is an entry in the .conf file: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_set_iso_speed(camwire_handle_get_camera(c_handle),
				       convert_busspeed2dc1394(cfg->bus_speed)));
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_set_mode(camwire_handle_get_camera(c_handle),
				  video_mode));
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_set_framerate(camwire_handle_get_camera(c_handle),
				       frame_rate_index));
	
	ERROR_IF_DC1394_FAIL(
            dc1394_capture_setup(camwire_handle_get_camera(c_handle), 
				 set->num_frame_buffers, 
				 DC1394_CAPTURE_FLAGS_DEFAULT));
	internal_status->num_dma_buffers = set->num_frame_buffers;
	actual_coding = convert_videomode2pixelcoding(video_mode);
	actual_frame_rate = convert_index2framerate(frame_rate_index);
    }
    else if (variable_image_size(video_mode))  /* Format 7.*/
    {
	/* Prevent a segfault due to kalloc() bug in dma.c of the
	   linux1394 system.  This ought to be removed for later
	   versions: */
    ERROR_IF_CAMERA_FAIL(
	    camwire_pixel_depth(set->coding, &depth));

	/* FIXME: Use dc1394_video_get_iso_speed() for bus_speed, unless there is an entry in the .conf file: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_set_iso_speed(camwire_handle_get_camera(c_handle),
				       convert_busspeed2dc1394(cfg->bus_speed)));
	ERROR_IF_DC1394_FAIL(
	    dc1394_video_set_mode(camwire_handle_get_camera(c_handle),
				  video_mode));

	/* Set up the color_coding_id before calling
	   dc1394_capture_setup(), otherwise the wrong DMA buffer size
	   may be allocated: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_format7_get_color_codings(camwire_handle_get_camera(c_handle),
					     video_mode,
					     &coding_list));
	if (coding_list.num == 0)
	{
	    DPRINTF("dc1394_format7_get_color_codings() returned an empty list.");
        return CAMERA_FAILURE;
	}
	color_id = convert_pixelcoding2colorid(set->coding, &coding_list);
	if (color_id == 0)
	{
	    DPRINTF("Pixel colour coding is invalid or not supported  by the "
		    "camera.");
        return CAMERA_FAILURE;
	}
	ERROR_IF_DC1394_FAIL(
	    dc1394_format7_set_color_coding(
		camwire_handle_get_camera(c_handle),
		video_mode,
		color_id));
	actual_coding = convert_colorid2pixelcoding(color_id);

	/* Calculate the packet size from the wanted frame rate.  But
	   first set the image size because that (together with
	   color_id) can affect the max_bytes read by
	  dc1394_format7_get_packet_parameters() (or total_bytes read by
	  dc1394_format7_get_total_bytes()) in
	   convert_numpackets2packetsize(): */
	ERROR_IF_DC1394_FAIL(
	    dc1394_format7_set_image_position(
		camwire_handle_get_camera(c_handle),
		video_mode,
		set->left, set->top));  /* So that _image_size() doesn't fail.*/
	ERROR_IF_DC1394_FAIL(
	    dc1394_format7_set_image_size(
		camwire_handle_get_camera(c_handle),
		video_mode,
		set->width, set->height));  /* PACKET_PARA_INQ is now valid. */
	num_packets =
	    convert_framerate2numpackets(c_handle, set->frame_rate);
	ERROR_IF_ZERO(num_packets);
	packet_size = convert_numpackets2packetsize(c_handle,
						    num_packets,
						    set->width,
						    set->height,
						    actual_coding);
	ERROR_IF_ZERO(packet_size);

	/* Set up the camera and DMA buffers: */
	ERROR_IF_DC1394_FAIL(
	    dc1394_format7_set_packet_size(
		camwire_handle_get_camera(c_handle),
		video_mode,
		packet_size));
	ERROR_IF_DC1394_FAIL(
	    dc1394_capture_setup(camwire_handle_get_camera(c_handle),
				 set->num_frame_buffers,
				 DC1394_CAPTURE_FLAGS_DEFAULT));
	num_packets = convert_packetsize2numpackets(c_handle,
						    packet_size,
						    set->width,
						    set->height,
						    actual_coding);
	ERROR_IF_ZERO(num_packets);
	actual_frame_rate =
	    convert_numpackets2framerate(c_handle, num_packets);
    }
    else
    {
	DPRINTF("Unsupported camera format.");
    return CAMERA_FAILURE;
    }
    internal_status->camera_connected = 1;
    internal_status->frame = 0;
    internal_status->frame_lock = 0;
    internal_status->num_dma_buffers = set->num_frame_buffers;

    /* Find out camera capabilities (which should only be done after
       setting up the format and mode above): */
    dc1394_camera = camwire_handle_get_camera(c_handle);
    internal_status->extras->single_shot_capable =
	(dc1394_camera->one_shot_capable!=DC1394_FALSE ? 1 : 0);
    internal_status->extras->gamma_capable = probe_camera_gamma(c_handle);
    internal_status->extras->colour_corr_capable =
	probe_camera_colour_correction(c_handle);
    internal_status->extras->tiling_value = probe_camera_tiling(c_handle);
    ERROR_IF_DC1394_FAIL(
	dc1394_feature_get_all(camwire_handle_get_camera(c_handle),
			       &internal_status->feature_set));
    
    /* Update DMA-affected shadow states not done in
       set_non_dma_registers() calls below: */
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    shadow_state->num_frame_buffers = set->num_frame_buffers;
    shadow_state->left = set->left;
    shadow_state->top = set->top;
    shadow_state->width = set->width;
    shadow_state->height = set->height;
    shadow_state->coding = actual_coding;
    shadow_state->frame_rate = actual_frame_rate;

    /* Initialize camera registers not already done by
       dc1394_video_set_framerate() or dc1394_format7_set_roi() and
       update shadow state of these: */
    ERROR_IF_CAMERA_FAIL(
	set_non_dma_registers(c_handle, set));
    return CAMERA_SUCCESS;
} /* connect_cam() */

/*
  -----------------------------------------------------------------------------
  Initialize camera registers not already done by
  dc1394_video_set_framerate() or dc1394_format7_set_roi() and update
  their shadow state.  Note that the order of register writes may be
  significant for some cameras after power-up or reset/initilize.  */
static int set_non_dma_registers(const Camwire_handle c_handle,
				 const Camwire_state *set)
{
    User_handle internal_status;
    Camwire_state *shadow_state;
    dc1394feature_info_t *cap;
    
    internal_status = camwire_bus_get_userdata(c_handle);
    ERROR_IF_NULL(internal_status);
    shadow_state = internal_status->current_set;
    ERROR_IF_NULL(shadow_state);

    /* Update the state shadow flag: */
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_stateshadow(c_handle, set->shadow));

    /* Set up default shadow states for features which may not be
       supported by the camera.  These may be overwritten below.  The
       rest of the shadow states will be updated within each
       camwire_set_...() call: */
    shadow_state->trigger_polarity = set->trigger_polarity;
    shadow_state->external_trigger = set->external_trigger;
    shadow_state->shutter = set->shutter;
    shadow_state->gain = set->gain;
    shadow_state->brightness = set->brightness;
    shadow_state->white_balance[0] = set->white_balance[0];
    shadow_state->white_balance[1] = set->white_balance[1];

    /* Trigger source and polarity: */
    cap = get_feature_capability(c_handle, DC1394_FEATURE_TRIGGER);
    ERROR_IF_NULL(cap);
    if (feature_is_usable(cap))
    {
	/* Trigger never has auto capability, and its on-off setting is
	   done by camwire_set_trigger_source() below.*/
	ERROR_IF_DC1394_FAIL(
	    dc1394_external_trigger_set_mode(camwire_handle_get_camera(c_handle),
					     DC1394_TRIGGER_MODE_0)); 	/* Edge.*/
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_trigger_source(c_handle, set->external_trigger));
	if (cap->polarity_capable == DC1394_TRUE)
        ERROR_IF_CAMERA_FAIL(
		camwire_set_trigger_polarity(c_handle, set->trigger_polarity));
    }

    /* Shutter: */
    cap = get_feature_capability(c_handle, DC1394_FEATURE_SHUTTER);
    ERROR_IF_NULL(cap);
    if (feature_is_usable(cap))
    {
    ERROR_IF_CAMERA_FAIL(
	    feature_switch_on(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    feature_go_manual(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_shutter(c_handle, set->shutter));
    }
    
    /* Gain: */
    cap = get_feature_capability(c_handle, DC1394_FEATURE_GAIN);
    ERROR_IF_NULL(cap);
    if (feature_is_usable(cap))
    {
    ERROR_IF_CAMERA_FAIL(
	    feature_switch_on(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    feature_go_manual(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_gain(c_handle, set->gain));
    }
    
    /* Brightness: */
    cap = get_feature_capability(c_handle, DC1394_FEATURE_BRIGHTNESS);
    ERROR_IF_NULL(cap);
    if (feature_is_usable(cap))
    {
    ERROR_IF_CAMERA_FAIL(
	    feature_switch_on(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    feature_go_manual(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_brightness(c_handle, set->brightness));
    }
    
    /* White balance: */
    cap = get_feature_capability(c_handle, DC1394_FEATURE_WHITE_BALANCE);
    ERROR_IF_NULL(cap);
    if (feature_is_usable(cap))
    {
    ERROR_IF_CAMERA_FAIL(
	    feature_switch_on(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    feature_go_manual(c_handle, cap));
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_white_balance(c_handle, set->white_balance));
    }
    
    /* Pixel tiling: */
    shadow_state->tiling = internal_status->extras->tiling_value;
    /* Tiling cannot be set.  Ignore set->tiling.*/

    /* Colour correction and coefficients: */
    if (internal_status->extras->colour_corr_capable)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_colour_correction(c_handle, set->colour_corr));
    }
    else
    {
	shadow_state->colour_corr = 0;
    }
    if (internal_status->extras->colour_corr_capable)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_colour_coefficients(c_handle, set->colour_coef));
    }
    else
    {
	shadow_state->colour_coef[0] = 1;
	shadow_state->colour_coef[1] = 0;
	shadow_state->colour_coef[2] = 0;
	shadow_state->colour_coef[3] = 0;
	shadow_state->colour_coef[4] = 1;
	shadow_state->colour_coef[5] = 0;
	shadow_state->colour_coef[6] = 0;
	shadow_state->colour_coef[7] = 0;
	shadow_state->colour_coef[8] = 1;
    }

    /* Gamma: */
    if (internal_status->extras->gamma_capable)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_gamma(c_handle, set->gamma));
    }
    else
    {
	shadow_state->gamma = 0;  /* Ignore set->gamma.*/
    }

    /* Single-shot: */
    if (internal_status->extras->single_shot_capable)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_single_shot(c_handle, set->single_shot));
    }
    else
    {
	shadow_state->single_shot = 0;  /* Ignore set->single_shot.*/
    }

    /* Run-stop: */
    ERROR_IF_CAMERA_FAIL(
	camwire_set_run_stop(c_handle, set->running));

    /* The list of settings updated above does not include
       camwire_set_frame_size(), camwire_set_pixel_coding(), or
       camwire_set_framerate() because these are already set in
       dc1394_video_set_framerate() or dc1394_format7_set_roi() and
       because they could cause infinite recursion since they themselves
       contain calls to (re)connect_cam() which call this function.
       camwire_set_frame_offset() is a bit different in that it is set
       up with dc1394_format7_set_roi() but does not require a
       reconnect_cam() when it changes. */

    return CAMERA_SUCCESS;
} /* set_non_dma_registers() */

/*
  -----------------------------------------------------------------------------
  Disconnects the camera from the bus and frees memory allocated in
  connect_cam().  The camera should be stopped before calling this
  function.
*/
static void disconnect_cam(const Camwire_handle c_handle)
{
    User_handle internal_status;
    
    internal_status = camwire_bus_get_userdata(c_handle);
    if (internal_status)
    {
	if (internal_status->camera_connected)
	{
	    if (internal_status->frame_lock)
	    {
		dc1394_capture_enqueue(camwire_handle_get_camera(c_handle),
				       internal_status->frame);
		internal_status->frame = 0;
		internal_status->frame_lock = 0;
	    }
	    dc1394_capture_stop(camwire_handle_get_camera(c_handle));
	}
	internal_status->camera_connected = 0;
    }
} /* disconnect_cam() */

/*
  -----------------------------------------------------------------------------
  Disconnects the camera from and connects it to the bus.  Any changes
  in the cfg and set arguments take effect.  This function is used
  mainly to re-initialize the video1394 driver interface for things like
  flushing the frame buffers or changing the frame dimensions or frame
  rate.  If the camera is running, it is stopped and the process sleeps
  for at least one frame time before disconnecting.  Returns
  CAMERA_SUCCESS on success or CAMERA_FAILURE on failure.
*/
static int reconnect_cam(const Camwire_handle c_handle, Camwire_conf *cfg,
			 const Camwire_state *set)
{
    if (set->running)
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_set_run_stop(c_handle, 0));
    ERROR_IF_CAMERA_FAIL(
	    sleep_frametime(c_handle, 1.5));
    }
    disconnect_cam(c_handle);
    ERROR_IF_CAMERA_FAIL(
	connect_cam(c_handle, cfg, set));
    return CAMERA_SUCCESS;
} /* reconnect_cam() */

/*
  -----------------------------------------------------------------------------
  Frees the memory allocated in create().  Should only ever be called
  from create() and camwire_destroy().  Assumes a valid c_handle.
*/
static void free_internals(const Camwire_handle c_handle)
{
    User_handle internal_status;

    internal_status = camwire_bus_get_userdata(c_handle);
    if (internal_status)
    {
	if (internal_status->frame_lock)
	{
	    dc1394_capture_enqueue(camwire_handle_get_camera(c_handle),
				   internal_status->frame);
	    internal_status->frame = 0;
	    internal_status->frame_lock = 0;
	}
	free(internal_status->config_cache);
	free(internal_status->extras);
	free(internal_status);
	camwire_bus_set_userdata(c_handle, 0);
    }
} /* free_internals() */

/*
  -----------------------------------------------------------------------------
  Returns true if the given feature is available, readable, and manually
  controllable, as reported by the given dc1394feature_info_t structure.
  The trigger feature is an exception in that it does not have auto or
  manual settings.  */
inline static int feature_has_mode(const dc1394feature_info_t *cap,
				   const dc1394feature_mode_t mode)
{
    int m;
    
    for (m=0; m<cap->modes.num; ++m)
	if (cap->modes.modes[m] == mode)  return 1;
    return 0;
    
} /* feature_has_mode() */

/*
  -----------------------------------------------------------------------------
  Returns true if the given feature is available, readable, and manually
  controllable, as reported by the given dc1394feature_info_t structure.
  The trigger feature is an exception in that it does not have auto or
  manual settings.  */
inline static int feature_is_usable(const dc1394feature_info_t *cap)
{
    return (cap->available == DC1394_TRUE &&
	    cap->readout_capable == DC1394_TRUE &&
	    (cap->id == DC1394_FEATURE_TRIGGER ||
	     feature_has_mode(cap, DC1394_FEATURE_MODE_MANUAL)));
} /* feature_is_usable() */

/*
  -----------------------------------------------------------------------------
  Switches the given feature on if it is on-off capable.  (If it is not
  on-off capable we assume that it is on by default.)
*/
static int feature_switch_on(const Camwire_handle c_handle,
			     dc1394feature_info_t *cap)
{
    char error_message[ERROR_MESSAGE_MAX_CHARS+1];
    
    if (cap->on_off_capable == DC1394_TRUE && cap->is_on == DC1394_OFF)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_set_power(camwire_handle_get_camera(c_handle),
				     cap->id,
				     DC1394_ON));
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_get_power(camwire_handle_get_camera(c_handle),
				 cap->id,
				 &cap->is_on));
	if (cap->is_on != DC1394_ON)
	{
	    snprintf(error_message,
		     ERROR_MESSAGE_MAX_CHARS,
		     "Could not switch %s on.",
		     dc1394_feature_get_string(cap->id));
	    DPRINTF(error_message);
        return CAMERA_FAILURE;
	}
    }
    return CAMERA_SUCCESS;
} /* feature_switch_on() */

/*
  -----------------------------------------------------------------------------
  Switches the given feature to manual if it is auto capable and on
  auto, assuming that it is manual capable.  (If it is not auto capable
  we assume that it is manual by default.)
*/
static int feature_go_manual(const Camwire_handle c_handle,
			     dc1394feature_info_t *cap)
{
    char error_message[ERROR_MESSAGE_MAX_CHARS+1];
    
    if (cap->current_mode != DC1394_FEATURE_MODE_MANUAL)
    {
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_set_mode(camwire_handle_get_camera(c_handle),
				    cap->id,
				    DC1394_FEATURE_MODE_MANUAL));
	ERROR_IF_DC1394_FAIL(
	    dc1394_feature_get_mode(camwire_handle_get_camera(c_handle),
				    cap->id,
				    &cap->current_mode));
	if (cap->current_mode != DC1394_FEATURE_MODE_MANUAL)
	{
	    snprintf(error_message,
		     ERROR_MESSAGE_MAX_CHARS,
		     "Could not switch %s to manual.",
		     dc1394_feature_get_string(cap->id));
	    DPRINTF(error_message);
        return CAMERA_FAILURE;
	}
    }
    return CAMERA_SUCCESS;
} /* feature_go_manual() */

/*
  -----------------------------------------------------------------------------
  Returns a pointer to the dc1394feature_info_t structure for the given
  Camwire handle and dc1394 feature enumeration index, or 0 on error.
  Needed by functions that deal with camera features.
*/
inline static
dc1394feature_info_t * get_feature_capability(const Camwire_handle c_handle,
					      const dc1394feature_t feature)
{
    User_handle internal_status;

    internal_status = camwire_bus_get_userdata(c_handle);
    if (!internal_status)  return 0;
    
#ifdef CAMERA_DEBUG
    if (internal_status->feature_set.feature[feature-DC1394_FEATURE_MIN].id !=
	feature)
    {
	DPRINTF("Requested feature does not match feature_set.id.");
	return 0;
    }
#endif

    return &internal_status->feature_set.feature[feature-DC1394_FEATURE_MIN];
} /* get_feature_capability() */

/*
  -----------------------------------------------------------------------------
  Returns 1 (true) if the camera implements an internal gamma-correction
  look-up table, or 0 (false) otherwise.  Gamma correction is a
  non-standard advanced feature so it has to be tested differently on
  each supported model.
*/
    /* Gamma supported only in AVT cameras at the moment.*/
static int probe_camera_gamma(const Camwire_handle c_handle)
{
    int dc1394_return;
    dc1394_avt_adv_feature_info_t adv_capability;

    dc1394_return =
	dc1394_avt_get_advanced_feature_inquiry(camwire_handle_get_camera(c_handle),
						&adv_capability);
    return (dc1394_return == DC1394_SUCCESS &&
	    adv_capability.Lookup_Tables == DC1394_TRUE);
} /* probe_camera_gamma() */

/*
  -----------------------------------------------------------------------------
  Returns 1 (true) if the camera implements an internal
  colour-correction matrix, or 0 (false) otherwise.  Colour correction
  is a non-standard advanced feature so it has to be tested differently
  on each supported model.
*/

    /* Colour correction supported only in AVT cameras at the moment.*/
static int probe_camera_colour_correction(const Camwire_handle c_handle)
{
    int dc1394_return;
    dc1394_avt_adv_feature_info_t adv_feature;

    dc1394_return =
	dc1394_avt_get_advanced_feature_inquiry(camwire_handle_get_camera(c_handle),
						&adv_feature);
    return (dc1394_return == DC1394_SUCCESS &&
	    adv_feature.ColorCorrection != DC1394_FALSE);
} /* probe_camera_colour_correction() */

/*
  -----------------------------------------------------------------------------
  Returns the pixel tiling as obtained directly from the camera.
*/
static Camwire_tiling probe_camera_tiling(const Camwire_handle c_handle)
{
    dc1394video_mode_t video_mode;
    uint32_t filter_id;
    int dc1394_return;
    
    video_mode = get_1394_video_mode(c_handle);
    if (video_mode == 0)
    {
	DPRINTF("Video mode is zero.");
    return CAMERA_TILING_INVALID;
    }
    dc1394_return = dc1394_format7_get_color_filter(camwire_handle_get_camera(c_handle),
						    video_mode,
						    &filter_id);
    if (dc1394_return != DC1394_SUCCESS)  
    {
	DPRINTF("dc1394_format7_get_color_filter() failed.");
    return CAMERA_TILING_INVALID;
    }
    return convert_filterid2pixeltiling(filter_id);
} /* probe_camera_tiling() */

/*
  -----------------------------------------------------------------------------
  Returns a pointer to the dc1394video_frame_t structure for the given
  camwire handle, or 0 on error.  Needed by many libdc1394
  functions.
*/
inline static
dc1394video_frame_t * get_captureframe(const Camwire_handle c_handle)
{
    User_handle internal_status;

    internal_status = camwire_bus_get_userdata(c_handle);
    if (!internal_status)  return 0;
    return internal_status->frame;
} /* get_captureframe() */

/*
  -----------------------------------------------------------------------------
  Returns a pointer to the Camwire_state structure for the given camwire
  handle, or 0 on error.  Needed by many camwire_get/set_...()
  functions.
*/
inline static Camwire_state * get_shadow_state(const Camwire_handle c_handle)
{
    User_handle internal_status;

    internal_status = camwire_bus_get_userdata(c_handle);
    if (!internal_status)  return 0;
    return internal_status->current_set;
} /* get_shadow_state() */

/*
  -----------------------------------------------------------------------------
  Returns the IEEE 1394 image video_mode, or 0 on error.
*/
inline static dc1394video_mode_t get_1394_video_mode(const Camwire_handle c_handle)
{
    Camwire_conf config;

    if (camwire_get_config(c_handle, &config) != CAMERA_SUCCESS)  return 0;
    /* FIXME: Need a better way of checking cache initialization than bus_speed: */
    if (config.bus_speed == 0)  return 0;
    return convert_format_mode2dc1394video_mode(config.format, config.mode);
    /* FIXME: Kludge to get it working. Redo video_mode in config. */
} /* get_1394_video_mode() */

/*
  -----------------------------------------------------------------------------
  Returns 1 (true) if the IEEE 1394 image format is a fixed image size,
  or 0 (false) otherwise.
*/
inline static int fixed_image_size(const dc1394video_mode_t video_mode)
{
    return (dc1394_is_video_mode_scalable(video_mode) == DC1394_FALSE &&
	    dc1394_is_video_mode_still_image(video_mode) == DC1394_FALSE);
} /* fixed_image_size() */

/*
  -----------------------------------------------------------------------------
  Returns 1 (true) if the IEEE 1394 image format is a variable image
  size, or 0 (false) otherwise.
*/
inline static int variable_image_size(const dc1394video_mode_t video_mode)
{
    return (dc1394_is_video_mode_scalable(video_mode) == DC1394_TRUE &&
	    dc1394_is_video_mode_still_image(video_mode) == DC1394_FALSE);
} /* variable_image_size() */

/*
  -----------------------------------------------------------------------------
  Gets the camera's current settings from the state shadow or as
  physically read from the camera, depending on the state shadow flag.
*/
static int get_current_settings(const Camwire_handle c_handle,
				Camwire_state *set)
{
    Camwire_state *shadow_state;
    dc1394bool_t one_shot_set;
    
    shadow_state = get_shadow_state(c_handle);
    ERROR_IF_NULL(shadow_state);
    if (shadow_state->shadow)
    {
	memcpy(set, shadow_state, sizeof(Camwire_state));  /* Shortcut.*/
	
	/* One_Shot register self-clears after transmission, hence we
	   don't know if camera is still runnning: */
	if (shadow_state->running && shadow_state->single_shot)
	{
	    ERROR_IF_DC1394_FAIL(
		dc1394_video_get_one_shot(camwire_handle_get_camera(c_handle),
					  &one_shot_set));
	    if (one_shot_set == DC1394_FALSE)
		set->running = shadow_state->running = 0;
	}
    }
    else
    {
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_num_framebuffers(c_handle, &set->num_frame_buffers));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_gain(c_handle, &set->gain));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_brightness(c_handle, &set->brightness));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_white_balance(c_handle, set->white_balance));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_gamma(c_handle, &set->gamma));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_colour_correction(c_handle, &set->colour_corr));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_colour_coefficients(c_handle, set->colour_coef));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_frame_offset(c_handle, &set->left, &set->top));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_frame_size(c_handle, &set->width, &set->height));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_pixel_coding(c_handle, &set->coding));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_pixel_tiling(c_handle, &set->tiling));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_framerate(c_handle, &set->frame_rate));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_shutter(c_handle, &set->shutter));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_trigger_source(c_handle, &set->external_trigger));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_trigger_polarity(c_handle, &set->trigger_polarity));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_single_shot(c_handle, &set->single_shot));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_run_stop(c_handle, &set->running));
    ERROR_IF_CAMERA_FAIL(
	    camwire_get_stateshadow(c_handle, &set->shadow));
    }
    return CAMERA_SUCCESS;
} /* get_current_settings() */

/*
  -----------------------------------------------------------------------------
  Sleeps for multiple frame periods, where multiple is given by the
  second argument.  Typically used after stopping transmission to make
  sure any stray frames have been dispatched.  Returns CAMERA_SUCCESS
  on success or CAMERA_FAILURE on failure.
*/
static int sleep_frametime(const Camwire_handle c_handle,
			   const double multiple)
{
    double frame_rate;
    double sleep_period;
    struct timespec nap, left;
    
    ERROR_IF_CAMERA_FAIL(
	camwire_get_framerate(c_handle, &frame_rate));
    sleep_period = multiple/frame_rate;
    nap.tv_sec = (time_t) sleep_period; 	/* Trunc. to integer.*/
    nap.tv_nsec = (long)((sleep_period - nap.tv_sec)*1e9);
    if (nanosleep(&nap, &left) != 0)
    {
	DPRINTF("nanosleep() failed.");
    return CAMERA_FAILURE;
    }
    return CAMERA_SUCCESS;
} /* sleep_frametime() */

/*
  -----------------------------------------------------------------------------
  Returns the video frame rate for the given libdc1394 index, or -1.0 if
  it is not recognized.
*/
static double convert_index2framerate(const dc1394framerate_t frame_rate_index)
{
    int dc1394_return;
    float frame_rate;

    dc1394_return = dc1394_framerate_as_float(frame_rate_index, &frame_rate);
    if (dc1394_return == DC1394_SUCCESS)  return (double)frame_rate;
    else                                  return -1.0;
} /* convert_index2framerate() */

/*
  -----------------------------------------------------------------------------
  Returns the nearest valid libdc1394 index for the given video frame
  rate.  The list of supported frame rates must not be empty.
*/
static
dc1394framerate_t convert_framerate2index(const double frame_rate,
					  const dc1394framerates_t *framerate_list)
{
    float min_fr;
    double fps, log2f, best, diff;
    int nearest_index, r, rate_index;

    dc1394_framerate_as_float(DC1394_FRAMERATE_MIN, &min_fr);  /* 1.875.*/
    if (frame_rate > 0.0)  fps = frame_rate;
    else                   fps = min_fr;
    log2f = log(fps/min_fr)/log(2);  /* 1.875->0, 3.75->1, 7.5->2, etc.*/
    best = DBL_MAX;
    nearest_index = -1;
    for (r=0; r<framerate_list->num; ++r)
    {
	rate_index = framerate_list->framerates[r];
	diff = fabs(log2f - rate_index + DC1394_FRAMERATE_MIN);
	if (diff < best)
	{
	    best = diff;
	    nearest_index = rate_index;
	}
    }
    if (nearest_index >= 0)  return nearest_index;
    else                     return 0;  /* Empty list?*/
} /* convert_framerate2index() */

/*
  -----------------------------------------------------------------------------
  Returns the number of packets required to transmit a single frame, as
  obtained from the camera.
*/
static int get_numpackets(const Camwire_handle c_handle, uint32_t *num_p)
{
    dc1394video_mode_t video_mode;
/*     dc1394framerate_t frame_rate_index; */
    dc1394color_coding_t color_id;
    uint32_t packet_size, width, height;
    Camwire_pixel coding;
    
    video_mode = get_1394_video_mode(c_handle);
    ERROR_IF_ZERO(video_mode);

    /* The FORMAT_VGA_NONCOMPRESSED case is never called: */
/*     if (fixed_image_size(video_mode))  /\* Format 0, 1 or 2.*\/ */
/*     { */
/* 	ERROR_IF_DC1394_FAIL( */
/* 	    dc1394_video_get_framerate(camwire_handle_get_camera(c_handle), */
/* 				       &frame_rate_index)); */
/* 	switch (frame_rate_index)  */
/* 	{ */
/* 	    case DC1394_FRAMERATE_1_875: */
/* 	    	*num_p = 3840; */
/* 	    	break; */
/* 	    case DC1394_FRAMERATE_3_75: */
/* 		*num_p = 1920; */
/* 		break; */
/* 	    case DC1394_FRAMERATE_7_5: */
/* 		*num_p = 960; */
/* 		break; */
/* 	    case DC1394_FRAMERATE_15: */
/* 		*num_p = 480; */
/* 		break; */
/* 	    case DC1394_FRAMERATE_30: */
/* 		*num_p = 240; */
/* 		break; */
/* 	    case DC1394_FRAMERATE_60: */
/* 		*num_p = 120; */
/* 		break; */
/* 	    case DC1394_FRAMERATE_120: */
/* 		*num_p = 60; */
/* 		break; */
/* 	    case DC1394_FRAMERATE_240: */
/* 		*num_p = 30; */
/* 		break; */
/* 	    default: */
/* 		DPRINTF("dc1394_video_get_framerate() returned " */
/* 			"invalid index."); */
/* 		return CAMERA_FAILURE; */
/* 	} */
/*     } */
/*     else if (variable_image_size(video_mode))  /\* Format 7.*\/ */
/*     { */
	/* PACKET_PER_FRAME_INQ depends on BYTE_PER_PACKET which in turn
	   depends on IMAGE_SIZE and COLOR_CODING_ID.  It is only safe
	   to use the value returned by
	  dc1394_format7_get_packets_per_frame() (or
	  dc1394_format7_get_packet_parameters() below) if these registers
	   are not being changed by higher functions calling
	   get_numpackets(): */
	ERROR_IF_DC1394_FAIL(
	   dc1394_format7_get_packets_per_frame(
		camwire_handle_get_camera(c_handle),
		video_mode,
		num_p));
	if (*num_p == 0)
	{
	    /* If dc1394_format7_get_packets_per_frame() returns a zero
	       number of packets, then the IIDC spec says we should
	       calculate it ourselves: */
	    ERROR_IF_DC1394_FAIL(
		dc1394_format7_get_packet_size(camwire_handle_get_camera(c_handle),
					       video_mode,
					       &packet_size));
	    ERROR_IF_DC1394_FAIL(
		dc1394_format7_get_image_size(camwire_handle_get_camera(c_handle),
					      video_mode,
					      &width,
					      &height));
	    ERROR_IF_DC1394_FAIL(
		dc1394_format7_get_color_coding(camwire_handle_get_camera(c_handle),
						video_mode,
						&color_id));
	    coding = convert_colorid2pixelcoding(color_id);
	    *num_p = convert_packetsize2numpackets(c_handle, packet_size,
						   width, height, coding);
	    ERROR_IF_ZERO(*num_p);
	}
/*     } */
/*     else */
/*     { */
/* 	DPRINTF("Unsupported camera format."); */
/* 	return CAMERA_FAILURE; */
/*     } */

    return CAMERA_SUCCESS;
} /* get_numpackets() */

/*
  -----------------------------------------------------------------------------
  Returns the (positive, non-zero) number of video packets per frame for
  the given packet_size.  Returns 0 on error.
*/
static uint32_t convert_packetsize2numpackets(const Camwire_handle c_handle,
					 const uint32_t packet_size,
					 const int width,
					 const int height,
					 const Camwire_pixel coding)
{
    long denominator;
    uint32_t num_packets;
    int depth;
    Camwire_conf config;
    
    /* num_packets = ceil(framesize/packet_size): */
    if (camwire_pixel_depth(coding, &depth) != CAMERA_SUCCESS)
    {
	DPRINTF("camwire_pixel_depth() failed.");
	return 0;
    }
    denominator = (long)packet_size*8;
    if (denominator > 0)
    {
	num_packets =
	    ((long)width*height*depth + denominator - 1)/denominator;
	if (num_packets < 1)  num_packets = 1;
    }
    else
    {
    if (camwire_get_config(c_handle, &config) != CAMERA_SUCCESS)
	{
	    DPRINTF("camwire_get_config() failed.");
	    return 0;
	}
	num_packets = config.max_packets;
    }
    return num_packets;
} /* convert_packetsize2numpackets() */

/*
  -----------------------------------------------------------------------------
  Returns the nearest packet_size for the given number of video packets
  per frame.
*/
/* Required limitations imposed on intermediate values:
   num_packets is a positive integer
   num_packets <= max_packets
   packet_size = unit_bytes * n where n is a positive integer
   packet_size <= max_bytes
   where unit_bytes and max_bytes are obtained from the PACKET_PARA_INQ
   register.

   Note that we don't use the function dc1394_query_total_bytes() which
   reads the TOTAL_BYTE_INQ camera registers, because different
   manufacturers apparently interpret it differently. See comment in
   libdc1394/dc1394_format7.c.
*/

static uint32_t convert_numpackets2packetsize(const Camwire_handle c_handle,
					 const uint32_t num_packets,
					 const int width,
					 const int height,
					 const Camwire_pixel coding)
{
    Camwire_conf config;
    dc1394video_mode_t video_mode;
    uint32_t packet_size;
    uint32_t unit_bytes, max_bytes;
    int depth;
    long denominator;

    if (camwire_get_config(c_handle, &config) != CAMERA_SUCCESS)
    {
	DPRINTF("camwire_get_config() failed.");
	return 0;
    }
    video_mode = get_1394_video_mode(c_handle);

#ifdef CAMERA_DEBUG
    /* FIXME: Need a better way of checking cache initialization than bus_speed: */
    if (config.bus_speed == 0)
    {
	DPRINTF("camwire_get_config() returned null format.");
	return 0;
    }
    else if (!variable_image_size(video_mode))
    { 	/* Not Format 7.*/
	DPRINTF("Camera is not in Format 7.");
	return 0;
    }
    if (num_packets < 1 || num_packets > config.max_packets)
    {
	DPRINTF("Number of packets is out of range.");
	return 0;
    }
#endif

   if (camwire_pixel_depth(coding, &depth) != CAMERA_SUCCESS)
    {
	DPRINTF("camwire_pixel_depth() failed.");
	return 0;
    }

    /* Set unit_bytes quantum and max_bytes packet size, even if we
       cannot yet access the camera: */
    unit_bytes = max_bytes = 0;
    dc1394_format7_get_packet_parameters(camwire_handle_get_camera(c_handle),
					 video_mode,
					 &unit_bytes, &max_bytes);
    if (unit_bytes < 4)  unit_bytes = 4; 	/* At least a quadlet.*/
    if (max_bytes < unit_bytes)
	/* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
	max_bytes = 4*(4915*config.bus_speed/1600 - 3);
	/* Max 4915 quadlets in S1600, less 3 for header and data CRC. */

    /* packet_size = ceil(framesize/num_packets), quantized to unit_bytes: */
    denominator = (long)unit_bytes*num_packets*8;
    packet_size =
	(((long)width*height*depth + denominator - 1)/denominator)*unit_bytes;
    
    /* Check limits: */
/*     if (packet_size < unit_bytes)  packet_size = unit_bytes; */
    /* Testing (packet_size < unit_bytes) should not be necessary.*/
    if (packet_size > max_bytes)  packet_size = max_bytes;
    
    return packet_size;
} /* convert_numpackets2packetsize() */

/*
  -----------------------------------------------------------------------------
  Returns the number of video packets per frame corresponding to the
  given frame rate.
*/
static uint32_t convert_framerate2numpackets(const Camwire_handle c_handle,
					const double frame_rate)
{
    Camwire_conf config;
    uint32_t num_packets;
    
    if (camwire_get_config(c_handle, &config) != CAMERA_SUCCESS)
    {
	DPRINTF("camwire_get_config() failed.");
	return 0;
    }

#ifdef CAMERA_DEBUG
    /* FIXME: Need a better way of checking cache initialization than bus_speed: */
    if (config.bus_speed == 0)
    {
	DPRINTF("camwire_get_config() returned null format.");
	return 0;
    }
#endif

    if (frame_rate <= 0)  return config.max_packets;
    /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
    num_packets =
	(uint32_t)(convert_busspeed2busfreq(config.bus_speed)/frame_rate + 0.5);
    if (num_packets < 1)  num_packets = 1;
    if (num_packets > config.max_packets)  num_packets = config.max_packets;
    return num_packets;
} /* convert_framerate2numpackets() */

/*
  -----------------------------------------------------------------------------
  Returns the libdc1394 colour coding ID that supports the given pixel
  coding, or 0 on error.  The coding_list argument must not be empty.
*/
static uint32_t convert_pixelcoding2colorid(const Camwire_pixel coding,
					    const dc1394color_codings_t *coding_list)
{
    switch (coding)
    {
    case CAMERA_PIXEL_MONO8:  /* 8 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_MONO8))
		return DC1394_COLOR_CODING_MONO8;
	    break;
    case CAMERA_PIXEL_YUV411:  /* 12 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_YUV411))
		return DC1394_COLOR_CODING_YUV411;
	    break;
    case CAMERA_PIXEL_YUV422:  /* 16 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_YUV422))
		return DC1394_COLOR_CODING_YUV422;
	    break;
    case CAMERA_PIXEL_YUV444:  /* 24 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_YUV444))
		return DC1394_COLOR_CODING_YUV444;
	    break;
    case CAMERA_PIXEL_RGB8:  /* 24 bits/pixel.*/
	    if (is_in_coding_list(coding_list,DC1394_COLOR_CODING_RGB8))
		return DC1394_COLOR_CODING_RGB8;
	    break;
    case CAMERA_PIXEL_MONO16:  /* 16 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_MONO16))
		return DC1394_COLOR_CODING_MONO16;
	    break;
    case CAMERA_PIXEL_RGB16:  /* 48 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_RGB16))
		return DC1394_COLOR_CODING_RGB16;
	    break;
    case CAMERA_PIXEL_MONO16S:  /* 16 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_MONO16S))
		return DC1394_COLOR_CODING_MONO16S;
	    break;
    case CAMERA_PIXEL_RGB16S:  /* 48 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_RGB16S))
		return DC1394_COLOR_CODING_RGB16S;
	    break;
    case CAMERA_PIXEL_RAW8:  /* 8 bits/pixel.*/
	    if (is_in_coding_list(coding_list,DC1394_COLOR_CODING_RAW8 ))
		return DC1394_COLOR_CODING_RAW8;
	    break;
    case CAMERA_PIXEL_RAW16:  /* 16 bits/pixel.*/
	    if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_RAW16))
		return DC1394_COLOR_CODING_RAW16;
	    break;
	default:
	    return 0;  /* No such coding.*/
	    break;
    }
    return 0;  /* Not supported by camera.*/
} /* convert_pixelcoding2colorid() */

/*
  -----------------------------------------------------------------------------
  Returns the dc1394 video_mode corresponding to the given numeric
  format and mode.  */
inline static dc1394video_mode_t convert_format_mode2dc1394video_mode(const int format,
								       const int mode)
{
    return mode_dc1394_offset[format] + mode;
} /* convert_format_mode2dc1394video_mode() */

/*
  -----------------------------------------------------------------------------
  Returns the numeric format and mode corresponding to the given dc1394
  video_mode.  */
inline static void convert_dc1394video_mode2format_mode(
    const dc1394video_mode_t video_mode, int *format, int *mode)
{
    int fmt, mode_offset;

    for (fmt=7; fmt>=0; --fmt)
    {
	mode_offset = mode_dc1394_offset[fmt];
	if (mode_offset != 0 && video_mode >= mode_offset)
	{
	    *format = fmt;
	    *mode = video_mode - mode_offset;
	    break;
	}
    }
} /* convert_dc1394video_mode2format_mode() */

/*
  -----------------------------------------------------------------------------
  Returns the AVT signed 32-bit int register values corresponding to the
  9 given colour correction coefficients.
*/
static void convert_colourcoefs2avtvalues(const double coef[9], int32_t val[9])
{
    double safe[9];
    int c, rowsum, col;

    memcpy(safe, coef, 9*sizeof(double));
    for (c=0; c<9; ++c)
    {
	/* Limit each coef to [-1..+2]: */
	if (safe[c] < -1.0)  safe[c] = -1.0;
	if (safe[c] > 2.0)   safe[c] = 2.0;

	/* Convert by scaling and rounding in the right direction: */
	if (safe[c] >= 0)  val[c] = (int)(1000*safe[c] + 0.5);
	else               val[c] = (int)(1000*safe[c] - 0.5);
    }
    rowsum = 0.0;
    col = 0;
    for (c=0; c<9; ++c)
    {
	/* Limit sum of each row to +-2000: */
	rowsum += val[c];
	++col;
	if (col == 3)
	{
	    if (rowsum > 2000)
	    {
		val[c-2] = (val[c-2]*2000 + rowsum*(2000-1)/6000)/rowsum;
		val[c-1] = (val[c-1]*2000 + rowsum*(2000-1)/6000)/rowsum;
		val[c-0] = (val[c-0]*2000 + rowsum*(2000-1)/6000)/rowsum;
	    }
	    if (rowsum < -2000)
	    {
		val[c-2] = -(val[c-2]*2000 + rowsum*(2000-1)/6000)/rowsum;
		val[c-1] = -(val[c-1]*2000 + rowsum*(2000-1)/6000)/rowsum;
		val[c-0] = -(val[c-0]*2000 + rowsum*(2000-1)/6000)/rowsum;
	    }
	    rowsum = 0;
	    col = 0;
	}
    }
} /* convert_colourcoefs2avtvalues() */

/*
  -----------------------------------------------------------------------------
  Returns the colour correction coefficients corresponding to the
  9 given AVT signed 32-bit int register values.
*/
static void convert_avtvalues2colourcoefs(const int32_t val[9], double coef[9])
{
    int c;
    for (c=0; c<9; ++c)  coef[c] = val[c]/1000.0;
} /* convert_avtvalues2colourcoefs() */

/*
  ----------------------------------------------------------------------
  Returns a pointer to the 1st non-whitespace character in
  null-delimited string.  If the string contains only whitespace,
  returns a pointer to an empty string (pointer to the null character at
  the end of the string).
*/
static char * skip_whitespace(const char *string)
{
    return((char *) &string[strspn(string, WHITESPACE)]);
}

/*
  ----------------------------------------------------------------------
  Returns a pointer to the 1st whitespace character in null-delimited
  string.  If the string contains no whitespace, returns a pointer to an
  empty string (pointer to the null character at the end of the string).
*/
static char * skip_non_whitespace(const char *string)
{
    return((char *) &string[strcspn(string, WHITESPACE)]);
}

/*
  ----------------------------------------------------------------------
  Returns true (1) if the given color_id is in the given coding_list,
  else returns false (0).
 */

static int is_in_coding_list(const dc1394color_codings_t *coding_list,
			     const dc1394color_coding_t color_id)
{
    int c;
    
    for (c=0; c<coding_list->num; ++c)
    {
	if (coding_list->codings[c] == color_id)  return 1;
    }
    return 0;
}
