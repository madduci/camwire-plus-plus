#ifndef CAMWIRE_HPP
#define CAMWIRE_HPP
/******************************************************************************

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


    Title: Header for camwire.cpp

    Description:
    This module is about using a single named camera through its
    handle. The handle should be all a user need know about for complete
    access to all camera functions.  Finding cameras and assigning
    handles to them is done in the Camwire bus module.

Camwire++: Michele Adduci <info@micheleadduci.net>
******************************************************************************/

#include <camwire_handle.hpp>

namespace camwire
{
    class camwire
    {

        public:
            camwire();
            ~camwire();
            /* Sets the camera to default initialization settings and connects it to
               the bus.  This function is equivalent to getCameraState()
               followed by initFromStruct().  The handle c_handle is
               obtained from CameraBusHandle.init(). */
            int create(const Camwire_bus_handle_ptr &c_handle);
            /* Sets the camera to the given initialization settings and connects it
               to the bus.  The CameraState structure is returned unchanged.  The
               handle c_handle is obtained from CameraBusHandle.init(). */
            int create_from_struct(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set);
            /* Disconnects the camera from the bus and frees memory allocated in
               init() or initFromStruct().  All camera
               settings are lost.*/
            int destroy(const Camwire_bus_handle_ptr &c_handle);

            int get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);

            int set_run_stop(const Camwire_bus_handle_ptr &c_handle, const int runsts = 0);
            /* Gets the camera and its bus's static configuration settings for
               initialization from a configuration file.  They are bus-specific
               hardware parameters that the casual user need not know or care about.
               If a configuration file does not exist, an error message is printed which includes a
               best-guess default configuration. */
            int get_config(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg);
            /* Fills in the given camwire identifier structure.
               The identifier is uniquely and permanently associated with the camera
               hardware, such as might be obtained from configuration ROM data.
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_identifier(const Camwire_bus_handle_ptr &c_handle, Camwire_id &identifier);

            /* Sets the state shadow flag: 1 to get camera settings from an internal
               shadow structure or 0 to read them directly from the camera hardware.
               Setting state shadowing may result in faster camera responses and may
               increase robustness because the camera firmware is not interrupted by
               unnecessary status requests.  State shadowing may however be slightly
               risky because bugs in Camwire or in the camera firmware could cause
               the shadow state to differ from the actual state.  The state shadow
               flag of a created camera can be modified at any time because the
               shadow state is internally maintained irrespective of the state
               shadow flag status.  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure.*/
            int set_stateshadow(const Camwire_bus_handle_ptr &c_handle, const int shadow);
            /* Sets the camera's trigger source: 1 for external or 0 for internal.
               Fails if the camera does not have an external trigger.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int set_trigger_source(const Camwire_bus_handle_ptr &c_handle, const int external);
            /* Sets the camera's trigger polarity: 1 for rising edge (active high)
               or 0 for falling edge (active low).  Fails if the trigger polarity is
               not settable.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE
               on failure. */
            int set_trigger_polarity(const Camwire_bus_handle_ptr &c_handle, const int rising);
            /* Sets the given shutter speed (exposure time in seconds) to the
               nearest valid value for the camera.  The actual value can be checked
               afterwards with camwire_get_shutter().  Returns CAMWIRE_SUCCESS on
               success or CAMWIRE_FAILURE on failure. */
            int set_shutter(const Camwire_bus_handle_ptr &c_handle, const double shutter);
            /* Sets the gain factor (response slope) to the nearest value valid for
               the camera.  The gain can be checked afterwards with
               camwire_get_gain().  Gain values can range between the relative
               values of 0.0 (minimum) and 1.0 (maximum).  The user has to normalize
               the actual wanted gain to this range before calling
               camwire_set_gain().  For example, if the camera specifies the minimum
               gain as a slope of 1.0 (0dB) and the maximum as 4.0 (12dB), then the
               relative gain to set is (actual_gain - 1.0)/(4.0 - 1.0).  Fails if
               the gain is not settable.  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure. */
            int set_gain(const Camwire_bus_handle_ptr &c_handle, const double gain);
            /* Sets the brightness (black level) to the nearest value valid for the
               camera.  The brightness can be checked afterwards with
               camwire_get_brightness().  Brightness values can range between the
               relative values of -1.0 (minimum) and +1.0 (maximum).  The user has
               to normalize the actual wanted brightness to this range before
               calling camwire_set_brightness().  For example, if the camera
               specifies the minimum brightness level as 0 and the maximum as 1023,
               then the relative brightness to set is 2.0*actual_brightness/(1023 -
               0) - 1.0.  Fails if the brightness is not settable.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int set_brightness(const Camwire_bus_handle_ptr &c_handle, const double brightness);
            /* Sets the white balance levels (blue or U, red or V) for colour
               cameras to the nearest values valid for the camera.  bal containts
               numbers between 0.0 (minimum level) and 1.0 (maximum level).  The
               levels can be checked afterwards with camwire_get_white_balance().
               Fails if the white balance is not settable.  Returns CAMWIRE_SUCCESS
               on success or CAMWIRE_FAILURE on failure. */
            int set_white_balance(const Camwire_bus_handle_ptr &c_handle, const double bal[2]);
            /* Sets the camera's colour correction setting corr_on, 1 for
               colour-corrected or 0 for no correction.  Colour correction only
               takes effect if the current pixel coding is colour (such as
               CAMWIRE_PIXEL_YUV422 or CAMWIRE_PIXEL_RGB8, but not
               CAMWIRE_PIXEL_MONO8 etc.).  So far Camwire supports colour correction
               in AVT cameras only.  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure or if the camera is not capable of colour
               correction. */
            int set_colour_correction(const Camwire_bus_handle_ptr &c_handle, const int corr_on);
            /* Sets the camera's colour correction coefficients.  coef is an array
               of 9 colour correction coefficients, see the description of the
               colour_coef member of Camwire_state above.  Some cameras support
               colour correction with fixed coefficients in which case
               camwire_set_colour_correction() above can enable colour correction
               but this function would fail because it cannot change the
               coefficients.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE
               on failure or if the camera is not capable of setting colour
               coefficients. */
            int set_colour_coefficients(const Camwire_bus_handle_ptr &c_handle, const double coef[9]);

        /* Set to protected in case of Subclassing */
        protected:
            Camwire_id cam_id;
            Camwire_user_data user_data;
            camwire(const camwire &cam);
            camwire& operator=(const camwire &cam);
            /*
             Does the actual work of camwire_create() and
              camwire_create_from_struct(), after they have initialized the camera
              to factory settings and sorted out where the start-up settings come
              from.
            */
            int create(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set);
            /* Queries the camera for supported features and attempts to create
               sensible default settings.  Note that the camera itself is initialized
               to factory settings in the process. */
            int generate_default_config(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &conf);
            /*
              Queries the camera for supported features and attempts to create
              sensible default settings.  Note that the camera itself is initialized
              to factory settings in the process.  Returns CAMWIRE_SUCCESS on
              success or CAMWIRE_FAILURE on failure.
            */
            int generate_default_settings(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);
            /* Gets the camera's current settings from the state shadow or as
              physically read from the camera, depending on the state shadow flag. */
            int get_current_settings(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);
            /*
              Stores in set a pointer to the Camwire_state structure for the given camwire
              handle, or a null pointer on error. Returns success on correct creation or failure on error.
              Needed by many camwire_get/set_...() functions.
            */
            int get_shadow_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);

            int sleep_frametime(const Camwire_bus_handle_ptr &c_handle, const double multiple);
            /*
              Connects the camera to the bus and sets it to the given configuration
              and initial settings.  Returns CAMWIRE_SUCCESS on success or
              CAMWIRE_FAILURE on failure.  The function disconnect_cam() must be
              called when done to free the allocated memory.
            */
            int connect_cam(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg, const Camwire_state_ptr &set);
            /*
              Disconnects the camera from and connects it to the bus.  Any changes
              in the cfg and set arguments take effect.  This function is used
              mainly to re-initialize the video1394 driver interface for things like
              flushing the frame buffers or changing the frame dimensions or frame
              rate.  If the camera is running, it is stopped and the process sleeps
              for at least one frame time before disconnecting.  Returns
              CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure.
            */
            int reconnect_cam(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg, const Camwire_state_ptr &set);

            /* Disconnects the camera from the bus and frees memory allocated in
              connect_cam().  The camera should be stopped before calling this
              function.
            */
            void disconnect_cam(const Camwire_bus_handle_ptr &c_handle);
            /*
              Frees the memory allocated in create().  Should only ever be called
              from create() and camwire_destroy().  Assumes a valid c_handle.
            */
            void free_internals(const Camwire_bus_handle_ptr &c_handle);
            /*
              Returns true if the configuration cache exists and has been
              initialized.  It is assumed that User_handle exists and
              is not 0.
            */
            int config_cache_exists(const User_handle &internal_status);
            /*
              Attempts to open a configuration file for reading.  Returns 1 if
              stream pointer creation was successful or 0 on failure.
            */
            int find_conf_file(const Camwire_id &id, std::shared_ptr<FILE> &conffile);
            /*
              Attempts to open the named configuration file for reading after
              appending the configuration filename extension.  Returns the stream
              pointer on success or 0 on failure.
            */
            int open_named_conf_file(const std::string &path, const std::string &filename, std::shared_ptr<FILE> &conffile);
            /*
              Reads configuration from the given conf file into the given
              configuration structure. Returns CAMWIRE_SUCCESS on success or
              CAMWIRE_FAILURE on failure.
            */
            int read_conf_file(const std::shared_ptr<FILE> &conffile, Camwire_conf_ptr &cfg);
            /* Writes the static configuration settings as obtained from
               camwire_get_config() to the given file.  The print format is the same
               as that expected by camwire_get_config() when it reads configuration
               files.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on
               failure.*/
            int write_config_to_file(const std::shared_ptr<FILE> &outfile, const Camwire_conf_ptr &cfg);
            int write_config_to_output(const Camwire_conf_ptr &cfg);
            /*
              Returns 1 (true) if the IEEE 1394 image format is a fixed image size,
              or 0 (false) otherwise.
            */
            int fixed_image_size(const dc1394video_mode_t video_mode);
            /*
              Returns 1 (true) if the IEEE 1394 image format is a variable image
              size, or 0 (false) otherwise.
            */
            int variable_image_size(const dc1394video_mode_t video_mode);
            /*
              Returns the IEEE 1394 image video_mode, or 0 on error.
            */
            dc1394video_mode_t get_1394_video_mode(const Camwire_bus_handle_ptr &c_handle);
            /*
              Returns a pointer to the dc1394feature_info_t structure for the given
              Camwire handle and dc1394 feature enumeration index, or 0 on error.
              Needed by functions that deal with camera features.
            */
            int get_feature_capability(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394feature_info_t> &cap, const dc1394feature_t feature);
            /*
              Returns true if the given feature is available, readable, and manually
              controllable, as reported by the given dc1394feature_info_t structure.
              The trigger feature is an exception in that it does not have auto or
              manual settings.  */
            int feature_is_usable(const std::shared_ptr<dc1394feature_info_t> &cap);
            /*
              Returns true if the given feature is available, readable, and manually
              controllable, as reported by the given dc1394feature_info_t structure.
              The trigger feature is an exception in that it does not have auto or
              manual settings.  */
            int feature_has_mode(const std::shared_ptr<dc1394feature_info_t> &cap, const dc1394feature_mode_t mode);
            /*
              Switches the given feature on if it is on-off capable.  (If it is not
              on-off capable we assume that it is on by default.)
            */
            int feature_switch_on(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394feature_info_t> &cap);
            /*
              Switches the given feature to manual if it is auto capable and on
              auto, assuming that it is manual capable.  (If it is not auto capable
              we assume that it is manual by default.)
            */
            int feature_go_manual(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394feature_info_t> &cap);
            /*
              Returns 1 (true) if the camera implements an internal
              colour-correction matrix, or 0 (false) otherwise.  Colour correction
              is a non-standard advanced feature so it has to be tested differently
              on each supported model.
            */
            /* Colour correction supported only in AVT cameras at the moment.*/
            int probe_camera_colour_correction(const Camwire_bus_handle_ptr &c_handle);
            /*
               Returns 1 (true) if the camera implements an internal gamma-correction
               look-up table, or 0 (false) otherwise.  Gamma correction is a
               non-standard advanced feature so it has to be tested differently on
               each supported model.
            */
            /* Gamma supported only in AVT cameras at the moment.*/
            int probe_camera_gamma(const Camwire_bus_handle_ptr &c_handle);
            /*
              Returns the frame rate corresponding to the given number of packets
              per frame.
            */
            double convert_numpackets2framerate(const Camwire_bus_handle_ptr &c_handle, const uint32_t num_packets);
            /*
              Returns the bus frequency (cycles per second) corresponding to the
              given bus speed (megabits per second).
            */
            double convert_busspeed2busfreq(const int bus_speed);
            /*
              Returns the libdc1394 data speed enumeration (SPEED_100, SPEED_200,
              etc.) corresponding to the given bus speed (megabits per second).
            */
            int convert_busspeed2dc1394(const int bus_speed);
            /*
              Returns the video frame rate for the given libdc1394 index, or -1.0 if
              it is not recognized.
            */
            double convert_index2framerate(const dc1394framerate_t frame_rate_index);
            /*
              Returns the nearest valid libdc1394 index for the given video frame
              rate.  The list of supported frame rates must not be empty.
            */
            int convert_framerate2index(const double frame_rate, const dc1394framerates_t &framerate_list);
            /*
              Returns the colour correction coefficients corresponding to the
              9 given AVT signed 32-bit int register values.
            */
            void convert_avtvalues2colourcoefs(const int32_t val[9], double coef[9]);
            /*
              Returns the pixel tiling as obtained directly from the camera.
            */
            Camwire_tiling probe_camera_tiling(const Camwire_bus_handle_ptr &c_handle);
            /*
              Returns the pixel coding given the libdc1394 mode in Formats 0, 1 and 2.
            */
            Camwire_pixel convert_videomode2pixelcoding(const dc1394video_mode_t video_mode);
            /*
              Returns the pixel coding given the libdc1394 colour coding ID in
              Format 7.
            */
            Camwire_pixel convert_colorid2pixelcoding(const dc1394color_coding_t color_id);
            /*
              Returns the pixel tiling given the libdc1394 colour coding ID in
              Format 7.
            */
            Camwire_tiling convert_filterid2pixeltiling(const dc1394color_filter_t filter_id);
            /*
              Returns the number of video packets per frame corresponding to the
              given frame rate.
            */
            uint32_t convert_framerate2numpackets(const Camwire_bus_handle_ptr &c_handle, const double frame_rate);
            /*
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
            uint32_t convert_numpackets2packetsize(const Camwire_bus_handle_ptr &c_handle, const uint32_t num_packets, const int width, const int height, const Camwire_pixel coding);
            /*
              Returns the (positive, non-zero) number of video packets per frame for
              the given packet_size.  Returns 0 on error.
            */
            uint32_t convert_packetsize2numpackets(const Camwire_bus_handle_ptr &c_handle, const uint32_t packet_size, const int width, const int height, const Camwire_pixel coding);

            /*
              Returns the libdc1394 colour coding ID that supports the given pixel
              coding, or 0 on error.  The coding_list argument must not be empty.
            */
            uint32_t convert_pixelcoding2colorid(const Camwire_pixel coding, const dc1394color_codings_t &coding_list);
            /*
              Returns true (1) if the given color_id is in the given coding_list,
              else returns false (0).
             */
            int is_in_coding_list(const dc1394color_codings_t &coding_list, const dc1394color_coding_t color_id);
            /*
              Returns the dc1394 video_mode corresponding to the given numeric
              format and mode.  */
            dc1394video_mode_t convert_format_mode2dc1394video_mode(const int format, const int mode);
            /* Translates the given Camwire pixel colour coding into the
               corresponding pixel depth in bits per pixel.  Returns CAMWIRE_SUCCESS
               on success or CAMWIRE_FAILURE on failure. */
            int pixel_depth(const Camwire_pixel coding, int &depth);
            /*
              Initialize camera registers not already done by
              dc1394_video_set_framerate() or dc1394_format7_set_roi() and update
              their shadow state.  Note that the order of register writes may be
              significant for some cameras after power-up or reset/initilize.  */
            int set_non_dma_registers(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set);

    };

}

#endif
