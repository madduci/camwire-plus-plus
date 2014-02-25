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
            /* Debugging tool for developers.  Prints the contents of
               c_handle->userdata to stderr no matter what the state of the camera
               is.  Always returns CAMWIRE_SUCCESS. */
            int debug_print_status(const Camwire_bus_handle_ptr &c_handle);
            /* Sets the string to Camwire's version as a string such as
               "1.2.3", or empty if an internal error  happens).
               This function can be called at any time.
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on internal
               error. */
            int version(std::string &version_str);
            /* Reads camera initialization settings from the given infile, changing
               only the members of set that are present in the file.  The remaining
               members are left as they were, and are assumed to contain previous
               settings initialized before this function call.  The format of the
               infile is one line per setting, where each line contains a tag and
               value(s) separated by whitespace.  The tags are chosen from the names
               of the members of Camwire_state and may be abbreviated as long as no
               ambiguity results.  The value(s) is (are) the setting as understood
               by the corresponding camwire_get/set_...() functions.  Most settings
               have a single value, and some (like white balance) have two or more.
               Empty lines and lines starting with the '#' character in the file are
               ignored, as are unrecognized tags.  The function
               camwire_write_state_to_file() can provide an example of the file
               format.  If there are no errors, infile is at end-of-file on return.
               Physical cameras are unchanged (there is no camera handle in the
               argument list).  This function can be called at any time.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int read_state_from_file(const std::shared_ptr<FILE> &infile, Camwire_state_ptr &set);
            /* Writes camera initialization settings to the given outfile, in the
               format understood by camwire_read_state_from_file().  The file can
               subsequently be edited to change settings, or to remove lines of
               settings that one wants to leave unchanged with
               camwire_read_state_from_file().  outfile is at end-of-file on return.
               This function can be called at any time.  Returns CAMWIRE_SUCCESS on
               success or CAMWIRE_FAILURE on failure.*/
            int write_state_to_file(const std::shared_ptr<FILE> &outfile, const Camwire_state_ptr &set);
            /* Tries to flush num_to_flush buffered frames.  If num_flushed is not
               the null pointer, the actual number of frames flushed is returned in
               *num_flashed.  *num_flushed will be less than num_to_flush if there
               were fewer buffered frames available to flush than requested.
               *num_flushed will never be more than num_to_flush.  It is safe to
               make num_to_flush a larger number than the total number of buffered
               frames.  If buffer_lag is not the null pointer, the number of bus
               frame buffers by which we are behind after flushing is returned in
               *buffer_lag.  This is the same number as can be obtained from
               camwire_get_framebuffer_lag().  *buffer_lag is only valid if
               num_to_flush was 1 or more.  The historically older frames are
               flushed first.  Frames currently being transmitted by the camera are
               not affected.  Any frames accessed by camwire_point_next_frame() or
               camwire_point_next_frame_poll() should first be released with
               camwire_unpoint_frame() before flushing.  If you want to be sure of
               completely emptying all bus frame buffers, you should stop the
               camera, wait for more than one frame transmission period and then
               flush.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on
               failure. */
            int flush_framebuffers(const Camwire_bus_handle_ptr &c_handle, const int num_to_flush, int &num_flushed, int &buffer_lag);
            /* Waits until a frame has been received and copies it into the given
               buffer.  All 16-bit camwire images are in network (big-endian) byte
               order.  Note that this function may never return to the calling
               program if a frame does not become available, for example if the
               camera is not running or an external trigger does not arrive.  If
               more than one frame are buffered when this function is called then
               the earliest frame is copied.  If buffer_lag is not the null pointer,
               the number of bus frame buffers by which we are behind is returned in
               *buffer_lag.  This is the same number as can be obtained from
               camwire_get_framebuffer_lag().  The frame's number and time stamp are
               available afterwards from camwire_get_framenumber() and
               get_timestamp().  If speed is important then
               point_next_frame() or camwire_point_next_frame_poll() might
               be faster.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on
               failure. */
            int copy_next_frame(const Camwire_bus_handle_ptr &c_handle, void *buffer, int &buffer_lag);
            /* Sets the given buffer pointer buf_ptr to the next received frame
               buffer.  If a frame is ready it returns immediately, otherwise it
               waits until a frame has been received.  All 16-bit camwire images are
               in network (big-endian) byte order.  Note that this function may
               never return to the calling program if a frame does not become
               available, for example if the camera is not running or an external
               trigger does not arrive.  The frame buffer is locked, which prevents
               it from being overwritten by subsequent frames.  The calling program
               may freely read and write the buffer.  If more than one frames are
               buffered when this function is called then buf_ptr points to the
               earliest frame.  If buffer_lag is not the null pointer, the number of
               bus frame buffers by which we are behind is returned in *buffer_lag.
               This is the same number as can be obtained from
               camwire_get_framebuffer_lag().  The frame's number and time stamp are
               available afterwards from camwire_get_framenumber() and
               get_timestamp().  This function may be faster than
               copy_next_frame() because no copying is involved.  The
               downside is that the function camwire_unpoint_frame() must be called
               each time when done to release the frame buffer again.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int point_next_frame(const Camwire_bus_handle_ptr &c_handle, void **buf_ptr, int &buffer_lag);
            /* Sets the given buffer pointer buf_ptr to the next received frame
               buffer and returns immediately.  If no frame is ready it sets buf_ptr
               to the null pointer and returns 0 in *buffer_lag.  All 16-bit camwire
               images are in network (big-endian) byte order.  Although it may not
               always return a frame pointer, this function always returns to the
               calling program even if a frame does not become available.  Otherwise
               its behaviour is similar to camwire_point_next_frame().  Note that
               the values returned from camwire_get_framenumber() and
               get_timestamp() are not valid if no frame was returned.  Like
               point_next_frame(), the function camwire_unpoint_frame() must
               be called each time a frame is obtained to release the frame buffer
               again.  If no frame was obtained then camwire_unpoint_frame() should
               not be called.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE
               on failure. */
            int point_next_frame_poll(const Camwire_bus_handle_ptr &c_handle, void **buf_ptr, int &buffer_lag);
            /* Releases the bus frame buffer that was pointed to with the pointer
               access functions camwire_point_next_frame() or
               camwire_point_next_frame_poll(), so that it can be used again for
               receiving new image data.  Calls to pointer access functions and
               camwire_unpoint_frame() should be strictly interleaved, otherwise the
               next call to a pointer access function will fail.
               camwire_unpoint_frame() itself is however safe to call if no frame is
               locked in which case it has no effect.  Returns CAMWIRE_SUCCESS on
               success or CAMWIRE_FAILURE on failure. */
            int unpoint_frame(const Camwire_bus_handle_ptr &c_handle);
            /* Transforms cam_buf into lin_buf by inverse gamma correction assuming
               the Rec.709 specification.  This should only be used to linearize the
               response of images that were captured while gamma correction was
               enabled with camwire_set_gamma().  It should restore the image
               sensor's 10-bit (or so) linear dynamic range at the expense of some
               loss of resolution.  The size and pixel coding of the image pointed
               to by cam_buf must match the current setings of image size and pixel
               coding.  The current pixel coding should have 8 bits per component
               (see description of camwire_set_gamma()).  The resultant image
               pointed to by lin_buf must have the same image size, and an assumed
               pixel coding with 16 bits per component such as CAMWIRE_PIXEL_MONO16
               or CAMWIRE_PIXEL_RGB16.  The image in lin_buf therefore requires
               twice as much allocated memory as the one in cam_buf.  All 16-bit
               camwire images are in network (big-endian) byte order.  The max_val
               argument scales the output to give the wanted saturation value when
               the input value is 255.  Typical values for max_val are 1023, 4095 or
               65535.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on
               failure. */
            int inv_gamma(const Camwire_bus_handle_ptr &c_handle, const void *cam_buf, void *lin_buf, const unsigned long max_val);
            /* Sets the camera run status in runsts: 1 for running or 0 for stopped.
               If a stopped camera is set to running while the acquisition type (as
               set by camwire_set_single_shot()) is single-shot, then only one frame
               is transmitted (pending an external trigger if used) after which the
               camera reverts to the stopped state.  Otherwise, frames are
               transmitted from camera to bus frame buffers at the programmed frame
               rate until the camera is stopped.  Returns CAMWIRE_SUCCESS on success
               or CAMWIRE_FAILURE on failure. */
            int set_run_stop(const Camwire_bus_handle_ptr &c_handle, const int runsts = 0);
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
            /* Sets the camera's gamma setting in gamma_on: 1 for gamma-corrected or
               0 for linear pixel values.  Camwire assumes that gamma correction
               non-linearly compresses the camera's 10 or 12-bit internal data path
               into 8 bits per pixel component, according to the Rec.709
               specification.  To use gamma, the current pixel coding should
               therefore have 8 bits per component (such as CAMWIRE_PIXEL_MONO8,
               CAMWIRE_PIXEL_YUV422 or CAMWIRE_PIXEL_RGB8, but not
               CAMWIRE_PIXEL_MONO16 etc.).  So far Camwire supports gamma in AVT
               cameras only.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE
               on failure or if the camera is not capable of gamma correction or if
               gamma correction is not supported for the current pixel coding. */
            int set_gamma(const Camwire_bus_handle_ptr &c_handle, const int gamma_on);
            /* Sets the camera's acquisition type in single_shot_on: 1 for
               single-shot or 0 for continuous.  To capture a single frame, make
               sure that the camera is stopped, set the acquisition type to
               single-shot, and set the camera running (with camwire_set_run_stop())
               at the moment the frame is needed.  If the camera is already running
               while changing acquisition type from continous to single-shot, then
               the single frame is acquired immediately (pending an external trigger
               if used).  Changing acquisition type from single-shot to continuous
               while the camera is running is an unreliable thing to do, because you
               can't be sure if the single frame acquisition has started and hence
               you don't know whether the camera is stopped or running.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int set_single_shot(const Camwire_bus_handle_ptr &c_handle, const int &single_shot_on);
            /* Sets the number of frame reception buffers allocated for the camera
               bus in the host computer.  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure. */
            int set_num_framebuffers(const Camwire_bus_handle_ptr &c_handle, const int &num_frame_buffers);
            /* Sets the frame size (width, height) in units of pixels to the nearest
               values valid for the camera.  The sizes available may be constrained
               by the maximum available frame size and the current frame offsets.
               The actual size set can be checked afterwards with
               camwire_get_frame_size().  For some camera buses like IEEE 1394, the
               frame rate may also change, especially with small frame sizes.  Check
               afterwards with camwire_get_framerate().  Returns CAMWIRE_SUCCESS on
               success or CAMWIRE_FAILURE on failure. */
            int set_frame_size(const Camwire_bus_handle_ptr &c_handle, const int width, const int height);
            /* Sets the pixel coding, given one of the Camwire_pixel enumeration
               members above.  For some camera buses like IEEE 1394, the frame rate
               may also change, especially with small frame sizes.  Check afterwards
               with camwire_get_framerate().  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure or if gamma is switched on and the new
               coding does not support gamma correction. */
            int set_pixel_coding(const Camwire_bus_handle_ptr &c_handle, const Camwire_pixel coding);
            /* Gets the camera's current settings (running/stopped, trigger source,
               frame rate, frame size, etc).  If the camera has not been created,
               the camera is physically reset to factory default settings and those
               are probed and returned (without creating the camera).  If the camera
               has been created and the state shadow flag is set, the shadow state
               settings are returned, else the camera is queried for current
               settings.  If the camera does not support some settings their
               returned values are undefined.  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure.*/
            int get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);
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
            /* Gets the camera's colour correction setting corr_on, 1 for
               colour-corrected or 0 for no correction or if the camera is not
               capable of colour correction.  So far Camwire supports colour
               correction in AVT cameras only.  Returns CAMWIRE_SUCCESS on success
               or CAMWIRE_FAILURE on failure. */
            int get_colour_correction(const Camwire_bus_handle_ptr &c_handle, int &corr_on);
            /* Gets the camera's colour correction coefficients.  coef is an array
               of 9 colour correction coefficients, see the description of the
               colour_coef member of Camwire_state above.  If the camera is not
               capable of colour correction, coef forms the 3x3 identity matrix.
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_colour_coefficients(const Camwire_bus_handle_ptr &c_handle, double coef[9]);
            /* Gets the camera's gamma setting in gamma_on: 1 for gamma-corrected or
               0 for linear pixel values or if the camera is not capable of gamma
               correction.  So far Camwire supports gamma in AVT cameras only.
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_gamma(const Camwire_bus_handle_ptr &c_handle, int &gamma_on);
            /* Gets the white balance levelss (blue or U, red or V) for colour
               cameras.  bal contains relative numbers between 0.0 and 1.0,
               corresponding respectively to the minimum and maximum levels
               implemented by the camera.  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure. */
            int get_white_balance(const Camwire_bus_handle_ptr &c_handlee, double bal[2]);
            /* Gets the camera's acquisition type setting in single_shot_on: 1 for
               single-shot or 0 for continuous or if the camera is not capable of
               single-shot.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE
               on failure. */
            int get_single_shot(const Camwire_bus_handle_ptr &c_handle, int &single_shot_on);
            /* Gets the camera run status in runsts: 1 for running or 0 for stopped.
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_run_stop(const Camwire_bus_handle_ptr &c_handle, int &runsts);
            /* Gets the relative gain factor (response slope) of the camera, as a
               number between 0.0 (minimum) and 1.0 (maximum).  The user has to
               interpret the meaning of this relative gain in terms of the
               individual camera's specification of its minimum and maximum settable
               slope.  For example, if the camera specifies the minimum gain as a
               slope of 1.0 (0dB) and the maximum as 4.0 (12dB), then the actual
               gain is (4.0 - 1.0)*gain + 1.0.  Returns CAMWIRE_SUCCESS on success
               or CAMWIRE_FAILURE on failure. */
            int get_gain(const Camwire_bus_handle_ptr &c_handle, double &gain);
            /* Gets the relative brightness (black level) of the camera, as a number
               between -1.0 (minimum) and +1.0 (maximum).  The user has to interpret
               the meaning of this relative brightness in terms of the individual
               camera's specification of its minimum and maximum settable
               brightness.  For example, if the camera specifies the minimum
               brightness level as 0 and the maximum as 1023, then the actual
               brightness level is (1023 - 0)*(brightness + 1.0)/2.0.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_brightness(const Camwire_bus_handle_ptr &c_handle, double &brightness);
            /* Gets the camera's trigger polarity setting: 1 for rising edge (active
               high) or 0 for falling edge (active low).  Returns CAMWIRE_SUCCESS on
               success or CAMWIRE_FAILURE on failure. */
            int get_trigger_polarity(const Camwire_bus_handle_ptr &c_handle, int &rising);
            /* Gets the camera's trigger source setting: 1 for external or 0 for
               internal.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on
               failure. */
            int get_trigger_source(const Camwire_bus_handle_ptr &c_handle, int &external);
            /* Gets the camera's shutter speed (exposure time in seconds).  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_shutter(const Camwire_bus_handle_ptr &c_handle, double &shutter);
            /* Gets the video frame rate in frames per second.  See the description
               of the meaning of 'frame rate' under camwire_set_framerate().
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_framerate(const Camwire_bus_handle_ptr &c_handle, double &frame_rate);
            /* Gets the pixel tiling, as one of the Camwire_tiling enumeration
               members above.  The tiling is not a typical camera setting because it
               is not settable and hence there is no camwire_set_pixel_tiling()
               function.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on
               failure. */
            int get_pixel_tiling(const Camwire_bus_handle_ptr &c_handle, Camwire_tiling &tiling);
            /* Gets the pixel coding, as one of the Camwire_pixel enumeration
               members above.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE
               on failure. */
            int get_pixel_coding(const Camwire_bus_handle_ptr &c_handle, Camwire_pixel &coding);
            /* Gets the frame size (width, height) in units of pixels.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_frame_size(const Camwire_bus_handle_ptr &c_handle, int &width, int &height);
            /* Gets the frame offsets (left, top) in units of pixels.  Returns
               CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_frame_offset(const Camwire_bus_handle_ptr &c_handle, int &left, int &top);
            /* Warning: The use of this function is deprecated because it creates
               the impression that it returns a currently fresh measure of the
               buffer lag.  Rather use the buffer_lag returned by calls to
               copy_next_frame(), point_next_frame(),
               point_next_frame_poll(), and calls to
               flush_framebuffers() with argument num_to_flush >= 1.
               Ideally it should get the number of bus frame buffers which have been
               filled by the camera but not yet accessed or, in other words, the
               number of frames by which we are behind.  In the current
               implementation this number is only updated by the calls listed above
               and should otherwise be considered stale.  Returns CAMWIRE_SUCCESS on
               success or CAMWIRE_FAILURE on failure. */
            int get_framebuffer_lag(const Camwire_bus_handle_ptr &c_handle, int &buffer_lag);
            /* Gets the state shadow flag: 1 to get camera settings from an internal
               shadow structure or 0 to read them directly from the camera hardware.
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure.*/
            int get_stateshadow(const Camwire_bus_handle_ptr &c_handle, int &shadow);
            /* Gets the number of frame reception buffers allocated for the camera
               bus in the host computer.  Returns CAMWIRE_SUCCESS on success or
               CAMWIRE_FAILURE on failure. */
            int get_num_framebuffers(const Camwire_bus_handle_ptr &c_handle, int &num_frame_buffers);

        /* Set to protected in case of Subclassing */
        protected:
            /* Inverse gamma transform look-up table used by camwire_inv_gamma().
               Assumed to be initialized when Camwire_user_data.gamma_maxval
               is non-zero: */
            uint16_t gamma_lut[256];
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
              Returns the AVT signed 32-bit int register values corresponding to the
              9 given colour correction coefficients.
            */
            void convert_colourcoefs2avtvalues(const double coef[9], int32_t val[9]);
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
            /*
              Returns the number of packets required to transmit a single frame, as
              obtained from the camera.
            */
            int get_numpackets(const Camwire_bus_handle_ptr &c_handle, u_int32_t &num_p);
            /*
              Returns a pointer to the dc1394video_frame_t structure for the given
              camwire handle, or 0 on error.  Needed by many libdc1394
              functions.
            */
            int get_captureframe(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394video_frame_t> &frame);
            /*
              Returns the number of bits per component in the given pixel coding.
            */
            int component_depth(const Camwire_pixel coding);
    };

}

#endif
