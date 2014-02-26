#include <dc1394/vendor/avt.h>
#include <camwire_config.hpp>
#include <camwire.hpp>
#include <cstring>
#include <unistd.h>         //sleep function
#include <cmath>            //log function
#include <cfloat>           //definition of DBL_MAX
#include <netinet/in.h>     //htons on Linux
#include <climits>          //definition of INT_MAX
#include <sstream>         //stringstream

camwire::camwire::camwire()
{

}

camwire::camwire::~camwire()
{

}

camwire::camwire::camwire(const camwire &cam)
{

}

camwire::camwire &camwire::camwire::operator=(const camwire &cam)
{
    return *this;
}


bool camwire::camwire::getenv(const char *name, std::string &env)
{
    const char *ret = std::getenv(name);
    if (ret) env = std::string(ret);
    return !!ret;
}

int camwire::camwire::create(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        /* Allocate zero-filled space for internal status, and register a
                   pointer to it in the camera handle: */
        if(!c_handle->userdata)
            c_handle->userdata.reset(new Camwire_user_data);
        User_handle internal_status = c_handle->userdata; //std::shared_ptr<Camwire_user_data>(new Camwire_user_data);
        ERROR_IF_NULL(internal_status); 	/* Allocation failure.*/
        Camwire_conf_ptr config(new Camwire_conf);
        ERROR_IF_NULL(config);

        /* Camwire_user_data is defined above.*/
        /*if (c_handle->handle_set_userdata(internal_status) == CAMWIRE_FAILURE)
        {   /* Already in use.*/
         /*   DPRINTF("camwire_bus_set_userdata() failed.");
            return CAMWIRE_FAILURE;
        }
        */
        /* Allocate zero-filled space for the extra features: */
        internal_status->extras.reset(new Extra_features);
        if (!internal_status->extras)
        { 	/* Allocation failure.*/
            DPRINTF("initializing (Extra_features) failed.");
            return CAMWIRE_FAILURE;
        }

        /* Allocate zero-filled space for the config cache: */

        /*Actually, the following commented lines will cause a problem in
        read_conf_file() which checks the config_cache and since it is already
        existing, it thinks it can copy that one instead of the data coming from file

        Need to be reorganized

        */
        /*internal_status->config_cache.reset(new Camwire_conf);
        if (!internal_status->config_cache)
        {
            DPRINTF("initializing (Camwire_conf) failed."); // Allocation failure.
            free_internals(c_handle);
            return CAMWIRE_FAILURE;
        }*/

        /* Allocate space and make a copy of the initial settings: */
        internal_status->current_set.reset(new Camwire_state);
        if (!internal_status->current_set)
        { 	/* Allocation failure.*/
            DPRINTF("initializing (Camwire_state) failed.");
            free_internals(c_handle);
            return CAMWIRE_FAILURE;
        }

        internal_status->current_set = set;
        /* Get 1394-specific hardware configuration: */
        if (get_config(c_handle, config) != CAMWIRE_SUCCESS)
        {
            DPRINTF("camwire_get_config() failed.");
            free_internals(c_handle);
            return CAMWIRE_FAILURE;
        }
        /* Make sure the video1394 device is not already listening on this
           channel (due to a provious process being killed without resetting
           the camera).  Hopefully this won't hurt anything: */
        dc1394_capture_stop(c_handle->camera.get());
        dc1394_iso_release_all(c_handle->camera.get());

        /* Connect the camera to the bus and initialize it with our
           settings: */
        DPRINTF("Connecting camera");
        if (connect_cam(c_handle, config, set) != CAMWIRE_SUCCESS)
        {
            DPRINTF("connect_cam() failed.");
            free_internals(c_handle);
            return CAMWIRE_FAILURE;
        }
        DPRINTF("Creation completed.");
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to init camera");
        return CAMWIRE_FAILURE;
    }
}

/* Queries the camera and attempts to create a sensible default
   configuration.  Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE
   on failure.  */
/* The IEEE 1394 IIDC digital camera (DCAM) specification gives camera
   manufacturers a choice of several predefined resolutions called
   "formats":

   Format 0 is for 160x120 up to 640x480 (VGA).
   Format 1 is for 800x600 up to 1024x768 (SVGA).
   Format 2 is for 1280x960 up to 1600x1200 (XVGA).
   Format 6 is "memory channels" or still images.
   Format 7 is for scalable image sizes. */
int camwire::camwire::generate_default_config(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg)
{
    try
    {
        dc1394video_mode_t video_mode;
        dc1394video_modes_t mode_list;
        /* Initialize the camera to factory settings: */
        ERROR_IF_NULL(c_handle);
        /* dc1394_camera_reset() does not work on all cameras, so we are
           lenient on the test: */
        int dc1394_return = dc1394_camera_reset(c_handle->camera.get());
        if (dc1394_return != DC1394_SUCCESS)
        {
            DPRINTF("Warning: dc1394_camera_reset() failed.  Continuing configuration, "
                "but camera may not be properly initialized.");
            sleep(1);  /* Increase chances that camera may recover.*/
        }

        /* video_mode = get_1394_video_mode(c_handle); */

        /* Determine the highest supported format and mode: */
        ERROR_IF_DC1394_FAIL(dc1394_video_get_supported_modes(c_handle->camera.get(), &mode_list));
        if (mode_list.num == 0)
        {
            DPRINTF("dc1394_video_get_supported_modes() returned an empty list.");
            return CAMWIRE_FAILURE;
        }

        video_mode = mode_list.modes[mode_list.num-1];  /* Highest format and mode. */
        convert_dc1394video_mode2format_mode(video_mode, cfg->format, cfg->mode);

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

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to generate default config.");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::generate_default_settings(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set)
{
    try
    {
        /* Initialize the camera to factory settings: */
        ERROR_IF_NULL(c_handle);
        /* dc1394_camera_reset() does not work on all cameras, so we are
           lenient on the test: */
        int dc1394_return = dc1394_camera_reset(c_handle->camera.get());
        if (dc1394_return != DC1394_SUCCESS)
        {
            DPRINTF("Warning: dc1394_camera_reset() failed.  Continuing configuration, "
            "but camera may not be properly initialized.");
            sleep(1);  /* Increase chances that camera may recover.*/
        }

        dc1394video_mode_t video_mode = get_1394_video_mode(c_handle);
        dc1394color_coding_t color_id;
        dc1394framerates_t supported_fr;
        std::shared_ptr<dc1394feature_info_t> capability(new dc1394feature_info_t);
        dc1394bool_t on_off;
        uint32_t num_packets = 0;
        /* Merging the functions, reducing the number of checks on variables */
        /* Format and mode-specific frame dimensions and pixel coding: */
        if(fixed_image_size(video_mode))    /* Formats 0-2.*/
        {
            set->left = 0;  /* pixels */
            set->top = 0;   /* pixels */
            ERROR_IF_DC1394_FAIL(
                        dc1394_get_image_size_from_video_mode(c_handle->camera.get(),
                                                             video_mode,
                                                             (uint32_t *)&set->width,
                                                             (uint32_t *)&set->height));
            set->coding = convert_videomode2pixelcoding(video_mode);

            /* Determine the maximum supported framerate in this mode and
               format: */
            ERROR_IF_DC1394_FAIL(
                    dc1394_video_get_supported_framerates(c_handle->camera.get(),
                                      video_mode,
                                      &supported_fr));
            if(supported_fr.num == 0)
            {
                DPRINTF("dc1394_video_get_supported_framerates() returned no "
                            "framerates.");
                return CAMWIRE_FAILURE;
            }

            set->frame_rate = 0;
            double fr = 0.0f;
            for (int f=0; f < supported_fr.num; ++f)
            {
                fr = convert_index2framerate(supported_fr.framerates[f]);
                if (fr < 0.0f)
                {
                    DPRINTF("convert_index2framerate() failed.");
                    return CAMWIRE_FAILURE; 	/* Invalid index.*/
                }
                if (fr > set->frame_rate)  set->frame_rate = fr;
            }
        }
        else if(variable_image_size(video_mode)) /* Format 7.*/
        {
            ERROR_IF_DC1394_FAIL(
               dc1394_format7_get_image_position(c_handle->camera.get(),
                                video_mode,
                                reinterpret_cast<uint32_t *>(&set->left),
                                reinterpret_cast<uint32_t *>(&set->top)));
            ERROR_IF_DC1394_FAIL(
               dc1394_format7_get_image_size(c_handle->camera.get(),
                                video_mode,
                                reinterpret_cast<uint32_t *>(&set->width),
                                reinterpret_cast<uint32_t *>(&set->height)));
            ERROR_IF_DC1394_FAIL(
               dc1394_format7_get_color_coding(
                c_handle->camera.get(),
                video_mode,
                &color_id));
            set->coding = convert_colorid2pixelcoding(color_id);

            /* Determine the maximum supported framerate in this mode and
               format: */

            /* PACKET_PER_FRAME_INQ depends on BYTE_PER_PACKET which in turn
               depends on IMAGE_SIZE and COLOR_CODING_ID.  Since we are not
               changing these registers, it is safe to use the value
               returned by get_num_packets(): */
            ERROR_IF_CAMWIRE_FAIL(get_numpackets(c_handle, num_packets));
            set->frame_rate = convert_numpackets2framerate(c_handle, num_packets);
        }
        else
        {
            DPRINTF("Camera's format is not supported");
            return CAMWIRE_FAILURE;
        }

        /* Get the shutter speed and try to fit it into one frame period: */
        if(set->frame_rate != 0)
            set->shutter = 0.5/set->frame_rate; /* Seconds, default.*/

        capability->id = DC1394_FEATURE_SHUTTER;
        ERROR_IF_DC1394_FAIL(dc1394_feature_get(c_handle->camera.get(), capability.get()));

        double max_shutter, min_shutter;
        Camwire_conf_ptr config(new Camwire_conf);
        if(feature_is_usable(capability))
        {
            ERROR_IF_CAMWIRE_FAIL(get_config(c_handle, config));
            set->shutter = config->exposure_offset + capability->value * config->exposure_quantum;
            max_shutter = config->exposure_quantum * ((unsigned long int)(1.0 / (set->frame_rate * config->exposure_quantum)));

            if(set->shutter > max_shutter)
                set->shutter = max_shutter;

            min_shutter = config->exposure_offset + capability->min * config->exposure_quantum;

            if(set->shutter < min_shutter)
                set->shutter = min_shutter;
        }
        else
            DPRINTF("Camera reported no usable shutter");

        /* Format and mode-independent settings: */
        set->external_trigger = 0;  /* Flag */
        set->trigger_polarity = 1;  /* Flag, default */
        capability->id = DC1394_FEATURE_TRIGGER;
        ERROR_IF_DC1394_FAIL(dc1394_feature_get(c_handle->camera.get(), capability.get()));
        if (feature_is_usable(capability))
        {
            if (capability->trigger_polarity == DC1394_TRIGGER_ACTIVE_LOW)
                set->trigger_polarity = 0;
            else
                set->trigger_polarity = 1;
        }

        /* Get the factory gain and set our normalized gain accordingly: */
        set->gain = 0.0;			/* Units, default.*/
        capability->id = DC1394_FEATURE_GAIN;
        ERROR_IF_DC1394_FAIL(dc1394_feature_get(c_handle->camera.get(), capability.get()));
        if (feature_is_usable(capability))
        {
            if (capability->max != capability->min)
                set->gain = (double)(capability->value - capability->min) /
                (capability->max - capability->min);
        }
        else
            DPRINTF("Camera reported no usable gain.");

        /* Get the factory brightness and set our normalized brightness
           accordingly: */
        set->brightness = 0.0;		/* Units, default.*/
        capability->id = DC1394_FEATURE_BRIGHTNESS;
        ERROR_IF_DC1394_FAIL(dc1394_feature_get(c_handle->camera.get(), capability.get()));
        if (feature_is_usable(capability))
        {
            set->brightness = 2.0*(double)(capability->value - capability->min) /
                (capability->max - capability->min) - 1.0;
        }
        else
            DPRINTF("Camera reported no usable brightness.");

        /* Get the factory white balance and set our normalized levels
           accordingly: */
        set->white_balance[0] = set->white_balance[1] = 0.0; 	/* Units, default.*/
        capability->id = DC1394_FEATURE_WHITE_BALANCE;
        ERROR_IF_DC1394_FAIL(dc1394_feature_get(c_handle->camera.get(), capability.get()));
        if (feature_is_usable(capability))
        {
            if (capability->max != capability->min)
            {
                set->white_balance[0] = (double)(capability->BU_value - capability->min) /
                (capability->max - capability->min);
                set->white_balance[1] = (double)(capability->RV_value - capability->min) /
                (capability->max - capability->min);
            }
        }
        else
            DPRINTF("Camera reported no usable white balance.");

        /* Enable colour correction by default if the camera supports it,
           and get the factory colour correction coefficients: */
        int32_t coef_reg[9];
        set->colour_corr = probe_camera_colour_correction(c_handle);
        if (set->colour_corr) 		/* Flag, on by default.*/
        {
        ERROR_IF_DC1394_FAIL(dc1394_avt_get_color_corr(c_handle->camera.get(),
                          &on_off,
                          &coef_reg[0], &coef_reg[1], &coef_reg[2],
                          &coef_reg[3], &coef_reg[4], &coef_reg[5],
                          &coef_reg[6], &coef_reg[7], &coef_reg[8]));
            convert_avtvalues2colourcoefs(coef_reg, set->colour_coef);
        }
        else
            DPRINTF("Camera reported no usable colour correction.");

        /* Enable gamma if the camera supports it: */
        set->gamma = probe_camera_gamma(c_handle);  /* Flag.*/
        if (!set->gamma)
            DPRINTF("Camera reported no usable gamma correction.");


        /* Other defaults: */
        set->tiling = probe_camera_tiling(c_handle);  /* Pattern.*/
        set->num_frame_buffers = 10;        /* Frames.*/
        set->single_shot = 0;               /* Flag.*/
        set->running = 0;                   /* Flag.*/
        set->shadow = 1;                    /* Flag.*/

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to generate default settings");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_current_settings(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set)
{
    try
    {
        Camwire_state_ptr shadow_state(new Camwire_state);
        dc1394bool_t one_shot_set;

        //*shadow_state = get_shadow_state(c_handle);
        ERROR_IF_NULL(shadow_state);
        if(shadow_state->shadow)
        {
            set = shadow_state;
            /* One_Shot register self-clears after transmission, hence we
               don't know if camera is still runnning: */
            if (shadow_state->running && shadow_state->single_shot)
            {
                ERROR_IF_DC1394_FAIL(dc1394_video_get_one_shot(c_handle->camera.get(), &one_shot_set));
                if (one_shot_set == DC1394_FALSE)
                    set->running = shadow_state->running = 0;
            }
        }
        else
        {

            ERROR_IF_CAMWIRE_FAIL(get_num_framebuffers(c_handle, set->num_frame_buffers));

            ERROR_IF_CAMWIRE_FAIL(get_gain(c_handle, set->gain));

            ERROR_IF_CAMWIRE_FAIL(get_brightness(c_handle, set->brightness));

            ERROR_IF_CAMWIRE_FAIL(get_white_balance(c_handle, set->white_balance));

            ERROR_IF_CAMWIRE_FAIL(get_gamma(c_handle, set->gamma));

            ERROR_IF_CAMWIRE_FAIL(get_colour_correction(c_handle, set->colour_corr));

            ERROR_IF_CAMWIRE_FAIL(get_colour_coefficients(c_handle, set->colour_coef));

            ERROR_IF_CAMWIRE_FAIL(get_frame_offset(c_handle, set->left, set->top));

            ERROR_IF_CAMWIRE_FAIL(get_frame_size(c_handle, set->width, set->height));

            ERROR_IF_CAMWIRE_FAIL(get_pixel_coding(c_handle, set->coding));

            ERROR_IF_CAMWIRE_FAIL(get_pixel_tiling(c_handle, set->tiling));

            ERROR_IF_CAMWIRE_FAIL(get_framerate(c_handle, set->frame_rate));

            ERROR_IF_CAMWIRE_FAIL(get_shutter(c_handle, set->shutter));

            ERROR_IF_CAMWIRE_FAIL(get_trigger_source(c_handle, set->external_trigger));

            ERROR_IF_CAMWIRE_FAIL(get_trigger_polarity(c_handle, set->trigger_polarity));

            ERROR_IF_CAMWIRE_FAIL(get_single_shot(c_handle, set->single_shot));

            ERROR_IF_CAMWIRE_FAIL(get_run_stop(c_handle, set->running));

            ERROR_IF_CAMWIRE_FAIL(get_stateshadow(c_handle, set->shadow));
        }

        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve current settings");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_shadow_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set)
{
    try
    {
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);
        set = internal_status->current_set;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get shadow state");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::sleep_frametime(const Camwire_bus_handle_ptr &c_handle, const double multiple)
{
    double frame_rate = 0.0f;
    double sleep_period = 0.0f;
    struct timespec nap, left;

    try
    {
        //ERROR_IF_CAMWIRE_FAIL(camwire_get_framerate(c_handle, &frame_rate));
        if(frame_rate != 0.0f)                  /* Avoiding division by 0 */
            sleep_period = multiple/frame_rate;
        nap.tv_sec = (time_t) sleep_period; 	/* Trunc. to integer.*/
        nap.tv_nsec = (long)((sleep_period - nap.tv_sec)*1e9);
        if (nanosleep(&nap, &left) != 0)
        {
            DPRINTF("nanosleep() failed.");
            return CAMWIRE_FAILURE;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set sleep time");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::connect_cam(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg, const Camwire_state_ptr &set)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);
        /* If dc1394_capture_stop() is called without a preceding
           successful call to dc1394_capture_setup(), libdc1394 used to get
           into a tangled state.  That is why we keep track with the
           camera_connected flag, and check it in disconnect_cam(): */
        internal_status->camera_connected = 0;

        /* Set 1394B mode if it is available.  Don't check the return
            status, because older cameras won't support it: */
        dc1394_video_set_operation_mode(c_handle->camera.get(), DC1394_OPERATION_MODE_1394B);
        dc1394video_mode_t video_mode = convert_format_mode2dc1394video_mode(cfg->format, cfg->mode);
        ERROR_IF_ZERO(video_mode);

        dc1394framerates_t framerate_list;
        dc1394framerate_t  frame_rate_index;
        dc1394color_codings_t coding_list;
        dc1394color_coding_t  color_id;
        Camwire_pixel actual_coding;
        double actual_frame_rate = 0.0f;
        uint32_t num_packets, packet_size;
        int depth = 0;
        if(fixed_image_size(video_mode))    /* Format 0, 1 or 2 */
        {
            ERROR_IF_DC1394_FAIL(dc1394_video_get_supported_framerates(c_handle->camera.get(), video_mode, &framerate_list));
            if(framerate_list.num == 0)
            {
                DPRINTF("dc1394_video_get_supported_framerates returned an empty list");
                return CAMWIRE_FAILURE;
            }
            frame_rate_index = static_cast<dc1394framerate_t>(convert_framerate2index(set->frame_rate, framerate_list));
            ERROR_IF_ZERO(frame_rate_index);
            /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed, unless there is an entry in the .conf file: */
            ERROR_IF_DC1394_FAIL(
                dc1394_video_set_iso_speed(c_handle->camera.get(),
                               static_cast<dc1394speed_t>(convert_busspeed2dc1394(cfg->bus_speed))));
            ERROR_IF_DC1394_FAIL(
                dc1394_video_set_mode(c_handle->camera.get(),
                          video_mode));
            ERROR_IF_DC1394_FAIL(
                dc1394_video_set_framerate(c_handle->camera.get(),
                               frame_rate_index));
            ERROR_IF_DC1394_FAIL(
                    dc1394_capture_setup(c_handle->camera.get(),
                         set->num_frame_buffers,
                         DC1394_CAPTURE_FLAGS_DEFAULT));
            internal_status->num_dma_buffers = set->num_frame_buffers;
            actual_coding = convert_videomode2pixelcoding(video_mode);
            actual_frame_rate = convert_index2framerate(frame_rate_index);
        }
        else if (variable_image_size(video_mode))   /* Format 7 */
        {
            /* Prevent a segfault due to kalloc() bug in dma.c of the
               linux1394 system.  This ought to be removed for later
               versions: */
            ERROR_IF_CAMWIRE_FAIL(pixel_depth(set->coding, depth));
            /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed, unless there is an entry in the .conf file: */
            ERROR_IF_DC1394_FAIL(dc1394_video_set_iso_speed(c_handle->camera.get(), static_cast<dc1394speed_t>(convert_busspeed2dc1394(cfg->bus_speed))));
            ERROR_IF_DC1394_FAIL(dc1394_video_set_mode(c_handle->camera.get(), video_mode));

            /* Set up the color_coding_id before calling
               dc1394_capture_setup(), otherwise the wrong DMA buffer size
               may be allocated: */
            ERROR_IF_DC1394_FAIL(
                dc1394_format7_get_color_codings(c_handle->camera.get(),
                                 video_mode,
                                 &coding_list));

            if(coding_list.num == 0)
            {
                DPRINTF("dc1394_format7_get_color_codings returned an empty list");
                return CAMWIRE_FAILURE;
            }

            color_id = static_cast<dc1394color_coding_t>(convert_pixelcoding2colorid(set->coding, coding_list));
            if (color_id == 0)
            {
                DPRINTF("Pixel colour coding is invalid or not supported by the camera.");
                return CAMWIRE_FAILURE;
            }
            ERROR_IF_DC1394_FAIL(
                dc1394_format7_set_color_coding(
                c_handle->camera.get(),
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
                c_handle->camera.get(),
                video_mode,
                set->left, set->top));  /* So that _image_size() doesn't fail.*/
            ERROR_IF_DC1394_FAIL(
                dc1394_format7_set_image_size(
                c_handle->camera.get(),
                video_mode,
                set->width, set->height));  /* PACKET_PARA_INQ is now valid. */

            num_packets = convert_framerate2numpackets(c_handle, set->frame_rate);

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
                c_handle->camera.get(),
                video_mode,
                packet_size));
            ERROR_IF_DC1394_FAIL(
                dc1394_capture_setup(c_handle->camera.get(),
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
            DPRINTF("Unsupported Camera format.");
            return CAMWIRE_FAILURE;
        }

        internal_status->camera_connected = 1;
        internal_status->frame = 0;
        internal_status->frame_lock = 0;
        internal_status->num_dma_buffers = set->num_frame_buffers;
        /* Find out camera capabilities (which should only be done after
           setting up the format and mode above): */
        Camera_handle dc1394_camera = c_handle->camera;
        internal_status->extras->single_shot_capable = (dc1394_camera->one_shot_capable != DC1394_FALSE ? 1 : 0);
        internal_status->extras->gamma_capable = probe_camera_gamma(c_handle);
        internal_status->extras->colour_corr_capable = probe_camera_colour_correction(c_handle);
        internal_status->extras->tiling_value = probe_camera_tiling(c_handle);
        ERROR_IF_DC1394_FAIL(dc1394_feature_get_all(c_handle->camera.get(), &internal_status->feature_set));
        /* Update DMA-affected shadow states not done in
           set_non_dma_registers() calls below: */
        Camwire_state_ptr shadow_state(new Camwire_state);
        if(get_shadow_state(c_handle, shadow_state) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get shadow state");
            return CAMWIRE_FAILURE;
        }
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
        ERROR_IF_CAMWIRE_FAIL(set_non_dma_registers(c_handle, set));
        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to connect camera");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::reconnect_cam(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg, const Camwire_state_ptr &set)
{
    try
    {
        if (set->running)
        {
            ERROR_IF_CAMWIRE_FAIL(set_run_stop(c_handle, 0));
            ERROR_IF_CAMWIRE_FAIL(sleep_frametime(c_handle, 1.5));
        }
        disconnect_cam(c_handle);
        ERROR_IF_CAMWIRE_FAIL(connect_cam(c_handle, cfg, set));
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to reconnect camera.");
        return CAMWIRE_FAILURE;
    }
}

void camwire::camwire::disconnect_cam(const Camwire_bus_handle_ptr &c_handle)
{    
    try
    {
        User_handle internal_status = c_handle->userdata;
        if (internal_status)
        {
            if (internal_status->camera_connected)
            {
                if (internal_status->frame_lock)
                {
                    dc1394_capture_enqueue(c_handle->camera.get(), internal_status->frame.get());
                    internal_status->frame.reset();
                    internal_status->frame_lock = 0;
                }
                dc1394_capture_stop(c_handle->camera.get());
            }
            internal_status->camera_connected = 0;
        }
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to disconnect camera");
    }
}

void camwire::camwire::free_internals(const Camwire_bus_handle_ptr &c_handle)
{
    try
    {
        User_handle internal_status = c_handle->userdata;
        if (internal_status)
        {
            if (internal_status->frame_lock)
            {
                dc1394_capture_enqueue(c_handle->camera.get(),
                           internal_status->frame.get());
                internal_status->frame.reset();
                internal_status->frame_lock = 0;
            }
        }
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to free internal values");
    }
}

int camwire::camwire::config_cache_exists(const User_handle &internal_status)
{
    #ifdef CAMWIRE_DEBUG
    if (!internal_status)
    DPRINTF("Internal error: User_handle internal_status pointer is 0.");
    #endif
    if(internal_status->config_cache)
        return CAMWIRE_SUCCESS;
    else
        return CAMWIRE_FAILURE;
}

int camwire::camwire::find_conf_file(const Camwire_id &id, std::string &conffilenam)
{
    try
    {
        std::string env_directory("");
        if(open_named_conf_file("", id.chip, conffilenam) == CAMWIRE_SUCCESS)
            return CAMWIRE_SUCCESS;
         if(open_named_conf_file("", id.model, conffilenam) == CAMWIRE_SUCCESS)
            return CAMWIRE_SUCCESS;
        if(open_named_conf_file("", id.vendor, conffilenam) == CAMWIRE_SUCCESS)
            return CAMWIRE_SUCCESS;

        if(!getenv(ENVIRONMENT_VAR_CONF, env_directory))    //getenv throws if not set
        {
            DPRINTF("No environment variable set.");
        }
        else
        {
            if(env_directory.size() > 0)
            {
                if(open_named_conf_file(env_directory, id.chip, conffilenam) == CAMWIRE_SUCCESS)
                    return CAMWIRE_SUCCESS;
                if(open_named_conf_file(env_directory, id.model, conffilenam) == CAMWIRE_SUCCESS)
                    return CAMWIRE_SUCCESS;
                if(open_named_conf_file(env_directory, id.vendor, conffilenam) == CAMWIRE_SUCCESS)
                    return CAMWIRE_SUCCESS;
            }
        }

        DPRINTF("No configuration file found");
        return CAMWIRE_FAILURE;
    }
    catch(std::exception &re)
    {
        DPRINTF("Failed to find configuration file");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::open_named_conf_file(const std::string &path, const std::string &filename, std::string &conffilename)
{
    try
    {
        conffilename = "";
        if(path.length() > 0)
        {
            conffilename = path;
            if(conffilename[conffilename.length() - 1] != '/')
                conffilename += "/";
        }

        FILE *conffile;
        conffilename += filename + CONFFILE_EXTENSION;
        /* Check if previously pointing to other data and release it */
        if((conffile = fopen(conffilename.c_str(), "r")) != NULL)
        {
            fclose(conffile);
            return CAMWIRE_SUCCESS;
        }
        else
        {
            conffilename = "";
            return CAMWIRE_FAILURE;
        }
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to open configuration file");
        conffilename = "";
        return CAMWIRE_FAILURE;
    }
}

/* Keeping C-style I/O operations just for compatibility with Camwire original code */
/* This will be in future converted into C++ style, using fstream: it's cleaner */
int camwire::camwire::read_conf_file(FILE *conffile, Camwire_conf_ptr &cfg)
{
    int scan_result = 0, speed = 0, num_bits_set = 0;
    try
    {
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
               "  dma_device_name:     %[^\n]s",
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
               &cfg->drop_frames,
               &cfg->dma_device_name);

        if (scan_result == EOF || scan_result < 12)
        {
            DPRINTF("fscanf() failed reading configuration file.");
            return CAMWIRE_FAILURE;
        }

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
            return CAMWIRE_FAILURE;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to read configuration file");
        return CAMWIRE_FAILURE;
    }
}

/* Keeping C-style I/O operations just for compatibility with Camwire original code */
/* This will be in future converted into C++ style, using fstream: it's cleaner */
int camwire::camwire::write_config_to_file(FILE *outfile, const Camwire_conf_ptr &cfg)
{
    int print_result = 0;
    try
    {
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
                   cfg->dma_device_name.c_str());

        if (print_result < 1)
        {
            return CAMWIRE_FAILURE;
        }
        fflush(outfile);
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to write config file to disk");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::write_config_to_output(const Camwire_conf_ptr &cfg)
{
    try
    {
        std::cout <<
                   "Camwire IEEE 1394 IIDC DCAM hardware configuration:\n"
                   "  bus_speed:           "<< cfg->bus_speed << std::endl <<
                   "  format:              "<< cfg->format << std::endl <<
                   "  mode:                "<< cfg->mode << std::endl <<
                   "  max_packets:         "<< cfg->max_packets << std::endl <<
                   "  min_pixels:          "<< cfg->min_pixels << std::endl <<
                   "  trig_setup_time:     "<< cfg->trig_setup_time << std::endl <<
                   "  exposure_quantum:    "<< cfg->exposure_quantum << std::endl <<
                   "  exposure_offset:     "<< cfg->exposure_offset << std::endl <<
                   "  line_transfer_time:  "<< cfg->line_transfer_time << std::endl <<
                   "  transmit_setup_time: "<< cfg->transmit_setup_time << std::endl <<
                   "  transmit_overlap:    "<< cfg->transmit_overlap << std::endl <<
                   "  drop_frames:         "<< cfg->bus_speed << std::endl <<
                   "  dma_device_name:     "<< cfg->dma_device_name << std::endl << std::endl;

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to write config file to disk");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::fixed_image_size(const dc1394video_mode_t video_mode)
{
    return (dc1394_is_video_mode_scalable(video_mode) == DC1394_FALSE &&
            dc1394_is_video_mode_still_image(video_mode) == DC1394_FALSE);
}

int camwire::camwire::variable_image_size(const dc1394video_mode_t video_mode)
{
    return (dc1394_is_video_mode_scalable(video_mode) == DC1394_TRUE &&
            dc1394_is_video_mode_still_image(video_mode) == DC1394_FALSE);
}

double camwire::camwire::convert_index2framerate(const dc1394framerate_t frame_rate_index)
{
    int dc1394_return = 0;
    float frame_rate = 0.0;

    try
    {
        dc1394_return = dc1394_framerate_as_float(frame_rate_index, &frame_rate);
        if (dc1394_return == DC1394_SUCCESS)
            return (double)frame_rate;
        else
            return -1.0;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to convert index to framerate");
        return -1.0;
    }
}

int camwire::camwire::convert_framerate2index(const double frame_rate, const dc1394framerates_t &framerate_list)
{
    try
    {
        float min_fr;
        double fps, log2f, best, diff;
        int nearest_index, r, rate_index;

        dc1394_framerate_as_float(DC1394_FRAMERATE_MIN, &min_fr);  /* 1.875.*/
        if (frame_rate > 0.0)
            fps = frame_rate;
        else
            fps = min_fr;

        log2f = log(fps/min_fr)/log(2);  /* 1.875->0, 3.75->1, 7.5->2, etc.*/
        best = DBL_MAX;
        nearest_index = -1;
        for (r=0; r < framerate_list.num; ++r)
        {
            rate_index = framerate_list.framerates[r];
            diff = fabs(log2f - rate_index + DC1394_FRAMERATE_MIN);
            if (diff < best)
            {
                best = diff;
                nearest_index = rate_index;
            }
        }
        if (nearest_index >= 0)
            return nearest_index;
        else
            return 0;  /* Empty list?*/
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to convert framerate to index");
        return 0;
    }
}

void camwire::camwire::convert_avtvalues2colourcoefs(const int32_t val[9], double coef[9])
{
    try
    {
        for(int c = 0; c < 9; ++c)
            coef[c] = val[c]/1000.0;
    }
    catch(std::out_of_range &oor)
    {
        DPRINTF("Cannot convert avt values to colour coefficients");
    }
}

void camwire::camwire::convert_dc1394video_mode2format_mode(const dc1394video_mode_t video_mode, int &format, int &mode)
{
    int mode_offset;

    for (int fmt = 7; fmt >= 0; --fmt)
    {
        mode_offset = mode_dc1394_offset[fmt];
        if (mode_offset != 0 && video_mode >= mode_offset)
        {
            format = fmt;
            mode = video_mode - mode_offset;
            break;
        }
    }
}

camwire::Camwire_pixel camwire::camwire::convert_videomode2pixelcoding(const dc1394video_mode_t video_mode)
{
    switch (video_mode)
    {
        case DC1394_VIDEO_MODE_160x120_YUV444:
            return CAMWIRE_PIXEL_YUV444;  /* 24 bits/pixel.*/
            break;
        case DC1394_VIDEO_MODE_320x240_YUV422:
        case DC1394_VIDEO_MODE_640x480_YUV422:
        case DC1394_VIDEO_MODE_800x600_YUV422:
        case DC1394_VIDEO_MODE_1024x768_YUV422:
        case DC1394_VIDEO_MODE_1280x960_YUV422:
        case DC1394_VIDEO_MODE_1600x1200_YUV422:
            return CAMWIRE_PIXEL_YUV422;  /* 16 bits/pixel.*/
            break;
        case DC1394_VIDEO_MODE_640x480_YUV411:
            return CAMWIRE_PIXEL_YUV411;  /* 12 bits/pixel.*/
            break;
        case DC1394_VIDEO_MODE_640x480_RGB8:
        case DC1394_VIDEO_MODE_800x600_RGB8:
        case DC1394_VIDEO_MODE_1024x768_RGB8:
        case DC1394_VIDEO_MODE_1280x960_RGB8:
        case DC1394_VIDEO_MODE_1600x1200_RGB8:
            return CAMWIRE_PIXEL_RGB8;  /* 24 bits/pixel.*/
            break;
        case DC1394_VIDEO_MODE_640x480_MONO8:
        case DC1394_VIDEO_MODE_800x600_MONO8:
        case DC1394_VIDEO_MODE_1024x768_MONO8:
        case DC1394_VIDEO_MODE_1280x960_MONO8:
        case DC1394_VIDEO_MODE_1600x1200_MONO8:
            return CAMWIRE_PIXEL_MONO8;  /* 8 bits/pixel.*/
            break;
        case DC1394_VIDEO_MODE_640x480_MONO16:
        case DC1394_VIDEO_MODE_800x600_MONO16:
        case DC1394_VIDEO_MODE_1024x768_MONO16:
        case DC1394_VIDEO_MODE_1280x960_MONO16:
        case DC1394_VIDEO_MODE_1600x1200_MONO16:
            return CAMWIRE_PIXEL_MONO16;  /* 16 bits/pixel.*/
            break;
        default:
            return CAMWIRE_PIXEL_INVALID;  /* Unknown.*/
            break;
    }
}

camwire::Camwire_pixel camwire::camwire::convert_colorid2pixelcoding(const dc1394color_coding_t color_id)
{
    switch (color_id)
    {
        case DC1394_COLOR_CODING_MONO8:
            return CAMWIRE_PIXEL_MONO8;  /* 8 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_YUV411:
            return CAMWIRE_PIXEL_YUV411;  /* 12 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_YUV422:
            return CAMWIRE_PIXEL_YUV422;  /* 16 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_YUV444:
            return CAMWIRE_PIXEL_YUV444;  /* 24 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_RGB8:
            return CAMWIRE_PIXEL_RGB8;  /* 24 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_MONO16:
            return CAMWIRE_PIXEL_MONO16;  /* 16 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_RGB16:
            return CAMWIRE_PIXEL_RGB16;  /* 48 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_MONO16S:
            return CAMWIRE_PIXEL_MONO16S;  /* 16 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_RGB16S:
            return CAMWIRE_PIXEL_RGB16S;  /* 48 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_RAW8:
            return CAMWIRE_PIXEL_RAW8;  /* 8 bits/pixel.*/
            break;
        case DC1394_COLOR_CODING_RAW16:
            return CAMWIRE_PIXEL_RAW16;  /* 16 bits/pixel.*/
            break;
        default:
            return CAMWIRE_PIXEL_INVALID;  /* Not supported.*/
            break;
    }
}

camwire::Camwire_tiling camwire::camwire::convert_filterid2pixeltiling(const dc1394color_filter_t filter_id)
{
    switch (filter_id)
    {
        case DC1394_COLOR_FILTER_RGGB:
            return CAMWIRE_TILING_RGGB;
            break;
        case DC1394_COLOR_FILTER_GBRG:
            return CAMWIRE_TILING_GBRG;
            break;
        case DC1394_COLOR_FILTER_GRBG:
            return CAMWIRE_TILING_GRBG;
            break;
        case DC1394_COLOR_FILTER_BGGR:
            return CAMWIRE_TILING_BGGR;
            break;
        default:
            return CAMWIRE_TILING_INVALID;  /* Not supported.*/
            break;
    }
}

uint32_t camwire::camwire::convert_framerate2numpackets(const Camwire_bus_handle_ptr &c_handle, const double frame_rate)
{
    try
    {
        Camwire_conf_ptr config(new Camwire_conf);
        u_int32_t num_packets;

        if(get_config(c_handle, config) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get config.");
            return 0;
        }
        #ifdef CAMWIRE_DEBUG
            /* FIXME: Need a better way of checking cache initialization than bus_speed: */
            if (config->bus_speed == 0)
            {
                DPRINTF("get_config() returned null format.");
                return 0;
            }
        #endif

            if (frame_rate <= 0)  return config->max_packets;
            /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
            num_packets = static_cast<u_int32_t>(convert_busspeed2busfreq(config->bus_speed)/frame_rate + 0.5);
            if (num_packets < 1)
                num_packets = 1;
            if (num_packets > config->max_packets)
                num_packets = config->max_packets;
            return num_packets;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to convert framerate to number of packets");
        return 0;
    }
}

uint32_t camwire::camwire::convert_numpackets2packetsize(const Camwire_bus_handle_ptr &c_handle, const uint32_t num_packets, const int width, const int height, const Camwire_pixel coding)
{
    try
    {
        Camwire_conf_ptr config(new Camwire_conf);
        if(get_config(c_handle, config) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get configuration");
            return 0;
        }

        dc1394video_mode_t video_mode = get_1394_video_mode(c_handle);
        #ifdef CAMWIRE_DEBUG
            /* FIXME: Need a better way of checking cache initialization than bus_speed: */
            if (config->bus_speed == 0)
            {
                DPRINTF("get_config() returned null format.");
                return 0;
            }
            else if (!variable_image_size(video_mode))
            { 	/* Not Format 7.*/
                DPRINTF("Camera is not in Format 7.");
                return 0;
            }
            if (num_packets < 1 || num_packets > config->max_packets)
            {
                DPRINTF("Number of packets is out of range.");
                return 0;
            }
        #endif
        int depth = 0;
        if (pixel_depth(coding, depth) != CAMWIRE_SUCCESS)
        {
            DPRINTF("pixel_depth() failed.");
            return 0;
        }
        uint32_t packet_size;
        uint32_t unit_bytes, max_bytes;
        long denominator;
        /* Set unit_bytes quantum and max_bytes packet size, even if we
           cannot yet access the camera: */
        unit_bytes = max_bytes = 0;
        dc1394_format7_get_packet_parameters(c_handle->camera.get(),
                         video_mode,
                         &unit_bytes, &max_bytes);
        if (unit_bytes < 4)
            unit_bytes = 4; 	/* At least a quadlet.*/
        if (max_bytes < unit_bytes)
        /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
        max_bytes = 4 * (4915 * config->bus_speed/1600 - 3);
        /* Max 4915 quadlets in S1600, less 3 for header and data CRC. */

        /* packet_size = ceil(framesize/num_packets), quantized to unit_bytes: */
        denominator = static_cast<long>(unit_bytes * num_packets * 8);
        packet_size = ((static_cast<long>(width*height * depth + denominator - 1))/denominator) * unit_bytes;

        /* Check limits: */
        /* if (packet_size < unit_bytes)  packet_size = unit_bytes; */
        /* Testing (packet_size < unit_bytes) should not be necessary.*/
        if (packet_size > max_bytes)
            packet_size = max_bytes;

        return packet_size;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to convert numpackets to packet size");
        return 0;
    }
}

uint32_t camwire::camwire::convert_packetsize2numpackets(const Camwire_bus_handle_ptr &c_handle, const uint32_t packet_size, const int width, const int height, const Camwire_pixel coding)
{
    try
    {
        Camwire_conf_ptr config(new Camwire_conf);
        int depth = 0;
        /* num_packets = ceil(framesize/packet_size): */
        if (pixel_depth(coding, depth) != CAMWIRE_SUCCESS)
        {
            DPRINTF("pixel_depth() failed.");
            return 0;
        }

        uint32_t num_packets;
        long denominator = static_cast<long>(packet_size * 8);
        if(denominator > 0)
        {
            num_packets = (static_cast<long>(width * height * depth + denominator - 1)) / denominator;
            if(num_packets < 1)
                num_packets = 1;
        }
        else
        {
            if(get_config(c_handle, config) != CAMWIRE_SUCCESS)
            {
                DPRINTF("Failed to get configuration");
                return 0;
            }
            num_packets = config->max_packets;
        }

        return num_packets;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to convert packet size to numpackets");
        return 0;
    }
}

uint32_t camwire::camwire::convert_pixelcoding2colorid(const Camwire_pixel coding, const dc1394color_codings_t &coding_list)
{
    switch (coding)
    {
        case CAMWIRE_PIXEL_MONO8:  /* 8 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_MONO8))
            return DC1394_COLOR_CODING_MONO8;
            break;
        case CAMWIRE_PIXEL_YUV411:  /* 12 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_YUV411))
            return DC1394_COLOR_CODING_YUV411;
            break;
        case CAMWIRE_PIXEL_YUV422:  /* 16 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_YUV422))
            return DC1394_COLOR_CODING_YUV422;
            break;
        case CAMWIRE_PIXEL_YUV444:  /* 24 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_YUV444))
            return DC1394_COLOR_CODING_YUV444;
            break;
        case CAMWIRE_PIXEL_RGB8:  /* 24 bits/pixel.*/
            if (is_in_coding_list(coding_list,DC1394_COLOR_CODING_RGB8))
            return DC1394_COLOR_CODING_RGB8;
            break;
        case CAMWIRE_PIXEL_MONO16:  /* 16 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_MONO16))
            return DC1394_COLOR_CODING_MONO16;
            break;
        case CAMWIRE_PIXEL_RGB16:  /* 48 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_RGB16))
            return DC1394_COLOR_CODING_RGB16;
            break;
        case CAMWIRE_PIXEL_MONO16S:  /* 16 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_MONO16S))
            return DC1394_COLOR_CODING_MONO16S;
            break;
        case CAMWIRE_PIXEL_RGB16S:  /* 48 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_RGB16S))
            return DC1394_COLOR_CODING_RGB16S;
            break;
        case CAMWIRE_PIXEL_RAW8:  /* 8 bits/pixel.*/
            if (is_in_coding_list(coding_list,DC1394_COLOR_CODING_RAW8 ))
            return DC1394_COLOR_CODING_RAW8;
            break;
        case CAMWIRE_PIXEL_RAW16:  /* 16 bits/pixel.*/
            if (is_in_coding_list(coding_list, DC1394_COLOR_CODING_RAW16))
            return DC1394_COLOR_CODING_RAW16;
            break;
        default:
            return 0;  /* No such coding.*/
            break;
    }
    return 0;  /* Not supported by camera.*/
}

void camwire::camwire::convert_colourcoefs2avtvalues(const double coef[9], int32_t val[9])
{
    double safe[9];
    int c, rowsum, col;

    memcpy(safe, coef, 9*sizeof(double));
    for (c = 0; c < 9; ++c)
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
    for (c = 0; c < 9; ++c)
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
}

int camwire::camwire::is_in_coding_list(const dc1394color_codings_t &coding_list, const dc1394color_coding_t color_id)
{
    for (int c = 0; c < coding_list.num; ++c)
    {
        if (coding_list.codings[c] == color_id)  return 1;
    }
    return 0;
}

dc1394video_mode_t camwire::camwire::convert_format_mode2dc1394video_mode(const int format, const int mode)
{
    try
    {
        int videomode = static_cast<dc1394video_mode_t>(mode_dc1394_offset[format] + mode);
        return static_cast<dc1394video_mode_t>(videomode);
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Cannot convert format to video mode");
        return DC1394_VIDEO_MODE_INVALID;
    }
}

int camwire::camwire::pixel_depth(const Camwire_pixel coding, int &depth)
{
    switch (coding)
    {
        case CAMWIRE_PIXEL_MONO8:
        case CAMWIRE_PIXEL_RAW8:
            depth = 8;
            break;
        case CAMWIRE_PIXEL_YUV411:
            depth = 12;
            break;
        case CAMWIRE_PIXEL_YUV422:
        case CAMWIRE_PIXEL_MONO16:
        case CAMWIRE_PIXEL_MONO16S:
        case CAMWIRE_PIXEL_RAW16:
            depth = 16;
            break;
        case CAMWIRE_PIXEL_YUV444:
        case CAMWIRE_PIXEL_RGB8:
            depth = 24;
            break;
        case CAMWIRE_PIXEL_RGB16:
        case CAMWIRE_PIXEL_RGB16S:
            depth = 48;
            break;
        default:
            depth = 0;
            return CAMWIRE_FAILURE;  /* Invalid or unknown coding.*/
            break;
    }
    return CAMWIRE_SUCCESS;
}

/* To-Do: the enabling of capabilities could be optimized passing to feature_is_usable() a structure
    containing all the possible features and probing them, instead of assigning pointers and recalling
    same functions. Here the DY concept could be exploited at its maximum */
int camwire::camwire::set_non_dma_registers(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set)
{
    try
    {
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);
        Camwire_state_ptr shadow_state = internal_status->current_set;
        ERROR_IF_NULL(shadow_state);
        /* Update the state shadow flag: */
        ERROR_IF_CAMWIRE_FAIL(set_stateshadow(c_handle, set->shadow));
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

        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        if(get_feature_capability(c_handle, cap, DC1394_FEATURE_TRIGGER) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get feature capability");
            return CAMWIRE_FAILURE;
        }
        ERROR_IF_NULL(cap);
        if (feature_is_usable(cap))
        {
            /* Trigger never has auto capability, and its on-off setting is
               done by camwire_set_trigger_source() below.*/
            ERROR_IF_DC1394_FAIL(dc1394_external_trigger_set_mode(c_handle->camera.get(), DC1394_TRIGGER_MODE_0)); 	/* Edge.*/
            ERROR_IF_CAMWIRE_FAIL(set_trigger_source(c_handle, set->external_trigger));
            if (cap->polarity_capable == DC1394_TRUE)
                ERROR_IF_CAMWIRE_FAIL(set_trigger_polarity(c_handle, set->trigger_polarity));
        }

        /* Shutter: */
        if(get_feature_capability(c_handle, cap, DC1394_FEATURE_SHUTTER) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get feature capability");
            return CAMWIRE_FAILURE;
        }
        ERROR_IF_NULL(cap);
        if (feature_is_usable(cap))
        {
            ERROR_IF_CAMWIRE_FAIL(feature_switch_on(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(feature_go_manual(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(set_shutter(c_handle, set->shutter));
        }

        /* Gain: */
        if(get_feature_capability(c_handle, cap, DC1394_FEATURE_GAIN) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get feature capability");
            return CAMWIRE_FAILURE;
        }
        ERROR_IF_NULL(cap);
        if (feature_is_usable(cap))
        {
            ERROR_IF_CAMWIRE_FAIL(feature_switch_on(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(feature_go_manual(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(set_gain(c_handle, set->gain));
        }

        /* Brightness: */
        if(get_feature_capability(c_handle, cap, DC1394_FEATURE_BRIGHTNESS) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get feature capability");
            return CAMWIRE_FAILURE;
        }
        ERROR_IF_NULL(cap);
        if (feature_is_usable(cap))
        {
            ERROR_IF_CAMWIRE_FAIL(feature_switch_on(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(feature_go_manual(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(set_brightness(c_handle, set->brightness));
        }

        /* White balance: */
        if(get_feature_capability(c_handle, cap, DC1394_FEATURE_WHITE_BALANCE) != CAMWIRE_SUCCESS)
        {
            DPRINTF("Failed to get feature capability");
            return CAMWIRE_FAILURE;
        }
        ERROR_IF_NULL(cap);
        if (feature_is_usable(cap))
        {
            ERROR_IF_CAMWIRE_FAIL(feature_switch_on(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(feature_go_manual(c_handle, cap));
            ERROR_IF_CAMWIRE_FAIL(set_white_balance(c_handle, set->white_balance));
        }

        /* Pixel tiling: */
        shadow_state->tiling = internal_status->extras->tiling_value;
        /* Tiling cannot be set.  Ignore set->tiling.*/

        /* Colour correction and coefficients: */
        if (internal_status->extras->colour_corr_capable)
        {
            ERROR_IF_CAMWIRE_FAIL(set_colour_correction(c_handle, set->colour_corr));
        }
        else
        {
            shadow_state->colour_corr = 0;
        }

        if (internal_status->extras->colour_corr_capable)
        {
            ERROR_IF_CAMWIRE_FAIL(set_colour_coefficients(c_handle, set->colour_coef));
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
            ERROR_IF_CAMWIRE_FAIL(set_gamma(c_handle, set->gamma));
        }
        else
        {
            shadow_state->gamma = 0;  /* Ignore set->gamma.*/
        }

        /* Single-shot: */
        if (internal_status->extras->single_shot_capable)
        {
            ERROR_IF_CAMWIRE_FAIL(set_single_shot(c_handle, set->single_shot));
        }
        else
        {
            shadow_state->single_shot = 0;  /* Ignore set->single_shot.*/
        }

        /* Run-stop: */
        ERROR_IF_CAMWIRE_FAIL(set_run_stop(c_handle, set->running));

        /* The list of settings updated above does not include
           set_frame_size(), set_pixel_coding(), or
           set_framerate() because these are already set in
           dc1394_video_set_framerate() or dc1394_format7_set_roi() and
           because they could cause infinite recursion since they themselves
           contain calls to (re)connect_cam() which call this function.
           camwire_set_frame_offset() is a bit different in that it is set
           up with dc1394_format7_set_roi() but does not require a
           reconnect_cam() when it changes. */

        return CAMWIRE_SUCCESS;


    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set non DMA registers");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_numpackets(const Camwire_bus_handle_ptr &c_handle, u_int32_t &num_p)
{
    try
    {
        dc1394video_mode_t video_mode;
        /* dc1394framerate_t frame_rate_index; */
        dc1394color_coding_t color_id;
        uint32_t packet_size, width, height;
        Camwire_pixel coding;
        video_mode = get_1394_video_mode(c_handle);
        ERROR_IF_ZERO(video_mode);


        ERROR_IF_DC1394_FAIL(dc1394_format7_get_packets_per_frame(c_handle->camera.get(), video_mode, &num_p));

        if(num_p == 0)
        {
            /* If dc1394_format7_get_packets_per_frame() returns a zero
               number of packets, then the IIDC spec says we should
               calculate it ourselves: */
            ERROR_IF_DC1394_FAIL(dc1394_format7_get_packet_size(c_handle->camera.get(), video_mode, &packet_size));
            ERROR_IF_DC1394_FAIL(dc1394_format7_get_image_size(c_handle->camera.get(), video_mode, &width, &height));
            ERROR_IF_DC1394_FAIL(dc1394_format7_get_color_coding(c_handle->camera.get(), video_mode, &color_id));
            coding = convert_colorid2pixelcoding(color_id);
            num_p = convert_packetsize2numpackets(c_handle, packet_size,width, height, coding);
            ERROR_IF_ZERO(num_p);
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get num packets");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_captureframe(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394video_frame_t> &frame)
{
    try
    {
        ERROR_IF_NULL(c_handle->userdata);
        frame.reset(c_handle->userdata->frame.get());
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get captured frame");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::component_depth(const Camwire_pixel coding)
{
    switch (coding)
    {
        case CAMWIRE_PIXEL_MONO8:
        case CAMWIRE_PIXEL_YUV411:
        case CAMWIRE_PIXEL_YUV422:
        case CAMWIRE_PIXEL_YUV444:
        case CAMWIRE_PIXEL_RGB8:
        case CAMWIRE_PIXEL_RAW8:
                                return 8;
        case CAMWIRE_PIXEL_MONO16:
        case CAMWIRE_PIXEL_RGB16:
        case CAMWIRE_PIXEL_MONO16S:
        case CAMWIRE_PIXEL_RGB16S:
        case CAMWIRE_PIXEL_RAW16:
                                return 16;
        default:
                                return 0;
    }
}

dc1394video_mode_t camwire::camwire::get_1394_video_mode(const Camwire_bus_handle_ptr &c_handle)
{
    try
    {
       Camwire_conf_ptr config(new Camwire_conf);
       if(get_config(c_handle, config) != CAMWIRE_SUCCESS)
           return DC1394_VIDEO_MODE_INVALID;
       /* FIXME: Need a better way of checking cache initialization than bus_speed: */
       if (config->bus_speed == 0)
           return DC1394_VIDEO_MODE_INVALID;

       return convert_format_mode2dc1394video_mode(config->format, config->mode);
       /* FIXME: Kludge to get it working. Redo video_mode in config. */

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get IEEE 1394 image video_mode");
        return DC1394_VIDEO_MODE_INVALID;
    }
}

int camwire::camwire::get_feature_capability(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394feature_info_t> &cap, const dc1394feature_t feature)
{
    try
    {
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);
        #ifdef CAMWIRE_DEBUG
            if (internal_status->feature_set.feature[feature-DC1394_FEATURE_MIN].id != feature)
            {
                DPRINTF("Requested feature does not match feature_set.id.");
                return 0;
            }
        #endif

        *cap = internal_status->feature_set.feature[feature - DC1394_FEATURE_MIN];
        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get feature capability");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::feature_is_usable(const std::shared_ptr<dc1394feature_info_t> &cap)
{
    try
    {
        return(cap.get()->available == DC1394_TRUE &&
               cap.get()->readout_capable == DC1394_TRUE &&
               (cap.get()->id == DC1394_FEATURE_TRIGGER ||
                feature_has_mode(cap, DC1394_FEATURE_MODE_MANUAL)));
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve feature usability");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::feature_has_mode(const std::shared_ptr<dc1394feature_info_t> &cap, const dc1394feature_mode_t mode)
{
    try
    {
        for (int m = 0; m < cap.get()->modes.num; ++m)
            if (cap.get()->modes.modes[m] == mode)
                return CAMWIRE_SUCCESS;
        return CAMWIRE_FAILURE;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve feature mode");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::feature_switch_on(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394feature_info_t> &cap)
{
    try
    {
        ERROR_IF_NULL(cap);
        if (cap.get()->on_off_capable == DC1394_TRUE && cap.get()->is_on == DC1394_OFF)
        {
            ERROR_IF_DC1394_FAIL(dc1394_feature_set_power(c_handle->camera.get(), cap.get()->id, DC1394_ON));
            ERROR_IF_DC1394_FAIL(dc1394_feature_get_power(c_handle->camera.get(), cap.get()->id, &cap.get()->is_on));
            if (cap.get()->is_on != DC1394_ON)
            {
                std::string error_id = std::to_string(cap.get()->id);
                std::string error_message = "Could not switch " + error_id + " on.";
                DPRINTF(error_message);
                return CAMWIRE_FAILURE;
            }
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to switch on feature");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::feature_go_manual(const Camwire_bus_handle_ptr &c_handle, std::shared_ptr<dc1394feature_info_t> &cap)
{
    try
    {
        ERROR_IF_NULL(cap);
        if (cap.get()->current_mode != DC1394_FEATURE_MODE_MANUAL)
        {
            ERROR_IF_DC1394_FAIL(dc1394_feature_set_mode(c_handle->camera.get(),
                            cap.get()->id,
                            DC1394_FEATURE_MODE_MANUAL));
            ERROR_IF_DC1394_FAIL(dc1394_feature_get_mode(c_handle->camera.get(),
                            cap.get()->id,
                            &cap.get()->current_mode));
            if (cap.get()->current_mode != DC1394_FEATURE_MODE_MANUAL)
            {
                std::string error_message = "Could not switch " + std::to_string(cap->id) + " to manual.";
                DPRINTF(error_message);
                return CAMWIRE_FAILURE;
            }
        }
        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to switch feature to manual");
        return CAMWIRE_FAILURE;
    }
}

double camwire::camwire::convert_numpackets2framerate(const Camwire_bus_handle_ptr &c_handle, const uint32_t num_packets)
{
    try
    {
        Camwire_conf_ptr config(new Camwire_conf);
        uint32_t actual;

        if (get_config(c_handle, config) != CAMWIRE_SUCCESS)
        {
            DPRINTF("get_config() failed.");
        }

        #ifdef CAMWIRE_DEBUG
            /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
            if (config.get()->bus_speed == 0)
            {
                DPRINTF("get_config() returned null format.");
            }
            else if (config.get()->format != 7)
            {
                DPRINTF("Camera is not in Format 7.");
            }
        #endif

        actual = num_packets;
        if (actual < 1)
            actual = 1;
        if (actual > config->max_packets)
            actual = config->max_packets;
        /* FIXME: Use dc1394_video_get_iso_speed() for bus_speed: */
        return convert_busspeed2busfreq(config->bus_speed)/actual;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to convert number of packets to framerate");
        return 0;
    }
}

double camwire::camwire::convert_busspeed2busfreq(const int bus_speed)
{
    return static_cast<double>(20 * bus_speed);
}

int camwire::camwire::convert_busspeed2dc1394(const int bus_speed)
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
}

int camwire::camwire::create(const Camwire_bus_handle_ptr &c_handle)
{
    Camwire_state_ptr settings(new Camwire_state);
    try
    {
        /* Get factory default start-up settings: */
        ERROR_IF_NULL(c_handle);
        if (get_state(c_handle, settings) != CAMWIRE_SUCCESS)
        {
            DPRINTF("camwire_get_state() failed.");
            return CAMWIRE_FAILURE;
        }

        return create(c_handle, settings);
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to initialize camera with default settings");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::probe_camera_colour_correction(const Camwire_bus_handle_ptr &c_handle)
{
    try
    {
        dc1394_avt_adv_feature_info_t adv_feature;

        int dc1394_return = dc1394_avt_get_advanced_feature_inquiry(c_handle->camera.get(),
                            &adv_feature);

        if(dc1394_return == DC1394_SUCCESS && adv_feature.ColorCorrection != DC1394_FALSE)
            return CAMWIRE_SUCCESS;
        else
            return CAMWIRE_FAILURE;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to probe camera colour correction");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::probe_camera_gamma(const Camwire_bus_handle_ptr &c_handle)
{
    try
    {
        dc1394_avt_adv_feature_info_t adv_capability;

        int dc1394_return = dc1394_avt_get_advanced_feature_inquiry(c_handle->camera.get(),
                            &adv_capability);

        if(dc1394_return == DC1394_SUCCESS && adv_capability.Lookup_Tables == DC1394_TRUE)
            return CAMWIRE_SUCCESS;
        else
            return CAMWIRE_FAILURE;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to probe gamma correction");
        return CAMWIRE_FAILURE;
    }
}

camwire::Camwire_tiling camwire::camwire::probe_camera_tiling(const Camwire_bus_handle_ptr &c_handle)
{
    try
    {
        dc1394video_mode_t video_mode;
        dc1394color_filter_t filter_id;
        int dc1394_return;

        video_mode = get_1394_video_mode(c_handle);
        if (video_mode == 0)
        {
            DPRINTF("Video mode is zero.");
            return CAMWIRE_TILING_INVALID;
        }
        dc1394_return = dc1394_format7_get_color_filter(c_handle->camera.get(),
                                video_mode,
                                &filter_id);
        if (dc1394_return != DC1394_SUCCESS)
        {
            DPRINTF("dc1394_format7_get_color_filter() failed.");
            return CAMWIRE_TILING_INVALID;
        }
        return convert_filterid2pixeltiling(static_cast<dc1394color_filter_t>(filter_id));
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to probe gamma correction");
        return CAMWIRE_TILING_INVALID;
    }
}

int camwire::camwire::create_from_struct(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        return create(c_handle, set);

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to initialize camera with settings");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::destroy(const Camwire_bus_handle_ptr &c_handle)
{
    ERROR_IF_NULL(c_handle);
    if (c_handle)
    {
        try
        {
            set_run_stop(c_handle);
            sleep_frametime(c_handle, 1.5);
            /* Reset causes problems with too many cameras, so comment it out: */
            dc1394_camera_reset(c_handle->camera.get());
            disconnect_cam(c_handle);
            free_internals(c_handle);
            return CAMWIRE_SUCCESS;
        }
        catch(std::runtime_error &re)
        {
            DPRINTF("Failed to disconnect camera from bus");
            return CAMWIRE_FAILURE;
        }
    }
}

int camwire::camwire::debug_print_status(const Camwire_bus_handle_ptr &c_handle)
{
    try
    {
        std::cerr << std::endl << "Internal Status: ";
        User_handle internal_status = c_handle->userdata;
        if(!internal_status)
        {
            std::cerr << "(null)" << std::endl << std::endl;
            return CAMWIRE_SUCCESS;
        }

        std::cerr << std::endl <<
                     "camera_connected: " << internal_status->camera_connected << std::endl <<
                     "frame_lock: "       << internal_status->frame_lock << std::endl <<
                     "frame_number: "     << internal_status->frame_number << std::endl <<
                     "num_dma_buffers: "  << internal_status->num_dma_buffers << std::endl << std::endl;

        std::cerr << std::endl << "Extras: ";
        Extra_features_ptr extra = internal_status->extras;
        if(extra)
        {
            std::cerr << std::endl <<
                         "single_shot_capable: " << extra->single_shot_capable << std::endl <<
                         "gamma_capable: "       << extra->gamma_capable << std::endl <<
                         "gamma_maxval: "        << extra->gamma_maxval << std::endl <<
                         "colour_corr_capable: " << extra->colour_corr_capable << std::endl <<
                         "tiling_value: "        << extra->tiling_value << std::endl << std::endl;
        }
        else
            std::cerr << "(null)" << std::endl << std::endl;

        std::cerr << "Features: ";
        if(internal_status->feature_set.feature)
        {
            std::cerr << std::endl;
            dc1394_feature_print_all(&(internal_status->feature_set), stderr);
            std::cerr << std::endl;
        }
        else
            std::cerr << "(null)" << std::endl << std::endl;

        std::cerr << "Frame: ";
        std::shared_ptr<dc1394video_frame_t> capture_frame = internal_status->frame;
        if(capture_frame)
        {
            std::string image_available;
            if(capture_frame->image)
                image_available = "(available)";
            else
                image_available = "(null)";

            std::string camera_available;
            if(capture_frame->camera)
                camera_available = "(available)";
            else
                camera_available = "(null)";

            std::cerr << std::endl <<
                         "image: "                << image_available << std::endl <<
                         "size [width, height]: " << capture_frame->size[0] << ", " << capture_frame->size[1] << std::endl <<
                         "position [hor, ver]: "  << capture_frame->position[0] << ", " << capture_frame->position[1] << std::endl <<
                         "color_coding: "         << capture_frame->color_coding << std::endl <<
                         "color_filter: "         << capture_frame->color_filter << std::endl <<
                         "yuv_byte_order: "       << capture_frame->yuv_byte_order << std::endl <<
                         "data_depth: "           << capture_frame->data_depth << std::endl <<
                         "data_stride: "          << capture_frame->stride << std::endl <<
                         "video_mode: "           << capture_frame->video_mode << std::endl <<
                         "total_bytes: "          << capture_frame->total_bytes << std::endl <<
                         "image_bytes: "          << capture_frame->image_bytes << std::endl <<
                         "padding_bytes: "        << capture_frame->padding_bytes << std::endl <<
                         "packet_size: "          << capture_frame->packet_size << std::endl <<
                         "packets_per_frame: "    << capture_frame->packets_per_frame << std::endl <<
                         "timestamp: "            << capture_frame->timestamp << std::endl <<
                         "frames_behind: "        << capture_frame->frames_behind << std::endl <<
                         "camera: "               << camera_available << std::endl <<
                         "id: "                   << capture_frame->id << std::endl <<
                         "allocated_image_bytes: "<< capture_frame->allocated_image_bytes << std::endl <<
                         "little_endian: "        << capture_frame->little_endian << std::endl <<
                         "data_in_padding: "      << capture_frame->data_in_padding << std::endl << std::endl;
        }
        else
            std::cerr << "(null)" << std::endl << std::endl;

        std::cerr << "config_cache: ";
        Camwire_conf_ptr cfg = internal_status->config_cache;
        if(cfg)
        {
            std::cerr << std::endl;
            write_config_to_file(stderr, cfg);
        }
        else
            std::cerr << "(null)" << std::endl << std::endl;

        std::cerr << "current_set: ";
        Camwire_state_ptr set = internal_status->current_set;
        if(set)
        {
            std::cerr << std::endl;
            write_state_to_file(stderr, set);
        }
        else
            std::cerr << "(null)" << std::endl << std::endl;

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to print debug status");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::version(std::string &version_str)
{
    try
    {
        std::string version_major, version_minor, version_patch;
        version_major = std::to_string(Camwire_VERSION_MAJOR);
        version_minor = std::to_string(Camwire_VERSION_MINOR);
        version_patch = std::to_string(Camwire_VERSION_PATCH);
        version_str = version_major + version_minor + version_patch;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve CamwirePlus version");
        version_str = "";
        return CAMWIRE_FAILURE;
    }
}

//To-Do
int camwire::camwire::read_state_from_file(FILE *infile, Camwire_state_ptr &set)
{
    try
    {
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to read state from file");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::write_state_to_file(FILE *outfile, const Camwire_state_ptr &set)
{
    try
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
            return CAMWIRE_FAILURE;
        }

        fflush(outfile);
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to write state to file");
        return CAMWIRE_FAILURE;
    }
}

/* This function could potentially throw exceptions */
int camwire::camwire::flush_framebuffers(const Camwire_bus_handle_ptr &c_handle, const int num_to_flush, int &num_flushed, int &buffer_lag)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        void *buffer; //cannot use shared_ptr here
        int flush_count;
        for(flush_count = 0; flush_count < num_to_flush; ++flush_count)
        {
            ERROR_IF_CAMWIRE_FAIL(point_next_frame_poll(c_handle, &buffer, buffer_lag));
            if(!buffer)
                break;
            ERROR_IF_CAMWIRE_FAIL(unpoint_frame(c_handle));
        }

        if(num_flushed)
            num_flushed = flush_count;

        /* Best-guess buffer lag if no frames got flushed: */
        if (buffer_lag && num_to_flush < 1)
        {
            ERROR_IF_CAMWIRE_FAIL(get_framebuffer_lag(c_handle, buffer_lag));
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to flush framebuffers");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::copy_next_frame(const Camwire_bus_handle_ptr &c_handle, void *buffer, int &buffer_lag)
{
    try
    {
        ERROR_IF_NULL(c_handle);

        void *buf_ptr;
        ERROR_IF_CAMWIRE_FAIL(point_next_frame(c_handle, &buf_ptr, buffer_lag));

        /*  Copy the minimum number of bytes to avoid segfaults if the user's
            frame buffer is only just big enough to take one frame.  Note
            that internal_status->frame->total_bytes should not be used
            because it may include padding that the user has not made
            provision for: */
        int width, height, depth;
        Camwire_pixel coding;

        ERROR_IF_CAMWIRE_FAIL(get_frame_size(c_handle, width, height));
        ERROR_IF_CAMWIRE_FAIL(get_pixel_coding(c_handle, coding));
        ERROR_IF_CAMWIRE_FAIL(pixel_depth(coding, depth));
        memcpy(buffer, buf_ptr, (size_t) width*height*depth/8);
        unpoint_frame(c_handle);
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to copy next frame");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::point_next_frame(const Camwire_bus_handle_ptr &c_handle, void **buf_ptr, int &buffer_lag)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);
        if(internal_status->frame_lock)
        {
            DPRINTF("Can't point to new frame before unpointing previous frame.");
            return CAMWIRE_FAILURE;
        }
        internal_status->frame.reset(new dc1394video_frame_t);
        dc1394video_frame_t *frame = internal_status->frame.get();
        int dc1394_return = dc1394_capture_dequeue(c_handle->camera.get(), DC1394_CAPTURE_POLICY_WAIT, &frame);
        if(dc1394_return != DC1394_SUCCESS)
        {
            internal_status->frame = 0;
            DPRINTF("dc1394_capture_dequeue() failed");
            return CAMWIRE_FAILURE;
        }

        ERROR_IF_NULL(frame);
        *buf_ptr = (void *)frame->image;
        internal_status->frame_lock = 1;
        /*  Record buffer timestamp for later use by camwire_get_timestamp(),
            because we don't want to assume that dc1394_capture_enqueue()
            does not mess with its frame arg:*/
        internal_status->dma_timestamp = frame->timestamp*1.0e-6;
        /* Increment the frame number if we have a frame: */
        ++internal_status->frame_number;
        if(buffer_lag > 0)
            ERROR_IF_CAMWIRE_FAIL(get_framebuffer_lag(c_handle, buffer_lag));

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to copy next frame");
        return CAMWIRE_FAILURE;
    }
}

/* DRY at its finest: with a flag, it will be possible to unify point_next_frame with this function */
int camwire::camwire::point_next_frame_poll(const Camwire_bus_handle_ptr &c_handle, void **buf_ptr, int &buffer_lag)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);

        if(internal_status->frame_lock)
        {
            DPRINTF("Can't point to new frame before unpointing previous frame.");
            return CAMWIRE_FAILURE;
        }
        internal_status->frame.reset(new dc1394video_frame_t);
        dc1394video_frame_t *frame = internal_status->frame.get();
        int dc1394_return = dc1394_capture_dequeue(c_handle->camera.get(), DC1394_CAPTURE_POLICY_POLL, &frame);
        if(dc1394_return != DC1394_SUCCESS)
        {
            internal_status->frame = 0;
            DPRINTF("dc1394_capture_dequeue() failed");
            return CAMWIRE_FAILURE;
        }

        ERROR_IF_NULL(frame);
        *buf_ptr = (void *)frame->image;
        internal_status->frame_lock = 1;

        /*  Record buffer timestamp for later use by camwire_get_timestamp(),
            because we don't want to assume that dc1394_capture_enqueue()
            does not mess with its frame arg:*/
        internal_status->dma_timestamp = frame->timestamp*1.0e-6;

        /* Increment the frame number if we have a frame: */
        ++internal_status->frame_number;

        if(buffer_lag > 0)
            ERROR_IF_CAMWIRE_FAIL(get_framebuffer_lag(c_handle, buffer_lag));

        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to point next frame poll");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::unpoint_frame(const Camwire_bus_handle_ptr &c_handle)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status(new Camwire_user_data);
        internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);

        if(internal_status->frame_lock)
        {
            ERROR_IF_DC1394_FAIL(dc1394_capture_enqueue(c_handle->camera.get(), internal_status->frame.get()));
            internal_status->frame.reset();
            internal_status->frame_lock = 0;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to unpoint frame");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::inv_gamma(const Camwire_bus_handle_ptr &c_handle, const void *cam_buf, void *lin_buf, const unsigned long max_val)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);

        /* Checking */
        if (max_val == 0 || max_val > UINT16_MAX)
        {
            DPRINTF("3rd argument is zero or exceeds 16-bit range.");
            return CAMWIRE_FAILURE;
        }

        Camwire_pixel coding;
        ERROR_IF_CAMWIRE_FAIL(get_pixel_coding(c_handle, coding));
        if (component_depth(coding) != 8)
        {
            DPRINTF("Pixel coding does not have 8-bit components.");
            return CAMWIRE_FAILURE;
        }

        /* Initialize look-up table if necessary: */
        if (static_cast<uint16_t>(max_val) != internal_status->extras->gamma_maxval)
        {
            for (size_t i = 0; i < 256; ++i)
                gamma_lut[i] = htons(static_cast<uint16_t>(max_val*gamma_inv[i] + 0.5));

            internal_status->extras->gamma_maxval = static_cast<uint16_t>(max_val);
        }

        int width, height, depth;
        uint16_t *endp, *outp;
        size_t num_components;
        /* Transform: */
        ERROR_IF_CAMWIRE_FAIL(get_frame_size(c_handle, width, height));
        ERROR_IF_CAMWIRE_FAIL(pixel_depth(coding, depth));
        num_components = static_cast<size_t>(width*height*depth/8);

        const uint8_t *inp = reinterpret_cast<const uint8_t *>(cam_buf);
        endp = reinterpret_cast<uint16_t *>(lin_buf) + num_components;
        for (outp = reinterpret_cast<uint16_t *>(lin_buf); outp != endp; ++outp)
            *outp = gamma_lut[*inp++];

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to transform buffers using inv_gamma");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_run_stop(const Camwire_bus_handle_ptr &c_handle, const int runsts)
{
    try
    {
        dc1394switch_t iso_en;
        dc1394bool_t one_shot_set;

        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        ERROR_IF_NULL(internal_status);
        Camwire_state_ptr shadow_state = internal_status->current_set;
        ERROR_IF_NULL(shadow_state);
        if (shadow_state->shadow)
        {
            if (shadow_state->single_shot)
            {  /* Single-shot.*/
                if (shadow_state->running && !runsts)
                { 	/* Stop camera (even if it has already stopped): */
                ERROR_IF_DC1394_FAIL(
                    dc1394_video_set_one_shot(c_handle->camera.get(),
                                  DC1394_OFF));
                }
                else if (runsts)
                {  /* Run in single-shot mode (even if we think it is already): */
                ERROR_IF_DC1394_FAIL(
                    dc1394_video_set_one_shot(c_handle->camera.get(),
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
                    c_handle->camera.get(),
                    DC1394_OFF));
                }
                else if (!shadow_state->running && runsts)
                { 	/* Run camera: */
                ERROR_IF_DC1394_FAIL(
                    dc1394_video_set_transmission(
                    c_handle->camera.get(),
                    DC1394_ON));
                }
                /* else do nothing.*/
            }
        }
        else
        {  /* Don't use shadow: ask the camera: */
            ERROR_IF_DC1394_FAIL(
                dc1394_video_get_transmission(c_handle->camera.get(),
                              &iso_en));
            if (iso_en == DC1394_ON)
            {  /* Camera is running in continuous mode: */
                shadow_state->single_shot = 0;
                if (!runsts)
                {  /* Stop camera: */
                ERROR_IF_DC1394_FAIL(
                    dc1394_video_set_transmission(
                    c_handle->camera.get(),
                    DC1394_OFF));
                }
                /* else do nothing.*/
            }
            else
            {
                if (internal_status->extras->single_shot_capable)
                {
                ERROR_IF_DC1394_FAIL(
                    dc1394_video_get_one_shot(c_handle->camera.get(),
                                  &one_shot_set));
                if (one_shot_set == DC1394_TRUE)
                {  /* Camera is running in single-shot mode: */
                    shadow_state->single_shot = 1;
                    if (!runsts)
                    { 	/* Stop camera: */
                    ERROR_IF_DC1394_FAIL(
                        dc1394_video_set_one_shot(c_handle->camera.get(),
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
                        c_handle->camera.get(),
                        DC1394_ON));
                    }
                    else
                    {  /* Run in single-shot mode: */
                    ERROR_IF_DC1394_FAIL(
                        dc1394_video_set_one_shot(c_handle->camera.get(),
                                      DC1394_ON));
                    }
                }
                }
                else if (runsts)
                {  /* Camera is stopped.  Run in continuous mode: */
                ERROR_IF_DC1394_FAIL(
                    dc1394_video_set_transmission(
                    c_handle->camera.get(),
                    DC1394_ON));
                }
            }
        }
        shadow_state->running = runsts;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to start/stop camera");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_stateshadow(const Camwire_bus_handle_ptr &c_handle, const int shadow)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        shadow_state->shadow = shadow;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set state shadow");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_trigger_source(const Camwire_bus_handle_ptr &c_handle, const int external)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_TRIGGER));
        ERROR_IF_NULL(cap);
        if(feature_is_usable(cap))
        {
            shadow_state->external_trigger = external;
            DPRINTF("Camera reported no usable trigger");
            return CAMWIRE_FAILURE;
        }

        dc1394switch_t on_off;
        if(external != 0)
            on_off = DC1394_ON;
        else
            on_off = DC1394_OFF;

        ERROR_IF_DC1394_FAIL(dc1394_external_trigger_set_power(c_handle->camera.get(), on_off));
        shadow_state->external_trigger = external;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set trigger source");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_trigger_polarity(const Camwire_bus_handle_ptr &c_handle, const int rising)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        shadow_state->trigger_polarity = rising;    /* Duplicated? */
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_TRIGGER));
        ERROR_IF_NULL(cap);
        if(feature_is_usable(cap))
        {
            shadow_state->trigger_polarity = rising;
            DPRINTF("Camera reported no usable trigger");
            return CAMWIRE_FAILURE;
        }

        if(cap.get()->polarity_capable != DC1394_TRUE)
        {
            DPRINTF("Camera reported no changeable trigger polarity.");
            return CAMWIRE_FAILURE;
        }

        dc1394trigger_polarity_t polarity;
        if(rising != 0)
            polarity = DC1394_TRIGGER_ACTIVE_HIGH;
        else
            polarity = DC1394_TRIGGER_ACTIVE_LOW;

        ERROR_IF_DC1394_FAIL(dc1394_external_trigger_set_polarity(c_handle->camera.get(), polarity));
        shadow_state->trigger_polarity = rising;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set trigger polarity");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_shutter(const Camwire_bus_handle_ptr &c_handle, const double shutter)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_SHUTTER));
        ERROR_IF_NULL(cap);
        if(feature_is_usable(cap))
        {
            shadow_state->shutter = shutter;
            DPRINTF("Camera reported no usable shutter");
            return CAMWIRE_FAILURE;
        }

        /* Transform to register value: */
        Camwire_conf_ptr config(new Camwire_conf);
        ERROR_IF_CAMWIRE_FAIL(get_config(c_handle, config));
        int shutter_reg = static_cast<int>((shutter - config->exposure_offset)/config->exposure_quantum + 0.5);

        /* Limit shutter_reg to the allowed range: */
        if (shutter_reg < cap->min)
            shutter_reg = cap->min;
        if (shutter_reg > cap->max)
            shutter_reg = cap->max;

        ERROR_IF_DC1394_FAIL(dc1394_feature_set_value(c_handle->camera.get(),
                    DC1394_FEATURE_SHUTTER,
                    shutter_reg));
        shadow_state->shutter = config->exposure_offset + shutter_reg * config->exposure_quantum;

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set shutter");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_gain(const Camwire_bus_handle_ptr &c_handle, const double gain)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_GAIN));
        ERROR_IF_NULL(cap);
        if(feature_is_usable(cap))
        {
            shadow_state->gain = gain;
            DPRINTF("Camera reported no usable shutter");
            return CAMWIRE_FAILURE;
        }

        /* Check limits: */
        if (gain < 0.0 || gain > 1.0)
        {
            DPRINTF("Gain argument should be in the range [0.0, 1.0].");
            return CAMWIRE_FAILURE;
        }

        /* Update the camera with new gains: */
        uint32_t gain_reg;
        if (cap->max >= cap->min)
            gain_reg = cap->min + gain*(cap->max - cap->min) + 0.5;
        else
            gain_reg = 0;

        ERROR_IF_DC1394_FAIL(dc1394_feature_set_value(c_handle->camera.get(),
                     DC1394_FEATURE_GAIN,
                     gain_reg));

        if (cap->max > cap->min)
            shadow_state->gain = static_cast<double>(gain_reg - cap->min)/(cap->max - cap->min);
        else
            shadow_state->gain = 0.0;

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set gain");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_brightness(const Camwire_bus_handle_ptr &c_handle, const double brightness)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_BRIGHTNESS));
        ERROR_IF_NULL(cap);
        if(feature_is_usable(cap))
        {
            shadow_state->brightness = brightness;
            DPRINTF("Camera reported no usable brightness");
            return CAMWIRE_FAILURE;
        }

        /* Check limits: */
        if (brightness < -1.0 || brightness > 1.0)
        {
            DPRINTF("Brightness argument should be in the range [-1.0, +1.0].");
            return CAMWIRE_FAILURE;
        }

        /* Update the camera with new brightness: */
        uint32_t brightness_reg;
        if (cap->max >= cap->min)
            brightness_reg = cap->min + 0.5*(brightness + 1.0)*(cap->max - cap->min) + 0.5;
        else
            brightness_reg = 0;

        ERROR_IF_DC1394_FAIL(
        dc1394_feature_set_value(c_handle->camera.get(),
                     DC1394_FEATURE_BRIGHTNESS,
                     brightness_reg));

        if (cap->max > cap->min)
            shadow_state->brightness = 2.0 * static_cast<double>(brightness_reg - cap->min)/(cap->max - cap->min) - 1.0;
        else
            shadow_state->brightness = 0.0;

        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set brightness");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_white_balance(const Camwire_bus_handle_ptr &c_handle, const double bal[2])
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_WHITE_BALANCE));
        ERROR_IF_NULL(cap);
        if(feature_is_usable(cap))
        {
            shadow_state->white_balance[0] = bal[0];
            shadow_state->white_balance[1] = bal[1];
            DPRINTF("Camera reported no usable white balance");
            return CAMWIRE_FAILURE;
        }

        /* Check limits: */
        if (bal[0] < 0.0 || bal[0] > 1.0 || bal[1] < 0.0 || bal[1] > 1.0)
        {
            DPRINTF("White balance arguments should be in the range [0.0, 1.0].");
            return CAMWIRE_FAILURE;
        }

        /* Update the camera with new balance: */
        uint32_t blue_reg, red_reg;
        if (cap->max >= cap->min)
        {
            blue_reg = cap->min + bal[0]*(cap->max - cap->min) + 0.5;
            red_reg  = cap->min + bal[1]*(cap->max - cap->min) + 0.5;
        }
        else
        {
            blue_reg = red_reg = 0;
        }
        ERROR_IF_DC1394_FAIL(dc1394_feature_whitebalance_set_value(c_handle->camera.get(),
                              blue_reg, red_reg));

        if (cap->max > cap->min)
        {
            shadow_state->white_balance[0] = static_cast<double>(blue_reg - cap->min)/(cap->max - cap->min);
            shadow_state->white_balance[1] = static_cast<double>(red_reg - cap->min)/(cap->max - cap->min);
        }
        else
        {
            shadow_state->white_balance[0] = shadow_state->white_balance[1] = 0.0;
        }

        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set white balance");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_colour_correction(const Camwire_bus_handle_ptr &c_handle, const int corr_on)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        if (!internal_status->extras->colour_corr_capable)
        {
            /* Colour correction is always switched off if the camera can't do it: */
            shadow_state->colour_corr = 0;
            DPRINTF("Camera reported no colour correction capability.");
            return CAMWIRE_FAILURE;
        }

        double coef[9];
        int32_t val[9];
        dc1394bool_t on_off;
        on_off = (corr_on ? DC1394_FALSE : DC1394_TRUE);
        /* Note 0 means on (see AVT Stingray Tech Manual).  There is a bug
           in dc1394_avt_get_color_corr() &
           dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/

        ERROR_IF_CAMWIRE_FAIL(get_colour_coefficients(c_handle, coef));
        convert_colourcoefs2avtvalues(coef, val);

        ERROR_IF_DC1394_FAIL(dc1394_avt_set_color_corr(c_handle->camera.get(),
                      on_off,
                      DC1394_FALSE,
                      val[0], val[1], val[2],
                      val[3], val[4], val[5],
                      val[6], val[7], val[8]));
        shadow_state->colour_corr = (corr_on ? 1 : 0);
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set colour correction");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_colour_coefficients(const Camwire_bus_handle_ptr &c_handle, const double coef[])
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        if (!internal_status->extras->colour_corr_capable)
        {
            /* Colour coefficient matrix is fixed if the camera can't set it: */
            DPRINTF("Camera reported no capability to change colour coefficients.");
            return CAMWIRE_FAILURE;
        }

        int32_t val[9];
        int corr_on;
        dc1394bool_t on_off;
        convert_colourcoefs2avtvalues(coef, val);

        ERROR_IF_CAMWIRE_FAIL(get_colour_correction(c_handle, corr_on));
        /* Note 0 means on (see AVT Stingray Tech Manual).  There is a bug
               in dc1394_avt_get_color_corr() &
               dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/
        on_off = (corr_on ? DC1394_FALSE : DC1394_TRUE);
        ERROR_IF_DC1394_FAIL(dc1394_avt_set_color_corr(c_handle->camera.get(),
                      on_off,
                      DC1394_FALSE,
                      val[0], val[1], val[2],
                      val[3], val[4], val[5],
                      val[6], val[7], val[8]));

        convert_avtvalues2colourcoefs(val, shadow_state->colour_coef);

        return CAMWIRE_SUCCESS;


    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set colour coefficients");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_gamma(const Camwire_bus_handle_ptr &c_handle, const int gamma_on)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        if (!internal_status->extras->gamma_capable)
        {
            /* Camera has no gamma: */
            DPRINTF("Camera reported no capability to change gamma.");
            return CAMWIRE_SUCCESS;
        }

        dc1394bool_t lut_set;
        uint32_t lut_num;
        if (!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_avt_get_lut(c_handle->camera.get(), &lut_set, &lut_num));
            if (lut_set == DC1394_TRUE)
                shadow_state->gamma = 1;
            else
                shadow_state->gamma = 0;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set gamma");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_single_shot(const Camwire_bus_handle_ptr &c_handle, const int &single_shot_on)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        if (!internal_status->extras->single_shot_capable)
        {
            /* Single-shot is always switched off if the camera can't do it: */
            shadow_state->single_shot = 0;
            DPRINTF("Camera reported no single shot capability.");
            return CAMWIRE_FAILURE;
        }

        dc1394switch_t iso_en;
        dc1394bool_t one_shot_set;
        if(shadow_state->shadow)
        {
            /* We *think* the camera is running.*/
            if(shadow_state->running)
            {
                /* Camera is running: change to single-shot: */
                if(!shadow_state->single_shot && single_shot_on)
                {
                    ERROR_IF_DC1394_FAIL(dc1394_video_set_transmission(c_handle->camera.get(), DC1394_OFF));
                    ERROR_IF_DC1394_FAIL(dc1394_video_set_one_shot(c_handle->camera.get(), DC1394_ON));
                }
                /* Don't know if camera is still runnning: let's find out: */
                else if(shadow_state->single_shot && !single_shot_on)
                {
                    ERROR_IF_DC1394_FAIL(dc1394_video_get_one_shot(c_handle->camera.get(), &one_shot_set));
                    /* Camera is still runnning: change to continuous: */
                    if(one_shot_set == DC1394_TRUE)
                    {
                        ERROR_IF_DC1394_FAIL(dc1394_video_set_one_shot(c_handle->camera.get(), DC1394_OFF));
                        ERROR_IF_DC1394_FAIL(dc1394_video_set_transmission(c_handle->camera.get(), DC1394_ON));

                    }
                    else    /* Camera has finished single shot: update shadow state: */
                    {
                        shadow_state->running = 0;
                    }
                }
            }
        }
        else /* else change only the internal state.*/
        {
            ERROR_IF_DC1394_FAIL(dc1394_video_get_transmission(c_handle->camera.get(), &iso_en));
            if (iso_en == DC1394_ON && single_shot_on)
            { 	 /* Camera is running: change to single-shot: */
                 ERROR_IF_DC1394_FAIL(dc1394_video_set_transmission(c_handle->camera.get(), DC1394_OFF));
                 ERROR_IF_DC1394_FAIL(dc1394_video_set_one_shot(c_handle->camera.get(), DC1394_ON));
                 shadow_state->running = 1;
            }
            else if (iso_en == DC1394_OFF && !single_shot_on)
            {
                ERROR_IF_DC1394_FAIL(dc1394_video_get_one_shot(c_handle->camera.get(), &one_shot_set));
                if (one_shot_set == DC1394_TRUE)
                { 	/* Camera is still runnning: change to continuous: */
                    ERROR_IF_DC1394_FAIL(dc1394_video_set_one_shot(c_handle->camera.get(), DC1394_OFF));
                    ERROR_IF_DC1394_FAIL(dc1394_video_set_transmission(c_handle->camera.get(), DC1394_ON));
                    shadow_state->running = 1;
                }
                /* else change only the internal state.*/
            }
        }

        shadow_state->single_shot = single_shot_on;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set single shot mode");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_num_framebuffers(const Camwire_bus_handle_ptr &c_handle, const int &num_frame_buffers)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_conf_ptr config(new Camwire_conf);
        Camwire_state_ptr settings(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_current_settings(c_handle, settings));
        int temp_num_bufs;
        /* Ensure that video1394 lower limit is met: */
        if (num_frame_buffers < 2)
            temp_num_bufs = 2;
        else
            temp_num_bufs = num_frame_buffers;

        /* Only proceed if number of buffers has changed: */
        if (settings->num_frame_buffers != temp_num_bufs)
        {
            ERROR_IF_CAMWIRE_FAIL(get_config(c_handle, config));

            /* Set new number of buffers by re-initializing the camera: */
            settings->num_frame_buffers = temp_num_bufs;
            ERROR_IF_CAMWIRE_FAIL(reconnect_cam(c_handle, config, settings));
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set num framebuffers");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::set_frame_size(const Camwire_bus_handle_ptr &c_handle, const int width, const int height)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        dc1394video_mode_t video_mode;
        video_mode = get_1394_video_mode(c_handle);
        ERROR_IF_ZERO(video_mode);

        uint32_t max_width, max_height;
        uint32_t hor_pixel_unit, ver_pixel_unit;
        int left, top;
        int hor_limit, ver_limit;
        int new_width, new_height;
        int min_pixel_units;
        Camwire_state_ptr settings(new Camwire_state);
        Camwire_conf_ptr config(new Camwire_conf);
        if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
        {
            DPRINTF("Attempt to change frame size in a fixed image size format.");
            return CAMWIRE_FAILURE;
        }
        else if (variable_image_size(video_mode))  /* Format 7.*/
        {
            ERROR_IF_CAMWIRE_FAIL(get_current_settings(c_handle, settings));
            /* Width and height: */
            if (width == settings->width && height == settings->height)
            {
                return CAMWIRE_SUCCESS; 	/* Nothing has changed.*/
            }
            /* Get maximum width, maximum height, unit pixel sizes, and
               offsets from the camera: */
            ERROR_IF_DC1394_FAIL(dc1394_format7_get_max_image_size(c_handle->camera.get(),
                video_mode,
                &max_width,
                &max_height));

            if (max_width  == 0 || max_height == 0)
            {
                DPRINTF("dc1394_format7_get_max_image_size() returned a zero "
                    "maximum size.");
                return CAMWIRE_FAILURE;
            }
            ERROR_IF_DC1394_FAIL(
               dc1394_format7_get_unit_size(
                c_handle->camera.get(),
                video_mode,
                &hor_pixel_unit,
                &ver_pixel_unit));
            if (hor_pixel_unit == 0 || ver_pixel_unit == 0)
            {
                DPRINTF("dc1394_format7_get_unit_size() returned a zero "
                    "unit size.");
                return CAMWIRE_FAILURE;
            }

            ERROR_IF_CAMWIRE_FAIL(get_frame_offset(c_handle, left, top));
            /* Adjust input arguments if necessary, taking maximum frame
               sizes, current offsets and unit pixel sizes into account: */
            if (width < INT_MAX - static_cast<int>(hor_pixel_unit)/2)
                new_width  = (width  + hor_pixel_unit/2)/hor_pixel_unit;
            else
                new_width = INT_MAX/hor_pixel_unit;

            if (height < INT_MAX - static_cast<int>(ver_pixel_unit)/2)
                new_height = (height + ver_pixel_unit/2)/ver_pixel_unit;
            else
                new_height = INT_MAX/ver_pixel_unit;

            if (new_width  < 1)
                new_width = 1;
            if (new_height < 1)
                new_height = 1;

            hor_limit = (max_width - left)/hor_pixel_unit;
            ver_limit = (max_height - top)/ver_pixel_unit;
            if (new_width  > hor_limit)  new_width  = hor_limit;
            if (new_height > ver_limit)  new_height = ver_limit;

            /* Maintain the minimum number of pixels: */
            ERROR_IF_CAMWIRE_FAIL(get_config(c_handle, config));
            min_pixel_units = config->min_pixels/(hor_pixel_unit*ver_pixel_unit);
            if (new_width*new_height < min_pixel_units)
            {
                new_width = (min_pixel_units + new_height - 1)/new_height;
                if (new_width > hor_limit)
                {
                    new_width = hor_limit;
                    new_height = (min_pixel_units + new_width - 1)/new_width;
                    if (new_height > ver_limit)
                        new_height = ver_limit;
                }
            }
            new_width  *= hor_pixel_unit;
            new_height *= ver_pixel_unit;

                /* Only proceed if size has changed after all: */
            if (new_width != settings->width || new_height != settings->height)
            {
                settings->width  = new_width;
                settings->height = new_height;

                /* Set the new dimensions by re-initializing the camera: */
                ERROR_IF_CAMWIRE_FAIL(
                reconnect_cam(c_handle, config, settings));
            }
        }
        else
        {
            DPRINTF("Unsupported camera format.");
            return CAMWIRE_FAILURE;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set frame size");
        return CAMWIRE_FAILURE;
    }
}

/* The pixel colour coding is updated by disconnecting and reconnecting
   the camera.  I have not been able to do it less brutally.  It seems
   that the video1394 driver does not expect the frame size to change
   even if enough memory has been allocated for larger frames. */
int camwire::camwire::set_pixel_coding(const Camwire_bus_handle_ptr &c_handle, const Camwire_pixel coding)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        dc1394video_mode_t video_mode;
        video_mode  = get_1394_video_mode(c_handle);
        ERROR_IF_ZERO(video_mode);

        Camwire_pixel old_coding;
        Camwire_state_ptr shadow_state(new Camwire_state), settings(new Camwire_state);
        Camwire_conf_ptr config(new Camwire_conf);
        dc1394color_codings_t coding_list;
        dc1394color_coding_t color_id;
        int old_depth, new_depth;

        if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
        {
            DPRINTF("Attempt to set pixel coding in a fixed image size format.");
            /* TODO: Set it by changing the mode?*/
            return CAMWIRE_FAILURE;
        }
        else if (variable_image_size(video_mode))  /* Format 7.*/
        {
            ERROR_IF_CAMWIRE_FAIL(get_pixel_coding(c_handle, old_coding));

            /* Only proceed if pixel colour coding has changed: */
            if (coding != old_coding)
            {
                /* Check if new pixel coding is supported by camera: */
                ERROR_IF_DC1394_FAIL(dc1394_format7_get_color_codings(c_handle->camera.get(), video_mode,&coding_list));
                if (coding_list.num == 0)
                {
                    DPRINTF("dc1394_format7_get_color_codings() returned an empty list.");
                    return CAMWIRE_FAILURE;
                }

                color_id = static_cast<dc1394color_coding_t>(convert_pixelcoding2colorid(coding, coding_list));
                if (color_id == 0)
                {
                    DPRINTF("Pixel colour coding is invalid or not supported  by "
                        "the camera.");
                    return CAMWIRE_FAILURE;
                }

                /* Set the new coding: */
                ERROR_IF_CAMWIRE_FAIL(pixel_depth(old_coding, old_depth));
                ERROR_IF_CAMWIRE_FAIL(pixel_depth(coding, new_depth));
                if (new_depth == old_depth)
                {
                    /* Set the new coding directly: */
                    ERROR_IF_DC1394_FAIL(
                        dc1394_format7_set_color_coding(
                        c_handle->camera.get(),
                        video_mode,
                        color_id));
                    ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
                    ERROR_IF_NULL(shadow_state);
                    shadow_state->coding = coding;
                }
                else
                {
                    /* Re-initialize the camera with the new coding: */
                    ERROR_IF_CAMWIRE_FAIL(get_config(c_handle, config));
                    ERROR_IF_CAMWIRE_FAIL(get_current_settings(c_handle, settings));
                    settings->coding = coding;
                    ERROR_IF_CAMWIRE_FAIL(reconnect_cam(c_handle, config, settings));
                }
            }
        }
        else
        {
            DPRINTF("Unsupported camera format.");
            return CAMWIRE_FAILURE;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to set pixel coding.");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        if (!internal_status || !internal_status->camera_connected)
        {  /* Camera does not exit.*/

           return generate_default_settings(c_handle, set);
        }
        else
        {  /* Camera exists.*/
           return generate_default_settings(c_handle, set);
        }
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get camera state");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_config(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_id identifier;

        /* Use cached config if it is available: */
        if(internal_status && config_cache_exists(internal_status))
        {
            cfg = internal_status->config_cache;
            DPRINTF("Cached config available");
        }
        else
        {
            /* Read a conf file and cache it.*/
            ERROR_IF_CAMWIRE_FAIL(get_identifier(c_handle, identifier));
            FILE *conffile;
            std::string conffilename("");
            cfg.reset(new Camwire_conf);
            if (find_conf_file(identifier, conffilename) == CAMWIRE_SUCCESS)
            {
                if((conffile = fopen(conffilename.c_str(), "r")) == NULL)
                {
                    DPRINTF("Failed to open configuration file: ");
                    DPRINTF(conffilename);
                    return CAMWIRE_FAILURE;
                }
                ERROR_IF_CAMWIRE_FAIL(read_conf_file(conffile, cfg));
                fclose(conffile);

                if (internal_status && cfg)
                { /* A camera has been created (not strictly necessary).*/
                    internal_status->config_cache = cfg;
                }
            }
            else
            {
                std::cerr << std::endl <<
                "Camwire could not find a hardware configuration file.\n"
                "Generating a default configuration..." << std::endl;
                ERROR_IF_CAMWIRE_FAIL(generate_default_config(c_handle, cfg));
                std::cout << std::endl <<
                "----------------------------------------------------------------" << std::endl;

                    ERROR_IF_CAMWIRE_FAIL(
                    write_config_to_output(cfg));

                std::cout << std::endl <<
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
                "For the current camera suitable filenames are: \n" <<
                identifier.chip << ".conf \t(chip)" << std::endl <<
                identifier.model << ".conf \t(model)" << std::endl <<
                identifier.vendor << ".conf \t(vendor)\n"
                "Camwire checks for filenames like these in this\n"
                "chip-model-vendor order.  It first looks for the three filenames\n"
                "in the current working directory and after that in a directory\n"
                "given by the CAMWIRE_CONF environment variable.\n\n" << std::endl;
                return CAMWIRE_FAILURE;
            }
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve camera configuration");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_identifier(const Camwire_bus_handle_ptr &c_handle, Camwire_id &identifier)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camera_handle camera = c_handle->camera;
        identifier.vendor = camera->vendor;
        identifier.model = camera->model;
        char chip[32];
        snprintf(chip, 32, "%" PRIX64 "h", camera->guid);
        identifier.chip = std::string(chip);
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get identifier");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_colour_correction(const Camwire_bus_handle_ptr &c_handle, int &corr_on)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        corr_on = shadow_state->colour_corr;

        if (!internal_status->extras->colour_corr_capable)
        {
            /* Camera has no colour correction.  Return the default corr-on of 0: */
            return CAMWIRE_SUCCESS;
        }

        int32_t val[9];
        double coef[9];
        dc1394bool_t on_off;
        if (!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_avt_get_color_corr(c_handle->camera.get(), &on_off,
                              &val[0], &val[1], &val[2],
                              &val[3], &val[4], &val[5],
                              &val[6], &val[7], &val[8]));

            /* Note 0 means on (see AVT Stingray Tech Manual).  There is a
               bug in dc1394_avt_get_color_corr() &
               dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/
            corr_on = (on_off == DC1394_FALSE);
            shadow_state->colour_corr = corr_on;
            convert_avtvalues2colourcoefs(val, coef);
            memcpy(shadow_state->colour_coef, coef, 9*sizeof(coef[0]));
        }
        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve colour corrections");
        corr_on = 0;
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_colour_coefficients(const Camwire_bus_handle_ptr &c_handle, double coef[9])
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        memcpy(coef, shadow_state->colour_coef, 9*sizeof(coef[0]));
        if (!internal_status->extras->colour_corr_capable)
        {
            /* Camera cannot change colour correction coefficients: */
            return CAMWIRE_SUCCESS;
        }

        dc1394bool_t on_off;
        int32_t val[9];

        if (!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_avt_get_color_corr(c_handle->camera.get(),
                              &on_off,
                              &val[0], &val[1], &val[2],
                              &val[3], &val[4], &val[5],
                              &val[6], &val[7], &val[8]));
            convert_avtvalues2colourcoefs(val, coef);
            memcpy(shadow_state->colour_coef, coef, 9*sizeof(coef[0]));
            /* Note 0 means on (see AVT Stingray Tech Manual).  There is a
                   bug in dc1394_avt_get_color_corr() &
                   dc1394_avt_get_advanced_feature_inquiry() v2.1.2.*/
            shadow_state->colour_corr = (on_off == DC1394_FALSE);
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve colour coefficients");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_gamma(const Camwire_bus_handle_ptr &c_handle, int &gamma_on)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        if (!internal_status->extras->gamma_capable)
        {
            /* Camera cannot change colour correction coefficients: */
            return CAMWIRE_SUCCESS;
        }

        dc1394bool_t lut_set;
        uint32_t lut_num;
        if (!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_avt_get_lut(c_handle->camera.get(), &lut_set, &lut_num));
            if (lut_set == DC1394_TRUE)
                gamma_on = 1;
            else
                gamma_on = 0;
            shadow_state->gamma = gamma_on;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve gamma value");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_white_balance(const Camwire_bus_handle_ptr &c_handle, double bal[2])
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        bal[0] = shadow_state->white_balance[0];
        bal[1] = shadow_state->white_balance[1];

        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_WHITE_BALANCE));
        if(!feature_is_usable(cap))
            return CAMWIRE_SUCCESS; /* Camera has no usable white balance:*/ /* Return shadow values.*/

        uint32_t blue_reg, red_reg;
        if(!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_feature_whitebalance_get_value(c_handle->camera.get(), &blue_reg, &red_reg));
            if (static_cast<int>(blue_reg) >= cap->min && static_cast<int>(blue_reg) <= cap->max && static_cast<int>(red_reg) >= cap->min && static_cast<int>(red_reg) <= cap->max)
            {
                if (cap->max != cap->min)
                {
                    bal[0] = static_cast<double>((blue_reg - cap->min)/(cap->max - cap->min));
                    bal[1] = static_cast<double>((red_reg - cap->min)/(cap->max - cap->min));
                }
                else
                {
                    bal[0] = bal[1] = 0.0;
                }
            }
            else
            {
                DPRINTF("Invalid white balance min and max values");
                return CAMWIRE_FAILURE;
            }
            shadow_state->white_balance[0] = bal[0];
            shadow_state->white_balance[1] = bal[1];
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_single_shot(const Camwire_bus_handle_ptr &c_handle, int &single_shot_on)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        single_shot_on = shadow_state->single_shot;

        if(!internal_status->extras->single_shot_capable)
            return CAMWIRE_SUCCESS; /* Camera has no single-shot:*/

        dc1394switch_t iso_en;
        dc1394bool_t one_shot_set;
        if(!shadow_state->shadow)
        {
            if (iso_en == DC1394_ON)
            {  /* Running in continuous mode.*/
                shadow_state->running = 1;
                single_shot_on = 0;
            }
            else
            {  /* Running in single-shot mode or stopped.*/
                ERROR_IF_DC1394_FAIL(dc1394_video_get_one_shot(c_handle->camera.get(), &one_shot_set));
                if (one_shot_set == DC1394_TRUE)
                {  /* Camera is running.*/
                    shadow_state->running = 1;
                    single_shot_on = 1;
                }
                else
                {  /* Camera is stopped.*/
                    shadow_state->running = 0;
                    single_shot_on = shadow_state->single_shot;  /* Remember.*/
                }
            }
            shadow_state->single_shot = single_shot_on;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve single shot mode");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_run_stop(const Camwire_bus_handle_ptr &c_handle, int &runsts)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        dc1394switch_t iso_en;
        dc1394bool_t one_shot_set;
        if(shadow_state->shadow)
        {
            runsts = shadow_state->running;
            /* One_Shot register self-clears after transmission: */
            if(shadow_state->running && shadow_state->single_shot)
            {
                /* Don't know if camera is still runnning: let's find out: */
                ERROR_IF_DC1394_FAIL(dc1394_video_get_one_shot(c_handle->camera.get(), &one_shot_set));
                if (one_shot_set == DC1394_FALSE)
                {  /* Camera has finished single shot: update shadow state: */
                    shadow_state->running = runsts = 0;
                }
            }
        }
        else /* Don't use shadow: ask the camera: */
        {
            ERROR_IF_DC1394_FAIL(dc1394_video_get_transmission(c_handle->camera.get(), &iso_en));
            if(iso_en == DC1394_ON)
            {
                /* Camera is running in continuous mode: */
                shadow_state->single_shot = 0;
                runsts = 1;
            }
            else
            {
                runsts = 0;
                if(internal_status->extras->single_shot_capable)
                {
                    ERROR_IF_DC1394_FAIL(dc1394_video_get_one_shot(c_handle->camera.get(), &one_shot_set));
                    if(one_shot_set == DC1394_TRUE)
                    {
                        /* Camera is running in single-shot mode: */
                        shadow_state->single_shot = 1;
                        runsts = 1;
                    }
                }
            }
            shadow_state->running = runsts;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve running state");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_gain(const Camwire_bus_handle_ptr &c_handle, double &gain)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        gain = shadow_state->gain;
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);

        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_GAIN));
        if(!feature_is_usable(cap))
            /* Camera has no usable gain:*/
            return CAMWIRE_SUCCESS; /* Return shadow values.*/

        uint32_t gain_reg;
        if(!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_feature_get_value(c_handle->camera.get(), DC1394_FEATURE_GAIN, &gain_reg));

            if(static_cast<int>(gain_reg) >= cap->min && static_cast<int>(gain_reg) <= cap->max)
            {
                if(cap->max != cap->min)
                    gain = static_cast<double>((gain_reg - cap->min) / (cap->max - cap->min));
                else
                    gain = 0.0;
            }
            else
            {
                DPRINTF("Invalid gain min and max values");
                return CAMWIRE_FAILURE;
            }
            shadow_state->gain = gain;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve gain");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_brightness(const Camwire_bus_handle_ptr &c_handle, double &brightness)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        brightness = shadow_state->brightness;
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_BRIGHTNESS));
        if(!feature_is_usable(cap))
            /* Camera has no usable brightness:*/
            return CAMWIRE_SUCCESS; /* Return shadow values.*/

        uint32_t brightness_reg;
        if(!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_feature_get_value(c_handle->camera.get(), DC1394_FEATURE_BRIGHTNESS, &brightness_reg));
            if(static_cast<int>(brightness_reg) >= cap->min && static_cast<int>(brightness_reg) <= cap->max)
            {
                if(cap->max != cap->min)
                    brightness = 2.0 * static_cast<double>((brightness_reg - cap->min)) / (cap->max - cap->min) - 1.0;
                else
                    brightness = 0.0;
            }
            else
            {
                DPRINTF("Invalid brightness min and max values");
                return CAMWIRE_FAILURE;
            }
            shadow_state->brightness = brightness;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve brightness value");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_trigger_polarity(const Camwire_bus_handle_ptr &c_handle, int &rising)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        rising = shadow_state->trigger_polarity;
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_TRIGGER));
        if(!feature_is_usable(cap))
            /* Camera has no usable trigger:*/
            return CAMWIRE_SUCCESS; /* Return shadow values.*/

        dc1394trigger_polarity_t polarity;
        if(!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_external_trigger_get_polarity(c_handle->camera.get(), &polarity));
            if(polarity == DC1394_TRIGGER_ACTIVE_LOW)
                rising = 0;
            else
                rising = 1;

            shadow_state->trigger_polarity = rising;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_trigger_source(const Camwire_bus_handle_ptr &c_handle, int &external)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        external = shadow_state->external_trigger;
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_TRIGGER));
        if(!feature_is_usable(cap))
            /* Camera has no usable trigger:*/
            return CAMWIRE_SUCCESS; /* Return shadow values.*/

        dc1394switch_t trigger_on;
        if(!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_external_trigger_get_power(c_handle->camera.get(), &trigger_on));
            if(trigger_on == DC1394_OFF)
                external = 0;
            else
                external = 1;

            shadow_state->external_trigger = external;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve trigger source");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_shutter(const Camwire_bus_handle_ptr &c_handle, double &shutter)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        shutter = shadow_state->shutter;
        std::shared_ptr<dc1394feature_info_t> cap(new dc1394feature_info_t);
        ERROR_IF_CAMWIRE_FAIL(get_feature_capability(c_handle, cap, DC1394_FEATURE_SHUTTER));
        if(!feature_is_usable(cap))
            /* Camera has no usable shutter:*/
            return CAMWIRE_SUCCESS; /* Return shadow values.*/

        uint32_t shutter_reg;
        Camwire_conf_ptr config(new Camwire_conf);
        if(!shadow_state->shadow)
        {
            ERROR_IF_DC1394_FAIL(dc1394_feature_get_value(c_handle->camera.get(), DC1394_FEATURE_SHUTTER, &shutter_reg));
            ERROR_IF_CAMWIRE_FAIL(get_config(c_handle, config));
            shutter = config->exposure_offset + shutter_reg * config->exposure_quantum;

            shadow_state->shutter = shutter;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve shutter");
        return CAMWIRE_FAILURE;
    }
}

/* In Formats 0, 1 and 2, the frame rate is stored in the camera as an
   index, which we translate into a frame rate in frames per second.

   In format 7, the camera's frame rate index is ignored.  One has to
   calculate the frame rate from the number of packets required to send
   one frame.  For example, since exactly one packet is sent every 125
   microseconds (assuming bus speed of 400 Mb/s), the frame rate is
   1/(num_packets*125us).  The camera calculates num_packets and we read
   it from a register called PACKET_PER_FRAME_INQ.
*/
int camwire::camwire::get_framerate(const Camwire_bus_handle_ptr &c_handle, double &frame_rate)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        frame_rate = shadow_state->frame_rate;

        dc1394video_mode_t video_mode;
        dc1394framerate_t frame_rate_index;
        uint32_t num_packets;

        if(!shadow_state->shadow)
        {
            video_mode = get_1394_video_mode(c_handle);
            ERROR_IF_ZERO(video_mode);
            if (fixed_image_size(video_mode))  /* Format 0, 1 or 2.*/
            {
                ERROR_IF_DC1394_FAIL(dc1394_video_get_framerate(c_handle->camera.get(), &frame_rate_index));
                frame_rate = convert_index2framerate(frame_rate_index);
                if (frame_rate < 0.0)
                {
                    DPRINTF("convert_index2framerate() failed.");
                    return CAMWIRE_FAILURE; 	/* Invalid index.*/
                }
            }
            else if (variable_image_size(video_mode))  /* Format 7.*/
            {
                /* It is safe to call get_numpackets() because we are not
                   changing the image_size or color_id: */
                ERROR_IF_CAMWIRE_FAIL(get_numpackets(c_handle, num_packets));
                frame_rate = convert_numpackets2framerate(c_handle, num_packets);
            }
            else
            {
                DPRINTF("Unsupported camera format.");
                return CAMWIRE_FAILURE;
            }
            shadow_state->frame_rate = frame_rate;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve framerate");
        return CAMWIRE_FAILURE;
    }
}


int camwire::camwire::get_pixel_tiling(const Camwire_bus_handle_ptr &c_handle, Camwire_tiling &tiling)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        tiling = shadow_state->tiling;
        dc1394video_mode_t video_mode;
        if(!shadow_state->shadow)
        {
            video_mode = get_1394_video_mode(c_handle);
            ERROR_IF_ZERO(video_mode);
            if(fixed_image_size(video_mode))
                tiling = CAMWIRE_TILING_INVALID;
            else if(variable_image_size(video_mode))
                tiling = probe_camera_tiling(c_handle);
            else
            {
                DPRINTF("Unsupported camera format.");
                return CAMWIRE_FAILURE;
            }
            shadow_state->tiling = tiling;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve pixel tiling");
        return CAMWIRE_FAILURE;
    }
}


int camwire::camwire::get_pixel_coding(const Camwire_bus_handle_ptr &c_handle, Camwire_pixel &coding)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        coding = shadow_state->coding;

        dc1394video_mode_t video_mode;
        dc1394color_coding_t color_id;
        if(!shadow_state->shadow)
        {
            video_mode = get_1394_video_mode(c_handle);
            ERROR_IF_ZERO(video_mode);
            if(fixed_image_size(video_mode))
                coding = convert_videomode2pixelcoding(video_mode);
            else if(variable_image_size(video_mode))
            {
                ERROR_IF_DC1394_FAIL(dc1394_format7_get_color_coding(c_handle->camera.get(), video_mode, &color_id));
                coding = convert_colorid2pixelcoding(color_id);
            }
            else
            {
                DPRINTF("Unsupported camera format.");
                return CAMWIRE_FAILURE;
            }
            shadow_state->coding = coding;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve pixel coding");
        return CAMWIRE_FAILURE;
    }
}


int camwire::camwire::get_frame_size(const Camwire_bus_handle_ptr &c_handle, int &width, int &height)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        width = shadow_state->width;
        height = shadow_state->height;

        dc1394video_mode_t video_mode;
        std::shared_ptr<dc1394video_frame_t> capture_frame(new dc1394video_frame_t);
        uint32_t width_val, height_val;

        if(!shadow_state->shadow)
        {
            video_mode = get_1394_video_mode(c_handle);
            ERROR_IF_ZERO(video_mode);
            if(fixed_image_size(video_mode))
            {
                ERROR_IF_CAMWIRE_FAIL(get_captureframe(c_handle, capture_frame));
                if(capture_frame->size[0] == 0 || capture_frame->size[1] == 0)
                {
                    DPRINTF("dc1394video_frame_t containes a zero frame size");
                    return CAMWIRE_FAILURE;
                }
                width = capture_frame->size[0];
                height = capture_frame->size[1];
            }
            else if(variable_image_size(video_mode))
            {
                ERROR_IF_DC1394_FAIL(dc1394_format7_get_image_size(c_handle->camera.get(),
                            video_mode,
                            &width_val,
                            &height_val));
                width = width_val;
                height = height_val;
            }
            else
            {
                DPRINTF("Unsupported camera format.");
                return CAMWIRE_FAILURE;
            }

            shadow_state->width = width;
            shadow_state->height = height;
        }
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve frame size");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_frame_offset(const Camwire_bus_handle_ptr &c_handle, int &left, int &top)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));

        left = shadow_state->left;
        top = shadow_state->top;

        dc1394video_mode_t video_mode;
        uint32_t left_val, top_val;

        if(!shadow_state->shadow)
        {
            video_mode = get_1394_video_mode(c_handle);
            ERROR_IF_ZERO(video_mode);
            if(fixed_image_size(video_mode))
            {
                left = top = 0;
            }
            else if(variable_image_size(video_mode))
            {
                ERROR_IF_DC1394_FAIL(dc1394_format7_get_image_position(c_handle->camera.get(),
                            video_mode,
                            &left_val,
                            &top_val));
                left = left_val;
                top = top_val;
            }
            else
            {
                DPRINTF("Unsupported camera format.");
                return CAMWIRE_FAILURE;
            }

            shadow_state->left = left;
            shadow_state->top = top;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve frame offset");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_framebuffer_lag(const Camwire_bus_handle_ptr &c_handle, int &buffer_lag)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        std::shared_ptr<dc1394video_frame_t> capture_frame(new dc1394video_frame_t);
        if(!capture_frame)
            buffer_lag = 0;
        else
            buffer_lag = capture_frame->frames_behind;

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve buffer lag");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_stateshadow(const Camwire_bus_handle_ptr &c_handle, int &shadow)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        shadow = shadow_state->shadow;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve state shadow");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::get_num_framebuffers(const Camwire_bus_handle_ptr &c_handle, int &num_frame_buffers)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        Camwire_state_ptr shadow_state(new Camwire_state);
        ERROR_IF_CAMWIRE_FAIL(get_shadow_state(c_handle, shadow_state));
        num_frame_buffers = internal_status->num_dma_buffers;
        shadow_state->num_frame_buffers = num_frame_buffers;
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve num framebuffers");
        return CAMWIRE_FAILURE;
    }
}
