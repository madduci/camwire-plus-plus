#include <dc1394/vendor/avt.h>

#include <camwire_config.hpp>
#include <camwire.hpp>

#include <cstring>

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

int camwire::camwire::create(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set)
{
    try
    {
        /* Allocate zero-filled space for internal status, and register a
                   pointer to it in the camera handle: */
        User_handle internal_status(new Camwire_user_data);
        Camwire_conf_ptr config;

        /* Camwire_user_data is defined above.*/

        ERROR_IF_NULL(internal_status); 	/* Allocation failure.*/
        if (!c_handle->handle_set_userdata(internal_status))
        {   /* Already in use.*/
            DPRINTF("camwire_bus_set_userdata() failed.");
            return CAMWIRE_FAILURE;
        }

        /* Allocate zero-filled space for the extra features: */
        internal_status->extras = Extra_features_ptr(new Extra_features);
        if (!internal_status->extras)
        { 	/* Allocation failure.*/
            DPRINTF("calloc(Extra_features) failed.");
            return CAMWIRE_FAILURE;
        }

        /* Allocate zero-filled space for the config cache: */
        internal_status->config_cache = Camwire_conf_ptr(new Camwire_conf);
        if (!internal_status->config_cache)
        { 	/* Allocation failure.*/
            DPRINTF("calloc(Camwire_conf) failed.");
            free_internals(c_handle);
            return CAMWIRE_FAILURE;
        }

        /* Allocate space and make a copy of the initial settings: */
        internal_status->current_set = Camwire_state_ptr(new Camwire_state);
        if (!internal_status->current_set)
        { 	/* Allocation failure.*/
            DPRINTF("malloc(Camwire_state) failed.");
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
        if (connect_cam(c_handle, &config, set) != CAMWIRE_SUCCESS)
        {
            DPRINTF("connect_cam() failed.");
            free_internals(c_handle);
            return CAMWIRE_FAILURE;
        }

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to init camera");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::generate_default_config(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set)
{
    return CAMWIRE_SUCCESS;
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
          /*
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_num_framebuffers(c_handle, &set->num_frame_buffers));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_gain(c_handle, &set->gain));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_brightness(c_handle, &set->brightness));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_white_balance(c_handle, set->white_balance));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_gamma(c_handle, &set->gamma));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_colour_correction(c_handle, &set->colour_corr));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_colour_coefficients(c_handle, set->colour_coef));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_frame_offset(c_handle, &set->left, &set->top));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_frame_size(c_handle, &set->width, &set->height));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_pixel_coding(c_handle, &set->coding));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_pixel_tiling(c_handle, &set->tiling));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_framerate(c_handle, &set->frame_rate));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_shutter(c_handle, &set->shutter));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_trigger_source(c_handle, &set->external_trigger));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_trigger_polarity(c_handle, &set->trigger_polarity));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_single_shot(c_handle, &set->single_shot));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_run_stop(c_handle, &set->running));
            ERROR_IF_CAMWIRE_FAIL(
                camwire_get_stateshadow(c_handle, &set->shadow));*/
        }

        return CAMWIRE_SUCCESS;

    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to retrieve current settings");
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
                    internal_status->frame = 0;
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
                internal_status->frame = 0;
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

int camwire::camwire::find_conf_file(const Camwire_id &id, std::shared_ptr<FILE> &conffile)
{
    try
    {
        if(!conffile)
            conffile = std::shared_ptr<FILE>(open_named_conf_file(0, id->chip));

        std::string env_directory;
        conffile
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to find configuration file");
        return CAMWIRE_FAILURE;
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

        //return create(c_handle, &settings);

        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to initialize camera with default settings");
        return CAMWIRE_FAILURE;
    }
    //return create(c_handle, &settings);

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
            /* dc1394_camera_reset(camwire_handle_get_camera(c_handle)); */
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

int camwire::camwire::get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set)
{
    try
    {
        ERROR_IF_NULL(c_handle);
        User_handle internal_status = c_handle->userdata;
        if (!internal_status || !internal_status->camera_connected)
        {  /* Camera does not exit.*/

           return generate_default_config(c_handle, set);
        }
        else
        {  /* Camera exists.*/
           return generate_default_config(c_handle, set);
        }
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get camera state");
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
        }
        else
        { 	/* Read a conf file and cache it.*/
            ERROR_IF_CAMWIRE_FAIL(get_identifier(c_handle, identifier));
            std::shared_ptr<FILE> conffile(find_conf_file(identifier));
            if (conffile)
            {
                ERROR_IF_CAMWIRE_FAIL(read_conf_file(conffile, cfg));
                fclose(conffile.get());
                if (internal_status && internal_status->config_cache)
                { /* A camera has been created (not strictly necessary).*/
                    internal_status->config_cache = cfg;
                }
            }
            else
            {
                std::cerr << std::endl <<
                "Camwire could not find a hardware configuration file.\n"
                "Generating a default configuration..." << std::endl;
                    ERROR_IF_CAMWIRE_FAIL(
                    generate_default_config(c_handle, cfg));
                std::cout << std::endl <<
                "----------------------------------------------------------------" << std::endl;

                    ERROR_IF_CAMWIRE_FAIL(
                    camwire_write_config_to_file(stdout, cfg));

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
        identifier.vendor = camera.get()->vendor;
        identifier.model = camera.get()->model;
        identifier.chip = std::to_string(camera.get()->guid);
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to get identifier");
        return CAMWIRE_FAILURE;
    }
}
