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

int camwire::camwire::generate_default_config(const Camwire_bus_handle_ptr &c_handle, Camwire_state &set)
{
    return CAMWIRE_SUCCESS;
}

int camwire::camwire::get_current_settings(const Camwire_bus_handle_ptr &c_handle, Camwire_state &set)
{
    try
    {
        Camwire_state_ptr shadow_state(new Camwire_state);
        dc1394bool_t one_shot_set;

        //*shadow_state = get_shadow_state(c_handle);
        ERROR_SMART_POINTER(shadow_state);
        if(shadow_state->shadow)
        {
            memcpy(&set, shadow_state.get(), sizeof(Camwire_state));  /* Shortcut.*/
            /* One_Shot register self-clears after transmission, hence we
               don't know if camera is still runnning: */
            if (shadow_state->running && shadow_state->single_shot)
            {
                ERROR_IF_DC1394_FAIL(dc1394_video_get_one_shot(c_handle->camera.get(), &one_shot_set));
                if (one_shot_set == DC1394_FALSE)
                    set.running = shadow_state->running = 0;
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
    User_handle internal_status;
    try
    {
        *internal_status = *c_handle->userdata;
        if (internal_status)
        {
            if (internal_status->camera_connected)
            {
                if (internal_status->frame_lock)
                {
                    dc1394_capture_enqueue(c_handle->camera.get(), internal_status->frame);
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
    User_handle internal_status;
    try
    {
        internal_status = c_handle->userdata;
        if (internal_status)
        {
            if (internal_status->frame_lock)
            {
                dc1394_capture_enqueue(c_handle->camera.get(),
                           internal_status->frame);
                internal_status->frame = 0;
                internal_status->frame_lock = 0;
            }
            free(internal_status->config_cache);
            //free(internal_status->extras);
            internal_status.reset();
        }
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to free internal values");
    }
}

int camwire::camwire::create(const Camwire_bus_handle_ptr &c_handle)
{
    Camwire_state settings;
    try
    {
        /* Get factory default start-up settings: */
        ERROR_SMART_POINTER(c_handle);
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

int camwire::camwire::create_from_struct(const Camwire_bus_handle_ptr &c_handle, const Camwire_state &set)
{
    try
    {
        ERROR_SMART_POINTER(c_handle);
        //return create(c_handle, set);
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        DPRINTF("Failed to initialize camera with settings");
        return CAMWIRE_FAILURE;
    }
}

int camwire::camwire::destroy(const Camwire_bus_handle_ptr &c_handle)
{
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

int camwire::camwire::get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state &set)
{
    User_handle internal_status;

    try
    {
        ERROR_SMART_POINTER(c_handle);
        internal_status.reset(c_handle->userdata.get());
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

int camwire::camwire::set_run_stop(const Camwire_bus_handle_ptr &c_handle, const bool singleShotMode)
{

}
