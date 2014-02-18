#include <dc1394/vendor/avt.h>

#include <camwire_config.hpp>
#include <camwire.hpp>
#include <camwire_macros.hpp>

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
    return CAMWIRE_SUCCESS;
}

int camwire::camwire::sleep_frametime(const Camwire_bus_handle_ptr &c_handle, double time)
{
    return 1;
}

void camwire::camwire::disconnect(const Camwire_bus_handle_ptr &c_handle)
{

}

void camwire::camwire::free_internals(const Camwire_bus_handle_ptr &c_handle)
{

}

bool camwire::camwire::create(const Camwire_bus_handle_ptr &c_handle)
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

bool camwire::camwire::create_from_struct(const Camwire_bus_handle_ptr &c_handle, const Camwire_state &set)
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

bool camwire::camwire::destroy(const Camwire_bus_handle_ptr &c_handle)
{
    if (c_handle)
    {
        try
        {
            set_run_stop(c_handle);
            sleep_frametime(c_handle, 1.5);
            /* Reset causes problems with too many cameras, so comment it out: */
            /* dc1394_camera_reset(camwire_handle_get_camera(c_handle)); */
            disconnect(c_handle);
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

bool camwire::camwire::get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state &set)
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

bool camwire::camwire::set_run_stop(const Camwire_bus_handle_ptr &c_handle, const bool singleShotMode)
{

}
