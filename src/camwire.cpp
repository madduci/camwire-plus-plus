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

int camwire::camwire::create(const Camwire_bus_handle_ptr &c_handle)
{
    Camwire_state settings;
    /* Get factory default start-up settings: */
    ERROR_IF_NULL(c_handle.get());
    if (get_state(c_handle, settings) != CAMWIRE_SUCCESS)
    {
        DPRINTF("camwire_get_state() failed.");
        return CAMWIRE_FAILURE;
    }
    /* CAMERA_SUCCESS & CAMERA_FAILURE are defined in CameraMacros.h.*/

    //return create(c_handle, &settings);
    return CAMWIRE_SUCCESS;
}

int camwire::camwire::create_from_struct(const Camwire_bus_handle_ptr &c_handle, const Camwire_state &set)
{
    ERROR_IF_NULL(c_handle);
    //return create(c_handle, set);
    return CAMWIRE_SUCCESS;
}

bool camwire::camwire::destroy(const Camwire_bus_handle_ptr &c_handle)
{
    if (c_handle)
    {
        toggleStartStop(c_handle);
        sleep_frametime(c_handle, 1.5);
        /* Reset causes problems with too many cameras, so comment it out: */
        /* dc1394_camera_reset(camwire_handle_get_camera(c_handle)); */
        disconnect(c_handle);
        free_internals(c_handle);
    }
}

int camwire::camwire::get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state &set)
{
    User_handle internal_status;

    ERROR_IF_NULL(c_handle);
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

int camwire::camwire::toggleStartStop(const Camwire_bus_handle_ptr &c_handle, const bool singleShotMode)
{

}
