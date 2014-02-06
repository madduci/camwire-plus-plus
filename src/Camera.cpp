#include <dc1394/vendor/avt.h>

#include <CameraConfig.h>
#include <Camera.h>
#include <CameraMacros.h>

cw::Camera::Camera()
{

}

cw::Camera::~Camera()
{

}

cw::Camera::Camera(const cw::Camera &cam)
{

}

cw::Camera &cw::Camera::operator=(const cw::Camera &cam)
{
    return *this;
}

int cw::Camera::generateDefaultSettings(const cw::CameraBusHandlePtr &c_handle, cw::CameraState &set)
{
    return 1;
}

int cw::Camera::getCurrentSettings(const cw::CameraBusHandlePtr &c_handle, cw::CameraState &set)
{
    return 1;
}

int cw::Camera::sleepFrameTime(const cw::CameraBusHandlePtr &c_handle, double time)
{
    return 1;
}

void cw::Camera::disconnect(const cw::CameraBusHandlePtr &c_handle)
{

}

void cw::Camera::clearInternalData(const cw::CameraBusHandlePtr &c_handle)
{

}

int cw::Camera::init(const cw::CameraBusHandlePtr &c_handle)
{
    CameraState settings;
    /* Get factory default start-up settings: */
    ERROR_IF_NULL(c_handle.get());
    if (getCameraState(c_handle, settings) != CAMERA_SUCCESS)
    {
        DPRINTF("camwire_get_state() failed.");
        return CAMERA_FAILURE;
    }
    /* CAMERA_SUCCESS & CAMERA_FAILURE are defined in CameraMacros.h.*/

    //return create(c_handle, &settings);
    return 1;
}

int cw::Camera::initFromStruct(const cw::CameraBusHandlePtr &c_handle, const cw::CameraState &set)
{
    ERROR_IF_NULL(c_handle);
    //return create(c_handle, set);
    return 1;
}

bool cw::Camera::destroy(const cw::CameraBusHandlePtr &c_handle)
{
    if (c_handle)
    {
        toggleStartStop(c_handle);
        sleepFrameTime(c_handle, 1.5);
        /* Reset causes problems with too many cameras, so comment it out: */
        /* dc1394_camera_reset(camwire_handle_get_camera(c_handle)); */
        disconnect(c_handle);
        clearInternalData(c_handle);
    }
}

int cw::Camera::getCameraState(const cw::CameraBusHandlePtr &c_handle, CameraState &set)
{
    UserHandle internal_status;

    ERROR_IF_NULL(c_handle);
    internal_status.reset(c_handle->userdata.get());
    if (!internal_status || !internal_status->camera_connected)
    {  /* Camera does not exit.*/

       return generateDefaultSettings(c_handle, set);
    }
    else
    {  /* Camera exists.*/
       return getCurrentSettings(c_handle, set);
    }
}

int cw::Camera::toggleStartStop(const cw::CameraBusHandlePtr &c_handle, const bool singleShotMode)
{

}
