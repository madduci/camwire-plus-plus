#ifndef CAMERA_H
#define CAMERA_H
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


    Title: Header for Camera.cpp

    Description:
    This module is about using a single named camera through its
    handle. The handle should be all a user need know about for complete
    access to all camera functions.  Finding cameras and assigning
    handles to them is done in the Camwire bus module.

CamwirePlus: Michele Adduci <adducimi@informatik.hu-berlin.de>
******************************************************************************/

#include <CameraData.h>

namespace cw
{
    class Camera
    {

        public:
            Camera();
            ~Camera();
            /* Sets the camera to default initialization settings and connects it to
               the bus.  This function is equivalent to getCameraState()
               followed by initFromStruct().  The handle c_handle is
               obtained from CameraBusHandle.init(). */
            int init(const CameraBusHandlePtr &c_handle);
            /* Sets the camera to the given initialization settings and connects it
               to the bus.  The CameraState structure is returned unchanged.  The
               handle c_handle is obtained from CameraBusHandle.init(). */
            int initFromStruct(const CameraBusHandlePtr &c_handle, const CameraState &set);
            /* Disconnects the camera from the bus and frees memory allocated in
               init() or initFromStruct().  All camera
               settings are lost.*/
            bool destroy(const CameraBusHandlePtr &c_handle);

            int getCameraState(const cw::CameraBusHandlePtr &c_handle, CameraState &set);

            int toggleStartStop(const CameraBusHandlePtr &c_handle, const bool singleShotMode = false);
        private:
            CameraID camID;
            CameraUserData camData;
            Camera(const Camera &cam);
            Camera& operator=(const Camera &cam);
            /* Queries the camera for supported features and attempts to create
               sensible default settings.  Note that the camera itself is initialized
               to factory settings in the process. */
            int generateDefaultSettings(const cw::CameraBusHandlePtr &c_handle, CameraState &set);
            /* Gets the camera's current settings from the state shadow or as
              physically read from the camera, depending on the state shadow flag. */
            int getCurrentSettings(const cw::CameraBusHandlePtr &c_handle, CameraState &set);
            int sleepFrameTime(const cw::CameraBusHandlePtr &c_handle, double time);
            void disconnect(const cw::CameraBusHandlePtr &c_handle);
            void clearInternalData(const cw::CameraBusHandlePtr &c_handle);

    };

}

#endif
