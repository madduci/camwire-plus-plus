#ifndef CAMERABUS_H
#define CAMERABUS_H
/***********************************************************************

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


    Title: Header for camwirebus.c

    Description: This Camwirebus module is about finding all visible
    cameras and providing handles to them.  The handles are all a user
    should need for implementation-independent access to all camera
    functions.  Functions to control individual cameras through their
    handles are defined in the main Camwire module.

***********************************************************************/


#include <CameraData.h>  /* Camwire_handle */
#include <dc1394/dc1394.h>
#include <vector>

namespace cw
{
    class CameraBus
    {
        public:
            CameraBus();
            ~CameraBus();
            /* Generates an array of all available Camera Handlers.
             * The camera are not initialized. If no camera are found,
                the function returns false and num_cams is 0.
                If init() is called again, without calling release(), it
                doesn't do anything else
            */
            bool init();
            /* Returns true if a bus has been created succesfully */
            bool isExisting();
            /* Returns true if the memory allocations are freed completely */
            bool release();
            /* Requests a reset of each bus which has cameras attached. */
            bool reset();
            /* Registers a pointer to a user data structure for the given camwire.
               Needed internally by Camwire++   -- To be changed
            */
            bool setCameraUserData(const int num_camera, UserHandle user_data);
            /* Returns the number of camera discovered */
            int getNumberOfCameras();
            /* Returns all the dc1394camera_t camera handlers
               Needed by many dc1394 functions in Camwire. */
            std::vector<CameraBusHandlePtr> getCameraHandlers();
            /* Returns the dc1394camera_t camera handler for the given camera number
               Needed by many dc1394 functions in Camwire. */
            CameraBusHandlePtr getSingleCameraHandler(const int num_camera = 0);

        private:
            int num_cams;
            dc1394_t* dc1394_lib;
            std::vector<CameraBusHandlePtr> handlers;
            CameraBus(const CameraBus &cb);
            CameraBus& operator=(const CameraBus &cb);
    };

}

#endif
