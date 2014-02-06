/***********************************************************************

    Copyright (c) Industrial Research Limited 2004

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

   
    Title: Camera Bus module

    Description:
    This Camera bus module is about finding all visible cameras and
    providing handles to them.  The handles should be all a user need
    know about for complete access to all camera functions.  Functions
    to control individual cameras through their handles are defined in
    the Camera main module.

CamwirePlus: Michele Adduci <adducimi@informatik.hu-berlin.de>
***********************************************************************/
#include <cstdlib>  /* calloc, malloc, free */

#include <CameraHandle.h>  /* Camwire_handle */
#include <CameraBus.h>
#include <iostream>

cw::CameraBus::CameraBus(): num_cams(0)
{
    handlers.clear();
}

cw::CameraBus::~CameraBus()
{

}

cw::CameraBus::CameraBus(const cw::CameraBus &cb): num_cams(cb.num_cams)
{
    handlers.clear();
}

cw::CameraBus &cw::CameraBus::operator=(const cw::CameraBus &cb)
{
    num_cams = cb.num_cams;
    handlers.clear();
    return *this;
}

bool cw::CameraBus::init()
{
    int camcount;
    dc1394camera_list_t *list = 0;
    dc1394error_t err;
    CameraHandle camera;

    /* Check whether the bus has not already been created: */
    if (isExisting())
    {
        std::cout << "Camera Bus already created" << std::endl;
        return false;
    }

    /* Set default (error) value: */
    num_cams = 0;
    /* Deletes the existing array  */
    handlers.clear();
    /* Initialize the dc1394 library: */
    dc1394_lib = dc1394_new();
    if (!dc1394_lib)
    {
        std::cout << "Failed to initialize dc1394 library" << std::endl;
        return false;
    }
    /* Make a list of the visible cameras: */
    err = dc1394_camera_enumerate(dc1394_lib, &list);
    if (err != DC1394_SUCCESS)
    {
        dc1394_free(dc1394_lib);
        dc1394_lib = 0;
        std::cout << "Error accessing camera" << std::endl;
        return false; 	/* Error accessing a camera.*/
    }

    camcount = list->num;
    if (camcount == 0) 	/* Nothing wrong, just found no cameras.*/
    {
        dc1394_camera_free_list(list);
        dc1394_free(dc1394_lib);
        dc1394_lib = 0;
        num_cams = 0;
        std::cout << "No camera found." << std::endl;
        return false;
    }

    try
    {
        /* Allocate memory for camwire handles: */
        handlers.resize(camcount);
    }
    catch(std::bad_alloc &ba)
    {
        dc1394_camera_free_list(list);
        dc1394_free(dc1394_lib);
        dc1394_lib = 0;
        std::cout << "Failed to allocate camera handlers: " << ba.what() << std::endl;
        return false;
    }

    /* Fill in the camera handle list: */
    for (int h = 0; h < camcount; ++h)
    {
        camera.reset(dc1394_camera_new(dc1394_lib, list->ids[h].guid));
        if (camera)
        {
            handlers[num_cams].reset(new cw::CameraBusHandle);
            if (!handlers[num_cams])
            { 	/* Allocation failure.*/
                dc1394_camera_free_list(list);
                release();
                return false;
            }
            handlers[num_cams].get()->camera = camera;
            handlers[num_cams].get()->userdata = 0;
            num_cams++;
        }
    }

    dc1394_camera_free_list(list);
    if (num_cams == 0)  release();

    return true;
}

bool cw::CameraBus::isExisting()
{
    return (dc1394_lib && num_cams > 0);
}

bool cw::CameraBus::release()
{
    CameraHandle last_camera(new dc1394camera_t);
    for(unsigned int i = 0; i < num_cams; ++i)
    {
        if(i == 0 || handlers[i]->camera != last_camera)
        {
            last_camera = handlers[i].get()->camera;
            handlers[i].reset();
        }
        handlers[i].reset();
    }
    handlers.clear();
    num_cams = 0;
    if(dc1394_lib)
    {
        dc1394_free(dc1394_lib);
        dc1394_lib = 0;
    }
}

bool cw::CameraBus::reset()
{
    CameraHandle last_camera(new dc1394camera_t);
    if(isExisting())
    {
        for(unsigned int i = 0; i < num_cams; ++i)
        {
            if(i == 0 || handlers[i]->camera.get() != last_camera.get())
            {
                *last_camera = *handlers[i]->camera.get();
                dc1394_reset_bus(handlers[i].get()->camera.get());
            }
        }
    }
    else
        release();
}

bool cw::CameraBus::setCameraUserData(const int num_camera, UserHandle user_data)
{
    if(num_camera > handlers.size())
    {
        std::cout << "Cannot set user data for camera " << num_camera << ": out of array" << std::endl;
        return false;
    }
    if(handlers[num_camera] && handlers[num_camera]->userdata)
    {
        handlers[num_camera].get()->userdata = user_data;
        return true;
    }
    else return false;
}

int cw::CameraBus::getNumberOfCameras()
{
    return num_cams;
}

std::vector<cw::CameraBusHandlePtr> cw::CameraBus::getCameraHandlers()
{
    return handlers;
}

cw::CameraBusHandlePtr cw::CameraBus::getSingleCameraHandler(const int num_camera)
{
    if(num_camera > handlers.size())
        return 0;
    if(handlers[num_camera])
        return handlers[num_camera];
    else return 0;
}
