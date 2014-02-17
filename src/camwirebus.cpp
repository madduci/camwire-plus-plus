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

Camwire++: Michele Adduci <info@micheleadduci.net>
***********************************************************************/
#include <cstdlib>  /* calloc, malloc, free */
#include <camwirebus.hpp>
#include <iostream>
#include <camwire_macros.hpp>

camwire::camwirebus::camwirebus(): num_cams(0)
{
    handlers.clear();
}

camwire::camwirebus::~camwirebus()
{
    handlers.clear();
}

camwire::camwirebus::camwirebus(const camwirebus &cb): num_cams(cb.num_cams)
{
    handlers.clear();
}

camwire::camwirebus& camwire::camwirebus::operator=(const camwirebus &cb)
{
    num_cams = cb.num_cams;
    handlers.clear();
    return *this;
}

bool camwire::camwirebus::create()
{
    int camcount;
    dc1394camera_list_t *list = 0;
    dc1394error_t err;
    Camera_handle camera;

    /* Check whether the bus has not already been created: */
    if (exists())
    {
        DPRINTF("Camera Bus already created");
        return CAMWIRE_FAILURE;
    }

    try
    {
        /* Set default (error) value: */
        num_cams = 0;
        /* Deletes the existing array  */
        handlers.clear();
        /* Initialize the dc1394 library: */
        dc1394_lib = dc1394_new();
        if (!dc1394_lib)
        {
            std::cout << "Failed to initialize dc1394 library" << std::endl;
            return CAMWIRE_FAILURE;
        }
        /* Make a list of the visible cameras: */
        err = dc1394_camera_enumerate(dc1394_lib, &list);
        if (err != DC1394_SUCCESS)
        {
            dc1394_free(dc1394_lib);
            dc1394_lib = 0;
            std::cout << "Error accessing camera" << std::endl;
            return CAMWIRE_FAILURE; 	/* Error accessing a camera.*/
        }

        camcount = list->num;
        if (camcount == 0) 	/* Nothing wrong, just found no cameras.*/
        {
            dc1394_camera_free_list(list);
            dc1394_free(dc1394_lib);
            dc1394_lib = 0;
            num_cams = 0;
            std::cout << "No camera found." << std::endl;
            return CAMWIRE_FAILURE;
        }

        /* Allocate memory for camwire handles: */
        handlers.resize(camcount);

        /* Fill in the camera handle list: */
        for (int h = 0; h < camcount; ++h)
        {
            camera.reset(dc1394_camera_new(dc1394_lib, list->ids[h].guid));
            if (camera)
            {
                handlers[num_cams].reset(new Camwire_bus_handle);
                if (!handlers[num_cams])
                { 	/* Allocation failure.*/
                    dc1394_camera_free_list(list);
                    destroy();
                    return CAMWIRE_FAILURE;
                }
                handlers[num_cams].get()->camera = camera;
                handlers[num_cams].get()->userdata = 0;
                num_cams++;
            }
        }

        dc1394_camera_free_list(list);
        if (num_cams == 0)  destroy();

        return CAMWIRE_SUCCESS;
    }
    catch(std::bad_alloc &ba)
    {
        dc1394_camera_free_list(list);
        dc1394_free(dc1394_lib);
        dc1394_lib = 0;
        std::cout << "Failed to allocate camera handlers: " << ba.what() << std::endl;
        return CAMWIRE_FAILURE;
    }
    catch(std::runtime_error &re)
    {
        dc1394_camera_free_list(list);
        dc1394_free(dc1394_lib);
        dc1394_lib = 0;
        std::cout << "Error during the initialization of camera handlers: " << re.what() << std::endl;
        return CAMWIRE_FAILURE;
    }
}

bool camwire::camwirebus::exists()
{
    return (dc1394_lib && num_cams > 0);
}

bool camwire::camwirebus::destroy()
{
    try
    {
        Camera_handle last_camera(new dc1394camera_t);
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
        return CAMWIRE_SUCCESS;
    }
    catch(std::runtime_error &re)
    {
        if(dc1394_lib)
        {
            dc1394_free(dc1394_lib);
            dc1394_lib = 0;
        }
        std::cout << "Error during the releasing of camera handlers: " << re.what() << std::endl;
        return CAMWIRE_FAILURE;
    }
}

bool camwire::camwirebus::reset()
{
    Camera_handle last_camera(new dc1394camera_t);
    if(exists())
    {
        try
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
        catch(std::runtime_error &re)
        {
            std::cout << "Error resetting the camera handlers: " << re.what() << std::endl;
            return CAMWIRE_FAILURE;
        }
    }
    else
        destroy();

    return CAMWIRE_SUCCESS;
}

bool camwire::camwirebus::set_handle_userdata(const int num_camera, User_handle user_data)
{
    if(num_camera > handlers.size())
    {
        std::cout << "Cannot set user data for camera " << num_camera << ": out of array" << std::endl;
        return CAMWIRE_FAILURE;
    }
    try
    {
        if(handlers[num_camera] && handlers[num_camera]->userdata)
        {
            handlers[num_camera].get()->userdata = user_data;
            return CAMWIRE_SUCCESS;
        }
        else return CAMWIRE_FAILURE;
    }
    catch(std::runtime_error &re)
    {
        std::cout << "Failed to set user data for Camera handler: " << re.what() << std::endl;
        return CAMWIRE_FAILURE;
    }

}

int camwire::camwirebus::get_number_cameras()
{
    return num_cams;
}

camwire::Camwire_bus_handle_ptr camwire::camwirebus::get_camera(const int num_camera)
{
    if(num_camera > handlers.size())
        return 0;
    if(handlers[num_camera])
        return handlers[num_camera];
    else return 0;
}

std::vector<camwire::Camwire_bus_handle_ptr> camwire::camwirebus::get_camera_handlers()
{
    return handlers;
}

