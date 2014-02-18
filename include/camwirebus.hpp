#ifndef CAMWIREBUS_HPP
#define CAMWIREBUS_HPP
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


    Title: Header for camwirebus.cpp

    Description: This CameraBus module is about finding all visible
    cameras and providing handles to them.  The handles are all a user
    should need for implementation-independent access to all camera
    functions.  Functions to control individual cameras through their
    handles are defined in the main Camwire module.

Camwire++: Michele Adduci <info@micheleadduci.net>
***********************************************************************/


#include <camwire_handle.hpp>  /* Camwire_handle */
#include <dc1394/dc1394.h>
#include <vector>

namespace camwire
{
    class camwirebus
    {
        public:
            camwirebus();
            ~camwirebus();
            /* Generates an array of all available Camwire Handlers.
             * The camera are not initialized. If no camera are found,
                the function returns false and num_cams is 0.
                If init() is called again, without calling release(), it
                doesn't do anything else
            */
            int create();
            /* Returns true if a bus has been created succesfully */
            int exists();
            /* Returns true if the memory allocations are freed completely */
            int destroy();
            /* Requests a reset of each bus which has cameras attached. */
            int reset();
            /* Registers a pointer to a user data structure for the given camwire.
               Needed internally by Camwire++   -- To be changed
            */
            int set_handle_userdata(const int num_camera, User_handle user_data);

            /* Returns the number of camera discovered */
            int get_number_cameras();
            /* Returns the dc1394camera_t camera handler for the given camera number
               Needed by many dc1394 functions in Camwire. */
            Camwire_bus_handle_ptr get_bus_handler(const int num_camera = 0);
            /* Returns all the dc1394camera_t camera handlers
               Needed by many dc1394 functions in Camwire. */
            std::vector<Camwire_bus_handle_ptr> get_bus_handlers();

        private:
            int num_cams;
            dc1394_t* dc1394_lib;
            std::vector<Camwire_bus_handle_ptr> handlers;
            camwirebus(const camwirebus &cb);
            camwirebus& operator=(const camwirebus &cb);
    };

}

#endif
