#ifndef CAMWIRE_HPP
#define CAMWIRE_HPP
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


    Title: Header for camwire.cpp

    Description:
    This module is about using a single named camera through its
    handle. The handle should be all a user need know about for complete
    access to all camera functions.  Finding cameras and assigning
    handles to them is done in the Camwire bus module.

Camwire++: Michele Adduci <info@micheleadduci.net>
******************************************************************************/

#include <camwire_handle.hpp>

namespace camwire
{
    class camwire
    {

        public:
            camwire();
            ~camwire();
            /* Sets the camera to default initialization settings and connects it to
               the bus.  This function is equivalent to getCameraState()
               followed by initFromStruct().  The handle c_handle is
               obtained from CameraBusHandle.init(). */
            int create(const Camwire_bus_handle_ptr &c_handle);
            /* Sets the camera to the given initialization settings and connects it
               to the bus.  The CameraState structure is returned unchanged.  The
               handle c_handle is obtained from CameraBusHandle.init(). */
            int create_from_struct(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set);
            /* Disconnects the camera from the bus and frees memory allocated in
               init() or initFromStruct().  All camera
               settings are lost.*/
            int destroy(const Camwire_bus_handle_ptr &c_handle);

            int get_state(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);

            int set_run_stop(const Camwire_bus_handle_ptr &c_handle, const int runsts = 0);
            /* Gets the camera and its bus's static configuration settings for
               initialization from a configuration file.  They are bus-specific
               hardware parameters that the casual user need not know or care about.
               If a configuration file does not exist, an error message is printed which includes a
               best-guess default configuration. */
            int get_config(const Camwire_bus_handle_ptr &c_handle, Camwire_conf_ptr &cfg);
            /* Fills in the given camwire identifier structure.
               The identifier is uniquely and permanently associated with the camera
               hardware, such as might be obtained from configuration ROM data.
               Returns CAMWIRE_SUCCESS on success or CAMWIRE_FAILURE on failure. */
            int get_identifier(const Camwire_bus_handle_ptr &c_handle, Camwire_id &identifier);

        /* Set to protected in case of Subclassing */
        protected:
            Camwire_id cam_id;
            Camwire_user_data user_data;
            camwire(const camwire &cam);
            camwire& operator=(const camwire &cam);
            /*
             Does the actual work of camwire_create() and
              camwire_create_from_struct(), after they have initialized the camera
              to factory settings and sorted out where the start-up settings come
              from.
            */
            int create(const Camwire_bus_handle_ptr &c_handle, const Camwire_state_ptr &set);
            /* Queries the camera for supported features and attempts to create
               sensible default settings.  Note that the camera itself is initialized
               to factory settings in the process. */
            int generate_default_config(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);
            /* Gets the camera's current settings from the state shadow or as
              physically read from the camera, depending on the state shadow flag. */
            int get_current_settings(const Camwire_bus_handle_ptr &c_handle, Camwire_state_ptr &set);
            int sleep_frametime(const Camwire_bus_handle_ptr &c_handle, const double multiple);
            /* Disconnects the camera from the bus and frees memory allocated in
              connect_cam().  The camera should be stopped before calling this
              function.
            */
            void disconnect_cam(const Camwire_bus_handle_ptr &c_handle);
            /*
              Frees the memory allocated in create().  Should only ever be called
              from create() and camwire_destroy().  Assumes a valid c_handle.
            */
            void free_internals(const Camwire_bus_handle_ptr &c_handle);
            /*
              Returns true if the configuration cache exists and has been
              initialized.  It is assumed that User_handle exists and
              is not 0.
            */
            int config_cache_exists(const User_handle &internal_status);
            /*
              Attempts to open a configuration file for reading.  Returns 1 if
              stream pointer creation was successful or 0 on failure.
            */
            int find_conf_file(const Camwire_id &id, std::shared_ptr<FILE> &conffile);
    };

}

#endif
