/***********************************************************************
    This file is in the public domain.

    Description:
    
    A simple invocation of the Camwire library for IEEE 1394 cameras.

    Compile and link with something like:

    $ gcc hello.cpp -o hello -lcamwireplus -ldc1394 -lm

    $Id: hello.cpp,v 25.2 2014 micheleadduic Exp $

***********************************************************************/

#include <iostream>

#include "camwire.hpp"          /* Camwire_handle, Camwire_id, camwire_...() */
#include "camwirebus.hpp"       /* Camwire_handle, camwire_bus_...() */

int main()
{
    int num_cameras;
    std::shared_ptr<camwire::camwirebus> bus(new camwire::camwirebus);
    std::vector<std::shared_ptr<camwire::camwire> > camera_managers;
    int camwire_return;
    int w = 0, h = 0;
    camwire::Camwire_id camid;
    
    /* Initialize the bus: */
    if(bus->create() != CAMWIRE_SUCCESS)
    {
        std::cout << "Failed to initialize the bus" << std::endl;
        return -1;
    }

    num_cameras = bus->get_number_cameras();
    if(num_cameras == 0)
    {
        std::cout << "No camera connected." << std::endl;
        return -1;
    }

    std::cout << "Number of cameras attached: " << num_cameras << std::endl;

    /* Initialize the cameras found */
    camera_managers.resize(num_cameras);
    for(unsigned int i = 0; i < num_cameras; ++i)
    {
        camera_managers[i] = std::shared_ptr<camwire::camwire>(new camwire::camwire);
        ERROR_IF_CAMWIRE_FAIL(camera_managers[i]->create(bus->get_bus_handler(i)));
    }

    /* Initialize the camera: */
    /*c_handle = handle_array[0];
    camwire_return = camwire_create(c_handle);
    if (camwire_return != CAMWIRE_SUCCESS)
    {
	fprintf(stderr, "Could not initialize the camera.\n");
	camwire_destroy(c_handle);
	camwire_bus_destroy();
	return 1;
    }
    */
    /* Print some info: */
    /*camwire_get_identifier(c_handle, &camid);
    camwire_get_frame_size(c_handle, &w, &h);

    printf("Vendor name:       %s\n", camid.vendor);
    printf("Model name:        %s\n", camid.model);
    printf("Vendor & chip ID:  %s\n", camid.chip);


    printf("Frame sizes:  %d x %d\n", w, h);
    */
    /* Clean up: */
    /*
    camwire_destroy(c_handle);
    camwire_bus_destroy();
    */
    return 0;
} /* main() */
