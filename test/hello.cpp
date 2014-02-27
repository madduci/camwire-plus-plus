/***********************************************************************
    This file is in the public domain.

    Description:
    
    A simple invocation of the Camwire library for IEEE 1394 cameras.

    Compile and link with something like:

    $ gcc hello.cpp -o hello -lcamwire++ -ldc1394 -lm

    $Id: hello.cpp,v 27.2 2014 micheleadduci Exp $

***********************************************************************/

#include <iostream>

#include "camwire.hpp"          /* Camwire_handle, Camwire_id, camwire_...() */
#include "camwirebus.hpp"       /* Camwire_handle, camwire_bus_...() */

int main()
{
    int num_cameras;
    std::shared_ptr<camwire::camwirebus> bus(new camwire::camwirebus);
    std::vector<std::shared_ptr<camwire::camwire> > camera_managers;
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
        /* Print some info: */
        camera_managers[i]->get_identifier(bus->get_bus_handler(i), camid);
        camera_managers[i]->get_frame_size(bus->get_bus_handler(i), w, h);

        std::cout << "Vendor name: " << camid.vendor << std::endl;
        std::cout << "Model name: " << camid.model << std::endl;
        std::cout << "Chip id: " << camid.chip << std::endl;
        std::cout << "Frame size: " << w << "x" << h << std::endl;
    }

    return 0;
} /* main() */
