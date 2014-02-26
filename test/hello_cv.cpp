/***********************************************************************
    This file is in the public domain.

    Description:

    A simple invocation of the Camwire library for IEEE 1394 cameras.

    Compile and link with something like:

    $ gcc hello.cpp -o hello -lcamwireplus -ldc1394 -lm

    $Id: hello.cpp,v 25.2 2014 micheleadduic Exp $

***********************************************************************/

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "camwire.hpp"          /* Camwire_handle, Camwire_id, camwire_...() */
#include "camwirebus.hpp"       /* Camwire_handle, camwire_bus_...() */

int main()
{
    int num_cameras;
    std::shared_ptr<camwire::camwirebus> bus(new camwire::camwirebus);
    std::vector<std::shared_ptr<camwire::camwire> > camera_managers;
    int w = 0, h = 0;
    int counter = 0;
    void *capturebuffer = NULL;
    cv::Mat frame;
    camwire::Camwire_id camid;
    camwire::Camwire_state_ptr settings(new camwire::Camwire_state);
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
    //test just one camera:

    std::shared_ptr<camwire::camwire> camera_manager(new camwire::camwire);
    ERROR_IF_CAMWIRE_FAIL(camera_manager->create(bus->get_bus_handler(0)));
    /* Print some info: */
    camera_manager->get_identifier(bus->get_bus_handler(0), camid);
    camera_manager->get_frame_size(bus->get_bus_handler(0), w, h);

    std::cout << "Vendor name: " << camid.vendor << std::endl;
    std::cout << "Model name: " << camid.model << std::endl;
    std::cout << "Chip id: " << camid.chip << std::endl;
    std::cout << "Frame size: " << w << "x" << h << std::endl;
    frame = cv::Mat(h, w, CV_8UC1);
    //Start grabbing frames and visualize them
    camera_manager->get_stateshadow(bus->get_bus_handler(0), settings->shadow);
    int isrunning;
    camera_manager->get_run_stop(bus->get_bus_handler(0), isrunning);
    if(isrunning)
    {
        std::cout << "Camera is running" << std::endl;
    }
    else
    {
        std::cout << "Camera is stopped" << std::endl;
    }

    ERROR_IF_CAMWIRE_FAIL(camera_manager->set_run_stop(bus->get_bus_handler(0), 1));
    std::cout << "Running Camera" << std::endl;
    isrunning = 1;
    int bufferlag = 0;
    for(;;)
    {
        if(isrunning)
        {
            std::cout << "Querying frame" << std::endl;
            if (camera_manager->point_next_frame(bus->get_bus_handler(0), &capturebuffer, bufferlag) != CAMWIRE_SUCCESS)
            {
                std::cout << "Could not point to next frame" << std::endl;
                exit(-1);
            }
            camera_manager->unpoint_frame(bus->get_bus_handler(0));
            std::cout << "Frame: " << counter << std::endl;
            frame = cv::Mat(h, w, CV_8U, capturebuffer);
            cv::imshow("Image", frame);
            cv::waitKey(10);
        }
    }

    return 0;
} /* main() */

