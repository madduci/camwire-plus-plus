/***********************************************************************
    This file is in the public domain.

    Description:

    A simple invocation of the Camwire library for IEEE 1394 cameras.

    Compile and link with something like:

    $ gcc hello.cpp -o hello -lcamwireplus -ldc1394 -lm

    $Id: hello.cpp,v 25.2 2014 micheleadduic Exp $

***********************************************************************/

#include <iostream>
#include <thread>
#include <X11/Xlib.h>
#include <opencv2/highgui/highgui.hpp>
#include "camwire.hpp"          /* Camwire_handle, Camwire_id, camwire_...() */
#include "camwirebus.hpp"       /* Camwire_handle, camwire_bus_...() */

int retrieveAndDisplay(int num_camera, const std::shared_ptr<camwire::camwire> &camera_manager, const camwire::Camwire_bus_handle_ptr &handler)
{
    int w = 0, h = 0;
    void *capturebuffer = NULL;
    cv::Mat frame;
    camwire::Camwire_id camid;
    camwire::Camwire_state_ptr settings(new camwire::Camwire_state);
    std::string windowtitle = "Camera"+std::to_string(num_camera);

    std::cout << "Starting thread " << num_camera << std::endl;


    ERROR_IF_CAMWIRE_FAIL(camera_manager->create(handler));
    /* Print some info: */
    camera_manager->get_identifier(handler, camid);
    camera_manager->get_frame_size(handler, w, h);
    std::cout << "Camera " << num_camera << ": Vendor name: " << camid.vendor << std::endl;
    std::cout << "Camera " << num_camera << ": Model name: " << camid.model << std::endl;
    std::cout << "Camera " << num_camera << ": Chip id: " << camid.chip << std::endl;
    std::cout << "Camera " << num_camera << ": Frame size: " << w << "x" << h << std::endl;
    //init frame
    frame = cv::Mat(h, w, CV_8UC1);
    //Start grabbing frames and visualize them
    camera_manager->get_stateshadow(handler, settings->shadow);
    int isrunning;
    camera_manager->get_run_stop(handler, isrunning);

    ERROR_IF_CAMWIRE_FAIL(camera_manager->set_single_shot(handler, 0));
    ERROR_IF_CAMWIRE_FAIL(camera_manager->set_run_stop(handler, 1));
    std::cout << "Running Camera " << num_camera << std::endl;
    isrunning = 1;
    int bufferlag = 0;
    for(;;)
    {
        if(isrunning)
        {
            if (camera_manager->point_next_frame(handler, &capturebuffer, bufferlag) != CAMWIRE_SUCCESS)
            {
                std::cout << "Could not point to next frame" << std::endl;
                exit(-1);
            }
            camera_manager->unpoint_frame(handler);
            frame = cv::Mat(h, w, CV_8U, capturebuffer);
            cv::imshow(windowtitle, frame);
            cv::waitKey(30);
        }
    }

}


int main()
{
    XInitThreads();
    int num_cameras;
    std::shared_ptr<camwire::camwirebus> bus(new camwire::camwirebus);
    std::vector<std::shared_ptr<camwire::camwire> > camera_managers;
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
    camera_managers.resize(num_cameras);
    std::cout << "Number of cameras attached: " << num_cameras << std::endl;
    std::vector<std::thread> threads;
    for(int i = 0; i < num_cameras; ++i)
    {
        std::cout << "Creating camera manager " << i << std::endl;
        camera_managers[i] = std::shared_ptr<camwire::camwire>(new camwire::camwire);
        std::cout << "Creating thread " << i << std::endl;
        threads.push_back(std::thread(retrieveAndDisplay, i, camera_managers[i], bus->get_bus_handler(i)));
    }

    for(int i = 0; i < num_cameras; ++i)
    {
        threads[i].join();
    }

    return 0;
} /* main() */

