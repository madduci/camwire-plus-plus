#ifndef CAMWIRE_HANDLE_HPP
#define CAMWIRE_HANDLE_HPP
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

    Title: Header for camwire_handle.cpp

    Description:
    This module is about using a single named camera through its
    handle. The handle should be all a user need know about for complete
    access to all camera functions.  Finding cameras and assigning
    handles to them is done in the Camwire bus module.

Camwire++: Michele Adduci <info@micheleadduci.net>

Differences from Camwire:

Included all the structure definition contain in camwire.c directly in one
header file, to extract data representation from logic.
******************************************************************************/

#include <cinttypes>
#include <memory>
#include <camwire_macros.hpp>
#include <dc1394/camera.h>  /* dc1394camera_t.*/
//#include <ctime>          /* For struct timespec.*/


namespace camwire
{
    /*  Type for a unique camera identifier comprising null-terminated vendor
        name, model name, and chip number strings, such as used by
        camwire_get_identifier() below. */
    struct Camwire_id
    {
        std::string vendor;
        std::string model;
        std::string chip;
        Camwire_id(): vendor(""), model(""), chip(""){};
    };

    /* Type for selecting the pixel encoding and pixel depth, as used by
       camwire_get/set_pixel_coding() and camwire_pixel_depth() below.
    */
    enum Camwire_pixel
    {
        CAMWIRE_PIXEL_INVALID,
        CAMWIRE_PIXEL_MONO8,	/*  8 bits average.*/
        CAMWIRE_PIXEL_YUV411,	/* 12 bits average.*/
        CAMWIRE_PIXEL_YUV422,	/* 16 bits average.*/
        CAMWIRE_PIXEL_YUV444,	/* 24 bits average.*/
        CAMWIRE_PIXEL_RGB8,		/* 24 bits average.*/
        CAMWIRE_PIXEL_MONO16,	/* 16 bits average.*/
        CAMWIRE_PIXEL_RGB16,	/* 48 bits average.*/
        CAMWIRE_PIXEL_MONO16S,	/* 16 bits average.*/
        CAMWIRE_PIXEL_RGB16S,	/* 48 bits average.*/
        CAMWIRE_PIXEL_RAW8,		/*  8 bits average.*/
        CAMWIRE_PIXEL_RAW16		/* 16 bits average.*/
    };

    /* Type for selecting the pixel tiling of colour-tiled camera sensors,
       as used by camwire_get_pixel_tiling() below.
    */
    enum Camwire_tiling
    {
        CAMWIRE_TILING_INVALID,
        CAMWIRE_TILING_RGGB,
        CAMWIRE_TILING_GBRG,
        CAMWIRE_TILING_GRBG,
        CAMWIRE_TILING_BGGR,
        CAMWIRE_TILING_UYVY,
        CAMWIRE_TILING_YUYV
    };

    /* To translate to or from dc1394 mode enums: */
    static const int mode_dc1394_offset[] = {
        DC1394_VIDEO_MODE_160x120_YUV444,   /* Format 0.*/
        DC1394_VIDEO_MODE_800x600_YUV422,   /* Format 1.*/
        DC1394_VIDEO_MODE_1280x960_YUV422,  /* Format 2.*/
        0, 0, 0,                            /* Reserved formats.*/
        DC1394_VIDEO_MODE_EXIF,             /* Format 6.*/
        DC1394_VIDEO_MODE_FORMAT7_0         /* Format 7.*/
    };

    /* Type for holding camera settings, such as those returned from
       getCameraState() or passed to
       createCameraFromStruct().  Camwire uses this struct internally
       for shadowing current settings but these are not available to the
       user via this type.  Use the individual access functions instead.

       Each member corresponds to the arguments of the camwire_get/set_...()
       functions:

       num_frame_buffers:	The number of frames to store, not counting any
                            buffers internal to the camera.  Minimum 2,
                            maximum according to how much free memory (RAM) you have.

       gain:		Relative pixel gain which sets the slope of the
                response, between 0.0 and 1.0.

       brightness:		Relative image brightness which sets the pixel
                black level, between -1.0 and +1.0.

       white_balance:	Relative blue (U-value) and red (V-value) white
                balance levels for colour cameras, each between
                0.0 and 1.0.

       gamma:		Flag set to 1 (true) if the camera does gamma
                correction on the image, or 0 (false) for linear
                pixel values.  Only applicable to cameras that
                support a gamma setting and for supported pixel
                encodings.

       colour_corr:		Flag set to 1 (true) if the camera does colour
                correction on the image, or 0 (false) for no
                correction.  Only applicable to cameras that
                support colour correction and for colour pixel
                encodings.

       colour_coef:		Colour correction coefficients, each between
                -1.0 and +2.0, if the camera is capable of
                internal colour correction.  The 9 array
                elements fill the colour correction matrix row
                by row:
                    cc[0] cc[1] cc[2]     Crr Cgr Cbr
                    cc[3] cc[4] cc[5] <-> Crg Cgg Cbg
                    cc[6] cc[7] cc[8]     Crb Cgb Cbb
                The identity matrix implies no correction.

       left, top:		Top left-hand corner of region-of-interest in
                            sensor pixel coordinates.  The top left-hand
                corner pixel of the sensor is at (0,0), and the
                bottom right-hand corner pixel is at
                (sensorwidth-1, sensorheight-1).  Only
                applicable to scalable image sizes.

      width, height:	Width (X dimension) and height (Y dimension) in
                            pixels of the region-of-interest (scalable
                sizes) or the whole image (fixed sizes).

      coding:               One of the CameraPixel enumeration (see
                            above).

      frame_rate:		The rate at which the camera transmits frames to
                            the computer, in frames per second.  The actual
                frame rate may be determined by an external
                trigger frequency or image processing speed.

      shutter:		The integration time, in seconds.

      external_trigger:	Flag set to non-zero to trigger frame
                            acquisition on a hardware trigger signal (if
                available), otherwise the camera sends frames at
                the rate programmed with frame_rate above.

      trigger_polarity:	Flag set to non-zero for triggering on a rising
                            edge of the extenal trigger signal, otherwise it
                triggers on the falling edge.  Ignored if
                external triggering is not used.

      single_shot:		Flag set to non-zero to put camera in
                            single-shot mode (in which only one frame is
                transmitted), otherwise frame transmission is
                continuous when the camera is running.

      running:		Flag set to non-zero to start frame transmission
                            (possibly pending an external trigger if used),
                otherwise the camera is stopped and does not
                transmit frames to the computer.

      shadow:		Flag set to non-zero to cause, wherever
                possible, camera settings to be read from this
                struct, otherwise they are read directly from
                the camera hardware (which takes longer and may
                cause congestion of the asynchronous bandwidth).
    */
    struct Camwire_state
    {
        int num_frame_buffers;
        double gain;
        double brightness;
        double white_balance[2];
        int gamma;
        int colour_corr;
        double colour_coef[9];
        int left, top, width, height;
        Camwire_pixel coding;
        Camwire_tiling tiling;
        double frame_rate, shutter;
        int external_trigger, trigger_polarity;
        int single_shot, running;
        int shadow;
    };

    typedef std::shared_ptr<Camwire_state>  Camwire_state_ptr;

    /* Type for holding IEEE 1394 and IIDC DCAM hardware configuration data
       that the casual user probably does not want to know about.  See
       CONFIGURATION documentation for a detailed description of each
       member. */
    struct Camwire_conf
    {
        int bus_speed;
        int format;
        int mode;
        int max_packets;
        int min_pixels;
        double trig_setup_time;
        double exposure_quantum;
        double exposure_offset;
        double line_transfer_time;
        double transmit_setup_time;
        int transmit_overlap;
        int drop_frames;
        std::string dma_device_name;
    };

    typedef std::shared_ptr<Camwire_conf>  Camwire_conf_ptr;

    /* A few unusual camera-specific capabilities.  This struct exists
       because we only want to ask the camera once what it is capable of,
       and remember the results.  The more usual camera capabilities are
       recorded in the dc1394featureset_t in Camwire_user_data below.
       struct extra_features is initialized (by calloc() in function
       create()) to all zeros: */
    struct Extra_features
    {
        int single_shot_capable;  /* Flag.*/
        int gamma_capable;        /* Flag.*/
        uint16_t gamma_maxval;
        int colour_corr_capable;  /* Flag.*/
        Camwire_tiling tiling_value;
    };

    typedef std::shared_ptr<Extra_features> Extra_features_ptr;

    /* Internal camera state parameters.  If the current_set->shadow flag is
       set then, wherever possible, settings are read from the current_set
       member, else they are read directly from the camera hardware.  Each
       camwire handle structure contains a userdata pointer which is set to
       an instance of this structure.  It is initialized (by calloc() in
       function create()) to all zeros: */
    struct Camwire_user_data
    {
        int camera_connected;  /* Flag.*/
        int frame_lock;        /* Flag.*/
        int64_t frame_number;  /* About 300,000 years @ 1 million fps
                        before 63-bit overflow.*/
        int num_dma_buffers;   /* What capturing was set up with.*/
        double dma_timestamp;  /* Persistent record of last DMA buffer timestamp.*/
        Extra_features_ptr extras;
        dc1394featureset_t feature_set;
        std::shared_ptr<dc1394video_frame_t> frame;
        Camwire_conf_ptr config_cache;
        Camwire_state_ptr current_set;
    };

    typedef std::shared_ptr<dc1394camera_t>       Camera_handle;
    typedef std::shared_ptr<Camwire_user_data>    User_handle;

    struct Camwire_bus_handle
    {
        Camera_handle camera;
        User_handle userdata;
        /* Returns the dc1394camera_t camera handle for the given camwire
           handle.  Needed by many dc1394 functions in Camwire. */
        Camera_handle handle_get_camera()
        {
            return camera;
        }

        /* Returns a pointer to the user data structure for the given camwire
           handle.  Needed for internal status maintenance in Camwire. */
        User_handle handle_get_userdata()
        {
            return userdata;
        }

        int handle_set_userdata(User_handle user_data)
        {
            try
            {
                if(userdata)
                {
                    userdata = user_data;
                    return CAMWIRE_SUCCESS;
                }
                else return CAMWIRE_FAILURE;
            }
            catch(std::runtime_error &re)
            {
                DPRINTF("Failed to set user data for Camera handler");
                return CAMWIRE_FAILURE;
            }
        }

    };

    typedef std::shared_ptr<Camwire_bus_handle>   Camwire_bus_handle_ptr;
}

#endif
