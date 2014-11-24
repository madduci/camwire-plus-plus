Camwire++ [ ![Codeship Status for blackibiza/camwire-plus-plus](https://codeship.com/projects/78064ae0-55dd-0132-0d16-1ad869e028a1/status)](https://codeship.com/projects/49394)
=========
@author Michele Adduci <adducimi@informatik.hu-berlin.de>

A Modified C++11 Object-Oriented version of Camwire project
(http://kauri.auck.irl.cri.nz/~johanns/camwire/)

Original Author: Johann Schoonees <johann.schoonees@callaghaninnovation.govt.nz> 

Why Camwire?
Get going sooner with less pain

The traditional IEEE 1394 digital camera library on GNU/Linux is dc1394. 
dc1394 is a low-level wrapper around DCAM register set access, mixed 
with higher-level DMA transfer and buffering functions. It works if you 
treat it right but it bristles with thorns and traps. Understanding how 
to run your camera typically involves a long learning curve of 
segmentation faults and frozen cameras. There is almost no first-party 
documentation and I had to read most of the libdc1394 source code to be 
able to use it effectively.

For example: you might think of using the innocent-looking (libdc1394-1) 
function dc1394_set_format7_image_size() to change a running camera's 
region-of-interest. And you would be annoyed as I was when it instantly 
segfaults.

Or look at a more subtle gotcha in libdc1394-2: the function 
dc1394_get_color_coding_from_video_mode() succeeds or fails depending on 
the combination of its video_mode argument and the mode the camera 
happens to be in at the time. But who has time for grokking video modes 
when all you want to do is use the camera?

Some dc1394 functions are incompatible with preceding functions and 
there is no sure way of knowing which can be mixed, without reading a 
lot of source code. Camwire was written to ease the pain.
It's reliable

Camwire protects the application programmer from most of the gotchas of 
the dc1394 library. It contains work-arounds for known kernel bugs and 
avoids using camera registers that are implemented differently by 
different manufacturers. It is quite difficult to crash a camera with 
Camwire. Things generally work as expected and you don't need to know 
how much dirt has been swept under the carpet.

Camwire is being used for industrial-scientific purposes at my company 
and by many others around the world since 2004. Occasional bugs still 
turn up but the code is basically stable.
It's intuitive

Camwire focuses on the camera and its generic functions (such as taking 
pictures), not on the DCAM spec and the IEEE 1394 subsystem. Camwire 
allows you to think in terms of frame rates, pixel codings and shutter 
speeds, while dc1394 forces you to think in terms of formats, modes and 
bytes_per_packet. Camwire adds a few camera-centric extensions to the 
DCAM spec, such as frame numbers and timestamps of the trigger instants 
- things a camera user might want to know but which do not concern the 
bus implementers.
It's maintainable

Camwire is a higher-level API. Working at a higher level is less 
error-prone. Example: You don't need to know about DCAM Formats if you 
don't want to, but if you should decide to switch from Format 0 to 
Format 7, you may have to rewrite and debug a lot of code using dc1394. 
Different Formats need different dc1394 function calls. With Camwire, 
you edit one number in a configuration file and the function calls stay 
the same.
It's efficient

Camwire is thinly wrapped around dc1394 where it is safe to do so. The 
code only gets thick in places where you would have had to write a lot 
of glue yourself anyway. Camwire also offers a simple way of shadowing 
camera settings (caching a local copy) which can dramatically reduce the 
asynchronous traffic on the bus.
It's comprehensive (well, almost)

Most of the things you might want to do with a DCAM-compatible IEEE 1394 
digital camera are covered by Camwire. If it isn't, email me or submit a 
patch!
Why should I not use Camwire?

Camwire is intended for C/C++ programmers who would like to work at a 
higher level of abstraction than dc1394. If you want nuts-and-bolts 
access to the DCAM registers you will need to call dc1394 directly.

Camwire does not exploit every trick in the DCAM spec. The biggest hole 
is probably its not using absolute value registers. If you don't know 
what that is, you may not miss it.

If you are not a programmer and need a pre-made GUI interface, something 
like Coriander might be better.
Some features

    Camera manufacturer, model and serial number
    Run/stop camera
    Single-shot/continuous mode
    Trigger source internal/external
    External trigger polarity rising/falling
    Frame rate
    Shutter speed
    Frame dimensions width/height
    Frame offset left/top
    Pixel colour coding and depth
    White balance
    Frame capture by waiting/polling/copying
    Timestamp of last frame trigger
    Serial number of last frame
    Number of frame buffers
    Flush a number of frame buffers
    Optional camera register shadowing
    etc.

The C functions form an interface which is implementation-independent in 
the sense that it does not know or care whether the camera is on an IEEE 
1394 (Firewire, iLink), USB, CameraLink, or whatever bus. They are about 
camera features, not bus features. 
