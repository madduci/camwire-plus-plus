#ifndef CAMERAHANDLE_H
#define CAMERAHANDLE_H
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


    Title: Camwire handle definition

    Description: Header file to provide the definition of the Camwire
    handle struct.

    This header would not be needed for normal use of the Camwire
    library.  It should only be included in user code that wishes to
    access dc1394 functions directly.

***********************************************************************/



/* Below is the type definition of the camwire handle structure.  The
   camwire handle itself is typedefed as a pointer to this structure.

   The user need not know this structure nor should anyone use its
   members directly.  The camwire_bus_get_... functions defined in this
   module provide efficient maintainable access.

   The camera member is needed by many libdc1394 functions.  It is a
   handle to the current IEEE 1394 camera connected to an OHCI port and
   node.

   The userdata member provides a way for a user (in our case this is
   the Camwire main module) to associate private data with the camwire
   handle, such as status information.  A pointer to the user data
   structure can be stored in userdata by the camwire_bus_set_userdata()
   function, without this module needing to know the user data structure
   definition.  */


#endif /* CAMERAHANDLE_H */
