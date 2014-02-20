#ifndef CAMWIRE_MACROS_HPP
#define CAMWIRE_MACROS_HPP
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


    Title: Macro header for Camwire modules

    Description:
    This header file provides some macros for printing debugging
    information and for error status returns to make the code more
    readable and maintainable.  This header should only be included in
    implementation-specific files, i.e. into files that contain `1394'
    in the file name.

Camwire++: Michele Adduci <info@micheleadduci.net>
***********************************************************************/

#include <iostream>
#include <dc1394/types.h>

/* Print debugging error message: */
#ifdef CAMWIRE_DEBUG
#define DPRINTF(m) std::err << "In " << __FILE__ << " line "<< __LINE__ << " function " << __func__ << ": " << m << std::endl;
#else
#define DPRINTF(m) std::cout << m << std::endl;
#endif

/* If consistently used, it should be possible to change these return
   codes without breaking anything: */
const int CAMWIRE_SUCCESS = 1;  /* true */
const int CAMWIRE_FAILURE = 0;  /* false */

/* Function error status returns, for readability and
   maintainability: */
#define ERROR_IF_NULL(p) \
    if ((p)==NULL) {DPRINTF("Null pointer."); return(CAMWIRE_FAILURE);}
#define ERROR_IF_ZERO(v) \
    if ((v)==0) {DPRINTF("Bad (zero) value."); return(CAMWIRE_FAILURE);}
#define ERROR_IF_CAMWIRE_FAIL(r) \
    if ((r)!=CAMWIRE_SUCCESS) \
      {DPRINTF("Camwire function call failed."); \
       return(CAMWIRE_FAILURE);}
#define ERROR_IF_DC1394_FAIL(r)	\
    if ((r)!= DC1394_SUCCESS) \
      {DPRINTF("dc1394 function call failed."); \
       return(CAMWIRE_FAILURE);}

#define CONFFILE_EXTENSION		".conf"
#define ENVIRONMENT_VAR_CONF	"CAMWIRE_CONF"

/*
    Since libdc1394 doesn't offer and "Invalid video mode" enum type, here I add it:
*/

static const dc1394video_mode_t DC1394_VIDEO_MODE_INVALID = static_cast<dc1394video_mode_t>(0);

#endif /* ndef CAMWIREMACROS_H */
