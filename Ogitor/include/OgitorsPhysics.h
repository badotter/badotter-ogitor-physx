/*/////////////////////////////////////////////////////////////////////////////////
/// An
///    ___   ____ ___ _____ ___  ____
///   / _ \ / ___|_ _|_   _/ _ \|  _ \
///  | | | | |  _ | |  | || | | | |_) |
///  | |_| | |_| || |  | || |_| |  _ <
///   \___/ \____|___| |_| \___/|_| \_\
///                              File
///
/// Copyright (c) 2008-2010 Ismail TARIM <ismail@royalspor.com> and the Ogitor Team
//
/// The MIT License
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
////////////////////////////////////////////////////////////////////////////////*/

#pragma once

#include "OgitorsSingleton.h"


namespace Ogitors
{
    //! Ogitor Physics class
    /*!  
        A class that is responsible for handling physics requests and extra utility functions
    */
    class OgitorExport OgitorsPhysics: public Singleton<OgitorsPhysics>, public Ogre::GeneralAllocatedObject
    {
    public:
        /**
        * Constructor
        */
        OgitorsPhysics();
        /**
        * Destructor
        */
        virtual ~OgitorsPhysics();
    };

    //! Dummy class
    /*!  
        Dummy class with empty implementations
        @deprecated
    */
    class OgitorsDummyPhysics: public OgitorsPhysics
    {
    public:
        /**
        * @copydoc OgitorsPhysics::OgitorsPhysics()
        */
        OgitorsDummyPhysics() {};
        /**
        * @copydoc OgitorsPhysics::~OgitorsPhysics()
        */
        virtual ~OgitorsDummyPhysics() {};
    };
}
