//#pragma once
//
//#include <sofa/config.h>
//
//#ifdef SOFA_BUILD_SOFACV
//#define SOFA_SOFACV_API SOFA_EXPORT_DYNAMIC_LIBRARY
//#else
//#define SOFA_SOFACV_API SOFA_IMPORT_DYNAMIC_LIBRARY
//#endif

#ifndef SOFACV_INITPLUGIN_H
#define SOFACV_INITPLUGIN_H

#include <sofa/config.h>

#ifdef SOFA_BUILD_SOFACV
#define SOFA_SOFACV_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_SOFACV_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#endif  // SOFACV_INITPLUGIN_H
