#ifndef SOFACV_IMAGEFILTER_H
#define SOFACV_IMAGEFILTER_H

#ifdef SOFACV_NEW_GUI_ENABLED
#include "ImageFilter_newGUI.h"
#else
#include "ImageFilter_oldGUI.h"
#endif  // SOFACV_NEW_GUI_ENABLED

// int ImageFilterClass = sofa::core::RegisterObject("Base class for SofaOR's
// image processing filters")
//        .add<sofaor::processor::ImageFilter>();

#endif  // SOFACV_IMAGEFILTER_H
