/******************************************************************************
 *       SOFAOR, SOFA plugin for the Operating Room, development version       *
 *                        (c) 2017 INRIA, MIMESIS Team                         *
 *                                                                             *
 * This program is a free software; you can redistribute it and/or modify it   *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 1.0 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: Bruno Marques and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact-mimesis@inria.fr                               *
 ******************************************************************************/

#ifndef SOFACV_DATASLIDERMGR_H
#define SOFACV_DATASLIDERMGR_H

#include <SofaCV/SofaCV.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofacv
{
struct DSM
{
  virtual ~DSM() {}
  virtual void createSlider(const std::string& winName) = 0;
};

template <class T>
struct ScalarSliderManager : DSM
{
  ScalarSliderManager(sofa::Data<T>* d, T min = T(0), T max = T(255),
                      T step = T(1));
  ~ScalarSliderManager() {}

  sofa::Data<T>* m_data;
  T m_min, m_max, m_step;

  void createSlider(const std::string& winName);
  int getTrackbarMaxValue();
  int getTrackbarRangedValue();
  static void callback(int, void*);
};

template <class T, class U>
struct CustomSliderManager : DSM
{
  CustomSliderManager(sofa::Data<T>* d, U min = U(0), U max = U(255),
                      U step = U(1));
  ~CustomSliderManager() {}

  sofa::Data<T>* m_data;
  U m_min, m_max, m_step;

  void createSlider(const std::string& winName);
  int getTrackbarMaxValue();
  int getTrackbarRangedValue();
  static void callback(int, void*);
};

template <unsigned int N, class U>
struct VecSliderManager : DSM
{
  VecSliderManager(sofa::Data<sofa::defaulttype::Vec<N, U> >* d, U min = U(0),
                   U max = U(255), U step = U(1));
  ~VecSliderManager() {}

  void createSlider(const std::string& winName);

  int getTrackbarMaxValue();
  int getTrackbarRangedValueX();
  int getTrackbarRangedValueY();
  int getTrackbarRangedValueZ();
  int getTrackbarRangedValueW();

  static void callback_x(int val, void* mgr);
  static void callback_y(int val, void* mgr);
  static void callback_z(int val, void* mgr);
  static void callback_w(int val, void* mgr);

  sofa::Data<sofa::defaulttype::Vec<N, U> >* m_data;
  U m_min, m_max, m_step;
};

}  // namespace sofacv
#endif  // SOFACV_DATASLIDERMGR_H
