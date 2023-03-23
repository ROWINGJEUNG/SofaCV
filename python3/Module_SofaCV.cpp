/********************************************************************
 Contributors:
    - bruno.josue.marques@inria.fr
********************************************************************/

#include <pybind11/eval.h>
namespace py = pybind11;

#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <SofaPython3/Sofa/Core/Binding_BaseObject.h>
#include <SofaPython3/Sofa/Core/Binding_BaseData.h>
#include <SofaPython3/Sofa/Core/Data/Binding_DataContainer.h>
#include "Binding_cvMat.h"

#include <SofaCV/SofaCV.h>

namespace sofapython3
{

PYBIND11_MODULE(SofaCV, m) {
    m.doc() = R"doc(
              Binding for the SofaCV plugin
              -----------------------------------

              Provides python bindings for the SofaCV module

              Example of use:

              .. code-block:: python

                import SofaCV

              .. autosummary::
                  :toctree: _autosummary/_autosummary


              )doc";

    py::module::import("Sofa.Core");

    moduleAddcvMat(m);
    moduleAddcvMatContext(m);
}

}  // namespace sofapython3

