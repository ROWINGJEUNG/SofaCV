/********************************************************************
 Contributors:
    - bruno.josue.marques@inria.fr
********************************************************************/

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <sofa/core/objectmodel/BaseData.h>

namespace sofapython3
{
/// Makes an alias for the pybind11 namespace to increase readability.
namespace py { using namespace pybind11; }
using sofa::core::objectmodel::BaseData;

void moduleAddcvMat(py::module& m);
void moduleAddcvMatContext(py::module& m);


class cvMat : public BaseData {
public:
    ///TODO implementation ?
    py::tuple getShape() { return py::tuple(); }
    size_t getNDim() { return 3; }
    size_t getSize() { return 1; }
} ;

/// @brief A python context manager to control writeable state of a cvMat.
///
/// This context manager handle in a graceful way the writeable state of a cvMat
/// so the Data counter gets updated correctly when we leave the context.
///
/// It allows to implement stuff like that:
///   with myobject.position.arrayContext() as wa:
///         wa[3] = [1,2,3]
///         wa[4] = ...
///  # When we leave the context the data counter gets updated
///
/// NB: context is a classic python idom defined in
///    https://docs.python.org/3/reference/datamodel.html#context-managers
///
class cvMatContext
{
public:
    cvMatContext(BaseData* data_, py::object fct_) : data(data_), fct(fct_){}

    BaseData* data {nullptr};
    py::object wrap;
    py::object fct;
};

}
