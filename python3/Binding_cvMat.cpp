/*********************************************************************
Copyright 2019, CNRS, University of Lille, INRIA

This file is part of sofaPython3

sofaPython3 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

sofaPython3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with sofaqtquick. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/
/********************************************************************
 Contributors:
    - damien.marchal@univ-lille.fr
    - bruno.josue.marques@inria.fr
    - eve.le-guillou@centrale.centralelille.fr
    - jean-nicolas.brunet@inria.fr
    - thierry.gaugry@inria.fr
********************************************************************/


#include <sofa/defaulttype/DataTypeInfo.h>
using sofa::defaulttype::AbstractTypeInfo;

#include <sofa/core/objectmodel/BaseData.h>
using sofa::core::objectmodel::BaseData;

#include <sofa/core/objectmodel/BaseObject.h>
using  sofa::core::objectmodel::BaseObject;

#include <sofa/core/objectmodel/BaseNode.h>
using  sofa::core::objectmodel::BaseNode;

#include <SofaPython3/DataHelper.h>
#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <SofaPython3/Sofa/Core/Binding_BaseData.h>

#include <SofaCV/datatypes/cvMat.h>

#include "Binding_cvMat.h"
#include "Binding_cvMat_doc.h"

namespace sofapython3
{

py::buffer_info cvMatToBufferInfo(BaseData* m)
{
    sofa::Data<sofacv::cvMat>* matData = dynamic_cast<sofa::Data<sofacv::cvMat>*>(m);
    sofacv::cvMat* mat = matData->beginEdit();

    std::string format = "";
    size_t elemsize = 1;
    size_t channels = size_t(mat->channels());
    switch(mat->depth())
    {
    case CV_8U:
        format = py::format_descriptor<uint8_t>::format();
        elemsize = 1;
        break;
    case CV_8S:
        format = py::format_descriptor<int8_t>::format();
        elemsize = 1;
        break;
    case CV_16U:
        format = py::format_descriptor<uint16_t>::format();
        elemsize = 2;
        break;
    case CV_16S:
        format = py::format_descriptor<int16_t>::format();
        elemsize = 2;
        break;
    case CV_32S:
        format = py::format_descriptor<int32_t>::format();
        elemsize = 4;
        break;
    case CV_32F:
        format = py::format_descriptor<_Float32>::format();
        elemsize = 4;
        break;
    case CV_64F:
        format = py::format_descriptor<_Float64>::format();
        elemsize = 8;
        break;
    }

    std::vector<size_t> bufferdim;
    std::vector<size_t> strides;

    if (channels == 1)
    {
        bufferdim = {size_t(mat->rows), size_t(mat->cols)};
        strides = {elemsize * size_t(mat->cols), elemsize};
    }
    else
    {
        bufferdim = {size_t(mat->rows), size_t(mat->cols), channels};
        strides = {elemsize * size_t(mat->cols) * channels, elemsize * channels, elemsize};
    }
    size_t ndim = bufferdim.size();

    void* ptr = const_cast<void*>(reinterpret_cast<void*>(mat->data));

    py::buffer_info ninfo(
                ptr,              /* Pointer to buffer */
                long(elemsize),   /* Size of one scalar */
                format,           /* Python struct-style format descriptor */
                long(ndim),       /* Number of dimensions */
                bufferdim,        /* Buffer dimensions */
                strides           /* Strides (in bytes) for each index */
                );
    return ninfo;
}


py::array getPythonArrayForCVMat(cvMat* d)
{
    auto& memcache = getObjectCache();
    if(memcache.find(d) == memcache.end())
    {
        auto capsule = py::capsule(new Base::SPtr(d->getOwner()));

        py::buffer_info ninfo = cvMatToBufferInfo(d);
        py::array a(pybind11::dtype(ninfo), ninfo.shape,
                    ninfo.strides, ninfo.ptr, capsule);

        memcache[d] = a;
        return a;
    }
    return memcache[d];
}



void moduleAddcvMat(py::module& m)
{
    py::module::import("numpy");

    py::class_<cvMat, BaseData, raw_ptr<cvMat>> p(m, "cvMat",
                                                  py::buffer_protocol(), sofapython3::doc::cvmat::Class);

    PythonFactory::registerType("cvMat", [](BaseData* data) -> py::object {
        return py::cast(reinterpret_cast<cvMat*>(data));
    });

//    p.def("__getitem__", [](cvMat* self, py::object& index) -> py::object
//    {
//        py::array a = getPythonArrayFor(self);
//        py::buffer_info parentinfo = a.request();

//        return py::reinterpret_steal<py::object>(PyObject_GetItem(a.ptr(), index.ptr() ));
//    });

//    p.def("__setitem__", [](cvMat* self, size_t& index, py::object& value)
//    {
//        scoped_writeonly_access access(self);
//        setItem(getPythonArrayFor(self), py::slice(index, index+1, 1), value);
//        return py::none();
//    });

//    p.def("__setitem__", [](cvMat* self, py::slice& slice, py::object& value) -> py::object
//    {
//        scoped_writeonly_access access(self);

//        setItem(getPythonArrayFor(self), slice, value);
//        return py::none();
//    });

//    p.def("__setitem__", [](cvMat* self, py::tuple key, py::object& value)
//    {
//        scoped_writeonly_access access(self);

//        py::array a=getPythonArrayFor(self);
//        py::slice s0=toSlice(key[0]);
//        py::slice s1=toSlice(key[1]);

//        setItem2D(a, s0, s1, value);

//        return py::none();
//    });

    p.def("__getattr__", [](cvMat* self, const std::string& attrname){
        if(attrname == "value")
        {
            std::cout << "get attr value" << std::endl;
            return getPythonArrayForCVMat(self);
        }
        throw py::attribute_error("There is no attribute '"+attrname+"'");
    });

    p.def("apply", [](cvMat* self, py::function f)
    {
        scoped_write_access access(self);
        py::array a = getPythonArrayForCVMat(self);

        auto aa=a.mutable_unchecked<double>();
        for(auto i=0;i<aa.shape(0);++i){
            for(auto j=0;j<aa.shape(1);++j){
                aa(i,j) = py::cast<double>(f(i,j, aa(i,j)));
            }
        }
    });

    p.def("__str__", [](BaseData* self)
    {
        std::stringstream tmp;
        tmp << "SofaCV.cvMat <name='" << self->getName() << "', address=" << self << ">";
        return py::str(tmp.str());
    });

//    p.def("__repr__", [](BaseData* self)
//    {
//        std::stringstream tmp;
//        tmp << "SofaCV.cvMat <name='" << self->getName() << "', address=" << self << ">";
//        return py::str(tmp.str());
//    });

//    /// This is the standard terminology in numpy so we keep it for consistency purpose.
//    p.def_property_readonly("ndim", [](BaseData* self)
//    {
//        self->
//    }, sofapython3::doc::cvmat::ndim);    /// 1 or 2D array
//    p.def_property_readonly("shape", [](BaseData* self)
//    {
//        std::tuple<int, int> shape = getShape(self);
//        if(std::get<1>(shape)==-1)
//        {
//            py::tuple r {1};
//            r[0] = std::get<0>(shape);
//            return r;
//        }
//        py::tuple r {2};
//        r[0] = std::get<0>(shape);
//        r[1] = std::get<1>(shape);
//        return r;
//    }, sofapython3::doc::cvmat::shape);  /// array containing the size in each dimmension

//    p.def_property_readonly("size", &getSize, sofapython3::doc::cvmat::size);    /// get the total number of elements
//    p.def("__len__", [](cvMat* self)                     /// In numpy the len is the number of element in the first
//                                                                 /// dimmension.
//    {
//        auto nfo = self->getValueTypeInfo();
//        auto itemNfo = nfo->BaseType();

//        /// This is a 1D container
//        if( itemNfo->Container() )
//            return nfo->size(self->getValueVoidPtr()) / itemNfo->size();

//        return nfo->size(self->getValueVoidPtr());
//    }, sofapython3::doc::cvmat::__len__);

    p.def("array", [](cvMat* self){
        auto capsule = py::capsule(new Base::SPtr(self->getOwner()));
        py::buffer_info ninfo = cvMatToBufferInfo(self);
        py::array a(pybind11::dtype(ninfo), ninfo.shape,
                    ninfo.strides, ninfo.ptr, capsule);
        a.attr("flags").attr("writeable") = false;
        return a;
    });

    p.def("writeable", [](cvMat* self, py::object f) -> py::object
    {
        if(self!=nullptr)
            return py::cast(new cvMatContext(self, f));

        return py::none();
    });

    p.def("writeable", [](cvMat* self) -> py::object
    {
        if(self!=nullptr)
            return py::cast(new cvMatContext(self, py::none()));

        return py::none();
    });
}

void moduleAddcvMatContext(py::module& m)
{
    py::class_<cvMatContext> wa(m, "cvMatContextManager");
    wa.def("__enter__", [](cvMatContext& wa)
    {
        dynamic_cast<cvMat*>(wa.data)->beginEditVoidPtr();
        py::array mainbuffer = getPythonArrayForCVMat(dynamic_cast<cvMat*>(wa.data));
        py::buffer_info info = mainbuffer.request();
        wa.wrap = py::array(pybind11::dtype(info), info.shape,
                            info.strides, info.ptr, mainbuffer);

        if(!wa.fct.is_none())
            wa.wrap = wa.fct(wa.wrap);

        return wa.wrap;
    });

    wa.def("__exit__",
           [](cvMatContext& wa, py::object type, py::object value, py::object traceback)
    {
        SOFA_UNUSED(type);
        SOFA_UNUSED(value);
        SOFA_UNUSED(traceback);
        wa.wrap.attr("flags").attr("writeable") = false;
        wa.data->endEditVoidPtr();
    });
}

}
