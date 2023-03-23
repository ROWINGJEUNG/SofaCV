#include "Binding_cvMatData.h"
#include <SofaPython/Binding_Data.h>
#include <SofaPython/PythonToSofa.inl>

using namespace sofa::core::objectmodel;
using namespace sofa::defaulttype;

void depthAsString(unsigned int d, std::string& s)
{
    switch (d)
    {
    case CV_64F:
        s = "double";
        break;
    case CV_32F:
        s = "float";
        break;
    case CV_32S:
        s = "int";
        break;
    case CV_16S:
        s = "short";
        break;
    case CV_16U:
        s = "unsigned short";
        break;
    case CV_8S:
        s = "char";
        break;
    case CV_8U:
    default:
        s = "unsigned char";
        break;
    }
}

unsigned int dtypeAsDepth(const std::string& s)
{
    if (s == "int8")
        return CV_8S;
    else if (s == "int16")
        return CV_16S;
    else if (s == "int32")
        return CV_32S;
    else if (s == "uint8")
        return CV_8U;
    else if (s == "uint16")
        return CV_16U;
    else if (s == "float32")
        return CV_32F;
    else if (s == "float64")
        return CV_64F;
    else
        return CV_8U;
}

static PyObject* cvMatData_getPtr(PyObject* self, PyObject* /*args*/)
{
  Data<sofacv::cvMat>* data =
      sofa::py::unwrap<Data<sofacv::cvMat> >(self);
  const sofacv::cvMat& image =
      data->getValue();  // where should be the endedit?

  PyObject* imgdata = PyLong_FromVoidPtr(image.data); // data


  PyObject* shape = PyTuple_New(3);
  PyTuple_SetItem(shape, 0, PyInt_FromSsize_t(image.rows));        // rows
  PyTuple_SetItem(shape, 1, PyInt_FromSsize_t(image.cols));        // cols
  PyTuple_SetItem(shape, 2, PyInt_FromSsize_t(image.channels()));  // channels

  // output = tuple( list(pointers), shape tuple, type name)
  PyObject* res = PyTuple_New(3);

  // the data pointer
  PyTuple_SetItem(res, 0, imgdata);

  // the shape
  PyTuple_SetItem(res, 1, shape);

  // the channel depth as a recognisable string for ctypeFromName (c.f SofaNumpy
  // in SofaPython plugin)

  std::string s = "";
  depthAsString(image.depth(), s);
  PyTuple_SetItem(res, 2, PyString_FromString(s.c_str()));

  return res;
}

static PyObject* cvMatData_getData(PyObject* self, PyObject* /*args*/)
{
  Data<sofacv::cvMat>* data =
      sofa::py::unwrap<Data<sofacv::cvMat> >(self);
  sofacv::cvMat& image =
      *data->beginEdit();  // where should be the endedit?

  PyObject* imgdata = PyString_FromStringAndSize(
      reinterpret_cast<char*>(image.data), long(image.total() * image.elemSize()));
  PyObject* shape = PyTuple_New(3);
  PyTuple_SetItem(shape, 0, PyInt_FromSsize_t(image.rows));        // rows
  PyTuple_SetItem(shape, 1, PyInt_FromSsize_t(image.cols));        // cols
  PyTuple_SetItem(shape, 2, PyInt_FromSsize_t(image.channels()));  // channels

  // output = tuple( list(pointers), shape tuple, type name)
  PyObject* res = PyTuple_New(3);

  // the data pointer
  PyTuple_SetItem(res, 0, imgdata);

  // the shape
  PyTuple_SetItem(res, 1, shape);

  // the type name
  std::string s = "";
  depthAsString(image.depth(), s);

  PyTuple_SetItem(res, 2, PyString_FromString(s.c_str()));
  data->endEdit();
  return res;
}

static PyObject* cvMatData_reshape(PyObject* self, PyObject* args)
{
    Data<sofacv::cvMat>* data =
        sofa::py::unwrap<Data<sofacv::cvMat> >(self);

    char* depth;
    PyObject* shape;
    int nx, ny, nz;
    if (!PyArg_ParseTuple(args, "Os",&shape, &depth))
    {
        return nullptr;
    }
    if (!PyArg_ParseTuple(shape, "iii",&nx, &ny, &nz))
    {
        return nullptr;
    }
    data->setValue(cv::Mat(nx,ny, CV_MAKETYPE(dtypeAsDepth(depth), nz)));
    return self;

}

SP_CLASS_METHODS_BEGIN(cvMatData)
SP_CLASS_METHOD_DOC(cvMatData, getPtr,
                    "retrieves a Tuple containing:\n 1. The pointer to "
                    "Mat::data (as a numpy array)\n 2. The dimensions of the "
                    "matrix (rows, cols, channels, depth\n 3. The name of the "
                    "Sofa type ('cvMat').")
SP_CLASS_METHOD_DOC(cvMatData, getData,
                    "retrieves a Tuple containing:\n 1. a deep copy of the "
                    "Mat::data byte array (as a numpy array)\n 2. The "
                    "dimensions of the matrix (rows, cols, channels, depth\n "
                    "3. The name of the Sofa type ('cvMat').")
SP_CLASS_METHOD_DOC(cvMatData, reshape,
                    "To call before changing the shape of a python-bound sofa"
                    "cvMat data. Changes the shape of the matrix")
SP_CLASS_METHODS_END

// eventual attributes
SP_CLASS_ATTRS_BEGIN(cvMatData)
// SP_CLASS_ATTR(CVMat, paramName)
SP_CLASS_ATTRS_END

SP_CLASS_TYPE_PTR_ATTR(cvMatData, sofa::Data<sofacv::cvMat>, Data)
