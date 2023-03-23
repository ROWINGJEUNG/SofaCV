/********************************************************************
 Contributors:
    - bruno.josue.marques@inria.fr
********************************************************************/

#pragma once

namespace sofapython3::doc::cvmat
{
static auto Class =
        R"(
        Numpy binding for SofaCV's cvMat data types.
        )";

static auto __len__ =
        R"(
        Return the number of elements in the container's 1st
        dimension. (equivalent to shape(0))
        )";

static auto size =
        R"(
        Return the total number of elements in the container
        ( equivalent to math.prod(self.shape()) )
        )";

static auto ndim =
        R"(
        Return the number of dimensions in the container
        )";

static auto shape =
        R"(
        Return the shape of the container (a tuple of length ndim,
        with each value being equal to the length of the given dimension)
        )";

}  // namespace sofapython3::doc::datacontainer
