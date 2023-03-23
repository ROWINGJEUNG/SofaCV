import Sofa
try:
    import numpy
    import SofaPython.SofaNumpy

    def getCVMat(img):
        '''maps image content as a numpy array with shared memory'''

        ptr, shape, typename = img.getPtr()

        t = SofaPython.SofaNumpy.ctypeFromName.get(typename, None)
        if not type: raise Exception("can't map image of type " + typename)
        array = SofaPython.SofaNumpy.ctypes.cast(SofaPython.SofaNumpy.ctypes.c_void_p(ptr), SofaPython.SofaNumpy.ctypes.POINTER(t))
        if 0 in shape:
            Sofa.msg_error("SofaCV::mat_as_numpy", "Sofa::cvMat has invalid shape")
            return None
        nparr = numpy.ctypeslib.as_array(array, shape)
        return nparr

    def setCVMat(img, data):
        # check that img has correct shape (rows, cols, channels)
        if len(img.shape) != 3:
            img = img.reshape(img.shape[0],img.shape[1], 1)
        # retrive pointer to data
        image = getCVMat(data)
        # Checks that the data pointer has the correct shape and depth type, reshape if necessary
        if image is None or image.shape != img.shape or image.dtype != img.dtype:
            data.reshape(img.shape, str(img.dtype))
        # Set img in the datafield
        image[:] = img


except ImportError: # numpy is not mandatory
    pass
