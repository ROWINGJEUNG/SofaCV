import Sofa
import SofaCV
import sys
import cv2
import numpy as np

# This implementation can lead to unexpected resutls. It is an example that uses a PythonScriptController instead of a PyhtonScriptController. In case of an asynchronous grabber, the FrameViewer might display an image that is not, or only partially processed.
# class ImageFilter(Sofa.PythonScriptController):

#     def initGraph(self,node):
#         self.input = node.getObject('input') ## get pointer to engine component holding the image to manipulate
#         return 0


#     def onBeginAnimationStep(self,dt):
#         self.image = SofaCV.getCVMat(self.input.img_out) ## Call SofaOR.mat_as_numpy (declared in SofaORCommon/python/SofaOR/__init__.py) on the data of interest 
#         self.image[:] = cv2.flip(self.image, 1) ## Use 
#         self.img = cv2.Canny(self.image, 100, 200)
#         self.image[:] = cv2.cvtColor(self.img,cv2.COLOR_GRAY2RGB)

#         return 0

class ImageFilter(Sofa.PythonScriptDataEngine):

    def parse(self):
        self.addNewInput('img', value="@input.img_out")
        self.addNewOutput('img_out')
        
    def init(self):
        self.img_out = self.img
        return 0

    def onBeginAnimationStep(self,dt):
        self.image = SofaCV.getCVMat(self.img) ## Call SofaOR.mat_as_numpy (declared in SofaORCommon/python/SofaOR/__init__.py) on the data of interest 
        self.image[:] = cv2.flip(self.image, 1) ## Use 
        self.img = cv2.Canny(self.image, 100, 200)
        self.image[:] = cv2.cvtColor(self.img,cv2.COLOR_GRAY2RGB)

        self.image_out = SofaCV.getCVMat(self.img)
        self.image_out[:] = self.image
        return 0

    
