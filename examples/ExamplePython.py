
import Sofa.Core
import SofaCV
import cv2

class MyController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.parent = kwargs.get('parent')
        
    def onAnimateBeginEvent(self, e):
        print(e)
        imgData = self.parent.input.img_out
        print(imgData)
        print(imgData.value)
        cv2.imwrite("plop.jpg", imgData.value)

        
def createScene(node):
    node.addObject('RequiredPlugin', name='SofaCV')
    # node.addObject('VideoGrabber', name='input', videoFile="/home/bruno/dev/similar/plugins/SofaCV/resources/test_video.avi")

    node.addObject("VideoGrabber", name="input", cam_index=0)
    node.addObject(MyController(name="MyController", parent=node))
    node.addObject('FrameViewer', name='viewer', img='@input.img_out', corners="-1 1 0 1 1 0 1 -1 0 -1 -1 0", mode="ORTHO")
    
    return 0
