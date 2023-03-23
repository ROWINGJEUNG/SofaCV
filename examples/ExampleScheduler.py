
import Sofa
import SofaCV

def createScene(node):
    node.createObject('RequiredPlugin', pluginName='SofaCV')
    node.createObject('RequiredPlugin', pluginName='SofaPython')

    # Async has to be "false" for now when using a scheduler
    node.createObject('VideoGrabber', name='input', videoFile="../resources/vtest.avi", interpolate=False, async="false", printlog="true")
    # The Scheduler "registers" a grabber and takes control over input dispatch. A DT lower than the simulation's will release a new frame at each step (possibly discarding some if schedulerDt < 0.5 * simuDt), while a higher DT might wait multiple steps before releasing a new frame.  
    node.createObject('DTScheduler', name='input_scheduler', grabber="@input", dt=0.01)
    node.createObject('FrameViewer', name='viewer', img='@input.img_out', corners="-1 1 0 1 1 0 1 -1 0 -1 -1 0", mode="ORTHO")
    
    return 0
