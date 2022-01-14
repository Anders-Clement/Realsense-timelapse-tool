import pyrealsense2.pyrealsense2 as rs
from pyrealsense2.pyrealsense2 import distortion
import numpy as np
import cv2
import time
from datetime import datetime
from pathlib import Path


class Realsense:

    def __init__(self, emitterState=True):
        self.name = "realsense"
        self.emitterState = emitterState
        print("({}): Initializing".format(self.name))
        self.setup()

    def setup(self):

        config = rs.config()

        #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(config)

        #Enable/Disable emitter
        if self.emitterState:
            device = self.profile.get_device()
            ds = device.query_sensors()[0]
            ds.set_option(rs.option.emitter_enabled, 1)
        else:
            device = self.profile.get_device()
            ds = device.query_sensors()[0]
            ds.set_option(rs.option.emitter_enabled, 0)
        

    def waitAndGetFrame(self):
        frames = self.pipeline.wait_for_frames()

        color = frames.get_color_frame()
        #depth = frames.get_depth_frame()

        im_rgb = np.array(color.get_data())
        return im_rgb

    
    def stop(self):
        self.pipeline.stop()
        print('Stopping Realsense camera')


    def get_info(self, cfg):
        info = dict()
        info["name"] = self.name
        streams = cfg.get_streams()
        for s in streams:
            try:
                intrinsics = s.as_video_stream_profile().get_intrinsics()
            except Exception as e:
                print(e)
                
            values = dict()
            values["coeffs"] = intrinsics.coeffs
            values["fx"] = intrinsics.fx
            values["fy"] = intrinsics.fy
            values["height"] = intrinsics.height
            values["width"] = intrinsics.width
            values["ppx"] = intrinsics.ppx
            values["ppy"] = intrinsics.ppy
            
            if intrinsics.model == distortion.brown_conrady:
                values["model"] = 4
            elif intrinsics.model == distortion.ftheta:
                values["model"] = 3
            elif intrinsics.model == distortion.inverse_brown_conrady:
                values["model"] = 2
            elif intrinsics.model == distortion.kannala_brandt4:
                values["model"] = 1
            elif intrinsics.model == distortion.none:
                values["model"] = 0

            values["type"] = s.stream_type()
            info[s.stream_name()] = values
        return info


def convertToVideo(path):
    if not Path.exists(path):
        return False
    imageFileNames = Path(path).glob('*.jpg')
    imageList = []
    for imageFileName in imageFileNames:
        imageList.append(imageFileName)

    imageList.sort(key=lambda x: int(x.stem))

    videoPath = Path(str(path) + '/video.avi')
    videoWriter = cv2.VideoWriter(str(videoPath), cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1920, 1080))
    print('Writing video to: ', videoPath)
    for i, imagePath in enumerate(imageList):
        if imageFileName.is_file():
            frame = cv2.imread(str(imagePath))
            videoWriter.write(frame)
            print('\r', int(100*i/len(imageList)), '% done...      ', end='')
        else:
            print('Not a file? ', imagePath)
    print('\rDone saving video!                          ')
    videoWriter.release()
    return True
    
    
def recordVideo():
    cam = Realsense()
    print('Timelapse recorder program, using Intel Realsense camera\n \
           You can choose target framerate and set a max duration if desired.')
    fps = input('Input target FPS: ')
    
    runTimeInMinutes = input('Input stop time after x minutes, -1 to never stop: ')

    convertToVid = input('Convert to video when done? [y/n]: ')
    if convertToVid == 'y':
        convertToVid = True
    elif convertToVid == 'n':
        convertToVid = False
    else:
        print('Wrong input. Try again dummy.')
        exit(-1)
    
    try:
        fps = int(fps)
        runTimeInSeconds = int(runTimeInMinutes) * 60
    except Exception as e:
        print(e)
        exit(-1)

    if runTimeInSeconds < 0:
        runTimeInSeconds = 10000000000 # run for ~300 years

    stopTime = time.time() + runTimeInSeconds
    Ts = 1/fps
    lastSampleTime = time.time()

    # make folder for images
    path = Path(str(Path.cwd()) + "/timelapse/" + datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    Path.mkdir(path, parents=True, exist_ok=True)
    print('Saving images to path: ', path)

    frameNum = 0
    while True:
        now = time.time()
        if now < stopTime and lastSampleTime + Ts < now:
            lastSampleTime = now
            frame = cam.waitAndGetFrame()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # cv2.imshow('img', frame)
            # cv2.waitKey(1)
            savePath = f"{path}\\{frameNum}.jpg"
            cv2.imwrite(savePath, frame)
            frameNum += 1
        elif now > stopTime:
            break
        else:
            time.sleep(.01)

    cam.stop()
    cv2.destroyAllWindows()
    if convertToVid:
        if not convertToVideo(path):
            print('Failed to make the video...')



if __name__ == '__main__':
    recordVideo()
    #p = Path("C:\\Users\\ander\\timelapse\\2021-12-22_17-05-27")
    #convertToVideo(p)
