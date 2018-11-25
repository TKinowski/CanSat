from time import sleep
from picamera import PiCamera
import threading

class camera (PiCamera, threading.Thread):
	def __init__(self, threadID, name, cam_dir = '/mnt/fat/', preview=False, verbose = False, delay = 2):

		#threading setup
		threading.Thread.__init__(self)
		self.name = name
		self.threadID = threadID
		self._end = False

		#camera setup
		if (not self.try_setup()):
			return
		
		self.use_preview = preview
		self.counter = 0
		self._verbose = verbose
		self.name_base = cam_dir + "img"
		self._delay = delay
	
	def try_setup(self):
		self.state = 1
		try:
			PiCamera.__init__(self, resolution = (3280, 2464))
		except:
			self.state = 0
		return self.state


	def apply_settings(self):
		#self.iso = 100
		#self.shutter_speed = 1000
		self.awb_mode = 'auto'
		sleep(3)
		g = self.awb_gains
		self.awb_mode = 'off'
		self.awb_gains = g
		self.brightness = 50
		self.contrast = 0
		self.exposure_mode = 'off'
		sleep(1)

	def run(self):
		self.apply_settings()
		if self.use_preview:
			self.start_preview(fullscreen = False, window = (800,100,640,480))
		while not self._end:
			try:
				if not self._verbose:
					self.saveImage()
				sleep(self._delay)
			except:
				pass
			finally:
				pass
		if self.use_preview:
			self.stop_preview()
		self.close()
		
	def saveImage(self):
		filename = self.name_base + '_%05d.jpg' % self.counter
		self.capture(filename)
		if self._verbose:
			print('Captured %s' % filename)
		self.counter += 1

	def endThread(self):
		self._end = True



if __name__=="__main__":
	try:
		cam = camera(1, name = "test", preview=True, verbose=True)
		cam.start()
		raw_input("Press Enter to start...")
		while True:
			cam.saveImage()
			raw_input("next - enter, stop - ctrl+C")
	except KeyboardInterrupt:
		pass
	finally:
		cam.endThread()
		cam.join()
