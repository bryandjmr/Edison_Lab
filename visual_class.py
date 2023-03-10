import fresnel
import PIL
import numpy as np
import os

class visual:
    def __init__(self, bead_radius):
        self.r = bead_radius
        self.original = os.getcwd()
        os.chdir("/Users/bryan/Desktop/College/Research/Edison/point_sphere/images")
        self.counter = 0
   

    def add_images(self, position_array):
        self.counter += 1
        
        scene = fresnel.Scene()
        
        geometry_1 = fresnel.geometry.Sphere(scene, position = position_array, radius=self.r, 
            material = fresnel.material.Material(color=fresnel.color.linear([1,0,0])))
        
        geometry_2 = fresnel.geometry.Sphere(scene, np.array([0,0,0]), radius=1, 
            material = fresnel.material.Material(color=fresnel.color.linear([1,1,1])))

        scene.camera = fresnel.camera.Orthographic.fit(scene)

        self.save_images(scene)

    def save_images(self, figure):
        image = fresnel.preview(figure)
        image_saved = PIL.Image.fromarray(image[:])
        image_saved.save('simg_{0:03}.png'.format(self.counter))


    def create_video(self, frame_rate=0.5):
        os.system('convert -delay 15 ../images/simg_*.png -loop 1 movie.gif')
        #[os.remove(file) for file in os.listdir('../images') if file.endswith('.png')]
        os.chdir(self.original)
