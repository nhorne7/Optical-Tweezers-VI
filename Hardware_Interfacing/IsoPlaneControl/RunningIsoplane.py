# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 10:56:45 2024

@author: ohe000
"""
# NB! Other method: Simply check or open folder in upper right corner of Spyder
# Check that working directory is the right one
import os
print("Current working directory:")
print(os.getcwd())

# If not happy with cwd, change it
path = r"C:\Users\Public\Public Assets\Py-IsoPlane\Scripts\Olav Gaute"
os.chdir(path)
print("Current working directory:")
print(os.getcwd())

from Adv_features import *

spec = initialize(True)

spec.spec_grating = 600
spec.spec_unit = "nm"

spec.spec_center = 750

spec.speed = 1
spec.gain = 2

spec.t_exposure = 200

spec.set_params()

spec.auto_ROI(minwidth = 15)

te = spec.auto_exposure(Dyn = 0.5)

spec.acquire_background(te = te[0],N = 20)
# Get single spectrum
measurement = spec.grab_ROI(10)

measurement_mean = mean_frames(measurement)
wl = spec.get_wavelength()

data_dir = r"C:\Users\Public\Public Assets\Py-IsoPlane\Data\ohe000\Session 3 (29-8-2024)\SNG 1 (29-8-2024 12.15.15)\test"
name = "meas"
with open(data_dir+r"\{}.csv".format(name),mode = 'w',newline = '') as csvfile:
    writer=csv.writer(csvfile)
    for k in range(0,len(measurement_mean[0])):
        writer.writerow([wl[k],measurement_mean[0][k]])

plt.plot(wl,measurement_mean[0])
plt.show()

# Get step & glue

config_sng(spec, 500, 1000)

spec.buffer["steps"]

autoconfigSNG(spec,Dyn = 0.5,tot_time = [0,1,0])

spec.buffer["steps_te"]
spec.buffer["steps_n"]

res = adv_sng(spec,plot = True,clip_L = 0.2,clip_H = 0.02)

plt.plot(res[1],res[0])
plt.show()


