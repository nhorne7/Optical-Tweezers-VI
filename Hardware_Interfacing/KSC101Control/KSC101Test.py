import Hardware_Interfacing.KSC101Control.KSC101Interface
import time

KSC101 = Hardware_Interfacing.KSC101Control.KSC101Interface.CreateDevice("68801184")
KSC101Controller = Hardware_Interfacing.KSC101Control.KSC101Interface.KSC101Controller(KSC101)

KSC101Controller.initialize()

for i in range(5): # open and close the shutter 5 times.
    time.sleep(1)
    KSC101Controller.open()
    time.sleep(1)
    KSC101Controller.close()

KSC101Controller.release()