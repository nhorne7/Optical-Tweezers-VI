import clr
import os
import time
kinesis_path = r"C:\Program Files\Thorlabs\Kinesis" # Replace this with your own KINESIS Path!

# Instantiation of CLR namespaces for DeviceManagerCLI and SolenoidCLI (order matters)
clr.AddReference(os.path.join(kinesis_path, "Thorlabs.MotionControl.DeviceManagerCLI.dll"))
clr.AddReference(os.path.join(kinesis_path, "Thorlabs.MotionControl.GenericMotorCLI.dll"))
clr.AddReference(os.path.join(kinesis_path, "Thorlabs.MotionControl.KCube.SolenoidCLI.dll"))
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.KCube.SolenoidCLI import *


# SERIAL_NUM = "68801184"

def CreateDevice(SERIAL_NUM):
    DeviceManagerCLI.BuildDeviceList()
    device = KCubeSolenoid.CreateKCubeSolenoid(SERIAL_NUM)
    device.Connect(SERIAL_NUM)
    print(f"Device under SERIAL NO. {SERIAL_NUM} created")
    return device


class KSC101Controller:
    DEBOUNCE_DELAY = 0.25
    def __init__(self, device):
        self.device = device
        self.last_command_time = 0
        self.is_open = False
        print(f"KSC101 Controller Object created")


    def initialize(self):
        if not self.device.IsSettingsInitialized():
            print("Attempting KSC101 device Initialization..")
            self.device.WaitForSettingsInitialized(10000)
            assert self.device.IsSettingsInitialized() is True
        self.device.StartPolling(100)
        self.device.EnableDevice()
        device_info = self.device.GetDeviceInfo()
        print(f"[{device_info.Description}] is ONLINE in MANUAL")
        self.device.SetOperatingMode(SolenoidStatus.OperatingModes.Manual)
        self.open()
    
    
    def release(self):
        device_info = self.device.GetDeviceInfo()
        self.close()
        print(f'Disconnecting [{device_info.Description}]')
        self.device.StopPolling()
        self.device.Disconnect()
        print(f"[{device_info.Description}] is OFFLINE")

    
    def set_state(self, state):
        current_time = time.time()
        if current_time - self.last_command_time < self.DEBOUNCE_DELAY:
            print("Debounced State Change: (Too Fast)")
            return
        if self.device.GetOperatingState() == state:
            print("Redundant Call")
            return
        if self.device.GetOperatingMode() != SolenoidStatus.OperatingModes.Manual:
            self.device.SetOperatingMode(SolenoidStatus.OperatingModes.Manual)
        if state == SolenoidStatus.OperatingStates.Active:
            self.device.SetOperatingState(state)
            print("Shutter OPEN")
            self.is_open = True
        elif state == SolenoidStatus.OperatingStates.Inactive:
            self.device.SetOperatingState(state)
            print("Shutter CLOSE")
            self.is_open = False

        self.last_command_time = current_time

    def open(self):
        current_time = time.time()
        if current_time - self.last_command_time < self.DEBOUNCE_DELAY:
            print("Debounced State Change: (Too Fast)")
            return
        if self.device.GetOperatingState() == SolenoidStatus.OperatingStates.Active:
            print("Redundant Call")
            return
        if self.device.GetOperatingMode() != SolenoidStatus.OperatingModes.Manual:
            self.device.SetOperatingMode(SolenoidStatus.OperatingModes.Manual)
        self.device.SetOperatingState(SolenoidStatus.OperatingStates.Active)
        self.last_command_time = current_time
        self.is_open = True
        print("Shutter OPEN")
    
    def close(self):
        current_time = time.time()
        if current_time - self.last_command_time < self.DEBOUNCE_DELAY:
            print("Debounced State Change: (Too Fast)")
            return
        if self.device.GetOperatingState() == SolenoidStatus.OperatingStates.Inactive:
            print("Redundant Call")
            return
        if self.device.GetOperatingMode() != SolenoidStatus.OperatingModes.Manual:
            self.device.SetOperatingMode(SolenoidStatus.OperatingModes.Manual)
        self.device.SetOperatingState(SolenoidStatus.OperatingStates.Inactive)
        self.last_command_time = current_time
        self.is_open = False
        print("Shutter CLOSE")