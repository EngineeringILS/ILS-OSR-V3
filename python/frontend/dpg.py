import dearpygui.dearpygui as dpg
import state_manager as state

## Run Frontend Code Here, asynchronously call in data types from state_manager.py to read and display data.
dpg.create_context()

## Example code that demonstrates the concept, may not be asynchronous:
BatteryData = state.DerivedDataInput 
print(BatteryData.data)