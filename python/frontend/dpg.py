import dearpygui.dearpygui as dpg
import state_manager as state
import time
import threading
## Run Frontend Code Here, asynchronously call in data types from state_manager.py to read and display data.
dpg.create_context()

## Example code that demonstrates the concept, may not be asynchronous:
batterydata = state.BatteryData(1,4.0)

        



#init all datatypes for DEARPYGUI
with dpg.value_registry():
    dpg.add_string_value(tag="Volt",default_value="0.0")

# Creates  the main ROBOT GUI window and set tag and size
with dpg.window(label="Robot GUI",tag = "main window",width=800, height=800,):

    #Creates the first dropdown window in this case is are robot information
    with dpg.collapsing_header(label="Robot info"):
        # Everything created within this ^^^  will be in the dropdown menu called Robot info 
        
        #This (VVV) created a table here all the robot info will be display
        with dpg.table(header_row=True):

            # Makes the colum of the tables
            dpg.add_table_column(label="Robot")
            dpg.add_table_column(label="Controller")
            
            #Makes the rows
            with dpg.table_row():
                #Adds text to the table
                dpg.add_text("Volt of baterry", source="Volt")
                dpg.add_text("Controler input")
                
            with dpg.table_row():
                dpg.add_text("")
                dpg.add_text("Controler input3")

    #Creates A new dropdown window (Camera POV)
    with dpg.collapsing_header(label="Camera POV"):
        with dpg.table(header_row=True):
            dpg.add_table_column(label="ADD CAMERA POV HERE")

    #with dpg.collapsing_header(label="Inputs"):
        

        




dpg.create_viewport(title='Control Panel', width=800, height=600)
dpg.set_primary_window("main window",True)
dpg.setup_dearpygui()
dpg.show_viewport()

# Replaces dpg.start_dearpygui() and manually does render loop so our data from the API can be fecth
while dpg.is_dearpygui_running():
    v=batterydata.read()
    dpg.set_value("Volt",f"{v:.4f}")
    dpg.render_dearpygui_frame()
    

dpg.destroy_context()
