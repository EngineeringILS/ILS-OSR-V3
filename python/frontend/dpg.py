import dearpygui.dearpygui as dpg
import state_manager as state
import time
import controller_input as con
## Run Frontend Code Here, asynchronously call in data types from state_manager.py to read and display data.
dpg.create_context()

## Example code that demonstrates the concept, may not be asynchronous:
sensors = [
    state.BaterryVoltage("Baterry Voltage"),  
    state.RegolithCollected("Regolith"),
    state.ESP32Data("State")     
]
controller = con.controller()
buttnames = controller.input_name()
driverMode  = 1

        
def swichtDriverMode():
    # Add logic to swap driver mode
    for s in sensors:
        if isinstance(s, state.ESP32Data):
            s.toggleState()
    
    print("Driver Mode Swap")


#init all datatypes for DEARPYGUI
with dpg.value_registry():
    for s in sensors:
        dpg.add_string_value(tag=s.label(),default_value="")
    for n in  (buttnames):
        dpg.add_string_value(tag=n,default_value="0.0")

    #dpg.add_string_value(tag="Baterry Voltage",default_value="0.0")

# Creates  the main ROBOT GUI window and set tag and size
with dpg.window(label="Robot GUI",tag = "main window",width=800, height=800,):

    #Creates the first dropdown window in this case is are robot information
    with dpg.collapsing_header(label="Robot info"):
        # Everything created within this ^^^  will be in the dropdown menu called Robot info 
        
        #This (VVV) created a table here all the robot info will be display
        with dpg.group(horizontal=True):
            with dpg.table( header_row=True, width=350):

                # Makes the colum of the tables
                dpg.add_table_column(label="Robot")
                #Makes the rows

                for s in sensors:
                    with dpg.table_row():
                        #Adds text to the table
                        with dpg.group(horizontal=True):
                            
                            dpg.add_text(source=s.label())

            dpg.add_spacer(width=20)
            with dpg.table(header_row=True,width=350):
                dpg.add_table_column(label="Controller")

                for  n in buttnames:
                    with dpg.table_row():
                        dpg.add_text(source=n)

                    

            
        dpg.add_button(label="SWICHT DRIVER MODE", callback=swichtDriverMode)

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
    
    for s in sensors:
        dpg.set_value(s.label(), s.getData())
    
    controllerinputs = controller.get_controller_data()
    for i, n in enumerate(buttnames):
        dpg.set_value(str(n),f"{buttnames[i]}: {controllerinputs[i]}")
    
    dpg.render_dearpygui_frame()
    

dpg.destroy_context()
