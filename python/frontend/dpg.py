import dearpygui.dearpygui as dpg
import state_manager as state
import time
import controller_input as con
## Run Frontend Code Here, asynchronously call in data types from state_manager.py to read and display data.
dpg.create_context()

## Example code that demonstrates the concept, may not be asynchronous:


# List of sensors, Add sensor here then it will display on gui
sensors = [
    state.BaterryVoltage("Baterry Voltage"),  
    state.RegolithCollected("Regolith"),
    state.ESP32Data("State")   ,
    state.IMU("IMU")
    

]

#Init varibles for gui
controller = con.controller()
buttnames = controller.input_name()
driverMode  = 1

global startTime 
startTime = 0
global stopTime 
stopTime = 0
global elapsedTime
elapsedTime = 0
global timerBool
timerBool = False

runTime =0
runTimeMin =0
runTimeSec=0

#image init
# loads in images from path 

Nasawidth, Nasaheight, Nasachannels, Nasadata = dpg.load_image("python/frontend/images/64px-NASA_logo.svg.png")
LunaLogoW, LunaLogoH, LunaLogoC, LunaLogoD = dpg.load_image("python/frontend/images/Final LOGO.png")
TAMULogoW, TAMULogoH, TAMULogoC, TAMULogoD = dpg.load_image("python/frontend/images/TAM-MaroonBox.png")

#Fuction to keep desired imaged desire location
def keepLogoBottom():
    # Get current window dimensions
    windowWidth = dpg.get_item_width("main window")
    windowLenght = dpg.get_item_height("main window")
    Ypos = windowLenght - Nasaheight -30

    # Calc image pos
    padding = 10
    posxNASA = windowWidth//2 - 200
    posyNASA = windowLenght - Nasaheight - padding - 20 
    
    dpg.set_item_pos("logoGroup", [posxNASA, Ypos])








# GUI buttons callback fuction here     
def swichtDriverMode():
    # Add logic to swap driver mode
    for s in sensors:
        if isinstance(s, state.ESP32Data):
            s.toggleDrivingMode()
    
    print("Driver Mode Swap")

def startRun():

    #Add fuction call to start the robot run


    global startTime
    global elapsedTime
    startTime = time.time()-elapsedTime
    global timerBool
    timerBool = True
    

def resetRun():
    #Add function call to reset run
    global timerBool
    timerBool = False
    global runTime , startTime , elapsedTime
    global runTimeMin 
    global runTimeSec

    runTime = startTime = elapsedTime =0
    runTimeMin =0
    runTimeSec=0


def stopRun():
    #Add function call to stop run
    global timerBool
    global runTime , startTime , elapsedTime
    if timerBool:
        elapsedTime = time.time() -startTime
        timerBool =False

def skipState():
    for s in sensors:
        if isinstance(s, state.ESP32Data):
            s.toggleState()




    


#Init of values diplay on gui
with dpg.value_registry():
    for s in sensors:
        dpg.add_string_value(tag=s.label(),default_value="")
    for n in  (buttnames):
        dpg.add_string_value(tag=n,default_value="0.0")
    
    dpg.add_string_value(tag="runTime", default_value= ("00:00"))
    
# Init images for gui here
with dpg.texture_registry():
    dpg.add_static_texture(width=Nasawidth, height=Nasaheight, default_value=Nasadata, tag="NASA")
    dpg.add_static_texture(width=LunaLogoW, height=LunaLogoH, default_value=LunaLogoD, tag="Lunabotics")
    dpg.add_static_texture(width=TAMULogoW, height=TAMULogoH, default_value=TAMULogoD, tag="TAMU")


    

# Creates  the main ROBOT GUI window and set tag and size
with dpg.window(label="Robot GUI",tag = "main window",width=800, height=800,):

    
    dpg.add_text(source="runTime")

    with dpg.group(horizontal=True):
        dpg.add_button(label="Start Run ", callback=startRun)
        dpg.add_button(label="Reset", callback=resetRun)
        dpg.add_button(label="Stop Run", callback=stopRun)


    #Creates the first dropdown window in this case is are robot information
    with dpg.collapsing_header(label="Robot info"):
        # Everything created within this ^^^  will be in the dropdown menu called Robot info 
        
        #This (VVV) created a table here all the robot info will be display
        with dpg.group(horizontal=True):
            with dpg.table( header_row=True, width=500):

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

                    
        dpg.add_spacer(height=20)
        with dpg.group(horizontal=True):    
            dpg.add_button(label="SWICHT DRIVER MODE", callback=swichtDriverMode)
            dpg.add_button(label="SKIP STATE", callback=skipState)
            

    #Creates A new dropdown window (Camera POV)
    with dpg.collapsing_header(label="Camera POV"):
        with dpg.table(header_row=True):
            dpg.add_table_column(label="ADD CAMERA POV HERE")

    # Creates image in the gui  
    with dpg.group(tag="logoGroup", horizontal=True, horizontal_spacing=100):      
        dpg.add_image("NASA", tag ="NASAimage",width=Nasawidth,height= Nasaheight)
        dpg.add_image("TAMU",uv_min= [0.17,.2], uv_max=[.78,.8] , tag = "TAMUimage", width=TAMULogoW//15,height= TAMULogoH//15)
        dpg.add_image("Lunabotics", tag = "Lunaimage", width=LunaLogoW//17,height= LunaLogoH//17)
        

# Callsback fuction that run in the backgroung
with dpg.item_handler_registry(tag="window handler"):
    dpg.add_item_resize_handler(callback=keepLogoBottom)
    
    
dpg.bind_item_handler_registry("main window", "window handler")
    
        

        


#DPGGUI Init

dpg.create_viewport(title='TAMU SEDS LUNABOTICS CONTROL PANEL', width=800, height=800)
dpg.set_primary_window("main window",True)
dpg.setup_dearpygui()
dpg.show_viewport()

# Replaces dpg.start_dearpygui() and manually does render loop so our data from the API can be fecth
while dpg.is_dearpygui_running():
    
    #Set dpg value for the sensors
    for s in sensors:
        dpg.set_value(s.label(), s.getData())
    
    #set dpg values for controller inputs
    controllerInputs = controller.get_controller_data()
    for i, n in enumerate(buttnames):
        dpg.set_value(str(n),f"{buttnames[i]}: {controllerInputs[i]}")
    
    #set dpg values for run time
    if timerBool:
        runTime =  time.time() - startTime
        runTimeMin =int( runTime / 60)
        runTimeSec = int(runTime%60)
        elapsedTime = runTime
    dpg.set_value("runTime",f"Run Time: {runTimeMin:02}:{runTimeSec:02} ")
    
    dpg.render_dearpygui_frame()
    

dpg.destroy_context()
