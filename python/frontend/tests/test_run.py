import dearpygui.dearpygui as dpg

dpg.create_context()

with dpg.value_registry():
    dpg.add_bool_value(default_value=True, tag="bool_value")
    dpg.add_string_value(default_value="Default string", tag="string_value")
    dpg.add_float_value(default_value=0.0,tag = "baterry_V")

with dpg.window(label="Tutorial"):
    dpg.add_checkbox(label="Radio Button1", source="bool_value")
    dpg.add_checkbox(label="Radio Button2", source="bool_value")

    dpg.add_input_text(label="Text Input 1", source="string_value")
    dpg.add_input_text(label="Text Input 2", source="string_value", password=True)

    #baterry  
    dpg.add_input_float(label="baterry input voltage", source= "baterry_V")

    with dpg.plot(label="Baterry V ", height=-1, width=-1):
        dpg.add_plot_legend()
        dpg.add_plot_axis()
        

   
        



dpg.create_viewport(title='Custom Title', width=800, height=600)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()