import dearpygui.dearpygui as dpg

dpg.create_context()

# 1. Load your texture
# Replace 'path_to_your_image.png' with your actual file
width, height, channels, data = dpg.load_image("/home/enrique/lunabotics-cdh-dev/python/frontend/images/64px-NASA_logo.svg.png")

with dpg.texture_registry(show=False):
    dpg.add_static_texture(width=width, height=height, default_value=data, tag="image_tag")

def update_image_pos():
    # Get current window dimensions
    win_width = dpg.get_item_width("main_window")
    win_height = dpg.get_item_height("main_window")
    
    # Calculate bottom-left (with a small 10px offset)
    padding = 10
    pos_x = win_width - width- padding
    pos_y = win_height - height - padding - 20 # -20 accounts for title bar/borders
    
    dpg.set_item_pos("ui_image", [pos_x, pos_y])

with dpg.window(label="Image Window", tag="main_window", width=400, height=400, on_close=dpg.stop_dearpygui):
    # 2. Add the image widget
    dpg.add_image("image_tag", tag="ui_image")

# 3. Create a handler to update position when window resizes
with dpg.item_handler_registry(tag="window_handler"):
    dpg.add_item_resize_handler(callback=update_image_pos)

dpg.bind_item_handler_registry("main_window", "window_handler")

dpg.create_viewport(title='Bottom Left Image Example', width=600, height=600)
dpg.setup_dearpygui()
dpg.show_viewport()

# Initial call to set position
dpg.set_frame_callback(1, update_image_pos)

dpg.start_dearpygui()
dpg.destroy_context()