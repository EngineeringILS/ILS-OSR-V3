import dearpygui.dearpygui as dpg

dpg.create_context()

with dpg.window(label="Tutorial"):

    with dpg.table(header_row=True, policy=dpg.mvTable_SizingFixedFit, row_background=True, reorderable=True,
                   resizable=True, no_host_extendX=False, hideable=True,
                   borders_innerV=True, delay_search=True, borders_outerV=True, borders_innerH=True,
                   borders_outerH=True):

        dpg.add_table_column(label="AAA", width_fixed=True)
        dpg.add_table_column(label="BBB", width_fixed=True)
        dpg.add_table_column(label="CCC", width_stretch=True, init_width_or_weight=0.0)
        dpg.add_table_column(label="DDD", width_stretch=True, init_width_or_weight=0.0)

        for i in range(0, 5):
            with dpg.table_row():
                for j in range(0, 4):
                    if j == 2 or j == 3:
                        dpg.add_text(f"Stretch {i},{j}")
                    else:
                        dpg.add_text(f"Fixed {i}, {j}")

dpg.create_viewport(title='Custom Title', width=800, height=600)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()