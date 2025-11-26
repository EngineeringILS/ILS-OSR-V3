idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS ".")

# --- ADD THIS ---
# Link against the *real* target name "__idf_main"
target_link_libraries(__idf_main
    PRIVATE
    common_lib
)