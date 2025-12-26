import pygame

pygame.init()
pygame.joystick.init()

# Connect to the first controller
joystick = pygame.joystick.Joystick(0)
joystick.init()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 1. Get Axis Movement (Sticks and Triggers)
    left_stick_x = joystick.get_axis(0)
    left_stick_y = joystick.get_axis(1)
    
    # 2. Get Button Presses
    a_button = joystick.get_button(0)
    b_button = joystick.get_button(1)

    # 3. Get D-Pad (Hat)
    # Returns a tuple (x, y) like (0, 1) for Up, (1, 0) for Right
    d_pad = joystick.get_hat(0)

    # Print debug info (use formatting to keep it clean)
    print(f"Stick: {left_stick_x:.2f}, {left_stick_y:.2f} | A: {a_button} | D-Pad: {d_pad}", end="\r")

pygame.quit()