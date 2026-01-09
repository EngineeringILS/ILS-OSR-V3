import pygame
class controller():
    def __init__(self):
        self.axis_list = ["Hori_L:","Vert_L","Hori_R","Vert_R","LT",'RT']
        self.but_list = ["A","B","X","Y","LB","RB","Select","Start","LStick","RStick"]

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            
            return
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        
        if pygame.joystick.get_count() == 0:
            print("No controller found!  .")
            


    def get_controller_data(self):

        if pygame.joystick.get_count() == 0:
            values=["No controller"]*16
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    self.joy = pygame.joystick.Joystick(0)
                    self.joy.init()

            return values
        

        pygame.event.pump()
        

        values=[]
        for i in range (len(self.axis_list)):
            values.append(str(self.joy.get_axis(i)))

        for i in range (len(self.but_list)):
            values.append(str(self.joy.get_button(i)))

        return values
    
    def input_name(self):
        


        input_names = self.axis_list + self.but_list
        return input_names