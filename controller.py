# Xbox Controller Input Handling
import pygame

# pygame joystick link : https://www.pygame.org/docs/ref/joystick.html
# XBox 360 Controller

class XboxController:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise Exception("No controller found. Connect an Xbox controller.")
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

    def read(self):
        # Reads controller inputs and returns a value between [-1.0, 1.0]
        pygame.event.pump()

        # joystick config
        left_x = self.joy.get_axis(0)           # left joystick X (right --> +1.00   ;   left --> -1.00)
        left_y = -self.joy.get_axis(1)          # left joystick Y (up --> +1.00   ;   down --> -1.00)
        right_x = self.joy.get_axis(2)          # right stick X (right --> +1.00   ;   left --> -1.00)
        right_y = -self.joy.get_axis(3)         # right stick Y (up --> +1.00   ;   down --> -1.00)
        left_in = self.joy.get_button(8)        # left joystick pressed
        right_in = self.joy.get_button(9)       # right joystick pressed
        
        # trigger config
        rawtrigger_L = self.joy.get_axis(4)    # left trigger (unpressed --> +1.00   ;   pressed --> -1.00)
        rawtrigger_R = self.joy.get_axis(5)    # right trigger (unpressed --> +1.00   ;   pressed --> -1.00)
        trigger_L = (rawtrigger_L + 1) / 2      # normalize to [0, 1]
        trigger_R = (rawtrigger_R + 1) / 2      # normalize to [0, 1]

        # button config
        a_button = self.joy.get_button(0)
        b_button = self.joy.get_button(1)
        x_button = self.joy.get_button(2)
        y_button = self.joy.get_button(3)
        lb = self.joy.get_button(4)
        rb = self.joy.get_button(5)
        back_button = self.joy.get_button(6)        # back button (looks like a copy icon)
        start_button = self.joy.get_button(7)       # start button (3 horizontal line icon)

        # D-pad config
        dpad_up = self.joy.get_hat(0) == (0,1)          # up on the D-pad
        dpad_down = self.joy.get_hat(0) == (0,-1)       # down on the D-pad
        dpad_right = self.joy.get_hat(0) == (1,0)       # right on the D-pad
        dpad_left = self.joy.get_hat(0) == (-1,0)       # left on the D-pad


        return {
            'leftVertical': left_y,
            'leftHorizontal': left_x,
            'rightVertical': right_y,
            'rightHorizontal': right_x,
            'leftIn': left_in,
            'rightIn': right_in,
            
            'triggerL': trigger_L,
            'triggerR': trigger_R,
            
            'a': a_button,
            'b': b_button,
            'x': x_button,
            'y': y_button,
            'lb': lb,
            'rb': rb,

            'return': back_button,
            'start': start_button,

            'D_up': dpad_up,
            'D_down': dpad_down,
            'D_right': dpad_right,
            'D_left': dpad_left
        }

while True:
    data = XboxController().read()

    # button states
    if data['a']:
        print("A button pressed")
    if data['b']:
        print("B button pressed")
    if data['x']:
        print("X button pressed")
    if data['y']:
        print("Y button pressed")
    if data['lb']:
        print("Left bumper pressed")
    if data['rb']:
        print("Right bumper pressed")
    if data['return']:
        print("return button pressed")
    if data['start']:
        print("start button pressed")

    # joystick states
    if data['leftVertical'] < -0.1 or data['leftVertical'] > 0.1:
        print(f"Left Stick Y: {data['leftVertical']:.2f}")
    if data['leftHorizontal'] < -0.1 or data['leftHorizontal'] > 0.1:
        print(f"Left Stick X (Turn): {data['leftHorizontal']:.2f}")
    if data['rightVertical'] < -0.1 or data['rightVertical'] > 0.1:
        print(f"Right Stick Y: {data['rightVertical']:.2f}")
    if data['rightHorizontal'] < -0.1 or data['rightHorizontal'] > 0.1:
        print(f"Right Stick X: {data['rightHorizontal']:.2f}")

    # trigger states
    if data['triggerL'] > 0.1:
        print(f"Left Trigger: {data['triggerL']:.2f}")
    if data['triggerR'] > 0.1:
        print(f"Right Trigger: {data['triggerR']:.2f}")

    # joystick in/out states
    if data['leftIn']:
        print("Left joystick pressed")
    if data['rightIn']:
        print("Right joystick pressed")

    # D-pad states
    if data['D_up']:
        print("D-pad up pressed")
    if data['D_down']:
        print("D-pad down pressed")
    if data['D_right']:
        print("D-pad right pressed")
    if data['D_left']:
        print("D-pad left pressed")
   
    pygame.time.wait(100)
    