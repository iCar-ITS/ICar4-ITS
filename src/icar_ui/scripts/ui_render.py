#!/usr/bin/python3

import rospy
import rospkg
import pygame
from icar_ui.msg import ui_input_pointer
from icar_ui.msg import ui_input_keyboard
from icar_ui.msg import ui_output_image
from icar_ui.msg import ui_output_sound

INPUT_WIDTH = 1920
INPUT_HEIGHT = 1080
OUTPUT_WIDTH = 960
OUTPUT_HEIGHT = 540
RATIO_WIDTH = OUTPUT_WIDTH / INPUT_WIDTH
RATIO_HEIGHT = OUTPUT_HEIGHT / INPUT_HEIGHT
RESOURCES_PATH = rospkg.RosPack().get_path('icar_ui') + '/resources/'

class ui_mixer:
    def __init__(self, id, action, filename):
        # Loading the sound file.
        self.mixer = pygame.mixer.Sound(filename)

        # Calling the routine function.
        self.routine(id, action, filename)

    def routine(self, id, action, filename):
        # A variable that is used to identify the mixer.
        self.id = id

        # Stopping the sound, then loading the sound, then stopping the sound.
        pygame.mixer.Sound.stop(self.mixer)
        self.mixer = pygame.mixer.Sound(filename)
        pygame.mixer.Sound.stop(self.mixer)

        # Checking if the action is play once or play loop.
        if action == ui_output_sound.ACTION_PLAY_ONCE:
            pygame.mixer.Sound.play(self.mixer)
        elif action == ui_output_sound.ACTION_PLAY_LOOP:
            pygame.mixer.Sound.play(self.mixer, -1)

class ui_render:
    def __init__(self):
        # =====Timer
        self.tim_50hz = rospy.Timer(rospy.Duration(0.02), self.cllbck_tim_50hz)
        self.tim_100hz = rospy.Timer(rospy.Duration(0.01), self.cllbck_tim_100hz)
        # =====Subscriber
        self.sub_ui_output_image = rospy.Subscriber('/ui_output/image', ui_output_image, self.cllbck_sub_ui_output_image, queue_size=None)
        self.sub_ui_output_sound = rospy.Subscriber('/ui_output/sound', ui_output_sound, self.cllbck_sub_ui_output_sound, queue_size=None)
        # =====Publisher
        self.pub_ui_input_pointer = rospy.Publisher('/ui_input/pointer', ui_input_pointer, queue_size=1)
        self.pub_ui_input_keyboard = rospy.Publisher('/ui_input/keyboard', ui_input_keyboard, queue_size=1)

        self.screen = None
        self.surface = None
        self.mixers = []

        if self.ui_render_init() == -1:
            rospy.signal_shutdown('')

    # --------------------------------------------------------------------------
    # ==========================================================================

    def cllbck_tim_50hz(self, event):
        if self.ui_render_routine() == -1:
            rospy.signal_shutdown('')

    def cllbck_tim_100hz(self, event):
        pass

    # --------------------------------------------------------------------------
    # ==========================================================================

    def cllbck_sub_ui_output_image(self, msg):
        # Scaling the image to the correct size.
        output_x = int(msg.x * RATIO_WIDTH)
        output_y = int(msg.y * RATIO_HEIGHT)
        output_w = int(msg.w * RATIO_WIDTH)
        output_h = int(msg.h * RATIO_HEIGHT)

        self.surface.blit(pygame.image.frombuffer(msg.data, (msg.w, msg.h), 'RGB'), (msg.x, msg.y))

        # If the message is a refresh, update the screen.
        if msg.refresh:
            pygame.transform.scale(self.surface, (OUTPUT_WIDTH, OUTPUT_HEIGHT), self.screen)
            pygame.display.update()

    def cllbck_sub_ui_output_sound(self, msg):
        # Checking if the mixer is already loaded.
        mixer_loaded = False
        mixer_index = 0
        for i, mixer in enumerate(self.mixers):
            if mixer.id == msg.id:
                mixer_loaded = True
                mixer_index = i
                break

        if not mixer_loaded:
            # If the mixer is not loaded, load the mixer.
            self.mixers.append(ui_mixer(msg.id, msg.action, RESOURCES_PATH + msg.filename))
        else:
            # If the mixer is loaded, call the routine function.
            self.mixers[mixer_index].routine(msg.id, msg.action, RESOURCES_PATH + msg.filename)

    # --------------------------------------------------------------------------
    # ==========================================================================

    def ui_render_init(self):
        # Initializing the pygame library.
        pygame.init()
        pygame.mixer.init()

        # Setting the title of the window and the icon of the window.
        pygame.display.set_caption('Institut Teknologi Sepuluh Nopember')
        pygame.display.set_icon(pygame.image.load(RESOURCES_PATH + 'favicon.png'))

        # Creating a window.
        self.screen = pygame.display.set_mode((OUTPUT_WIDTH, OUTPUT_HEIGHT))

        # Creating a surface.
        self.surface = pygame.Surface((INPUT_WIDTH, INPUT_HEIGHT))

        return 0

    def ui_render_routine(self):
        if not pygame.display.get_init():
            return 0

        for event in pygame.event.get():
            # Checking if the user has pressed the close button on the window.
            # If so, it will shutdown the node.
            if event.type == pygame.QUIT:
                rospy.signal_shutdown('')

            # Checking if the mouse is pressed down, up, or moving.
            # If so, it will publish the mouse position and the mouse button state.
            elif (event.type == pygame.MOUSEBUTTONDOWN or
                  event.type == pygame.MOUSEBUTTONUP or
                  event.type == pygame.MOUSEMOTION):
                msg_ui_input_pointer = ui_input_pointer()
                msg_ui_input_pointer.x = int(event.pos[0] / RATIO_WIDTH)
                msg_ui_input_pointer.y = int(event.pos[1] / RATIO_HEIGHT)
                msg_ui_input_pointer.pressed = bool(pygame.mouse.get_pressed()[0])
                self.pub_ui_input_pointer.publish(msg_ui_input_pointer)

            # Checking if the user has pressed a key, or released a key.
            # If so, it will publish the key code and the key state.
            elif (event.type == pygame.KEYDOWN or
                    event.type == pygame.KEYUP):
                # Checking if the key is a valid key.
                if event.key < 0 or event.key > 255:
                    continue
                # Checking if the user has pressed the escape key.
                if event.key == pygame.K_ESCAPE:
                    rospy.signal_shutdown('')

                msg_ui_input_keyboard = ui_input_keyboard()
                msg_ui_input_keyboard.key = int(event.key)
                msg_ui_input_keyboard.pressed = bool(event.type == pygame.KEYDOWN)
                self.pub_ui_input_keyboard.publish(msg_ui_input_keyboard)

        return 0


if __name__ == '__main__':
    rospy.init_node('ui_render')
    ui_render()
    rospy.spin()
