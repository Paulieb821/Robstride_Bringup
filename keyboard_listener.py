import pynput.keyboard as keyboard

class Keyboard_Listener:

    # Initialize Key States
    def __init__(self):
        self.key_states = {
            'a': False, 'd': False, 'w': False, 's': False,
            '1': False, '2': False, 'up': False, 'down': False,
            'space': False
        }

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
    
    # Press function
    def on_press(self, key):
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = True
        except AttributeError:
            if key == keyboard.Key.up:
                self.key_states['up'] = True
            elif key == keyboard.Key.down:
                self.key_states['down'] = True
            elif key == keyboard.Key.space:
                self.key_states['space'] = True

    # Release function
    def on_release(self, key):
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = False
        except AttributeError:
            if key == keyboard.Key.up:
                self.key_states['up'] = False
            elif key == keyboard.Key.down:
                self.key_states['down'] = False
            elif key == keyboard.Key.space:
                self.key_states['space'] = False
