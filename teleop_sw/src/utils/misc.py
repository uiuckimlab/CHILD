import termcolor
def print_color(*args, color=None, attrs=(), **kwargs):
    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)

try:
    import getch
    special_keys = {'[A': 'up', '[B': 'down', '[C': 'right', '[D': 'left',
                    '[1': 'home', '[2': 'insert', '[3': 'delete', '[4': 'end',
                    '[5': 'page up', '[6': 'page down', '[H': 'home', '[F': 'end'}

    def get_key():
        first_char = getch.getch()
        if first_char == '\x1b':
            more_chars = getch.getch() + getch.getch()
            if more_chars not in special_keys: return first_char + more_chars
            return special_keys[more_chars]
        else:
            return first_char
except: pass
