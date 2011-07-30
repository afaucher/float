env = Environment()

env.ParseConfig('pkg-config --libs --cflags gl')
env.ParseConfig('pkg-config --libs --cflags glu')
env.ParseConfig('pkg-config --libs --cflags sdl')
env.ParseConfig('echo -lglut')
env.ParseConfig('pkg-config --libs --cflags ode')
env.ParseConfig('pkg-config --libs --cflags sigc++-2.0')
env.ParseConfig('pkg-config --libs --cflags glibmm-2.4')

include_dirs = Split('''

  ''')

env.Append(CPPPATH = include_dirs)

env.Append(CCFLAGS = '-g')

code_files = Split('''
    main.cpp
    joystick.cpp
    ''')

env.Program("test",code_files)
