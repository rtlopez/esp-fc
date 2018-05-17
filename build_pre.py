Import("env")

build_board = env['BOARD']
build_flags = env.ParseFlags(env['BUILD_FLAGS'])
build_defs = {k: v for (k, v) in build_flags.get("CPPDEFINES")}
build_ver = build_defs.get("ESPFC_VERSION")

env.Replace(PROGNAME="ESPFC-%s-%s.elf" % (build_board.upper(), build_ver))

#print env.Dump()
