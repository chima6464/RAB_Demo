module_sources_lc = files(
    'lc.h',
    'lc.c'
)

module_lc_includes = include_directories('.')

lib_lc = static_library('LC',
    module_sources_lc,
    include_directories : [module_lc_includes, global_inc],
    dependencies : [declare_dependency(include_directories: '../gy')])

lib_lc_dep = declare_dependency(
    link_with : lib_lc,
    include_directories : module_lc_includes
)

# set_variable('lib_lc_dep', lib_lc_dep)

# This is a key step for making the dependency easily discoverable
# by the parent meson.build (or any other meson.build file) under a known name.
meson.override_dependency('LcLib', lib_lc_dep)
message('LcLib configured and its dependency "LcLib" is available.')