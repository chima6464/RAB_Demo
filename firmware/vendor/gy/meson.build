module_sources_gy = files(
    'gy.h',
    'gy.c'
)

module_gy_includes = include_directories('.')

lib_gy = static_library('GY',
    module_sources_gy,
    include_directories : [module_gy_includes, global_inc],
    dependencies : [declare_dependency(include_directories: '../lc')])

lib_gy_dep = declare_dependency(
    link_with : lib_gy,
    include_directories : module_gy_includes
)

# set_variable('lib_gy_dep', lib_gy_dep)

# This is a key step for making the dependency easily discoverable
# by the parent meson.build (or any other meson.build file) under a known name.
meson.override_dependency('GyLib', lib_gy_dep)
message('GyLib configured and its dependency "GyLib" is available.')