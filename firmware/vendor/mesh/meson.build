# project('Mesh', 'c')

module_sources_mesh = files(
    'app_att.c',
    'app_config_8258.h',
    'app_config_8269.h',
    'app_config_8278.h',
    'app.c',
    'app.h'
    # 'main.c'
)

gy_inc = include_directories('../gy')
lc_inc = include_directories('../lc')

module_mesh_includes = include_directories('.')

lib_mesh = static_library('Mesh', 
    sources: module_sources_mesh,
    include_directories : [module_mesh_includes, global_inc, gy_inc, lc_inc]
)

lib_mesh_dep = declare_dependency(
    link_with : lib_mesh,
    include_directories : module_mesh_includes
)

# This is a key step for making the dependency easily discoverable
# by the parent meson.build (or any other meson.build file) under a known name.
meson.override_dependency('Mesh', lib_mesh_dep)
message('MeshLib configured and its dependency "MeshLib" is available.')