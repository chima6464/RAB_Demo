module_sources_user_app = files(
    'user_app_config.h',
    'user_app_default.h',
    'user_app.c',
    'user_app.h'
)

module_user_app_includes = include_directories('.')

lib_user_app = static_library('UserApp',
    module_sources_user_app,
    include_directories : [module_user_app_includes, global_inc])

lib_user_app_dep = declare_dependency(
    link_with : lib_user_app,
    include_directories : module_user_app_includes
)

# This is a key step for making the dependency easily discoverable
# by the parent meson.build (or any other meson.build file) under a known name.
meson.override_dependency('UserAppLib', lib_user_app_dep)
message('UserAppLib configured and its dependency "UserAppLib" is available.')