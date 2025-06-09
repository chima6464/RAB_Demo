module_sources_8267_master_kma_dongle = files(
    'app_att.c',
    'app_config.h',
    'app_ota.c',
    'app_pair.c',
    'app_pair.h',
    'app.c',
    'audio_buffer.c',
    'button.c',
    'e2prom.c',
    'e2prom.h',
    'main.c',
    'mouse_type.h',
    'rf_frame.h',
    'tl_audio.c',
    'tl_audio.h',
    'trace.h'
)

# Declares that the current directory ('.') should be an include path
# for compiling sources of this 8267_master_kma_dongle AND for anything that links to it.
module_8267_master_kma_dongle_includes = include_directories('.')

lib_8267_master_kma_dongle = static_library('8267MasterKmaDongle',
    module_sources_8267_master_kma_dongle,
    include_directories : [module_8267_master_kma_dongle_includes, global_inc])

# Declare the dependency information for this library.
# This is what other targets will use to link against lib_a_static
# and get its include directories.
lib_8267_master_kma_dongle_dep = declare_dependency(
    link_with : lib_8267_master_kma_dongle,
    include_directories : module_8267_master_kma_dongle_includes
)

# This is a key step for making the dependency easily discoverable
# by the parent meson.build (or any other meson.build file) under a known name.
meson.override_dependency('8267MasterKmaDongleLib', lib_8267_master_kma_dongle_dep)
message('8267MasterKmaDongleLib configured and its dependency "8267MasterKmaDongleLib" is available.')