project('RAB Demo',
	'c',
	default_options : [
		'optimization=' + get_option('optimization'),
		'buildtype=' + get_option('buildtype'),
		'c_std=gnu11',
		'warning_level=1'    # Adjust as needed
	],
	license: 'MIT',
	meson_version: '>=1.1.0',
	version: '1.0')

add_project_arguments('-DCHIP_TYPE=8', language : 'c')

add_project_arguments('-D__PROJECT_MESH__=1', language : 'c')

add_project_arguments('-DAIS_ENABLE=0', language: 'c')

add_project_arguments(
  '-fpack-struct=1',
  '-fshort-enums',
  language: 'c'
)

# Add global compiler arguments if needed for all stack compilations.
# Many toolchain/architecture-specific flags should be in your cross file.
# add_project_arguments('-DCOMMON_STACK_DEFINE', language : 'c')
# Global include directories that will be available to all parts of the project.
# Adding 'firmware' to the include path is the key to fixing the error,
# as it allows the compiler to find 'vendor/common/user_config.h'.
global_inc = include_directories(
    'firmware',
	'firmware/include',
    'firmware/proj',
    'firmware/proj/common',
    'firmware/proj/config',
    'firmware/proj/drivers',
    'firmware/proj/mcu',
    'firmware/proj/mcu_spec',
    'firmware/proj/simu',
    'firmware/proj/usbstd',
    'firmware/stack/ble',
    'firmware/vendor/common',
	'firmware/vendor/common/mi_api/mijia_ble_api',
	# 'firmware/vendor/common/certify_base',
	# 'firmware/vendor/common/mi_api',
	# 'firmware/vendor/common/nl_api',
	'firmware/drivers/8258',
	'firmware/drivers/8278'
)


message('Building selected BLE stack library from firmware/stack/')
message('Selected variant from option: ' + get_option('ble_stack_variant'))

# This line will execute firmware/stack/meson.build.
# That file is responsible for:
# 1. Reading the 'ble_stack_variant' option.
# 2. Conditionally building either the library in 'firmware/stack/ble/' or 'firmware/stack/ble_8278/'.
# 3. Making the chosen library and its headers available via a common dependency name
#    (e.g., by calling meson.override_dependency('chosen_ble_stack_library_dep', ...)).
subdir('firmware/stack')
subdir('firmware/vendor/common')
subdir('firmware/vendor/mesh')
subdir('firmware/vendor/user_app')
# subdir('firmware/vendor/8267_master_kma_dongle')

# For this focused task, we are not defining any executables in the root.
# The output will be the compiled static library from the selected stack variant.
# If you were to build an application later, it would get the stack dependency like this:
#   selected_stack_dep = dependency('chosen_ble_stack_library_dep', native: false) # Name must match what firmware/stack/meson.build overrides
#   app_exe = executable('my_app', 'my_app.c', dependencies : [selected_stack_dep, ...])

message('Root meson.build configuration complete. Check firmware/stack/ for library build.')