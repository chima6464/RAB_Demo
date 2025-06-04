project('RAB Demo',
	'c',
	default_options : [
		'optimization=' + get_option('optimization'),
		'buildtype=' + get_option('buildtype'),
		'c_std=c11',
		'warning_level=1'    # Adjust as needed
	],
	license: 'MIT',
	meson_version: '>=1.1.0',
	version: '1.0')

# Add global compiler arguments if needed for all stack compilations.
# Many toolchain/architecture-specific flags should be in your cross file.
# add_project_arguments('-DCOMMON_STACK_DEFINE', language : 'c')

message('Building selected BLE stack library from firmware/stack/')
message('Selected variant from option: ' + get_option('ble_stack_variant'))

# This line will execute firmware/stack/meson.build.
# That file is responsible for:
# 1. Reading the 'ble_stack_variant' option.
# 2. Conditionally building either the library in 'firmware/stack/ble/' or 'firmware/stack/ble_8278/'.
# 3. Making the chosen library and its headers available via a common dependency name
#    (e.g., by calling meson.override_dependency('chosen_ble_stack_library_dep', ...)).
subdir('firmware/stack')

# For this focused task, we are not defining any executables in the root.
# The output will be the compiled static library from the selected stack variant.
# If you were to build an application later, it would get the stack dependency like this:
#   selected_stack_dep = dependency('chosen_ble_stack_library_dep', native: false) # Name must match what firmware/stack/meson.build overrides
#   app_exe = executable('my_app', 'my_app.c', dependencies : [selected_stack_dep, ...])

message('Root meson.build configuration complete. Check firmware/stack/ for library build.')