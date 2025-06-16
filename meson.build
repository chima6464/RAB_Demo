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

add_project_arguments(
  '-DCHIP_TYPE=8',
  '-D__PROJECT_MESH__=1',
  '-DAIS_ENABLE=0',
  '-fpack-struct=1',
  '-fshort-enums',
  '-fshort-wchar',
  '-fms-extensions',
  '-fms-extensions',
  '-finline-small-functions',
  language: 'c'
)

# 1. DEFINE THE PRODUCT DATABASE
# This dictionary maps product families and PIDs to their required features and macro values.
# Note: Unfortunately, Meson does not support float 
product_db = {
  'RABBlueController' : {
    '108F' : {
      'features' : ['feature_a', 'feature_b', 'feature_c'],
      'macros'   : { 'GY_DEV_108F' : 4239, 'GY_DEV_TYPE' : 'GY_DEV_108F', 'GY_PID' : 4111,
	  'GY_CID' : 1957, 'GY_DIM_PIN' : 'GPIO_PD3', 'GY_LED_R_PIN' : 'GPIO_PC0', 'GY_LED_G_PIN' : 'GPIO_PD4', 
	  'GY_PWM_FUNC_R' : 'AS_PWM', 'GY_BUTTON_PIN' : 'GPIO_PB6', 'GY_RELAY_PIN' : 'GPIO_PA1', 
	  'GY_I2C_SDA_PIN' : 'GPIO_PC2', 'GY_I2C_SCK_PIN' : 'GPIO_PC3', 'GY_DIMMER_CURVE_TRANSFORMATION_1V_VALUE' : 12,
	  'GY_DIMMER_CURVE_TRANSFORMATION_MAX_VALUE' : 100, 'GY_ZCD_PIN ' : 'GPIO_PD2'},
    },
  },
  'RabHighbayNewId' : {
	'1091' : {
	  'features' : ['feature_a', 'feature_b', 'feature_c'],
      'macros'   : { 'GY_DEV_1091' : 4241, 'GY_DEV_TYPE' : 'GY_DEV_1091'},
	  },
  },
}

# 2. GET USER SELECTION
selected_family = get_option('product_family')
selected_pid = get_option('product_pid')

# 3. LOOKUP CONFIGURATION & PREPARE HEADER DATA
if not product_db.has_key(selected_family) or not product_db[selected_family].has_key(selected_pid)
  error('Selected product family "' + selected_family + '" with PID "' + selected_pid + '" is not defined in the product_db.')
endif

config = product_db[selected_family][selected_pid]
conf_data = configuration_data() # Meson object to hold our #define values

# Set the simple replacement macros from our database
foreach name, value : config.get('macros', {})
  conf_data.set(name, value)
endforeach

# 4. GENERATE THE HEADER FILE
# This creates product_config.h in the build directory
generated_product_config = configure_file(
  input : 'product_config.h.in',
  output : 'product_config.h',
  configuration : conf_data
)

# Create a dependency object that just provides the include path to this generated header
generated_config_dep = declare_dependency(
  sources : [generated_product_config]
  # The include directory is implicitly the build directory where the file is generated
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
subdir('firmware/vendor/gy')
subdir('firmware/vendor/lc')
# subdir('firmware/vendor/8267_master_kma_dongle')

# For this focused task, we are not defining any executables in the root.
# The output will be the compiled static library from the selected stack variant.
# If you were to build an application later, it would get the stack dependency like this:
#   selected_stack_dep = dependency('chosen_ble_stack_library_dep', native: false) # Name must match what firmware/stack/meson.build overrides
#   app_exe = executable('my_app', 'my_app.c', dependencies : [selected_stack_dep, ...])

message('Root meson.build configuration complete. Check firmware/stack/ for library build.')
