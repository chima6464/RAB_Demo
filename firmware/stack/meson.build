# This project declaration is minimal as we are only selecting include paths.

# Get the value of the option defined in the root meson_options.txt
selected_ble_header_option = get_option('ble_stack_variant')

message('Selected BLE header set: ' + selected_ble_header_option)

# This variable will hold the include_directories object for the chosen set.
conditional_ble_includes = disabler() # Initialize with a "disabled" object, acts as a NOP.

if selected_ble_header_option == 'standard'
  # This points to the 'tc_ble_mesh-master/firmware/stack/ble/' directory
  conditional_ble_includes = include_directories('ble')
  message('Including headers from: firmware/stack/ble/')
elif selected_ble_header_option == 'ble8278'
  # This points to the 'tc_ble_mesh-master/firmware/stack/ble_8278/' directory
  conditional_ble_includes = include_directories('ble_8278')
  message('Including headers from: firmware/stack/ble_8278/')
else
  error('Invalid "ble_stack_variant" option selected: ' + selected_ble_header_option + '. Valid choices are "standard" or "ble8278".')
endif

# Create a dependency object that *only* provides these include directories.
# Since the subfolders 'ble' and 'ble8278' only contain headers and are not compiled
# into libraries themselves for this selection purpose, we don't use 'link_with'.
ble_header_variant_dep = declare_dependency(
  include_directories : conditional_ble_includes
  # No 'sources' or 'link_with' arguments needed here.
)

# Make this conditional dependency available to other parts of your build system
# (like the root meson.build or any library/executable that needs these headers)
# under a consistent, well-known name.
meson.override_dependency('ble_selected_stack_headers', ble_header_variant_dep)