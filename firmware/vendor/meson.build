# firmware/vendor/meson.build

# Process the subdirectories for gy and lc
gy_proj = subproject('gy')
lc_proj = subproject('lc')

# Get the dependency variables
lib_gy_dep = gy_proj.get_variable('lib_gy_dep')
lib_lc_dep = lc_proj.get_variable('lib_lc_dep')

# Override the dependencies to make them available project-wide
# meson.override_dependency('gy', lib_gy_dep)
# meson.override_dependency('lc', lib_lc_dep)

# Now, any other part of the build system can find 'gy' and 'lc'
# For example, your mesh component:
subproject('mesh') 