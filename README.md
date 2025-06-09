# RAB Demo

This project is designed to demonstate the efficacy of a Build System for RAB products with Meson

## Installation

TODO

## Compilation

meson setup builddir --cross-file cross_configs/telink_target_cross_file.ini -Dble_stack_variant=<choice>

Eventually.......
meson setup builddir \
  --reconfigure \
  --cross-file cross_configs/telink_target_cross_file.ini \
  -Dble_stack_variant=standard \
  -Dtarget_product_id=101F # Or GY_DEV_108F, etc.

## Usage

TODO: Wherever MD_PRIVACY_BEACON really should be 8258_Mesh product

## Contributing

## License
