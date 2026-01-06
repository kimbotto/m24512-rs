# M24512 EEPROM Driver

A `no_std` Rust driver for the M24512 EEPROM using `embedded-hal-async`.

## Features
- 16-bit memory addressing (64 KB capacity)
- 128-byte page write support
- Automatic page boundary handling
- Hardware write protection control (via nWC pin)
- `embedded-storage-async` trait implementation
- Async support via `embassy-time`

## License
Licensed under either of:
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.
