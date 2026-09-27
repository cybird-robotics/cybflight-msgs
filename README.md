# cybflight-msgs

Shared `no_std` messages for the Cybflight flight controller, ESP32 bridge, and ground station.

```toml
[dependencies]
cybflight-msgs = "=0.2.0"
```

The default configuration provides dependency-free wire messages and the UDP batch envelope. Firmware can enable `embedded` for Embassy timestamps, nalgebra types, and defmt support. Desktop tools should use the default configuration.

The wire format uses little-endian packed fields and fixed message IDs. UART transports add COBS framing; UDP transports carry frames or the batch envelope described in `udp_batch`. Message IDs are never reassigned. Breaking changes to layouts require a coordinated release of all endpoints; a Cargo version does not negotiate compatibility over the link.

`WireMessage` is an unsafe trait: implementations must have no padding and must accept every possible bit pattern. Validate ranges and freshness after decoding a structurally valid frame.

```sh
cargo test
cargo check --no-default-features --target thumbv7em-none-eabihf
cargo check --features embedded --target thumbv7em-none-eabihf
```

Copyright 2026 The Cybird project contributors. See [NOTICE](NOTICE).

Licensed under Apache-2.0.
