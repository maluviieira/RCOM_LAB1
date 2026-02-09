# seriallink- Serial Port Protocol

This repository contains an implementation of a simple serial-port-based file transfer protocol used for teaching link-layer concepts (framing, error detection, retransmission). The project includes:

- A transmitter/receiver program (`bin/main`) that implements the link-layer and application-layer code.
- A virtual cable program (`cable/`) that simulates a serial connection and can inject disconnections and noise for testing.
- Source files in `src/` for the link and application layers.

This README explains how to build, run and test the project.

## Prerequisites

- Linux (project tested on Ubuntu).
- `gcc` and `make` to build the project.
- `socat` (required by the virtual cable program):

```bash
sudo apt-get update && sudo apt-get install -y socat
```

## Project structure

- `bin/` — compiled executables (created by `make`).
- `cable/` — virtual cable program source and built binary (do not modify `cable/cable`).
- `src/` — implementation sources: `link_layer.c`, `application_layer.c`, `serial_port.c`, and headers.
- `Makefile` — build targets and helpers (build, run, test).
- `penguin.gif` — example file used for transfer tests.

## Build

To compile the application and the virtual cable program, run:

```bash
make
```

This will produce the main executable in `bin/main` and the cable program in `bin/cable_app` (or the `cable/` folder binary depending on the Makefile).

## Running the virtual cable (test harness)

The virtual cable simulates a serial link between two pseudo-terminals and can introduce noise or disconnects. It requires root privileges for `socat`.

Start the cable in a separate terminal:

```bash
sudo ./bin/cable_app
# or
sudo make run_cable
```

When the cable program is running, use its console to control the link:

- Press `0` to simulate an unplugged cable (disconnect).
- Press `2` to add noise to the link.
- Press `1` to restore normal operation.

## Running the receiver and transmitter

Open two terminals in addition to the cable terminal.

1. Receiver (Rx):

```bash
./bin/main /dev/ttyS11 9600 rx penguin-received.gif
# or
make run_rx
```

2. Transmitter (Tx):

```bash
./bin/main /dev/ttyS10 9600 tx penguin.gif
# or
make run_tx
```

Adjust `/dev/ttyS10` and `/dev/ttyS11` as printed or configured by the `cable_app` program — the cable program shows which pseudo-terminals to use.

## Verifying the transfer

After the transfer completes, compare the sent and received files:

```bash
diff -s penguin.gif penguin-received.gif
# or
make check_files
```

The `diff` should report the files as identical when the protocol works correctly.

## Testing with disconnections and noise

1. Start the cable program.
2. Start the receiver and transmitter.
3. While transfer is running, interact with the cable console to inject failures (unplug or noise) and observe retransmissions, timeouts, and recovery.

The link-layer implementation should handle these transient failures and still produce a correct received file.

## Source overview

- `src/link_layer.c` / `src/link_layer.h`: link-layer framing, byte stuffing, control frames, retransmissions, timeouts.
- `src/application_layer.c` / `src/application_layer.h`: simple application protocol wrapping file reads/writes into data packets.
- `src/serial_port.c` / `src/serial_port.h`: platform serial port / pseudo-terminal helpers used by the link layer.
- `main.c`: program entry that sets up mode (`tx` or `rx`) and arguments.

## Makefile targets (common)

- `make` — build all binaries.
- `make run_cable` — run the virtual cable (needs `sudo`).
- `make run_tx` — run transmitter with default test file.
- `make run_rx` — run receiver with default output file.
- `make check_files` — compare penguin files.
- `make clean` — remove build artifacts.

Run `make` without arguments to inspect available targets.

## Troubleshooting

- If `socat` errors appear, ensure it is installed and you started the cable with `sudo`.
- If the pseudo-terminals differ from `/dev/ttyS10` or `/dev/ttyS11`, use the printed device names from the cable program.
- Use `strace` or `gdb` for low-level debugging of the binaries if they crash.

## Contact / Notes

This repository is for lab exercises. Do not modify the `cable/` program unless instructed. If you need help, ask your instructor or open an issue in your course repo.

