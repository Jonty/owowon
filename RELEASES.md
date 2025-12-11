# Release Installation Guide

## Download

Download the appropriate binary for your platform from the [Releases page](https://github.com/seritools/owowon/releases):

- **Linux**: `owowon-gui-linux-x86_64`
- **Windows**: `owowon-gui-windows-x86_64.exe`
- **macOS (Intel)**: `owowon-gui-macos-x86_64`
- **macOS (Apple Silicon)**: `owowon-gui-macos-aarch64`

## Platform-Specific Setup

### Linux

1. Make the binary executable:
   ```bash
   chmod +x owowon-gui-linux-x86_64
   ```

2. Install udev rule for USB access (one-time setup):
   ```bash
   # Download the udev rule from the repository
   sudo curl -o /etc/udev/rules.d/50-owon-hds.rules https://raw.githubusercontent.com/seritools/owowon/main/50-owon-hds.rules

   # Reload udev rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

3. Unplug and replug your oscilloscope, then run:
   ```bash
   ./owowon-gui-linux-x86_64
   ```

### Windows

1. Install the WinUSB driver using [Zadig](https://zadig.akeo.ie/):
   - Download and run Zadig
   - Connect your Owon oscilloscope
   - Select your device from the dropdown
   - Select "WinUSB" as the driver
   - Click "Install Driver" or "Replace Driver"

2. Run the application:
   ```
   owowon-gui-windows-x86_64.exe
   ```

### macOS

1. Make the binary executable:
   ```bash
   chmod +x owowon-gui-macos-x86_64  # or owowon-gui-macos-aarch64
   ```

2. Remove quarantine attribute (macOS security):
   ```bash
   xattr -d com.apple.quarantine owowon-gui-macos-x86_64  # or owowon-gui-macos-aarch64
   ```

3. Run the application:
   ```bash
   ./owowon-gui-macos-x86_64  # or ./owowon-gui-macos-aarch64
   ```

   You may see a security dialog on first run - click "Allow".

## Troubleshooting

### Linux: "Permission denied" when opening device
- Verify udev rule is installed: `cat /etc/udev/rules.d/50-owon-hds.rules`
- Check device appears: `lsusb | grep 5345:1234`
- Restart udev: `sudo udevadm control --reload-rules && sudo udevadm trigger`
- Unplug and replug the device

### Windows: Device not found
- Ensure WinUSB driver is installed using Zadig
- Check Device Manager for the device under "Universal Serial Bus devices"

### macOS: "App is damaged and can't be opened"
- Run: `xattr -d com.apple.quarantine owowon-gui-macos-*`
- This removes the quarantine flag added by macOS to downloaded files

## Building from Source

See the main [README.md](README.md) for instructions on building from source.
