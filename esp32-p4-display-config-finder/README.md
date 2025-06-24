# ESP32-P4 Display Configuration Finder

An interactive command-line tool for diagnosing and optimizing MIPI DSI display configurations on ESP32-P4 hardware. This project helps developers find the optimal timing parameters, troubleshoot display issues, and validate framebuffer operations for panels like the ILI9881C and EK79007.

## Purpose

The ESP32-P4 Display Configuration Finder addresses common challenges when working with MIPI DSI displays:

- **Parameter Tuning**: Finding the correct horizontal/vertical sync timing parameters
- **Signal Integrity**: Diagnosing flickering, horizontal lines, or display artifacts
- **Performance Analysis**: Measuring DMA transfer rates and bandwidth utilization
- **Configuration Validation**: Testing different clock frequencies and lane configurations
- **Automated Testing**: Systematic parameter sweeping and validation

## Hardware Requirements

- **ESP32-P4** development board (ESP32-P4-Function-EV-Board recommended)
- **MIPI DSI Display Panel**: 
  - ILI9881C-based panels
  - EK79007-based panels
  - Other compatible MIPI DSI panels
- **Proper connections**: DSI data lanes, power rails (VCI, IOVCC), and control signals

## Key Features

### 🔧 Interactive Configuration
- Real-time parameter adjustment without recompilation
- Live preview of changes on actual hardware
- Persistent configuration during session

### 🔍 Comprehensive Diagnostics
- Framebuffer integrity testing
- DMA performance measurement
- Memory alignment verification
- MIPI DSI lane utilization analysis

### 🧪 Automated Testing
- Color rotation tests
- Parameter sweep automation
- Quick vs. comprehensive test modes
- Systematic timing validation

### 📊 Performance Analysis
- Transfer rate measurement
- Bandwidth utilization calculations
- Memory usage monitoring
- Timing parameter recommendations

## Quick Start

### 1. Build and Flash
```bash
idf.py build
idf.py flash monitor
```

### 2. Basic Diagnostic Workflow
Run these commands in sequence for efficient troubleshooting:

```bash
# 1. View current configuration
show

# 2. Initialize panel and display single frame
oneframe

# 3. Run comprehensive diagnostics
diagnose

# 4. Examine framebuffer contents (optional)
peek 32
```

### 3. Advanced Diagnostics
If issues persist, use advanced commands:

```bash
# Automated testing with color rotation and critical parameters
autotest quick

# Fine-tune horizontal back porch (most critical parameter)
sweep hs_bp 160 180 5

# Adjust DPI clock if timing issues persist
sweep dpi 55 65 2
```

## Available Commands

### Core Commands

| Command | Description | Example |
|---------|-------------|---------|
| `show` | Display current configuration | `show` |
| `set` | Modify a parameter | `set hs_bp 172` |
| `oneframe` | Initialize panel and display single frame | `oneframe` |
| `start` | Start continuous color cycling | `start` |
| `stop` | Stop continuous operation | `stop` |

### Diagnostic Commands

| Command | Description | Example |
|---------|-------------|---------|
| `diagnose` | Comprehensive system diagnostics | `diagnose` |
| `peek` | Examine framebuffer contents | `peek 32` |
| `sweep` | Parameter range testing | `sweep hs_bp 160 180 5` |
| `autotest` | Automated test sequence | `autotest quick` |

### Utility Commands

| Command | Description | Example |
|---------|-------------|---------|
| `reboot` | Restart the ESP32 | `reboot` |
| `help` | Show available commands | `help` |

## Configuration Parameters

### Display Timing Parameters
- `dpi` - DPI clock frequency (MHz)
- `hsize` / `vsize` - Panel resolution
- `hs_pw` / `hs_bp` / `hs_fp` - Horizontal sync timing
- `vs_pw` / `vs_bp` / `vs_fp` - Vertical sync timing

### MIPI DSI Parameters
- `lanes` - Number of data lanes (1-2 for ESP32-P4)
- `lane_rate` - Lane bit rate (Mbps)

### Power Management
- `vci_chan` / `vci_mv` - Analog power rail settings
- `iovcc_chan` / `iovcc_mv` - Logic power rail settings

### Advanced Options
- `use_dma2d` - Enable DMA2D acceleration
- `disable_lp` - Disable low-power mode
- `color` - Default display color

## Troubleshooting Guide

### Common Issues and Solutions

#### 🚫 Display Not Working
```bash
# Check basic configuration
show
oneframe
diagnose
```
**Look for**: Panel initialization failures, power rail issues

#### 📺 Flickering or Horizontal Lines
```bash
# Test critical timing parameters
sweep hs_bp 160 180 5
sweep dpi 55 65 2
```
**Common fixes**: Adjust `hs_bp` (most critical), fine-tune `dpi` clock

#### 🎨 Wrong Colors or Artifacts
```bash
# Test color integrity
autotest quick
```
**Check**: Color order, framebuffer alignment, DMA transfer integrity

#### ⚡ Performance Issues
```bash
# Analyze performance
diagnose
```
**Monitor**: DMA transfer rates, lane utilization, memory alignment

### Parameter Recommendations

#### For ILI9881C (800x1280) Panels:
```
dpi=60, hs_bp=172, hs_fp=32, hs_pw=16
vs_bp=40, vs_fp=26, vs_pw=4
lanes=2, lane_rate=600
```

#### For Testing New Panels:
1. Start with conservative values
2. Use `autotest quick` for initial validation
3. Fine-tune `hs_bp` first (most critical)
4. Adjust `dpi` clock for stability
5. Use `diagnose` to validate final configuration

## Diagnostic Output Interpretation

### ✅ Good Indicators
- ✓ Framebuffer memory is writable
- ✓ DMA transfer completed in <50ms
- ✓ Bandwidth within normal range (100-2000 Mbps)
- ✓ Lane utilization 20-80%
- ✓ Pattern integrity verified

### ⚠️ Warning Signs
- ⚠ Low bandwidth detected (<100 Mbps)
- ⚠ High lane utilization (>80%)
- ⚠ Memory alignment issues
- ⚠ DMA transfer timeout

### ❌ Critical Issues
- ✗ Framebuffer memory write test failed
- ✗ DMA transfer initiation failed
- ✗ Pattern corruption detected
- ✗ Panel initialization failures

## Development Notes

### Architecture
- **Control Panel**: Handles initialization commands via DBI interface
- **Data Panel**: Manages pixel data transfer via DPI interface
- **Framebuffer**: Single contiguous memory region from DPI panel
- **DMA**: Full-frame transfers for optimal performance

### Timing Considerations
- `hs_bp` (horizontal back porch) is the most critical parameter
- DPI clock should match panel specifications
- Lane rate must provide sufficient bandwidth for target refresh rate
- panels require vendor-specific initialization sequences

### Memory Management
- Uses DPI panel's internal framebuffer (no manual allocation)
- RGB888 format (3 bytes per pixel)
- DMA2D acceleration available for improved performance
- Memory alignment critical for DMA efficiency


---

For technical support or panel-specific configurations, please refer to the panel datasheet and ESP32-P4 LCD documentation.
