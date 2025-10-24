# Windows Setup Guide

Complete guide for installing and running MicroSim on Windows 10/11.

## ⚠️ Important: Recommended Approach for Windows

**TL;DR: Use WSL2 (Windows Subsystem for Linux) with Docker. This is the most reliable and performant option.**

Windows has four options for running ROS 2:

| Option | Pros | Cons | Recommendation |
|--------|------|------|----------------|
| **WSL2 + Docker** | Best performance, most compatible | Docker complexity | ✅ **BEST OVERALL** |
| **WSL2 + Robostack** | Linux compatibility, conda workflow, no Docker | Slight VM overhead | ✅ **RECOMMENDED** |
| **Robostack (Native)** | Lightest, no VM overhead, native Windows | Limited to Windows packages | ⚠️ If WSL2 unavailable |
| **Native Windows ROS 2** | Native Windows | Complex setup, limited packages | ⚠️ Advanced users only |
| **Docker Desktop Only** | Simple setup | Slower, display issues | ⚠️ Fallback option |

---

## Option 1: WSL2 + Docker (Recommended) ✅

This gives you a Linux environment on Windows with near-native performance.

### Prerequisites

- Windows 10 version 2004+ (Build 19041+) or Windows 11
- Administrator access
- At least 8GB RAM (16GB recommended)
- 20GB free disk space

### Step 1: Install WSL2

**Open PowerShell as Administrator** and run:

```powershell
# Enable WSL
wsl --install

# This will:
# - Enable WSL and Virtual Machine Platform
# - Install Ubuntu (default distribution)
# - Restart your computer (required)
```

After restart, Ubuntu will open automatically and prompt you to create a username/password.

**Verify WSL2 installation:**
```powershell
wsl --list --verbose
```

You should see Ubuntu with VERSION 2.

### Step 2: Install Docker Desktop for Windows

1. Download Docker Desktop from: https://www.docker.com/products/docker-desktop/
2. Run the installer
3. **Important:** During installation, ensure "Use WSL 2 instead of Hyper-V" is checked
4. Restart your computer
5. Start Docker Desktop
6. Go to Settings → General → Check "Use the WSL 2 based engine"
7. Go to Settings → Resources → WSL Integration → Enable for Ubuntu

### Step 3: Install VSCode (Optional but Recommended)

1. Download VSCode: https://code.visualstudio.com/
2. Install the "Remote - WSL" extension
3. Install the "Docker" extension

This allows you to edit code in Windows but run it in WSL2.

### Step 4: Clone Repository in WSL2

**Open WSL2 terminal** (search "Ubuntu" in Windows Start menu):

```bash
# Update WSL2
sudo apt update && sudo apt upgrade -y

# Install git
sudo apt install git -y

# Clone the repository (in your WSL2 home directory)
cd ~
git clone <your-repo-url> robotic-ai-agents
cd robotic-ai-agents/simulator/microsim
```

**Important:** Clone in WSL2 filesystem (`~/` or `/home/username/`), NOT in Windows filesystem (`/mnt/c/`). This is much faster.

### Step 5: Run with Docker

```bash
# Start ROS 2 container
docker-compose up -d

# Enter the container
docker-compose exec ros2 bash

# Inside container: Build and run
./docker-scripts/build.sh
./docker-scripts/run_node.sh
```

### Step 6: Visualization (X11 Forwarding)

**For matplotlib visualization on Windows, you need an X server:**

#### Option A: VcXsrv (Recommended)

1. Download VcXsrv: https://sourceforge.net/projects/vcxsrv/
2. Install and launch XLaunch
3. Configuration:
   - Display number: 0
   - Start no client
   - **Disable access control** (important!)
4. Allow through Windows Firewall when prompted

**In WSL2, set DISPLAY variable:**

```bash
# Get Windows IP from WSL2
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0

# Add to ~/.bashrc for persistence
echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk "{print \$2}"):0.0' >> ~/.bashrc
```

**Test X11:**
```bash
# Install x11-apps in WSL2
sudo apt install x11-apps -y

# Test
xclock  # Should show a clock window
```

**Now run visualization:**
```bash
cd ~/robotic-ai-agents/simulator/microsim
python3 scripts/viz_2d.py
```

#### Option B: WSLg (Windows 11 only)

Windows 11 has built-in GUI support:

```bash
# Should work out of the box
python3 scripts/viz_2d.py
```

If not working, update WSL2:
```powershell
# In PowerShell (as admin)
wsl --update
```

### Complete Workflow (WSL2 + Docker)

```bash
# Terminal 1: WSL2
cd ~/robotic-ai-agents/simulator/microsim
docker-compose up -d
docker-compose exec ros2 bash
ros2 run microsim microsim_node

# Terminal 2: WSL2
cd ~/robotic-ai-agents/simulator/microsim
python3 scripts/viz_2d.py

# Terminal 3: WSL2
cd ~/robotic-ai-agents/simulator/microsim
python3 scripts/drone_controller.py
```

**Opening multiple WSL2 terminals:**
- Option 1: Windows Terminal app (recommended) - open multiple Ubuntu tabs
- Option 2: Multiple Ubuntu windows from Start menu
- Option 3: VSCode terminal with WSL2 integration

---

## Option 2: WSL2 + Robostack (Conda) - Hybrid Approach

**Get Linux compatibility with the conda package manager approach.** This combines the best of both worlds: native Unix environment with Robostack's conda-based installation (no Docker complexity).

### How It Works

1. WSL2 provides a Linux VM on Windows
2. Install Miniforge/conda inside WSL2
3. Create ROS 2 environment with Robostack channel
4. Run everything directly in WSL2 (not in Docker)

This gives you full Linux compatibility while keeping the familiar conda workflow.

### Prerequisites

- Windows 10 version 2004+ or Windows 11
- WSL2 installed and configured (see Option 1, Steps 1-3)
- ~10GB free space in WSL2

### Step 1: Install Miniforge in WSL2

**Open WSL2 terminal:**

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Download Miniforge installer
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh

# Install
bash Miniforge3-Linux-x86_64.sh

# Follow prompts (press Enter for defaults)
# After installation, restart terminal
```

### Step 2: Create ROS 2 Environment

```bash
# Activate conda (may be automatic after installation)
conda init bash
source ~/.bashrc

# Create environment
conda create -n ros2_humble -c robostack-staging ros-humble-desktop python=3.11 -y

# Install colcon
conda activate ros2_humble
conda install -c robostack-staging colcon-common-extensions -y

# Install dependencies
conda install -c conda-forge matplotlib numpy pyyaml opencv-python -y
```

### Step 3: Clone and Build Repository

```bash
# Clone (in WSL2 filesystem for best performance)
cd ~
git clone <your-repo-url> robotic-ai-agents
cd robotic-ai-agents/simulator/microsim

# Build
conda activate ros2_humble
colcon build --packages-select microsim
source install/setup.bash
```

### Step 4: Run Simulator

```bash
# Terminal 1: Simulator
conda activate ros2_humble
cd ~/robotic-ai-agents/simulator/microsim
source install/setup.bash
ros2 run microsim microsim_node

# Terminal 2: Visualization
conda activate ros2_humble
cd ~/robotic-ai-agents/simulator/microsim
python scripts/viz_2d.py

# Terminal 3: Controller (interactive, works natively!)
conda activate ros2_humble
cd ~/robotic-ai-agents/simulator/microsim
python scripts/drone_controller.py
```

### Advantages of WSL2 + Robostack

- ✅ Full Linux compatibility (interactive controller works natively)
- ✅ Familiar conda workflow
- ✅ No Docker complexity
- ✅ Direct access to Linux package ecosystem
- ✅ Better performance than native Windows (WSL2 is optimized)
- ✅ Simpler than Docker, more compatible than native Windows

### When to Choose This Option

- ✅ WSL2 is available and you want to avoid Docker
- ✅ You want native Unix compatibility for interactive controller
- ✅ You're comfortable with conda from macOS
- ✅ You want the simplest path to full Linux environment
- ⚠️ Slightly more overhead than native Windows

---

## Option 3: Robostack (Conda) - Native Windows Alternative

**Similar to the macOS setup, Windows users can also use Robostack with conda for a purely native Windows installation.** This is useful if WSL2 is not available or if you prefer zero virtualization overhead.

### How Robostack Works on Windows

Like on macOS, Robostack provides pre-built ROS 2 binaries through the conda package manager:

- **Package Manager:** Conda/Miniforge provides dependency management
- **Pre-built Binaries:** Robostack channel (`robostack-staging`) hosts compiled ROS 2 packages
- **Installation:** Everything installs into your conda environment directory
- **No separate ROS 2 installer needed:** Unlike native Windows binary installation

### Prerequisites

- Windows 10 version 1909+ or Windows 11
- **Visual Studio 2022 with C++ Build Tools** (this is the key requirement for Windows)
  - Download: https://visualstudio.microsoft.com/visual-cpp-build-tools/
  - During installation: Select "Desktop development with C++"
  - You need this for building ROS 2 packages from source
- Administrator access
- At least 15GB free disk space

### Step 1: Install Visual Studio 2022 Build Tools

1. Download Visual Studio Build Tools: https://visualstudio.microsoft.com/visual-cpp-build-tools/
2. Run the installer
3. Select **"Desktop development with C++"** workload
4. Complete installation (this is required for conda ROS 2 packages)

### Step 2: Install Miniforge (Conda)

**Open PowerShell and run:**

```powershell
# Download Miniforge installer
curl -L -o miniforge.exe https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Windows-x86_64.exe

# Run installer
.\miniforge.exe

# Choose installation path (default is usually fine)
# Check "Add Miniforge to PATH"
```

**Verify installation:**
```powershell
conda --version
```

### Step 3: Create ROS 2 Conda Environment

```powershell
# Create new environment with ROS 2 Humble
conda create -n ros2_humble -c robostack-staging ros-humble-desktop python=3.11 -y

# This installs:
# - ROS 2 Humble (core)
# - Pre-built binary packages
# - Python 3.11
# - All dependencies via conda
```

### Step 4: Install Development Tools

```powershell
# Activate environment
conda activate ros2_humble

# Install colcon and additional tools
conda install -c robostack-staging colcon-common-extensions -y

# Install Python dependencies for our simulator
conda install -c conda-forge matplotlib numpy pyyaml opencv-python -y
```

### Step 5: Clone and Build MicroSim

```powershell
# Create workspace
mkdir robotic-ai-agents
cd robotic-ai-agents

# Clone repository (or copy your code)
git clone <your-repo-url> .

# Activate environment
conda activate ros2_humble

# Source ROS 2 setup
# On Windows, use the batch file instead of bash
call %CONDA_PREFIX%\etc\conda\activate.d\setup_ros2.bat

# Navigate to simulator
cd simulator\microsim

# Build with colcon
colcon build --packages-select microsim
call install\setup.bat
```

### Step 6: Run Simulator

```powershell
# Terminal 1: Simulator node
conda activate ros2_humble
cd robotic-ai-agents\simulator\microsim
call install\setup.bat
ros2 run microsim microsim_node

# Terminal 2: Visualization
conda activate ros2_humble
cd robotic-ai-agents\simulator\microsim
python scripts\viz_2d.py

# Terminal 3: Controller (if using autonomous controller)
conda activate ros2_humble
cd robotic-ai-agents\simulator\microsim
python scripts\autonomous_drone_controller.py
```

### Windows-Specific Considerations

#### 1. Path Separators
Windows uses backslashes (`\`) instead of forward slashes (`/`). Our Python code should handle this automatically, but be aware when writing paths manually.

#### 2. Interactive Controller Issues
The `drone_controller.py` uses Unix-only `termios` library. Options:

**Option A:** Modify for Windows using `msvcrt`:
```python
# Add to drone_controller.py
import sys
if sys.platform == 'win32':
    import msvcrt
    def get_key():
        if msvcrt.kbhit():
            return msvcrt.getch().decode()
        return None
else:
    import termios, tty
    def get_key():
        # Unix version...
```

**Option B:** Use WSL2 for interactive controller (WSL has full Unix compatibility)

#### 3. Terminal Emulator
- Recommended: **Windows Terminal** (modern, handles conda well)
- Alternative: PowerShell or Command Prompt
- Avoid: Basic cmd.exe (poor conda integration)

### Troubleshooting Robostack on Windows

#### Issue: "Visual C++ Build Tools not found"
```
CMake error: MSVC not found
```

**Solution:** Install Visual Studio 2022 Build Tools with C++ support (see Step 1)

#### Issue: "No module named 'rclpy'"
```
ModuleNotFoundError: No module named 'rclpy'
```

**Solution:** Make sure to activate conda environment and source ROS 2:
```powershell
conda activate ros2_humble
call %CONDA_PREFIX%\etc\conda\activate.d\setup_ros2.bat
```

#### Issue: "colcon not found"
```
'colcon' is not recognized as an internal or external command
```

**Solution:** Install colcon in the environment:
```powershell
conda activate ros2_humble
conda install -c robostack-staging colcon-common-extensions -y
```

#### Issue: Long file paths causing issues

Windows has a 260-character path limit in older versions. Solutions:

```powershell
# Option 1: Enable long path support (Windows 10+ v1607)
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force

# Option 2: Keep paths short
# Clone to: C:\ws instead of C:\Users\YourName\Documents\Projects\...
```

### Comparison: Robostack Native Windows vs WSL2 Approaches

| Aspect | Native Robostack (Conda) | WSL2 + Robostack | WSL2 + Docker |
|--------|--------------------------|------------------|---------------|
| **Setup Time** | 20 minutes | 25 minutes | 30 minutes |
| **Setup Complexity** | Medium (VS2022) | Low (just conda) | Medium (Docker) |
| **Layers** | 1 (Windows → ROS 2) | 2 (Windows → Linux → ROS 2) | 3 (Windows → Linux → Docker → ROS 2) |
| **Native Windows** | Yes ✓ | No (Linux VM) | No (Linux VM) |
| **Performance** | 85-90% | 90-95% | 95%+ |
| **File Access** | Direct Windows paths | WSL2 paths (faster) | WSL2 paths (fastest) |
| **Compatibility** | Good (Windows Robostack) | Excellent (full Linux) | Excellent (full Linux) |
| **Interactive Controller** | Needs msvcrt modification | Works natively | Works natively |
| **Resource Usage** | Lightest | Medium (VM overhead) | Medium (VM + Docker) |
| **Package Availability** | Good (core packages) | Excellent (all Linux packages) | Excellent (all Linux packages) |
| **When to Use** | Prefer Windows, WSL2 unavailable | Hybrid (Windows IDE + Linux tools) | Best compatibility, Docker workflows |

### When to Choose Each Approach

#### Option 1: WSL2 + Docker (Best Overall Performance)
- ✅ You want maximum compatibility and performance (95%+ of Linux)
- ✅ You use Docker in your workflow
- ✅ You want reproducible containerized environments
- ✅ Interactive controller works natively
- ✅ Best for production deployments
- ⚠️ More setup complexity with Docker
- **Recommendation:** Choose this if you want the absolute best performance and compatibility

#### Option 2: WSL2 + Robostack (Best Balanced Approach) ⭐
- ✅ WSL2 is available but you prefer conda over Docker
- ✅ You want native Unix compatibility (interactive controller works perfectly)
- ✅ You want full Linux package ecosystem
- ✅ You prefer simpler setup than Docker
- ✅ You're familiar with conda from macOS
- ✅ You edit code in Windows IDE but want Linux tools
- **Recommendation:** This is the sweet spot—Linux compatibility with conda workflow

#### Option 3: Robostack Native (Pure Windows, No VM)
- ✅ WSL2 is not available (older Windows, hardware limitations)
- ✅ You want purely native Windows development (no VM overhead)
- ✅ You're familiar with conda from macOS
- ✅ You want lightest resource footprint (85-90% performance)
- ❌ You need interactive controller support without modification
- ❌ You need full Linux package ecosystem
- **Recommendation:** Use only if WSL2 is unavailable

---

## Option 4: Native Windows ROS 2 (Advanced)

**⚠️ Warning:** This is more complex and some ROS 2 packages may not be available on Windows. This is different from Robostack—it's the official ROS 2 Windows binary installation.

### Prerequisites

- Windows 10/11 (64-bit)
- Visual Studio 2019 or 2022 (Community Edition is free)
- Chocolatey package manager

### Step 1: Install Chocolatey

**Open PowerShell as Administrator:**

```powershell
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

### Step 2: Install Dependencies

```powershell
# Install Python 3.11
choco install python311 -y

# Install other tools
choco install cmake git visualstudio2022buildtools -y
```

### Step 3: Install ROS 2 Humble

1. Download ROS 2 Humble for Windows: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
2. Extract to `C:\dev\ros2_humble`
3. Run `C:\dev\ros2_humble\local_setup.bat` in every terminal

### Step 4: Install Python Dependencies

```powershell
# In a command prompt with ROS 2 sourced
python -m pip install numpy pyyaml matplotlib opencv-python
```

### Step 5: Build MicroSim

```powershell
cd C:\dev\robotic-ai-agents\simulator\microsim

# Create workspace
mkdir ..\ws
cd ..\ws
mkdir src
mklink /D src\microsim ..\microsim

# Build
call C:\dev\ros2_humble\local_setup.bat
colcon build --packages-select microsim
call install\setup.bat
```

### Step 6: Run

```powershell
# Terminal 1
call C:\dev\ros2_humble\local_setup.bat
call install\setup.bat
ros2 run microsim microsim_node

# Terminal 2
call C:\dev\ros2_humble\local_setup.bat
python scripts\viz_2d.py
```

### Known Issues on Windows Native

1. **matplotlib backend issues** - May need to set backend:
   ```python
   # Add to viz_2d.py before importing pyplot
   import matplotlib
   matplotlib.use('TkAgg')
   ```

2. **Path separators** - Windows uses `\` instead of `/`. Our code should handle this, but be aware.

3. **Terminal control** - The drone controller uses `termios` which is Unix-only. You'll need to use:
   - WSL2 for the interactive controller
   - OR modify it to use `msvcrt` (Windows keyboard library)

4. **Some ROS 2 packages missing** - Not all packages available on Windows

---

## Option 5: Docker Desktop Only (Fallback)

If WSL2 doesn't work for some reason, you can use Docker Desktop alone.

### Setup

1. Install Docker Desktop for Windows (Hyper-V mode)
2. Enable Linux containers
3. Clone repository in Windows
4. Run from Windows Command Prompt:

```powershell
cd C:\path\to\robotic-ai-agents\simulator\microsim
docker-compose up -d
docker-compose exec ros2 bash
```

### Display Issues

Without WSL2, X11 forwarding is more complicated. Options:

1. **VcXsrv** (same as WSL2 setup above)
2. **Web-based visualization** (would require modification)
3. **headless mode** - Run simulation without visualization

---

## Recommended Setup Summary

### Setup Path 1: WSL2 + Docker (Best Overall Performance) 🥇

```
Windows 10/11
    ↓
WSL2 (Ubuntu)
    ↓
Docker Desktop (with WSL2 integration)
    ↓
MicroSim Container
    ↓
VcXsrv (for visualization)
```

- **Setup time:** ~30 minutes
- **Performance:** 95%+ of native Linux
- **Complexity:** Medium
- **Reliability:** High ✅
- **When:** Maximum compatibility needed, using Docker workflows

### Setup Path 2: WSL2 + Robostack (Balanced - Recommended) ⭐ 🥈

```
Windows 10/11
    ↓
WSL2 (Ubuntu)
    ↓
Miniforge (Conda)
    ↓
Robostack ROS 2 Environment
    ↓
MicroSim via colcon build
```

- **Setup time:** ~25 minutes
- **Performance:** 90-95% of native Linux
- **Complexity:** Low
- **Reliability:** High ✅
- **When:** Want Linux compatibility with familiar conda workflow
- **Best For:** Most users—Linux tools without Docker complexity

### Setup Path 3: Native Robostack (Pure Windows) 🥉

```
Windows 10/11
    ↓
Visual Studio 2022 Build Tools
    ↓
Miniforge (Conda)
    ↓
Robostack ROS 2 Environment
    ↓
MicroSim via colcon build
```

- **Setup time:** ~20 minutes
- **Performance:** 85-90% of native Linux
- **Complexity:** Medium (VS2022 required)
- **Reliability:** High ✅
- **When:** WSL2 unavailable or no virtualization wanted
- **Trade-off:** No interactive controller support without modification

---

## Common Issues & Solutions

### Issue: "WSL 2 requires an update to its kernel component"

**Solution:**
```powershell
wsl --update
```

Or download manually: https://aka.ms/wsl2kernel

### Issue: Docker can't start containers

**Solution:**
1. Open Docker Desktop
2. Go to Settings → Resources → WSL Integration
3. Enable integration for Ubuntu
4. Restart Docker Desktop

### Issue: X11 not working (can't see matplotlib windows)

**Solution:**
1. Make sure VcXsrv is running
2. Disable firewall for VcXsrv OR add firewall rule for WSL2
3. Check DISPLAY variable:
   ```bash
   echo $DISPLAY  # Should show IP:0.0
   ```

### Issue: "Permission denied" errors in WSL2

**Solution:**
```bash
# Fix ownership
sudo chown -R $USER:$USER ~/robotic-ai-agents
```

### Issue: Slow performance

**Solution:**
- Make sure files are in WSL2 filesystem (`~/`), not Windows (`/mnt/c/`)
- Increase Docker memory in Docker Desktop settings
- Enable WSL2 memory limits in `.wslconfig`

### Issue: Can't use interactive drone controller

**Solution:**
The `drone_controller.py` uses Unix-only `termios`. Options:

1. **Use WSL2** (has full Linux compatibility)
2. **Modify for Windows:** Replace `termios` with `msvcrt`:
   ```python
   # Windows version
   import msvcrt
   def get_key():
       if msvcrt.kbhit():
           return msvcrt.getch().decode()
       return None
   ```
3. **Use alternative:** Control via ROS 2 commands instead

---

## Performance Comparison

| Metric | Native Linux | WSL2 + Docker | Native Windows | Docker Desktop |
|--------|--------------|---------------|----------------|----------------|
| Setup Time | Fast | Medium | Slow | Fast |
| Performance | 100% | 95% | 90% | 80% |
| Compatibility | Perfect | Excellent | Good | Excellent |
| Ease of Use | Easy | Medium | Hard | Medium |

---

## File Locations

### WSL2 File System

- **WSL2 home:** `\\wsl$\Ubuntu\home\username\` (accessible from Windows Explorer)
- **Windows from WSL2:** `/mnt/c/Users/YourName/`

**Tip:** Right-click folder in Windows Explorer → "Open Linux shell here" to open WSL2 terminal

### Editing Code

**Option 1: VSCode with WSL2**
1. Install "Remote - WSL" extension
2. Click green icon (bottom-left) → "Connect to WSL"
3. Open folder in WSL2 filesystem
4. Edit in Windows, run in Linux

**Option 2: Windows Editor + WSL2 Terminal**
1. Access WSL2 files: `\\wsl$\Ubuntu\home\username\robotic-ai-agents`
2. Edit with any Windows editor
3. Run in WSL2 terminal

---

## Quick Start (Copy-Paste Ready)

### Complete WSL2 + Docker Setup

```powershell
# 1. Install WSL2 (PowerShell as Admin, requires restart)
wsl --install

# 2. After restart, in WSL2/Ubuntu terminal:
cd ~
git clone <your-repo> robotic-ai-agents
cd robotic-ai-agents/simulator/microsim

# 3. Set up X11 display (for visualization)
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0
echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk "{print \$2}"):0.0' >> ~/.bashrc

# 4. Start Docker container
docker-compose up -d
docker-compose exec ros2 bash

# 5. Inside container
./docker-scripts/build.sh
./docker-scripts/run_node.sh
```

**Don't forget:** Install VcXsrv on Windows and launch it before step 3!

---

## Support

If you encounter issues not covered here:

1. Check Docker Desktop is running
2. Verify WSL2 version: `wsl --list --verbose` (should show VERSION 2)
3. Check our [main documentation](../README.md)
4. Open a GitHub issue with:
   - Windows version (run `winver`)
   - WSL version (`wsl --version`)
   - Docker version (`docker --version`)
   - Error message

---

## Summary

**For Windows developers:** Three excellent paths exist!

### Path 1: WSL2 + Docker (Best Overall) 🥇
- ✅ **Maximum performance:** 95%+ of native Linux
- ✅ **Best compatibility:** Full Linux environment
- ✅ **Docker support:** Built-in for reproducible deployments
- ✅ VcXsrv for visualization
- ✅ VSCode with Remote-WSL for seamless editing
- ✅ Keep files in WSL2 filesystem for speed
- **Setup time:** 30 minutes | **Complexity:** Medium

### Path 2: WSL2 + Robostack (Recommended) ⭐ 🥈
- ✅ **Familiar conda workflow** (like macOS)
- ✅ **Native Unix compatibility** (interactive controller works!)
- ✅ **Full Linux package access** (90-95% performance)
- ✅ **Simple setup** (no Docker complexity)
- ✅ Best for hybrid workflows (Windows IDE + Linux tools)
- **Setup time:** 25 minutes | **Complexity:** Low
- **Recommendation:** Best for most users

### Path 3: Native Robostack (Windows Only) 🥉
- ✅ **Pure Windows, no VM** (lightest footprint)
- ✅ **Familiar conda workflow** (like macOS)
- ✅ **Minimal overhead:** 85-90% performance
- ⚠️ **Limited to Windows packages** (not all available)
- ⚠️ **Interactive controller needs modification** (msvcrt)
- **Setup time:** 20 minutes | **Complexity:** Medium
- **When:** WSL2 unavailable only

### Avoid
- ❌ Native Windows ROS 2 binary installation (complex, limited packages)
- ❌ Docker Desktop without WSL2 (slower, display issues)

**Key Insight:** Windows has three viable approaches, with **WSL2 + Robostack being the sweet spot** for most developers—it gives you Linux compatibility with the familiar conda workflow you know from macOS!

Good luck! 🚀
