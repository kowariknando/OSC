# OSC Motion Controller & Pendulum (CodeCell C3/C6)

This repository contains the firmware and software patches to use a CodeCell (ESP32-C3 or ESP32-C6) as a wireless motion controller. It reads accelerometer data, smooths it, and sends it via UDP (OSC) to a computer. 

The project includes a **Pure Data (PD)** patch that receives this UDP data and translates it into MIDI signals, which can be routed to any DAW or visual software (Ableton Live, Reaper, TouchDesigner, etc.) to control parameters like synthesizer cutoffs, effects, or video manipulation.

## 📂 Repository Structure
- `src/`: Contains the C++ firmware for the ESP32 microcontrollers.
- `include/`: Folder for your Wi-Fi credentials (see Setup section).
- `puredata/`: Contains the Pure Data `.pd` patches to receive UDP and output MIDI.
- `platformio.ini`: The configuration file that automates library management and board settings.

---

## 🛠️ 1. Firmware Setup (PlatformIO)

This project is built using **Visual Studio Code** and the **PlatformIO** extension. Please do not use the standard Arduino IDE, as PlatformIO automatically handles the complex library dependencies and custom ESP32 Core 3.x configurations required for the new ESP32-C6 chips.

### Prerequisites
1. Download and install [Visual Studio Code](https://code.visualstudio.com/).
2. Open VS Code, go to the Extensions tab, and install **PlatformIO IDE**.

### Installation Steps
1. **Clone or Download** this repository.
2. Open the cloned folder in VS Code (`File > Open Folder`).
3. Allow a few minutes for PlatformIO to initialize and download the necessary ESP32 toolchains automatically.
4. **⚠️ CRITICAL - Setup your Credentials:**
   The code requires a file with your Wi-Fi settings that is ignored by Git for security. 
   - Navigate to the `include/` folder.
   - Create a new file named `wifi_configs.h`.
   - Paste the following code and update it with your actual network details:
     ```cpp
     // include/wifi_configs.h
     #define WIFI_NAME "YOUR_WIFI_NAME"
     #define WIFI_PASS "YOUR_WIFI_PASSWORD"
     #define WIFI_IP   "192.168.1.XXX" // Your computer's local IP address
     ```

### Flashing the Board
This project supports both **CodeCell C3** and **CodeCell C6**.
1. Look at the blue bottom status bar in VS Code.
2. Click the Environment Switcher (it might say `env:codecell_c6` or `Default`).
3. Select the environment that matches your physical board (`codecell_c3` or `codecell_c6`).
4. Connect your CodeCell via USB.
5. Click the **Upload (→)** arrow button on the bottom bar. PlatformIO will automatically fetch all required libraries (CodeCell, ArduinoJson, Adafruit NeoPixel), compile the code, and flash the board.

---

## 🎹 2. Pure Data & MIDI Routing

Once the CodeCell is powered and connected to your Wi-Fi, it will start sending UDP packets to your computer. We use **Pure Data (PD)** to catch these packets and translate them into standard MIDI.

### Step 1: Virtual MIDI Cable
To send MIDI from Pure Data to your DAW (Ableton, Reaper) or TouchDesigner, you need a virtual MIDI bridge.

**🖥️ For Windows Users:**
Windows does not have native virtual MIDI ports.
1. Download and install [loopMIDI](https://www.tobias-erichsen.de/software/loopmidi.html).
2. Open loopMIDI and click the **+** button to create a new port (e.g., name it `loopMIDI Port`).
3. Leave the program running in the background.

**🍎 For macOS Users:**
macOS handles this natively via the IAC Driver.
1. Open the **Audio MIDI Setup** app (found in Applications > Utilities).
2. Go to `Window > Show MIDI Studio`.
3. Double-click the **IAC Driver** icon.
4. Check the box that says **"Device is online"** to enable it.

### Step 2: Pure Data Configuration
1. Install [Pure Data](https://puredata.info/downloads/pure-data).
2. Open the `.pd` patch located in the `puredata/` folder of this repository.
3. In Pure Data, go to the top menu: `Edit > Preferences > MIDI...` (or `Media > MIDI Settings`).
4. Set the **Output Device** to the virtual port you just created (`loopMIDI Port` on Windows, or `IAC Driver` on macOS).
5. Click Apply/OK.

### Step 3: DAW / Visualizer Setup
Open Ableton, Reaper, or TouchDesigner.
1. Go to the MIDI Preferences of your software.
2. Enable your virtual MIDI port (`loopMIDI` or `IAC Driver`) as a recognized **MIDI Input**.
3. Map the incoming MIDI CC values to your desired parameters!