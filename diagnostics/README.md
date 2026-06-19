# Diagnostics

Small tools to check that the Arduinos are talking to the Raspberry Pi.
You run them in a terminal **on the Pi**, not in MATLAB. They only *listen* —
they can't change or break anything, so feel free to try them.

## To stop any tool

Hold **Ctrl** and press **C**. The tool stops and you get your prompt back.

## The tools

Copy a line, paste it into the terminal, press Enter. Run **one at a time**.

**Is each Arduino plugged in and reachable?**
```bash
./diagnostics/check_network.sh
```
Pings every board. You want to see each one come back "reachable."

**Which Arduinos are sending data to the Pi right now?**
```bash
python3 diagnostics/check_splitter.py
```
Each board that's sending shows up by its IP address. Good for checking the
splitter is carrying more than one board.

**What is one board actually sending? (raw, no interpretation)**
```bash
python3 diagnostics/raw_listen.py 55020
```
Shows the raw data arriving on one port. Change `55020` to whatever port you
want to watch. Use this when something looks wrong and you want the plain truth.

**What are the acoustic boards sending? (as numbers)**
```bash
python3 diagnostics/check_acoustic_fields.py
```
Shows every field (az, range, detect, mic_id, board_id, zone) from every
acoustic board, labelled by board and field. Boards that aren't plugged in
just don't appear — it works with whatever is connected.

**What is the RF board sending? (as numbers)**
```bash
python3 diagnostics/check_rf.py
```
Shows each RF value (range, azimuth, elevation, etc.) as a number.

## If you see nothing

The board isn't sending to this Pi/port. Check that the sender (the Arduino
sketch or the MATLAB/Simulink UDP block) is pointed at the Pi's IP address and
the right port number. `raw_listen.py` is the best tool for figuring this out —
it shows the real address the data is coming from.
