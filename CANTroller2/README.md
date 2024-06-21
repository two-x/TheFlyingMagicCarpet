# Flying Carpet Controller v3.1

## CPU Board

1. Our board is ESP32-S3-DevKitC-1-N8R8 (<https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1-v1.0.html>)
1. Note these come in v1 and v1.1, the difference being the neopixel is on pin 48 (v1) or pin 38 (v1.1), which is stupid. The code assumes pin 48 (v1) since those are more available it seems.
1. Buy one from here: <https://www.amazon.com/gp/product/B0BSCXHB5S/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1>
1. To set up a breadboard for development, make sure to connect pins as specified in the comments in .../Cantroller2/src/globals.h , as well as external cpacitors and resistors also detailed in the comments.
1. The board has two micro-USB ports labeled "UART" and "USB". Code can be uploaded to the board using either one of these. However, to see serial console output from the board you need to connect to the "UART" port (then open console output using the plug icon in bottom toolbar).
1. Sometimes uploading files or code will fail using one or the other of the two ports, so if that happens switch the port and try again.  Note also the upload data goes at 1MBps so crappy-ass microusb cords can result in failures too.

## Environment

1. Download Visual Studio Code (VSCode) [here](<https://code.visualstudio.com/>)
1. Open VSCode and install the extension "PlatformIO IDE" (Ctrl+Shift+X to add an extension on Mac) -OR- Run `brew install platformio`
1. Save the json file [here](<https://github.com/platformio/platform-espressif32/blob/master/boards/esp32-s3-devkitc-1.json>) as `~/.platformio/boards/esp32-s3-devkitc-1.json`
1. Clone the <https://github.com/two-x/TheFlyingMagicCarpet> github repo into your filesystem. You can do this from the command line or figure out how to do it from within vscode. Whatever you're into.
1. Click the platformio alien head in VScode, and pick "Projects...", "Add Existing", and find the TheFlyingMagicCarpet/Cantroller2 directory. Once platformio reads the platformio.ini file, a bunch of libraries should automatically install themselves.
1. Install clang-format linter. On MacOS you can do "brew install clang-format", otherwise use your package manager instead. Make sure clang-format is in your path. In VScode, In vscode, View-Extensions and install the Clang-format extension, then open the extension settings. Enter the full clang-format executable path into the "Executable" setting (eg "/opt/homebrew/bin/clang-format"). Enter ".clang-format" into the "Assume Filename" setting. Assuming you have the repo updated now you should be able to right-click and format code in the editor to make it all format consistently.

## Dev Env nice-to-haves

(this assumes you're using iTerm + Mac + VSCode)

1. [Oh My Zsh](<https://ohmyz.sh/#install>) makes iTerm prompt a lot more colorful / easy to read.  But most importantly it's compatible with a number of other cool plugins.
1. Check out the in-terminal short cuts you're offered [here](<https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/macos#commands>)
1. zsh autosuggestions.  In iTerm, suggestions for end of line come up in gray.  Click R arrow to select; tab to see more options.  Needs to be [installed](<https://github.com/zsh-users/zsh-autosuggestions>) in addition to being added to ~/.zshrc file. FWIW, in my ~/.zshrc file, I have `plugins=(git zsh-autosuggestions macos)`

## Uploading

1. The code base contains C++ source files in CANTroller2/src and also runtime files in Cantroller2/data, and configuration files in the CANTroller directory.
1. Before programming a new board, its flash must be partitioned correctly. Plug in board. Click the alien head and then "Platform/Erase flash". This will only need to be done again if changes are made to the partition map.
1. To compile the filesystem based on /data contents, under alien head click "Platform/Upload filesystem". The result (a file CANTroller/.pio/build/littlefs.bin) will be written to the board's littlefs partition over USB. If it fails see notes in hardware section above.
1. To compile the code in /src, click the alien head and then "Upload". The resulting binary (a file CANTroller/.pio/build/firmware.bin) will be sent to the board over USB.
1. USB upload: You can write these to the board flash over usb using "Platform/Upload Filesystem Image", or "Upload", respectively.
1. OTA upload: Alternately, load using the over the air updater. Power the board, log in to the "artcarpet" wifi ssid, and browse to 192.168.1.69/update . First select the intended OTA Mode for each file, then browse to the above files one at a time, to load and flash them.  Note: OTA can only work after board has already been programmed with code that supports OTA.

## Wifi

1. Once code is running, if web server is enabled then the board will act as an access point and web server. To access, scan for wifi networks on any nearby computer or phone, ssid = artcarpet, passwd = checkmate. Then browse to 192.168.1.69/ .

## Connected Hardware

1. Display
1. Touchscreen
1. SD Card
SD interface (not yet implemented) is integrated with touchscreen module. We hope to use this storage area for logging runtime data.  The 16GB SD card is pre-formatted in windows with FAT32 file system, 32kB allocation unit size. MacOS recognizes this as "MS-DOS (FAT32)".
1. Digital IO
1. Analog Inputs
1. Hotrc Radio Receiver
1. Rotary encoder
1. Neopixels
1. Servos/Jaguars
1. Hall Effect Sensors
1. Temperature Sensors
1. Manifold Mass Air Flow Sensors

## JTAG Debugging

1. [espressif guide](<https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/jtag-debugging/index.html#>)
1. [a youtube video](<https://www.youtube.com/watch?v=HGB9PI3IDL0&t=316s&ab_channel=Tech-Relief>)
1. [jtag interface tool for windows](<https://zadig.akeo.ie/>)

## Links

1. [free images](<http://iconarchive.com/>) 1. resize to pixels needed, jpg w/ black backgd  2. convert to rgb565 color
1. [image to rgb565 color converter](<https://www.youtube.com/redirect?event=video_description&redir_token=QUFFLUhqbkYtMGJvMS1VVWV0ZUpIb1Y4U2U2QzRLM3BKZ3xBQ3Jtc0tudG5MS1hVdmlLajdrNHFMWWtWUkFGTFNadUhaWkVob2ExNV8ya29kLXFmcDh1SEVINDFEeWtSX3A0SW40UlNTcy1CYVlSTTV5cXJKM25VcGxoWjdxSk9kZVFadURVWHhJcU9hMVRUWENyVGVjRkw4aw&q=http%3A%2F%2Fwww.rinkydinkelectronics.com%2Ft_imageconverter565.php&v=U4jOFLFNZBI>)
1. [rgb565 color picker](<http://www.barth-dev.de/online/rgb565>)
1. [dope rgb332 color picker](<https://roger-random.github.io/RGB332_color_wheel_three.js>)
1. [named colors](<https://wiki.tcl-lang.org/page/Colors+with+Names>)
1. [font0 character map (use right-side map)](<https://learn.adafruit.com/assets/103682>)
1. [SD card formatter](<https://www.sdcard.org/downloads/formatter/>)
