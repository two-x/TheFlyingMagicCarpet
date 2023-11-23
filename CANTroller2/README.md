# Hardware
1. Our board is ESP32-S3-DevKitC-1-N8 which can be got from here: https://www.amazon.com/gp/product/B0BSCXHB5S/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
2. To set up a breadboard for development, make sure to connect pins as specified in the comments in .../Cantroller2/src/globals.h , as well as external cpacitors and resistors also detailed in the comments.
3. The board has two micro-USB ports labeled "UART" and "USB". Code can be uploaded to the board using either one of these. However, to see serial console output from the board you need to connect to the "UART" port.
4. Sometimes uploading files or code will fail using one or the other of the two ports, so if that happens switch the port and try again.  Note also the upload data goes at 1MBps so crappy-ass microusb cords can result in failures too.

# Environment
1. Download Visual Studio Code (VSCode) [here](https://code.visualstudio.com/)
2. Open VSCode and install the extension "PlatformIO IDE" (Ctrl+Shift+X to add an extension on Mac)
3. Clone the https://github.com/two-x/TheFlyingMagicCarpet github repo into your filesystem. You can do this from the command line or figure out how to do it from within vscode. Whatever you're into.
4. Click the platformio alien head in VScode, and pick "Projects...", "Add Existing", and find the TheFlyingMagicCarpet/Cantroller2 directory. Once platformio reads the platformio.ini file, a bunch of libraries should automatically install themselves.
5. Edit the display library setup file. See directions in the .../Cantroller2/src/tftsetup.h file.

# Uploading
1. The code base contains C++ source files in <project>/src and also runtime files in <project>/data.
2. Each board needs a one-time filesystem format. Connect the board, click the alien head and then "Build filesystem". If it fails see notes in hardware section above.
3. Upload files in data directory: click the alien head and then "Upload Filesystem". Redo this whenever files have been changed.
4. Compile and upload code: click the alien head and then "Upload". Redo this whenever code has been modified.

# Wifi
1. Once code is running, if web server is enabled then the board will act as an access point and web server. This can be accessed from your computer or another computer or phone or whatever, by doing this:
2. Scan for wifi networks, look for "magiccarpet". Use password "checkmate".
3. Open a browser, navigate to 192.168.1.69/ . That should do it.

--
Notes from before:

Now in the PlatformIO menu, you can click to "Build" (builds `cantroller2.cpp`) and then click "Upload" (to upload the compiled code to your board)

# Install platformio etc  on Mac
1. Run `brew install platformio`
2. Save the json file [here](https://github.com/platformio/platform-espressif32/blob/master/boards/esp32-s3-devkitc-1.json) as `~/.platformio/boards/esp32-s3-devkitc-1.json`


## Dev Env nice-to-haves
(this assumes you're using iTerm + Mac + VSCode)
- [Oh My Zsh](https://ohmyz.sh/#install) makes iTerm prompt a lot more colorful / easy to read.  But most importantly it's compatible with a number of other cool plugins. 

1. macos.  Check out the in-terminal short cuts you're offered [here](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/macos#commands)
1. zsh autosuggestions.  In iTerm, suggestions for end of line come up in gray.  Click R arrow to select; tab to see more options.  Needs to be [installed](https://github.com/zsh-users/zsh-autosuggestions) in addition to being added to ~/.zshrc file

FWIW, in my ~/.zshrc file, I have `plugins=(git zsh-autosuggest

ions macos)`