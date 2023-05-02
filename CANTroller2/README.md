# Getting started
1. Download Visual Studio Code (VSCode) [here](https://code.visualstudio.com/)
1. Open VSCode and install the extension "PlatformIO IDE" (Ctrl+Shift+X to add an extension on Mac)
1. After installation and reloading VSCode, click the PlatformIO icon on the LHS nav bar (alien head) 
1. Click "Import Arduino Project"
1. Navigate to the CANTroller2 folder in the file explorer
1. For the board option: select "Arduino Due: Programming Port"
1. Go to VSCode view of files, go to platformio.ini and add this line: `monitor_speed = 115200`

Now in the PlatformIO menu, you can click to "Build" (builds `cantroller2.cpp`) and then click "Upload" (to upload the compiled code to your board)


## Dev Env nice-to-haves
(this assumes you're using iTerm + Mac + VSCode)
- [Oh My Zsh](https://ohmyz.sh/#install) makes iTerm prompt a lot more colorful / easy to read.  But most importantly it's compatible with a number of other cool plugins. 

1. macos.  Check out the in-terminal short cuts you're offered [here](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/macos#commands)
1. zsh autosuggestions.  In iTerm, suggestions for end of line come up in gray.  Click R arrow to select; tab to see more options.  Needs to be [installed](https://github.com/zsh-users/zsh-autosuggestions) in addition to being added to ~/.zshrc file

FWIW, in my ~/.zshrc file, I have `plugins=(git zsh-autosuggestions macos)`