
## VS Code IntelliSense
The C/C++ extension is configured to use `build/compile_commands.json`.
Activate the ESP-IDF environment and run `idf.py build` to generate this file.
If you change the build directory, also update `C_Cpp.default.compileCommands` in VS Code.