## Activate venv 
### Windows
```bash
.\.venv\Scripts\Activate.ps1
```

If you dont have permissions to execute this command use: 
```bash
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
```

## CoppeliaSim Folder
### starters
Scripts used to initialize the comunnication with the Api. 
To apply, right click on any item of the scene hierarchy (normally the robot or floor), add -> script -> simulation script -> Threaded -> Lua. Here you should paste the content of the [Path to starter script](coppeliaSim/starters/script.lua).

## test
After doing this you can test the comunication via Api using [testing script](coppeliaSim/test/test_connection.py).

## control
