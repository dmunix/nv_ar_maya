SETLOCAL
SET PATH=%PATH%;projects\external\opencv\bin;C:/Program Files/Autodesk/maya2019/bin;
build\Release\AR_maya_bodyTracking.exe  --model_path=models --temporal=true --in_file=videos/sample_rom_low.mp4 --offline_mode=true --writeOutFiles=false