# nv_ar_maya
Using Nvidia AR SDK to bring body tracking data into maya.

![Powered By: NVIDIA RTX](https://user-images.githubusercontent.com/29297318/159997280-2131e876-42bd-4c8b-9472-7c88c6c7ba60.png)

# Projects
- AR_sample_body : AR SDK used for body tracking, use this to test that your Nvidia AR setup is correct
- AR_maya_sample : This is a app to maya connection example, using Socket to connect 
- AR_maya_bodyTracking : This is the body tracking app, allows streaming video to maya and bake the animation.

# Installation
- Install CMake
- Install Visual Studio 2019 or older
- Install [Nvidia Augmented Reality SDK](https://developer.nvidia.com/maxine-getting-started#augmented-reality-sdk) 
- Add the bin directory to the Environment variables or to add them to the %PATH% in the ".bat" files before running the apps.
- Edit modules/FindARSDK.cmake to the SDK installed directory
- Using VSCode to configure CMake or build the project using CMake making sure to use Release as target

# Running the apps
To run the app there are 2 bat files which configure the command, set the models_path to the model folder of your AR SDK install.
If using the video app, set the video file on the .bat itself.


