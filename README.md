# Drones_Localisation

After cloning the repository you must compile the CmakeList, for doing that get inside the folder, open a terminal window and run this command:

```
mkdir build && cd build && cmake ..
```

Then a makefile must be create. For build it:


```
make
```

At this point two executables have been created in your build folder:

	a) detection_photo : Test the algorithm over a drone’s images set.
	
	b) detection_video : Test this algorithm over a video of a “flying” drone.
  
For run them:

```
./detection_<'photo' or 'video'>
```
