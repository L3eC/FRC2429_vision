`main.py` is our main program.

- How do we upload programs?
    - Connect to the network the Raspberry Pi is connected to then navigate to wpilibpi.local, then use the steps described [in the docs](https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/the-raspberry-pi-frc-console.html)

- What network is the Raspberry Pi connected to?
    - If it's on the robot, it's connected to the robot's wifi. If you're trying to test at home, connect your Pi to your router via an ethernet cable, and then your pi will be connected to your home network. Now, connected to internet as usual, you can use wpilibpi.local

- Why can't I see raspberry pi outputs on outlineviewer/glass/networktables at home?
    - Make sure to call NetworkTableInstance.startServer(). Even if nt client mode is off in the wpilib.local dashboard, you have to do this. Once you do that, connect to the pi's IP (which you can find using angry ip scanner on a windows machine) and you should see it.

- What is the pi's IP (for sshing into there)?
    - For our robots, it's usually 10.24.29.12 or 10.24.29.13

