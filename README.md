# NAO
It's Not Another Odometry library

This library is my first attempt at making an odometry library for my robotics team (this library is just a few custom classes that go in a single-file VEXcode V5 web project). This is as much a learning opportunity for me as it is a useful tool for someone else (I just thought I might as well share it online, and code backups are nice).

## References
I started working on this after getting my hands dirty trying to port [JAR-template](https://github.com/JacksonAreaRobotics/JAR-Template) to PROS and realizing how little I understood about odometry and c++. If this goes well, I may attempt to make a library similar to either this or JAR-template for PROS (no promises). While I won't provide any resources for learning c++, here are some useful ones for odometry and PID which acted as inspiration for this library:
- https://wiki.purduesigbots.com/software/odometry
  - The original document this page is based on can be found [here](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf), although it may show a warning or fail to download depending on your browser.
- https://georgegillard.com/resources/documents

## Installing
**Copying just the library**
- Since this is all designed to fit within the one-file restraint of the VEXcode V5 web editor, installing it is pretty straightforward. Just copy everything in from `/* --- Start drive library --- */` to `/* --- Start robot configuration --- */` into the top of your main project file (after the collapsible section in the web editor). For our project, I've opted to manually configure the motors, controller, etc., but the automatic (GUI) configuration works too.

**Using the entire project as a template**
- You can also download the [vexcode project file](<./NAO library.v5cpp>) and upload it to the vexcode web editor. From here, you can modify our main driver control and autonomous code. I highly recommend looking through the library and understanding how it works, in accordance with [this](https://www.robotevents.com/V5RC/2023-2024/QA/1582) official ruling (from 2023-2024, appologies if there is something more recent).

## Using
Initialize the drive class with the left and right side motor groups (only tank drive is supported), the directions for the left and right side motor groups (optional, default left forward and right reversed), the inertial sensor and the controller. The code has an example of this in the body of the `main` function, and looking at the constructors in the Drive class itself should be fairly self-explanatory. The inertial sensor isn't optional (this library is designed for my team's use case, and we use an inertial).

The Drive class has a default drive function that can be called every frame for basic tank driver control. This is demonstrated in the `main` function as well. Since the odometry library isn't finished yet, there is no other functionality.

## Contributing
I'm not really looking for contributions, since this is my team's current robot code. If you find a bug, issue reports could be helpful, and pull requests might be accepted but I won't accept anything that changes functionality (most likely). Feel free to fork or copy this, but give credit as appropriate.

## License
See [LICENSE](./LICENSE). All code is under the GPL v3 unless otherwise specified, or unless I don't have rights to it (if that somehow happens and you notice, make an issue to let me know to add a disclaimer or remove it).

## Shoutouts
Check out [JAR-template](https://github.com/JacksonAreaRobotics/JAR-Template), it has a lot of cool functionality. Also check out [PROS](https://github.com/purduesigbots/pros), a cool open-source alternative to VEXcode.
