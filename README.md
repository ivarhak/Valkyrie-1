This repo contains the files for the flight computer on my "road to 1 kilometer" rocket named Valkyrie 1

The "display code" controls a TFT screen using an arduino feather v2 and displays some useful information like altitude records, graphs, pressure, acceleration.
the display is mainly to check that everything is working before launch and to make it easier to diagnose a launch afterwords, acting like a failsafe for the most important data points like how far it flew.

The data logging code is for a raspberry pi based adalogger feather board thats made to log data onto a micro sd card. It logs pretty much anything youd want.

Both computers are connected to a switch that when pressed zeroes sensors, so you can simply place the rocket on the rod, zero it, and have it ready.

Sensors used are a BMP390 and ADXL375, both adafruit

Im 100% okay with anyone using this code for whatever they want I just kindly ask you put me in the credits or notify me, Id love to see anything you make from this!
If you have any questions or want the pcb files feel free to contact me at ivarhak08@gmail.com
