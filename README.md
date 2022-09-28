# Flip_Chip_Test_System
This Flip-Chip Tester System enables verification testing and debugging of most M-series and some G-series Flip Chip modules. The system is comprised of a Tester board and a Tester Controller board. An accessory called the IC Test Jig plugs into the UUT socket of the tester which enables functional testing of digital ICs.
<p>This tester PCB is heavily based on a tester designed by Michael Thompson with PCB by Vince Slyngstad. Refer to the StearnsMTvrs files in this folder: 
https://svn.so-much-stuff.com/svn/trunk/Eagle/projects/Stearns%20Tester/
<p>The original Flip-Chip Tester was designed by Warren Stearns.  A more detailed and complete history of the tester evolution is here: 
https://so-much-stuff.com/pdp8/repair/fc-tester.php
<p>This Flip Chip Tester PCB is an updated version of the StearnsMTvrs PCB. The changes to this board include:
<ul>
<li>Increased ground connections between the Tester Controller header and the GPIO Expander chips, and between the GPIO Expander chips and the UUT connector. This was accomplished by using smaller vias and re-routing traces that divided pieces of the ground plane. There are also many more vias that connect between layers and ground fill areas that connect across ground plane sections that are separated by signal traces. </li>
<li>A separate chip select signal is connected to each GPIO Expander.</li>
<li>Updated the pinout of the Tester Controller header to connect the separate SPI chip selects and VCC between the Tester and the Tester Controller circuit card.</li>
</ul>
<p>The Tester Controller circuit card has an ESP32 CPU to control the tester and microSD adapter card for storing test vector files. A possible future update is to combine the Tester and Tester Controller onto a single circuit board. 
<p>The original Stearns Tester software has been ported so it can be built in the Arduino development environment. Some software modifications were necessary due to use of the integrated SPI controller in the ESP32 to drive the SPI link to the GPIO Expanders and microSD adapter. Some changes were necessary to run this as an Arduino application. 
