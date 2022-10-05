<b>known issues Flip Chip Tester PCB</b>
<p>
<ol>
<li>Testpoint pads are too small - the testpoint pads are too small and do not provide a strong base. After moderate use, the solder joints on some of the testpoint pins can come loose. A fix has been identified, the testpoint pads are larger, to be included in v3.</li>
<li>Some cuts and jumps are required. A fix will be included in v3.</li>
<li>There are two current limiting resistors for the UUT power LED. Delete one of them. A fix will be included in v3.</li>
<li>Header PC1 is 26 pins. Need only 20 pins. Use a 20-pin symbol instead.</li>
<li>Add the ESP32 to the Tester PCB as a stuff option, so the Test System can be built with only one PCB (without the Tester Controller board). Leave the 20-pin header on the Tester PCB schematic so the old configuration is still possible by installing the 20-pin header and not installing the ESP32.</li>
<li>The UUT power switch is nice but mechanically weak. Figure out another option.</li>
<li>IC6 is not needed. Remove it.</li>
<li>IC3 pin 23 GPIO signal is connected directly to VPP. Instead, connect IC3 pin 23 through a small-value resistor to VPP.</li>
<li>IC2 pin 1 GPIO signal is connected directly to GND. Instead, connect IC2 pin 1 through a small-value resistor to GND.</li>
<li>Mounting holes, 6 places, need updated pads for soldermask clearance underneath screw heads.</li>
</ol>
