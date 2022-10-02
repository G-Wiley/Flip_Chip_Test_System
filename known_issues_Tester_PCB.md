<b>known issues Flip Chip Tester PCB</b>
<p>
<ol>
<li>Testpoint pads too small - the testpoint pads are too small and do not provide a strong base. After moderate use the solder joints on some of the testpoint pins 
can come loose. Fix is identified, to be included in v3.</li>
<li>Some cuts and jumps are required. Fix to be included in v3.</li>
<li>Two current limiting resistors for the UUT power LED. Delete one of them. Fix to be included in v3.</li>
<li>Header PC1 is 26 pins. Need only 20 pins. Use a 20-pin symbol instead.</li>
<li>Add the ESP32 to the Tester PCB as a stuff option, so the Test System can be built with only one PCB. Leave the 20-pin header on the schematic so the
old configuration is still possible by installing the 20-pin header and not installing the ESP32</li>
<li>The UUT power switch is nice but mechanically weak. Figure out another option.</li>
<li>IC6 is no needed. Remove it.</li>
<li>IC3 pin 23 DPIO signal is connected directly to VPP. Change this to a small value pullup to VPP.</li>
<li>IC2 pin 1 DPIO signal is connected directly to GND. Change this to a small value pulldown to GND.</li>
</ol>
