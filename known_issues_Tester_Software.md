<b>known issues Flip Chip Tester Software</b>
<p>
<ol>
<li>Edit Test Filename - cannot use the backspace key to delete characters while entering a filename. The characters on the screen are deleted
but the name that is actually entered doesn't match what you see on the screen. Fix is identified, to be included in v01.</li>
<li>Sometimes when running one test using the "O" command, errors are flagged when testing a good board. The scope loop test, "S", doesn't have
this issue. However, after testing multiple times using the "O" command when false errors are flagged, then when running the "S" command
afterward, the "S" command output will indicate test failures. So, running the "O" command seems to pollute the "S" command result. 
The root cause still needs to be found.</li>
<li>The "set test delay", main menu command "2", has not yet been verified on the ESP32 code.</li>
<li>The "set test trigger", main menu command "3", has not yet been verified on the ESP32 code.</li>
<li>In the sub-menu under "run test", the following have not yet been verified:
  <ul>
    <li>"F" - "run, stop on fail".</li>
    <li>"G" - "fo (run tests)".</li>
    <li>"N" - "run, stop on fail, no print".</li>
    <li>"C" - "turn on comment printout".</li>
    <li>"+" - "increase speed (less delay)".</li>
    <li>"-" - "decrease speed (more delay)(slower)".</li>
  </ul>
<li>The "output loading test", main menu command "58", has not yet been verified on the ESP32 code.</li>
<li>The "set test delay", main menu command "2", has not yet been verified on the ESP32 code.</li>
</ol>
