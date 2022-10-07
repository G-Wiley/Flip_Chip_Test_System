<b>known issues Flip Chip Tester Software</b>
<p>
<ol>
<li>Sometimes when running one test using the "O" command, errors are flagged when testing a good board. The scope loop test, "S", doesn't have
this issue. However, after testing multiple times using the "O" command when false errors are flagged, then when running the "S" command
afterward, the "S" command output will indicate test failures. So, running the "O" command seems to pollute the "S" command result. 
The root cause still needs to be found.</li>
<li>The "set test delay", main menu command "2", has not yet been verified on the ESP32 code.</li>
<li>The "output loading test", main menu command "5", has not yet been verified on the ESP32 code.</li>
<li>In the sub-menu under "run test", the following have not yet been verified:
  <ul>
    <li>"F" - "run, stop on fail".</li>
    <li>"G" - "go (run tests)".</li>
    <li>"N" - "run, stop on fail, no print".</li>
    <li>"C" - "turn on comment printout".</li>
    <li>"+" - "increase speed (less delay)".</li>
    <li>"-" - "decrease speed (more delay)(slower)".</li>
  </ul></li>
  <li>Writing to the log file has temporarily been commented out in the print() function.</li>
</ol>
<p>Regarding items that have not yet been verified: these functions probably worked properly in the original Stearns tester code. They need to be re-confirmed on the ESP32 code to verify that nothing has been broken when the code was ported.
