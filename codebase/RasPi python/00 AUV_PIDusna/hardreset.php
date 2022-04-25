<?php

function runScript() {
	exec("pkill -f auv01min.py");
	exec("python3 /home/pi/hardreset.py");
}

runScript();

?>
