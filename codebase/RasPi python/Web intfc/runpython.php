<?php

function runScript() {
	exec("pkill -f auv01min.py");
	exec("python3 /home/pi/auv01min.py");
}

runScript();

?>
