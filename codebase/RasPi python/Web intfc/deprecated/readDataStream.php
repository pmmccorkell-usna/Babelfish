<!DOCTYPE html>
<?php
	define('SERIAL_FILE', '/dev/shm/mjpeg/serial_data.txt');
    $serial = array();
	function readSerial() {
		//echo ("start function<br>");
		global $serial;
		if (file_exists(SERIAL_FILE)) {
			//echo ("file exists<br>");
			$lines = array();
			//echo ("lines = array<br>");
			$data = file_get_contents(SERIAL_FILE);
			//echo ("data get contents<br>");
			//echo ("$data");
			$lines = explode("\n", $data);
			foreach($lines as $line) {
				if (strlen($line) && substr($line, 0, 1) != '#') {
					$index = strpos($line, ' ');
					if ($index !== false) {
						$key = substr($line, 0, $index);
						$value = substr($line, $index +1);
						if ($value == 'true') $value = 1;
						if ($value == 'false') $value = 0;
						$serial[$key] = $value;
					} else {
						$serial[$line] = "";
					}
				}
			}
		}
		//echo ("<br><br>"."$serial[ser_h]"."<br>");
		return $serial;
	}

readSerial();
 
?>
