<!DOCTYPE html>
<?php
	function readSerial() {
		global $serial;
		if (file_exists(SERIAL_FILE)) {
			$lines = array();
			$data = file_get_contents(SERIAL_FILE);
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
		//return $serial;
	}
 function DisplaySerial($selKey) {
	 $size=4;
	 global $serial;
	 $value=$serial[$selKey];
	 echo "<input type='{$selKey}' size=$size id='$selKey' value='$value' style='width:4em;'>";
 }
 
 readSerial();
 
?>

<html>
	<body>
		<?php echo RunSerial(); ?>
	</body>
</html>
