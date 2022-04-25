<?php
define ("SERIAL_DEVICE_NOTSET", 0);
define ("SERIAL_DEVICE_SET", 1);
define ("SERIAL_DEVICE_OPENED", 2);

/**
 * Serial port control class
 *
 * THIS PROGRAM COMES WITH ABSOLUTELY NO WARRANTIES !
 * USE IT AT YOUR OWN RISKS !
 *
 * @author Rémy Sanchez <remy.sanchez@hyperthese.net>
 * @author Rizwan Kassim <rizwank@geekymedia.com>
 * @thanks Aurélien Derouineau for finding how to open serial ports with windows
 * @thanks Alec Avedisyan for help and testing with reading
 * @thanks Jim Wright for OSX cleanup/fixes.
 * @copyright under GPL 2 licence
 */
class PhpSerial
{
    public $_device = "/dev/ttyACM0";
    public $_winDevice = null;
    public $_dHandle = null;
    public $_dState = SERIAL_DEVICE_SET;
    public $_buffer = "";
    public $_os = "linux";

    public $autoFlush = true;

    public function deviceOpen($mode = "r+b")
    {
        if ($this->_dState == SERIAL_DEVICE_OPENED) {
            echo("The device is already opened<br>");

            return true;
        }

        if ($this->_dState == SERIAL_DEVICE_NOTSET) {
            echo("The device must be set before to be open<br>");

            return false;
        }

        if (!preg_match("@^[raw]\\+?b?$@", $mode)) {
            echo("Invalid opening mode : ".$mode.". Use fopen() modes.<br>");
				return false;
        }

        $this->_dHandle = @fopen($this->_device, $mode);

        if ($this->_dHandle !== false) {
            stream_set_blocking($this->_dHandle, 0);
            $this->_dState = SERIAL_DEVICE_OPENED;

            return true;
        }

        $this->_dHandle = null;
        echo("Unable to open the device" );

        return false;
    }

    public function deviceClose()
    {
        if ($this->_dState !== SERIAL_DEVICE_OPENED) {
            return true;
        }
        if (fclose($this->_dHandle)) {
            $this->_dHandle = null;
            $this->_dState = SERIAL_DEVICE_SET;
            return true;
        }
        echo("Unable to close the device<br>");
        return false;
    }

    public function confBaudRate($rate)
    {
        if ($this->_dState !== SERIAL_DEVICE_SET) {
            echo("Unable to set the baud rate : the device is either not set or opened<br>" );
            return false;
        }
        $validBauds = array (
            110    => 11,
            150    => 15,
            300    => 30,
            600    => 60,
            1200   => 12,
            2400   => 24,
            4800   => 48,
            9600   => 96,
            19200  => 19,
            38400  => 38400,
            57600  => 57600,
            115200 => 115200
        );
        if (isset($validBauds[$rate])) {
            if ($this->_os === "linux") {
                $ret = $this->_exec(
                    "stty -F " . $this->_device . " " . (int) $rate, $out);
            } else {
                return false;
            }
            if ($ret !== 0) {
                echo("Unable to set baud rate: " . $out[1] . "<br>" );
                return false;
            }
            return true;
        } else {
            return false;
        }
    }

    public function confParity($parity)
    {
        if ($this->_dState !== SERIAL_DEVICE_SET) {
            echo("Unable to set parity : the device is either not set or opened<br>" );
            return false;
        }
        $args = array(
            "none" => "-parenb",
            "odd"  => "parenb parodd",
            "even" => "parenb -parodd",
        );
        if (!isset($args[$parity])) {
            echo("Parity mode not supported" );

            return false;
        }
        if ($this->_os === "linux") {
            $ret = $this->_exec(
                "stty -F " . $this->_device . " " . $args[$parity],
                $out
            );
        }
        if ($ret === 0) {
            return true;
        }
        echo("Unable to set parity : " . $out[1] . "<br>");
        return false;
    }

    public function confCharacterLength($int)
    {
        if ($this->_dState !== SERIAL_DEVICE_SET) {
            echo("Unable to set length of a character : the device is either not set or opened<br>" );
            return false;
        }
        $int = (int) $int;
        if ($int < 5) {
            $int = 5;
        } elseif ($int > 8) {
            $int = 8;
        }
        if ($this->_os === "linux") {
            $ret = $this->_exec(
                "stty -F " . $this->_device . " cs" . $int,
                $out
            );
        }
        if ($ret === 0) {
            return true;
        }
        echo(
            "Unable to set character length : " .$out[1] . "<br>");
        return false;
    }

    public function confStopBits($length)
    {
        if ($this->_dState !== SERIAL_DEVICE_SET) {
            echo("Unable to set the length of a stop bit : the device is either not set or opened<br>" );
            return false;
        }
        if ($length != 1
                and $length != 2
                and $length != 1.5
                and !($length == 1.5 and $this->_os === "linux")
        ) {
            echo("Specified stop bit length is invalid");
            return false;
        }
        if ($this->_os === "linux") {
            $ret = $this->_exec(
                "stty -F " . $this->_device . " " .
                    (($length == 1) ? "-" : "") . "cstopb",
                $out
            );
        }
        if ($ret === 0) {
            return true;
        }
        echo("Unable to set stop bit length : " . $out[1] . "<br>");
        return false;
    }

    public function confFlowControl($mode)
    {
        if ($this->_dState !== SERIAL_DEVICE_SET) {
            echo("Unable to set flow control mode : the device is either not set or opened<br>" );
            return false;
        }
        $linuxModes = array(
            "none"     => "clocal -crtscts -ixon -ixoff",
            "rts/cts"  => "-clocal crtscts -ixon -ixoff",
            "xon/xoff" => "-clocal -crtscts ixon ixoff"
        );
        if ($mode !== "none" and $mode !== "rts/cts" and $mode !== "xon/xoff") {
            echo("Invalid flow control mode specified<br>");
            return false;
        }
        if ($this->_os === "linux") {
            $ret = $this->_exec(
                "stty -F " . $this->_device . " " . $linuxModes[$mode],
                $out
            );
        }
        if ($ret === 0) {
            return true;
        } else {
            echo(
                "Unable to set flow control : " . $out[1] . "<br>" );
            return false;
        }
    }

    public function setSetserialFlag($param, $arg = "")
    {
        if (!$this->_ckOpened()) {
            return false;
        }
        $return = exec("setserial " . $this->_device . " " . $param . " " . $arg . " 2>&1");
        if ($return{0} === "I") {
            echo("setserial: Invalid flag<br>" );
            return false;
        } elseif ($return{0} === "/") {
            echo("setserial: Error with device file<br>" );
            return false;
        } else {
            return true;
        }
    }

    public function sendMessage($str, $waitForReply = 0.1)
    {
        $this->_buffer .= $str;
        if ($this->autoFlush === true) {
            $this->serialflush();
        }
        usleep((int) ($waitForReply * 1000000));
    }

    public function readPort($count = 0)
    {
        if ($this->_dState !== SERIAL_DEVICE_OPENED) {
            echo("Device must be opened to read it<br>" );
            return false;
        }
        if ($this->_os === "linux") {
            $content = ""; $i = 0;
            if ($count !== 0) {
                do {
                    if ($i > $count) {
                        $content .= fread($this->_dHandle, ($count - $i));
                    } else {
                        $content .= fread($this->_dHandle, 128);
                    }
                } while (($i += 128) === strlen($content));
            } else {
                do {
                    $content .= fread($this->_dHandle, 128);
                } while (($i += 128) === strlen($content));
            }
            return $content;
        }
        return false;
    }

    public function serialflush()
    {
        if (!$this->_ckOpened()) {
            return false;
        }

        if (fwrite($this->_dHandle, $this->_buffer) !== false) {
            $this->_buffer = "";
            return true;
        } else {
            $this->_buffer = "";
            echo("Error while sending message<br>" );
            return false;
        }
    }

    public function _ckOpened()
    {
        if ($this->_dState !== SERIAL_DEVICE_OPENED) {
            echo("Device must be opened<br>" );
            return false;
        }
        return true;
    }

    public function _ckClosed()
    {
        if ($this->_dState === SERIAL_DEVICE_OPENED) {
            echo("Device must be closed<br>" );
            return false;
        }
        return true;
    }

    public function _exec($cmd, &$out = null)
    {
        $desc = array(
            1 => array("pipe", "w"),
            2 => array("pipe", "w")
        );

        $proc = proc_open($cmd, $desc, $pipes);
        $ret = stream_get_contents($pipes[1]);
        $err = stream_get_contents($pipes[2]);

        fclose($pipes[1]);
        fclose($pipes[2]);

        $retVal = proc_close($proc);

        if (func_num_args() == 2) $out = array($ret, $err);
        return $retVal;
    }

}

	function RunSerial() {
		echo (exec('whoami') . "<br>");
//		include 'PhpSerial.php';
		echo ("include<br>");
		// Let's start the class
		$serial = new PhpSerial();
		echo ("new class<br>");
		// First we must specify the device. This works on both linux and windows (if
		// your linux serial device is /dev/ttyS0 for COM1, etc)
		//$serial->deviceSet("/dev/ttyACM0");
		// We can change the baud rate, parity, length, stop bits, flow control
		$serial->confBaudRate(115200);
        echo ("baud<br>");
		$serial->confParity("none");
        echo ("parity<br>");
		$serial->confCharacterLength(8);
        echo ("length<br>");
		$serial->confStopBits(1);
        echo ("stopbits<br>");
		$serial->confFlowControl("none");
        echo ("flowcontrol<br>");
		// Then we need to open it
		$serial->deviceOpen();
        echo ("device open<br>");
		// To write into
		$serial->sendMessage('vel:845');
        echo ("sendMessage<br>");
		// Or to read from
		$read = $serial->readPort();
        echo("$read"."<br>");
        echo ("readPort<br>");
		// If you want to change the configuration, the device must be closed
		//$serial->deviceClose();
        echo ("deviceClose<br>");
	}
?>

<html>
	<body>
		<?php echo RunSerial(); ?>
	</body>
</html>
