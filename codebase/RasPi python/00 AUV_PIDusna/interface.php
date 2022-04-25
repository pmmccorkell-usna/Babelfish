<!DOCTYPE html>
<?php
   define('BASE_DIR', dirname(__FILE__));
   require_once(BASE_DIR.'/config.php');

   $config = array();
   $debugString = "";
   $macros = array('error_soft','error_hard','start_img','end_img','start_vid','end_vid','end_box','do_cmd','motion_event','startstop');
   $options_mm = array('Average' => 'average', 'Spot' => 'spot', 'Backlit' => 'backlit', 'Matrix' => 'matrix');
   $options_em = array('Off' => 'off', 'Auto' => 'auto', 'Night' => 'night', 'Nightpreview' => 'nightpreview', 'Backlight' => 'backlight', 'Spotlight' => 'spotlight', 'Sports' => 'sports', 'Snow' => 'snow', 'Beach' => 'beach', 'Verylong' => 'verylong', 'Fixedfps' => 'fixedfps');
   $options_wb = array('Off' => 'off', 'Auto' => 'auto', 'Sun' => 'sun', 'Cloudy' => 'cloudy', 'Shade' => 'shade', 'Tungsten' => 'tungsten', 'Fluorescent' => 'fluorescent', 'Incandescent' => 'incandescent', 'Flash' => 'flash', 'Horizon' => 'horizon');
   $options_ie = array('None' => 'none', 'Negative' => 'negative', 'Solarise' => 'solarise', 'Sketch' => 'sketch', 'Denoise' => 'denoise', 'Emboss' => 'emboss', 'Oilpaint' => 'oilpaint', 'Hatch' => 'hatch', 'Gpen' => 'gpen', 'Pastel' => 'pastel', 'Watercolour' => 'watercolour', 'Film' => 'film', 'Blur' => 'blur', 'Saturation' => 'saturation', 'Colourswap' => 'colourswap', 'Washedout' => 'washedout', 'Posterise' => 'posterise', 'Cartoon' => 'cartoon');
   $options_ce_en = array('Disabled' => '0', 'Enabled' => '1');
   $options_ro = array('No rotate' => '0', 'Rotate_90' => '90', 'Rotate_180' => '180', 'Rotate_270' => '270');
   $options_fl = array('None' => '0', 'Horizontal' => '1', 'Vertical' => '2', 'Both' => '3');
   $options_bo = array('Off' => '0', 'Background' => '2');
   $options_av = array('V2' => '2', 'V3' => '3');
   $options_at_en = array('Disabled' => '0', 'Enabled' => '1');
   $options_ac_en = array('Disabled' => '0', 'Enabled' => '1');
   $options_ab = array('Off' => '0', 'On' => '1');
   $options_vs = array('Off' => '0', 'On' => '1');
   $options_rl = array('Off' => '0', 'On' => '1');
   $options_vp = array('Off' => '0', 'On' => '1');
   $options_mx = array('Internal' => '0', 'External' => '1', 'Monitor' => '2');
   $options_mf = array('Off' => '0', 'On' => '1');
   $options_cn = array('First' => '1', 'Second' => '2');
   $options_st = array('Off' => '0', 'On' => '1');

 function DisplaySerial($selKey) {
     $size=5;
     global $serial;
     $value=$serial[$selKey];
     echo "<input type='{$selKey}' size=$size id='$selKey' value='$value' style='width:5em;'>";
 }

   function initCamPos() {
      $tr = fopen("pipan_bak.txt", "r");
      if($tr){
         while(($line = fgets($tr)) != false) {
           $vals = explode(" ", $line);
           echo '<script type="text/javascript">init_pt(',$vals[0],',',$vals[1],');</script>';
         }
         fclose($tr);
      }
   }

   function user_buttons() {
      $buttonString = "";
      $buttonCount = 0;
      if (file_exists("userbuttons")) {
        $lines = array();
        $data = file_get_contents("userbuttons");
        $lines = explode("\n", $data);
        foreach($lines as $line) {
            if (strlen($line) && (substr($line, 0, 1) != '#') && buttonCount < 6) {
                $index = explode(",",$line);
                if ($index !== false) {
                    $buttonName = $index[0];
                    $macroName = $index[1];
                    $className = $index[2];
                    if ($className == false) {
                        $className = "btn btn-primary";
                    }
                    $otherAtt  = $index[3];
                    $buttonString .= '<input id="' . $buttonName . '" type="button" value="' . $buttonName . '" onclick="send_cmd(' . "'sy " . $macroName . "'" . ')" class="' . $className . '" ' . $otherAtt . '>' . "\r\n";
                    $buttonCount += 1;
                }
            }
        }
      }
      if (strlen($buttonString)) {
          echo '<div class="container-fluid text-center">' . $buttonString . "</div>\r\n";
      }
   }

   function pan_controls() {
      $mode = 0;
      if (file_exists("pipan_on")){
         initCamPos();
         $mode = 1;
      } else if (file_exists("servo_on")){
         $mode = 2;
      }
      if ($mode <> 0) {
         echo '<script type="text/javascript">set_panmode(',$mode,');</script>';
         echo "<div class='container-fluid text-center liveimage'>";
         echo "<div alt='Up' id='arrowUp' style='margin-bottom: 2px;width: 0;height: 0;border-left: 20px solid transparent;border-right: 20px solid transparent;border-bottom: 40px solid #428bca;font-size: 0;line-height: 0;vertical-align: middle;margin-left: auto; margin-right: auto;' onclick='servo_up();'></div>";
         echo "<div>";
         echo "<div alt='Left' id='arrowLeft' style='margin-right: 22px;display: inline-block;height: 0;border-top: 20px solid transparent;border-bottom: 20px solid transparent;border-right: 40px solid #428bca;font-size: 0;line-height: 0;vertical-align: middle;' onclick='servo_left();'></div>";
         echo "<div alt='Right' id='arrowRight' style='margin-left: 22px;display: inline-block;height: 0;border-top: 20px solid transparent;border-bottom: 20px solid transparent;border-left: 40px solid #428bca;font-size: 0;line-height: 0;vertical-align: middle;' onclick='servo_right();'></div>";
         echo "</div>";
         echo "<div alt='Down' id='arrowDown' style='margin-top: 2px;width: 0;height: 0;border-left: 20px solid transparent;border-right: 20px solid transparent;border-top: 40px solid #428bca;font-size: 0;line-height: 0;vertical-align: middle;margin-left: auto; margin-right: auto;' onclick='servo_down();'></div>";
         echo "</div>";
      }
   }
  
   function pilight_controls() {
      echo "<tr>";
        echo "<td>Pi-Light:</td>";
        echo "<td>";
          echo "R: <input type='text' size=4 id='pilight_r' value='255'>";
          echo "G: <input type='text' size=4 id='pilight_g' value='255'>";
          echo "B: <input type='text' size=4 id='pilight_b' value='255'><br>";
          echo "<input type='button' value='ON/OFF' onclick='led_switch();'>";
        echo "</td>";
      echo "</tr>";
   }

   function getExtraStyles() {
      $files = scandir('css');
      foreach($files as $file) {
         if(substr($file,0,3) == 'es_') {
            echo "<option value='$file'>" . substr($file,3, -4) . '</option>';
         }
      }
   }

   function makeOptions($options, $selKey) {
      global $config;
      switch ($selKey) {
         case 'flip': 
            $cvalue = (($config['vflip'] == 'true') || ($config['vflip'] == 1) ? 2:0);
            $cvalue += (($config['hflip'] == 'true') || ($config['hflip'] == 1) ? 1:0);
            break;
         case 'MP4Box': 
            $cvalue = $config[$selKey];
            if ($cvalue == 'background') $cvalue = 2;
            break;
         default: $cvalue = $config[$selKey]; break;
      }
      if ($cvalue == 'false') $cvalue = 0;
      else if ($cvalue == 'true') $cvalue = 1;
      foreach($options as $name => $value) {
         if ($cvalue != $value) {
            $selected = '';
         } else {
            $selected = ' selected';
         }
         echo "<option value='$value'$selected>$name</option>";
      }
   }

   function makeInput($id, $size, $selKey='', $type='text') {
      global $config, $debugString;
      if ($selKey == '') $selKey = $id;
      switch ($selKey) {
         case 'tl_interval': 
            if (array_key_exists($selKey, $config)) {
               $value = $config[$selKey] / 10;
            } else {
               $value = 3;
            }
            break;
         case 'watchdog_interval':
            if (array_key_exists($selKey, $config)) {
               $value = $config[$selKey] / 10;
            } else {
               $value = 0;
            }
            break;
         default: $value = $config[$selKey]; break;
      }
      echo "<input type='{$type}' size=$size id='$id' value='$value' style='width:{$size}em;'>";
   }

   function macroUpdates() {
      global $config, $debugString, $macros;
      $m = 0;
      $mTable = '';
      foreach($macros as $macro) {
          $value = $config[$macro];
          if(substr($value,0,1) == '-') {
              $checked = '';
              $value = substr($value,1);
          } else {
              $checked = 'checked';
          }
          $mTable .= "<TR><TD>Macro:$macro</TD><TD><input type='text' size=16 id='$macro' value='$value'>\r\n";
          $mTable .= "<input type='checkbox' $checked id='$macro" . "_chk'>\r\n";
          $mTable .= "<input type='button' value='OK' onclick=" . '"send_macroUpdate' . "($m,'$macro')\r\n" . ';"></TD></TR>';
          $m++;
      }
      echo $mTable;
   }

   function getImgWidth() {
      global $config;
      if($config['vector_preview'])
         return 'style="width:' . $config['width'] . 'px;"';
      else
         return '';
   }
   
   function getLoadClass() {
      global $config;
      if(array_key_exists('fullscreen', $config) && $config['fullscreen'] == 1)
         return 'class="fullscreen" ';
      else
         return '';
   }

   function simple_button() {
       global $toggleButton, $userLevel;
       if ($toggleButton != "Off" && $userLevel > USERLEVEL_MIN) {
          echo '<input id="toggle_display" type="button" class="btn btn-primary" value="' . $toggleButton . '" style="position:absolute;top:60px;right:10px;" onclick="set_display(this.value);">';
       }
   }

   if (isset($_POST['extrastyle'])) {
      if (file_exists('css/' . $_POST['extrastyle'])) {
         $fp = fopen(BASE_DIR . '/css/extrastyle.txt', "w");
         fwrite($fp, $_POST['extrastyle']);
         fclose($fp);
      }
   }

   function getDisplayStyle($context, $userLevel) {
        global $Simple;
        if ($Simple == 1) {
            echo 'style="display:none;"';
        } else {
            switch($context) {
                case 'navbar':
                    if ((int)$userLevel < (int)USERLEVEL_MEDIUM)
                        echo 'style="display:none;"';
                    break;
                case 'preview':
                    if ((int)$userLevel < (int)USERLEVEL_MINP)
                        echo 'style="display:none;"';
                    break;
                case 'actions':
                    if ((int)$userLevel < (int)USERLEVEL_MEDIUM)
                        echo 'style="display:none;"';
                    break;
                case 'settings':
                    if ((int)$userLevel != (int)USERLEVEL_MAX)
                        echo 'style="display:none;"';
                    break;
            }
        }
   }

   $toggleButton = "Off";
   $Simple = 0;
   $allowSimple = "SimpleOn";
   if(isset($_COOKIE["display_mode"])) {
      if($_COOKIE["display_mode"] == "Full") {
         $allowSimple = "SimpleOff";
         $toggleButton = "Simple";
         $Simple = 2;
      } else if($_COOKIE["display_mode"] == "Simple") {
         $allowSimple = "SimpleOff";
         $toggleButton = "Full";
         $Simple = 1;
      } else {
         $allowSimple = "SimpleOn";
         $toggleButton = "Off";
         $Simple = 0;
      }
   }

   $streamButton = "MJPEG-Stream";
   $mjpegmode = 0;
   if(isset($_COOKIE["stream_mode"])) {
      if($_COOKIE["stream_mode"] == "MJPEG-Stream") {
         $streamButton = "Default-Stream";
         $mjpegmode = 1;
      }
   }
   $config = readConfig($config, CONFIG_FILE1);
   $config = readConfig($config, CONFIG_FILE2);
   $video_fps = $config['video_fps'];
   $divider = $config['divider'];
   $user = getUser();
   writeLog("Logged in user:" . $user . ":");
   $userLevel =  getUserLevel($user);
   writeLog("UserLevel " . $userLevel);
   
  ?>

<html>
   <head>
      <meta name="viewport" content="width=550, initial-scale=1">
      <title>TSD Robotics Web Interface</title>
      <link rel="stylesheet" href="css/style_minified.css" />
      <link rel="stylesheet" href="<?php echo getStyle(); ?>" />
      <script src="js/style_minified.js"></script>
      <script src="js/script2.js"></script>
      <script src="js/pipan.js"></script>
   </head>
  <body onload="setTimeout('init(0,25,1);', 100);">
     <div>
         <input type="button" value="EMERGENCY STOP" onclick="send_cmd2('STOP');">
         <input id="leak_button" type="button" class="btn btn-primary">
         &nbsp&nbsp&nbsp
         &nbsp&nbsp&nbsp
         &nbsp&nbsp&nbsp
         Status:&nbsp&nbsp&nbsp
         <?php echo DisplaySerial(ser_status); ?>
      </div>
      <div>
         <input type="button" value="Reset Mbed Microcontroller" onclick="stop_python(); reset_mbed();">
         <input type="button" value="Start Data Stream" onclick="start_python();">
         <input type="button" value="Stop Data Stream" onclick="stop_python();">
     </div>
     <div>
      <table align="center">
         <tr>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Port:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_port); ?></td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Heading:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_h); ?></td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Roll:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_r); ?></td>
         </tr>
         <tr>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Stbd:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_stbd); ?></td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>H gain:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(h_gain); ?></td>
         </tr>
         <tr><td>&nbsp&nbsp&nbsp</td></tr>
         <tr>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Fore:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_fwd); ?></td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Depth:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_d); ?></td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Pitch:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_p); ?></td>
         </tr>
         <tr>
            <td>&nbsp&nbsp&nbsp</td>
            <td>Aft:</td>
            <td><?php echo DisplaySerial(ser_aft); ?></td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>D gain:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(d_gain); ?></td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>&nbsp&nbsp&nbsp</td>
            <td>P gain:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(p_gain); ?></td>
         </tr>
         <tr>
            <td>&nbsp&nbsp&nbsp</td>
            <td>BNO Cal:&nbsp&nbsp&nbsp</td>
            <td><?php echo DisplaySerial(ser_cal); ?></td>
           </tr>
         <tr><td>&nbsp&nbsp&nbsp</td></tr>
      </table>
     </div>
      <center>
         <!--<input type="button" value="New Window" onclick="window.open('interface.php','_blank');">-->
         <input type="button" value="Zeroize Depth" onclick="send_cmd2('zer:000');">     
         <input type="button" value="Stop All Thrusters" onclick="stop_all();">
      </center>
      <div class="container-fluid text-center" align="left">
         <div class="panel-group" id="accordion"  >
            <div class="panel panel-default">
               <div class="panel-heading">
                  <h2 class="panel-title">
                     <a data-toggle="collapse" data-parent="#accordion" href="#collapseOne">Controllers</a>
                  </h2>
               </div>
               <div id="collapseOne" class="panel-collapse collapse">
                  <div class="panel-body">
                     <table class="settingsTable">
                        <tr>
                           <td>
                              <input type="button" value="v" onclick="send_pitch(811);">
                              <input type="button" value="^" onclick="send_pitch(817);">
                              Pitch [-180,180]deg:<br>
                              <input type="button" value="v" onclick="send_depth(821);">
                              <input type="button" value="^" onclick="send_depth(827);">
                              Depth [0,820]cm:<br>
                              <input type="button" value="<" onclick="send_heading(831);">
                              <input type="button" value=">" onclick="send_heading(837);">
                              Heading [0,360]deg:<br>
                              <input type="button" value="v" onclick="send_speed(841);">
                              <input type="button" value="^" onclick="send_speed(847);">
                              Speed [-400,400]ms:  <br>
                           </td>
                           <td>
                               <?php makeInput('set_pitch', 3); ?><br>
                               <?php makeInput('set_depth', 3); ?><br>
                               <?php makeInput('set_heading', 3); ?><br>
                               <?php makeInput('set_speed', 3); ?><br>
                           </td>
                           <td>
                              <input type="button" value="Start Pitch" onclick="get_pitch();"><br>
                              <input type="button" value="Start Depth" onclick="get_depth();"><br>
                              <input type="button" value="Start Heading" onclick="get_heading();"><br>
                              <input type="button" value="Start Speed" onclick="get_speed();"><br>
                           </td>
                           <td>
                              <input type="button" value="Stop Pitch" onclick="send_pitch(999);"><br>
                              <input type="button" value="Stop Depth" onclick="send_depth(999);"><br>
                              <input type="button" value="Stop Heading" onclick="send_heading(999);"><br>
                              <input type="button" value="Stop Speed" onclick="send_speed(999);"><br>
                           </td>
                        </tr>
                     </table>
                     <input type="button" value="DO MY BIDDING" onclick="get_values();"><br>
                  </div>
               </div>
             </div>
            <div class="panel panel-default">
               <div class="panel-heading">
                  <h2 class="panel-title">
                     <a data-toggle="collapse" data-parent="#accordion" href="#collapseTwo">Camera Settings</a>
                  </h2>
               </div>
               <div id="collapseTwo" class="panel-collapse collapse">
                  <div class="panel-body">
                    <div class="container-fluid text-center liveimage">
                        <div id="main-buttons">
                            <input id="video_button" type="button" class="btn btn-primary" <?php getdisplayStyle('actions', $userLevel); ?>>
                            <input id="image_button" type="button" class="btn btn-primary" <?php getdisplayStyle('actions', $userLevel); ?>>
                            <input id="halt_button" type="button" class="btn btn-danger" <?php getdisplayStyle('settings', $userLevel); ?>>
                        </div>
                    </div>
                    <div id="secondary-buttons" class="container-fluid text-center">
                        <?php pan_controls(); ?>
                        <?php user_buttons(); ?>
                        <a href="preview.php" target="_blank" class="btn btn-default" <?php getdisplayStyle('preview', $userLevel); ?>>Download Videos and Images</a>
                        &nbsp;&nbsp;
                        <a href="schedule.php" target="_blank" class="btn btn-default" <?php getdisplayStyle('settings', $userLevel); ?>>Edit schedule settings</a>
                    </div>
                  </div>
               </div>
            </div>
         </div>
      </div>
      <center>
         <div><img id="mjpeg_dest" onclick="toggle_fullscreen(this);" /></div>
      </center>
   </body>
</html>

