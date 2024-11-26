<launch>
  <arg name="port" default="/dev/ttyUSB0" />
 
  <arg name="mode" default="0" />

  <node pkg="robo_base" type="mode" name="mode" output="screen"> 
    <param name="port" value="$(arg port)" />
    <param name="mode" value="$(arg mode)" />
  </node>
</launch>
