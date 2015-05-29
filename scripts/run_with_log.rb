	#! /usr/bin/env ruby
# -*- coding: utf-8 -*-
# If you want to start the Microsoft Life Cam or the Gumstix camera e-CAM32
# you should use the corresponding ruby run-script. 
if ARGV.empty?
    puts "ERROR: missing argument: You have to inform the log path"
    exit
end

require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

Orocos.run 'apriltags::Task' => 'marker_detector'  do
   
    address = ARGV[0].to_s    
 
    log = Orocos::Log::Replay.open(address)

    md = TaskContext.get_provides 'apriltags::Task'
    md.apply_conf_file('./ConfigFiles/apriltags.yml',['flatfish_left_front'])
    #md.apply_conf_file('./ConfigFiles/apriltags.yml',['default'])
    
  
    md.configure

    log.camera_front_left.frame.connect_to md.image
    #log.camera.frame_raw.connect_to md.image

    md.start
     
    Vizkit.control log
    #Vizkit.display md
    Vizkit.display md.output_image
    Vizkit.exec
end
