#! /usr/bin/env ruby
# -*- coding: utf-8 -*-
# If you want to start the Microsoft Life Cam or the Gumstix camera e-CAM32
# you should use the corresponding ruby run-script. 

require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

Orocos.run "camera_aravis::Task" => "Camera", 'apriltags::Task' => 'marker_detector' do
    
    #Orocos.log_all

    camera = TaskContext.get 'Camera'
    Orocos.apply_conf_file(camera, './ConfigFiles/camera_aravis.yml', ['default','aravis_basler'])
    
    md = TaskContext.get_provides 'apriltags::Task'
    md.apply_conf_file('./ConfigFiles/apriltags.yml',['default'])
    
  
    camera.configure
    camera.start
    md.configure
    md.start
    
    camera.frame.connect_to md.image
 
    Vizkit.display md
    Vizkit.display md.output_image
    Vizkit.exec
end
