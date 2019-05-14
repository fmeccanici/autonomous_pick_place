#!/bin/bash
sudo ntpdate -u 10.68.0.1

roslaunch autonomous_pick_place startup.launch args1:="-0.04736501 1.17632553"
