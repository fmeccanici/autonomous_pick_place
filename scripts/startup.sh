#!/bin/bash
sudo ntpdate -u 10.68.0.1

roslaunch autonomous_pick_place startup.launch
