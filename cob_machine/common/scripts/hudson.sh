#!/bin/bash

rosdep install cob_extern cob_common cob_driver cob_simulation cob_apps -y
rosmake cob_extern cob_common cob_driver cob_simulation cob_apps --skip-blacklist
