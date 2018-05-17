# Lab 4: Visual Servoing
The handout is here: https://docs.google.com/document/d/1OM_tiuhjr1fm-PiuEH0yIozQ206bkbcC_6Yc_etQqQQ/export?format=pdf

## Configuration

See ```cfg/params.yaml```.

## Debugging using a webcam

I added a node that publishes images from the webcam to the ```visual_servoing/rbg_topic```, so I could debug without the actual racecar.

To run it, use:
```rosrun visual_servoing webcam_node```
