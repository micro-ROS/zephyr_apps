# Notes

This application requires `tf2_msgs` package, before building the app download and add this package to `mcu_ws` folder:

```bash
cd firmware/mcu_ws

git clone -b foxy https://github.com/ros2/geometry2
cp -R geometry2/tf2_msgs .

rm -rf geometry2
```