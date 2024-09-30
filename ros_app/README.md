# Example ROS application running on ubuntu Docker Container

## Running
### In the root of ros_app build the image:

```docker build -t ros_app . ```

### Then start the image:

```docker-compose up ```

### Then, to open a new terminal inside the container:

```docker exec -it ros_app_container bash```

### You should start out in the /home/catkin_ws directory, the packages in “ros_packages” should be in the /home/catkin_ws/src directory. NOTE: you can use the last command several times to open up more terminals in the container, but to use any ros functionality, you will need to run: 

```source /opt/ros/noetic/setup.bash```

### You can run the example script like this:

```$ catkin_make```

```$ source devel/setup.bash```

### For Single Node:

```$ chmod +x src/python_package_example/src/ros_python_app.py```

### For Multiple Nodes:

`chmod +x ~/catkin_ws/src/multiple_nodes_example/src/*.py `
`chmod +x ~/catkin_ws/src/multiple_nodes_example/launch/launch_nodes.launch`

### For both options:

```$ roscore&```

### For Single Node:
```$ rosrun python_package_example ros_python_app.py```

### For Multiple Nodes:
`roslaunch multiple_nodes_example launch_nodes.launch`

### And you should see a message like:

`[INFO] [1718687646.404314]: HELLO FROM MY PYTHON NODE`

## Creating a new package

### If you want to create a new package with this setup (this is kind of hacky but it works just the same), follow these steps. 

1.	On the host computer or VM, cd into ros_app/ros_packages and do a:

```$ mkdir <new_package_name>```

2.	Run the container with docker-compose, open up a terminal and source the ros files (just run the lines highlighted in black from the How to use section)

3.	```$ cd src/<new_package_name>```

4.	(create the package):

```$ catkin_create_pkg <new_package_name> rospy```

At this point your $ pwd should show:

`/home/catkin_ws/src/<new_package_name>`

And an $ ls should show a single directory:

`<new_package_name>`

5.	(the hacky part, follow these exactly):

```$ cp -r <new_package_name>/* ./```

```$ rm -r <new_package_name>```

Now the $ ls should show:
`CMakeLists.txt  package.xml  src`

### And now you have a new package.

## Protips:

If anytime in the container you see bash: roscore: command not found  (this one gets me a lot) you probably just need to run

```source /opt/ros/noetic/setup.bash```

If anytime you run into an `y: /usr/bin/python3^M: bad interpreter: No such file or directory` while trying to `ros run` check your `Shebang Line` or run:

```sed -i 's/\r$//' your_script.py```
