# ros_mpc_simple

## 1. How to execute the plant node.

```
roslaunch simple_system simple_system_bringup.launch
```

Topic info

```
/state 
message type: float64[4] s
s[:2] : x, y
s[2:4] : v_x, v_y

/input
message type: float64[2] u
u[0] : u_x
u[1] : u_y
```

## 2. MPC-ECBF node

Navigate to ros_mpc/nodes.

Make the MPC-ECBF node executable.

```
chmod +x ros_mpc.py
```

Run the node.

```
rosrun ros_mpc ros_mpc.py
```

Topic info
```
/ref
message_type: float64[2] ref

ref[0] = x_ref
ref[1] = y_ref
```

## 3. MPC-ECBF Result

<img src="/figures/real_time_plot.gif"/>