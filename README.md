# Robotic_Lego_Manipulation

This repo includes Lego manipulation using Fanuc and Yaskawa robots.


## Instructions
1. Modify the configuration file `path_to_repo/config/user_config.json`.
2. Download the controller at [ST Motion Controller](https://github.com/intelligent-control-lab/Stream_Motion_Controller) and the digital twin at [Robot Digital Twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin).
3. Follow the instructions to launch the controller and digital twin (if needed).
4. Launch the manipulation node:
```
roslaunch lego_manipulation lego_manipulation_node.launch
```

### Dual arm simulation
1. Modify the configuration file `path_to_repo/config/user_config.json`.
2. Download the controller at [ST Motion Controller](https://github.com/intelligent-control-lab/Stream_Motion_Controller) and the digital twin at [Robot Digital Twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin).
3. Switch all three repos to `dual-arm-mfi` branch.
4. 
```
roslaunch lego_manipulation dual_arm_lego_manipulation_node.launch num_b2:=25 color_b2:=Blue color_b6:=Yellow color_b9:=Orange
```

## Robot Manipulation Skills
| <img src="./images/transit.gif" alt="Transit image" width="auto" height="220" title="Transit"/><br>Transit a brick</center> | <img src="./images/pick.gif" alt="Pick image" width="auto" height="220" title="Pick"/><br>Pick/Disassemble</center> | <img src="./images/place.gif" alt="Place image" width="auto" height="220" title="Place"/><br>Place/Assemble</center> | <img src="./images/support.gif" alt="Support image" width="auto" height="220" title="Support"/> <br>Support</center>|
| -------------------------------- | -------------------------- | ---------------------------- | -------------------------------- |




## Robotic Lego Assembly in Motion
| <img src="./images/vday.gif" alt="vday image" width="auto" height="210" title="vday"/><br>Surprise on Valentine's Day</center> | <img src="./images/dual_arm.gif" alt="dualarm" width="auto" height="210" title="dualarm"/><br>Multi-Robot Collaboration</center> | <img src="./images/fanuc.gif" alt="fanuc" width="auto" height="210" title="fanuc"/><br>Build More!</center> |
| -------------------------------- | -------------------------- | ---------------------------- |



## Citation
If you find this repository helpful, please kindly cite our work.
```
@article{liu2023lightweight,
  title={A Lightweight and Transferable Design for Robust LEGO Manipulation},
  author={Liu, Ruixuan and Sun, Yifan and Liu, Changliu},
  journal={arXiv preprint arXiv:2309.02354},
  year={2023}
}
```

## License
This project is licensed under the MIT License.
