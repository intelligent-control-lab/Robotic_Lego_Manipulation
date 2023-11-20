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
