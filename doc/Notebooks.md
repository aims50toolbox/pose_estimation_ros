# Notebooks
The tool contains a couple of Jupyter notebooks presenting the main ideas behind the algorithms. Please check them to get better understanding in the problem.

To use Jupyter notebooks, you might use a [JupyterHub](https://jupyter.org/hub) installation, or you can use [Visual Code Studio](https://code.visualstudio.com/). For the latter, please follow the instructions at the Visual Studio Code [website](https://code.visualstudio.com/docs/datascience/jupyter-notebooks). 

For better experience, please use *virtual environments*. The [requirements.txt](../requirements.txt) file contains all the dependencies required for the notebooks to run. The notebooks were evaluated using a Python 3.8 interpreter, newer version might have some issues regarding the versions of the dependencies.

## ROS bag reader
To read ROS 2 bags, and collect messages for matching timestamps, you can use the [ROS Bag reader notebook](../notebooks/rosbag_reader.ipynb).

## Algorithm based on the Iterative Closest Point
The notebook [sam_pcm.ipynb](../notebooks/sam_pcm.ipynb) presents an algorithm based on the ICP (Iterative Closest Point) algorithm supported by object detection and segmentation. For object detection and segmentation, algorithms provided by [Ultralytics](https://github.com/ultralytics/ultralytics) are used. These AI based methods helps the ICP algorithm to be much more robust and convergent.

For better understanding, please check the notebook itself, which can be executed on an example RGBD image and a provided bottle CAD model. For deeper understanding of the ICP algorithm, you can check the [Wikipedia](https://en.wikipedia.org/wiki/Iterative_closest_point) page, or [other sources](https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf). The tool itself uses the [registration pipeline](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html) of the Open3D library to implement the ICP algorithm. *Please note, that while GPU support might speeds up the object detection and segmentation, for fast evaluation, CPU support might be sufficient.*