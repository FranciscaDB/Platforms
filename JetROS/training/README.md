# Usage Explanation

This only needs to be done on a robot and in tracks/environments that are different. Similar tracks may work with previous training.

The files must be on the robot that will be trained, and the code must be executed from the Jupyter Lab environment.

## Road Following 

In short, the idea is to execute a flow of code:

1. 1_data_collection.ipynb -> Ideally, have more than 50 images, where the center of the path is captured with a mouse click, especially in curves and non-straight sections.

2. 2_train_model.ipynb

3. 3_optimized_model.ipynb 

The final file .pth can be copy in the other robots.
