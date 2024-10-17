# Best Practices

## Selecting the fitness threshold
The ICP (Iterative Closest Point) algorithm defines a threshold for two points to be neighbours. If a point in the reference object has no neighbour, it is considered to be an outlier. The proportion of the inlier points is represented as a *fitness* score. To ensure proper fit, the fitness score shall be high enough to be a good fit, but using to high threshold might result in missed fits. 

Since objects can be seen only from one side, as a rule of thumb, you should apply fitness threshold around $0.5$. E.g. seeing only the half of a cylinder will result in inlier only 50 percent even at the best case.

*Please note*, that the algorithm do not apply the threshold, and post any pose transformations that are calculated. It is the *responsibility* of the receiver to select and filter the proper fits based on the fitness score.