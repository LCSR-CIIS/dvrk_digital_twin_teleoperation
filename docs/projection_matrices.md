
## da Vinci projection matrices
Projection matrix needs to be set from the projection matrix in the camera info rostopic. Results can be slighty different if using raw intrinsics. See below.

From raw intrinsics
```yaml
projection matrix: [[ 2.51391,  0.     ,  0.     ,  0.     ],
                    [0.     ,  3.20167,  0.     ,  0.     ],
                    [0.     ,  0.     , -1.0002 , -0.002  ],
                    [0.     ,  0.     , -1.     ,  0.     ]]
```

From projection matrix
```yaml
projection matrix: [[ 2.8694, 0.0000, -0.0385, 0.0000 ],
                    [ 0.0000, 3.6428, 0.0267, 0.0000 ],
                    [ 0.0000, 0.0000, -1.0002, -0.0020 ],
                    [ 0.0000, 0.0000, -1.0000, 0.0000 ] ]
```