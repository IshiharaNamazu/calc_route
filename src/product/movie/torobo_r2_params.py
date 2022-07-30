import numpy as np

machine_size = np.array([[ 0.280215, 0.2965],
                         [-0.280215, 0.2965],
                         [-0.20988, -0.2965],
                         [ 0.20988, -0.2965]])

wheel_place_vec = np.array([[ 0.25, 0.25],
                            [-0.25, 0.25],
                            [-0.19, -0.25],
                            [ 0.19, -0.25]])


machine_margine = 0.01

fence = [[0.0-0.038, 0.0-0.038],
         [0.038-0.038, 0.0-0.038],
         [0.038-0.038, 4.576-0.038],
         [0.0-0.038, 4.576-0.038],
         [0.0-0.038, 0.0-0.038],
         [0.0-0.038, 0.038-0.038],
         [4.140-0.038, 0.038-0.038],
         [4.140-0.038, 0.0-0.038],
         [0.0-0.038, 0.0-0.038],
         [4.102-0.038, 0.0-0.038],
         [4.102-0.038, 4.576-0.038],
         [4.140-0.038, 4.576-0.038],
         [4.140-0.038, 0.0-0.038],
         [0.0-0.038, 0.0-0.038],
         [0.0-0.038, 4.538-0.038],
         [4.140-0.038, 4.538-0.038],
         [4.140-0.038, 4.576-0.038],
         [0.0-0.038, 4.576-0.038],
         [0.0-0.038, 0.0-0.038],
         #ここまでフェンス
         [0.0-0.038, 0.83-0.038],
         [0.838-0.038, 0.838-0.038],
         [0.838-0.038,0.0-0.038],
         [0.0-0.038, 0.0-0.038],
         #ここまでsz
         [0.0-0.038,4.576-0.038],
         [4.140-0.038, 4.576-0.038],
         [4.140-0.038,0.0-0.038],
         [0.0-0.038, 0.0-0.038],
         [0.0-0.038, 3.138-0.038],
         [0.538-0.038, 3.138-0.038],
         [0.538-0.038, 4.576-0.038],
         [0.0-0.038, 4.576-0.038],
         [0.0-0.038, 0.0-0.038],
         #ここらへんまでoz
         [1.319-0.038, 0.0-0.038],
         [1.319-0.038, 3.538-0.038],
         [1.357-0.038, 3.538-0.038],
         [1.357-0.038, 0.0-0.038],
         [0.0-0.038, 0.0-0.038],
         [0.0-0.038, 4.576-0.038],
         [2.257-0.038, 4.576-0.038],
         [2.219-0.038, 4.576-0.038],
         [2.219-0.038, 1.038-0.038],
         [2.257-0.038, 1.038-0.038],
         [2.257-0.038, 4.576-0.038],
         [0.0-0.038, 4.576-0.038],
         [0.0-0.038, 0.0-0.038],
         [0.869-0.038,0.0-0.038],
         [0.869-0.038,0.838-0.038],
         [1.319-0.038,0.838-0.038],
         [1.319-0.038,0.0-0.038],
         [0.0-0.038, 0.0-0.038],
         #関所とかフェンスとか
         [0.0-0.038, 4.576-0.038],
         [2.8975-0.038, 4.576-0.038],
         [2.8975-0.038, 4.038-0.038],
         [3.4595-0.038, 4.038-0.038],
         [3.4595-0.038, 4.576-0.038],
         [0.0-0.038, 4.576-0.038],
         [0.0-0.038, 0.0-0.038]
         ]

table_array = [[[3.1375-0.038, 1.178-0.038],
                [3.1375-0.038, 1.298-0.038],
                [3.2575-0.038, 1.298-0.038],
                [3.2575-0.038, 1.178-0.038],
                [3.1375-0.038, 1.178-0.038]],
               [[3.1375-0.038, 2.178-0.038],
                [3.1375-0.038, 2.298-0.038],
                [3.2575-0.038, 2.298-0.038],
                [3.2575-0.038, 2.178-0.038],
                [3.1375-0.038, 2.178-0.038]],
               [[3.1375-0.038, 3.178-0.038],
                [3.1375-0.038, 3.298-0.038],
                [3.2575-0.038, 3.298-0.038],
                [3.2575-0.038, 3.178-0.038],
                [3.1375-0.038, 3.178-0.038]]]