

import numpy as np


class PyCostmap2D:

    def __init__(self, occupancy_map):
        self.size_x = occupancy_map.info.width
        self.size_y = occupancy_map.info.height
        self.resolution = occupancy_map.info.resolution
        self.origin_x = occupancy_map.info.origin.position.x
        self.origin_y = occupancy_map.info.origin.position.y
        self.global_frame_id = occupancy_map.header.frame_id
        self.costmap_timestamp = occupancy_map.header.stamp
        self.costmap = np.array(occupancy_map.data, dtype=np.uint8)

    def getSizeInCellsX(self):
        return self.size_x

    def getSizeInCellsY(self):
        return self.size_y

    def getSizeInMetersX(self):
        return (self.size_x - 1 + 0.5) * self.resolution

    def getSizeInMetersY(self):
        return (self.size_y - 1 + 0.5) * self.resolution

    def getOriginX(self):
        return self.origin_x

    def getOriginY(self):
        return self.origin_y

    def getResolution(self):
        return self.resolution

    def getGlobalFrameID(self):
        return self.global_frame_id

    def getCostmapTimestamp(self):
        return self.costmap_timestamp

    def getCostXY(self, mx: int, my: int) -> np.uint8:
        return self.costmap[self.getIndex(mx, my)]

    def getCostIdx(self, index: int) -> np.uint8:
        return self.costmap[index]

    def setCost(self, mx: int, my: int, cost: np.uint8) -> None:
        self.costmap[self.getIndex(mx, my)] = cost

    def mapToWorld(self, mx: int, my: int) -> tuple[float, float]:
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return (wx, wy)

    def worldToMap(self, wx: float, wy: float) -> tuple[int, int]:
        mx = int((wx - self.origin_x) // self.resolution)
        my = int((wy - self.origin_y) // self.resolution)
        return (mx, my)

    def getIndex(self, mx: int, my: int) -> int:
        return my * self.size_x + mx
