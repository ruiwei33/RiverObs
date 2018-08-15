"""
Extract all of the reaches overlapping a given bounding box and
computes the coordinates in the same projection used by the data.
The result is contained in a set of RiverReach objects, which can be clipped
to the same bounding box  as the data set.
"""

from __future__ import absolute_import, division, print_function

import numpy as np
import pysal
import rtree

from .RiverReach import RiverReach


class NodeExtractor:
    """Extract all of the nodes overlapping a given bounding box and
    computes the coordinates in the same projection used by the data.
    The result is contained in a set of RiverReach objects, which can be clipped
    to the same bounding box  as the data set.

    Initialize with the shape file database location and a lat_lon_region
    instance.

    Parameters
    ----------

    shape_file_root : str
        path to shapefile database (no suffix)

    lat_lon_region : object
        an object providing the following members:
        lat_lon_region.bounding_box (lonmin,latmin,lonmax,latmax)
        lat_lon_region.proj: a pyproj.Proj projection (lon,lat) -> (x,y)
        and (x,y) -> (lon,lat) when called when called with inverse=True


    Notes
    ------

    If clip is true, the reach is clipped to lie in a bounding box defined
    by the data bounding box plus a buffer given by clip_buffer
    (default is 0.1 deg or ~11 km).

    """

    def __init__(self,
                 shape_file_root,
                 lat_lon_region):

        # Open the shape and dbf files
        self.shp = pysal.open(shape_file_root + '.shp')
        self.dbf = pysal.open(shape_file_root + '.dbf')
        self.dbf_header = self.dbf.header
        
        bbox = lat_lon_region.bounding_box
        
        record = self.dbf
        max_width = None

        # find reach index field from dbf_header
        reach_id_index = self.dbf_header.index("Reach_ID")
        geoid_height_index = self.dbf_header.index("GeoidH")  #--
                    
        # Get the list of applicable reaches and extract them
        all_nodes = self.shp.read() 
        idx = rtree.index.Index()
        idx.insert(0, bbox)
        
        metadata = {}
        node_Reach_ID = []
        node_lat = []
        node_lon = []
        node_geoH = []  # --
        for ind, nodes in enumerate(all_nodes):
            if list(idx.intersection(list(nodes) + list(nodes))):
                node_lat.append(list(nodes)[1])
                node_lon.append(list(nodes)[0])
                node_Reach_ID.append(record[ind,reach_id_index])
                node_geoH.append(record[ind,geoid_height_index])  #--
        
        node_x, node_y = lat_lon_region.proj(node_lon, node_lat)
        
        reach_id_list = np.unique(node_Reach_ID)
        node_Reach_ID = np.reshape(node_Reach_ID,(len(node_lon)))
        node_lon = np.asarray(node_lon)
        node_lat = np.asarray(node_lat)
        node_x = np.asarray(node_x)
        node_y = np.asarray(node_y)
        node_geoH = np.asarray(node_geoH) # --
        
        self.reach = [] 
        
        for i in reach_id_list:
            index = node_Reach_ID ==i
            self.reach.append(
                RiverReach(
                    lon=node_lon[index],
                    lat=node_lat[index],
                    x=node_x[index],
                    y=node_y[index],
                    metadata=metadata,
                    reach_index=i,
                    geoid_height=node_geoH[index])) # --
            
            
        self.nreaches = len(self.reach)   
        self.reach_idx = reach_id_list
        
        self.reach_idx
        
    def __iter__(self):
        """This and the next function define an iterator over reaches."""
        return self

    def __next__(self):  ## Python 3: def __next__(self)
        """This and the previous function define an iterator over reaches."""
        if self.idx >= self.nreaches:
            self.idx = 0
            raise StopIteration

        self.idx += 1
        return self.reach[self.idx - 1]

    next = __next__

    def __len__(self):
        """Number of reaches."""
        return self.nreaches

    def __getitem__(self, index):
        """Get reaches or slices of reaches."""
        return self.reach[index]

