"""
Given a SWOTL2 file, fit all of the reaches observed and output results.
"""

from __future__ import absolute_import, division, print_function

import os
import scipy.ndimage
import numpy as np
import pandas as pd
import netCDF4 as nc
import collections
import scipy.stats
import statsmodels.api

from .SWOTL2 import SWOTL2
from RiverObs import ReachExtractor
from RiverObs import NodeExtractor
from RiverObs import WidthDataBase
from RiverObs import IteratedRiverObs
from RiverObs import FitRiver
from RiverObs import RiverNode
from RiverObs import RiverReach
from Centerline.Centerline import CenterLineException

class SWOTRiverEstimator(SWOTL2):
    """
    Given a SWOTL2 file, fit all of the reaches observed and output results.

    This class is derived from the SWOTL2 class.

    This class contains four basic components:
    1. It is a subclass of the SWOTL2, and therefor is a LatLonRegion and
    also contains SWOT data.
    2. For each reach of interest, it keeps a dictionary RiverObs instances
    for each reach.
    3. It keeps a dictionary of RiverReach instances, containing the results
    of height/slope fitting and width estimation.
    4. A dictionary of FitRiver instances, containing the full fit diagnostics.

    Keeping copies for each observation, although useful for visualization,
    can be a tax on memory for a large number of reaches and observation,
    so that storing can be turned off, if so desired.

    Parameters
    ----------

    height_kwd : str
        name of netcdf variable to use as a measurement.
    true_height_kwd: str
        name of variable to use as truth for comparison.
    no_noise_height_kwd : str
        name for no noise measurement
    xtrack_kwd : str
        name of cross-track distance variable.
    bounding_box : 4-tuple or array_like
        If the bounding_box is provided, select only the data in the
        bounding box, otherwise, get the bounding box from the data
        itself. bounding_box should be of the form
        (lonmin,latmin,lonmax,latmax)
    lon_kwd, lat_kwd : str
        netcdf names for the longitudes and latitudes to be used
        for georeferencing the data set. The a priori geolocation can be
        retrieved using 'no_layover_longitude' and 'no_layover_latitude'.
        The estimated latitude and longitude can be retrieved using
        'longitude' and 'latitude', respectively.
    class_kwd : str, default 'classification'
        the netcdf name of the classification layer to use.
        'classification' will return the result of applying a classifier
        to the simulated data. 'no_layover_classification' can be used as the
        thruth
    class_list : list, default [2,3,4,5]
        a list of the class labels for what is considered good water data.
        This should be piched as any classes that might contain water, even
        if partially. If truth data are desired (class_kwd =
        'no_layover_classification'), the this should be set to [1].
        The default has interior water pixels(4), border water pixels (3),
        and border land pixels (2). This should be used with
        inundation_fraction turned on.
    fractional_inundation_kwd : str, default 'continuous_classification'
        Netcdf keyword containing the inundation fraction. If None, the no
        inundation fraction is used. The inundation fraction is, in theory,
        a number between 0 and 1 estimating the fraction of the pixel covered
        with water. In practice, because of noise, it can be outside the
        bounds and even be negative! It will produced an ensemble
        unbiased estimate if the class mean cross sections are known.
    use_fractional_inundation : bool list, default [True, True, False, False]
        For which classes should the inundation fraction be used instead of a
        binary classification? the default is to assume that interior pixels
        are 100% water, but to use both land and water edge pixels partially
        by using the fractional inundation kwd.
    use_segmentation :  bool list, default [False, True, True, True]
        Selects which classes to assume as water for segmentation purposes
    use_heights : bool list, default [False, False, True, False]
        Selects which classes to use for estimating heights
    min_points : int
        If the number of good points is less than this, raise an exception.
    height_kwd : str, default 'height'
        These are the heights to exract from the water file.
    true_height_kwd : str, default 'water_height'
        These are the reference heights to use for performance assessment
    no_noise_height_kwd : str, default 'no_noise_height'
        These are the a priori heights used for phase unwrapping
    xtrack_kwd : str, 'no_layover_cross_track'
        This is the distance to the nadir track.
    xtrack_res_kwd : str, default 'flat_pixel_resolution'
        This is the cross-track dimension of the radar pixel. The netcdf file
        has an estimate 'no_layover_ground_resolution' that takes into account
        a priori surface slopes to derive an incidence angle. However, this
        runs into trouble if water
    trim_ends : bool, default False
        If True, the first and last nodes are not used, to avoid potential
        edge effects.
    store_obs : bool, default True
        If True, store each RiverObs instance in a dictionary.
    store_reaches : bool, default True
        If True, store each RiverRiver instance in a dictionary.
    store_fits : bool, default True
        If True, store each fit result in a dictionary.
    verbose : bool, default False
        If True, print to sdtout while processing.
    use_segmentation : bool list, default [False, True, True, True]
        Defines which classes should the assumed as water for segmatation
        algorithm to label disjoint features
    The final set of keywords are projection options for pyproj. See Notes.

    Notes
    -----

    A full list of projection options to set plus explanations of their
    meaning can be found here: https://trac.osgeo.org/proj/wiki/GenParms

    The default projection is Lambert equiarea, which has a proj4 string with
    the following parameters:

    +proj=laea
    +lat_0=Latitude at projection center, set to bounding box center lat
    +lon_0=Longitude at projection center, set to bounding box center lon
    +x_0=False Easting, set to 0
    +y_0=False Northing, set to 0
    """

    xtrack_res_max = 200.  # Maximum allowed cross-track range resolution
    platform_height = 970.e3
    earth_radius = 6378e3

    def __init__(self,
                 swotL2_file,
                 bounding_box=None,
                 lat_kwd='no_layover_latitude',
                 lon_kwd='no_layover_longitude',
                 class_list=[2, 3, 4, 5],
                 class_kwd='classification',
                 fractional_inundation_kwd='continuous_classification',
                 use_fractional_inundation=[True, True, False, False],
                 use_segmentation=[False, True, True, True],
                 use_heights=[False, False, True, False],
                 min_points=100,
                 height_kwd='height',
                 trim_ends=False,
                 store_obs=True,
                 store_reaches=True,
                 store_fits=True,
                 verbose=True,
                 xtrack_kwd='no_layover_cross_track',
                 proj='laea',
                 x_0=0,
                 y_0=0,
                 lat_0=None,
                 lon_0=None,
                 ellps='WGS84',
                 output_file=None,
                 subsample_factor=1,
                 min_reach_obsrvd_ratio=0.5,
                 **proj_kwds):

        self.trim_ends = trim_ends
        self.store_obs = store_obs
        self.store_reaches = store_reaches
        self.store_fits = store_fits
        self.verbose = verbose
        self.input_file = os.path.split(swotL2_file)[-1]
        self.output_file = output_file  # index file
        self.subsample_factor = subsample_factor
        self.min_reach_obsrvd_ratio = min_reach_obsrvd_ratio

        # Classification inputs
        self.class_kwd = class_kwd
        self.class_list = class_list
        self.fractional_inundation_kwd = fractional_inundation_kwd
        self.use_fractional_inundation = use_fractional_inundation

        self.use_segmentation = use_segmentation
        self.use_heights = use_heights

        # Initialize the base class
        SWOTL2.__init__(
            self,
            swotL2_file,
            bounding_box=bounding_box,
            class_list=class_list,
            lat_kwd=lat_kwd,
            lon_kwd=lon_kwd,
            class_kwd=class_kwd,
            min_points=min_points,
            verbose=verbose,
            proj=proj,
            x_0=x_0,
            y_0=y_0,
            lat_0=lat_0,
            lon_0=lon_0,
            ellps=ellps,
            subsample_factor=subsample_factor,
            **proj_kwds)

        self.create_index_file()

        if np.ma.is_masked(self.lat):
            mask = self.lat.mask
        else:
            mask = np.zeros(len(self.lat), dtype=np.bool)

        self.h_noise = self.get(height_kwd)
        if np.ma.is_masked(self.h_noise):
            mask = mask | self.h_noise.mask

        self.xtrack = (self.get(xtrack_kwd)
                       if xtrack_kwd in self.nc.variables.keys() else None)
        
        self.wet_tropo_error = (self.get('wet_tropo_error')
                       if 'wet_tropo_error' in self.nc.variables.keys() else None) 
        
        self.inst_error = (self.get('height_instrument_error')
                       if 'height_instrument_error' in self.nc.variables.keys() else None) 

        good = ~mask
        self.lat = self.lat[good]
        self.lon = self.lon[good]
        self.x = self.x[good]
        self.y = self.y[good]
        self.klass = self.klass[good]
        if self.wet_tropo_error is not None:
            print('wet_tropo and inst error added')
            self.h_noise = self.h_noise[good] + self.wet_tropo_error[good] + self.inst_error[good]  
        else: self.h_noise = self.h_noise[good]   
        if self.xtrack is not None:
            self.xtrack = self.xtrack[good]
        self.img_x = self.img_x[good]  # range or x index
        self.img_y = self.img_y[good]  # azimuth or y index

        # set the pixcvec geolocations to the pixel cloud values
        # TODO: update these with height constrained geolocation
        self.lat_vec = self.lat
        self.lon_vec = self.lon
        self.height_vec = self.h_noise

        # Try to read the pixel area from the L2 file, or compute it
        # from look angle and azimuth spacing, or from azimuth spacing
        # and ground spacing
        if 'pixel_area' in self.nc.variables:
            self.pixel_area = self.get('pixel_area')

        else:
            if 'no_layover_look_angle' in self.nc.variables:
                look_angle = self.get('no_layover_look_angle')[good]
                incidence_angle = (look_angle) * (
                    1. + self.platform_height / self.earth_radius)
                self.xtrack_res = (float(self.nc.range_resolution) / np.sin(
                    np.radians(incidence_angle)))
                self.pixel_area = float(
                    self.nc.azimuth_spacing) * self.xtrack_res

            else:
                if 'ground_spacing' in vars(self.nc):
                    self.pixel_area = (float(self.nc.azimuth_spacing) * float(
                        self.nc.ground_spacing) * np.ones(
                            np.shape(self.h_noise)))

                else:
                    self.pixel_area = 10.0 * np.zeros(len(self.h_noise))
                    print("could not find correct pixel area parameters")

        # need to scale pixel area by the subsampling factor if subsampling
        if (self.subsample_factor > 1):
            self.pixel_area = self.pixel_area * self.subsample_factor
        #
        if fractional_inundation_kwd is None:  # all water pixels are inundated
            self.fractional_inundation = None
            self.inundated_area = self.pixel_area

        else:
            self.fractional_inundation = self.get(fractional_inundation_kwd)
            self.inundated_area = self.pixel_area
            for i, k in enumerate(class_list):
                if use_fractional_inundation[i]:
                    index = self.klass == k
                    self.inundated_area[index] = (self.pixel_area[
                        index] * self.fractional_inundation[index])

        # Segment the image into disjoint features
        if len(self.use_segmentation) == len(self.class_list):
            self.isWater = np.zeros(np.shape(self.h_noise))
            for i, k in enumerate(class_list):
                if self.use_segmentation[i]:
                    index = self.klass == k
                    self.isWater[index] = 1
            self.segment_water_class()
        else:
            self.seg_label = None

        # set which class pixels to use for heights (should be modified
        # by layover flag later)
        if len(self.use_heights) == len(self.class_list):
            self.h_flg = np.zeros(np.shape(self.h_noise))
            for i, k in enumerate(class_list):
                if self.use_heights[i]:
                    index = self.klass == k
                    self.h_flg[index] = 1
        else:
            self.h_flg = None

        if self.verbose:
            print('Data loaded')

        # Initialize the list of observations and reaches
        self.river_obs_collection = collections.OrderedDict()
        self.river_reach_collection = collections.OrderedDict()
        self.fit_collection = collections.OrderedDict()

    def segment_water_class(self):
        """
        do image segmentation algorithm on the water class to label
        unconnected features
        """
        maxX = np.max(self.img_x)
        maxY = np.max(self.img_y)
        cls_img = np.zeros((maxY + 1, maxX + 1))
        cls_img[self.img_y, self.img_x] = self.isWater

        # Do some regularization with morphological operations? (TODO)
        # segment the water class image
        lbl, nlbl = scipy.ndimage.label(cls_img)

        # assign land edge segments (label 0) to nearest water feature
        # (label >0) using grey dilation for this, probably need to re-think
        # how to handle land edge pixels that touch two different
        # segments (this will arbitrarily assign it to the one with
        # the largest label index)
        lbl2 = scipy.ndimage.grey_dilation(lbl, 3)
        lbl_out = lbl.copy()
        lbl_out[lbl == 0] = lbl2[lbl == 0]

        # write out segmentation images to a nc file for testing
        dumpLbl = False
        if dumpLbl:
            M0, M1 = np.shape(lbl)
            with nc.Dataset("segDump.nc", 'w') as f:
                f.createDimension('azimuth', M0)
                f.createDimension('range', M1)
                ol = f.createVariable(
                    'orig_label', 'i4', ('azimuth', 'range'), fill_value=-128)
                fl = f.createVariable(
                    'final_label', 'i4', ('azimuth', 'range'), fill_value=-128)
                ol[:] = lbl[:]
                fl[:] = lbl_out[:]

        # create the segmentation label variable
        self.seg_label = lbl_out[self.img_y, self.img_x]

    def get_reaches(self, shape_file_root, clip=True, clip_buffer=0.1):
        """Get all of the reaches using a ReachExtractor."""
        self.clip = clip
        self.clip_buffer = clip_buffer
        self.reaches = NodeExtractor(               
            shape_file_root, self)
        return self.reaches
    
    def get_centerlines(self, centerline_root, clip=True, clip_buffer=0.1):   
        """Get all of the reaches using a ReachExtractor."""
        self.clip = clip
        self.clip_buffer = clip_buffer
        self.centerlines = ReachExtractor(
            centerline_root, self, clip=clip, clip_buffer=clip_buffer)
        return self.centerlines

    def get_width_db(self, width_db_file):
        """Open the width data base for later use."""
        self.width_db = WidthDataBase(width_db_file)

    def set_width_db(self, width_db):
        """Set width data base from an already opened version."""
        self.width_db = width_db

    def get_max_width_from_db(self, reach_idx):
        """
        Get the width associated with a given reach from the width data base.
        """
        return self.width_db.get_river(
            reach_idx,
            columns=['width'],
            asarray=True,
            transpose=False,
            bounding_box=self.bounding_box,
            clip_buffer=self.clip_buffer)

    def process_reaches(self,
                        scalar_max_width=600.,
                        minobs=10,
                        min_fit_points=3,
                        fit_types=['OLS', 'WLS', 'RLM'],
                        use_width_db=False,
                        ds=None,
                        refine_centerline=False,
                        smooth=1.e-2,
                        alpha=1.,
                        max_iter=1,
                        max_window_size=10000,
                        min_sigma=1000,
                        window_size_sigma_ratio=5,
                        enhanced=False):
        """
        Process all of the reaches in the data bounding box.

        Parameters
        ----------

        scalar_max_width : float, default 600
            How far away to look for points
        minobs : int, default 10
            Minimum number of observations for valid node.
        min_fit_points : int, default 3
            Minimum number of populated nodes required for height/slope fit
        fit_types : list
            A list of fit types to perform. Can contain 'OLS','WLS','RLM'.
        use_width_db : bool, default False
            Use the width data base for setting widths?
        ds : float, optional
            Separation between centerline nodes (in m). If None, uses reach
            points.
        refine_centerline: bool, default True
            Refine the centerline?
        smooth : float, default 1.e-2
            Centerline smoothing constant (see Centerline)
        alpha : float, default 1
            Centerline refinement update weight
        max_iter : int, default 1
            Maximum number of centerline iterations
        max_window_size : max window for gaussian averaging, default is 10km
        min_sigma : min sigma for gaussian averaging, default is 1km
        window_size_sigma_ratio : default is 5

        Returns
        -------

        Returns a list containg a RiverReach instance for each reach in the
        bounding box.
        """
        # assign the reaches
        river_obs_list, reach_idx_list, ireach_list = self.assign_reaches(
            scalar_max_width, minobs, use_width_db, ds)

        river_reach_collection = []
        reach_zips = zip(river_obs_list, reach_idx_list, ireach_list)
        for river_obs, reach_idx, ireach in reach_zips:

            if use_width_db:
                max_width = self.get_max_width_from_db(reach_idx)
                if self.verbose:
                    print('max_width read')

            else:
                max_width = None

            if max_width is None:
                try:
                    # probably should scale this to look some fraction
                    # farther than the database width
                    max_width = (
                        self.reaches[i_reach].metadata['Wmean'] * np.ones(
                            np.shape(self.reaches[i_reach].x)))  # *2.0

                except:
                    max_width = scalar_max_width

            # Ugly way process_reach/process_node uses the data
            self.river_obs = river_obs

            river_reach = self.process_node(
                self.reaches[ireach],
                ireach,
                reach_idx,
                scalar_max_width=scalar_max_width,
                minobs=minobs,
                use_width_db=use_width_db,
                max_width=max_width,
                ds=ds,
                refine_centerline=refine_centerline,
                smooth=smooth,
                alpha=alpha,
                max_iter=max_iter)

            river_reach_collection.append(river_reach)
            if self.store_reaches:
                self.river_reach_collection[ireach] = river_reach

            if self.verbose:
                print('reach pocessed')

        # calculate reach enhanced slope, and add to river_reach_collection
        enhanced_slopes = self.compute_enhanced_slopes(
            river_reach_collection, max_window_size=max_window_size,
            min_sigma=min_sigma,
            window_size_sigma_ratio=window_size_sigma_ratio,
            enhanced=enhanced)

        out_river_reach_collection = []
        # Now iterate over reaches again and do reach average computations
        reach_zips = zip(
            river_reach_collection, river_obs_list, reach_idx_list,
            ireach_list, enhanced_slopes)
        for river_reach, river_obs, reach_idx, ireach, enhanced_slope in\
            reach_zips:

            # Ugly way process_reach/process_node uses the data
            self.river_obs = river_obs

            out_river_reach = self.process_reach(river_reach,
                                                 self.reaches[ireach],
                                                 ireach,
                                                 reach_idx,
                                                 min_fit_points=min_fit_points,
                                                 fit_types=fit_types)

            if out_river_reach is not None:
                # add enhanced slope to river reach outputs
                out_river_reach.metadata['slp_enhncd'] = np.float32(
                    enhanced_slope)
                
                SWOTRiverEstimator.reach_field_list(out_river_reach, enhanced_slope)

                out_river_reach_collection.append(out_river_reach)
                
        self.add_high_resolution_centerline(out_river_reach_collection, self.centerlines)          

        return out_river_reach_collection

    def assign_reaches(self,
                       scalar_max_width,
                       minobs=10,
                       use_width_db=False,
                       ds=None):
        """
        Assigns pixels to nodes for every reach.
        """
        # Iterate over reaches, assign pixels to nodes
        river_obs_list = []
        reach_idx_list = []
        ireach_list = []
        for i_reach, reach_idx in enumerate(self.reaches.reach_idx):

            if len(self.reaches[i_reach].x) <= 3:
                print("reach does not have enough points",
                      len(self.reaches[i_reach].x))
                continue

            if self.verbose:
                print('Reach %d/%d Reach index: %d' %
                      (i_reach + 1, self.reaches.nreaches, reach_idx))

            try:
                river_obs = IteratedRiverObs(
                    self.reaches[i_reach],
                    self.x,
                    self.y,
                    ds=ds,
                    seg_label=self.seg_label,
                    max_width=scalar_max_width,
                    minobs=minobs,
                    verbose=self.verbose)

            except CenterLineException as e:
                print("CenterLineException: ", e)
                continue

            if len(river_obs.x) == 0:
                print('No observations mapped to nodes in this reach')
                continue

            river_obs_list.append(river_obs)
            reach_idx_list.append(reach_idx)
            ireach_list.append(i_reach)

        # Ensure unique and optimal assignments of pixels to reach.
        min_dist = 9999999 * np.ones(self.x.shape)
        reach_ind = -1 * np.ones(self.x.shape, dtype=int)
        cnts_assigned = np.zeros(self.x.shape, dtype=int)
        for ii, river_obs in enumerate(river_obs_list):

            # Get current reach assingment and min distance to node for all
            # pixels assigned to this reach.
            these_reach_inds = reach_ind[river_obs.in_channel]
            these_min_dists = min_dist[river_obs.in_channel]

            # Figure out which ones are better than current assignment
            mask = river_obs.d < these_min_dists

            # Re-assign the pixels to reaches with a better assignment
            these_reach_inds[mask] = ii
            these_min_dists[mask] = river_obs.d[mask]
            reach_ind[river_obs.in_channel] = these_reach_inds
            min_dist[river_obs.in_channel] = these_min_dists
            cnts_assigned[river_obs.in_channel] += 1

        # iterate over river_obs again to set it so optimized
        for ii, river_obs in enumerate(river_obs_list):

            mask_keep = reach_ind[river_obs.in_channel] == ii

            # set in_channel mask to exlude the nodes to drop
            river_obs.in_channel[reach_ind != ii] = False

            # Drop pixels that were double-assigned to reaches and
            # recompute things set in RiverObs constructor
            river_obs.index = river_obs.index[mask_keep]
            river_obs.d = river_obs.d[mask_keep]
            river_obs.x = river_obs.x[mask_keep]
            river_obs.y = river_obs.y[mask_keep]
            river_obs.s = river_obs.s[mask_keep]
            river_obs.n = river_obs.n[mask_keep]
            river_obs.nedited_data = len(river_obs.d)
            river_obs.populated_nodes, river_obs.obs_to_node_map = \
                river_obs.get_obs_to_node_map(river_obs.index, river_obs.minobs)

            # Recompute things set in IteratedRiverObs constructor
            river_obs.add_obs('xo', river_obs.xobs)
            river_obs.add_obs('yo', river_obs.yobs)
            river_obs.load_nodes(['xo', 'yo'])

        # Iterate through and only return reaches with no pixels in them.
        # (don't iterate and modify!)
        river_obs_list_out = []
        reach_idx_list_out = []
        ireach_list_out = []
        reach_zips = zip(river_obs_list, reach_idx_list, ireach_list)
        for river_obs, reach_idx, ireach in reach_zips:
            #if len(river_obs.s) > 0:
            if len(river_obs.populated_nodes) > 3:  # for trim_end purpose   
                river_obs_list_out.append(river_obs)
                reach_idx_list_out.append(reach_idx)
                ireach_list_out.append(ireach)

        return river_obs_list_out, reach_idx_list_out, ireach_list_out

    def process_node(self,
                     reach,
                     reach_id,
                     reach_idx=None,
                     scalar_max_width=600.,
                     minobs=10,
                     use_width_db=False,
                     max_width=None,
                     ds=None,
                     refine_centerline=False,
                     smooth=1.e-2,
                     alpha=1.,
                     max_iter=1):
        """
        Estimate the node quantities for a single reach
        Parameters
        ----------
        reach : Reach instance
            One of the reaches from ReachExtractor.
        reach_id : int
            Index in the list of reaches extracted for this scene.
        reach_idx, int
            Reach index used as pointer to a reach collection that may be
            different than the one used as input (e.g., a global reach
            collection). If using a width database, the retrieved width will
            be associated with this reach index. If None, it will be set equal
            to the reach_id.
        scalar_max_width : float, default 600
            How far away to look for points
        minobs : int, default 10
            Minimum number of observations for valid node.
        use_width_db : bool, default False
            Use the width data base for setting widths?
        max_width: float or array_like, optional
            Maximum width to use for accepting points. From width database or
            apriori.
        ds : float, optional
            Separation between centerline nodes (in m). If None, uses reach
            points.
        refine_centerline: bool, default True
            Refine the centerline?
        smooth : float, default 1.e-2
            Centerline smoothing constant (see Centerline)
        alpha : float, default 1
            Centerline refinement update weight
        max_iter : int, default 1
            Maximum number of centerline iterations
        Returns
        -------
        Return a RiverReach instance with the node quantities populated but
        not the reach quantities.
        """
        # Refine the centerline, if desired
        # get the number of node inthe reach and only refine if there are
        # enough to do spline
        numNodes = len(np.unique(self.river_obs.index))
        enough_nodes = True if numNodes - 1 > self.river_obs.k else False
        if self.verbose:
            print("numNodes,k:", numNodes, self.river_obs.k)

        if refine_centerline and enough_nodes:
            self.river_obs.iterate(
                max_iter=max_iter, alpha=alpha, weights=True, smooth=smooth)

            # Associate the width to the new centerline
            if np.iterable(max_width):
                xw = reach.x
                yw = reach.y
                self.river_obs.add_centerline_obs(xw, yw, max_width,
                                                  'max_width')

            # Reinitialize to the new centerline and max_width
            self.river_obs.reinitialize()
            if self.verbose:
                print('centerline refined')

        else:
            # Associate the width to the new centerline
            if np.iterable(max_width):
                xw = reach.x
                yw = reach.y
                self.river_obs.add_centerline_obs(xw, yw, max_width,
                                                  'max_width')

        # Exclude beginning and end nodes, if desired
        if self.trim_ends:
            first_node = self.river_obs.populated_nodes[0]
            last_node = self.river_obs.populated_nodes[-1]
            self.river_obs.remove_nodes([first_node, last_node])

        # write out the image coordinates for each node in a netcdf file
        try:
            segOut = self.seg_label[self.river_obs.in_channel]

        except:
            segOut = None

        self.write_index_file(self.img_x[self.river_obs.in_channel],
                              self.img_y[self.river_obs.in_channel],
                              self.river_obs.index, self.river_obs.d,
                              self.river_obs.s, self.river_obs.n, reach_idx,
                              segOut, self.h_flg[self.river_obs.in_channel],
                              self.lat_vec[self.river_obs.in_channel],
                              self.lon_vec[self.river_obs.in_channel],
                              self.height_vec[self.river_obs.in_channel])

        # get the prior locations and indices of the nodes
        xw = reach.x
        yw = reach.y
        self.river_obs.add_centerline_obs(xw, yw, xw, 'x_prior')
        self.river_obs.add_centerline_obs(xw, yw, yw, 'y_prior')
        
        lonw = reach.lon
        latw = reach.lat
        self.river_obs.add_centerline_obs(lonw, latw, lonw, 'lon_prior')
        self.river_obs.add_centerline_obs(lonw, latw, latw, 'lat_prior')

        # should probably compute prior lat/lon from prior x,y too
        windex = self.river_obs.centerline_obs['x_prior'].populated_nodes
        x_prior = xw
        x_prior[windex] = self.river_obs.centerline_obs['x_prior'].v
        x_prior = x_prior[self.river_obs.populated_nodes]

        windex = self.river_obs.centerline_obs['y_prior'].populated_nodes
        y_prior = yw
        y_prior[windex] = self.river_obs.centerline_obs['y_prior'].v
        
        lon_prior = lonw                                   
        lon_prior = lon_prior[self.river_obs.populated_nodes]
        lat_prior = latw
        lat_prior = lat_prior[self.river_obs.populated_nodes]
        geoid_height = np.array(reach.geoid_height[self.river_obs.populated_nodes])  
        geoid_height_format = np.reshape(geoid_height,(len(geoid_height),))

        node_index = np.arange(len(y_prior))
        node_index = node_index[self.river_obs.populated_nodes]
        y_prior = y_prior[self.river_obs.populated_nodes]
        reach_index = np.ones(len(node_index)) * (reach_idx)

        # Add the observations
        self.river_obs.add_obs('h_noise', self.h_noise)
        self.river_obs.add_obs('h_flg', (self.h_flg > 0))
        self.river_obs.add_obs('lon', self.lon)
        self.river_obs.add_obs('lat', self.lat)
        self.river_obs.add_obs('xobs', self.x)
        self.river_obs.add_obs('yobs', self.y)
        if self.xtrack is not None:
            self.river_obs.add_obs('xtrack', self.xtrack)
        self.river_obs.add_obs('inundated_area', self.inundated_area)
        if self.wet_tropo_error is not None:   
            self.river_obs.add_obs('wet_tropo_error', self.wet_tropo_error)
            self.river_obs.add_obs('inst_error', self.inst_error)

        dsets_to_load = [
            'h_noise', 'h_flg', 'lon', 'lat', 'xobs', 'yobs', 'inundated_area'
        ]

        if self.xtrack is not None:
            dsets_to_load.append('xtrack')
        if self.wet_tropo_error is not None:  
            dsets_to_load.append('wet_tropo_error')
            dsets_to_load.append('inst_error')     

        self.river_obs.load_nodes(dsets_to_load)

        if self.verbose:
            print('Observations added to nodes')

        # Get various node statistics
        nobs = np.asarray(self.river_obs.get_node_stat('count', ''))
        s_median = np.asarray(self.river_obs.get_node_stat('median', 's'))
        x_median = np.asarray(self.river_obs.get_node_stat('median', 'xobs'))
        y_median = np.asarray(self.river_obs.get_node_stat('median', 'yobs'))

        if self.xtrack is not None:
            xtrack_median = np.asarray(
                self.river_obs.get_node_stat('median', 'xtrack'))
        else:
            xtrack_median = None
        if self.wet_tropo_error is not None:   
            wet_tropo_error_median = np.asarray(
                self.river_obs.get_node_stat('median', 'wet_tropo_error'))
            inst_error_median = np.asarray(
                self.river_obs.get_node_stat('median', 'inst_error'))
        else:
            wet_tropo_error_median = None  
            inst_error_median = None     

        lon_median = np.asarray(self.river_obs.get_node_stat('median', 'lon'))
        lat_median = np.asarray(self.river_obs.get_node_stat('median', 'lat'))
        ds = np.asarray(self.river_obs.get_node_stat('value', 'ds'))

        # number of good heights
        nobs_h = np.asarray(self.river_obs.get_node_stat('countGood', 'h_flg'))
        # heights using same pixels as widths
        h_noise_ave0 = np.asarray(
            self.river_obs.get_node_stat('median', 'h_noise'))
        h_noise_std0 = np.asarray(
            self.river_obs.get_node_stat('std', 'h_noise'))
        # heights using only "good" heights
        h_noise_ave = np.asarray(
            self.river_obs.get_node_stat(
                'median', 'h_noise', good_flag='h_flg'))
        h_noise_std = np.asarray(
            self.river_obs.get_node_stat('std', 'h_noise', good_flag='h_flg'))

        # The following are estimates of river width
        width_ptp = np.asarray(self.river_obs.get_node_stat('width_ptp', ''))
        width_std = np.asarray(self.river_obs.get_node_stat('width_std', ''))

        # These are area based estimates, the nominal SWOT approach
        area = np.asarray(
            self.river_obs.get_node_stat('sum', 'inundated_area'))
        width_area = np.asarray(
            self.river_obs.get_node_stat('width_area', 'inundated_area'))

        # These are the values from the width database
        width_db = np.ones(
            self.river_obs.n_nodes,
            dtype=np.float64) * self.river_obs.missing_value

        try:
            windex = self.river_obs.centerline_obs['max_width'].populated_nodes
            width_db[windex] = self.river_obs.centerline_obs['max_width'].v
            width_db = width_db[self.river_obs.populated_nodes]

        except:
            width_db = np.ones(
                self.river_obs.n_nodes,
                dtype=np.float64) * self.river_obs.missing_value
            width_db = width_db[self.river_obs.populated_nodes]
            
        # h_ort, and h_ell for node shapefiles, and nobs_h > 100 filter
        h_ort_ave = h_noise_ave - np.reshape(geoid_height,(len(geoid_height),))  # h_noise_ave_geoid
        h_ell_ave = h_noise_ave - 0
        h_ort_ave[nobs_h < 100] = 'NaN'   #replace nobs_h < 100 with NaN for node shapefiles  
        h_ell_ave[nobs_h < 100] = 'NaN'
        
        h_ell_4_all_h = h_noise_ave - 0
        h_ell_4_all_h[nobs_h < 100] = 'NaN'
        if xtrack_median is not None:
            h_ell_4_all_h[xtrack_median > 60000] = 'NaN'    
        
        #  add back all node for linear fit     
        populated_node_id = node_index          
        all_node_id = self.river_obs.all_nodes
  
        n_all_node=len(all_node_id)
        all_h= n_all_node * [None]
        all_s= np.asarray(range(n_all_node)) * 200
        
        for i, n_id in enumerate(populated_node_id):
            ii=list(all_node_id).index(n_id)
            all_h[ii] = h_ell_4_all_h[i]   
            
            
            
            
        
        all_h = np.asarray(all_h).astype('float64')
        all_h_fill = all_h + 0
        reach_obsrvd_ratio = np.count_nonzero(~np.isnan(all_h))/len(all_h)
        if  reach_obsrvd_ratio >= self.min_reach_obsrvd_ratio:
            idx = np.isfinite(all_s) & np.isfinite(all_h)
            coef = np.polyfit(all_s[idx], all_h[idx], 1)
            h_fill = coef[0] * all_s + coef[1]
            all_h_fill[np.isnan(all_h)] = h_fill[np.isnan(all_h)]
            
        else:
            all_h_fill = all_h

        # type cast node outputs and pack it up for RiverReach constructor
        river_reach_kw_args = {
            'lat': lat_median.astype('float64'),
            'lon': lon_median.astype('float64'),
            'x': x_median.astype('float64'),
            'y': y_median.astype('float64'),
            'nobs': nobs.astype('int32'),
            's': s_median.astype('float64'),
            'ds': ds.astype('float64'),
            'w_ptp': width_ptp.astype('float32'),
            'w_std': width_std.astype('float32'),
            'w_area': width_area.astype('float32'),
            'w_db': width_db.astype('float32'),
            'area': area.astype('float32'),
            'h_n_ave': h_noise_ave.astype('float32'),
            'h_n_std': h_noise_std.astype('float32'),
            'h_a_ave': h_noise_ave0.astype('float32'),
            'h_a_std': h_noise_std0.astype('float32'),
            'nobs_h': nobs_h.astype('int32'),
            'x_prior': x_prior.astype('float64'),
            'y_prior': y_prior.astype('float64'),
            'lon_prior': lon_prior.astype('float64'),                
            'lat_prior': lat_prior.astype('float64'),
            'geoid_hgt':geoid_height_format.astype('float64'),
            'all_s': all_s.astype('float64'),                  
            'all_h': all_h.astype('float64'),
            'all_h_fill': np.asarray(all_h_fill).astype('float64'),
            'reach_obsrvd_ratio' : reach_obsrvd_ratio,
            'node_indx': node_index.astype('int32'),
            'reach_indx': reach_index.astype('int32'),
        }

        if xtrack_median is not None:
            river_reach_kw_args['xtrack'] = xtrack_median
        else:
            river_reach_kw_args['xtrack'] = None

        SWOTRiverEstimator.node_field_list(river_reach_kw_args,
                                           node_index, reach_index, 
                                           lat_median, lon_median, 
                                           xtrack_median, h_ort_ave,
                                           h_ell_ave,geoid_height_format,
                                           width_area, area, nobs_h, 
                                           wet_tropo_error_median, inst_error_median)    
            
        river_reach = RiverReach(**river_reach_kw_args)
        # Store, if desired
        if self.store_obs:
            self.river_obs_collection[reach_idx] = self.river_obs

        return river_reach

    def process_reach(self,
                      river_reach,
                      reach,
                      reach_id,
                      reach_idx=None,
                      min_fit_points=3,
                      fit_types=['OLS', 'WLS', 'RLM']):
        """
        Estimate the width, height, and slope for one reach.

        Parameters
        ----------
        river_reach : partially populated RiverReach instance with node
            quantities already computed
        reach : Reach instance
            One of the reaches from ReachExtractor.
        reach_id : int
            Index in the list of reaches extracted for this scene.
        reach_idx, int
            Reach index used as pointer to a reach collection that may be
            different than the one used as input (e.g., a global reach
            collection). If using a width database, the retrieved width will
            be associated with this reach index. If None, it will be set equal
            to the reach_id.
        min_fit_points : int, default 3
            Minimum number of populated nodes required for height/slope fit
        fit_types : list
            A list of fit types to perform. Can contain 'OLS','WLS','RLM'.

        Returns
        -------
        Nothing

        Modifies river_reach with the reach quantities (river_reach.metadata)
        """
        # Initialize the fitter
        self.fitter = FitRiver(self.river_obs)

        s_median = river_reach.s
        lon_median = river_reach.lon
        lat_median = river_reach.lat
        xtrack_median = river_reach.xtrack
        width_ptp = river_reach.w_ptp
        width_std = river_reach.w_std
        width_area = river_reach.w_area
        area = river_reach.area
        
        geoid_hgt_node = river_reach.geoid_hgt
        
        all_s = river_reach.all_s                          
        all_h = river_reach.all_h
        reach_obsrvd_ratio = river_reach.reach_obsrvd_ratio
        
        print('reach_obsrvd_ratio',reach_obsrvd_ratio)
        
        

        if reach_obsrvd_ratio >= self.min_reach_obsrvd_ratio:
            all_s_for_fit = np.c_[(all_s-np.nanmax(all_s)/2), np.ones(len(all_s), dtype=all_s.dtype)]
            fit_model = statsmodels.api.OLS(all_h, all_s_for_fit, missing='drop')
            fit_results = fit_model.fit()
            fit_h_reach = fit_results.params[1]
            fit_s_reach = fit_results.params[0]     
            
        else:
            fit_h_reach = 'NaN'
            fit_s_reach = 'NaN'                            

        # Check to see if there are sufficient number of points for fit
        ngood = len(s_median)
        if self.verbose:
            print(('number of fit points: %d' % ngood))

        if ngood < min_fit_points:
            if self.verbose:
                print('not enough good points for fit')

            nresults = None
            return None

        else:
            # Get the start and end
            smin = s_median.min()
            smax = s_median.max()
            # Do the fitting for this reach
            nresults = self.estimate_height_slope(
                smin, smax, fit_types=fit_types, mean_stat='median')
            if self.verbose:
                print('Estimation finished')

        if self.store_fits:
            self.fit_collection[reach_id, 'noise'] = nresults

        # Get the reach statistics for this subreach
        reach_stats = self.get_reach_stats(
            reach_id, reach_idx, s_median, lon_median, lat_median,
            xtrack_median, width_ptp, width_std, width_area, area, nresults, geoid_hgt_node)

        # type-cast reach outputs explicity
        reach_stats['reach_id'] = np.int32(reach_stats['reach_id'])
        reach_stats['reach_idx'] = np.int32(reach_stats['reach_idx'])
        reach_stats['lon_min'] = np.float32(reach_stats['lon_min'])
        reach_stats['lon_max'] = np.float32(reach_stats['lon_max'])
        reach_stats['lat_min'] = np.float32(reach_stats['lat_min'])
        reach_stats['lat_max'] = np.float32(reach_stats['lat_max'])
        reach_stats['area'] = np.float32(reach_stats['area'])
        reach_stats['length'] = np.float32(reach_stats['length'])
        reach_stats['smin'] = np.float32(reach_stats['smin'])
        reach_stats['smax'] = np.float32(reach_stats['smax'])
        reach_stats['save'] = np.float32(reach_stats['save'])
        reach_stats['w_ptp_ave'] = np.float32(reach_stats['w_ptp_ave'])
        reach_stats['w_ptp_min'] = np.float32(reach_stats['w_ptp_min'])
        reach_stats['w_ptp_max'] = np.float32(reach_stats['w_ptp_max'])
        reach_stats['w_std_ave'] = np.float32(reach_stats['w_std_ave'])
        reach_stats['w_std_min'] = np.float32(reach_stats['w_std_min'])
        reach_stats['w_std_max'] = np.float32(reach_stats['w_std_max'])
        reach_stats['w_area_ave'] = np.float32(reach_stats['w_area_ave'])
        reach_stats['w_area_min'] = np.float32(reach_stats['w_area_min'])
        reach_stats['w_area_max'] = np.float32(reach_stats['w_area_max'])
        if xtrack_median is not None:
            reach_stats['xtrck_ave'] = np.float32(reach_stats['xtrck_ave'])
            reach_stats['xtrck_min'] = np.float32(reach_stats['xtrck_min'])
            reach_stats['xtrck_max'] = np.float32(reach_stats['xtrck_max'])
        reach_stats['h_no'] = np.float32(reach_stats['h_no'])
        reach_stats['slp_no'] = np.float32(reach_stats['slp_no'])
        reach_stats['no_rsqrd'] = np.float32(reach_stats['no_rsqrd'])
        reach_stats['no_mse'] = np.float32(reach_stats['no_mse'])
        reach_stats['h_nw'] = np.float32(reach_stats['h_nw'])
        reach_stats['slp_nw'] = np.float32(reach_stats['slp_nw'])
        reach_stats['nw_rsqrd'] = np.float32(reach_stats['nw_rsqrd'])
        reach_stats['nw_mse'] = np.float32(reach_stats['nw_mse'])
        reach_stats['h_nr'] = np.float32(reach_stats['h_nr'])
        reach_stats['slp_nr'] = np.float32(reach_stats['slp_nr'])
        reach_stats['nr_rsqrd'] = np.float32(reach_stats['nr_rsqrd'])
        reach_stats['nr_mse'] = np.float32(reach_stats['nr_mse'])
        reach_stats['h_fit'] = np.float32(fit_h_reach)   
        reach_stats['s_fit'] = np.float32(fit_s_reach)
        reach_stats['geoid_modl'] = np.float32(reach_stats['geoid_modl'])
        reach_stats['height'] = np.float32(reach_stats['h_fit']-reach_stats['geoid_modl'])
        reach_stats['reach_obsrvd_ratio'] = reach_obsrvd_ratio 

        river_reach.metadata = reach_stats
        return river_reach

    def estimate_height_slope(self,
                              smin,
                              smax,
                              fit_types=['OLS', 'WLS', 'RLM'],
                              mean_stat='median'):
        """Get fit results for this subreach."""

        if type(fit_types) == str:
            fit_types = [fit_types]

        nresults = collections.OrderedDict()
        load_inputs = True
        for fit_type in fit_types:
            nresults[fit_type] = self.fitter.fit_linear(
                smin,
                smax,
                'h_noise',
                fit=fit_type,
                mean_stat=mean_stat,
                load_inputs=load_inputs)

            load_inputs = False
        return nresults

    def get_reach_stats(self, reach_id, reach_idx, s_median, lon_median,
                        lat_median, xtrack_median, width_ptp, width_std,
                        width_area, area, nresults):
        """Get statistics for a given reach."""

        ds = np.divide(area, width_area)

        reach_stats = collections.OrderedDict()
        reach_stats['reach_id'] = reach_id
        reach_stats['reach_idx'] = reach_idx
        reach_stats['lon_min'] = np.min(lon_median)
        reach_stats['lon_max'] = np.max(lon_median)
        reach_stats['lat_min'] = np.min(lat_median)
        reach_stats['lat_max'] = np.max(lat_median)
        reach_stats['area'] = np.sum(area)
        reach_stats['length'] = np.sum(ds)
        reach_stats['smin'] = np.min(s_median)
        reach_stats['smax'] = np.max(s_median)
        reach_stats['save'] = np.median(s_median)
        reach_stats['w_ptp_ave'] = np.median(width_ptp)
        reach_stats['w_ptp_min'] = np.min(width_ptp)
        reach_stats['w_ptp_max'] = np.max(width_ptp)
        reach_stats['w_std_ave'] = np.median(width_std)
        reach_stats['w_std_min'] = np.min(width_std)
        reach_stats['w_std_max'] = np.max(width_std)
        reach_stats['w_area_ave'] = np.sum(area) / reach_stats['length']
        reach_stats['w_area_min'] = np.min(width_area)
        reach_stats['w_area_max'] = np.max(width_area)
        reach_stats['geoid_modl'] = np.median(geoid_hgt_node)
        if xtrack_median is not None:
            reach_stats['xtrck_ave'] = np.median(xtrack_median)
            reach_stats['xtrck_min'] = np.min(xtrack_median)
            reach_stats['xtrck_max'] = np.max(xtrack_median)

        # fitting results for h_true
        if nresults is not None:
            for fit_type in ['OLS', 'WLS', 'RLM']:

                tag = {'OLS': 'o', 'WLS': 'w', 'RLM': 'r'}[fit_type]
                try:
                    this_result = nresults[fit_type]

                except KeyError:
                    reach_stats['h_n%s' % tag] = self.river_obs.missing_value
                    reach_stats['slp_n%s' % tag] = self.river_obs.missing_value
                    reach_stats['n%s_rsqrd' %
                                tag] = self.river_obs.missing_value
                    reach_stats['n%s_mse' % tag] = self.river_obs.missing_value
                    continue

                reach_stats['h_n%s' % tag] = this_result.params[1]
                reach_stats['slp_n%s' % tag] = this_result.params[0]

                try:
                    reach_stats['n%s_rsqrd' % tag] = this_result.rsquared
                    reach_stats['n%s_mse' % tag] = this_result.mse_resid

                except AttributeError:
                    reach_stats['n%s_rsqrd' %
                                tag] = self.river_obs.missing_value
                    reach_stats['n%s_mse' % tag] = self.river_obs.missing_value
                    pass

        return reach_stats

    def create_index_file(self):
        """
        Create an empty file with appropriate variable to append to
        """
        # if the filename does not exist create it
        with nc.Dataset(self.output_file, 'w') as ofp:
            ofp.createDimension('record', None)
            ofp.createVariable('range_index', 'i4', 'record', fill_value=-128)
            ofp.createVariable(
                'azimuth_index', 'i4', 'record', fill_value=-128)
            ofp.createVariable('node_index', 'i4', 'record', fill_value=-128)
            ofp.createVariable('reach_index', 'i4', 'record', fill_value=-128)
            ofp.createVariable('river_tag', 'i4', 'record', fill_value=-128)
            ofp.createVariable(
                'segmentation_label', 'i4', 'record', fill_value=-128)
            ofp.createVariable(
                'good_height_flag', 'i1', 'record', fill_value=-1)
            ofp.createVariable(
                'distance_to_node', 'f4', 'record', fill_value=-128)
            ofp.createVariable(
                'along_reach', 'f4', 'record', fill_value=-9990000000)
            ofp.createVariable(
                'cross_reach', 'f4', 'record', fill_value=-9990000000)
            ofp.createVariable(
                'latitude_vectorproc', 'f8', 'record', fill_value=-9990000000)
            ofp.createVariable(
                'longitude_vectorproc', 'f8', 'record', fill_value=-9990000000)
            ofp.createVariable(
                'height_vectorproc', 'f8', 'record', fill_value=-9990000000)

            # copy attributes from pixel cloud product
            for key in self.L2_META_KEY_DEFAULTS:
                setattr(ofp, key, getattr(self, key))

        return

    def write_index_file(self, img_x, img_y, node_index, dst, along_reach,
                         cross_reach, reach_index, seg_lbl, h_flg, lat, lon,
                         height):
        """
        Write out the river obs indices for each pixel that get mapped to a
        node as well as the pixel cloud coordinates (range and azimuth, or
        original image coordinate [e.g., gdem along- and cross-track index])
        """
        # append the new data
        with nc.Dataset(self.output_file, 'a') as ofp:
            curr_len = len(ofp.variables['range_index'])
            new_len = curr_len + len(img_x)
            ofp.variables['range_index'][curr_len:new_len] = img_x
            ofp.variables['azimuth_index'][curr_len:new_len] = img_y
            ofp.variables['node_index'][curr_len:new_len] = node_index
            ofp.variables['reach_index'][curr_len:new_len] = reach_index
            # set river_tag to reach_index + 1 for now (0 assumes not a reach)
            #  (TODO: figure out what this should be)
            ofp.variables['river_tag'][curr_len:new_len] = reach_index + 1
            ofp.variables['segmentation_label'][curr_len:new_len] = seg_lbl
            ofp.variables['good_height_flag'][curr_len:new_len] = h_flg
            ofp.variables['distance_to_node'][curr_len:new_len] = dst
            ofp.variables['along_reach'][curr_len:new_len] = along_reach
            ofp.variables['cross_reach'][curr_len:new_len] = cross_reach
            # for improved geolocation
            ofp.variables['latitude_vectorproc'][curr_len:new_len] = lat
            ofp.variables['longitude_vectorproc'][curr_len:new_len] = lon
            ofp.variables['height_vectorproc'][curr_len:new_len] = height
        return

    def compute_enhanced_slopes(
        self, river_reach_collection, max_window_size, min_sigma,
        window_size_sigma_ratio, enhanced):
        """
        This function calculate enhanced reach slope from smoothed
        node height using Gaussian moving average.
        For more information, pleasec see Dr. Renato Frasson's paper:
        https://agupubs.onlinelibrary.wiley.com/doi/full/10.1002/2017WR020887

        inputs:
        river_reach_collection: collection of river_reach instance
        max_window_size: the max of Gaussian window, default is 10km
        min_sigma : min sigma for gaussian averaging, default is 1km
        window_size_sigma_ratio : default is 5

        output:
        enhanced_slopes: enhanced reach slopes
        """
        # get list of reach index
        n_reach = len(river_reach_collection)
        ind = [reach.reach_indx[0] for reach in river_reach_collection]
        print(ind)
        first_reach = ind[0]
        last_reach = ind[-1]

        enhanced_slopes = []
        for river_reach in river_reach_collection:

            this_reach_len = river_reach.all_s.max() - river_reach.all_s.min()
            this_reach_id = river_reach.reach_indx[0]
            this_reach_obsrvd_ratio= river_reach.reach_obsrvd_ratio

            if enhanced:
                # Build up array of data to be smoothed from downstream to
                # upstream.  Adjust along-reach to be cumulative across
                # reach boundaries.
                if this_reach_obsrvd_ratio >= self.min_reach_obsrvd_ratio and \
                            (this_reach_id-1 in ind or this_reach_id+1 in ind):
                    first_node = 0
                    distances = np.array([])
                    heights = np.array([])

                    if this_reach_id < last_reach and this_reach_id+1 in ind:
                        reach_downstream = river_reach_collection[
                            ind.index(this_reach_id+1)]
                        distances = np.concatenate([
                            reach_downstream.all_s, distances])
                        heights = np.concatenate([
                            reach_downstream.all_h_fill, heights])

                    distances = np.concatenate([
                        river_reach.all_s, distances+river_reach.all_s[-1]])
                    heights = np.concatenate([river_reach.all_h_fill, heights])

                    if this_reach_id > first_reach and this_reach_id-1 in ind:
                        reach_upstream = river_reach_collection[
                            ind.index(this_reach_id-1)]
                        first_node = first_node + len(reach_upstream.all_h_fill)

                        distances = np.concatenate([
                            reach_upstream.all_s, distances+reach_upstream.all_s[-1]])
                        heights = np.concatenate([
                            reach_upstream.all_h_fill, heights])

                    last_node = first_node + len(river_reach.all_h_fill) - 1

                    # window size and sigma for Gaussian averaging
                    window_size = np.min([max_window_size, this_reach_len])
                    sigma = np.max([
                        min_sigma, window_size/window_size_sigma_ratio])

                    # smooth h_n_ave, and get slope
                    idx = np.isfinite(distances) & np.isfinite(heights)
                    slope = np.polyfit(distances[idx], heights[idx], 1)[0]
                    heights_detrend = heights - slope*(
                        distances - distances[0])
                    heights_smooth = self.gaussian_averaging(
                        distances, heights_detrend, window_size, sigma)
                    heights_smooth = heights_smooth + slope*(
                        distances - distances[0])
                    enhanced_slopes.append(
                        -(heights_smooth[last_node-1] - heights_smooth[first_node+1]
                        )/this_reach_len)
                else:
                    enhanced_slopes.append('NaN')

            else:
                enhanced_slopes.append(
                    (river_reach.h_n_ave[0] - river_reach.h_n_ave[-1])
                    /this_reach_len)
        return enhanced_slopes

    @staticmethod
    def gaussian_averaging(distances, heights, window_size, sigma):
        """
        Gaussian smoothing of heights using distances
        distances:   along-flow distance
        heights:     water heights
        window_size: size of data window to use for averaging
        sigma:       STD of Gaussian used for averaging

        outputs:
        smooth_heights : smoothed elevations
        """
        smooth_heights = np.zeros(heights.shape)
        for ii, this_distance in enumerate(distances):

            # get data window
            mask = np.logical_and(
                np.abs(this_distance-distances) <= window_size / 2,
                ~np.isnan(heights))

            weights = scipy.stats.norm.pdf(
                this_distance, distances[mask], sigma)

            smooth_heights[ii] = (
                np.multiply(weights, heights[mask]).sum() /
                weights.sum())

        return smooth_heights
    
    @staticmethod
    def reach_field_list(out_river_reach, enhanced_slope):   
        """
        write out the reach fields to shapefiles from Prod_Des_HR_River_Vecst1c.180409.xlsx
        """
        out_river_reach.metadata['reach_id'] = out_river_reach.metadata['reach_idx']
        out_river_reach.metadata['time_day'] = np.float32(-9999)
        out_river_reach.metadata['time_sec'] = np.float32(-9999)
        out_river_reach.metadata['time_string'] = np.float32(-9999)
        out_river_reach.metadata['p_latitud'] = (out_river_reach.metadata['lat_max'] +                                                                                   out_river_reach.metadata['lat_min'])/2
        out_river_reach.metadata['p_longitud'] = (out_river_reach.metadata['lon_max'] +                                                                                   out_river_reach.metadata['lon_min'])/2
        out_river_reach.metadata['height'] = out_river_reach.metadata['height']
        out_river_reach.metadata['height_u'] = np.float32(-9999)
        out_river_reach.metadata['slope'] = out_river_reach.metadata['s_fit'] * -1000000
        out_river_reach.metadata['slope_u'] = np.float32(-9999)
        out_river_reach.metadata['width'] = out_river_reach.metadata['w_area_ave']
        out_river_reach.metadata['width_u'] = np.float32(-9999)
        out_river_reach.metadata['slope2'] = np.float32(enhanced_slope) * 1000000
        out_river_reach.metadata['slope2_u'] = np.float32(-9999)
        out_river_reach.metadata['d_x_area'] = np.float32(-9999)
        out_river_reach.metadata['d_x_area_u'] = np.float32(-9999)
        out_river_reach.metadata['area_detct'] = np.float32(-9999)
        out_river_reach.metadata['area_det_u'] = np.float32(-9999)
        out_river_reach.metadata['area_total'] = np.float32(-9999)
        out_river_reach.metadata['area_tot_u'] = np.float32(-9999)
        out_river_reach.metadata['area_of_ht'] = np.float32(-9999)
        out_river_reach.metadata['layovr_val'] = np.float32(-9999)
        out_river_reach.metadata['node_dist'] = np.float32(-9999)
        out_river_reach.metadata['xtrk_dist'] = np.float32(-9999)
        out_river_reach.metadata['discharge'] = np.float32(-9999)
        out_river_reach.metadata['dischg_u'] = np.float32(-9999)
        out_river_reach.metadata['discharge1'] = np.float32(-9999)
        out_river_reach.metadata['dischg1_u'] = np.float32(-9999)
        out_river_reach.metadata['discharge2'] = np.float32(-9999)
        out_river_reach.metadata['dischg2_u'] = np.float32(-9999)
        out_river_reach.metadata['discharge3'] = np.float32(-9999)
        out_river_reach.metadata['dischg3_u'] = np.float32(-9999)

        out_river_reach.metadata['f_dark'] = np.float32(-9999)
        out_river_reach.metadata['f_frozen'] = np.float32(-9999)
        out_river_reach.metadata['f_layover'] = np.float32(-9999)
        out_river_reach.metadata['n_good_nod'] = np.float32(-9999)
        out_river_reach.metadata['f_quailty'] = np.float32(-9999)
        out_river_reach.metadata['f_partial'] = np.float32(-9999)
        out_river_reach.metadata['f_xovr_cal'] = np.float32(-9999)
        
        out_river_reach.metadata['geoid_hght'] = out_river_reach.metadata['geoid_modl']
        out_river_reach.metadata['geoid_slop'] = np.float32(-9999)
        out_river_reach.metadata['earth_tide'] = np.float32(-9999)
        out_river_reach.metadata['pole_tide'] = np.float32(-9999)
        out_river_reach.metadata['load_tide'] = np.float32(-9999)
        
        out_river_reach.metadata['c_dry_trop'] = np.float32(-9999)
        out_river_reach.metadata['c_wet_trop'] = np.float32(-9999)
        out_river_reach.metadata['c_iono'] = np.float32(-9999)
        
        out_river_reach.metadata['c_xovr_cal'] = np.float32(-9999)
        out_river_reach.metadata['c_kar_att'] = np.float32(-9999)
        out_river_reach.metadata['c_h_bias'] = np.float32(-9999)
        out_river_reach.metadata['c_sys_cg'] = np.float32(-9999)
        out_river_reach.metadata['c_intr_cal'] = np.float32(-9999)
        
        
        out_river_reach.metadata['n_reach_up'] = np.float32(-9999)
        out_river_reach.metadata['n_reach_dn'] = np.float32(-9999)
        out_river_reach.metadata['rch_id_up'] = np.float32(-9999)
        out_river_reach.metadata['rch_id_dn'] = np.float32(-9999)
        out_river_reach.metadata['p_height'] = np.float32(-9999)
        out_river_reach.metadata['p_hght_var'] = np.float32(-9999)
        out_river_reach.metadata['p_width'] = np.float32(-9999)
        out_river_reach.metadata['p_wid_var'] = np.float32(-9999)
        out_river_reach.metadata['p_class'] = np.float32(-9999)
        out_river_reach.metadata['p_n_nodes'] = np.float32(-9999)
        out_river_reach.metadata['p_dist_out'] = np.float32(-9999)
        out_river_reach.metadata['p_length'] = np.float32(-9999)
        out_river_reach.metadata['mean_flow'] = np.float32(-9999)
        out_river_reach.metadata['grand_id'] = np.float32(-9999)
        out_river_reach.metadata['dischg1_c1'] = np.float32(-9999)
        out_river_reach.metadata['dischg1_c2'] = np.float32(-9999)
        out_river_reach.metadata['dischg2_c1'] = np.float32(-9999)
        out_river_reach.metadata['dischg2_c2'] = np.float32(-9999)
        out_river_reach.metadata['dischg3_c1'] = np.float32(-9999)
        out_river_reach.metadata['dischg3_c2'] = np.float32(-9999)
        out_river_reach.metadata['obs_ratio'] = out_river_reach.metadata['reach_obsrvd_ratio']          
   
    @staticmethod
    def node_field_list(river_reach_kw_args, node_index, 
                        reach_index, lat_median, lon_median, 
                        xtrack_median, h_ort_ave, h_ell_ave, geoid_height, 
                        width_area, area, nobs_h, wet_tropo_error, inst_error):
        river_reach_kw_args['reach_id'] = np.ones(len(node_index)) * (reach_index)
        river_reach_kw_args['node_id'] = node_index
        river_reach_kw_args['time_day'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['time_sec'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['time_string'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['latitude'] = lat_median
        river_reach_kw_args['longitude'] = lon_median
        river_reach_kw_args['latitude_u'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['longitud_u'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['height'] = h_ort_ave
        river_reach_kw_args['height_u'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['width'] = np.asarray(['NaN' if width < 0 else width for width in width_area])  # width_area   
        river_reach_kw_args['width_u'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['area_detct'] =  np.asarray(['NaN' if area_detct < 0 else area_detct for area_detct in area])   
        river_reach_kw_args['area_det_u'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['area_total'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['area_tot_u'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['area_of_ht'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['layovr_val'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['node_dist'] = np.ones(len(node_index)) * -9999
        
        if xtrack_median is not None:
            print('xtrack_median added for L2')
            river_reach_kw_args['xtrk_dist'] = xtrack_median
        else: river_reach_kw_args['xtrk_dist'] = np.ones(len(node_index)) * -9999
            
        if wet_tropo_error is not None:
            print('wet_tropo_median added for L2')
            river_reach_kw_args['wet_error'] = wet_tropo_error
        else: river_reach_kw_args['wet_error'] = np.ones(len(node_index)) * -9999   
            
        if inst_error is not None:
            print('inst_median added for L2')
            river_reach_kw_args['inst_error'] = inst_error
        else: river_reach_kw_args['inst_error'] = np.ones(len(node_index)) * -9999  
            
        river_reach_kw_args['height2'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['height2_u'] = np.ones(len(node_index)) * -9999    

        river_reach_kw_args['f_dark'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['f_frozen'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['f_layover'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['n_good_pix'] = nobs_h # np.ones(len(node_index)) * -9999
        river_reach_kw_args['f_quality'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['f_partial'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['f_xovr_cal'] = np.ones(len(node_index)) * -9999
        
        river_reach_kw_args['rdr_sigma0'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['rdr_sig0_u'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['sigma0_cal'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['c_sig0_atm'] = np.ones(len(node_index)) * -9999
        
        river_reach_kw_args['geoid_hght'] = np.asarray(geoid_height)
        river_reach_kw_args['earth_tide'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['pole_tide'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['load_tide'] = np.ones(len(node_index)) * -9999
        
        river_reach_kw_args['c_dry_trop'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['c_wet_trop'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['c_iono'] = np.ones(len(node_index)) * -9999
        
        river_reach_kw_args['c_xovr_cal'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['c_kar_att'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['c_h_bias'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['c_sys_cg'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['c_intr_cal'] = np.ones(len(node_index)) * -9999
        
        river_reach_kw_args['p_height'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['p_hght_var'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['p_width'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['p_wid_var'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['p_dist_out'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['p_class'] = np.ones(len(node_index)) * -9999
        river_reach_kw_args['grand_id'] = np.ones(len(node_index)) * -9999
    
    def add_high_resolution_centerline(self, out_river_reach_collection, centerlines):
        reach_id_list = [reach.metadata['reach_idx'] for reach in out_river_reach_collection]
        centerline_id_list = [centerline.metadata['Reach_ID'] for centerline in centerlines]
        reach_id_in_centerline_id = []
        for reach_id in reach_id_list:
            reach_id_in_centerline_id.append(centerline_id_list.index(reach_id))
        
        for i in range(len(out_river_reach_collection)):
            setattr(out_river_reach_collection[i], 'lat_cl', centerlines[reach_id_in_centerline_id[i]].lat)
            setattr(out_river_reach_collection[i], 'lon_cl', centerlines[reach_id_in_centerline_id[i]].lon)