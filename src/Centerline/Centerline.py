"""A class for computing the location of a point or set of points
relative to a curved line defined by a series of two dimentional
points."""

import numpy as N
from scipy.spatial import cKDTree
from scipy.interpolate import splrep, splev

class Centerline:
    """A class for computing the location of a point or set of points
    relative to a curved line defined by a series of two dimentional
    points.

    As an option, a set of ovservations (e.g., width) can be associated
    with the center line.

    Parameters
    ----------
        
    x,y : iterables of the same dimension.
    k : int
        order of the spline to use for interpolation (1<= k <=5)
    ds : float
        If not None, resample x,y points to this spacing 
        (approximately to not extend the interval).
    eps : float
        minimum distance for two points to be considered different
    wx, wy : array_like
        coordinate weights. Strictly positive rank-1 array of weights the same
        length as x and y. The weights are used in computing the weighted
        least-squares spline fit. If the errors in the y values have standard
        deviation given by the vector d, then w should be 1/d. Default is ones(len(x)).
    smooth : float
        A smoothing condition, related to the splrep s parameter
        by s = smooth*len(x). The amount of smoothness is determined by
        satisfying the conditions: sum((w * (y - g))**2,axis=0) <= s where g(x)
        is the smoothed interpolation of (x,y). The user can use s to control
        the tradeoff between closeness and smoothness of fit. Larger s means
        more smoothing while smaller values of s indicate less smoothing.
        Recommended values of s depend on the weights, w. If the weights
        represent the inverse of the standard-deviation of y, then a good s
        value should be found in the range (m-sqrt(2*m),m+sqrt(2*m)) where m is
        the number of datapoints in x, y, and w. default : s=m-sqrt(2*m) if
        weights are supplied. smooth = 0.0 (interpolating) if no weights are
        supplied.            

        As an option, observations at the centerline locations can be
        stored and interpolated to the resampled centerline. In this case:

    obs : a list of iterables, each of the same size as x or y
    obs_names: a list of names, of the same size as obs 
        Will be used to set the class members (e.g., ['width'])
    kobs : int
        interpolation method for observations (default 1: linear)
    wobs : array_like
        Observation weights for splrep.
    sobs : float
        Observations smoothing for splrep
    **kwds:
        Additional keywords will be passed to splrep.

    Notes
    ------

    Uses scipy.interpolate.splrep (FITPACK). See scipy documentation for
    further comments on the inputs.
    """

    def __init__(self, x, y, k=3, ds=None, eps=1.e-6,
                 wx=None, wy=None, smooth=None,
                 obs=None, obs_names=None, kobs=1,
                 wobs=None, sobs=None, **kwds):

        # The following holds the array for cKDTree, which
        # must not be touched during calls

        self.x = N.asarray(x)
        self.y = N.asarray(y)

        # If observations are provided, store them as class variables

        if obs != None:
            self.obs_names = obs_names
            for i,name in enumerate(obs_names):
                if len(obs[i]) != len(x):
                    raise Exception('obs size incompatible with x size')
                
                exec('self.%s = N.asarray(obs[i])'%name)

        # Compute the point separation along the curve

        self.delta = N.zeros((len(x),),dtype=N.float64)
        
        self.delta[1:] = N.sqrt((self.x[1:]-self.x[:-1])**2 + 
                                (self.y[1:]-self.y[:-1])**2 )

        # Remove any duplicate points (point separation < eps)

        self.delta[0] = 1 # to avoid removing the first point
        unique = self.delta > eps
        self.delta[0] = 0.
        self.x = self.x[unique]
        self.y = self.y[unique]
        self.delta = self.delta[unique]

        if obs != None:
            for name in self.obs_names:
                exec('self.%s = self.%s[unique]'%(name,name))

        # Compute the distance along the curve
                
        self.s = N.cumsum(self.delta)

        # Compute the spline for evaluating x and y as a function of s

        if smooth != None:
            smooth *= len(self.x)
        self.xtck = splrep(self.s,self.x,k=k,w=wx,s=smooth,**kwds)
        self.ytck = splrep(self.s,self.y,k=k,w=wy,s=smooth,**kwds)
        if obs != None:
            if sobs != None:
                sobs *= len(self.x)
            self.init_obs_tck(self.s,k=kobs,w=wobs,s=sobs,**kwds)

        # If resampling is desired, find the new s, x, and y

        if ds != None:
            ns = int(self.s[-1]/ds + 1)
            if ns < 2:
                raise Exception('This ds is too large for the data set:',ds,ns,self.s[-1])

            self.ds =  self.s[-1]/(ns - 1)
            self.s = N.arange(ns)*self.ds
            self.x = splev(self.s,self.xtck)
            self.y = splev(self.s,self.ytck)
            if obs != None:
                for name in self.obs_names:
                    exec('self.%s = splev(self.s,self.obs_tck["%s"])'%(name,name))

        # Initialize the cKDtree

        self.xy = N.zeros((len(self.x),2),dtype=N.float64)
        self.xy[:,0] = self.x
        self.xy[:,1] = self.y

        self.kdtree = cKDTree(self.xy)
                    
        # Calculate the tangent and normal vectors at each point

        self.dx_ds = splev(self.s,self.xtck,der=1)
        self.dy_ds = splev(self.s,self.ytck,der=1)

        self.tangent = N.zeros((len(self.x),2),dtype=N.float64)
        self.tangent[:,0] = self.dx_ds
        self.tangent[:,1] = self.dy_ds
        self.tangent /= N.sqrt(self.tangent[:,0]**2 + self.tangent[:,1]**2)[:,N.newaxis]

        self.normal = N.zeros((len(self.x),2),dtype=N.float64)
        self.normal[:,0] = -self.tangent[:,1]
        self.normal[:,1] = self.tangent[:,0]

    def __call__(self, x0, y0):
        return self.to_centerline(x0,y0)

    def to_centerline(self, x0, y0):
        """For each point in (x0,y0), return the nearest point, as well
        as the along and across track coordinates for that point in the
        local coordinate system as the point.

        Parameters
        ----------
        
        x0, y0 : array_like
                 1D iterables of the same dimension which can be cast to
                 numpy 1D arrays.

        Returns
        -------

        index : array_like
                The index of the nearest point
        x, y : array_like
               The coordiantes of the nearest point
        s,n : array_like
              The along and across track coordinates of the point
              relative to the nearest point coordinate system.

        Notes
        -----

        __call__ is equivalent to to_centerline.
        """

        xy = N.column_stack([x0,y0])
        d, i = self.kdtree.query(xy)
        x = self.x[i]
        y = self.y[i]
        tx = self.tangent[i][:,0]
        ty = self.tangent[i][:,1]
        s = (x0 - x)*tx + (y0 - y)*ty
        n = -(x0 - x)*ty + (y0 - y)*tx

        return i,d,x,y,s,n

    def init_obs_tck(self,scoord,k=3,s=None,w=None,**kwds):
        """Initialize the spline interpolators for the observations."""

        self.obs_tck = {}
        for name in self.obs_names:
            x = 0 # to fool cython compiler
            exec('x = self.%s'%name)
            self.obs_tck[name] = splrep(scoord,x,k=k,s=s,w=w,**kwds)

