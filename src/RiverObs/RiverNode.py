"""
A river node holds all of the measurements associated with a node in a centerline.
It returns various data characteristics when queried.
"""

from __future__ import absolute_import, division, print_function

import numpy as np

class RiverNode:
    """
    A river node holds all of the measurements associated with a node in a
    centerline.  It returns various data characteristics when queried.

    Initialize with the outputs of Centerline plus height and potentially
    other keywords that are turned into class variables.

    Parameters
    ----------

    index : int
        index in the center line corresponding to this node
    d : array_like
        distance from the node to each of the data points
    x : array_like
        x coordinate of each measurement associated with the node
    y : array_like
        y coordinate of each measurement associated with the node
    s : array_like
        along-track coordinate (relative to the node center) for each point
    n : array_like
        across-track (normal) coordinate (relative to the node center) for
        each point
    h_flg: array_like
        flag indicating which pixels to use for height stats
    ds : float
        along-track dimension for this node. Defaults to 1. Needs to be set
        correctly for width_area to work.

    Notes
    -----

    To edit the data, it is useful to sort it by order in one of the variables.
    To sort the data, set sort=True and set sort_variable one of
    ['d','s','n','h'].  If the data are not sorted intially, they can be
    sorted later, as desired by calling RiverNode.sort.
    """

    # These variables are sorted simultaneously when sort is called. Append
    # to this list after class instantiation if you want to add an additional
    # sort variable

    sort_vars = ['d', 'x', 'y', 's', 'n']

    def __init__(self, index, d, x, y, s, n, ds=1, **kwds):
        """
        """

        self.index = index
        self.ds = ds
        self.d = np.asarray(d)
        self.x = np.asarray(x)
        self.y = np.asarray(y)
        self.s = np.asarray(s)
        self.n = np.asarray(n)

        for k in kwds:
            v = kwds[k]
            #exec('self.%s = v'%k)
            setattr(self, k, v)

        self.ndata = len(self.x)
        self.good = np.ones(self.ndata, dtype=np.bool)
        self.sorted = False

    def add_obs(self, obs_name, obs, sort=True):
        """
        Add a new observations, and, if desired, sort them in accordance to
        the current sort index.

        obs_name: name of the observation, will be added as self.obs_name
        obs: iterable with observations of length self.ndata
        """

        if len(obs) != self.ndata:
            raise Exception(
                'length of observations not consistent with number of node points'
            )

        self.sort_vars.append(obs_name)

        if sort and self.sorted:
            setattr(self, obs_name, np.asarray(obs[self.sort_index]))

        else:
            setattr(self, obs_name, np.asarray(obs))

    def count(self, *pars, **kwds):
        """Return the number of points in the node."""
        return self.ndata

    def countGood(self, goodvar):
        """Return the number of good points in the node."""
        tmp = np.zeros(self.ndata)
        #exec('tmp[self.%s]=1'%goodvar)
        tmp[getattr(self, goodvar)] = 1
        return sum(tmp)

    def value(self, var):
        """
        Return self.value. Useful when getting stats for all nodes and a
        scalar variable is desired.
        """
        return getattr(self, var)

    def mean(self, var, goodvar='good'):
        """Return mean of a variable"""
        good = getattr(self, goodvar)
        mean = np.mean(getattr(self, var)[good])
        return mean

    def median(self, var, goodvar='good'):
        """Return mean of a variable"""
        return np.median(getattr(self, var)[getattr(self, goodvar)])

    def std(self, var, goodvar='good'):
        """Return std of a variable"""
        return np.std(getattr(self, var)[getattr(self, goodvar)])

    def stderr(self, var, goodvar='good'):
        """Return standrad error of a variable"""
        good = getattr(self, goodvar)
        scale = 1. / np.sqrt(np.sum(good).astype(np.float))
        stderr = scale * np.std(getattr(self, var)[good])
        return stderr

    def min(self, var, goodvar='good'):
        """Return min of a variable"""
        good = getattr(self, goodvar)
        vmin = np.min(getattr(self, var)[good])
        return vmin

    def max(self, var, goodvar='good'):
        """Return max of a variable"""
        good = getattr(self, goodvar)
        vmax = np.max(getattr(self, var)[good])
        return vmax

    def ptp(self, var, goodvar='good'):
        """Return peak to peak variation of a variable"""
        good = getattr(self, goodvar)
        ptp = np.ptp(getattr(self, var)[good])
        return ptp

    def percentile(self, var, q, goodvar='good'):
        """Return qth percentile of a variable"""
        good = getattr(self, goodvar)
        percentile = np.percentile(getattr(self, var)[good], q)
        return percentile

    def sum(self, var, goodvar='good'):
        """Return the sum of all variable values (e.g., for area)."""
        good = getattr(self, goodvar)
        Sum = np.sum(getattr(self, var)[good])
        return Sum

    def cdf(self, var, goodvar='good'):
        """Get the cdf for a variable."""
        good = getattr(self, goodvar)
        ngood = np.sum(good.astype(np.int32))
        cdf = np.cumsum(np.ones(ngood, dtype=np.float64))
        cdf /= cdf[-1]
        x = np.sort(getattr(self, var)[good])
        return x, cdf

    def flag_extent(self, var_name, var_min, var_max, goodvar='good'):
        """
        Update the good flag to places where the variable var_name
        is within a range of values.
        """
        good_init = getattr(self, goodvar)
        x = getattr(self, var_name)
        good = (x >= var_min) & (x <= var_max)
        self.good = good_init & good

    def add_sort_variables(self, vars):
        """
        Add sort variables to the sort list.
        vars is a string variable name, or an iterable of variable names.
        """
        if type(vars) == str:
            self.sort_vars.append(vars)
        else:
            for var in vars:
                self.sort_vars.append(var)

    def sort(self, sort_variable='n'):
        """
        Sort the data according to the variable sort_variable.
        self.sort_variable must exist.
        All of the variables in self.sort_vars are sorted in the same way.
        """
        # Get the index for sorting
        self.sort_index = np.argsort(getattr(self, sort_variable))

        # Sort all of the desired variables
        for var in self.sort_vars:
            setattr(self, var, getattr(self, var)[self.sort_index])

        # Make sure the good flag is also sorted
        self.good = self.good[self.sort_index]

        # This flag is checked when calling
        self.sorted = True

    def trim(self, fraction, mode='both'):
        """
        Calculate a index flag that trims the data, after sorting.

        fraction: 0 < f < 1. Fraction of the data to remove.
        mode is 'both', 'low', 'high' for which tails of the distribution
        need to be trimmed.
        """
        if not self.sorted:
            raise Exception('Run sort before calling trim')

        if mode == 'both':
            fraction = fraction / 2.

        ntrim = int(self.ndata * fraction + 0.5)

        self.trim_index = np.ones(self.ndata, dtype=np.bool)

        if mode == 'both':
            self.trim_index[:ntrim] = False
            self.trim_index[:-ntrim] = False
        if mode == 'low':
            self.trim_index[:ntrim] = False
        else:
            self.trim_index[:ntrim] = False

        self.good = self.good & self.trim_index

        self.trimmed = True

    def width_ptp(self, *pars, **kwds):
        """
        Return the river width at this node, estimated from the max/min
        extent of the normal coordinate.

        Call signature made compatible with other stat calls, but requires
        no inputs.
        """
        return self.max('n') - self.min('n')

    def width_std(self, *pars, **kwds):
        """
        Return the river width at this node, estimated as sqrt(12)*std(n).
        This would be appropriate for uniformly distributed point measurements.

        Call signature made compatible with other stat calls, but requires
        no inputs.
        """
        nstd = self.std('n')
        width_std = np.sqrt(12.) * nstd
        return width_std

    def width_area(self, area_var='pixel_area'):
        """
        Return the river width at this node, estimated as Area/ds.
        This would be appropriate for pixels representing areas.

        area_var: str
            name of the variable associated with the pixel area
            (default 'pixel_area')

        Call signature made compatible with other stat calls, but requires
        no inputs.
        """
        area = self.sum(area_var)
        width_area = area / self.ds
        return width_area
